// Rate tracker shared by the heart-rate and breathing estimators: a scalar
// Kalman filter with innovation gating, re-acquisition after persistent
// disagreement, lock hysteresis, and a peak-stability measure.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rppg {

enum State : uint8_t { NO_SIGNAL = 0, ACQUIRING = 1, LOCKED = 2 };

// Signed millisecond difference a - b. Timestamps are uint32 milliseconds of
// uptime: never convert them to float before subtracting (a float holds only
// ~7 digits, so resolution decays as uptime grows) and this also survives the
// 49.7-day wraparound.
inline float ms_diff(uint32_t a, uint32_t b)
{
    return float(int32_t(a - b));
}

// A window whose newest sample is older than this is stale: the camera stalled
// or frames stopped arriving, and no rate may be reported from it.
constexpr float STALE_MS = 2000.0f;
#ifndef LOCK_Q
#define LOCK_Q 0.5f      // quality that counts a window as good
#endif
#ifndef LOCK_RUN
#define LOCK_RUN 4       // consecutive good windows needed to report a reading
#endif
#ifndef LOCK_STRONG
#define LOCK_STRONG 0.85f  // a window this good counts double: clear evidence
#endif

// Normalised autocorrelation of x at a fractional lag (samples). A periodic
// signal correlates best at its true period, even when its second harmonic is
// stronger in the spectrum, so this resolves octave ambiguity where the
// spectrum alone cannot.
inline float autocorr(const float *x, int n, float lag)
{
    const int L = int(lag);
    if (L < 1 || L >= n - 4) return 0;
    const float u = lag - L;
    double num = 0, e0 = 0, e1 = 0;
    for (int i = 0; i + L + 1 < n; i++) {
        const float y = x[i + L] + u * (x[i + L + 1] - x[i + L]);
        num += x[i] * y;
        e0 += x[i] * x[i];
        e1 += y * y;
    }
    const double den = std::sqrt(e0 * e1);
    return den > 0 ? float(num / den) : 0;
}

// Resolve octave ambiguity for a spectral peak. `p_at(rate)` returns the fused
// spectral power at a rate. A candidate replaces the spectral peak only when it
// both has real spectral support and correlates clearly better in the waveform,
// so noise cannot halve a correct high rate.
#ifndef OCTAVE_SUPPORT
#define OCTAVE_SUPPORT 0.03f   // spectral power a rival octave needs, relative to the peak
#endif
#ifndef OCTAVE_MARGIN
#define OCTAVE_MARGIN 0.20f    // how much better it must repeat in the waveform
#endif
#ifndef OCTAVE_WAVE_EPS
#define OCTAVE_WAVE_EPS 0.10f  // half-period fitting the waveform better by this much is doubt
#endif
#ifndef OCTAVE_AMBIG
#define OCTAVE_AMBIG 0.08f     // rival power (relative to the peak) that makes the octave doubtful
#endif

struct Octave {
    float rate;       // chosen rate
    bool ambiguous;   // a rival octave had support but no clear winner
};

// Resolve octave ambiguity for a spectral peak: report a rate only when the
// waveform agrees. A signal whose 2nd harmonic dominates the spectrum still
// repeats only at its true (longer) period; a genuinely fast rate repeats at
// its own short period. `p_at(rate)` gives the fused spectral power at a rate.
template <typename PowerAt>
inline Octave resolve_octave(const float *wave, int n, float fs, float rate, float lo, float hi, PowerAt p_at)
{
    const float r0 = autocorr(wave, n, 60.0f * fs / rate);
    const float p0 = p_at(rate);
    Octave out{rate, false};
    const float half = rate / 2, dbl = rate * 2;
    if (half >= lo) {
        const float rh = autocorr(wave, n, 60.0f * fs / half);
        if (p_at(half) > OCTAVE_SUPPORT * p0 && rh > r0 + OCTAVE_MARGIN) {
            out.rate = half;   // the waveform clearly prefers the longer period
        } else if (p_at(half) > OCTAVE_AMBIG * p0 ||
                   (rh > r0 + OCTAVE_WAVE_EPS && p_at(half) > 0.02f * p0)) {
            // Either a rival with real spectral support, or a half-period that
            // explains the waveform better than the peak does. Not enough to
            // overturn the peak, but enough that no confident rate may be
            // reported: a dominant 2nd harmonic looks exactly like this.
            out.ambiguous = true;
        }
    }
    if (out.rate == rate && dbl <= hi && p_at(dbl) > p0) {
        const float rd = autocorr(wave, n, 60.0f * fs / dbl);
        if (rd > r0 + OCTAVE_MARGIN) out.rate = dbl;
        else out.ambiguous = true;
    }
    return out;
}

struct TrackerParams {
    float process_noise;  // rate^2 per second of allowed drift
    float gate_min;       // minimum innovation gate (rate units)
    float r_base, r_scale;  // measurement noise = r_base + r_scale * (1 - quality)^2
    float stab_tol;       // peaks within this distance count as agreeing
};

class Tracker {
public:
    explicit Tracker(const TrackerParams &p) : p_(p) { reset(); }

    void reset()
    {
        has_ = false;
        x_ = var_ = 0;
        good_ = bad_ = outlier_ = 0;
        hist_n_ = hist_i_ = 0;
    }

    bool has() const { return has_; }
    float x() const { return has_ ? x_ : 0; }
    int good_streak() const { return good_; }
    bool tracking() const { return has_ && good_ >= 2; }
    // locked: enough good updates, and not currently in a run of rejected ones
    bool locked() const { return has_ && good_ >= LOCK_RUN && bad_ <= 3; }

    // Grow the uncertainty for the time elapsed since the last call.
    void predict(uint32_t now_ms)
    {
        const float dt = (has_ && last_ms_) ? ms_diff(now_ms, last_ms_) / 1000.0f : 0;
        last_ms_ = now_ms;
        if (has_) var_ += p_.process_noise * dt;
    }

    // Records a candidate peak; returns the fraction of recent peaks that agree.
    float stability(float z)
    {
        hist_[hist_i_] = z;
        hist_i_ = (hist_i_ + 1) % kHist;
        if (hist_n_ < kHist) hist_n_++;
        int agree = 0;
        for (int i = 0; i < hist_n_; i++) agree += std::fabs(hist_[i] - z) <= p_.stab_tol;
        return hist_n_ >= 3 ? float(agree) / hist_n_ : 0.5f;
    }

    // A window without any usable signal.
    void no_signal()
    {
        if (++bad_ > 8) reset();
    }

    // Measurement update. `allowed` = false skips it (e.g. too much motion).
    void update(float z, float quality, bool allowed)
    {
        bool accepted = false;
        if (allowed && quality >= 0.2f) {
            const float R = p_.r_base + p_.r_scale * (1 - quality) * (1 - quality);
            if (!has_) {
                x_ = z; var_ = R; has_ = true; accepted = true;
            } else {
                const float innov = z - x_;
                const float gate = std::fmax(p_.gate_min, 3 * std::sqrt(var_ + R));
                if (std::fabs(innov) <= gate) {
                    const float K = var_ / (var_ + R);
                    x_ += K * innov;
                    var_ *= (1 - K);
                    accepted = true;
                    outlier_ = 0;
                } else if (++outlier_ >= 4) {  // persistent disagreement: re-acquire
                    x_ = z; var_ = R; outlier_ = 0; good_ = 0; accepted = true;
                }
            }
        }
        if (accepted && quality >= LOCK_Q) {
            // Strong evidence counts double, so a clean signal is reported after
            // about half as many windows, while a marginal one still has to
            // prove itself over the full run.
            good_ += quality >= LOCK_STRONG ? 2 : 1;
            bad_ = 0;
        } else if (accepted) {
            // Accepted but not good enough to count: a lock may not live on
            // indefinitely on evidence too weak to have earned it.
            if (good_ > 0) good_--;
        } else if (!accepted) {
            bad_++;
            if (good_ > 0 && bad_ > 3) good_--;
        }
        if (bad_ > 10) reset();
    }

private:
    static constexpr int kHist = 6;
    TrackerParams p_;
    bool has_ = false;
    float x_ = 0, var_ = 0;
    uint32_t last_ms_ = 0;
    int good_ = 0, bad_ = 0, outlier_ = 0;
    float hist_[kHist];
    int hist_n_ = 0, hist_i_ = 0;
};

} // namespace rppg
