// Rate tracker shared by the heart-rate and breathing estimators: a scalar
// Kalman filter with innovation gating, re-acquisition after persistent
// disagreement, lock hysteresis, and a peak-stability measure.
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace rppg {

enum State : uint8_t { NO_SIGNAL = 0, ACQUIRING = 1, LOCKED = 2 };

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
    bool locked() const { return has_ && good_ >= 4 && bad_ <= 3; }

    // Grow the uncertainty for the time elapsed since the last call.
    void predict(uint32_t now_ms)
    {
        const float dt = (has_ && last_ms_) ? (now_ms - last_ms_) / 1000.0f : 0;
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
        if (accepted && quality >= 0.5f) {
            good_++;
            bad_ = 0;
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
