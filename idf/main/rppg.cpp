#include "rppg.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace rppg {

namespace {

constexpr int POS_L = 32;  // 1.6 s POS sub-window
constexpr int PAD = 40;    // reflection padding for filtfilt
constexpr float SNR_MID = -3.5f;
#ifndef RPPG_Q
#define RPPG_Q 8.0f
#endif
#ifndef RPPG_MOTION_MAX
#define RPPG_MOTION_MAX 0.5f
#endif
constexpr float PROCESS_NOISE = RPPG_Q;      // bpm^2 per second of tracker drift
constexpr float MOTION_MAX = RPPG_MOTION_MAX; // max masked fraction for an update  // SNR (dB) at which the SNR quality term is 0.5

// 4th-order Butterworth bandpass 0.7-3.5 Hz @ 20 Hz (scipy butter(2, ..., output='sos'))
constexpr double SOS[2][6] = {
    {0.1173510367, 0.2347020734, 0.1173510367, 1.0, -0.7516016268, 0.382886852},
    {1.0, -2.0, 1.0, 1.0, -1.7155787262, 0.7695133067},
};

void biquads(double *x, int n)
{
    for (const auto &s : SOS) {
        double z1 = 0, z2 = 0;
        for (int i = 0; i < n; i++) {
            double in = x[i];
            double out = s[0] * in + z1;
            z1 = s[1] * in - s[4] * out + z2;
            z2 = s[2] * in - s[5] * out;
            x[i] = out;
        }
    }
}

// zero-phase bandpass with odd reflection padding
void filtfilt(float *x, int n)
{
    double buf[N + 2 * PAD];
    const int m = n + 2 * PAD;
    for (int i = 0; i < n; i++) buf[PAD + i] = x[i];
    for (int i = 0; i < PAD; i++) {
        buf[PAD - 1 - i] = 2 * x[0] - x[i + 1];
        buf[PAD + n + i] = 2 * x[n - 1] - x[n - 2 - i];
    }
    biquads(buf, m);
    for (int i = 0; i < m / 2; i++) { double t = buf[i]; buf[i] = buf[m - 1 - i]; buf[m - 1 - i] = t; }
    biquads(buf, m);
    for (int i = 0; i < n; i++) x[i] = buf[m - 1 - PAD - i];
}

inline float bin_bpm(float k) { return BPM_MIN + k * BPM_STEP; }

// SNR (dB): power near the fundamental and its 2nd harmonic vs the rest.
float snr_at(const float *P, int k0)
{
    float sig = 0, tot = 0;
    // main-lobe half-width of a Hann window is 2/WINDOW_S Hz (10 bpm at 12 s)
    const float lobe = 120.0f / WINDOW_S;
    const int w1 = int(lobe / BPM_STEP), w2 = int((lobe + 2) / BPM_STEP);
    const int k2 = 2 * k0 + int(BPM_MIN / BPM_STEP);
    for (int k = 0; k < NB_EXT; k++) {
        tot += P[k];
        if (std::abs(k - k0) <= w1 || std::abs(k - k2) <= w2) sig += P[k];
    }
    float noise = tot - sig;
    if (sig <= 0) return -30;
    if (noise <= 0) return 30;
    return 10 * std::log10(sig / noise);
}

int argmax(const float *P, int n)
{
    int b = 0;
    for (int k = 1; k < n; k++)
        if (P[k] > P[b]) b = k;
    return b;
}

float parabolic(const float *P, int k, int n)
{
    if (k <= 0 || k >= n - 1) return k;
    float a = P[k - 1], b = P[k], c = P[k + 1];
    float den = a - 2 * b + c;
    return den != 0 ? k + 0.5f * (a - c) / den : k;
}

} // namespace

// ------------------------------------------------------------------ Buffer
void Buffer::push(const Sample &s)
{
    buf_[head_] = s;
    head_ = (head_ + 1) % kCap;
    if (count_ < kCap) count_++;
}

int Buffer::copy(Sample *out) const
{
    for (int i = 0; i < count_; i++) out[i] = buf_[(head_ - count_ + i + kCap) % kCap];
    return count_;
}

// ------------------------------------------------------------------ Estimator
Estimator::Estimator() : trk_({PROCESS_NOISE, 12.0f, 4.0f, 250.0f, 4.0f})
{
    cos_ = new float[NB_EXT * N];
    sin_ = new float[NB_EXT * N];
    for (int k = 0; k < NB_EXT; k++) {
        double w = 2 * M_PI * bin_bpm(k) / 60.0 / FS;
        for (int i = 0; i < N; i++) {
            double hann = 0.5 - 0.5 * std::cos(2 * M_PI * i / (N - 1));
            cos_[k * N + i] = hann * std::cos(w * i);
            sin_[k * N + i] = hann * std::sin(w * i);
        }
    }
    reset();
}

Estimator::~Estimator()
{
    delete[] cos_;
    delete[] sin_;
}

void Estimator::reset()
{
    trk_.reset();
    std::memset(pulse_, 0, sizeof(pulse_));
}

// Linear resample of one ROI onto a uniform grid ending at the newest sample.
// Motion mask on the resampling grid: motion-flagged spans are removed
// (dilated by 0.3 s, with 0.4 s cosine tapers) instead of discarding the
// whole window. Returns the masked fraction.
float Estimator::motion_mask(const Sample *s, int n)
{
    const float t_end = s[n - 1].t_ms;
    const float t_start = t_end - WINDOW_S * 1000;
    const float step = 1000.0f / FS;
    bool m[N] = {};
    int k = 0;
    for (int i = 0; i < N; i++) {
        const float t = t_start + i * step;
        while (k + 1 < n && s[k + 1].t_ms <= t) k++;
        m[i] = s[k].motion || (k + 1 < n && s[k + 1].motion);
    }
    constexpr int DIL = int(0.3f * FS), TAPER = int(0.4f * FS);
    float masked = 0;
    for (int i = 0; i < N; i++) {
        int d = 1 << 20;  // distance (samples) to the nearest motion sample
        for (int j = std::max(0, i - DIL - TAPER); j <= std::min(N - 1, i + DIL + TAPER); j++)
            if (m[j]) d = std::min(d, std::abs(i - j));
        if (d <= DIL) w_[i] = 0;
        else if (d > DIL + TAPER) w_[i] = 1;
        else w_[i] = 0.5f - 0.5f * std::cos(M_PI * (d - DIL) / TAPER);
        masked += 1 - w_[i];
    }
    return masked / N;
}

bool Estimator::resample(const Sample *s, int n, int roi)
{
    if (n < 2) return false;
    const float t_end = s[n - 1].t_ms;
    const float t_start = t_end - WINDOW_S * 1000;
    const float step = 1000.0f / FS;

    // anchor: last valid sample at or before the window start (else the first
    // valid one, provided the window is at most 1 s short)
    int j = -1;
    for (int k = 0; k < n; k++) {
        if (!s[k].roi[roi].n) continue;
        if (s[k].t_ms <= t_start) {
            j = k;
        } else {
            if (j < 0) j = k;
            break;
        }
    }
    if (j < 0 || s[j].t_ms > t_start + 1000) return false;

    int a = j;
    for (int i = 0; i < N; i++) {
        const float t = t_start + i * step;
        // advance a to the last valid sample with time <= t
        for (int k = a + 1; k < n && s[k].t_ms <= t; k++)
            if (s[k].roi[roi].n) a = k;
        int b = a + 1;
        while (b < n && s[b].roi[roi].n == 0) b++;
        const RoiSample &ra = s[a].roi[roi];
        if (b >= n) {  // past the last valid sample: hold
            if (t - s[a].t_ms > 1000) return false;
            c_[0][i] = ra.r; c_[1][i] = ra.g; c_[2][i] = ra.b;
            continue;
        }
        const RoiSample &rb = s[b].roi[roi];
        const float dt = float(s[b].t_ms) - float(s[a].t_ms);
        if (dt > 1000) return false;  // gap too long
        float u = dt > 0 ? (t - s[a].t_ms) / dt : 0;
        u = u < 0 ? 0 : (u > 1 ? 1 : u);
        c_[0][i] = ra.r + u * (rb.r - ra.r);
        c_[1][i] = ra.g + u * (rb.g - ra.g);
        c_[2][i] = ra.b + u * (rb.b - ra.b);
    }
    return true;
}

void Estimator::pulse_signal(float *H)
{
    for (int i = 0; i < N; i++) H[i] = 0;
    if (mode_ == Mode::RGB) {
        // POS: project temporally-normalised RGB onto the plane orthogonal to
        // skin tone, then alpha-tune the two projections (Wang et al. 2017)
        float s1[POS_L], s2[POS_L];
        for (int n = 0; n + POS_L <= N; n++) {
            float m[3] = {0, 0, 0};
            for (int c = 0; c < 3; c++) {
                for (int k = 0; k < POS_L; k++) m[c] += c_[c][n + k];
                m[c] = m[c] / POS_L + 1e-6f;
            }
            float m1 = 0, m2 = 0;
            for (int k = 0; k < POS_L; k++) {
                float r = c_[0][n + k] / m[0], g = c_[1][n + k] / m[1], b = c_[2][n + k] / m[2];
                s1[k] = g - b;
                s2[k] = g + b - 2 * r;
                m1 += s1[k];
                m2 += s2[k];
            }
            m1 /= POS_L;
            m2 /= POS_L;
            float v1 = 0, v2 = 0;
            for (int k = 0; k < POS_L; k++) {
                v1 += (s1[k] - m1) * (s1[k] - m1);
                v2 += (s2[k] - m2) * (s2[k] - m2);
            }
            const float alpha = v2 > 0 ? std::sqrt(v1 / v2) : 0;
            float h[POS_L], mh = 0;
            for (int k = 0; k < POS_L; k++) { h[k] = s1[k] + alpha * s2[k]; mh += h[k]; }
            mh /= POS_L;
            for (int k = 0; k < POS_L; k++) H[n + k] += h[k] - mh;
        }
    } else {
        // mono / IR: blood absorbs light, so the pulse is the inverted
        // intensity relative to its local (1.6 s) mean
        for (int i = 0; i < N; i++) {
            int a = i - POS_L / 2, b = i + POS_L / 2;
            if (a < 0) a = 0;
            if (b > N) b = N;
            float m = 0;
            for (int k = a; k < b; k++) m += c_[1][k];
            m /= (b - a);
            H[i] = m > 0 ? 1 - c_[1][i] / m : 0;
        }
    }
    float mean = 0, wsum = 0;
    for (int i = 0; i < N; i++) { mean += w_[i] * H[i]; wsum += w_[i]; }
    mean = wsum > 0 ? mean / wsum : 0;
    for (int i = 0; i < N; i++) H[i] = (H[i] - mean) * w_[i];
    filtfilt(H, N);
    for (int i = 0; i < N; i++) H[i] *= w_[i];  // remove filter ringing from masked spans
}

void Estimator::spectrum(const float *x, float *P)
{
    for (int k = 0; k < NB_EXT; k++) {
        const float *c = cos_ + k * N, *s = sin_ + k * N;
        float re = 0, im = 0;
        for (int i = 0; i < N; i++) {
            re += x[i] * c[i];
            im += x[i] * s[i];
        }
        P[k] = re * re + im * im;
    }
}

Result Estimator::update(const Sample *samples, int n, uint32_t now_ms)
{
    Result r{};
    float fused[NB_EXT] = {};
    float wsum = 0, best_w = -1;
    int used = 0;
    float H[kRois][N];
    bool ok[kRois] = {};

    const float masked = n >= 2 ? motion_mask(samples, n) : 1;
    for (int roi = 0; roi < kRois; roi++) {
        r.roi_snr[roi] = -99;
        if (!resample(samples, n, roi)) continue;
        if (mode_ == Mode::MONO)  // collapse to luminance, stored in the G slot
            for (int i = 0; i < N; i++) c_[1][i] = 0.30f * c_[0][i] + 0.59f * c_[1][i] + 0.11f * c_[2][i];
        pulse_signal(H[roi]);
        float *P = P_[roi];
        spectrum(H[roi], P);
        float tot = 0;
        for (int k = 0; k < NB_EXT; k++) tot += P[k];
        if (tot <= 0) continue;
        for (int k = 0; k < NB_EXT; k++) P[k] /= tot;
        const float snr = snr_at(P, argmax(P, NB));
        r.roi_snr[roi] = snr;
        ok[roi] = true;
        used++;
        float w = snr > -8 ? std::pow(10.0f, snr / 10) : 0.02f;
        for (int k = 0; k < NB_EXT; k++) fused[k] += w * P[k];
        wsum += w;
        if (w > best_w) { best_w = w; std::memcpy(pulse_, H[roi], sizeof(pulse_)); }
    }
    r.rois_used = used;

    // Kalman predict
    trk_.predict(now_ms);

    if (!used) {
        r.state = NO_SIGNAL;
        trk_.no_signal();
        r.bpm = trk_.x();
        return r;
    }
    for (int k = 0; k < NB_EXT; k++) fused[k] /= wsum;
    r.motion = masked;

    // peak search: harmonic reinforcement + soft prior around tracked rate
    float F[NB];
    const bool tracking = trk_.tracking();
    for (int k = 0; k < NB; k++) {
        const int k2 = 2 * k + int(BPM_MIN / BPM_STEP);
        // A pulse's 2nd harmonic is weaker than its fundamental. If the power at
        // 2f exceeds that at f, the real rate is probably 2f: penalise f
        // instead of crediting it (avoids octave errors).
        const float h2 = k2 < NB_EXT ? fused[k2] : 0;
        F[k] = h2 <= fused[k] ? fused[k] + 0.5f * h2 : 0.5f * fused[k];
        if (tracking) {
            const float d = (bin_bpm(k) - trk_.x()) / 15.0f;
            F[k] *= 0.3f + 0.7f * std::exp(-0.5f * d * d);
        }
    }
    const int kb = argmax(F, NB);
    const float z = bin_bpm(parabolic(F, kb, NB));
    r.bpm_raw = z;
    r.snr_db = snr_at(fused, kb);
    // quality = SNR score x peak stability: a real pulse stays put from
    // window to window, noise peaks wander
    r.stability = trk_.stability(z);
    const float q_snr = 1.0f / (1.0f + std::exp(-(r.snr_db - SNR_MID) / 1.0f));
    // cross-ROI coherence: a real pulse beats in phase over forehead and
    // cheeks, while sensor noise in each region is independent
    float coh = 0;
    int pairs = 0;
    for (int a = 0; a < kRois; a++)
        for (int b = a + 1; b < kRois; b++) {
            if (!ok[a] || !ok[b]) continue;
            double sab = 0, saa = 0, sbb = 0;
            for (int i = 0; i < N; i++) {
                sab += H[a][i] * H[b][i];
                saa += H[a][i] * H[a][i];
                sbb += H[b][i] * H[b][i];
            }
            coh += (saa > 0 && sbb > 0) ? sab / std::sqrt(saa * sbb) : 0;
            pairs++;
        }
    r.coherence = pairs ? coh / pairs : 0;
    const float q_coh = pairs ? std::fmin(1.0f, std::fmax(0.0f, (r.coherence - 0.05f) / 0.25f)) : 0.5f;
    // hysteresis: coherence is required to acquire a lock, not to keep one
    const float q_keep = q_snr * (0.25f + 0.75f * r.stability);
    r.quality = trk_.good_streak() >= 4 ? std::fmax(q_keep, q_keep * q_coh) : q_keep * q_coh;

    trk_.update(z, r.quality, r.motion < MOTION_MAX);
    r.state = trk_.locked() ? LOCKED : ACQUIRING;
    r.bpm = trk_.x();
    return r;
}

} // namespace rppg
