#include "resp.hpp"

#include "fft.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace resp {

namespace {

constexpr int PAD = 50;
constexpr float SNR_MID = 5.0f;    // SNR (dB) at which the SNR quality term is 0.5
// (fused noise sits at 0-3 dB; real breathing is typically >8 dB)
constexpr float MOTION_MAX = 0.5f; // max masked fraction for an update
constexpr int MAX_FUSED = 8;
constexpr int MIN_FUSED = 3;   // fuse at least this many when available, so agreement means something
constexpr float SELECT_DB = 6.0f;  // fuse channels within this many dB of the best
constexpr float STICKY_DB = 3.0f;       // how much better a rival channel must be to take over
constexpr int STICKY_UPDATES = 3;       // ... and for how many consecutive windows
constexpr float OFF_SUBJECT_DB = 6.0f;  // penalty for a region that is not on the subject
constexpr float CHEST_BONUS_DB = 2.0f;  // preference for the chest box
constexpr float HEAD_BONUS_DB = 1.0f;   // ... and a smaller one for the head box

// 4th-order Butterworth bandpass @ 5 Hz (scipy butter(2, [lo, hi], output='sos'))
constexpr float SOS_ADULT[2][6] = {  // 0.10-0.75 Hz (6-45 /min)
    {0.1040783568, 0.2081567135, 0.1040783568, 1.0, -0.8924468669, 0.3769026242},
    {1.0, -2.0, 1.0, 1.0, -1.8306064659, 0.8483158497},
};
constexpr float SOS_INFANT[2][6] = {  // 0.10-1.30 Hz (6-78 /min)
    {0.274726851, 0.5494537021, 0.274726851, 1.0, 0.0102321417, 0.2054888841},
    {1.0, -2.0, 1.0, 1.0, -1.8234481508, 0.8396135456},
};

void biquads(const float (*sos)[6], float *x, int n)
{
    for (int k = 0; k < 2; k++) {
        const float *s = sos[k];
        float z1 = 0, z2 = 0;
        for (int i = 0; i < n; i++) {
            const float in = x[i];
            const float out = s[0] * in + z1;
            z1 = s[1] * in - s[4] * out + z2;
            z2 = s[2] * in - s[5] * out;
            x[i] = out;
        }
    }
}

void filtfilt(const float (*sos)[6], float *x, int n)
{
    float buf[N + 2 * PAD];
    const int m = n + 2 * PAD;
    for (int i = 0; i < n; i++) buf[PAD + i] = x[i];
    for (int i = 0; i < PAD; i++) {
        buf[PAD - 1 - i] = 2 * x[0] - x[std::min(i + 1, n - 1)];
        buf[PAD + n + i] = 2 * x[n - 1] - x[std::max(n - 2 - i, 0)];
    }
    biquads(sos, buf, m);
    std::reverse(buf, buf + m);
    biquads(sos, buf, m);
    for (int i = 0; i < n; i++) x[i] = buf[m - 1 - PAD - i];
}

inline float bin_br(float k) { return BR_MIN + k * BR_STEP; }

float snr_at(const float *P, int nb_ext, int k0)
{
    // Hann main lobe for a 30 s window: +/-4 breaths/min
    const int w1 = int(4 / BR_STEP), w2 = int(5 / BR_STEP);
    const int k2 = 2 * k0 + K0;
    float sig = 0, tot = 0;
    for (int k = 0; k < nb_ext; k++) {
        tot += P[k];
        if (std::abs(k - k0) <= w1 || std::abs(k - k2) <= w2) sig += P[k];
    }
    const float noise = tot - sig;
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
    const float a = P[k - 1], b = P[k], c = P[k + 1];
    const float den = a - 2 * b + c;
    return den != 0 ? k + 0.5f * (a - c) / den : k;
}

} // namespace

// ------------------------------------------------------------------ Buffer
void Buffer::push(const MotionSample &s)
{
    buf_[head_] = s;
    head_ = (head_ + 1) % kCap;
    if (count_ < kCap) count_++;
}

int Buffer::copy(MotionSample *out) const
{
    for (int i = 0; i < count_; i++) out[i] = buf_[(head_ - count_ + i + kCap) % kCap];
    return count_;
}

// ------------------------------------------------------------------ Estimator
// process noise 0.25 (/min)^2 per second: breathing changes slowly, and a
// steadier reading is more useful than a twitchy one
Estimator::Estimator() : trk_({0.25f, 4.0f, 1.0f, 60.0f, 1.5f})
{
    P_ = new float[kChan * NB_EXT_MAX];
    for (int i = 0; i < N; i++) hann_[i] = 0.5f - 0.5f * std::cos(2 * M_PI * i / (N - 1));
    set_band(Band::ADULT);
}

Estimator::~Estimator()
{
    delete[] P_;
}

void Estimator::set_band(Band b)
{
    if (b == band_ && nb_) return;
    band_ = b;
    const float max = b == Band::ADULT ? BR_MAX_ADULT : BR_MAX_INFANT;
    nb_ = int(max / BR_STEP) - K0 + 1;
    nb_ext_ = std::min(int(2 * max / BR_STEP) - K0 + 1, NB_EXT_MAX);
    reset();
}

void Estimator::reset()
{
    trk_.reset();
    prev_best_ = -1;
    std::memset(wave_, 0, sizeof(wave_));
}

// Motion mask on the resampling grid (1 = clean). ch < 0: whole-scene
// movement only (dilated 1 s, 1 s tapers). ch >= 0: whole-scene movement or
// a jump of that channel (dilated 0.5 s, 0.5 s tapers). Returns the masked fraction.
float Estimator::mask(const MotionSample *s, int n, int ch, float *w)
{
    const uint32_t t_end = s[n - 1].t_ms;
    const float step = 1000.0f / FS;
    const float t_start = -WINDOW_S * 1000;  // offsets relative to the newest sample
    bool g[N] = {}, m[N] = {};
    const uint64_t bit = ch >= 0 ? 1ull << ch : 0;
    int k = 0;
    for (int i = 0; i < N; i++) {
        const float lo = t_start + (i - 0.5f) * step, hi = lo + step;
        while (k < n && rppg::ms_diff(s[k].t_ms, t_end) < lo) k++;
        // Frames arrive faster than this grid (about 11 fps against 5 Hz), so
        // every sample in the interval must be examined: a flag on a sample
        // between two grid points would otherwise be lost entirely.
        for (int j = k; j < n && rppg::ms_diff(s[j].t_ms, t_end) < hi; j++) {
            g[i] = g[i] || s[j].gross;
            m[i] = m[i] || (s[j].jump & bit);
        }
        if (k > 0) {
            g[i] = g[i] || s[k - 1].gross;
            m[i] = m[i] || (s[k - 1].jump & bit);
        }
    }
    constexpr int GD = int(1.0f * FS), GT = int(1.0f * FS), JD = int(0.5f * FS), JT = int(0.5f * FS);
    float masked = 0;
    for (int i = 0; i < N; i++) {
        int dg = 1 << 20, dj = 1 << 20;
        for (int j = std::max(0, i - GD - GT); j <= std::min(N - 1, i + GD + GT); j++) {
            if (g[j]) dg = std::min(dg, std::abs(i - j));
            if (m[j]) dj = std::min(dj, std::abs(i - j));
        }
        auto taper = [](int d, int dil, int tap) {
            if (d <= dil) return 0.0f;
            if (d > dil + tap) return 1.0f;
            return 0.5f - 0.5f * std::cos(float(M_PI) * (d - dil) / tap);
        };
        w[i] = taper(dg, GD, GT) * (ch >= 0 ? taper(dj, JD, JT) : 1.0f);
        masked += 1 - w[i];
    }
    return masked / N;
}

bool Estimator::resample(const MotionSample *s, int n, int ch, float *x)
{
    const uint32_t t_end = s[n - 1].t_ms;
    const float t_start = -WINDOW_S * 1000;  // offsets relative to the newest sample
    const float step = 1000.0f / FS;
    const uint64_t bit = 1ull << ch;

    int j = -1;
    for (int k = 0; k < n; k++) {
        if (!(s[k].valid & bit)) continue;
        if (rppg::ms_diff(s[k].t_ms, t_end) <= t_start) {
            j = k;
        } else {
            if (j < 0) j = k;
            break;
        }
    }
    if (j < 0 || rppg::ms_diff(s[j].t_ms, t_end) > t_start + 3000) return false;

    // The displacement is an absolute, drifting quantity: work relative to the
    // first sample of the window so the values stay small (float precision).
    const float base = s[j].d[ch];
    int a = j, valid = 0;
    for (int i = 0; i < N; i++) {
        const float t = t_start + i * step;
        for (int k = a + 1; k < n && rppg::ms_diff(s[k].t_ms, t_end) <= t; k++)
            if (s[k].valid & bit) a = k;
        int b = a + 1;
        while (b < n && !(s[b].valid & bit)) b++;
        const float ta = rppg::ms_diff(s[a].t_ms, t_end);
        if (b >= n) {
            if (t - ta > 2000) return false;
            x[i] = s[a].d[ch] - base;
            continue;
        }
        const float tb = rppg::ms_diff(s[b].t_ms, t_end);
        const float dt = tb - ta;
        if (dt > 2000) return false;
        float u = dt > 0 ? (t - ta) / dt : 0;
        u = std::clamp(u, 0.0f, 1.0f);
        x[i] = (s[a].d[ch] + u * (s[b].d[ch] - s[a].d[ch])) - base;
        valid++;
    }
    if (valid <= N / 2) return false;

    // Anti-alias before decimation. Frames arrive at ~11 fps and the grid is
    // 5 Hz, so anything above 2.5 Hz folds into the breathing band and cannot be
    // told from breathing afterwards. Low-pass the raw samples (zero-phase,
    // one pole each way, 1.5 Hz - above the infant band) at their own irregular
    // rate, then interpolate that.
    static float rt[kCap], rv[kCap];
    int m = 0;
    for (int k = 0; k < n; k++) {
        if (!(s[k].valid & bit)) continue;
        rt[m] = rppg::ms_diff(s[k].t_ms, t_end);
        rv[m] = s[k].d[ch] - base;
        m++;
    }
    if (m < 4) return false;

    // A channel that barely moved cannot carry breathing. Testing that here,
    // on the raw samples, skips the filtering and spectrum for it entirely -
    // those dominate the cost of an update.
    float mn = rv[0], mx = rv[0];
    for (int k = 1; k < m; k++) { mn = std::fmin(mn, rv[k]); mx = std::fmax(mx, rv[k]); }
    if (mx - mn < 0.02f) { still_ = true; return false; }

    // A single pole only reaches about -15 dB near the frame rate, so cascade
    // three each way (six poles, zero phase). At 2 Hz this leaves the breathing
    // band (up to 1.3 /s for infants) usable while burying anything faster.
    constexpr float FC = 2.0f;
    constexpr int POLES = 3;
    // The frame interval barely varies, so cache the filter coefficient rather
    // than calling exp() for every sample of every channel (which cost ~300 ms
    // per update on the S3).
    // One coefficient for the whole window, from its mean sample interval: the
    // frame interval varies by a millisecond or two, far too little to matter
    // for an anti-alias filter, and calling exp() per sample per pole per
    // channel cost hundreds of milliseconds per update on the S3.
    const float mean_dt = std::max(1e-3f, (rt[m - 1] - rt[0]) / 1000.0f / (m - 1));
    const float alpha = 1 - std::exp(-2 * float(M_PI) * FC * mean_dt);
    for (int pass = 0; pass < POLES; pass++) {
        float y = rv[0];
        for (int k = 1; k < m; k++) { y += alpha * (rv[k] - y); rv[k] = y; }
        y = rv[m - 1];
        for (int k = m - 2; k >= 0; k--) { y += alpha * (rv[k] - y); rv[k] = y; }
    }
    for (int i = 0, k = 0; i < N; i++) {
        const float t = t_start + i * step;
        while (k + 1 < m - 1 && rt[k + 1] <= t) k++;
        const float dt = rt[k + 1] - rt[k];
        const float u = dt > 0 ? std::clamp((t - rt[k]) / dt, 0.0f, 1.0f) : 0;
        x[i] = rv[k] + u * (rv[k + 1] - rv[k]);
    }
    return true;
}

void Estimator::spectrum(const float *x, float *P)
{
    float re[NFFT], im[NFFT];
    for (int i = 0; i < N; i++) re[i] = x[i] * hann_[i];
    for (int i = N; i < NFFT; i++) re[i] = 0;
    for (int i = 0; i < NFFT; i++) im[i] = 0;
    rppg::fft(re, im, NFFT);
    for (int k = 0; k < nb_ext_; k++) P[k] = re[K0 + k] * re[K0 + k] + im[K0 + k] * im[K0 + k];
}

Result Estimator::update(const MotionSample *s, int n, uint32_t now_ms)
{
    Result r{};
    r.best = -1;
    for (int8_t &c : r.sel) c = -1;
    trk_.predict(now_ms);
    if (n < 2 || rppg::ms_diff(s[n - 1].t_ms, s[0].t_ms) < (WINDOW_S - 3) * 1000 ||
        rppg::ms_diff(now_ms, s[n - 1].t_ms) > rppg::STALE_MS) {
        r.state = rppg::NO_SIGNAL;
        trk_.no_signal();
        return r;  // no usable window: report nothing, not the last tracked value
    }
    const float (*sos)[6] = band_ == Band::ADULT ? SOS_ADULT : SOS_INFANT;
    r.motion = mask(s, n, -1, wg_);

    float x[N], snr[kChan];
    bool ok[kChan] = {};
    int peak[kChan];
    for (int ch = 0; ch < kChan; ch++) {
        snr[ch] = -99;
        still_ = false;
        if (!resample(s, n, ch, x)) { (still_ ? r.n_still : r.n_nodata)++; continue; }
        float w_[N];
        if (mask(s, n, ch, w_) > 0.6f) { r.n_masked++; continue; }  // moving most of the time
        // Weighted linear detrend of each unmasked segment separately: a gross
        // movement usually leaves the subject at a different position, and a
        // level step across the masked span would leak into the low bins.
        int segs = 0;
        for (int a = 0; a < N;) {
            if (w_[a] <= 0) { x[a++] = 0; continue; }
            int b = a;
            while (b < N && w_[b] > 0) b++;
            // (float on purpose: the S3 FPU is single precision; t is centred to keep sums small)
            const float tc = 0.5f * (a + b - 1);
            float sw = 0, st = 0, sx = 0, stt = 0, stx = 0;
            for (int i = a; i < b; i++) {
                const float w = w_[i], u = i - tc;
                sw += w; st += w * u; sx += w * x[i]; stt += w * u * u; stx += w * u * x[i];
            }
            const float den = sw * stt - st * st;
            const float slope = (b - a > 2 && den != 0) ? (sw * stx - st * sx) / den : 0;
            const float icpt = sw > 0 ? (sx - slope * st) / sw : 0;
            for (int i = a; i < b; i++) x[i] = (x[i] - (icpt + slope * (i - tc))) * w_[i];
            segs++;
            a = b;
        }
        if (!segs) continue;
        filtfilt(sos, x, N);
        float energy = 0;
        for (int i = 0; i < N; i++) { x[i] *= w_[i]; energy += x[i] * x[i]; }
        // Below this the "signal" is numerical residue, not motion: breathing
        // moves a tile by ~0.1 px or more.
        if (std::sqrt(energy / N) < 0.01f) { r.n_weak++; continue; }
        float *P = P_ + ch * NB_EXT_MAX;
        spectrum(x, P);
        float tot = 0;
        for (int k = 0; k < nb_ext_; k++) tot += P[k];
        if (tot <= 0) continue;
        for (int k = 0; k < nb_ext_; k++) P[k] /= tot;
        peak[ch] = argmax(P, nb_);
        snr[ch] = snr_at(P, nb_ext_, peak[ch]);
        ok[ch] = true;
        std::memcpy(waves_[ch], x, sizeof(x));
    }

    // select the strongest channels; the chest box (when a face gave us one) is
    // where breathing actually is, so it wins ties against background tiles
    // when a face told us where the subject is, background regions are demoted
    const uint64_t subject = s[n - 1].subject;
    auto rank = [&](int ch) {
        float r = snr[ch];
        if (ch >= kBoxChan0 + 2) r += HEAD_BONUS_DB;      // head box
        else if (ch >= kBoxChan0) r += CHEST_BONUS_DB;    // chest box
        if (subject && !(subject & (1ull << ch))) r -= OFF_SUBJECT_DB;
        return r;
    };
    int order[kChan], m = 0;
    for (int ch = 0; ch < kChan; ch++)
        if (ok[ch]) order[m++] = ch;
    std::sort(order, order + m, [&](int a, int b) { return rank(a) > rank(b); });
    // Fuse the channels close to the best, but always at least a few when they
    // exist: agreement between regions is what makes a rate believable, and a
    // single dominant channel (which is what a good zoom produces) would
    // otherwise be judged as having no corroboration at all.
    int sel = 0;
    while (sel < m && sel < MAX_FUSED && rank(order[sel]) >= rank(order[0]) - SELECT_DB) sel++;
    sel = std::min(std::max(sel, MIN_FUSED), m);
    if (!sel) {
        r.state = rppg::NO_SIGNAL;
        trk_.no_signal();
        return r;  // no usable window: report nothing, not the last tracked value
    }
    r.channels = sel;
    for (int i = 0; i < sel && i < 8; i++) r.sel[i] = int8_t(order[i]);
    // Keep reporting the same channel unless another is clearly better for a
    // while: the displayed region hopping between equally good tiles every
    // second is just noise to the eye.
    int best = order[0];
    if (prev_best_ >= 0 && ok[prev_best_] && rank(prev_best_) >= rank(best) - STICKY_DB) {
        best = prev_best_;
        sticky_ = 0;
    } else if (prev_best_ >= 0 && ok[prev_best_] && ++sticky_ < STICKY_UPDATES) {
        best = prev_best_;  // one better window is not enough to switch
    } else {
        sticky_ = 0;
    }
    prev_best_ = best;
    r.best = best;
    std::memcpy(wave_, waves_[best], sizeof(wave_));

    float fused[NB_EXT_MAX] = {};
    float wsum = 0;
    for (int i = 0; i < sel; i++) {
        const int ch = order[i];
        const float w = std::pow(10.0f, snr[ch] / 10);
        const float *P = P_ + ch * NB_EXT_MAX;
        for (int k = 0; k < nb_ext_; k++) fused[k] += w * P[k];
        wsum += w;
    }
    for (int k = 0; k < nb_ext_; k++) fused[k] /= wsum;

    float F[NB_EXT_MAX], Fp[NB_EXT_MAX];
    const bool tracking = trk_.tracking();
    for (int k = 0; k < nb_; k++) {
        const int k2 = 2 * k + K0;
        const float h2 = k2 < nb_ext_ ? fused[k2] : 0;
        F[k] = fused[k] + 0.5f * std::fmin(h2, fused[k]);  // capped harmonic credit
        Fp[k] = F[k];
        if (tracking) {
            const float d = (bin_br(k) - trk_.x()) / 6.0f;
            Fp[k] *= 0.3f + 0.7f * std::exp(-0.5f * d * d);
        }
    }
    // stability is judged on the unbiased peak; the tracking prior only picks z
    const int k_free = argmax(F, nb_);
    r.stability = trk_.stability(bin_br(parabolic(F, k_free, nb_)));
    const int kb = argmax(Fp, nb_);
    float z = bin_br(parabolic(Fp, kb, nb_));
    const float br_max = band_ == Band::ADULT ? BR_MAX_ADULT : BR_MAX_INFANT;
    const rppg::Octave oct = rppg::resolve_octave(wave_, N, FS, z, BR_MIN, br_max, [&](float br) {
        const int k = std::clamp(int((br / BR_STEP) - K0 + 0.5f), 0, nb_ext_ - 1);
        return fused[k];
    });
    z = oct.rate;
    r.raw = z;
    const int kz = std::clamp(int((z / BR_STEP) - K0 + 0.5f), 0, nb_ - 1);
    r.snr_db = snr_at(fused, nb_ext_, kz);
    int agree = 0;
    for (int i = 0; i < sel; i++) agree += std::fabs(bin_br(peak[order[i]]) - z) <= 2.0f;
    r.agreement = float(agree) / sel;
    const float q_snr = 1.0f / (1.0f + std::exp(-(r.snr_db - SNR_MID) / 1.2f));
    // a lone channel gets no agreement credit
    const float q_agree = sel >= 2 ? r.agreement : 0.5f;
    r.quality = q_snr * (0.25f + 0.75f * r.stability) * (0.2f + 0.8f * q_agree);
    // An unresolved octave must not be reported confidently: cap the quality
    // below the lock threshold rather than merely halving it.
    if (oct.ambiguous) r.quality = std::fmin(r.quality * 0.5f, 0.45f);

    trk_.update(z, r.quality, r.motion < MOTION_MAX);
    r.state = trk_.locked() ? rppg::LOCKED : rppg::ACQUIRING;
    r.brpm = trk_.x();
    return r;
}

} // namespace resp
