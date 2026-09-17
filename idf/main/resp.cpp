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
constexpr float SELECT_DB = 6.0f;  // fuse channels within this many dB of the best

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
Estimator::Estimator() : trk_({0.5f, 4.0f, 1.0f, 60.0f, 1.5f})
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
    const float t_end = s[n - 1].t_ms;
    const float t_start = t_end - WINDOW_S * 1000;
    const float step = 1000.0f / FS;
    bool g[N] = {}, m[N] = {};
    const uint32_t bit = ch >= 0 ? 1u << ch : 0;
    int k = 0;
    for (int i = 0; i < N; i++) {
        const float t = t_start + i * step;
        while (k + 1 < n && s[k + 1].t_ms <= t) k++;
        // flags from the samples on both sides of this grid point
        const MotionSample &a = s[k];
        const MotionSample *b = k + 1 < n ? &s[k + 1] : nullptr;
        g[i] = a.gross || (b && b->gross);
        m[i] = (a.jump & bit) || (b && (b->jump & bit));
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
    const float t_end = s[n - 1].t_ms;
    const float t_start = t_end - WINDOW_S * 1000;
    const float step = 1000.0f / FS;
    const uint32_t bit = 1u << ch;

    int j = -1;
    for (int k = 0; k < n; k++) {
        if (!(s[k].valid & bit)) continue;
        if (s[k].t_ms <= t_start) {
            j = k;
        } else {
            if (j < 0) j = k;
            break;
        }
    }
    if (j < 0 || s[j].t_ms > t_start + 3000) return false;

    int a = j, valid = 0;
    for (int i = 0; i < N; i++) {
        const float t = t_start + i * step;
        for (int k = a + 1; k < n && s[k].t_ms <= t; k++)
            if (s[k].valid & bit) a = k;
        int b = a + 1;
        while (b < n && !(s[b].valid & bit)) b++;
        if (b >= n) {
            if (t - s[a].t_ms > 2000) return false;
            x[i] = s[a].d[ch];
            continue;
        }
        const float dt = float(s[b].t_ms) - float(s[a].t_ms);
        if (dt > 2000) return false;
        float u = dt > 0 ? (t - s[a].t_ms) / dt : 0;
        u = std::clamp(u, 0.0f, 1.0f);
        x[i] = s[a].d[ch] + u * (s[b].d[ch] - s[a].d[ch]);
        valid++;
    }
    return valid > N / 2;
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
    trk_.predict(now_ms);
    if (n < 2 || s[n - 1].t_ms - s[0].t_ms < (WINDOW_S - 3) * 1000) {
        r.state = rppg::NO_SIGNAL;
        trk_.no_signal();
        r.brpm = trk_.x();
        return r;
    }
    const float (*sos)[6] = band_ == Band::ADULT ? SOS_ADULT : SOS_INFANT;
    r.motion = mask(s, n, -1, wg_);

    float x[N], snr[kChan];
    bool ok[kChan] = {};
    int peak[kChan];
    for (int ch = 0; ch < kChan; ch++) {
        snr[ch] = -99;
        if (!resample(s, n, ch, x)) continue;
        float w_[N];
        if (mask(s, n, ch, w_) > 0.6f) continue;  // this tile was moving most of the time
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
        if (energy <= 1e-9) continue;
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

    // select the strongest channels
    int order[kChan], m = 0;
    for (int ch = 0; ch < kChan; ch++)
        if (ok[ch]) order[m++] = ch;
    std::sort(order, order + m, [&](int a, int b) { return snr[a] > snr[b]; });
    int sel = 0;
    while (sel < m && sel < MAX_FUSED && snr[order[sel]] >= snr[order[0]] - SELECT_DB) sel++;
    if (!sel) {
        r.state = rppg::NO_SIGNAL;
        trk_.no_signal();
        r.brpm = trk_.x();
        return r;
    }
    r.channels = sel;
    // keep reporting the previous best channel unless another is clearly better
    int best = order[0];
    if (prev_best_ >= 0 && ok[prev_best_] && snr[prev_best_] >= snr[best] - 1.5f) best = prev_best_;
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
        F[k] = h2 <= fused[k] ? fused[k] + 0.5f * h2 : 0.5f * fused[k];  // octave check
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
    const float z = bin_br(parabolic(Fp, kb, nb_));
    r.raw = z;
    r.snr_db = snr_at(fused, nb_ext_, kb);
    int agree = 0;
    for (int i = 0; i < sel; i++) agree += std::fabs(bin_br(peak[order[i]]) - z) <= 2.0f;
    r.agreement = float(agree) / sel;
    const float q_snr = 1.0f / (1.0f + std::exp(-(r.snr_db - SNR_MID) / 1.2f));
    // a lone channel gets no agreement credit
    const float q_agree = sel >= 2 ? r.agreement : 0.5f;
    r.quality = q_snr * (0.25f + 0.75f * r.stability) * (0.2f + 0.8f * q_agree);

    trk_.update(z, r.quality, r.motion < MOTION_MAX);
    r.state = trk_.locked() ? rppg::LOCKED : rppg::ACQUIRING;
    r.brpm = trk_.x();
    return r;
}

} // namespace resp
