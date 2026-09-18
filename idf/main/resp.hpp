// Breathing-rate estimator over tile displacements (motion.hpp).
//
// Once per second over the last WINDOW_S seconds:
//   per channel: resample to FS -> detrend -> gross-motion mask -> bandpass
//                -> spectrum -> SNR
//   selection:   the best channels (breathing is local: chest, shoulders)
//   fusion:      SNR-weighted spectrum, octave check, agreement between channels
//   output:      shared Tracker (Kalman + gating + lock hysteresis)
// Plain C++ (host-testable).
#pragma once
#include <cstdint>

#include "motion.hpp"
#include "tracker.hpp"

namespace resp {

// The window is 30 s and frames can arrive at 25 fps (the zoomed sensor window
// reads out faster), so the ring must hold well over 750 samples or the
// estimator never sees a full window.
constexpr int kCap = 1024;
constexpr float FS = 5.0f;
constexpr float WINDOW_S = 30.0f;
constexpr int N = 150;  // WINDOW_S * FS
// Spectrum: Hann-windowed, zero-padded FFT. Bin k (relative) is (K0 + k) * BR_STEP /min.
constexpr int NFFT = 512;   // 0.59 /min per bin, interpolated: plenty for breathing
constexpr float BR_STEP = 60.0f * FS / NFFT;  // 0.293 breaths/min per bin
constexpr int K0 = 11;                        // first bin: 6.4 /min
constexpr float BR_MIN = K0 * BR_STEP;
constexpr float BR_MAX_ADULT = 45.0f, BR_MAX_INFANT = 78.0f;
constexpr int NB_EXT_MAX = NFFT / 2 - K0;     // up to Nyquist (150 /min)

enum class Band : uint8_t { ADULT, INFANT };

struct Result {
    rppg::State state;
    // Presence is deliberately separate from the rate: a 30 s window cannot say
    // whether breathing is happening *now*, and for a monitor that is the
    // question that matters. Measured over the last few seconds only.
    bool present;      // breathing motion seen in the recent window
    float presence;    // how strong it was, relative to the detection floor
    float swing;       // the strongest region's recent movement, in pixels
    float quiet_s;     // seconds since breathing motion was last seen
    float brpm;       // tracked breaths per minute (0 if none)
    float raw;        // this window's peak
    float snr_db;
    float quality;
    float stability;
    float agreement;  // fraction of independent regions whose own peak agrees
    float motion;     // masked fraction of the window
    int channels;     // channels fused
    int regions;      // how many of those are genuinely independent (see agreement)
    uint8_t n_nodata, n_still, n_masked, n_weak;  // why the others were dropped
    int best;         // best channel (2*tile + axis), -1 if none
    int8_t sel[8];    // the fused channels themselves (-1 where unused)
};

class Buffer {
public:
    void push(const MotionSample &s);
    void clear() { count_ = 0; }
    int copy(MotionSample *out) const;

private:
    MotionSample buf_[kCap];
    int head_ = 0, count_ = 0;
};

class Estimator {
public:
    Estimator();
    ~Estimator();
    void reset();
    void set_band(Band b);
    Band band() const { return band_; }
    Result update(const MotionSample *s, int n, uint32_t now_ms);
    // Band-passed displacement of the best channel from the last update (N samples).
    const float *wave() const { return wave_; }

private:
    bool resample(const MotionSample *s, int n, int ch, float *x);
    float mask(const MotionSample *s, int n, int ch, float *w);
    void spectrum(const float *x, float *P);
    void check_presence(const MotionSample *s, int n, Result &r);

    Band band_ = Band::ADULT;
    int nb_ = 0, nb_ext_ = 0;
    float hann_[N];
    float *P_ = nullptr;  // kChan * NB_EXT_MAX
    float wg_[N];  // global (whole-scene) motion mask
    float wave_[N];
    float waves_[kChan][N];
    rppg::Tracker trk_;
    int prev_best_ = -1;
    int sticky_ = 0;
    uint32_t last_seen_ms_ = 0;
    bool seen_ = false;
    bool still_ = false;   // last resample() bailed because the channel barely moved
    // The anti-alias filter's own response, divided back out of the spectrum
    // (see spectrum()). Cached: it only changes when the frame rate does.
    float aa_gain_[NB_EXT_MAX] = {};
    float aa_alpha_ = 0, aa_dt_ = 0;
    void aa_compensation(float alpha, float dt);
};

} // namespace resp
