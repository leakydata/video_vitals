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

constexpr int kCap = 512;
constexpr float FS = 5.0f;
constexpr float WINDOW_S = 30.0f;
constexpr int N = 150;  // WINDOW_S * FS
// Spectrum: Hann-windowed, zero-padded FFT. Bin k (relative) is (K0 + k) * BR_STEP /min.
constexpr int NFFT = 1024;
constexpr float BR_STEP = 60.0f * FS / NFFT;  // 0.293 breaths/min per bin
constexpr int K0 = 21;                        // first bin: 6.15 /min
constexpr float BR_MIN = K0 * BR_STEP;
constexpr float BR_MAX_ADULT = 45.0f, BR_MAX_INFANT = 78.0f;
constexpr int NB_EXT_MAX = NFFT / 2 - K0;     // up to Nyquist (150 /min)

enum class Band : uint8_t { ADULT, INFANT };

struct Result {
    rppg::State state;
    float brpm;       // tracked breaths per minute (0 if none)
    float raw;        // this window's peak
    float snr_db;
    float quality;
    float stability;
    float agreement;  // fraction of selected channels whose own peak agrees
    float motion;     // masked fraction of the window
    int channels;     // channels fused
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
};

} // namespace resp
