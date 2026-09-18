// rPPG heart-rate estimator. Plain C++ (no ESP-IDF dependencies) so it can be
// unit-tested on the host.
//
// Pipeline, run about once per second over the last WINDOW_S seconds:
//   per ROI:  resample to FS -> pulse signal (POS for RGB, inverted
//             normalised intensity for mono/IR) -> Butterworth bandpass ->
//             spectrum -> SNR
//   fusion:   SNR-weighted average of the ROI spectra
//   peak:     harmonic reinforcement + soft prior around the tracked rate
//   output:   Kalman filter with SNR-dependent measurement noise and
//             outlier rejection
#pragma once
#include <cstdint>

#include "tracker.hpp"

namespace rppg {

constexpr int kRois = 3;          // forehead, left cheek, right cheek
constexpr int kCap = 512;         // raw sample ring buffer
constexpr float FS = 20.0f;       // resample rate (Hz)
#ifndef RPPG_WINDOW_S
#define RPPG_WINDOW_S 10
#endif
constexpr float WINDOW_S = RPPG_WINDOW_S;  // analysis window (s)
constexpr int N = int(RPPG_WINDOW_S * 20); // WINDOW_S * FS
constexpr float BPM_MIN = 40.0f, BPM_MAX = 180.0f, BPM_STEP = 0.5f;
constexpr int NB = 281;           // bins between BPM_MIN and BPM_MAX
constexpr int NB_EXT = 641;       // bins up to 2*BPM_MAX (for harmonics)

enum class Mode : uint8_t { RGB, MONO };

struct RoiSample {
    float r, g, b;
    uint16_t n; // skin pixels used; 0 = invalid
};

struct Sample {
    uint32_t t_ms;
    RoiSample roi[kRois];
    bool motion;
};

struct Result {
    State state;
    float bpm;      // Kalman-filtered heart rate (0 if none)
    float bpm_raw;  // this window's spectral peak
    float snr_db;   // fused-spectrum SNR at the chosen peak
    float quality;  // 0..1
    float stability; // fraction of recent peaks agreeing with this one
    float coherence; // mean pairwise correlation of the ROI pulse signals
    float motion;   // fraction of the window masked out for motion
    float roi_snr[kRois];
    uint8_t rois_used;
};

class Buffer {
public:
    void push(const Sample &s);
    void clear() { count_ = 0; }
    int count() const { return count_; }
    // Copy samples oldest-first into out (capacity kCap); returns count.
    int copy(Sample *out) const;

private:
    Sample buf_[kCap];
    int head_ = 0, count_ = 0;
};

class Estimator {
public:
    Estimator();
    ~Estimator();
    void reset();
    void set_mode(Mode m) { mode_ = m; }
    Mode mode() const { return mode_; }
    // samples oldest-first. now_ms: current time (for Kalman prediction).
    Result update(const Sample *samples, int n, uint32_t now_ms);
    // Band-passed pulse waveform of the best ROI from the last update (N samples).
    const float *pulse() const { return pulse_; }

private:
    bool resample(const Sample *s, int n, int roi);
    float motion_mask(const Sample *s, int n);
    void pulse_signal(float *out);
    void spectrum(const float *x, float *P);

    Mode mode_ = Mode::RGB;
    float *cos_ = nullptr, *sin_ = nullptr; // NB_EXT * N tables
    float c_[3][N];
    float w_[N];  // 1 = clean, 0 = motion (tapered)
    float P_[kRois][NB_EXT];
    float pulse_[N];
    // How many windows in a row have shown no cross-ROI coherence. A lock is
    // allowed to ride out a few, but not to live on evidence that has expired.
    int coh_miss_ = 0;

    Tracker trk_;
};

} // namespace rppg
