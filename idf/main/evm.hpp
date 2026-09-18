// Eulerian video magnification, on the device.
//
// Wu et al., "Eulerian Video Magnification for Revealing Subtle Changes in the
// World" (SIGGRAPH 2012): band-pass each pixel of a blurred, downscaled copy of
// the video over time, amplify that, and add it back. Slow motion shows up as
// brightness change at edges, so amplifying the band that contains breathing
// exaggerates the movement itself.
//
// This runs on the ESP32-S3 at QVGA in a few milliseconds a frame, because the
// temporal filtering happens at a coarse pyramid level (40x30 by default) and
// only the add-back touches every pixel. It is applied to the streamed preview
// only: the vitals are always measured from the untouched frame.
//
// Plain C++ apart from the YUYV layout, so it can be tested on a host.
#pragma once
#include <cstdint>

namespace evm {

constexpr int LEVELS = 3;        // 320x240 -> 40x30
constexpr int MAX_SMALL = 64 * 48;

class Magnifier {
public:
    // lo/hi in Hz (the band to amplify), alpha the gain, dt the frame interval.
    void set_band(float lo, float hi) { lo_ = lo; hi_ = hi; }
    void set_alpha(float a) { alpha_ = a; }
    void reset() { have_ = false; }

    // In-place on a YUYV frame: amplifies luma (where motion lives) and leaves
    // chroma alone. w and h must be multiples of 1 << LEVELS.
    void process_yuyv(uint8_t *px, int w, int h, float dt);

private:
    float lo_ = 0.1f, hi_ = 0.8f, alpha_ = 25.0f;
    bool have_ = false;
    int sw_ = 0, sh_ = 0;
    float small_[MAX_SMALL];   // downscaled luma
    float blur_[MAX_SMALL];
    float low1_[MAX_SMALL], low2_[MAX_SMALL];  // the two one-pole filters
};

} // namespace evm
