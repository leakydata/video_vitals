// Sub-pixel motion of a grid of image tiles, for breathing detection.
//
// Each tile is reduced to a row profile (mean luminance of each row) and a
// column profile. A 1-D Lucas-Kanade step against a per-tile reference
// profile gives the vertical and horizontal shift with sub-pixel precision;
// the accumulated shift is the tile's displacement over time. Breathing
// shows up as a slow periodic displacement of the chest, shoulders or head.
// Plain C++ (host-testable).
#pragma once
#include <cstdint>

namespace resp {

constexpr int GRID_X = 4, GRID_Y = 4;
constexpr int kTiles = GRID_X * GRID_Y;
constexpr int kChan = 2 * kTiles;  // per tile: vertical, horizontal displacement
constexpr int kMaxProf = 64;       // max profile length

struct MotionSample {
    uint32_t t_ms;
    float d[kChan];   // displacement in pixels; channel 2*i = tile i vertical, 2*i+1 horizontal
    uint32_t valid;   // bit per channel: tile has enough texture
    uint32_t jump;    // bit per channel: this channel moved too much this frame (local movement)
    bool gross;       // large movement of the whole scene (not breathing)
};

class TileMotion {
public:
    // Frame is RGB565 big-endian (as captured by esp32-camera), w x h.
    void process_rgb565be(const uint8_t *px, int w, int h, uint32_t t_ms, bool external_motion, MotionSample &out);
    void reset() { have_ref_ = false; }

private:
    void profiles(const uint8_t *px, int w, int h);

    int rows_ = 0, cols_ = 0;  // profile lengths
    bool have_ref_ = false;
    float cur_row_[kTiles][kMaxProf], cur_col_[kTiles][kMaxProf];
    float ref_row_[kTiles][kMaxProf], ref_col_[kTiles][kMaxProf];
    float off_[kChan] = {};    // displacement accumulated at re-keying
    float last_[kChan] = {};   // previous displacement (for gross motion)
};

} // namespace resp
