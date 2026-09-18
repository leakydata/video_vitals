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
// Channels: two per tile (vertical, horizontal), plus two per face-anchored
// box. Box 0 is the chest (under the face), box 1 the head: breathing moves
// the chest most, but head motion carries it too when the chest is hidden.
constexpr int kBoxes = 2;
constexpr int kChan = 2 * kTiles + 2 * kBoxes;
constexpr int kBoxChan0 = 2 * kTiles;   // channel of box 0, vertical (then horizontal)
constexpr int kMaxProf = 64;       // max profile length
constexpr int kBoxProf = 48;       // profile bins across a box (whatever its size)

struct Box {
    int16_t x, y, w, h;
    bool valid() const { return w > 16 && h > 16; }
};

struct MotionSample {
    uint32_t t_ms;
    float d[kChan];   // displacement in pixels; channel 2*i = tile i vertical, 2*i+1 horizontal
    uint64_t valid;   // bit per channel: tile has enough texture (34 channels: 64-bit mask)
    uint64_t jump;    // bit per channel: this channel moved too much this frame (local movement)
    uint64_t subject; // bit per channel: this channel covers the subject (0 = unknown, treat all alike)
    // Which tiles each face-anchored box lies over (bit per tile). A box and
    // the tiles beneath it watch the same piece of the subject, so they are not
    // independent evidence; the estimator needs this to say how many genuinely
    // separate regions agree.
    uint16_t box_tiles[kBoxes];
    bool gross;       // large movement of the whole scene (not breathing)
};

class TileMotion {
public:
    // Frame is YUV422 (YUYV, as captured by esp32-camera), w x h.
    // `boxes` are the face-anchored boxes (chest, head); pass invalid Boxes for none.
    void process_yuyv(const uint8_t *px, int w, int h, uint32_t t_ms, bool external_motion,
                          const Box boxes[kBoxes], MotionSample &out);
    void reset() { have_ref_ = false; for (bool &b : have_box_) b = false; }

private:
    void profiles(const uint8_t *px, int w, int h);
    void box_profiles(const uint8_t *px, int w, int h, const Box &b, int i);

    int rows_ = 0, cols_ = 0;  // profile lengths
    bool have_ref_ = false;
    float cur_row_[kTiles][kMaxProf], cur_col_[kTiles][kMaxProf];
    float ref_row_[kTiles][kMaxProf], ref_col_[kTiles][kMaxProf];
    float off_[kChan] = {};    // displacement accumulated at re-keying
    float last_[kChan] = {};   // previous displacement (for gross motion)
    float cur_brow_[kBoxes][kBoxProf], cur_bcol_[kBoxes][kBoxProf];
    float ref_brow_[kBoxes][kBoxProf], ref_bcol_[kBoxes][kBoxProf];
    Box box_ref_[kBoxes] = {};   // the box each reference profile belongs to
    // Where the box sat when each box channel's reference profile was taken.
    // The profile is measured in box coordinates, so moving the box shifts the
    // content against the reference even when nothing in the scene moved; the
    // shift is known exactly from the geometry and is subtracted out.
    float gref_[2 * kBoxes] = {};
    bool have_box_[kBoxes] = {};
};

} // namespace resp
