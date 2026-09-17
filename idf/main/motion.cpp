#include "motion.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace resp {

namespace {

constexpr int STEP = 2;             // pixel subsampling (each profile bin = STEP px)
constexpr float MIN_TEXTURE = 0.004f; // RMS normalised gradient per bin needed for tracking
constexpr float REKEY = 1.0f;       // re-key when a tile drifts this many bins from its reference
constexpr float JUMP_STEP = 0.8f;   // bins per frame that count as a movement (breathing is far slower)
constexpr int GROSS_CHANNELS = 5;    // channels jumping at once = the whole scene moved

float interp(const float *q, int L, float pos)
{
    if (pos <= 0) return q[0];
    if (pos >= L - 1) return q[L - 1];
    const int i = int(pos);
    const float u = pos - i;
    return q[i] + u * (q[i + 1] - q[i]);
}

// Shift d (bins) such that cur[i] ~= ref[i - d]. Profiles are normalised by
// their means first, so global brightness changes do not register as motion.
bool lk_shift(const float *ref, const float *cur, int L, float &d)
{
    float mr = 0, mc = 0;
    for (int i = 0; i < L; i++) { mr += ref[i]; mc += cur[i]; }
    if (mr <= 0 || mc <= 0) return false;
    const float sr = L / mr, sc = L / mc;
    d = 0;
    for (int it = 0; it < 4; it++) {
        float num = 0, den = 0;
        int cnt = 0;
        for (int i = 4; i < L - 4; i++) {
            const float pos = i - d;
            const float q = interp(ref, L, pos) * sr;
            const float g = 0.5f * (interp(ref, L, pos + 1) - interp(ref, L, pos - 1)) * sr;
            num += g * (q - cur[i] * sc);
            den += g * g;
            cnt++;
        }
        if (cnt == 0 || std::sqrt(den / cnt) < MIN_TEXTURE) return false;
        const float dd = num / den;
        d = std::clamp(d + dd, -3.0f, 3.0f);
        if (std::fabs(dd) < 0.005f) break;
    }
    return true;
}

} // namespace

void TileMotion::profiles(const uint8_t *px, int w, int h)
{
    const int tw = w / GRID_X, th = h / GRID_Y;
    rows_ = std::min(th / STEP, kMaxProf);
    cols_ = std::min(tw / STEP, kMaxProf);
    for (int t = 0; t < kTiles; t++) {
        std::memset(cur_row_[t], 0, sizeof(float) * rows_);
        std::memset(cur_col_[t], 0, sizeof(float) * cols_);
    }
    for (int ty = 0; ty < GRID_Y; ty++) {
        for (int r = 0; r < rows_; r++) {
            const int y = ty * th + r * STEP;
            const uint8_t *row = px + y * w * 2;
            for (int tx = 0; tx < GRID_X; tx++) {
                const int t = ty * GRID_X + tx;
                float rs = 0;
                for (int c = 0; c < cols_; c++) {
                    const int x = tx * tw + c * STEP;
                    const uint16_t v = (row[x * 2] << 8) | row[x * 2 + 1];
                    const int Y = 77 * (((v >> 11) & 0x1f) << 3) + 150 * (((v >> 5) & 0x3f) << 2) + 29 * ((v & 0x1f) << 3);
                    rs += Y;
                    cur_col_[t][c] += Y;
                }
                cur_row_[t][r] = rs;
            }
        }
    }
}

void TileMotion::process_rgb565be(const uint8_t *px, int w, int h, uint32_t t_ms, bool external_motion,
                                  MotionSample &out)
{
    profiles(px, w, h);
    out.t_ms = t_ms;
    out.valid = 0;
    out.jump = 0;
    out.gross = external_motion;
    if (!have_ref_) {
        std::memcpy(ref_row_, cur_row_, sizeof(ref_row_));
        std::memcpy(ref_col_, cur_col_, sizeof(ref_col_));
        std::memset(off_, 0, sizeof(off_));
        std::memset(last_, 0, sizeof(last_));
        have_ref_ = true;
    }

    int jumps = 0;
    for (int t = 0; t < kTiles; t++) {
        for (int axis = 0; axis < 2; axis++) {
            const int ch = 2 * t + axis;
            float *ref = axis == 0 ? ref_row_[t] : ref_col_[t];
            const float *cur = axis == 0 ? cur_row_[t] : cur_col_[t];
            const int L = axis == 0 ? rows_ : cols_;
            float d;
            if (!lk_shift(ref, cur, L, d)) {
                std::memcpy(ref, cur, sizeof(float) * L);  // try again from here
                out.d[ch] = off_[ch] * STEP;
                continue;
            }
            const float disp = off_[ch] + d;
            const bool jumped = std::fabs(disp - last_[ch]) > JUMP_STEP;
            last_[ch] = disp;
            out.d[ch] = disp * STEP;
            out.valid |= 1u << ch;
            if (jumped) {
                jumps++;
                out.jump |= 1u << ch;
            }
            if (jumped || std::fabs(d) > REKEY) {
                off_[ch] = disp;
                std::memcpy(ref, cur, sizeof(float) * L);
            }
        }
    }
    if (jumps >= GROSS_CHANNELS) out.gross = true;
    if (out.gross) {
        // after a big movement the old references are meaningless
        std::memcpy(ref_row_, cur_row_, sizeof(ref_row_));
        std::memcpy(ref_col_, cur_col_, sizeof(ref_col_));
        for (int ch = 0; ch < kChan; ch++) off_[ch] = last_[ch];
    }
}

} // namespace resp
