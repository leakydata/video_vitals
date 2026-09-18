#include "evm.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace evm {

static constexpr float LIMIT = 60.0f;  // largest brightness change the magnifier may add

void Magnifier::process_yuyv(uint8_t *px, int w, int h, float dt)
{
    const int sw = w >> LEVELS, sh = h >> LEVELS;
    if (sw * sh > MAX_SMALL || sw < 4 || sh < 4) return;
    if (sw != sw_ || sh != sh_) {
        sw_ = sw;
        sh_ = sh;
        have_ = false;
    }
    const int step = 1 << LEVELS;

    // Downscale: average the luma of each step x step block. (A box average is
    // the cheap stand-in for the paper's Gaussian pyramid; at this reduction the
    // difference is not visible in the result.)
    // One sequential pass over the frame: reading each row once is much kinder
    // to the cache (and to PSRAM) than revisiting rows per output pixel.
    static uint32_t acc[MAX_SMALL];
    std::memset(acc, 0, sizeof(uint32_t) * sw * sh);
    for (int y = 0; y < h; y++) {
        const uint8_t *row = px + y * w * 2;
        uint32_t *arow = acc + (y >> LEVELS) * sw;
        for (int x = 0; x < w; x++) arow[x >> LEVELS] += row[x * 2];
    }
    const float inv = 1.0f / (step * step);
    for (int i = 0; i < sw * sh; i++) small_[i] = acc[i] * inv;

    // Spatial low-pass of the small image: the paper's spatial cut-off, which
    // stops fine detail (mostly sensor noise) being amplified.
    for (int y = 0; y < sh; y++) {
        for (int x = 0; x < sw; x++) {
            const int x0 = std::max(x - 1, 0), x1 = std::min(x + 1, sw - 1);
            const int y0 = std::max(y - 1, 0), y1 = std::min(y + 1, sh - 1);
            blur_[y * sw + x] = 0.25f * small_[y * sw + x] +
                                0.1875f * (small_[y * sw + x0] + small_[y * sw + x1] +
                                           small_[y0 * sw + x] + small_[y1 * sw + x]);
        }
    }

    if (!have_) {
        std::memcpy(low1_, blur_, sizeof(float) * sw * sh);
        std::memcpy(low2_, blur_, sizeof(float) * sw * sh);
        have_ = true;
        return;  // nothing to amplify from a single frame
    }

    // Two one-pole filters; their difference is the band to amplify.
    const float k1 = 1 - std::exp(-2 * float(M_PI) * hi_ * dt);
    const float k2 = 1 - std::exp(-2 * float(M_PI) * lo_ * dt);
    for (int i = 0; i < sw * sh; i++) {
        low1_[i] += k1 * (blur_[i] - low1_[i]);
        low2_[i] += k2 * (blur_[i] - low2_[i]);
    }

    // Add the amplified band back. The inner loop must stay cheap: the
    // interpolation weights are the same for every row, so they are computed
    // once into tables, and each output row is built by interpolating the band
    // into a single line buffer before touching the frame.
    static int16_t xi0[1024], xi1[1024];
    static float xw[1024];
    static int cached_w = 0, cached_sw = 0;
    if (cached_w != w || cached_sw != sw) {
        for (int x = 0; x < w && x < 1024; x++) {
            const float fx = (x + 0.5f) / step - 0.5f;
            int x0 = int(std::floor(fx));
            float wx = fx - x0;
            x0 = std::clamp(x0, 0, sw - 1);
            xi0[x] = x0;
            xi1[x] = std::min(x0 + 1, sw - 1);
            xw[x] = std::clamp(wx, 0.0f, 1.0f);
        }
        cached_w = w;
        cached_sw = sw;
    }

    static float line[1024];
    for (int y = 0; y < h; y++) {
        const float fy = (y + 0.5f) / step - 0.5f;
        int y0 = int(std::floor(fy));
        const float wy = std::clamp(fy - y0, 0.0f, 1.0f);
        y0 = std::clamp(y0, 0, sh - 1);
        const int y1 = std::min(y0 + 1, sh - 1);
        const float *b0 = low1_ + y0 * sw, *c0 = low2_ + y0 * sw;
        const float *b1 = low1_ + y1 * sw, *c1 = low2_ + y1 * sw;
        for (int x = 0; x < sw; x++) {
            const float top = b0[x] - c0[x], bot = b1[x] - c1[x];
            // The linear method assumes small changes. A real movement (someone
            // turning their head) breaks that and would blow the picture out, so
            // the amplified band is capped: subtle motion is exaggerated, gross
            // motion is simply not.
            const float v = alpha_ * (top + wy * (bot - top));
            line[x] = std::clamp(v, -LIMIT, LIMIT);
        }
        uint8_t *row = px + y * w * 2;
        for (int x = 0; x < w; x++) {
            const float a = line[xi0[x]], b = line[xi1[x]];
            const float v = row[x * 2] + a + xw[x] * (b - a);
            row[x * 2] = uint8_t(v < 0 ? 0 : (v > 255 ? 255 : v));
        }
    }
}

} // namespace evm
