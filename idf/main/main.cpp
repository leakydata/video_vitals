// video_vitals — contactless heart rate and breathing on a XIAO ESP32-S3 Sense.
//
// Tasks
//   cam     (core 1)  YUV422 QVGA capture, per-ROI skin colour means
//   detect  (core 0)  ESP-DL ESPDet-Pico + MNP landmarks, smoothing, motion flag
//   hr      (core 1)  rPPG (rppg.cpp) and breathing (resp.cpp) estimators once per second
//   stream  (core 0)  optional JPEG preview with face/ROI metadata
//   cmd, led
//
// Serial protocol (USB Serial/JTAG)
//   device -> host  text lines: "HR ...", "RR ..." (breathing), "S t r g b n ... motion",
//                   "M t valid jump subject gross d0..d35" (region displacements),
//                   "# log"; binary frames:
//                   'H','C','F','2' | u32 len | u32 t_ms | i16 meta[26] | jpeg
//                   meta = face box x1,y1,x2,y2 | 5 landmarks (x,y) | 3 ROIs x,y,w,h |
//                          chest box x,y,w,h
//                   (-1 when absent)
//   host -> device  one command per line:
//     s/x stream on/off     v/w per-frame samples on/off
//     a   auto-expose 2 s then lock          e<n> exposure   g<n> gain
//     q<n> JPEG quality (1-100)              m0/m1 RGB / mono(IR) mode
//     b0/b1 breathing band adult (6-45/min) / infant (6-78/min)
//     r   rotate image 180 degrees (also automatic when a face is upside down)
//     i   info
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <list>

#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <atomic>

#include "dl_image_jpeg.hpp"
#include "human_face_detect.hpp"
#include "motion.hpp"
#include "resp.hpp"
#include "rppg.hpp"

static const char *TAG = "heart_cam";

// ---------------------------------------------------------------- pins
static constexpr int CAM_XCLK = 10, CAM_SIOD = 40, CAM_SIOC = 39;
static constexpr int CAM_D[8] = {15, 17, 18, 16, 14, 12, 11, 48};
static constexpr int CAM_VSYNC = 38, CAM_HREF = 47, CAM_PCLK = 13;
static constexpr gpio_num_t LED_PIN = GPIO_NUM_21;  // active low

static constexpr int W = 320, H = 240;
static constexpr float JUMP_MOTION = 0.04f;   // relative frame-to-frame ROI brightness change counted as movement
static constexpr uint16_t MIN_ROI_PIXELS = 40;  // below this an ROI mean is mostly noise
static constexpr float FACE_MOTION = 0.35f;   // face-widths per second counted as movement
static constexpr int64_t FACE_HOLD_US = 2500000;  // keep using the last face this long after a miss

// ---------------------------------------------------------------- output
static SemaphoreHandle_t out_mtx;

static bool out_raw(const void *data, size_t len)
{
    if (!usb_serial_jtag_is_connected()) return false;
    const uint8_t *p = (const uint8_t *)data;
    while (len) {
        const size_t n = len > 4096 ? 4096 : len;
        const int w = usb_serial_jtag_write_bytes(p, n, pdMS_TO_TICKS(50));
        if (w <= 0) return false;
        p += w;
        len -= w;
    }
    return true;
}

static void out_printf(const char *fmt, ...)
{
    char buf[384];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n <= 0) return;
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
    xSemaphoreTake(out_mtx, portMAX_DELAY);
    out_raw(buf, n);
    xSemaphoreGive(out_mtx);
}

static int log_vprintf(const char *fmt, va_list ap)
{
    char buf[256];
    buf[0] = '#';
    buf[1] = ' ';
    int n = vsnprintf(buf + 2, sizeof(buf) - 2, fmt, ap);
    if (n <= 0) return n;
    n = std::min<int>(n + 2, sizeof(buf) - 1);
    xSemaphoreTake(out_mtx, portMAX_DELAY);
    out_raw(buf, n);
    xSemaphoreGive(out_mtx);
    return n;
}

// ---------------------------------------------------------------- shared state
struct Rect { int16_t x, y, w, h; };

struct Face {
    bool valid = false;
    int64_t t_us = 0;       // last detection time
    float box[4];           // smoothed x1,y1,x2,y2
    float kp[10];           // smoothed landmarks
    float eye_l[2], eye_r[2], mouth[2];
    int64_t motion_until_us = 0;
};

static SemaphoreHandle_t face_mtx, buf_mtx;
static Face g_face;
static rppg::Buffer *g_buf;
static rppg::Estimator *g_est;
static resp::Buffer *g_rbuf;
static resp::Estimator *g_rest;
static resp::TileMotion *g_tiles;

static struct {
    bool stream = false, samples = false, mono = false, infant = false;
    int quality = 80;
    volatile bool relock = false;
    volatile bool flip_request = false;  // rotate the sensor image 180 degrees
    bool flipped = false;
} cfg;

static struct {
    float cam_fps = 0, det_fps = 0, det_ms = 0, det_hit = 0;
    int brightness = 0;  // mean luminance of the frame centre
    rppg::Result res{};
    resp::Result rr{};
    SemaphoreHandle_t mtx;  // guards res/rr (written by hr, read by led)
} stats;

// detector / stream frame handoff
static uint8_t *det_frame, *str_frame;
static uint32_t str_t_ms;
static int16_t str_meta[30];  // face box, 5 landmarks, 3 ROIs, chest box
static SemaphoreHandle_t det_sem, str_sem;
static std::atomic<bool> det_busy{false}, str_busy{false};

// ---------------------------------------------------------------- camera
static void apply_flip();
static bool camera_init()
{
    camera_config_t c = {};
    c.pin_pwdn = -1;
    c.pin_reset = -1;
    c.pin_xclk = CAM_XCLK;
    c.pin_sccb_sda = CAM_SIOD;
    c.pin_sccb_scl = CAM_SIOC;
    c.pin_d0 = CAM_D[0]; c.pin_d1 = CAM_D[1]; c.pin_d2 = CAM_D[2]; c.pin_d3 = CAM_D[3];
    c.pin_d4 = CAM_D[4]; c.pin_d5 = CAM_D[5]; c.pin_d6 = CAM_D[6]; c.pin_d7 = CAM_D[7];
    c.pin_vsync = CAM_VSYNC;
    c.pin_href = CAM_HREF;
    c.pin_pclk = CAM_PCLK;
    // 24 MHz gives ~15 fps at QVGA (20 MHz gives 11). 30 MHz runs at ~17 fps and
    // looked clean in testing, but 24 keeps margin for long unattended runs.
    c.xclk_freq_hz = 24000000;
    c.ledc_timer = LEDC_TIMER_0;
    c.ledc_channel = LEDC_CHANNEL_0;
    // YUV422 (YUYV), not RGB565: 8 bits of luminance and chroma per sample
    // instead of 5/6/5, so quantisation noise no longer limits the pulse, which
    // is a few tenths of a percent of the signal. Uncompressed either way.
    c.pixel_format = PIXFORMAT_YUV422;
    c.frame_size = FRAMESIZE_QVGA;
    c.jpeg_quality = 12;
    c.fb_count = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
    c.grab_mode = CAMERA_GRAB_LATEST;
    esp_err_t err = esp_camera_init(&c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "camera init failed 0x%x", err);
        return false;
    }
    sensor_t *s = esp_camera_sensor_get();
    ESP_LOGI(TAG, "sensor PID=0x%04x", s->id.PID);
    apply_flip();
    s->set_lenc(s, 1);
    s->set_bpc(s, 1);
    s->set_wpc(s, 1);
    return true;
}

// Auto exposure / white balance would inject steps far larger than the pulse:
// let them settle once, then freeze.
static void auto_then_lock(int ms)
{
    sensor_t *s = esp_camera_sensor_get();
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    if (s->id.PID == OV3660_PID) s->set_reg(s, 0x3406, 0x01, 0x00);  // auto AWB gains
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);
    const int64_t end = esp_timer_get_time() + ms * 1000LL;
    while (esp_timer_get_time() < end) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) esp_camera_fb_return(fb);
    }
    s->set_exposure_ctrl(s, 0);
    s->set_gain_ctrl(s, 0);
    if (s->id.PID == OV3660_PID) {
        // Turning the AWB block off would drop the white-balance gains entirely
        // (green image). Instead copy the current auto gains (read-only
        // 0x519F..0x51A4) into the manual gain registers and switch to manual.
        uint8_t g[6];
        for (int i = 0; i < 6; i++) g[i] = s->get_reg(s, 0x519F + i, 0xFF);
        for (int i = 0; i < 6; i++) s->set_reg(s, 0x3400 + i, 0xFF, g[i]);
        s->set_reg(s, 0x3406, 0x01, 0x01);
        ESP_LOGI(TAG, "AWB gains R=%d G=%d B=%d (x1024)", (g[0] << 8) | g[1], (g[2] << 8) | g[3], (g[4] << 8) | g[5]);
    } else {
        // Other sensors (e.g. OV2640) have no readable AWB gains, and turning
        // the AWB block off drops white balance entirely (a green image). Leave
        // it on: its drift is slow, and POS normalises each channel anyway.
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        ESP_LOGI(TAG, "AWB left automatic (no manual gains on sensor 0x%04x)", s->id.PID);
    }
    xSemaphoreTake(buf_mtx, portMAX_DELAY);
    g_buf->clear();
    xSemaphoreGive(buf_mtx);
    ESP_LOGI(TAG, "exposure/gain/white balance locked");
}

static void apply_flip()
{
    sensor_t *s = esp_camera_sensor_get();
    const bool base_vflip = s->id.PID == OV3660_PID;
    s->set_vflip(s, base_vflip ^ cfg.flipped);
    s->set_hmirror(s, cfg.flipped);
}

// Chest box: below the face, about twice its width. Breathing moves it far more
// than it moves the face, so it is the best place to measure when a face is
// visible; the tile grid covers the rest (and the no-face case).
static resp::Box chest_box(const Face &f)
{
    const float fw = f.box[2] - f.box[0], fh = f.box[3] - f.box[1];
    if (fw < 20 || fh < 20) return {0, 0, 0, 0};
    const float want_w = 2.2f * fw, want_h = 1.2f * fh;
    const float cx = (f.box[0] + f.box[2]) / 2;
    // slide the box inside the frame before clipping, so a face near an edge
    // still gets a usable chest box
    float x0 = std::clamp(cx - want_w / 2, 0.0f, std::max(0.0f, W - want_w));
    float y0 = std::clamp(f.box[3] + 0.15f * fh, 0.0f, std::max(0.0f, H - want_h));
    const float x1 = std::min<float>(x0 + want_w, W), y1 = std::min<float>(y0 + want_h, H);
    const resp::Box b{int16_t(std::lround(x0)), int16_t(std::lround(y0)), int16_t(std::lround(x1 - x0)),
                      int16_t(std::lround(y1 - y0))};
    // too little of it left (face at the very bottom of the frame): no chest box
    if (b.w < 0.5f * want_w || b.h < 0.4f * want_h || b.w < 40 || b.h < 30) return {0, 0, 0, 0};
    return b;
}

// ---------------------------------------------------------------- ROIs
// Forehead and both cheeks, placed from the landmarks. Horizontal sizes scale
// with the eye distance, vertical ones with the eye-to-mouth distance, so the
// boxes follow foreshortening when the camera looks up or down at the face.
// Every ROI is clipped to the face box and dropped if less than half remains.
static int face_rois(const Face &f, Rect out[rppg::kRois])
{
    const float dx = f.eye_r[0] - f.eye_l[0], dy = f.eye_r[1] - f.eye_l[1];
    const float d = std::sqrt(dx * dx + dy * dy);
    if (d < 8) return 0;
    const float mx = (f.eye_l[0] + f.eye_r[0]) / 2, my = (f.eye_l[1] + f.eye_r[1]) / 2;
    // eye-to-mouth distance measured perpendicular to the eye line
    const float em = std::fabs((f.mouth[0] - mx) * (-dy / d) + (f.mouth[1] - my) * (dx / d));
    const float v = std::clamp(em / 1.1f, 0.5f * d, 1.5f * d);
    struct { float cx, cy, w, h; } r[rppg::kRois] = {
        {mx, my - 0.80f * v, 1.20f * d, 0.55f * v},                               // forehead
        {f.eye_l[0] - 0.10f * d, f.eye_l[1] + 0.60f * v, 0.60f * d, 0.45f * v},  // cheek (image left)
        {f.eye_r[0] + 0.10f * d, f.eye_r[1] + 0.60f * v, 0.60f * d, 0.45f * v},  // cheek (image right)
    };
    const float bw = f.box[2] - f.box[0];
    const float bx0 = std::max(0.0f, f.box[0] + 0.05f * bw), bx1 = std::min<float>(W, f.box[2] - 0.05f * bw);
    const float by0 = std::max(0.0f, f.box[1]), by1 = std::min<float>(H, f.box[3]);
    for (int i = 0; i < rppg::kRois; i++) {
        const float x0 = r[i].cx - r[i].w / 2, x1 = r[i].cx + r[i].w / 2;
        const float y0 = r[i].cy - r[i].h / 2, y1 = r[i].cy + r[i].h / 2;
        const float cx0 = std::max(x0, bx0), cx1 = std::min(x1, bx1);
        const float cy0 = std::max(y0, by0), cy1 = std::min(y1, by1);
        const float kept = std::max(0.0f, cx1 - cx0) * std::max(0.0f, cy1 - cy0);
        if (kept < 0.5f * r[i].w * r[i].h) {
            out[i] = {0, 0, 0, 0};
            continue;
        }
        const int ix0 = std::lround(cx0), iy0 = std::lround(cy0);
        out[i] = {int16_t(ix0), int16_t(iy0), int16_t(std::lround(cx1) - ix0), int16_t(std::lround(cy1) - iy0)};
    }
    return rppg::kRois;
}

// Mean colour of a ROI from a YUYV frame: the skin test uses the sensor's own
// chroma (Cb = U, Cr = V) and RGB is reconstructed at full 8-bit precision.
static rppg::RoiSample roi_mean(const uint8_t *px, const Rect &r, bool any_colour)
{
    rppg::RoiSample out{0, 0, 0, 0};
    if (r.w < 4 || r.h < 4) return out;
    uint32_t sr = 0, sg = 0, sb = 0, n = 0;       // skin-classified pixels
    uint32_t ar = 0, ag = 0, ab = 0, an = 0;      // all non-clipped pixels
    for (int y = r.y; y < r.y + r.h; y++) {
        const uint8_t *row = px + y * W * 2;
        for (int x = r.x; x < r.x + r.w; x++) {
            const int Y = row[x * 2];
            // chroma is shared by each pixel pair: U at the even sample, V at the odd
            const int cbase = (x & ~1) * 2;
            const int Cb = row[cbase + 1], Cr = row[cbase + 3];
            const int cd = Cr - 128, ce = Cb - 128;
            const int R = std::clamp(Y + ((91881 * cd) >> 16), 0, 255);
            const int G = std::clamp(Y - ((22554 * ce + 46802 * cd) >> 16), 0, 255);
            const int B = std::clamp(Y + ((116130 * ce) >> 16), 0, 255);
            if (R > 250 || G > 250 || B > 250) continue;  // clipped / specular
            if (Y < 25) continue;
            ar += R; ag += G; ab += B; an++;
            if (Cr < 133 || Cr > 173 || Cb < 77 || Cb > 127) continue;
            sr += R; sg += G; sb += B; n++;
        }
    }
    // colour mode: an ROI that is mostly not skin (hair, background) is dropped.
    // mono/IR mode has no usable colour, so all well-exposed pixels are used.
    if (!any_colour && n >= an * 4 / 10 && n >= 16) {
        out = {float(sr) / n, float(sg) / n, float(sb) / n, uint16_t(std::min<uint32_t>(n, 65535))};
    } else if (any_colour && an >= 16) {
        out = {float(ar) / an, float(ag) / an, float(ab) / an, uint16_t(std::min<uint32_t>(an, 65535))};
    }
    return out;
}

// ---------------------------------------------------------------- tasks
static void cam_task(void *)
{
    int frames = 0;
    int64_t fps_t0 = esp_timer_get_time();
    Rect rois[rppg::kRois];
    Rect held[rppg::kRois] = {};
    bool have_held = false;
    float gain[rppg::kRois][3] = {{1, 1, 1}, {1, 1, 1}, {1, 1, 1}};
    rppg::RoiSample prev[rppg::kRois] = {};
    int64_t jump_until = 0;

    for (;;) {
        if (cfg.relock) {
            cfg.relock = false;
            auto_then_lock(2000);
        }
        if (cfg.flip_request) {
            cfg.flip_request = false;
            cfg.flipped = !cfg.flipped;
            apply_flip();
            ESP_LOGI(TAG, "image rotated 180 (flipped=%d)", cfg.flipped);
            for (int i = 0; i < 3; i++) {  // drop frames captured before the change
                camera_fb_t *old = esp_camera_fb_get();
                if (old) esp_camera_fb_return(old);
            }
            xSemaphoreTake(buf_mtx, portMAX_DELAY);
            g_buf->clear();
            g_rbuf->clear();
            xSemaphoreGive(buf_mtx);
            g_tiles->reset();
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) continue;
        const uint32_t t_ms = fb->timestamp.tv_sec * 1000ULL + fb->timestamp.tv_usec / 1000;
        {
            // mean luminance of the frame centre: tells us whether a comparison
            // between cameras or lenses was made in comparable light
            uint32_t sum = 0, cnt = 0;
            for (int y = H / 4; y < 3 * H / 4; y += 4)
                for (int x = W / 4; x < 3 * W / 4; x += 4) { sum += fb->buf[(y * W + x) * 2]; cnt++; }
            stats.brightness = cnt ? sum / cnt : 0;
        }
        const int64_t now = esp_timer_get_time();

        Face f;
        xSemaphoreTake(face_mtx, portMAX_DELAY);
        f = g_face;
        xSemaphoreGive(face_mtx);
        const bool face_ok = f.valid && now - f.t_us < FACE_HOLD_US;

        rppg::Sample s{};
        s.t_ms = t_ms;
        s.motion = now < f.motion_until_us;
        int nroi = face_ok ? face_rois(f, rois) : 0;
        if (nroi) {
            // Dead-band: moving the ROI by a pixel changes which pixels are
            // averaged, which is itself noise. Only follow real movement.
            bool moved = !have_held;
            for (int i = 0; i < nroi && !moved; i++)
                moved = std::abs(rois[i].x - held[i].x) > 2 || std::abs(rois[i].y - held[i].y) > 2 ||
                        std::abs(rois[i].w - held[i].w) > 3 || std::abs(rois[i].h - held[i].h) > 3;
            if (moved) {
                // Relocating an ROI puts a step in its colour series. Measure
                // both placements on this frame and rescale so the series
                // stays continuous.
                for (int i = 0; i < nroi; i++) {
                    const rppg::RoiSample a = have_held ? roi_mean(fb->buf, held[i], cfg.mono) : rppg::RoiSample{};
                    const rppg::RoiSample b = roi_mean(fb->buf, rois[i], cfg.mono);
                    if (a.n && b.n && b.r > 0 && b.g > 0 && b.b > 0) {
                        gain[i][0] = std::clamp(gain[i][0] * a.r / b.r, 0.3f, 3.0f);
                        gain[i][1] = std::clamp(gain[i][1] * a.g / b.g, 0.3f, 3.0f);
                        gain[i][2] = std::clamp(gain[i][2] * a.b / b.b, 0.3f, 3.0f);
                    } else {
                        gain[i][0] = gain[i][1] = gain[i][2] = 1;
                    }
                }
                std::memcpy(held, rois, sizeof(held));
                have_held = true;
            }
            for (int i = 0; i < nroi; i++) {
                rppg::RoiSample m = roi_mean(fb->buf, held[i], cfg.mono);
                m.r *= gain[i][0];
                m.g *= gain[i][1];
                m.b *= gain[i][2];
                // a large frame-to-frame brightness jump means the subject or
                // camera moved (the pulse changes brightness by well under 1%)
                if (m.n < MIN_ROI_PIXELS) m = {0, 0, 0, 0};  // too few pixels: mostly noise
                if (m.n && prev[i].n) {
                    // the noise of an ROI mean grows as 1/sqrt(pixels): scale the threshold
                    const float y0 = prev[i].r + prev[i].g + prev[i].b, y1 = m.r + m.g + m.b;
                    const float thr = JUMP_MOTION * std::max(1.0f, std::sqrt(200.0f / std::min(m.n, prev[i].n)));
                    if (y0 > 0 && std::fabs(y1 - y0) / y0 > thr) jump_until = now + 300000;
                }
                prev[i] = m;
                s.roi[i] = m;
            }
            s.motion = s.motion || now < jump_until;
        } else {
            have_held = false;
            for (auto &p : prev) p.n = 0;
        }

        resp::MotionSample ms;
        resp::Box boxes[resp::kBoxes] = {};
        if (face_ok) {
            boxes[0] = chest_box(f);
            // head box: the face box itself, which breathing also moves slightly
            const int16_t hx = int16_t(std::lround(f.box[0])), hy = int16_t(std::lround(f.box[1]));
            boxes[1] = {hx, hy, int16_t(std::lround(f.box[2]) - hx), int16_t(std::lround(f.box[3]) - hy)};
        }
        const resp::Box &chest = boxes[0];
        g_tiles->process_yuyv(fb->buf, W, H, t_ms, now < f.motion_until_us, boxes, ms);

        xSemaphoreTake(buf_mtx, portMAX_DELAY);
        g_buf->push(s);
        g_rbuf->push(ms);
        xSemaphoreGive(buf_mtx);

        if (cfg.samples) {
            out_printf("S %lu %.2f %.2f %.2f %u %.2f %.2f %.2f %u %.2f %.2f %.2f %u %d\n", (unsigned long)t_ms,
                       s.roi[0].r, s.roi[0].g, s.roi[0].b, s.roi[0].n, s.roi[1].r, s.roi[1].g, s.roi[1].b,
                       s.roi[1].n, s.roi[2].r, s.roi[2].g, s.roi[2].b, s.roi[2].n, s.motion);
            char line[640];
            int len = snprintf(line, sizeof(line), "M %lu %llu %llu %llu %d", (unsigned long)t_ms,
                               (unsigned long long)ms.valid, (unsigned long long)ms.jump,
                               (unsigned long long)ms.subject, ms.gross);
            for (int c = 0; c < resp::kChan && len < (int)sizeof(line) - 12; c++)
                len += snprintf(line + len, sizeof(line) - len, " %.3f", ms.d[c]);
            line[len++] = '\n';
            xSemaphoreTake(out_mtx, portMAX_DELAY);
            out_raw(line, len);
            xSemaphoreGive(out_mtx);
        }

        if (!det_busy) {
            std::memcpy(det_frame, fb->buf, W * H * 2);
            det_busy = true;
            xSemaphoreGive(det_sem);
        }
        if (cfg.stream && !str_busy) {
            std::memcpy(str_frame, fb->buf, W * H * 2);
            str_t_ms = t_ms;
            for (auto &m : str_meta) m = -1;
            if (face_ok) {
                for (int i = 0; i < 4; i++) str_meta[i] = std::lround(f.box[i]);
                for (int i = 0; i < 10; i++) str_meta[4 + i] = std::lround(f.kp[i]);
            }
            if (have_held)
                for (int i = 0; i < rppg::kRois; i++) {
                    str_meta[14 + 4 * i] = held[i].x;
                    str_meta[15 + 4 * i] = held[i].y;
                    str_meta[16 + 4 * i] = held[i].w;
                    str_meta[17 + 4 * i] = held[i].h;
                }
            str_meta[26] = chest.valid() ? chest.x : -1;
            str_meta[27] = chest.valid() ? chest.y : -1;
            str_meta[28] = chest.valid() ? chest.w : -1;
            str_meta[29] = chest.valid() ? chest.h : -1;
            str_busy = true;
            xSemaphoreGive(str_sem);
        }
        esp_camera_fb_return(fb);

        if (++frames == 40) {
            stats.cam_fps = frames * 1e6f / (now - fps_t0);
            frames = 0;
            fps_t0 = now;
        }
    }
}

// Face detection: ESPDet-Pico (robust boxes) -> MNP (refined box + 5 landmarks).
// If MNP rejects the box, landmarks are placed from average face proportions.
static bool detect_face(human_face_detect::ESPDet *det, human_face_detect::MNP *mnp, const dl::image::img_t &img,
                        dl::detect::result_t &out)
{
    auto &boxes = det->run(img);
    const dl::detect::result_t *best = nullptr;
    int best_area = 0;
    for (auto &r : boxes) {
        const int area = (r.box[2] - r.box[0]) * (r.box[3] - r.box[1]);
        if (r.score >= 0.4f && area > best_area) { best_area = area; best = &r; }
    }
    if (!best) return false;
    out = *best;
    const float x = out.box[0], y = out.box[1], w = out.box[2] - out.box[0], h = out.box[3] - out.box[1];

    std::list<dl::detect::result_t> cand{out};
    auto &refined = mnp->run(img, cand);
    if (!refined.empty() && refined.front().keypoint.size() >= 10) {
        const auto &m = refined.front();
        // accept the landmarks only if the eyes lie inside the detector's box
        const bool inside = m.keypoint[0] > x && m.keypoint[0] < x + w && m.keypoint[6] > x && m.keypoint[6] < x + w &&
                            m.keypoint[1] > y && m.keypoint[1] < y + h;
        if (inside) {
            out.keypoint = m.keypoint;
            return true;
        }
    }
    // proportional fallback: left eye, left mouth, nose, right eye, right mouth
    out.keypoint = {int(x + 0.32f * w), int(y + 0.42f * h), int(x + 0.36f * w), int(y + 0.78f * h),
                    int(x + 0.50f * w), int(y + 0.60f * h), int(x + 0.68f * w), int(y + 0.42f * h),
                    int(x + 0.64f * w), int(y + 0.78f * h)};
    return true;
}

static void detect_task(void *)
{
    auto *det = new human_face_detect::ESPDet("espdet_pico_224_224_face.espdl", 0.4f, 0.7f);
    auto *mnp = new human_face_detect::MNP("human_face_detect_mnp_s8_v1.espdl", 0.3f, 0.5f);
    int count = 0;
    int64_t t0 = esp_timer_get_time();
    float prev_c[2] = {0, 0};
    int64_t prev_t = 0;
    int misses = 0;
    dl::detect::result_t found;

    for (;;) {
        xSemaphoreTake(det_sem, portMAX_DELAY);
        const int64_t start = esp_timer_get_time();
        // Auto-orientation: the detector only finds upright faces. After a
        // while without one, try the frame rotated 180 degrees; if that finds
        // a face, rotate the sensor.
        const bool try_flipped = misses >= 10 && misses % 5 == 0;
        if (try_flipped) {
            // 180 degrees on YUYV: reverse the order of the pixel pairs and swap
            // the two luminance samples within each (chroma stays with its pair)
            uint32_t *p32 = (uint32_t *)det_frame;
            auto swapY = [](uint32_t v) {
                return (v & 0x00FF00FFu) | ((v >> 16) & 0x0000FF00u) | ((v & 0x0000FF00u) << 16);
            };
            for (int i = 0, j = W * H / 2 - 1; i < j; i++, j--) {
                const uint32_t a = p32[i], b = p32[j];
                p32[i] = swapY(b);
                p32[j] = swapY(a);
            }
        }
        dl::image::img_t img = {det_frame, W, H, dl::image::DL_IMAGE_PIX_TYPE_YUYV};
        const dl::detect::result_t *best = detect_face(det, mnp, img, found) ? &found : nullptr;
        const int64_t now = esp_timer_get_time();
        stats.det_ms = 0.9f * stats.det_ms + 0.1f * (now - start) / 1000.0f;

        if (try_flipped) {
            if (best) {
                ESP_LOGI(TAG, "upside-down face found (score %.2f), rotating image", best->score);
                cfg.flip_request = true;
                misses = 0;
            } else {
                misses++;
            }
            det_busy = false;
            continue;
        }
        misses = best ? 0 : misses + 1;
        stats.det_hit = 0.95f * stats.det_hit + (best ? 0.05f : 0);

        xSemaphoreTake(face_mtx, portMAX_DELAY);
        if (best) {
            float box[4], kp[10];
            for (int i = 0; i < 4; i++) box[i] = best->box[i];
            for (int i = 0; i < 10; i++) kp[i] = best->keypoint[i];
            const float bw = box[2] - box[0];
            const float c[2] = {(box[0] + box[2]) / 2, (box[1] + box[3]) / 2};

            // motion: speed of the (lightly smoothed) face centre in face-widths per
            // second. The raw detector box jitters by a few pixels, so the
            // threshold sits well above that.
            if (prev_t && now - prev_t < 800000) {
                const float sc[2] = {0.5f * c[0] + 0.5f * prev_c[0], 0.5f * c[1] + 0.5f * prev_c[1]};
                const float v = std::hypot(sc[0] - prev_c[0], sc[1] - prev_c[1]) / bw / ((now - prev_t) / 1e6f);
                if (v > FACE_MOTION) g_face.motion_until_us = now + 400000;
                prev_c[0] = sc[0];
                prev_c[1] = sc[1];
            } else {
                prev_c[0] = c[0];
                prev_c[1] = c[1];
            }
            prev_t = now;

            const bool fresh = !g_face.valid || now - g_face.t_us > FACE_HOLD_US ||
                               std::fabs(c[0] - (g_face.box[0] + g_face.box[2]) / 2) > bw * 0.5f;
            const float a = fresh ? 1.0f : 0.3f;  // exponential smoothing of detector jitter
            for (int i = 0; i < 4; i++) g_face.box[i] += a * (box[i] - g_face.box[i]);
            for (int i = 0; i < 10; i++) g_face.kp[i] += a * (kp[i] - g_face.kp[i]);
            g_face.valid = true;
            g_face.t_us = now;

            // MNP landmark order: left eye, left mouth corner, nose, right eye, right mouth corner
            g_face.eye_l[0] = g_face.kp[0];
            g_face.eye_l[1] = g_face.kp[1];
            g_face.eye_r[0] = g_face.kp[6];
            g_face.eye_r[1] = g_face.kp[7];
            g_face.mouth[0] = (g_face.kp[2] + g_face.kp[8]) / 2;
            g_face.mouth[1] = (g_face.kp[3] + g_face.kp[9]) / 2;
        } else {
            prev_t = 0;
        }
        xSemaphoreGive(face_mtx);

        if (++count == 20) {
            stats.det_fps = count * 1e6f / (now - t0);
            count = 0;
            t0 = now;
        }
        det_busy = false;
    }
}

static const char *state_name(rppg::State s)
{
    switch (s) {
    case rppg::LOCKED: return "locked";
    case rppg::ACQUIRING: return "acquiring";
    default: return "no_signal";
    }
}

static void hr_task(void *)
{
    auto *tmp = (rppg::Sample *)heap_caps_malloc(sizeof(rppg::Sample) * rppg::kCap, MALLOC_CAP_SPIRAM);
    auto *rtmp = (resp::MotionSample *)heap_caps_malloc(sizeof(resp::MotionSample) * resp::kCap, MALLOC_CAP_SPIRAM);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        xTaskDelayUntil(&last, pdMS_TO_TICKS(1000));
        xSemaphoreTake(buf_mtx, portMAX_DELAY);
        const int n = g_buf->copy(tmp);
        const int rn = g_rbuf->copy(rtmp);
        xSemaphoreGive(buf_mtx);
        g_est->set_mode(cfg.mono ? rppg::Mode::MONO : rppg::Mode::RGB);
        const int64_t t0 = esp_timer_get_time();
        // "now" must be the wall clock, not the newest sample: otherwise a
        // stalled camera looks like a perfectly fresh window forever
        const uint32_t now_ms = esp_timer_get_time() / 1000;
        const rppg::Result r = n ? g_est->update(tmp, n, now_ms) : rppg::Result{};
        const float ms = (esp_timer_get_time() - t0) / 1000.0f;
        xSemaphoreTake(stats.mtx, portMAX_DELAY);
        stats.res = r;
        xSemaphoreGive(stats.mtx);
        bool face;
        xSemaphoreTake(face_mtx, portMAX_DELAY);
        face = g_face.valid && esp_timer_get_time() - g_face.t_us < FACE_HOLD_US;
        xSemaphoreGive(face_mtx);
        out_printf("HR bpm=%.1f raw=%.1f snr=%.1f q=%.2f stab=%.2f coh=%.2f state=%s motion=%.2f "
                   "rois=%d face=%d det_hit=%.2f fps=%.1f det_fps=%.1f det_ms=%.0f est_ms=%.0f mode=%s\n",
                   r.bpm, r.bpm_raw, r.snr_db, r.quality, r.stability, r.coherence, state_name(r.state),
                   r.motion, r.rois_used, face, stats.det_hit, stats.cam_fps, stats.det_fps, stats.det_ms, ms,
                   cfg.mono ? "mono" : "rgb");

        g_rest->set_band(cfg.infant ? resp::Band::INFANT : resp::Band::ADULT);
        const int64_t t1 = esp_timer_get_time();
        const resp::Result rr = rn ? g_rest->update(rtmp, rn, now_ms) : resp::Result{};
        const float rms = (esp_timer_get_time() - t1) / 1000.0f;
        xSemaphoreTake(stats.mtx, portMAX_DELAY);
        stats.rr = rr;
        xSemaphoreGive(stats.mtx);
        char sel[40] = "";
        for (int i = 0, o = 0; i < 8 && rr.sel[i] >= 0 && o < (int)sizeof(sel) - 4; i++)
            o += snprintf(sel + o, sizeof(sel) - o, o ? ",%d" : "%d", rr.sel[i]);
        out_printf("RR br=%.1f raw=%.1f snr=%.1f q=%.2f stab=%.2f agree=%.2f state=%s motion=%.2f ch=%d best=%d "
                   "sel=%s band=%s est_ms=%.0f\n",
                   rr.brpm, rr.raw, rr.snr_db, rr.quality, rr.stability, rr.agreement, state_name(rr.state), rr.motion,
                   rr.channels, rr.best, sel[0] ? sel : "-", cfg.infant ? "infant" : "adult", rms);
    }
}

static void stream_task(void *)
{
    for (;;) {
        xSemaphoreTake(str_sem, portMAX_DELAY);
        dl::image::img_t img = {str_frame, W, H, dl::image::DL_IMAGE_PIX_TYPE_YUYV};
        dl::image::jpeg_img_t jpg = dl::image::sw_encode_jpeg(img, cfg.quality);
        if (jpg.data) {
            uint32_t hdr[2] = {uint32_t(jpg.data_len), str_t_ms};
            xSemaphoreTake(out_mtx, portMAX_DELAY);
            if (out_raw("HCF2", 4) && out_raw(hdr, sizeof(hdr)) && out_raw(str_meta, sizeof(str_meta)))
                out_raw(jpg.data, jpg.data_len);
            xSemaphoreGive(out_mtx);
            heap_caps_free(jpg.data);
        }
        str_busy = false;
    }
}

static void led_task(void *)
{
    int64_t last_beat = 0;
    for (;;) {
        xSemaphoreTake(stats.mtx, portMAX_DELAY);
        const rppg::Result r = stats.res;
        xSemaphoreGive(stats.mtx);
        const int64_t now = esp_timer_get_time();
        if (r.state == rppg::LOCKED && r.bpm > 30) {
            const int64_t period = int64_t(60e6f / r.bpm);
            if (now - last_beat >= period) last_beat = now;
            gpio_set_level(LED_PIN, now - last_beat < 70000 ? 0 : 1);
        } else {
            gpio_set_level(LED_PIN, 1);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void handle_command(char *line)
{
    const char c = line[0];
    const int v = atoi(line + 1);
    sensor_t *s = esp_camera_sensor_get();
    auto reset_history = [] {
        xSemaphoreTake(buf_mtx, portMAX_DELAY);
        g_buf->clear();
        xSemaphoreGive(buf_mtx);
    };
    switch (c) {
    case 's': cfg.stream = true; out_printf("# stream on\n"); break;
    case 'x': cfg.stream = false; out_printf("# stream off\n"); break;
    case 'v': cfg.samples = true; out_printf("# samples on\n"); break;
    case 'w': cfg.samples = false; out_printf("# samples off\n"); break;
    case 'a': cfg.relock = true; break;
    case 'e':
        s->set_exposure_ctrl(s, 0);
        s->set_aec_value(s, std::clamp(v, 0, 1200));
        reset_history();
        out_printf("# exposure %d\n", v);
        break;
    case 'g':
        s->set_gain_ctrl(s, 0);
        s->set_agc_gain(s, std::clamp(v, 0, 30));
        reset_history();
        out_printf("# gain %d\n", v);
        break;
    case 'r': cfg.flip_request = true; break;
    case 'q': cfg.quality = std::clamp(v, 1, 100); out_printf("# quality %d\n", cfg.quality); break;
    case 'm':
        cfg.mono = v != 0;
        out_printf("# mode %s\n", cfg.mono ? "mono" : "rgb");
        break;
    case 'b':
        cfg.infant = v != 0;
        out_printf("# breathing band %s\n", cfg.infant ? "infant (6-78/min)" : "adult (6-45/min)");
        break;
    case 'i':
        // exposure and gain matter when comparing cameras or lenses: a fair
        // comparison needs both sensors to have settled on similar light
        out_printf("# heart_cam idf PID=0x%04x stream=%d samples=%d mode=%s cam_fps=%.1f det_fps=%.1f "
                   "exposure=%d gain=%d bright=%d psram_free=%u int_free=%u\n",
                   s->id.PID, cfg.stream, cfg.samples, cfg.mono ? "mono" : "rgb", stats.cam_fps, stats.det_fps,
                   s->status.aec_value, s->status.agc_gain, stats.brightness,
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
        break;
    default: out_printf("# unknown command '%s'\n", line);
    }
}

static void cmd_task(void *)
{
    char line[40];
    int len = 0;
    for (;;) {
        uint8_t ch;
        if (usb_serial_jtag_read_bytes(&ch, 1, portMAX_DELAY) <= 0) continue;
        if (ch == '\n' || ch == '\r') {
            line[len] = 0;
            if (len) handle_command(line);
            len = 0;
        } else if (len < (int)sizeof(line) - 1) {
            line[len++] = ch;
        }
    }
}

extern "C" void app_main(void)
{
    out_mtx = xSemaphoreCreateMutex();
    stats.mtx = xSemaphoreCreateMutex();
    face_mtx = xSemaphoreCreateMutex();
    buf_mtx = xSemaphoreCreateMutex();
    det_sem = xSemaphoreCreateBinary();
    str_sem = xSemaphoreCreateBinary();

    usb_serial_jtag_driver_config_t ucfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    ucfg.tx_buffer_size = 16384;
    ucfg.rx_buffer_size = 512;
    usb_serial_jtag_driver_install(&ucfg);
    esp_log_set_vprintf(log_vprintf);

    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 1);

    g_buf = new rppg::Buffer();
    g_est = new rppg::Estimator();
    g_rbuf = new resp::Buffer();
    g_rest = new resp::Estimator();
    g_tiles = new resp::TileMotion();
    det_frame = (uint8_t *)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);
    str_frame = (uint8_t *)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM);

    ESP_LOGI(TAG, "booting");
    if (!camera_init()) {
        for (;;) {
            gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    auto_then_lock(2500);

    xTaskCreatePinnedToCore(cam_task, "cam", 6144, nullptr, 6, nullptr, 1);
    xTaskCreatePinnedToCore(detect_task, "detect", 16384, nullptr, 4, nullptr, 0);
    xTaskCreatePinnedToCore(hr_task, "hr", 24576, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(stream_task, "stream", 8192, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(led_task, "led", 2048, nullptr, 2, nullptr, 1);
    xTaskCreatePinnedToCore(cmd_task, "cmd", 4096, nullptr, 5, nullptr, 0);
    ESP_LOGI(TAG, "ready");
}
