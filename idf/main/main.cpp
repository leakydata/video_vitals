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
//     E0/E1/E2 preview Eulerian magnification off / motion (breathing) / colour (pulse)
//     z0/z1 sensor-window zoom off / follow the face
//     b0/b1 breathing band adult (6-45/min) / infant (6-78/min)
//     r   rotate image 180 degrees (also automatic when a face is upside down)
//     i   info
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <new>
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
#include "evm.hpp"
#include "motion.hpp"
#include "resp.hpp"
#include "rppg.hpp"

static const char *TAG = "heart_cam";

[[noreturn]] static void fatal(const char *why);   // blink fast and say why, rather than run on

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
static evm::Magnifier *g_evm;   // preview magnification, on the device

static struct {
    bool stream = false, samples = false, mono = false, infant = false;
    volatile bool zoom = false;      // follow the face with a sensor window
    volatile int magnify = 0;        // 0 off, 1 breathing band, 2 pulse band
    volatile bool zoom_reset = false;
    int quality = 80;
    volatile bool relock = false;
    volatile bool flip_request = false;  // rotate the sensor image 180 degrees
    // Sensor settings asked for from the command task. The camera task owns the
    // sensor -- it is the one that windows it, flips it and reads from it -- so
    // a request is left here and applied between frames rather than written to
    // the sensor's registers from underneath a capture that is already running.
    // A region to watch when there is no face to anchor one to. The whole
    // face-anchored arrangement assumes a face can be found, and for the case
    // this is really for -- an infant lying in a cot, seen from above in
    // infrared -- it cannot: the detector found nothing in 201 frames of real
    // cot footage. Pointing the camera once and saying "watch there" is both
    // what a parent would do anyway and, measured on that footage, enough to
    // recover the breathing rate exactly.
    volatile int16_t watch_x = 0, watch_y = 0, watch_w = 0, watch_h = 0;
    volatile int exposure_request = -1;  // AEC value, -1 = nothing pending
    volatile int gain_request = -1;      // AGC gain, -1 = nothing pending
    volatile bool evm_reset_request = false;
    bool flipped = false;
} cfg;

// Each long-running task stamps its heartbeat; the LED task (which does almost
// nothing else) restarts the device if one stops. An unattended monitor that has
// quietly stopped measuring is worse than one that reboots and carries on.
enum Beat { BEAT_CAM, BEAT_DET, BEAT_HR, BEAT_N };
static std::atomic<int64_t> beats[BEAT_N];
static constexpr int64_t BEAT_TIMEOUT_US = 20000000;
static inline void beat(Beat b) { beats[b].store(esp_timer_get_time(), std::memory_order_relaxed); }

static struct {
    float cam_fps = 0, det_fps = 0, det_ms = 0, det_hit = 0;
    int brightness = 0;  // mean luminance of the frame centre
    float roi_level = 0; // mean of the brightest channel of the skin ROIs
    float evm_ms = 0;    // time spent magnifying the preview
    float jpeg_ms = 0, tx_ms = 0, jpeg_kb = 0;  // preview encode, transfer, size
    float roi_clip = 0;  // fraction of ROI pixels at or near saturation
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

// ---------------------------------------------------------------- zoom
// The sensor can read out a window of its array instead of the whole thing.
// Cropping to the subject puts far more sensor pixels on the skin at the same
// output size and frame rate — an electronic telephoto — which is what the
// heart-rate signal is short of. Geometry from the driver's 4:3 table:
// array 2048x1536, end margins +32/+12, offset 16,6, line length 2300.
static constexpr int ARR_W = 2048, ARR_H = 1536, TOTAL_X = 2300;
static struct { int x = 0, y = 0, w = ARR_W, h = ARR_H; } win;  // current sensor window

static void apply_window(int cx, int cy, int cw)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s->set_res_raw) return;
    cw = std::clamp(cw, 4 * W, ARR_W);      // never zoom past 4x, never beyond the array
    int ch = cw * H / W;
    if (ch > ARR_H) { ch = ARR_H; cw = ch * W / H; }
    cx = std::clamp(cx - cw / 2, 0, ARR_W - cw);
    cy = std::clamp(cy - ch / 2, 0, ARR_H - ch);
    const bool binning = cw >= 2 * W && ch >= 2 * H;
    const int total_y = binning ? (ch + 28) / 2 + 1 : ch + 28;
    if (s->set_res_raw(s, cx, cy, cx + cw - 1 + 32, cy + ch - 1 + 12, 16, 6, TOTAL_X, total_y, W, H, true,
                       binning) != 0) {
        ESP_LOGW(TAG, "window %dx%d at %d,%d rejected", cw, ch, cx, cy);
        return;
    }
    win = {cx, cy, cw, ch};
    ESP_LOGI(TAG, "sensor window %dx%d at %d,%d (%.1fx zoom)", cw, ch, cx, cy, float(ARR_W) / cw);
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

// Mean colour of a ROI from a YUYV frame.
//
// The landmarks already place the box on skin, so rather than testing pixels
// against a fixed "skin colour" range — which depends on the white balance and
// is biased across skin tones — the box learns its own colour: take the mean
// chroma of the well-exposed pixels, then keep those close to it. Hair,
// glasses, shadow and background are outliers and drop out; skin does not,
// whatever colour the light makes it.
static rppg::RoiSample roi_mean(const uint8_t *px, const Rect &r, bool any_colour)
{
    rppg::RoiSample out{0, 0, 0, 0};
    if (r.w < 4 || r.h < 4) return out;

    // pass 1: mean chroma of pixels that are neither clipped nor black
    int32_t scb = 0, scr = 0;
    uint32_t an = 0;
    for (int y = r.y; y < r.y + r.h; y++) {
        const uint8_t *row = px + y * W * 2;
        for (int x = r.x; x < r.x + r.w; x++) {
            const int Y = row[x * 2];
            if (Y < 25 || Y > 250) continue;
            const int cbase = (x & ~1) * 2;
            scb += row[cbase + 1];
            scr += row[cbase + 3];
            an++;
        }
    }
    if (an < 16) return out;
    const int mcb = scb / int(an), mcr = scr / int(an);

    // pass 2: average the pixels whose colour matches the box's own
    constexpr int TOL = 14;  // chroma distance still counted as the same surface
    uint32_t sr = 0, sg = 0, sb = 0, n = 0, ar = 0, ag = 0, ab = 0, un = 0;
    for (int y = r.y; y < r.y + r.h; y++) {
        const uint8_t *row = px + y * W * 2;
        for (int x = r.x; x < r.x + r.w; x++) {
            const int Y = row[x * 2];
            if (Y < 25 || Y > 250) continue;
            const int cbase = (x & ~1) * 2;
            const int Cb = row[cbase + 1], Cr = row[cbase + 3];
            const int cd = Cr - 128, ce = Cb - 128;
            const int R = std::clamp(Y + ((91881 * cd) >> 16), 0, 255);
            const int G = std::clamp(Y - ((22554 * ce + 46802 * cd) >> 16), 0, 255);
            const int B = std::clamp(Y + ((116130 * ce) >> 16), 0, 255);
            if (R > 250 || G > 250 || B > 250) continue;  // clipped / specular
            ar += R; ag += G; ab += B; un++;   // pixels actually in the fallback sums
            if (std::abs(Cb - mcb) > TOL || std::abs(Cr - mcr) > TOL) continue;
            sr += R; sg += G; sb += B; n++;
        }
    }
    if (!any_colour && n >= un / 3 && n >= 16) {
        out = {float(sr) / n, float(sg) / n, float(sb) / n, uint16_t(std::min<uint32_t>(n, 65535))};
    } else if (un >= 16) {
        // mono/IR has no usable chroma, and a box whose colour is not uniform is
        // better measured whole than not at all. Divide by the pixels that are
        // actually in these sums, not by the larger luminance-valid count:
        // otherwise a changing clipped fraction looks like a brightness change.
        out = {float(ar) / un, float(ag) / un, float(ab) / un, uint16_t(std::min<uint32_t>(un, 65535))};
    }
    return out;
}

// ---------------------------------------------------------------- tasks
// A window change is a discontinuity, not a reason to forget everything: the
// breathing estimator needs 30 continuous seconds, so clearing its history on
// every re-aim left it with nothing at all. Mark the affected frames as
// movement instead — the masks already handle exactly this — and rebuild the
// tile references.
static int64_t break_until = 0;

static void break_history(int64_t now)
{
    break_until = now + 1200000;  // ~1.2 s of samples marked as movement
    g_tiles->reset();
}

// Exposure and white balance changes alter the measured colours outright, so
// those do still discard the history.
static void clear_history()
{
    xSemaphoreTake(buf_mtx, portMAX_DELAY);
    g_buf->clear();
    g_rbuf->clear();
    xSemaphoreGive(buf_mtx);
    g_tiles->reset();
    g_est->reset();
    g_rest->reset();
    break_history(esp_timer_get_time());  // and mark the frames either side as movement
}

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
    int64_t last_relock = 0;
    int capture_fails = 0;

    for (;;) {
        if (cfg.relock) {
            cfg.relock = false;
            auto_then_lock(2000);
        }
        if (cfg.zoom_reset) {
            cfg.zoom_reset = false;
            apply_window(ARR_W / 2, ARR_H / 2, ARR_W);
            break_history(esp_timer_get_time());
        }
        if (cfg.exposure_request >= 0 || cfg.gain_request >= 0) {
            sensor_t *sen = esp_camera_sensor_get();
            if (cfg.exposure_request >= 0) {
                sen->set_exposure_ctrl(sen, 0);
                sen->set_aec_value(sen, cfg.exposure_request);
                cfg.exposure_request = -1;
            }
            if (cfg.gain_request >= 0) {
                sen->set_gain_ctrl(sen, 0);
                sen->set_agc_gain(sen, cfg.gain_request);
                cfg.gain_request = -1;
            }
            // the brightness step is not breathing, and not a pulse
            break_history(esp_timer_get_time());
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
            clear_history();
        }
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            // Capture can fail transiently; if it keeps failing the camera needs
            // restarting, and if that does not work there is nothing to measure.
            if (++capture_fails % 20 == 0) {
                ESP_LOGW(TAG, "%d capture failures: restarting the camera", capture_fails);
                esp_camera_deinit();
                vTaskDelay(pdMS_TO_TICKS(200));
                if (!camera_init()) ESP_LOGE(TAG, "camera restart failed");
                else {
                    apply_flip();
                    auto_then_lock(1500);
                }
            }
            if (capture_fails > 200) fatal("camera will not produce frames");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        capture_fails = 0;
        beat(BEAT_CAM);
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
        s.motion = now < f.motion_until_us || now < break_until;
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
            float level = 0;
            int levels = 0;
            for (int i = 0; i < nroi; i++) {
                rppg::RoiSample m = roi_mean(fb->buf, held[i], cfg.mono);
                if (m.n) {
                    level += std::max(m.r, std::max(m.g, m.b));
                    levels++;
                }
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
            // No usable ROI at all (face lost, or the image clipped/black) must
            // still be recoverable: fall back to the frame's own brightness,
            // otherwise a large lighting change strands the device permanently.
            if (!levels && (stats.brightness > 215 || stats.brightness < 25) &&
                now - last_relock > 30000000 && !cfg.relock) {
                ESP_LOGI(TAG, "frame brightness %d with no usable ROI: re-running auto exposure",
                         stats.brightness);
                cfg.relock = true;
                last_relock = now;
            }
            if (levels) {
                const float lvl = level / levels;
                stats.roi_level = stats.roi_level > 0 ? 0.98f * stats.roi_level + 0.02f * lvl : lvl;
                // Exposure is frozen at boot so that automatic adjustments cannot
                // imitate a pulse. If the light has since changed enough to clip
                // the skin (or leave it in the dark), that costs far more than a
                // re-exposure does, so redo it - at most every 30 s.
                const bool bad = stats.roi_level > 205 || stats.roi_level < 45;
                if (bad && now - last_relock > 30000000 && !cfg.relock) {
                    ESP_LOGI(TAG, "skin level %.0f: re-running auto exposure", stats.roi_level);
                    cfg.relock = true;
                    last_relock = now;
                    stats.roi_level = 0;
                }
            }
        } else {
            have_held = false;
            for (auto &p : prev) p.n = 0;
        }

        // Auto-zoom: crop the sensor to the subject once a face is settled, and
        // widen again when it is lost. Rate-limited, and only for real changes,
        // because every window change discards the measurement history.
        if (cfg.zoom) {
            static int64_t last_zoom = 0;
            // Only ever aim from a *fresh* detection: after the window moves, the
            // last known face position refers to the old view, and aiming from it
            // walks the crop off the subject.
            const bool face_fresh = f.valid && now - f.t_us < 700000;
            // A face against the edge of the frame is being cut off, and the
            // parts that are missing are the ones the measurement needs. That is
            // worth a second of lost signal straight away rather than waiting out
            // the ordinary rate limit.
            const float EDGE = 0.06f;
            const bool clipped = face_fresh &&
                                 (f.box[0] < EDGE * W || f.box[2] > (1 - EDGE) * W ||
                                  f.box[1] < EDGE * H || f.box[3] > (1 - EDGE) * H);
            // ... but only a few times in a row. If the subject is against the
            // edge of the sensor itself the crop cannot move any further, the
            // face stays clipped whatever we do, and re-aiming every three
            // seconds would throw away a second of signal each time for nothing.
            static int urgent_run = 0;
            const int64_t wait = clipped && urgent_run < 3 ? 3000000 : 20000000;
            if (face_fresh && now - last_zoom > wait) {  // re-aiming costs ~1 s of signal
                const float fw = f.box[2] - f.box[0], fh = f.box[3] - f.box[1];
                const float fcx = (f.box[0] + f.box[2]) / 2;
                // Frame what has to be in shot rather than a fixed multiple of
                // the face: the detector's box stops at the hairline, so leave
                // HEAD above it, and the chest box (0.15 fh below the face,
                // 1.2 fh tall) needs CHEST below. Sized from the face *height*,
                // which barely changes when the head turns, unlike its width.
                // The chest box may be trimmed (the estimator accepts 60% of it)
                // but the head may not: nothing tolerates half a face.
                const float HEAD = 0.40f, CHEST = 1.05f;
                const float fcy = (f.box[1] + f.box[3]) / 2 + (CHEST - HEAD) / 2 * fh;
                // Face box -> sensor coordinates through the current window. The
                // sensor may be mirroring and flipping what it reads out (the
                // auto-rotation does exactly that), and the window is in sensor
                // coordinates, so undo those first or the crop lands on the
                // subject's mirror image.
                sensor_t *sen = esp_camera_sensor_get();
                const float ox = fcx * win.w / W, oy = fcy * win.h / H;
                const int sx = win.x + int(sen->status.hmirror ? win.w - 1 - ox : ox);
                const int sy = win.y + int(sen->status.vflip ? win.h - 1 - oy : oy);
                // The window keeps the frame's aspect, so the height that must
                // fit sets the width; widen a little more when the face was
                // being clipped, since the crop was evidently too tight.
                const float span = (HEAD + 1.0f + CHEST) * fh * (clipped ? 1.15f : 1.0f);
                const float want = std::max(span * W / H, 2.6f * fw);
                // clamped, because the comparison below must be against the
                // window we would actually get: a centred subject could
                // otherwise never satisfy the size condition
                const int want_w = std::clamp(int(want * win.w / W), 4 * W, ARR_W);
                const bool moved = clipped ||
                                   std::abs(sx - (win.x + win.w / 2)) > win.w / 6 ||
                                   std::abs(sy - (win.y + win.h / 2)) > win.h / 6 ||
                                   std::abs(want_w - win.w) > win.w / 5;
                if (moved) {
                    urgent_run = clipped ? urgent_run + 1 : 0;
                    apply_window(sx, sy, want_w);
                    break_history(now);
                    last_zoom = now;
                    // the face we have was seen through the old window
                    xSemaphoreTake(face_mtx, portMAX_DELAY);
                    g_face.valid = false;
                    xSemaphoreGive(face_mtx);
                    for (int i = 0; i < 3; i++) {  // drop frames captured with the old window
                        camera_fb_t *old = esp_camera_fb_get();
                        if (old) esp_camera_fb_return(old);
                    }
                }
                if (!clipped) urgent_run = 0;
            } else if (!face_ok && win.w < ARR_W && now - last_zoom > 8000000) {
                urgent_run = 0;
                ESP_LOGI(TAG, "face lost: widening the sensor window");
                apply_window(ARR_W / 2, ARR_H / 2, ARR_W);
                break_history(now);
                last_zoom = now;
            }
        }

        resp::MotionSample ms;
        resp::Box boxes[resp::kBoxes] = {};
        if (face_ok) {
            boxes[0] = chest_box(f);
            // head box: the face box itself, which breathing also moves slightly
            const int16_t hx = int16_t(std::lround(f.box[0])), hy = int16_t(std::lround(f.box[1]));
            boxes[1] = {hx, hy, int16_t(std::lround(f.box[2]) - hx), int16_t(std::lround(f.box[3]) - hy)};
        } else if (cfg.watch_w > 16 && cfg.watch_h > 16) {
            // no face: watch where we were told to
            boxes[0] = {cfg.watch_x, cfg.watch_y, cfg.watch_w, cfg.watch_h};
        }
        const resp::Box &chest = boxes[0];
        g_tiles->process_yuyv(fb->buf, W, H, t_ms, now < f.motion_until_us || now < break_until, boxes, ms);

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
            // [Y0,U,Y1,V] -> [Y1,U,Y0,V]: keep the chroma bytes, swap the luma
            auto swapY = [](uint32_t v) {
                return (v & 0xFF00FF00u) | ((v >> 16) & 0x000000FFu) | ((v & 0x000000FFu) << 16);
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
        beat(BEAT_DET);
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
        beat(BEAT_HR);
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
                   "rois=%d face=%d det_hit=%.2f skin_lvl=%.0f fps=%.1f det_fps=%.1f det_ms=%.0f est_ms=%.0f evm_ms=%.1f "
                   "jpeg=%.0fms/%.1fkB tx=%.0fms mode=%s\n",
                   r.bpm, r.bpm_raw, r.snr_db, r.quality, r.stability, r.coherence, state_name(r.state),
                   r.motion, r.rois_used, face, stats.det_hit, stats.roi_level, stats.cam_fps, stats.det_fps,
                   stats.det_ms, ms, stats.evm_ms, stats.jpeg_ms, stats.jpeg_kb, stats.tx_ms,
                   cfg.mono ? "mono" : "rgb");

        g_rest->set_band(cfg.infant ? resp::Band::INFANT : resp::Band::ADULT);
        const int64_t t1 = esp_timer_get_time();
        resp::Result rr = rn ? g_rest->update(rtmp, rn, now_ms) : resp::Result{};
        // Whether anything is breathing cannot be answered from an image too
        // dark to measure it in. Below this the tile tracker's output is not
        // merely noisy but wrong: measured at 24 px of movement in an empty,
        // dark room, which passes every test for breathing that the movement
        // itself can be put to. The honest answer there is that it does not
        // know, and for a monitor that must be distinguishable from "not
        // breathing".
        const bool can_tell = stats.brightness >= 25;
        if (!can_tell) rr.present = false;
        const float rms = (esp_timer_get_time() - t1) / 1000.0f;
        xSemaphoreTake(stats.mtx, portMAX_DELAY);
        stats.rr = rr;
        xSemaphoreGive(stats.mtx);
        char sel[40] = "";
        for (int i = 0, o = 0; i < 8 && rr.sel[i] >= 0 && o < (int)sizeof(sel) - 4; i++)
            o += snprintf(sel + o, sizeof(sel) - o, o ? ",%d" : "%d", rr.sel[i]);
        out_printf("RR br=%.1f raw=%.1f snr=%.1f q=%.2f stab=%.2f agree=%.2f state=%s motion=%.2f ch=%d best=%d "
                   "reg=%d sel=%s drop=%d/%d/%d/%d breathing=%s x%.1f quiet=%.0fs bright=%d band=%s est_ms=%.0f\n",
                   rr.brpm, rr.raw, rr.snr_db, rr.quality, rr.stability, rr.agreement, state_name(rr.state), rr.motion,
                   rr.channels, rr.best, rr.regions, sel[0] ? sel : "-", rr.n_nodata, rr.n_still, rr.n_masked, rr.n_weak,
                   can_tell ? (rr.present ? "seen" : "none") : "unknown", rr.presence, rr.quiet_s,
                   stats.brightness, cfg.infant ? "infant" : "adult", rms);
    }
}

static void stream_task(void *)
{
    for (;;) {
        xSemaphoreTake(str_sem, portMAX_DELAY);
        // Eulerian magnification of the preview, on the device. The vitals are
        // measured from the untouched frame; this only changes what is watched.
        if (cfg.evm_reset_request) {
            cfg.evm_reset_request = false;
            g_evm->reset();   // between frames, never from under one being processed
        }
        if (cfg.magnify) {
            const int64_t t0 = esp_timer_get_time();
            g_evm->set_band(cfg.magnify == 1 ? 0.1f : 0.8f, cfg.magnify == 1 ? 0.8f : 2.5f);
            g_evm->set_alpha(cfg.magnify == 1 ? 12.0f : 30.0f);
            static uint32_t prev_t = 0;
            const float dt = prev_t ? std::clamp(rppg::ms_diff(str_t_ms, prev_t) / 1000.0f, 0.01f, 0.5f) : 0.07f;
            prev_t = str_t_ms;
            g_evm->process_yuyv(str_frame, W, H, dt);
            stats.evm_ms = 0.9f * stats.evm_ms + 0.1f * (esp_timer_get_time() - t0) / 1000.0f;
        }
        dl::image::img_t img = {str_frame, W, H, dl::image::DL_IMAGE_PIX_TYPE_YUYV};
        const int64_t enc0 = esp_timer_get_time();
        dl::image::jpeg_img_t jpg = dl::image::sw_encode_jpeg(img, cfg.quality);
        stats.jpeg_ms = 0.9f * stats.jpeg_ms + 0.1f * (esp_timer_get_time() - enc0) / 1000.0f;
        if (jpg.data) {
            stats.jpeg_kb = 0.9f * stats.jpeg_kb + 0.1f * jpg.data_len / 1024.0f;
            const int64_t tx0 = esp_timer_get_time();
            uint32_t hdr[2] = {uint32_t(jpg.data_len), str_t_ms};
            xSemaphoreTake(out_mtx, portMAX_DELAY);
            if (out_raw("HCF2", 4) && out_raw(hdr, sizeof(hdr)) && out_raw(str_meta, sizeof(str_meta)))
                out_raw(jpg.data, jpg.data_len);
            xSemaphoreGive(out_mtx);
            stats.tx_ms = 0.9f * stats.tx_ms + 0.1f * (esp_timer_get_time() - tx0) / 1000.0f;
            heap_caps_free(jpg.data);
        }
        str_busy = false;
    }
}

static void led_task(void *)
{
    int64_t last_beat = 0;
    const char *beat_name[BEAT_N] = {"camera", "detector", "estimator"};
    for (auto &b : beats) b.store(esp_timer_get_time(), std::memory_order_relaxed);
    for (;;) {
        xSemaphoreTake(stats.mtx, portMAX_DELAY);
        const rppg::Result r = stats.res;
        xSemaphoreGive(stats.mtx);

        // watchdog: a task that has stopped stamping is not coming back
        for (int i = 0; i < BEAT_N; i++) {
            if (esp_timer_get_time() - beats[i].load(std::memory_order_relaxed) > BEAT_TIMEOUT_US) {
                ESP_LOGE(TAG, "%s task stopped responding: restarting", beat_name[i]);
                vTaskDelay(pdMS_TO_TICKS(200));
                esp_restart();
            }
        }
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
    switch (c) {
    case 's': cfg.stream = true; out_printf("# stream on\n"); break;
    case 'x': cfg.stream = false; out_printf("# stream off\n"); break;
    case 'v': cfg.samples = true; out_printf("# samples on\n"); break;
    case 'w': cfg.samples = false; out_printf("# samples off\n"); break;
    case 'a': cfg.relock = true; break;
    case 'e':
        cfg.exposure_request = std::clamp(v, 0, 1200);
        out_printf("# exposure %d\n", v);
        break;
    case 'g':
        cfg.gain_request = std::clamp(v, 0, 30);
        out_printf("# gain %d\n", v);
        break;
    case 'r': cfg.flip_request = true; break;
    case 'q': cfg.quality = std::clamp(v, 1, 100); out_printf("# quality %d\n", cfg.quality); break;
    case 'm':
        cfg.mono = v != 0;
        out_printf("# mode %s\n", cfg.mono ? "mono" : "rgb");
        break;
    case 'E':
        cfg.magnify = std::clamp(v, 0, 2);
        cfg.evm_reset_request = true;   // the camera task owns the magnifier's state
        out_printf("# preview magnification %s\n",
                   cfg.magnify == 0 ? "off" : cfg.magnify == 1 ? "motion (breathing band)" : "colour (pulse band)");
        break;
    case 'C': {   // C x,y,w,h -- watch this region when no face is found; C0 clears
        int x = 0, y = 0, w = 0, h = 0;
        if (std::sscanf(line + 1, "%d,%d,%d,%d", &x, &y, &w, &h) == 4 && w > 16 && h > 16) {
            cfg.watch_x = int16_t(std::clamp(x, 0, W - 1));
            cfg.watch_y = int16_t(std::clamp(y, 0, H - 1));
            cfg.watch_w = int16_t(std::clamp(w, 0, W - cfg.watch_x));
            cfg.watch_h = int16_t(std::clamp(h, 0, H - cfg.watch_y));
            out_printf("# watching %d,%d %dx%d\n", cfg.watch_x, cfg.watch_y, cfg.watch_w, cfg.watch_h);
        } else {
            cfg.watch_w = cfg.watch_h = 0;
            out_printf("# watch region cleared\n");
        }
        break;
    }
    case 'z':
        cfg.zoom = v != 0;
        if (!cfg.zoom) cfg.zoom_reset = true;
        out_printf("# zoom %s\n", cfg.zoom ? "auto (sensor window follows the face)" : "off (full field of view)");
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

// Blink the LED fast and say why, rather than running on in an unknown state.
// Stops this task and blinks fast. If the tasks are already running when this
// is reached, the watchdog in led_task will see this one stop stamping and
// restart the device about twenty seconds later -- which is what an unattended
// monitor should do, since a reboot may clear a transient fault and a device
// blinking quietly to itself in a nursery helps nobody. Reached before the
// tasks start (an allocation that failed at boot), there is no watchdog yet and
// the blinking is all there is, which is also right: a reboot would only repeat
// it.
[[noreturn]] static void fatal(const char *why)
{
    ESP_LOGE(TAG, "fatal: %s", why);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    for (;;) {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        ESP_LOGE(TAG, "fatal: %s", why);
        vTaskDelay(pdMS_TO_TICKS(3000));
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

    g_buf = new (std::nothrow) rppg::Buffer();
    g_est = new (std::nothrow) rppg::Estimator();
    g_rbuf = new (std::nothrow) resp::Buffer();
    g_rest = new (std::nothrow) resp::Estimator();
    g_tiles = new (std::nothrow) resp::TileMotion();
    g_evm = new (std::nothrow) evm::Magnifier();
    // 16-byte aligned: the JPEG encoder silently falls back to a slow C path
    // for an unaligned input, which was costing most of the preview frame rate.
    det_frame = (uint8_t *)heap_caps_aligned_alloc(16, W * H * 2, MALLOC_CAP_SPIRAM);
    str_frame = (uint8_t *)heap_caps_aligned_alloc(16, W * H * 2, MALLOC_CAP_SPIRAM);
    if (!g_buf || !g_est || !g_rbuf || !g_rest || !g_tiles || !g_evm || !det_frame || !str_frame || !out_mtx ||
        !face_mtx || !buf_mtx || !stats.mtx || !det_sem || !str_sem) {
        fatal("out of memory during start-up");
    }

    ESP_LOGI(TAG, "booting");
    if (!camera_init()) fatal("camera did not initialise");
    auto_then_lock(2500);

    // A task that failed to start would leave the device quietly half-working,
    // which is worse than not starting at all.
    struct { TaskFunction_t fn; const char *name; int stack, prio, core; } tasks[] = {
        {cam_task, "cam", 6144, 6, 1},   {detect_task, "detect", 16384, 4, 0},
        {hr_task, "hr", 24576, 5, 1},    {stream_task, "stream", 8192, 3, 0},
        {led_task, "led", 2048, 2, 1},   {cmd_task, "cmd", 4096, 5, 0},
    };
    // Start every heartbeat now, so the watchdog's deadline is measured from
    // when the tasks began rather than from boot. Otherwise a slow start is
    // indistinguishable from a task that has hung.
    for (auto &b : beats) b.store(esp_timer_get_time(), std::memory_order_relaxed);
    for (const auto &t : tasks)
        if (xTaskCreatePinnedToCore(t.fn, t.name, t.stack, nullptr, t.prio, nullptr, t.core) != pdPASS)
            fatal("could not start a task");
    ESP_LOGI(TAG, "ready");
}
