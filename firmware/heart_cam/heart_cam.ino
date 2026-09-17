// heart_cam — remote photoplethysmography (rPPG) on a XIAO ESP32-S3 Sense.
//
// The camera captures QVGA JPEG. Each frame is decoded at 1/2 scale, skin
// pixels in the central region are averaged into one (R,G,B) sample, and the
// POS algorithm (Wang et al. 2017) turns ~10 s of samples into a pulse signal
// whose spectral peak is the heart rate. Runs fully on-device; the user LED
// blinks at the detected rate.
//
// Serial protocol (USB CDC):
//   device -> host  text lines ("# log", "HR ...", "RGB ...")
//                   binary frames: 'H','C','F','1' | u32 len | u32 t_ms | jpeg
//   host -> device  single-letter commands, one per line:
//     s / x   start / stop JPEG streaming
//     h / o   on-device HR on / off
//     a       re-run auto exposure + white balance for 2 s, then lock
//     e<n>    manual exposure (0..1200)      g<n>  manual gain (0..30)
//     q<n>    JPEG quality (4..63, lower = better)
//     i       print info

#include "esp_camera.h"
#include "img_converters.h"
#include <math.h>

// XIAO ESP32-S3 Sense camera pins
#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13
#define LED_PIN        21  // active low

// ---- rPPG parameters ----
#define FS          20.0f   // resample rate (Hz)
#define WIN_SEC     10.0f   // analysis window
#define N_SAMP      200     // WIN_SEC * FS
#define POS_L       32      // POS sub-window (1.6 s)
#define RAW_CAP     512     // raw sample ring buffer
#define BPM_MIN     42.0f
#define BPM_MAX     180.0f
#define BPM_STEP    0.5f
#define MIN_SKIN    150     // minimum skin pixels for a valid sample

static bool streaming = false;
static bool hrEnabled = true;
static uint8_t *rgbBuf = nullptr;
static int decW = 160, decH = 120;

struct Sample { uint32_t t; float r, g, b; };
static Sample raw[RAW_CAP];
static int rawHead = 0, rawCount = 0;

static float hrSmooth = 0, hrSnr = 0;
static float hrHist[5]; static int hrHistN = 0, hrHistI = 0;
static uint32_t lastEstimate = 0, lastBeat = 0, lastFpsT = 0;
static int fpsFrames = 0; static float fps = 0;
static int lastSkin = 0;
static uint32_t lastInvalidT = 0;

static const uint8_t MAGIC[4] = {'H', 'C', 'F', '1'};

// ------------------------------------------------------------------ camera
static bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM; c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_QVGA;
  c.jpeg_quality = 8;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) { Serial.printf("# camera init failed 0x%x\n", err); return false; }
  sensor_t *s = esp_camera_sensor_get();
  Serial.printf("# sensor PID=0x%04x\n", s->id.PID);
  if (s->id.PID == OV3660_PID) { s->set_vflip(s, 1); }
  s->set_lenc(s, 1);
  s->set_bpc(s, 1);
  s->set_wpc(s, 1);
  return true;
}

// Auto exposure/white balance is poison for rPPG: every adjustment is a step
// in the signal far larger than the pulse. Let it settle, then freeze it.
static void autoThenLock(uint32_t ms) {
  sensor_t *s = esp_camera_sensor_get();
  s->set_whitebal(s, 1); s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1); s->set_gain_ctrl(s, 1);
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
  }
  s->set_exposure_ctrl(s, 0);
  s->set_gain_ctrl(s, 0);
  s->set_whitebal(s, 0);
  s->set_awb_gain(s, 0);
  rawCount = 0;  // history is invalid across an exposure change
  Serial.println("# exposure/gain/awb locked");
}

// ------------------------------------------------------------------ rPPG
// Mean colour of skin-like pixels in the central 70% of the decoded frame.
static bool skinMean(const uint8_t *px, float &R, float &G, float &B, int &count) {
  double sr = 0, sg = 0, sb = 0; int n = 0;
  int x0 = decW * 15 / 100, x1 = decW * 85 / 100;
  int y0 = decH * 10 / 100, y1 = decH * 90 / 100;
  for (int y = y0; y < y1; y++) {
    const uint8_t *row = px + y * decW * 2;
    for (int x = x0; x < x1; x++) {
      // jpg2rgb565 writes big-endian RGB565
      uint16_t v = (row[x * 2] << 8) | row[x * 2 + 1];
      int r = ((v >> 11) & 0x1f) << 3;
      int g = ((v >> 5) & 0x3f) << 2;
      int b = (v & 0x1f) << 3;
      if (r > 245 || g > 245 || b > 245) continue;  // clipped
      int Y = (77 * r + 150 * g + 29 * b) >> 8;
      if (Y < 40) continue;
      int Cr = ((128 * r - 107 * g - 21 * b) >> 8) + 128;
      int Cb = ((-43 * r - 85 * g + 128 * b) >> 8) + 128;
      if (Cr < 133 || Cr > 173 || Cb < 77 || Cb > 127) continue;
      sr += r; sg += g; sb += b; n++;
    }
  }
  count = n;
  if (n < MIN_SKIN) return false;
  R = sr / n; G = sg / n; B = sb / n;
  return true;
}

static void pushSample(uint32_t t, float r, float g, float b) {
  raw[rawHead] = {t, r, g, b};
  rawHead = (rawHead + 1) % RAW_CAP;
  if (rawCount < RAW_CAP) rawCount++;
}

static const Sample &rawAt(int i) {  // i = 0 oldest
  return raw[(rawHead - rawCount + i + RAW_CAP) % RAW_CAP];
}

static float cr[N_SAMP], cg[N_SAMP], cb[N_SAMP], H[N_SAMP];

// Linear resample of the last WIN_SEC seconds onto a uniform FS grid.
static bool resample() {
  if (rawCount < 20) return false;
  uint32_t tEnd = rawAt(rawCount - 1).t;
  uint32_t span = (uint32_t)(WIN_SEC * 1000);
  if (tEnd - rawAt(0).t < span) return false;
  uint32_t tStart = tEnd - span;
  int j = 0;
  for (int i = 0; i < N_SAMP; i++) {
    float t = tStart + i * (1000.0f / FS);
    while (j < rawCount - 2 && rawAt(j + 1).t < t) j++;
    const Sample &a = rawAt(j), &b = rawAt(j + 1);
    float dt = (float)(b.t - a.t);
    if (dt > 500) return false;  // gap (face lost) inside window
    float u = dt > 0 ? (t - a.t) / dt : 0;
    if (u < 0) u = 0; if (u > 1) u = 1;
    cr[i] = a.r + u * (b.r - a.r);
    cg[i] = a.g + u * (b.g - a.g);
    cb[i] = a.b + u * (b.b - a.b);
  }
  return true;
}

static void pos() {
  for (int i = 0; i < N_SAMP; i++) H[i] = 0;
  float s1[POS_L], s2[POS_L];
  for (int n = 0; n + POS_L <= N_SAMP; n++) {
    float mr = 0, mg = 0, mb = 0;
    for (int k = 0; k < POS_L; k++) { mr += cr[n + k]; mg += cg[n + k]; mb += cb[n + k]; }
    mr /= POS_L; mg /= POS_L; mb /= POS_L;
    float m1 = 0, m2 = 0;
    for (int k = 0; k < POS_L; k++) {
      float r = cr[n + k] / mr, g = cg[n + k] / mg, b = cb[n + k] / mb;
      s1[k] = g - b;
      s2[k] = g + b - 2 * r;
      m1 += s1[k]; m2 += s2[k];
    }
    m1 /= POS_L; m2 /= POS_L;
    float v1 = 0, v2 = 0;
    for (int k = 0; k < POS_L; k++) {
      v1 += (s1[k] - m1) * (s1[k] - m1);
      v2 += (s2[k] - m2) * (s2[k] - m2);
    }
    float alpha = v2 > 0 ? sqrt(v1 / v2) : 0;
    float mh = 0, h[POS_L];
    for (int k = 0; k < POS_L; k++) { h[k] = s1[k] + alpha * s2[k]; mh += h[k]; }
    mh /= POS_L;
    for (int k = 0; k < POS_L; k++) H[n + k] += h[k] - mh;
  }
}

// Detrend, window, and scan the heart-rate band with a direct DFT.
static bool estimate(float &bpm, float &snr) {
  // remove a 1 s moving average (kills respiration / slow drift)
  static float tmp[N_SAMP];
  const int M = (int)FS;
  for (int i = 0; i < N_SAMP; i++) {
    int a = i - M / 2, b = i + M / 2;
    if (a < 0) a = 0; if (b > N_SAMP - 1) b = N_SAMP - 1;
    float s = 0; for (int k = a; k <= b; k++) s += H[k];
    tmp[i] = H[i] - s / (b - a + 1);
  }
  for (int i = 0; i < N_SAMP; i++)
    tmp[i] *= 0.5f - 0.5f * cos(2 * M_PI * i / (N_SAMP - 1));

  const int NB = (int)((BPM_MAX - BPM_MIN) / BPM_STEP) + 1;
  static float P[400];
  int best = 0; float total = 0;
  for (int k = 0; k < NB; k++) {
    float w = 2 * M_PI * (BPM_MIN + k * BPM_STEP) / 60.0f / FS;
    float re = 0, im = 0;
    for (int i = 0; i < N_SAMP; i++) { re += tmp[i] * cos(w * i); im += tmp[i] * sin(w * i); }
    P[k] = re * re + im * im;
    total += P[k];
    if (P[k] > P[best]) best = k;
  }
  if (total <= 0) return false;
  float d = 0;
  if (best > 0 && best < NB - 1) {
    float a = P[best - 1], b = P[best], c = P[best + 1];
    float den = a - 2 * b + c;
    if (den != 0) d = 0.5f * (a - c) / den;
  }
  bpm = BPM_MIN + (best + d) * BPM_STEP;
  // SNR: power within ±6 BPM of the peak vs. the rest of the band
  float inBand = 0;
  int w = (int)(6 / BPM_STEP);
  for (int k = best - w; k <= best + w; k++) if (k >= 0 && k < NB) inBand += P[k];
  float rest = total - inBand;
  snr = rest > 0 ? 10 * log10(inBand / rest) : 30;
  return true;
}

static void updateHr() {
  uint32_t now = millis();
  if (now - lastEstimate < 1000) return;
  lastEstimate = now;
  float bpm, snr;
  if (!resample()) {
    Serial.printf("HR bpm=0 snr=0 fps=%.1f skin=%d status=%s\n", fps, lastSkin,
                  rawCount ? "collecting" : "no_skin");
    hrSmooth = 0;
    return;
  }
  pos();
  if (!estimate(bpm, snr)) return;
  hrHist[hrHistI] = bpm; hrHistI = (hrHistI + 1) % 5;
  if (hrHistN < 5) hrHistN++;
  float s[5]; memcpy(s, hrHist, sizeof(s));
  for (int i = 0; i < hrHistN; i++)  // insertion sort for median
    for (int j = i; j > 0 && s[j] < s[j - 1]; j--) { float t = s[j]; s[j] = s[j - 1]; s[j - 1] = t; }
  hrSmooth = s[hrHistN / 2];
  hrSnr = snr;
  Serial.printf("HR bpm=%.1f raw=%.1f snr=%.1f fps=%.1f skin=%d status=ok\n",
                hrSmooth, bpm, snr, fps, lastSkin);
}

// ------------------------------------------------------------------ commands
static void handleCommand(String cmd) {
  cmd.trim();
  if (!cmd.length()) return;
  sensor_t *s = esp_camera_sensor_get();
  char c = cmd[0];
  int v = cmd.substring(1).toInt();
  switch (c) {
    case 's': streaming = true; Serial.println("# stream on"); break;
    case 'x': streaming = false; Serial.println("# stream off"); break;
    case 'h': hrEnabled = true; rawCount = 0; Serial.println("# hr on"); break;
    case 'o': hrEnabled = false; digitalWrite(LED_PIN, HIGH); Serial.println("# hr off"); break;
    case 'a': autoThenLock(2000); break;
    case 'e': s->set_exposure_ctrl(s, 0); s->set_aec_value(s, constrain(v, 0, 1200));
              rawCount = 0; Serial.printf("# exposure %d\n", v); break;
    case 'g': s->set_gain_ctrl(s, 0); s->set_agc_gain(s, constrain(v, 0, 30));
              rawCount = 0; Serial.printf("# gain %d\n", v); break;
    case 'q': s->set_quality(s, constrain(v, 4, 63)); Serial.printf("# quality %d\n", v); break;
    case 'i': Serial.printf("# heart_cam PID=0x%04x stream=%d hr=%d fps=%.1f psram=%u\n",
                            s->id.PID, streaming, hrEnabled, fps, (unsigned)ESP.getFreePsram()); break;
    default: Serial.printf("# unknown command '%s'\n", cmd.c_str());
  }
}

// ------------------------------------------------------------------ main
void setup() {
  Serial.setTxBufferSize(32768);
  Serial.begin(921600);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  Serial.println("# heart_cam booting");
  if (!psramFound()) Serial.println("# WARNING: no PSRAM");
  if (!cameraInit()) { while (true) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(100); } }
  rgbBuf = (uint8_t *)ps_malloc(decW * decH * 2);
  autoThenLock(2500);
  Serial.println("# ready");
}

void loop() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { handleCommand(line); line = ""; }
    else if (line.length() < 32) line += ch;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  uint32_t t = fb->timestamp.tv_sec * 1000UL + fb->timestamp.tv_usec / 1000UL;

  fpsFrames++;
  if (millis() - lastFpsT >= 2000) {
    fps = fpsFrames * 1000.0f / (millis() - lastFpsT);
    fpsFrames = 0; lastFpsT = millis();
  }

  if (hrEnabled && jpg2rgb565(fb->buf, fb->len, rgbBuf, JPG_SCALE_2X)) {
    float r, g, b;
    if (skinMean(rgbBuf, r, g, b, lastSkin)) {
      pushSample(t, r, g, b);
    } else if (millis() - lastInvalidT > 500) {
      lastInvalidT = millis();
      rawCount = 0;  // lost the subject: restart the window
      hrHistN = 0;
    }
  }

  if (streaming && Serial) {
    uint32_t hdr[2] = {(uint32_t)fb->len, t};
    Serial.write(MAGIC, 4);
    Serial.write((uint8_t *)hdr, 8);
    Serial.write(fb->buf, fb->len);
  }
  esp_camera_fb_return(fb);

  if (hrEnabled) {
    updateHr();
    // blink the LED with the detected pulse
    uint32_t now = millis();
    if (hrSmooth > 0 && hrSnr > -3) {
      uint32_t period = (uint32_t)(60000.0f / hrSmooth);
      if (now - lastBeat >= period) { lastBeat = now; digitalWrite(LED_PIN, LOW); }
      else if (now - lastBeat > 60) digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(LED_PIN, HIGH);
    }
  }
}
