// Host test for breathing detection: renders synthetic 320x240 YUYV frames
// (textured scene with a "person" whose chest moves sub-pixel), runs them
// through TileMotion + resp::Estimator exactly as the firmware does.
// Build: g++ -O2 -std=c++17 -I../main test_resp.cpp ../main/motion.cpp ../main/resp.cpp -o test_resp
#include "motion.hpp"
#include "resp.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <random>
#include <string>
#include <vector>

using namespace resp;

static constexpr int W = 320, H = 240;

struct Scene {
    std::vector<float> bg, person;  // luminance textures, larger than the frame for shifting
    int pad = 16;
    void build(unsigned seed)
    {
        std::mt19937 rng(seed);
        std::uniform_real_distribution<float> u(0, 1);
        const int w = W + 2 * pad, h = H + 2 * pad;
        auto make = [&](std::vector<float> &img, int blobs, float lo, float hi) {
            img.assign(w * h, 0);
            for (int b = 0; b < blobs; b++) {
                const float cx = u(rng) * w, cy = u(rng) * h, r = 3 + u(rng) * 14, a = (u(rng) - 0.5f) * 80;
                for (int y = std::max(0, int(cy - 3 * r)); y < std::min(h, int(cy + 3 * r)); y++)
                    for (int x = std::max(0, int(cx - 3 * r)); x < std::min(w, int(cx + 3 * r)); x++) {
                        const float d2 = ((x - cx) * (x - cx) + (y - cy) * (y - cy)) / (r * r);
                        img[y * w + x] += a * std::exp(-0.5f * d2);
                    }
            }
            for (auto &v : img) v = std::clamp(lo + (hi - lo) * 0.5f + v, 10.0f, 245.0f);
        };
        make(bg, 400, 60, 140);
        make(person, 300, 90, 170);  // clothing / skin texture
    }
    float sample(const std::vector<float> &img, float x, float y) const
    {
        const int w = W + 2 * pad;
        x += pad;
        y += pad;
        const int x0 = int(x), y0 = int(y);
        const float fx = x - x0, fy = y - y0;
        const float a = img[y0 * w + x0], b = img[y0 * w + x0 + 1];
        const float c = img[(y0 + 1) * w + x0], d = img[(y0 + 1) * w + x0 + 1];
        return (a + fx * (b - a)) * (1 - fy) + (c + fx * (d - c)) * fy;
    }
};

struct Scenario {
    const char *name;
    float seconds;
    std::function<float(float)> rate;  // breaths/min vs time (0 = not breathing)
    float amp = 0.5f;                  // chest displacement amplitude (px)
    Band band = Band::ADULT;
    float noise = 2.0f;                // pixel noise (levels)
    float flicker = 0.0f;              // relative global brightness modulation
    float flicker_hz = 0.3f;
    float bcg = 0.05f;                 // heartbeat micro-motion (px) at 1.2 Hz
    std::function<bool(float)> gross = [](float) { return false; };
    float tol = 1.5f;
    enum { LOCK, NEVER } expect = LOCK;
    bool local_motion = false;         // hand-like random movement in the lower-left tile
};

static int run(const Scenario &sc, unsigned seed)
{
    static Scene scene;
    scene.build(seed * 7 + 1);
    static TileMotion tm;
    static Buffer buf;
    static Estimator est;
    static MotionSample tmp[kCap];
    tm.reset();
    buf.clear();
    est.set_band(sc.band);
    est.reset();
    std::mt19937 rng(seed);
    std::normal_distribution<float> g(0, 1);
    std::uniform_real_distribution<float> u(-1, 1);
    std::vector<uint8_t> frame(W * H * 2);

    float t = 0, phase = 0, jump_x = 0, jump_y = 0, next = 1;
    int checks = 0, fails = 0, locked = 0, confident = 0, wrong = 0;
    float err = 0;
    while (t < sc.seconds) {
        const float dt = 0.09f + 0.01f * u(rng);
        t += dt;
        const float br = sc.rate(t);
        if (br > 0) phase += 2 * M_PI * br / 60 * dt;
        // breathing waveform: faster inhale than exhale
        const float s = br > 0 ? sc.amp * (std::sin(phase) + 0.25f * std::sin(2 * phase)) : 0;
        const float beat = sc.bcg * std::sin(2 * M_PI * 1.2f * t);
        const bool gross = sc.gross(t);
        if (gross) { jump_x += 1.5f * g(rng); jump_y += 1.5f * g(rng); }
        jump_x = std::clamp(jump_x, -10.0f, 10.0f);
        jump_y = std::clamp(jump_y, -10.0f, 10.0f);
        const float light = 1 + sc.flicker * std::sin(2 * M_PI * sc.flicker_hz * t);
        static float hand_x = 0, hand_y = 0;
        if (sc.local_motion) { hand_x = 6 * std::sin(1.7f * t) + 2 * g(rng); hand_y = 5 * std::cos(2.3f * t) + 2 * g(rng); }

        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                float v;
                const bool chest = x >= 70 && x < 250 && y >= 130;
                const bool head = x >= 120 && x < 200 && y >= 30 && y < 130;
                const bool hand = sc.local_motion && x < 60 && y >= 180;
                if (hand) v = scene.sample(scene.person, std::clamp(x - hand_x, 0.0f, W - 2.0f), std::clamp(y - hand_y, 0.0f, H - 2.0f));
                else if (chest) v = scene.sample(scene.person, x - jump_x, y - s - beat - jump_y);
                else if (head) v = scene.sample(scene.person, x - jump_x, y - 0.3f * s - beat - jump_y);
                else v = scene.sample(scene.bg, x, y);
                v = std::clamp(v * light + sc.noise * g(rng), 0.0f, 255.0f);
                // YUYV as the sensor delivers it; the scene is grey, so chroma is neutral
                frame[(y * W + x) * 2] = uint8_t(v);
                frame[(y * W + x) * 2 + 1] = 128;
            }
        MotionSample ms;
        // the firmware anchors this under the detected face; the rendered chest is at x 70-250, y >= 130
        const Box boxes[kBoxes] = {{70, 130, 180, 100}, {120, 30, 80, 100}};  // chest, head
        tm.process_yuyv(frame.data(), W, H, uint32_t(t * 1000), false, boxes, ms);
        buf.push(ms);

        if (t >= next) {
            next += 1;
            const int n = buf.copy(tmp);
            const Result r = est.update(tmp, n, ms.t_ms);
            if (getenv("DBG"))
                std::printf("    t=%5.1f true=%5.1f br=%5.1f raw=%5.1f snr=%5.1f q=%.2f stab=%.2f agr=%.2f st=%d ch=%d best=%d mot=%.2f\n",
                            t, br, r.brpm, r.raw, r.snr_db, r.quality, r.stability, r.agreement, r.state,
                            r.channels, r.best, r.motion);
            const bool lk = r.state == rppg::LOCKED;
            if (lk) locked++;
            // every confident output is checked, for the whole run
            // A 30 s window cannot follow a rate change instantly, so a window
            // spanning a change is not scored for accuracy (the change itself is
            // covered by the end-of-run coverage check).
            const bool spans_change = std::fabs(sc.rate(t) - sc.rate(std::max(0.0f, t - 30))) > 0.1f;
            if (lk && t > 40 && !spans_change) {
                confident++;
                if (std::fabs(r.brpm - br) > sc.tol) wrong++;
            }
            if (lk && sc.expect == Scenario::NEVER) wrong++;
            if (t > sc.seconds - 15) {
                checks++;
                err += std::fabs(r.brpm - br);
                if (sc.expect == Scenario::LOCK && !lk) fails++;
            }
        }
    }
    const bool ok = wrong == 0 && fails <= checks / 5;
    std::printf("  %-36s seed=%u  err=%5.2f  locked=%3ds  wrong=%d/%d  %s\n", sc.name, seed, err / checks, locked,
                wrong, confident, ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}

// Direct MotionSample feeds for the failure modes found in review: skipped
// motion flags, aliasing, constant input, timestamp precision and a stalled
// camera. None of these may produce a locked breathing rate.
static int feed_tests()
{
    struct Case {
        const char *name;
        float fps;
        uint32_t t0;
        std::function<float(float, int)> disp;    // displacement (px) per time and channel
        std::function<bool(float)> gross;
        std::function<bool(float)> jump;
        float stale_after;
        bool expect_lock;
        float rate;                               // expected rate when expect_lock
    };
    const std::vector<Case> cases = {
        // real breathing, but the newest sample stops arriving after 45 s
        {"stalled camera", 11.1f, 0, [](float t, int) { return 0.5f * std::sin(2 * M_PI * 15 / 60 * t); },
         [](float) { return false; }, [](float) { return false; }, 45, false, 0},
        // 4-pixel steps every 2 s, flagged, but falling between 5 Hz grid points
        {"flagged steps between grid points", 16.0f, 0,
         [](float t, int) { return 4.0f * std::floor(t / 2); },
         [](float t) { return std::fmod(t, 2.0f) < 1.0f / 16; }, [](float) { return false; }, 0, false, 0},
        // fast motion that aliases into the breathing band when point-sampled
        {"4.7 Hz motion (aliasing)", 11.1f, 0, [](float t, int) { return 0.5f * std::sin(2 * M_PI * 4.7f * t); },
         [](float) { return false; }, [](float) { return false; }, 0, false, 0},
        // a perfectly constant, large displacement
        {"constant 10000.1 px", 11.1f, 0, [](float, int) { return 10000.1f; },
         [](float) { return false; }, [](float) { return false; }, 0, false, 0},
        // real breathing after 25 days of uptime
        {"15/min after 25 days uptime", 11.1f, 2147483647u,
         [](float t, int) { return 0.5f * std::sin(2 * M_PI * 15 / 60 * t); },
         [](float) { return false; }, [](float) { return false; }, 0, true, 15},
    };
    int fails = 0;
    for (const auto &c : cases) {
        static Buffer buf;
        static Estimator est;
        static MotionSample tmp[kCap];
        buf.clear();
        est.reset();
        std::mt19937 rng(1);
        std::normal_distribution<float> g(0, 0.01f);
        float t = 0, next = 1;
        int locked = 0, checks = 0;
        float err = 0;
        bool hard_fail = false;
        while (t < 80) {
            t += 1 / c.fps;
            MotionSample ms{};
            ms.t_ms = c.t0 + uint32_t(t * 1000);
            ms.valid = 0xFFFFFFFFu;
            ms.gross = c.gross(t);
            ms.jump = c.jump(t) ? 0xFFFFFFFFu : 0;
            for (int ch = 0; ch < kChan; ch++) ms.d[ch] = c.disp(t, ch) + g(rng);
            const bool frozen = c.stale_after > 0 && t > c.stale_after;
            if (!frozen) buf.push(ms);
            if (t >= next) {
                next += 1;
                const int n = buf.copy(tmp);
                const Result r = est.update(tmp, n, ms.t_ms);
                const bool lk = r.state == rppg::LOCKED;
                if (lk) locked++;
                if (t > 40) {
                    checks++;
                    // locks before a stall are legitimate; only a stale lock counts
                    if (!c.expect_lock && lk && (c.stale_after == 0 || t > c.stale_after + 3)) err += 1;
                    if (!c.expect_lock && lk && c.stale_after == 0) hard_fail = true;  // must never lock
                    if (c.expect_lock && (!lk || std::fabs(r.brpm - c.rate) > 1.5f)) err += 1;
                }
            }
        }
        const bool ok = err <= checks / 10 && !hard_fail;
        std::printf("  %-36s locked=%3ds  bad=%2.0f/%d  %s\n", c.name, locked, err, checks, ok ? "PASS" : "FAIL");
        fails += !ok;
    }
    return fails;
}

// Presence is the monitor's real question: is anything breathing *now*. A rate
// computed over thirty seconds cannot answer it, so this measures how long the
// separate presence check takes to notice breathing starting and, more
// importantly, stopping. The numbers it prints are the specification: they are
// what "it would tell you" means in seconds.
static int presence_tests()
{
    auto trial = [](float amp, float stop_at, float &t_seen, float &t_lost) {
        static Buffer buf;
        static Estimator est;
        static MotionSample tmp[kCap];
        buf.clear();
        est.reset();
        std::mt19937 rng(5);
        std::normal_distribution<float> g(0, 0.01f);
        t_seen = t_lost = -1;
        float phase = 0;
        for (float t = 0; t < stop_at + 40; t += 1 / 20.0f) {
            MotionSample ms{};
            ms.t_ms = uint32_t(t * 1000);
            ms.valid = 0xFFFFFFFFFull;
            const bool breathing = t < stop_at;
            if (breathing) phase += 2 * float(M_PI) * (15.0f / 60) * (1 / 20.0f);
            const float b = breathing ? amp * std::sin(phase) : 0.0f;
            for (int ch = 0; ch < kChan; ch++) ms.d[ch] = g(rng);
            ms.d[kBoxChan0] += b;               // the chest box carries it
            ms.d[2 * 9] += b;
            buf.push(ms);
            const Result r = est.update(tmp, buf.copy(tmp), ms.t_ms);
            if (r.present && t_seen < 0) t_seen = t;
            if (t_seen >= 0 && t > stop_at && !r.present && t_lost < 0) t_lost = t - stop_at;
        }
    };
    float seen_normal, lost_normal, seen_shallow, lost_shallow;
    trial(0.5f, 60, seen_normal, lost_normal);        // an adult's chest
    trial(0.12f, 60, seen_shallow, lost_shallow);     // shallow: an infant, or under a blanket
    struct { const char *name; float v; bool ok; } checks[] = {
        {"notices breathing within 10 s", seen_normal, seen_normal > 0 && seen_normal < 10.0f},
        {"notices it stopping within 15 s", lost_normal, lost_normal > 0 && lost_normal < 15},
        {"notices shallow breathing at all", seen_shallow, seen_shallow > 0},
        {"notices shallow breathing stopping", lost_shallow, lost_shallow > 0 && lost_shallow < 15},
    };
    int fails = 0;
    for (auto &c : checks) {
        std::printf("  %-52s %5.1f s  %s\n", c.name, c.v, c.ok ? "PASS" : "FAIL");
        fails += !c.ok;
    }
    return fails;
}

// The chest box is anchored to the face box, and the face detector's idea of
// where the face is wobbles by a pixel or two between frames even when nobody
// moves. Since the box profile is measured in box coordinates, that wobble
// shifts the content against the reference and reads as motion -- at exactly the
// amplitude real breathing has. Check that a jittering box on a frozen scene
// reports nothing, while a still box over moving content still reports it.
static int box_jitter_tests()
{
    Scene scene;
    scene.build(3);
    std::vector<uint8_t> frame(W * H * 2);
    auto render = [&](float content_dy) {
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                const bool chest = x >= 70 && x < 250 && y >= 130;
                const float v = chest ? scene.sample(scene.person, x, y - content_dy)
                                      : scene.sample(scene.bg, x, y);
                frame[(y * W + x) * 2] = uint8_t(std::clamp(v, 0.0f, 255.0f));
                frame[(y * W + x) * 2 + 1] = 128;
            }
    };
    // returns the peak-to-peak vertical displacement the chest box reported
    auto run = [&](bool jitter_box, bool move_content) {
        TileMotion tm;
        float lo = 1e9f, hi = -1e9f;
        for (int f = 0; f < 120; f++) {
            const float t = f / 20.0f;
            const float content = move_content ? std::sin(2 * float(M_PI) * 0.25f * t) : 0.0f;
            render(content);
            // a detector wobble of a pixel or two, the size seen in practice
            const int16_t dy = jitter_box ? int16_t(std::lround(std::sin(2 * float(M_PI) * 0.25f * t))) : 0;
            const Box boxes[kBoxes] = {{70, int16_t(130 + dy), 180, 100}, {120, 30, 80, 100}};
            MotionSample ms;
            tm.process_yuyv(frame.data(), W, H, uint32_t(t * 1000), false, boxes, ms);
            if (f < 40) continue;  // let the reference settle
            if (!(ms.valid & (1ull << kBoxChan0))) continue;
            lo = std::fmin(lo, ms.d[kBoxChan0]);
            hi = std::fmax(hi, ms.d[kBoxChan0]);
        }
        return hi - lo;
    };
    const float jitter_only = run(true, false);
    const float content_only = run(false, true);
    // A slide bigger than the fit can measure must be re-keyed, not corrected:
    // past a couple of bins the correction saturates and would report a large
    // displacement that never happened.
    auto big_slide = [&]() {
        TileMotion tm;
        float lo = 1e9f, hi = -1e9f;
        for (int f = 0; f < 120; f++) {
            const float t = f / 20.0f;
            render(0.0f);                          // the scene never moves
            const int16_t dy = int16_t(f > 60 ? 12 : 0);   // the box jumps once, a long way
            const Box boxes[kBoxes] = {{70, int16_t(130 + dy), 180, 100}, {120, 30, 80, 100}};
            MotionSample ms;
            tm.process_yuyv(frame.data(), W, H, uint32_t(t * 1000), false, boxes, ms);
            if (f < 40 || !(ms.valid & (1ull << kBoxChan0))) continue;
            lo = std::fmin(lo, ms.d[kBoxChan0]);
            hi = std::fmax(hi, ms.d[kBoxChan0]);
        }
        return hi - lo;
    };
    const float slid = big_slide();
    struct { const char *name; bool ok; } checks[] = {
        {"a jittering box on a frozen scene reports no motion", jitter_only < 0.4f},
        {"a still box over moving content still reports it", content_only > 1.0f},
        {"a box sliding further than the fit can see is re-keyed", slid < 1.0f},
    };
    int fails = 0;
    for (auto &c : checks) {
        std::printf("  %-52s jitter=%.2f content=%.2f slide=%.2f px  %s\n", c.name, jitter_only,
                    content_only, slid, c.ok ? "PASS" : "FAIL");
        fails += !c.ok;
    }
    return fails;
}

// One movement seen through several overlapping regions must not be counted as
// several independent witnesses. Feed the same oscillation twice: once confined
// to a single tile and the chest box lying over it (four channels, one physical
// region), once spread over four separate tiles (four channels, four regions),
// and check the estimator can tell the difference.
static int region_tests()
{
    auto run_case = [](bool overlapping) {
        static Buffer buf;
        static Estimator est;
        static MotionSample tmp[kCap];
        buf.clear();
        est.reset();
        std::mt19937 rng(7);
        std::normal_distribution<float> g(0, 0.01f);
        Result r{};
        for (float t = 0; t < 80; t += 1 / 20.0f) {
            MotionSample ms{};
            ms.t_ms = uint32_t(t * 1000);
            ms.valid = 0xFFFFFFFFFull;
            for (int ch = 0; ch < kChan; ch++) ms.d[ch] = g(rng);
            const float b = 0.5f * std::sin(2 * float(M_PI) * (15.0f / 60) * t);
            if (overlapping) {
                ms.box_tiles[0] = 1u << 5;               // the chest box lies over tile 5
                ms.d[2 * 5] += b;                        // tile 5, both axes
                ms.d[2 * 5 + 1] += b;
                ms.d[kBoxChan0] += b;                    // the box itself, both axes
                ms.d[kBoxChan0 + 1] += b;
            } else {
                for (int tile : {5, 6, 9, 10}) ms.d[2 * tile] += b;   // four separate tiles
            }
            buf.push(ms);
            r = est.update(tmp, buf.copy(tmp), ms.t_ms);
        }
        return r;
    };
    const Result one = run_case(true), many = run_case(false);
    // The point of grouping is not the count but what it costs: one region,
    // however many channels saw it, must not be scored as corroborated. So the
    // check that matters is that the overlapping case is trusted *less*.
    struct { const char *name; bool ok; } checks[] = {
        {"one region seen four ways counts as one", one.regions == 1},
        {"four separate tiles count as several", many.regions >= 3},
        {"one region is trusted less than several", one.quality < many.quality},
    };
    int fails = 0;
    for (auto &c : checks) {
        std::printf("  %-52s regions=%d/%d q=%.2f/%.2f  %s\n", c.name, one.regions, many.regions,
                    one.quality, many.quality, c.ok ? "PASS" : "FAIL");
        fails += !c.ok;
    }
    return fails;
}

int main(int argc, char **argv)
{
    auto hr = [](float v) { return [v](float) { return v; }; };
    std::vector<Scenario> sc = {
        {"adult 15/min, 0.5 px", 75, hr(15)},
        {"slow 8/min, 0.8 px", 75, hr(8), 0.8f},
        {"fast 30/min, 0.4 px", 75, hr(30), 0.4f},
        {"shallow 12/min, 0.15 px", 75, hr(12), 0.15f},
        {"infant 45/min (infant band)", 75, hr(45), 0.3f, Band::INFANT},
        {"infant 70/min (top of the band)", 75, hr(70), 0.3f, Band::INFANT},
        {"change 12->20/min at 40-50 s", 90, [](float t) { return t < 40 ? 12 : (t > 50 ? 20.0f : 12 + 0.8f * (t - 40)); },
         0.5f, Band::ADULT, 2.0f, 0, 0.3f, 0.05f, [](float) { return false; }, 2.5f},
        {"gross motion 2 s every 20 s, 16/min", 90, hr(16), 0.5f, Band::ADULT, 2.0f, 0, 0.3f, 0.05f,
         [](float t) { return std::fmod(t, 20.0f) < 2; }},
        {"light flicker 3% + 14/min", 75, hr(14), 0.5f, Band::ADULT, 2.0f, 0.03f},
        {"hand moving in a corner, 13/min", 75, hr(13), 0.5f, Band::ADULT, 2.0f, 0, 0.3f, 0.05f,
         [](float) { return false; }, 1.5f, Scenario::LOCK, true},
        {"no breathing (must not lock)", 75, hr(0), 0.5f, Band::ADULT, 2.0f, 0, 0.3f, 0.05f,
         [](float) { return false; }, 1.5f, Scenario::NEVER},
        {"no breathing, light flicker 0.3 Hz", 75, hr(0), 0.5f, Band::ADULT, 2.0f, 0.05f, 0.3f, 0.05f,
         [](float) { return false; }, 1.5f, Scenario::NEVER},
        {"no breathing, heartbeat 0.2 px", 75, hr(0), 0.5f, Band::ADULT, 2.0f, 0, 0.3f, 0.2f,
         [](float) { return false; }, 1.5f, Scenario::NEVER},
    };
    const int seeds = argc > 1 ? atoi(argv[1]) : 3;
    if (argc > 2) {  // run only scenarios whose name contains argv[2]
        std::vector<Scenario> f;
        for (auto &s : sc)
            if (std::string(s.name).find(argv[2]) != std::string::npos) f.push_back(s);
        sc = f;
    }
    int fails = feed_tests() + region_tests() + box_jitter_tests() + presence_tests(), total = 15;
    for (auto &s : sc)
        for (int seed = 1; seed <= seeds; seed++) {
            fails += run(s, seed);
            total++;
        }
    std::printf("%d/%d passed\n", total - fails, total);
    return fails ? 1 : 0;
}
