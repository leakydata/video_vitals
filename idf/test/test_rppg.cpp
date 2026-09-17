// Host test for the rPPG estimator on synthetic face-colour traces.
// Build: g++ -O2 -std=c++17 -I../main test_rppg.cpp ../main/rppg.cpp -o test_rppg
#include "rppg.hpp"

#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <cstdlib>

using namespace rppg;

struct Scenario {
    const char *name;
    float seconds;
    std::function<float(float)> bpm;  // true HR vs time; <=0 means no pulse
    Mode mode = Mode::RGB;
    float amp = 0.003f;               // pulse amplitude (relative)
    float noise = 0.15f;              // per-channel noise of an ROI mean (levels)
    std::function<bool(float)> face = [](float) { return true; };
    std::function<bool(float)> moving = [](float) { return false; };
    float tol = 3.0f;                 // required accuracy at the end
    // LOCK: must be locked and accurate; ABSTAIN_OK: may stay unlocked but any
    // locked reading must be accurate; NEVER: must not lock
    enum { LOCK, ABSTAIN_OK, NEVER } expect = LOCK;
    enum { NONE, RAMP_LAG, STEP } metric = NONE;
};

static int run(const Scenario &sc, unsigned seed)
{
    static Estimator est;
    static Buffer buf;
    static Sample tmp[kCap];
    est.reset();
    est.set_mode(sc.mode);
    buf.clear();
    std::mt19937 rng(seed);
    std::normal_distribution<float> gauss(0, 1);
    std::uniform_real_distribution<float> uni(-1, 1);

    const float base[kRois][3] = {{175, 115, 95}, {165, 105, 88}, {168, 108, 90}};
    const float pv[3] = {0.33f, 0.77f, 0.53f};  // blood-volume pulse colour signature
    float phase = 0, drift = 0, t = 0, next_est = 1;
    int locked_s = 0, checks = 0, fails = 0;
    float err_sum = 0;
    float lag_sum = 0, settle = -1;
    int lag_n = 0;

    while (t < sc.seconds) {
        const float dt = 0.055f + 0.012f * uni(rng);
        t += dt;
        const float hr = sc.bpm(t);
        if (hr > 0) phase += 2 * M_PI * hr / 60 * dt;
        const float p = hr > 0 ? std::sin(phase) + 0.3f * std::sin(2 * phase + 0.5f) : 0;
        drift += 0.0008f * gauss(rng);
        const bool mv = sc.moving(t);
        float I = 0.02f * std::sin(2 * M_PI * 0.25f * t) + drift + (mv ? 0.03f * gauss(rng) : 0);
        // motion also shifts which pixels are averaged: a chromatic artefact POS cannot cancel
        const float art = mv ? 0.004f * std::sin(2 * M_PI * 0.85f * t) : 0;

        Sample s{};
        s.t_ms = uint32_t(t * 1000);
        s.motion = mv;
        for (int r = 0; r < kRois; r++) {
            if (!sc.face(t)) continue;
            float c[3];
            for (int k = 0; k < 3; k++) {
                const float pulse = sc.mode == Mode::RGB ? sc.amp * pv[k] * p : -sc.amp * p;
                const float chroma = k == 0 ? art : -art;
                c[k] = base[r][k] * (1 + I) * (1 + pulse + chroma) + sc.noise * (1 + 0.5f * r) * gauss(rng);
            }
            s.roi[r] = {c[0], c[1], c[2], 400};
        }
        buf.push(s);

        if (t >= next_est) {
            next_est += 1;
            const int n = buf.copy(tmp);
            const Result res = est.update(tmp, n, s.t_ms);
            if (getenv("DBG")) std::printf("    t=%4.0f true=%5.1f bpm=%5.1f raw=%5.1f snr=%5.1f q=%.2f st=%d roi=%.1f/%.1f/%.1f mot=%.2f stab=%.2f coh=%.2f\n", t, hr, res.bpm, res.bpm_raw, res.snr_db, res.quality, res.state, res.roi_snr[0], res.roi_snr[1], res.roi_snr[2], res.motion, res.stability, res.coherence);
            if (res.state == LOCKED) locked_s++;
            if (sc.metric == Scenario::RAMP_LAG && t > 25 && t < 60 && res.state == LOCKED) {
                lag_sum += (hr - res.bpm) / (40.0f / 60);  // slope of the ramp scenario
                lag_n++;
            }
            if (sc.metric == Scenario::STEP && t > 30 && settle < 0 && std::fabs(res.bpm - hr) < 5 && res.state == LOCKED)
                settle = t - 30;
            // score the last 10 s
            if (t > sc.seconds - 10) {
                checks++;
                const float e = std::fabs(res.bpm - hr);
                err_sum += e;
                const bool locked = res.state == LOCKED;
                if (sc.expect == Scenario::LOCK && (!locked || e > sc.tol)) fails++;
                if (sc.expect == Scenario::ABSTAIN_OK && locked && e > sc.tol) fails++;
                if (sc.expect == Scenario::NEVER && locked) fails++;
            }
        }
    }
    const bool ok = fails <= checks / 5;
    char extra[48] = "";
    if (sc.metric == Scenario::RAMP_LAG) std::snprintf(extra, sizeof(extra), "  lag=%.1fs", lag_n ? lag_sum / lag_n : -1);
    if (sc.metric == Scenario::STEP) std::snprintf(extra, sizeof(extra), "  settle=%.0fs", settle);
    std::printf("  %-34s seed=%u  mean_err_last10s=%5.2f bpm  locked=%3ds  %s%s\n", sc.name, seed,
                checks ? err_sum / checks : 0, locked_s, ok ? "PASS" : "FAIL", extra);
    return ok ? 0 : 1;
}

int main()
{
    auto always = [](float) { return true; };
    auto never = [](float) { return false; };
    auto hr = [](float v) { return [v](float) { return v; }; };
    std::vector<Scenario> sc = {
        {"steady 72 bpm", 40, hr(72)},
        {"steady 52 bpm (resting)", 40, hr(52)},
        {"steady 150 bpm (exercise)", 40, hr(150)},
        {"ramp 60->100 bpm", 70, [](float t) { return 60 + 40 * std::min(t / 60, 1.0f); }, Mode::RGB, 0.003f, 0.15f,
         always, never, 5, Scenario::LOCK, Scenario::RAMP_LAG},
        {"step 65->95 bpm at 30 s", 60, [](float t) { return t < 30 ? 65.0f : 95.0f; }, Mode::RGB, 0.003f, 0.15f,
         always, never, 5, Scenario::LOCK, Scenario::STEP},
        // hand-held: motion flagged ~25% of the time with a strong low-frequency artefact
        {"hand-held motion 25%, 80 bpm", 60, hr(80), Mode::RGB, 0.003f, 0.15f, always,
         [](float t) { return std::fmod(t, 4.0f) < 1.0f; }, 5, Scenario::ABSTAIN_OK},
        // borderline amplitude: the estimator may decline to lock, but must not be wrong
        {"weak pulse (0.15%) 80 bpm", 50, hr(80), Mode::RGB, 0.0015f, 0.15f, always, never, 3, Scenario::ABSTAIN_OK},
        {"face lost 20-24 s, 68 bpm", 50, hr(68), Mode::RGB, 0.003f, 0.15f, [](float t) { return t < 20 || t > 24; }},
        {"motion bursts, 76 bpm", 50, hr(76), Mode::RGB, 0.003f, 0.15f, always,
         [](float t) { return std::fmod(t, 15.0f) < 2; }},
        {"mono/IR 58 bpm", 40, hr(58), Mode::MONO, 0.004f},
        // mono cannot separate illumination drift from pulse; transient errors of up to
        // ~10 bpm are expected until a background reference ROI is added (IR mode TODO)
        {"mono/IR weak 0.1% 64 bpm", 50, hr(64), Mode::MONO, 0.001f, 0.15f, always, never, 11},
        {"stress: noise x3, 72 bpm", 50, hr(72), Mode::RGB, 0.003f, 0.45f, always, never, 4, Scenario::ABSTAIN_OK},
        {"stress: noise x3, 150 bpm", 50, hr(150), Mode::RGB, 0.003f, 0.45f, always, never, 4, Scenario::ABSTAIN_OK},
        {"stress: noise x3, 52 bpm", 50, hr(52), Mode::RGB, 0.003f, 0.45f, always, never, 4, Scenario::ABSTAIN_OK},
        {"no pulse (must not lock)", 40, hr(0), Mode::RGB, 0.003f, 0.15f, always, never, 3, Scenario::NEVER},
        {"no pulse, noise x3", 40, hr(0), Mode::RGB, 0.003f, 0.45f, always, never, 3, Scenario::NEVER},
    };
    int fails = 0, total = 0;
    for (auto &s : sc)
        for (unsigned seed : {1u, 2u, 3u, 4u, 5u}) {
            fails += run(s, seed);
            total++;
        }
    std::printf("%d/%d passed\n", total - fails, total);
    return fails ? 1 : 0;
}
