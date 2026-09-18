// Runs the firmware breathing pipeline on frames piped from
// tools/video_frames.py (binary: "FRM1" | u32 t_ms | u16 w | u16 h | YUYV),
// or on "M ..." motion lines from a heartcam.py recording (--motion file).
// Prints one CSV row per second.
// Build: g++ -O2 -std=c++17 -I../main resp_replay.cpp ../main/motion.cpp ../main/resp.cpp -o resp_replay
// Usage: video_frames.py clip.mp4 | resp_replay [--infant]
//        resp_replay --motion recording.bin [--infant]
//        video_frames.py clip.mp4 | resp_replay --chest x,y,w,h
#include "motion.hpp"
#include "resp.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace resp;

static Buffer buf;
static Estimator est;
static MotionSample tmp[kCap];
static uint32_t first = 0, next = 0;
static bool have_first = false;

static void feed(const MotionSample &ms)
{
    if (!have_first) { first = next = ms.t_ms; have_first = true; }
    buf.push(ms);
    if (ms.t_ms >= next + 1000) {
        next += 1000;
        const int n = buf.copy(tmp);
        const Result r = est.update(tmp, n, ms.t_ms);
        std::printf("%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%d,%d,%d,%.2f,%.3f\n", (ms.t_ms - first) / 1000.0, r.brpm, r.raw,
                    r.snr_db, r.quality, r.stability, r.agreement, r.state, r.motion, r.channels, r.best, r.present ? 1 : 0, r.presence, r.swing);
    }
}

int main(int argc, char **argv)
{
    const char *motion_file = nullptr;
    Box boxes[kBoxes] = {};
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--infant")) est.set_band(Band::INFANT);
        else if (!std::strcmp(argv[i], "--motion") && i + 1 < argc) motion_file = argv[++i];
        else if (!std::strcmp(argv[i], "--chest") && i + 1 < argc) {
            int x, y, w, h;
            if (std::sscanf(argv[++i], "%d,%d,%d,%d", &x, &y, &w, &h) == 4)
                boxes[0] = {int16_t(x), int16_t(y), int16_t(w), int16_t(h)};
        }
    }
    std::printf("t,br,raw,snr,q,stab,agree,state,motion,channels,best,present,presence,swing\n");

    if (motion_file) {
        std::ifstream in(motion_file, std::ios::binary);
        std::string line;
        while (std::getline(in, line)) {
            const size_t p = line.rfind("M ");
            if (p == std::string::npos) continue;
            std::istringstream ss(line.substr(p + 2));
            MotionSample ms{};
            unsigned long t;
            unsigned long long valid, jump, subject;
            int gross;
            if (!(ss >> t >> valid >> jump >> subject >> gross)) continue;
            bool ok = true;
            for (float &d : ms.d)
                if (!(ss >> d)) { ok = false; break; }
            if (!ok) continue;
            ms.t_ms = t;
            ms.valid = valid;
            ms.jump = jump;
            ms.subject = subject;
            ms.gross = gross;
            feed(ms);
        }
        return 0;
    }

    TileMotion tm;
    std::vector<uint8_t> frame;
    char hdr[12];
    while (std::fread(hdr, 1, 12, stdin) == 12) {
        if (std::memcmp(hdr, "FRM1", 4)) {
            std::fprintf(stderr, "bad frame header\n");
            return 1;
        }
        uint32_t t;
        uint16_t w, h;
        std::memcpy(&t, hdr + 4, 4);
        std::memcpy(&w, hdr + 8, 2);
        std::memcpy(&h, hdr + 10, 2);
        frame.resize(size_t(w) * h * 2);
        if (std::fread(frame.data(), 1, frame.size(), stdin) != frame.size()) break;
        MotionSample ms;
        tm.process_yuyv(frame.data(), w, h, t, false, boxes, ms);
        feed(ms);
    }
    return 0;
}
