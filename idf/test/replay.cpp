// Replays "S ..." sample lines (from tools/video_to_samples.py or a raw
// heartcam.py --record capture) through the firmware estimator, once per
// second of sample time, and prints CSV.
// Build: g++ -O2 -std=c++17 -I../main replay.cpp ../main/rppg.cpp -o replay
// Usage: replay samples.txt [--mono]
#include "rppg.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

using namespace rppg;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "usage: %s samples.txt [--mono]\n", argv[0]);
        return 1;
    }
    static Estimator est;
    static Buffer buf;
    static Sample tmp[kCap];
    if (argc > 2 && !std::strcmp(argv[2], "--mono")) est.set_mode(Mode::MONO);

    std::ifstream in(argv[1], std::ios::binary);
    std::string line;
    uint32_t next = 0, first = 0;
    bool have_first = false;
    std::printf("t,bpm,raw,snr,q,stab,coh,state,motion,rois\n");
    while (std::getline(in, line)) {
        // raw captures mix binary JPEG data with text: find the sample marker
        const size_t p = line.rfind("S ");
        if (p == std::string::npos) continue;
        std::istringstream ss(line.substr(p + 2));
        Sample s{};
        unsigned long t;
        int motion;
        if (!(ss >> t)) continue;
        bool ok = true;
        for (auto &r : s.roi) {
            unsigned n;
            if (!(ss >> r.r >> r.g >> r.b >> n)) { ok = false; break; }
            r.n = n;
        }
        if (!ok || !(ss >> motion)) continue;
        s.t_ms = t;
        s.motion = motion;
        if (!have_first) { first = next = t; have_first = true; }
        buf.push(s);
        if (t >= next + 1000) {
            next += 1000;
            const int n = buf.copy(tmp);
            const Result r = est.update(tmp, n, t);
            std::printf("%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%d\n", (t - first) / 1000.0, r.bpm, r.bpm_raw,
                        r.snr_db, r.quality, r.stability, r.coherence, r.state, r.motion, r.rois_used);
        }
    }
    return 0;
}
