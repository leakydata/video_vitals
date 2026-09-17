# heart_cam

Contactless heart rate (rPPG) on a XIAO ESP32-S3 Sense (OV3660). The heart
rate is computed **on the ESP32**; the PC viewer is optional.

## Layout
- `idf/` — ESP-IDF firmware (current)
  - `main/main.cpp` — camera (RGB565, uncompressed), ESP-DL face detection
    (ESPDet-Pico + MNP landmarks), forehead/cheek ROIs, streaming, commands
  - `main/rppg.cpp` — estimator: POS (or mono/IR) per ROI → Butterworth
    bandpass → spectra → SNR-weighted fusion → harmonic/octave check →
    quality (SNR × peak stability × cross-ROI coherence) → gated Kalman tracker
  - `test/test_rppg.cpp` — synthetic scenario tests (70 cases)
  - `test/replay.cpp` — runs the estimator on recorded/converted samples
- `tools/video_to_samples.py` — converts a face video into device-equivalent samples
- `tools/evaluate.py` — compares replay output with dataset ground truth
- `heartcam.py` — PC viewer (video + overlays, Eulerian magnification, pulse plot)
- `data/` — public test videos (see `data/SOURCES.md`)
- `firmware/heart_cam/` — first Arduino prototype (superseded)

## Build & flash
    . ~/esp-idf/export.sh
    cd idf && idf.py build && idf.py -p /dev/ttyACM0 flash

## Run
    .venv/bin/python heartcam.py                          # viewer
    .venv/bin/python heartcam.py --record recordings/x.bin
    .venv/bin/python heartcam.py --headless               # text only

The board also works on its own: the user LED blinks with the pulse when locked.
Viewer keys: `q` quit, `m` magnification, `a` re-auto-expose, `r` rotate 180,
`+/-` exposure, `M` mono(IR) mode, `s` snapshot.

Serial commands (one per line): `s`/`x` stream, `v`/`w` per-frame samples,
`a` auto-expose then lock, `e<n>` exposure, `g<n>` gain, `q<n>` JPEG quality,
`m0`/`m1` RGB/mono, `r` rotate, `i` info.

## Test
    cd idf/test
    g++ -O2 -std=c++17 -I../main test_rppg.cpp ../main/rppg.cpp -o test_rppg && ./test_rppg
    g++ -O2 -std=c++17 -I../main replay.cpp ../main/rppg.cpp -o replay

    # real video with ground truth (device-equivalent 320x240 RGB565 @ 11 fps)
    .venv/bin/python tools/video_to_samples.py data/ubfc_rppg_dataset2_subject1/vid_first27s_ffv1.mkv /tmp/u.txt
    idf/test/replay /tmp/u.txt > /tmp/u.csv
    .venv/bin/python tools/evaluate.py /tmp/u.csv ubfc data/ubfc_rppg_dataset2_subject1/ground_truth.txt

    # a live recording
    idf/test/replay recordings/x.bin

Current results: synthetic 70/70; UBFC subject 1 at 11 fps: locked 88% of
windows, MAE 0.92 bpm. (The MPU-rPPG sample is not used: its PPG and HR
columns disagree and the lossy video carries almost no pulse.)

## Getting a good reading
- Face 30–60 cm away, well lit, **still** (rest the module on something).
- Steady light: no flicker, no screen light changing on the face.
- Exposure and white balance lock at boot. If the lighting changes, press `a`.
- The rate is reported once the tracker is `locked` (needs ~15 s).

## Roadmap
- Breathing rate (chest/face motion, 0.1–0.5 Hz)
- IR / night mode: remove IR-cut filter + IR LEDs, mono pipeline with a
  background reference ROI to cancel illumination drift
- Sleep / baby monitor: PDM microphone audio + WiFi streaming
