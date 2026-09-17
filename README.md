# heart_cam

Contactless heart rate (rPPG) and breathing rate on a XIAO ESP32-S3 Sense
(OV3660). Both are computed **on the ESP32**; the PC viewer is optional.

## Layout
- `idf/` — ESP-IDF firmware (current)
  - `main/main.cpp` — camera (RGB565, uncompressed), ESP-DL face detection
    (ESPDet-Pico + MNP landmarks), forehead/cheek ROIs, streaming, commands
  - `main/rppg.cpp` — estimator: POS (or mono/IR) per ROI → Butterworth
    bandpass → spectra → SNR-weighted fusion → harmonic/octave check →
    quality (SNR × peak stability × cross-ROI coherence) → gated Kalman tracker
  - `main/motion.cpp` — breathing motion: 4x4 tile grid, row/column
    luminance profiles, 1-D Lucas-Kanade sub-pixel shifts, local/global
    movement flags
  - `main/resp.cpp` — breathing estimator: per-tile resampling, per-segment
    detrend, motion masks, bandpass, FFT, best-channel fusion, octave check,
    quality (SNR x stability x channel agreement)
  - `main/tracker.hpp` — Kalman rate tracker shared by both estimators
  - `test/test_rppg.cpp` — heart-rate scenario tests
  - `test/test_resp.cpp` — breathing tests on rendered frames
  - `test/replay.cpp`, `test/resp_replay.cpp` — run the estimators on
    recordings or converted videos
- `tools/video_to_samples.py` — converts a face video into device-equivalent samples
- `tools/evaluate.py` — compares replay output with dataset ground truth
- `tools/video_frames.py`, `tools/eval_air400.py` — breathing evaluation on AIR-400
- `tools/radar_ref.py` — live comparison against a Seeed MR60BHA2 radar (independent
  breathing/heart reference); finds both boards by probing the USB serial ports
- `tools/session_report.py` — plots a recording: rates, quality, motion, pacer marks
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
`+/-` exposure, `M` mono(IR) mode, `b` adult/infant breathing band,
`p` breathing pacer (reference: breathe with the circle), `[ ]` pacer rate,
`s` snapshot. The viewer finds the board by USB id and reconnects by itself.

Serial commands (one per line): `s`/`x` stream, `v`/`w` per-frame samples,
`a` auto-expose then lock, `e<n>` exposure, `g<n>` gain, `q<n>` JPEG quality,
`m0`/`m1` RGB/mono, `b0`/`b1` adult/infant breathing band, `r` rotate, `i` info.
Output: `HR ...` and `RR ...` once per second; with `v`, per-frame `S` (ROI
colour) and `M` (tile displacement) lines.

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
    idf/test/resp_replay --motion recordings/x.bin

    # breathing
    g++ -O2 -std=c++17 -I../main test_resp.cpp ../main/motion.cpp ../main/resp.cpp -o test_resp && ./test_resp
    g++ -O2 -std=c++17 -I../main resp_replay.cpp ../main/motion.cpp ../main/resp.cpp -o resp_replay

Current results:
- Heart rate: synthetic 80/80; UBFC subject 1 (device-equivalent, 11 fps):
  locked 100%, MAE 1.7 bpm against the current HR; matched a KardiaMobile 6L
  live. (MPU-rPPG sample unused: its PPG and HR columns disagree.)
- Breathing: synthetic 36/36 (rendered frames: 0.15 px chest motion, 6-45/min,
  infant band, rate changes, flicker, local and whole-scene movement, no
  false locks). AIR-400 infant clips are **not** solved: at 320x240 the
  compressed night-vision videos show no usable chest motion in any tile, and
  the pipeline locked at half the annotated rate on them.

## Getting a good reading (breathing)
- The camera must be **fixed** (not hand-held, not on a desk that gets bumped):
  breathing moves the chest by well under a pixel.
- Chest/shoulders visible, not covered by moving arms. Hands moving in one
  area are tolerated; movement of the whole scene pauses the estimate.
- A 30 s window: the first rate appears ~30-40 s after sitting still.

## Getting a good reading
- Face 30–60 cm away, well lit, **still** (rest the module on something).
- Steady light: no flicker, no screen light changing on the face.
- Exposure and white balance lock at boot. If the lighting changes, press `a`.
- The rate is reported once the tracker is `locked` (needs ~15 s).

## Roadmap
- Breathing: validate against the MR60BHA2 radar; a face-anchored chest ROI;
  make the infant videos work (higher resolution crop around the subject)
- IR / night mode: remove IR-cut filter + IR LEDs, mono pipeline with a
  background reference ROI to cancel illumination drift
- Sleep / baby monitor: PDM microphone audio + WiFi streaming
