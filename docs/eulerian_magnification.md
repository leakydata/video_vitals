# Eulerian video magnification on the ESP32-S3

**Status: proved, then parked.** It works on the board and is off by default. The
firmware keeps it (`idf/main/evm.cpp`, ~120 lines) because it costs nothing when
idle, and the vitals never depend on it.

This was the project's opening question: can Eulerian magnification run on a $15
microcontroller rather than a workstation? It can.

## What it does

Wu et al., *Eulerian Video Magnification for Revealing Subtle Changes in the
World* (SIGGRAPH 2012): band-pass every pixel over time, amplify that band, add
it back. Slow movement appears as brightness change at edges, so amplifying the
breathing band exaggerates the movement itself; amplifying the pulse band
exaggerates the blood flush in skin.

## Measured

| | |
|---|---|
| Magnification cost | **28 ms per QVGA frame** on the ESP32-S3 (≈35 fps of capacity) |
| First attempt | 528 ms per frame — 19× slower |
| Effect on vitals | none: the vitals are measured from the untouched frame |
| Verified against the paper's own clips | `face.mp4` colour 3.1×; `baby.mp4` motion 16×, magnified torso ≈31 breaths/min |

The preview streams at only 2–3 fps, but that is JPEG encoding and USB, not the
magnification: face detection occupies a core for ~250 ms at a time and the
encoder competes with it.

## How it got to 28 ms

- Temporal filtering at a coarse pyramid level (40×30), so only the add-back
  touches every pixel.
- One sequential pass over the frame for the downscale — revisiting rows per
  output pixel thrashed PSRAM.
- Interpolation weights precomputed into tables; the inner loop is a lookup, an
  add and a clamp. No divisions, no `floor`, per pixel.
- The amplified band is capped at ±60 levels. The linear method assumes small
  changes, so a head turn would otherwise blow the picture out. Subtle motion is
  exaggerated; gross motion is not.

## Using it

    E1   magnify the preview in the breathing band (motion)
    E2   magnify the preview in the pulse band (colour)
    E0   off (default)

In the viewer, `E` cycles the same thing, and `--device-magnify motion` starts
with it on. `m` cycles the viewer's *own* magnified panel, which is separate and
runs on the PC.

A side-by-side recording is in `recordings/evm_ondevice.mp4` (plain against
magnified, captured in sequence rather than simultaneously — the board runs one
mode at a time).

## If picked up again

- The preview rate is the thing to fix, not the magnifier: move JPEG encoding off
  the core the detector monopolises, and ease detection while streaming.
- Phase-based magnification (Wadhwa et al. 2013) is far better at motion but far
  more expensive — unlikely to fit at any useful frame rate.
- Magnifying only inside the chest box would cut the cost several times over and
  is where the interesting motion is anyway.
