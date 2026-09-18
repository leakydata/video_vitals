#!/usr/bin/env python3
"""Measure the device's noise floor, to compare cameras, lenses or settings.

Point the camera at a STATIC, textured scene (a book, a patterned cushion — not
a person, not a blank wall) and leave it untouched for the run. What is reported:

  frame rate         frames per second the sensor delivers
  tracker noise      median frame-to-frame jitter of the quiet tile channels,
                     in pixels: the floor that breathing (<1 px) must beat
  ROI colour noise   high-frequency noise of the skin-ROI means, in levels and
                     as a fraction: the floor the pulse (~0.3%) must beat
  exposure/gain      what the sensor settled on, since the comparison is only
                     fair at similar settings

usage: noise_bench.py [--seconds 60] [--label "OV3660 stock lens"]
"""
import argparse
import re
import sys
import time

import numpy as np
import serial
import serial.tools.list_ports


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == 0x303A:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--label", default="")
    a = ap.parse_args()
    port = find_port()
    if not port:
        print("no device found")
        return 1
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.0)          # the port has just enumerated; commands sent now can be lost
    s.reset_input_buffer()
    print(f"recording {a.seconds:.0f} s from {port} — keep the camera and scene still...")
    buf, t0, asked = b"", time.time(), 0
    while time.time() - t0 < a.seconds:
        buf += s.read(65536)
        # keep asking for per-frame samples until they actually start arriving
        if b"\nM " not in buf[-20000:] and time.time() - t0 > asked:
            s.write(b"v\n")
            asked += 2
    s.write(b"w\n")
    s.write(b"i\n")
    time.sleep(0.4)
    buf += s.read(65536)
    s.close()

    M, S, info, fps = [], [], "", []
    for line in buf.split(b"\n"):
        p = line.rfind(b"M ")
        if p >= 0:
            v = line[p + 2:].split()
            if len(v) == 41:
                try:
                    M.append([float(x) for x in v])
                except ValueError:
                    pass
        p = line.rfind(b"S ")
        if p >= 0:
            v = line[p + 2:].split()
            if len(v) == 14:
                try:
                    S.append([float(x) for x in v])
                except ValueError:
                    pass
        if line.startswith(b"# heart_cam"):
            info = line.decode(errors="replace")
        m = re.search(rb"\bfps=([\d.]+)", line)
        if m and line.startswith(b"HR "):
            fps.append(float(m.group(1)))

    print(f"\n=== {a.label or 'run'} ===")
    if fps:
        print(f"frame rate        {max(fps):.1f} fps")
    if M:
        M = np.array(M)
        d = M[:, 6:]                       # channel displacements (px)
        valid = M[:, 1].astype(np.int64)   # bit per channel: tracked this frame
        jump = M[:, 3].astype(np.int64)
        step = np.abs(np.diff(d, axis=0))
        med, cover = [], []
        for c in range(d.shape[1]):
            # only consecutive frames where the channel was tracked and not
            # flagged: an untracked channel reads as perfectly quiet otherwise
            ok = np.array([bool((valid[i] >> c) & 1) and bool((valid[i + 1] >> c) & 1)
                           and not ((jump[i] >> c) & 1) and not ((jump[i + 1] >> c) & 1)
                           for i in range(len(d) - 1)])
            cover.append(ok.mean())
            med.append(np.median(step[ok, c]) if ok.sum() > 20 else np.nan)
        med = np.array(med)
        good = med[~np.isnan(med)]
        quiet = np.sort(good)[: max(1, len(good) // 2)]
        print(f"tracker noise     {np.median(quiet):.4f} px per frame (quietest half of "
              f"{len(good)}/{d.shape[1]} tracked channels)   best {quiet.min():.4f}")
        print(f"tracking coverage {100 * np.mean(cover):.0f}% of channel-frames")
    else:
        print("tracker noise     no motion samples (is the firmware current?)")
    if S:
        S = np.array(S)
        for i, name in enumerate(("forehead", "cheek L", "cheek R")):
            r, g, b, n = S[:, 1 + 4 * i], S[:, 2 + 4 * i], S[:, 3 + 4 * i], S[:, 4 + 4 * i]
            ok = n > 0
            if ok.sum() < 20:
                continue
            # noise = high-frequency part of the green mean (successive differences)
            gg = g[ok]
            noise = np.diff(gg).std() / np.sqrt(2)
            print(f"ROI colour noise  {name}: {noise:.3f} levels on a mean of {gg.mean():.1f}"
                  f"  ({100 * noise / max(gg.mean(), 1):.3f}%, {int(n[ok].mean())} px)")
    else:
        print("ROI colour noise  no face ROIs (point it at a face for this part)")
    if info:
        m = dict(re.findall(r"(\w+)=([\w.x]+)", info))
        print(f"sensor            PID {m.get('PID','?')}  exposure {m.get('exposure','?')}  "
              f"gain {m.get('gain','?')}  scene brightness {m.get('bright','?')}/255")
        print("(comparisons are only fair at similar brightness/exposure)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
