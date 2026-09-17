#!/usr/bin/env python3
"""Evaluate the firmware breathing pipeline on AIR-400 infant clips.

For each clip: stream frames through idf/test/resp_replay, and compare each
second's estimate with the annotated breath count in the same 30 s window.
usage: eval_air400.py [--infant] [--verbose] clip.mp4 ...
"""
import argparse
import csv
import io
import os
import subprocess
import sys

import h5py
import numpy as np
from scipy.signal import find_peaks

HERE = os.path.dirname(os.path.abspath(__file__))
REPLAY = os.path.join(HERE, "..", "idf", "test", "resp_replay")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("clips", nargs="+")
    ap.add_argument("--infant", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()
    all_err = []
    for clip in a.clips:
        resp = np.array(h5py.File(clip.replace(".mp4", ".hdf5"))["respiration"])
        fs = len(resp) / 60.0
        peaks, _ = find_peaks(resp, height=0.05, distance=int(fs * 0.8))
        peak_t = peaks / fs
        frames = subprocess.Popen([sys.executable, os.path.join(HERE, "video_frames.py"), clip], stdout=subprocess.PIPE)
        out = subprocess.run([REPLAY] + (["--infant"] if a.infant else []), stdin=frames.stdout,
                             capture_output=True, text=True).stdout
        frames.wait()
        errs, locked, n = [], 0, 0
        for r in csv.DictReader(io.StringIO(out)):
            t = float(r["t"])
            if t < 30:
                continue
            # breaths per minute from the annotated breath times in the window
            inw = peak_t[(peak_t > t - 30) & (peak_t <= t)]
            ref = (len(inw) - 1) / (inw[-1] - inw[0]) * 60 if len(inw) >= 3 else float("nan")
            n += 1
            if r["state"] == "2":
                locked += 1
                errs.append(float(r["br"]) - ref)
            if a.verbose:
                print(f"  t={t:4.0f} ref={ref:5.1f} br={float(r['br']):5.1f} raw={float(r['raw']):5.1f} "
                      f"snr={r['snr']} q={r['q']} stab={r['stab']} agr={r['agree']} st={r['state']} best={r['best']}")
        e = np.abs(errs)
        all_err += list(e)
        print(f"{os.path.basename(clip)}: annotated {len(peaks)} breaths/60 s; windows={n} locked={locked} "
              f"MAE={e.mean() if len(e) else float('nan'):.2f} /min  within 2/min={100 * (e <= 2).mean() if len(e) else float('nan'):.0f}%")
    if all_err:
        print(f"ALL: locked MAE={np.mean(all_err):.2f} /min over {len(all_err)} windows")


if __name__ == "__main__":
    main()
