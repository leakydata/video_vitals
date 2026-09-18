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
    # The device anchors its chest box to a detected face, and there is no face
    # to find in a cot seen from above -- the detector scores 0 on these clips.
    # This is the region a parent would point it at instead.
    ap.add_argument("--chest", help="watch region in device frame coords, x,y,w,h")
    a = ap.parse_args()
    all_err = []
    all_wrong = 0
    all_raw = []
    for clip in a.clips:
        resp = np.array(h5py.File(clip.replace(".mp4", ".hdf5"))["respiration"])
        fs = len(resp) / 60.0
        peaks, _ = find_peaks(resp, height=0.05, distance=int(fs * 0.8))
        peak_t = peaks / fs
        frames = subprocess.Popen([sys.executable, os.path.join(HERE, "video_frames.py"), clip], stdout=subprocess.PIPE)
        cmd = [REPLAY] + (["--infant"] if a.infant else []) + (["--chest", a.chest] if a.chest else [])
        out = subprocess.run(cmd, stdin=frames.stdout, capture_output=True, text=True).stdout
        frames.wait()
        errs, locked, n = [], 0, 0
        raws = []   # the peak it found, whether or not it was confident enough to report it
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
            if ref == ref and float(r["raw"]) > 0:
                raws.append(float(r["raw"]) - ref)
            if a.verbose:
                print(f"  t={t:4.0f} ref={ref:5.1f} br={float(r['br']):5.1f} raw={float(r['raw']):5.1f} "
                      f"snr={r['snr']} q={r['q']} stab={r['stab']} agr={r['agree']} st={r['state']} best={r['best']}")
        e = np.abs(errs)
        all_err += list(e)
        # The number that matters for a monitor is not the average error but how
        # often it states a rate that is simply wrong: a confident wrong reading
        # is worse than no reading, because it is the one that gets believed.
        rw = np.abs(raws)
        all_raw += list(rw)
        wrong = int((e > 5).sum())
        all_wrong += wrong
        print(f"{os.path.basename(clip)}: annotated {len(peaks)} breaths/60 s; windows={n} locked={locked} "
              f"MAE={e.mean() if len(e) else float('nan'):.2f} /min  within 2/min={100 * (e <= 2).mean() if len(e) else float('nan'):.0f}%"
              f"  confidently wrong (>5/min)={wrong}\n"
              f"    peak found (regardless of confidence): MAE={rw.mean() if len(rw) else float('nan'):.2f} /min, "
              f"within 2/min={100 * (rw <= 2).mean() if len(rw) else float('nan'):.0f}%")
    if all_raw:
        r = np.array(all_raw)
        print(f"ALL: peak found MAE={r.mean():.2f} /min, within 2/min={100 * (r <= 2).mean():.0f}% "
              f"over {len(r)} windows")
    if all_err:
        print(f"ALL: locked MAE={np.mean(all_err):.2f} /min over {len(all_err)} windows; "
              f"confidently wrong in {all_wrong}")
    else:
        print(f"ALL: never locked; confidently wrong in {all_wrong}")
    return 1 if all_wrong else 0


if __name__ == "__main__":
    sys.exit(main())
