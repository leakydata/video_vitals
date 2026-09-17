#!/usr/bin/env python3
"""Compare replay output (idf/test/replay CSV) with ground-truth heart rate.

usage: evaluate.py replay.csv gt_times_s gt_hr   (as .npy pair via --ubfc/--mpu loaders)
"""
import argparse
import csv

import numpy as np


def load_gt(kind, path):
    if kind == "ubfc":
        _, hr, t = np.loadtxt(path)
        return t, hr
    if kind == "mpu":
        d = np.genfromtxt(path, delimiter=",", names=True)
        return d["Count"] / 60.0, d["HR"]
    raise ValueError(kind)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("replay_csv")
    ap.add_argument("kind", choices=["ubfc", "mpu"])
    ap.add_argument("gt")
    ap.add_argument("--offset", type=float, default=0, help="seconds skipped at the start of the video")
    ap.add_argument("--verbose", action="store_true")
    ap.add_argument("--ref-window", type=float, default=2.0, help="seconds of ground truth averaged as the reference (current HR)")
    a = ap.parse_args()
    gt_t, gt_hr = load_gt(a.kind, a.gt)
    rows = list(csv.DictReader(open(a.replay_csv)))
    err_locked, err_all, n_locked, n = [], [], 0, 0
    for r in rows:
        t = float(r["t"]) + a.offset
        m = (gt_t > t - a.ref_window) & (gt_t <= t)
        if not m.any() or float(r["t"]) < 12:
            continue
        ref = gt_hr[m].mean()
        n += 1
        est = float(r["bpm"])
        if est > 0:
            err_all.append(est - ref)
        if r["state"] == "2":
            n_locked += 1
            err_locked.append(est - ref)
        if a.verbose:
            print(f"t={t:6.1f} gt={ref:6.1f} est={est:6.1f} raw={float(r['raw']):6.1f} st={r['state']} q={r['q']} "
                  f"snr={r['snr']} coh={r['coh']} mot={r['motion']}")
    el = np.abs(err_locked)
    print(f"{a.replay_csv}: windows={n} locked={n_locked} ({100 * n_locked / max(n, 1):.0f}%)  "
          f"locked MAE={el.mean() if len(el) else float('nan'):.2f} bpm  "
          f"locked within 5 bpm={100 * (el <= 5).mean() if len(el) else float('nan'):.0f}%  "
          f"any-output MAE={np.abs(err_all).mean() if err_all else float('nan'):.2f}")


if __name__ == "__main__":
    main()
