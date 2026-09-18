#!/usr/bin/env python3
"""Plot a heart_cam recording: heart rate, breathing, quality and motion.

Reads the raw serial capture written by `heartcam.py --record` (mixed text and
JPEG) and draws what the device reported over time, so a session can be judged
at a glance: when it was locked, what the quality was, and how much of each
window was masked for movement. Pacer marks written by the viewer are shown as
a reference line.

usage: session_report.py recordings/x.bin [-o report.png] [--csv out.csv]
"""
import argparse
import csv
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

KV = re.compile(rb"(\w+)=([\w.\-]+)")
STATE = {"no_signal": 0, "acquiring": 1, "locked": 2}


def _lines(path, chunk=1 << 20):
    """Yield newline-terminated chunks of a file without loading it all.

    A telemetry line is short, so anything absurdly long is a JPEG payload that
    happens to contain no newline: it is truncated rather than buffered, which
    costs nothing here because such a chunk never holds an HR/RR line.
    """
    buf = b""
    with open(path, "rb") as fh:
        while True:
            block = fh.read(chunk)
            if not block:
                break
            buf += block
            parts = buf.split(b"\n")
            buf = parts.pop()
            if len(buf) > 1 << 22:      # no newline in 4 MB: binary, not telemetry
                buf = buf[-4096:]
            for part in parts:
                yield part
    if buf:
        yield buf


def parse(path):
    hr, rr, pacer = [], [], []
    # annotations live beside the recording: the recording itself is a byte-exact
    # copy of the serial stream and nothing may be written into it
    marks_path = path + ".marks.txt"
    if os.path.exists(marks_path):
        for line in open(marks_path):
            m = re.match(r"([\d.]+)\s+pacer (on|off) rate=(\d+)", line)
            if m:
                pacer.append((float(m.group(1)), m.group(2) == "on", float(m.group(3))))
    # Stream the file rather than reading it whole: a night's recording with
    # video is hundreds of megabytes, and splitting that in memory needs several
    # times its size again. Binary JPEG payloads contain stray newlines, so a
    # "line" here is only a candidate; the HR/RR search below rejects the rest.
    for line in _lines(path):
        p = max(line.rfind(b"HR "), line.rfind(b"RR "))
        if p < 0:
            continue
        kind = line[p:p + 2]
        kv = {k.decode(): v.decode() for k, v in KV.findall(line[p:])}
        if "state" not in kv:
            continue

        def f(key):
            try:
                return float(kv.get(key, "nan"))
            except ValueError:
                return float("nan")

        row = dict(state=STATE.get(kv["state"], 0), q=f("q"), snr=f("snr"), motion=f("motion"))
        if kind == b"HR":
            row.update(rate=f("bpm"), raw=f("raw"), face=f("face"))
            hr.append(row)
        else:
            row.update(rate=f("br"), raw=f("raw"), ch=f("ch"))
            rr.append(row)
    # the device prints one HR and one RR per second; use the sample index as time
    for seq in (hr, rr):
        for i, r in enumerate(seq):
            r["t"] = i
    return hr, rr, pacer


def series(rows, key):
    return np.array([r.get(key, np.nan) for r in rows], float)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--csv")
    a = ap.parse_args()
    hr, rr, pacer = parse(a.recording)
    if not hr and not rr:
        print("no HR/RR lines found (was the recording made with heartcam.py?)")
        return 1
    out = a.out or a.recording.rsplit(".", 1)[0] + ".png"

    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    for rows, name, col, lim in ((hr, "heart rate (bpm)", "tab:red", (40, 180)),
                                 (rr, "breathing (/min)", "tab:blue", (4, 46))):
        if not rows:
            continue
        ax = axs[0] if "heart" in name else axs[1]
        t, rate, raw, st = series(rows, "t"), series(rows, "rate"), series(rows, "raw"), series(rows, "state")
        ax.plot(t, np.where(raw > 0, raw, np.nan), color=col, alpha=0.3, lw=0.8, label="raw peak")
        ax.plot(t, np.where((rate > 0) & (st == 2), rate, np.nan), color=col, lw=2, label="locked")
        ax.plot(t, np.where((rate > 0) & (st < 2), rate, np.nan), color=col, lw=1, ls=":", label="acquiring")
        ax.set_ylabel(name)
        ax.set_ylim(*lim)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8)
        locked = (st == 2).mean() * 100 if len(st) else 0
        vals = rate[(st == 2) & (rate > 0)]
        ax.set_title(f"{name}: locked {locked:.0f}% of the session"
                     + (f", median {np.median(vals):.1f}" if len(vals) else ""), fontsize=10)
    for t, on, rate in pacer:
        if on:
            axs[1].axhline(rate, color="orange", ls="--", lw=1)
            axs[1].annotate(f"pacer {rate:.0f}", (0, rate), color="orange", fontsize=8)

    for rows, name, col in ((hr, "heart", "tab:red"), (rr, "breathing", "tab:blue")):
        if not rows:
            continue
        axs[2].plot(series(rows, "t"), series(rows, "q"), color=col, lw=1, label=f"{name} quality")
        axs[2].plot(series(rows, "t"), series(rows, "motion"), color=col, lw=1, ls="--", alpha=0.6,
                    label=f"{name} masked for motion")
    axs[2].set_ylabel("quality / motion")
    axs[2].set_xlabel("time (s)")
    axs[2].set_ylim(0, 1.05)
    axs[2].grid(alpha=0.3)
    axs[2].legend(loc="upper right", fontsize=8, ncol=2)
    fig.suptitle(a.recording)
    fig.tight_layout()
    fig.savefig(out, dpi=90)
    print("wrote", out)

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "hr", "hr_state", "hr_q", "hr_motion", "br", "br_state", "br_q", "br_motion"])
            for i in range(max(len(hr), len(rr))):
                h = hr[i] if i < len(hr) else {}
                r = rr[i] if i < len(rr) else {}
                w.writerow([i, h.get("rate", ""), h.get("state", ""), h.get("q", ""), h.get("motion", ""),
                            r.get("rate", ""), r.get("state", ""), r.get("q", ""), r.get("motion", "")])
        print("wrote", a.csv)
    return 0


if __name__ == "__main__":
    sys.exit(main())
