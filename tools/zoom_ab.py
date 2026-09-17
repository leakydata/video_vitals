#!/usr/bin/env python3
"""A/B the sensor-window zoom: same subject, same light, minutes apart.

Runs the device wide, then zoomed, then wide again (so any drift in lighting or
posture shows up as a difference between the two wide phases, not as a result).
Reports what each phase achieved.

Sit as still as you reasonably can for the whole run, facing the camera.

usage: zoom_ab.py [--phase 120]
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


def collect(s, seconds, label):
    print(f"  {label}: {seconds:.0f} s ...", flush=True)
    buf, t0 = b"", time.time()
    while time.time() - t0 < seconds:
        buf += s.read(65536)
    hr, rr = [], []
    for line in buf.split(b"\n"):
        p = line.rfind(b"HR ")
        if p >= 0:
            kv = dict(re.findall(rb"(\w+)=([\w.\-]+)", line[p:]))
            if b"state" in kv:
                hr.append(kv)
        p = line.rfind(b"RR ")
        if p >= 0:
            kv = dict(re.findall(rb"(\w+)=([\w.\-]+)", line[p:]))
            if b"state" in kv:
                rr.append(kv)
    return hr, rr


def num(kv, key, default=0.0):
    try:
        return float(kv.get(key.encode(), default))
    except (ValueError, TypeError):
        return default


def summarise(label, hr, rr):
    if not hr:
        print(f"{label:12s} no data")
        return
    locked = [k for k in hr if k.get(b"state") == b"locked"]
    bpm = [num(k, "bpm") for k in locked if num(k, "bpm") > 0]
    print(f"{label:12s} heart: locked {100 * len(locked) / len(hr):3.0f}% of {len(hr)} windows"
          f"   quality {np.mean([num(k, 'q') for k in hr]):.2f}"
          f"   snr {np.mean([num(k, 'snr') for k in hr]):+5.1f} dB"
          f"   coherence {np.mean([num(k, 'coh') for k in hr]):.2f}"
          f"   motion {np.mean([num(k, 'motion') for k in hr]):.2f}"
          f"   {np.mean([num(k, 'fps') for k in hr]):.1f} fps"
          + (f"   {np.median(bpm):.1f} bpm" if bpm else ""))
    if rr:
        rlocked = [k for k in rr if k.get(b"state") == b"locked"]
        rb = [num(k, "br") for k in rlocked if num(k, "br") > 0]
        print(f"{'':12s} breathing: locked {100 * len(rlocked) / len(rr):3.0f}%"
              f"   quality {np.mean([num(k, 'q') for k in rr]):.2f}"
              + (f"   {np.median(rb):.1f} /min" if rb else ""))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", type=float, default=120, help="seconds per phase")
    a = ap.parse_args()
    port = find_port()
    if not port:
        print("no device found (close the viewer first)")
        return 1
    s = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(1.0)
    s.reset_input_buffer()
    print("Sit still and face the camera for the whole run.\n")
    phases = []
    for label, cmd in (("wide (1)", b"z0\n"), ("zoomed", b"z1\n"), ("wide (2)", b"z0\n")):
        s.write(cmd)
        time.sleep(2)
        s.reset_input_buffer()
        # the first 35 s after a change are the estimators refilling their windows
        collect(s, 35, f"{label} settling")
        phases.append((label, *collect(s, a.phase, label)))
    s.write(b"z0\n")
    s.close()
    print()
    for label, hr, rr in phases:
        summarise(label, hr, rr)
    print("\nCompare 'zoomed' against the two 'wide' phases: if the wide phases differ from each\n"
          "other as much as they differ from zoomed, the run was not conclusive.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
