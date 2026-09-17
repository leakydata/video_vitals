#!/usr/bin/env python3
"""heart_cam PC viewer for the ESP-IDF firmware.

The heart rate is computed on the ESP32. This viewer shows:
  * the camera image with the device's face box, landmarks and skin ROIs
  * an Eulerian colour-magnification view (pulse visible as a red flush)
  * the pulse waveform, computed here with POS from the device's own
    uncompressed per-frame ROI samples (for display only)
  * the device's heart rate, quality, and lock state

Keys: q quit | m magnification | a re-auto-expose | r rotate 180
      +/- exposure | M toggle mono(IR) mode | s snapshot
"""
import argparse
import re
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np
import serial
from scipy.signal import butter, sosfiltfilt

FS = 20.0


class Device(threading.Thread):
    """Reads the mixed text/binary stream from the camera."""

    def __init__(self, port, record=None):
        super().__init__(daemon=True)
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.frames = deque(maxlen=4)
        self.samples = deque(maxlen=600)  # (t, [[r,g,b,n] x3], motion)
        self.hr = {}
        self.hr_time = 0
        self.running = True
        self.rec = open(record, "wb") if record else None

    def send(self, cmd):
        self.ser.write((cmd + "\n").encode())

    def _line(self, line):
        if line.startswith("HR "):
            self.hr = dict(re.findall(r"(\w+)=([\w.\-]+)", line))
            self.hr_time = time.time()
        elif line.startswith("S "):
            v = line.split()
            if len(v) == 15:
                vals = list(map(float, v[1:14]))
                rois = [vals[1 + 4 * i : 5 + 4 * i] for i in range(3)]
                self.samples.append((vals[0] / 1000.0, rois, int(v[14])))
        elif line.startswith("#") and "minimize()" not in line:
            print("[dev]", line)

    def _text(self, data):
        for line in data.decode(errors="replace").splitlines():
            line = line.strip()
            if line:
                self._line(line)

    def run(self):
        buf = b""
        while self.running:
            try:
                chunk = self.ser.read(65536)
            except serial.SerialException:
                break
            if self.rec and chunk:
                self.rec.write(chunk)
            buf += chunk
            while True:
                j = buf.find(b"HCF2")
                if j < 0:
                    nl = buf.rfind(b"\n")
                    if nl >= 0:
                        self._text(buf[: nl + 1])
                        buf = buf[nl + 1 :]
                    if len(buf) > 8192:
                        buf = buf[-4:]
                    break
                if j > 0:
                    nl = buf.rfind(b"\n", 0, j)
                    if nl >= 0:
                        self._text(buf[: nl + 1])
                    buf = buf[j:]
                if len(buf) < 64:
                    break
                n, t = struct.unpack("<II", buf[4:12])
                if n > 200_000:
                    buf = buf[4:]
                    continue
                if len(buf) < 64 + n:
                    break
                meta = struct.unpack("<26h", buf[12:64])
                self.frames.append((t / 1000.0, meta, buf[64 : 64 + n]))
                buf = buf[64 + n :]

    def close(self):
        self.running = False
        self.join(timeout=1)
        try:
            self.send("x")
            self.send("w")
        finally:
            self.ser.close()
            if self.rec:
                self.rec.close()


def pos_pulse(samples, seconds=10.0):
    """POS on the device's forehead+cheek samples -> band-passed pulse."""
    ts = np.array([s[0] for s in samples])
    if len(ts) < 30 or ts[-1] - ts[0] < seconds:
        return None
    rgb = np.array([[sum(r[k] * r[3] for r in s[1]) / max(sum(r[3] for r in s[1]), 1) for k in range(3)]
                    for s in samples])
    valid = rgb.sum(axis=1) > 0
    if valid.sum() < 30:
        return None
    ts, rgb = ts[valid], rgb[valid]
    tu = np.arange(ts[-1] - seconds, ts[-1], 1 / FS)
    c = np.stack([np.interp(tu, ts, rgb[:, k]) for k in range(3)], axis=1)
    L = int(1.6 * FS)
    H = np.zeros(len(tu))
    for i in range(len(tu) - L + 1):
        cn = c[i : i + L] / c[i : i + L].mean(axis=0)
        s1 = cn[:, 1] - cn[:, 2]
        s2 = cn[:, 1] + cn[:, 2] - 2 * cn[:, 0]
        h = s1 + (s1.std() / (s2.std() + 1e-9)) * s2
        H[i : i + L] += h - h.mean()
    sos = butter(2, [0.7, 3.5], btype="band", fs=FS, output="sos")
    return sosfiltfilt(sos, H)


class Magnifier:
    """Eulerian colour magnification with an IIR temporal bandpass."""

    def __init__(self, levels=3, lo=0.8, hi=2.5, alpha=60.0):
        self.levels, self.lo, self.hi, self.alpha = levels, lo, hi, alpha
        self.low1 = self.low2 = None

    def __call__(self, frame, dt):
        small = frame.astype(np.float32) / 255
        for _ in range(self.levels):
            small = cv2.pyrDown(small)
        if self.low1 is None or self.low1.shape != small.shape:
            self.low1, self.low2 = small.copy(), small.copy()
        self.low1 += (1 - np.exp(-2 * np.pi * self.hi * dt)) * (small - self.low1)
        self.low2 += (1 - np.exp(-2 * np.pi * self.lo * dt)) * (small - self.low2)
        band = (self.low1 - self.low2) * self.alpha
        ycc = cv2.cvtColor(band, cv2.COLOR_BGR2YCrCb)
        ycc[..., 0] *= 0.2  # amplify colour, not brightness
        band = cv2.cvtColor(ycc, cv2.COLOR_YCrCb2BGR)
        up = cv2.resize(band, (frame.shape[1], frame.shape[0]))
        return (np.clip(frame.astype(np.float32) / 255 + up, 0, 1) * 255).astype(np.uint8)


def draw_overlay(img, meta):
    if meta[0] >= 0:
        cv2.rectangle(img, meta[0:2], meta[2:4], (255, 160, 0), 1)
    for i in range(5):
        if meta[4 + 2 * i] >= 0:
            cv2.circle(img, (meta[4 + 2 * i], meta[5 + 2 * i]), 2, (0, 0, 255), -1)
    for i in range(3):
        x, y, w, h = meta[14 + 4 * i : 18 + 4 * i]
        if x >= 0 and w > 0:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 1)


def draw_plot(img, x, y, w, h, data, color, label):
    cv2.rectangle(img, (x, y), (x + w, y + h), (35, 35, 35), -1)
    if data is not None and len(data) > 1:
        d = np.asarray(data, float)
        d = (d - d.min()) / (np.ptp(d) + 1e-9)
        pts = np.stack([np.linspace(x, x + w - 1, len(d)), y + h - 3 - d * (h - 6)], 1)
        cv2.polylines(img, [pts.astype(np.int32)], False, color, 1, cv2.LINE_AA)
    cv2.putText(img, label, (x + 5, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)


def fmt_status(hr):
    return (f"{hr.get('state', '?')}  q={hr.get('q', '?')} snr={hr.get('snr', '?')}dB "
            f"stab={hr.get('stab', '?')} coh={hr.get('coh', '?')} motion={hr.get('motion', '?')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--scale", type=float, default=2.0, help="display scale")
    ap.add_argument("--alpha", type=float, default=60.0, help="magnification factor")
    ap.add_argument("--headless", action="store_true", help="no window; print the device heart rate")
    ap.add_argument("--no-video", action="store_true", help="do not stream video (faster on-device fps)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds")
    ap.add_argument("--record", help="save the raw serial stream (video + samples + HR) to this file")
    args = ap.parse_args()

    dev = Device(args.port, args.record)
    dev.start()
    dev.send("v")
    if not args.no_video and not args.headless:
        dev.send("s")
    mag = Magnifier(alpha=args.alpha)
    magnify = True
    exposure = 300
    last_t = None
    last_print = 0
    view = None
    start = time.time()
    try:
        while not args.seconds or time.time() - start < args.seconds:
            if args.headless:
                time.sleep(0.2)
                if dev.hr_time > last_print:
                    last_print = dev.hr_time
                    h = dev.hr
                    print(f"bpm={h.get('bpm')} raw={h.get('raw')} {fmt_status(h)} face={h.get('face')} "
                          f"det_hit={h.get('det_hit')} fps={h.get('fps')}", flush=True)
                continue

            if dev.frames:
                t, meta, jpg = dev.frames.popleft()
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    dt = t - last_t if last_t and 0 < t - last_t < 1 else 0.1
                    last_t = t
                    left = frame.copy()
                    draw_overlay(left, meta)
                    right = mag(frame, dt) if magnify else frame
                    view = cv2.resize(np.hstack([left, right]), None, fx=args.scale, fy=args.scale)
            if view is None:
                view = np.zeros((int(240 * args.scale), int(640 * args.scale), 3), np.uint8)
                cv2.putText(view, "waiting for video..." if not args.no_video else "video off",
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

            W = view.shape[1]
            panel = np.zeros((190, W, 3), np.uint8)
            pulse = pos_pulse(list(dev.samples))
            draw_plot(panel, 5, 5, W - 10, 95, pulse, (90, 90, 255), "pulse (POS on device samples, last 10 s)")
            h = dev.hr
            state = h.get("state", "")
            bpm = float(h.get("bpm", 0) or 0)
            col = {"locked": (80, 255, 80), "acquiring": (0, 200, 255)}.get(state, (120, 120, 120))
            big = f"{bpm:5.1f} bpm" if bpm > 0 else "--.- bpm"
            cv2.putText(panel, big, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1.5, col, 3)
            cv2.putText(panel, fmt_status(h), (300, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
            cv2.putText(panel, f"face={h.get('face')} det_hit={h.get('det_hit')} rois={h.get('rois')} "
                               f"cam {h.get('fps')} fps  det {h.get('det_ms')} ms  mode={h.get('mode')}  "
                               f"mag={'on' if magnify else 'off'}",
                        (300, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)
            try:
                q = float(h.get("q", 0))
            except ValueError:
                q = 0
            cv2.rectangle(panel, (300, 162), (300 + int(300 * q), 176), col, -1)
            cv2.rectangle(panel, (300, 162), (600, 176), (90, 90, 90), 1)
            if state == "locked" and pulse is not None and pulse[-1] > 0:
                cv2.circle(panel, (W - 30, 140), 14, (60, 60, 255), -1)
            cv2.imshow("heart_cam", np.vstack([view, panel]))

            key = cv2.waitKey(15) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("m"):
                magnify = not magnify
            elif key == ord("a"):
                dev.send("a")
            elif key == ord("r"):
                dev.send("r")
            elif key == ord("M"):
                dev.send("m1" if h.get("mode") != "mono" else "m0")
            elif key in (ord("+"), ord("="), ord("-")):
                exposure = int(np.clip(exposure + (50 if key != ord("-") else -50), 0, 1200))
                dev.send(f"e{exposure}")
            elif key == ord("s"):
                fn = time.strftime("snapshot_%Y%m%d_%H%M%S.png")
                cv2.imwrite(fn, np.vstack([view, panel]))
                print("saved", fn)
    except KeyboardInterrupt:
        pass
    finally:
        dev.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
