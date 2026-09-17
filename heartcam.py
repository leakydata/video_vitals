#!/usr/bin/env python3
"""heart_cam PC viewer for the ESP-IDF firmware.

The heart rate is computed on the ESP32. This viewer shows:
  * the camera image with the device's face box, landmarks and skin ROIs
  * an Eulerian colour-magnification view (pulse visible as a red flush)
  * the pulse waveform, computed here with POS from the device's own
    uncompressed per-frame ROI samples (for display only)
  * the device's heart rate, quality, and lock state
  * the device's breathing rate, every region it is fused from (the best one
    highlighted), and that region's displacement waveform

Keys: q quit | m magnification | a re-auto-expose | r rotate 180
      +/- exposure | M toggle mono(IR) mode | b adult/infant breathing band
      p breathing pacer on/off, [ ] pacer rate | s snapshot

The pacer rate can also be set from outside while the viewer runs, by writing a
number (or "off") to the file given by --pacer-file, e.g.
    echo 20 > /tmp/heartcam_pacer
Every change is timestamped into <recording>.marks.txt for scoring afterwards.

The pacer is a reference for testing: breathe in while the circle grows and
out while it shrinks, and compare the device's rate with the pacer's.
"""
import argparse
import os
import re
import struct
import threading
import time
from collections import deque

import cv2
import numpy as np
import serial
import serial.tools.list_ports
from scipy.signal import butter, sosfiltfilt

FS = 20.0
N_CHAN = 36             # 16 tiles x 2 axes + chest and head boxes (vertical, horizontal)
META_END = 12 + 30 * 2  # magic + length + timestamp + metadata


ESPRESSIF_VID = 0x303A


def find_port():
    """First Espressif USB serial device (the port name changes across replugs)."""
    for p in serial.tools.list_ports.comports():
        if p.vid == ESPRESSIF_VID:
            return p.device
    return None


class Device(threading.Thread):
    """Reads the mixed text/binary stream from the camera."""

    def __init__(self, port, record=None):
        super().__init__(daemon=True)
        self.port = port
        self.ser = None
        self.startup = []  # commands re-sent after every (re)connect
        self.connected = False
        self.frames = deque(maxlen=4)
        self.samples = deque(maxlen=600)  # (t, [[r,g,b,n] x3], motion)
        self.hr = {}
        self.hr_time = 0
        self.rr = {}
        self.motion = deque(maxlen=600)  # (t, displacements[32], valid mask, gross)
        self.running = True
        self.rec = open(record, "wb") if record else None
        self.marks = open(record + ".marks.txt", "w") if record else None

    def mark(self, text):
        """Note an annotation beside the recording, never inside it: the
        recording is a byte-exact copy of the serial stream, so text inserted
        between chunks would land inside a JPEG or split a telemetry line."""
        if self.marks:
            self.marks.write(f"{time.time():.3f} {text}\n")
            self.marks.flush()

    def send(self, cmd, startup=False):
        if startup:
            self.startup.append(cmd)
        try:
            if self.ser:
                self.ser.write((cmd + "\n").encode())
        except (serial.SerialException, OSError):
            pass

    def _connect(self):
        """Open the port, retrying until it appears (board unplugged or rebooting)."""
        while self.running:
            port = find_port() if self.port == "auto" else self.port
            try:
                if not port:
                    raise serial.SerialException("no Espressif device")
                self.ser = serial.Serial(port, 115200, timeout=0.05)
                time.sleep(0.2)
                for c in self.startup:
                    self.ser.write((c + "\n").encode())
                if not self.connected:
                    print("[viewer] connected to", port)
                self.connected = True
                return True
            except (serial.SerialException, OSError):
                self.connected = False
                time.sleep(1.0)
        return False

    def _line(self, line):
        if line.startswith("HR "):
            self.hr = dict(re.findall(r"(\w+)=([\w.\-]+)", line))
            self.hr_time = time.time()
        elif line.startswith("RR "):
            self.rr = dict(re.findall(r"(\w+)=([\w.\-]+)", line))
        elif line.startswith("M "):
            v = line.split()
            if len(v) == 4 + N_CHAN:
                try:
                    self.motion.append((int(v[1]) / 1000.0, np.array(v[4:], float), int(v[2]), int(v[3])))
                except ValueError:
                    pass
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
        if not self._connect():
            return
        while self.running:
            try:
                chunk = self.ser.read(65536)
            except (serial.SerialException, OSError):
                print("[viewer] board disconnected, waiting for it to come back...")
                self.connected = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
                buf = b""
                if not self._connect():
                    break
                continue
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
                if len(buf) < META_END:
                    break
                n, t = struct.unpack("<II", buf[4:12])
                if n > 200_000:
                    buf = buf[4:]
                    continue
                if len(buf) < META_END + n:
                    break
                meta = struct.unpack("<30h", buf[12:META_END])
                jpg = buf[META_END : META_END + n]
                # A frame abandoned mid-write (USB timeout) would otherwise eat
                # the telemetry that follows it: accept only a complete JPEG and
                # resynchronise on the next marker otherwise.
                if jpg[:2] == b"\xff\xd8" and jpg[-2:] == b"\xff\xd9":
                    self.frames.append((t / 1000.0, meta, jpg))
                    buf = buf[META_END + n :]
                else:
                    buf = buf[4:]

    def close(self):
        self.running = False
        self.join(timeout=1)
        try:
            self.send("x")
            self.send("w")
        finally:
            if self.ser:
                self.ser.close()
            if self.rec:
                self.rec.close()
            if self.marks:
                self.marks.close()


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


def breathing_wave(motion, ch, seconds=30.0):
    """Band-passed displacement of the device's best breathing channel."""
    if ch < 0 or len(motion) < 30:
        return None
    ts = np.array([m[0] for m in motion])
    keep = ts > ts[-1] - seconds
    ts = ts[keep]
    d = np.array([m[1][ch] for m in motion])[keep]
    if len(ts) < 30 or ts[-1] - ts[0] < 10:
        return None
    tu = np.arange(ts[0], ts[-1], 0.2)
    x = np.interp(tu, ts, d)
    x = x - np.polyval(np.polyfit(tu, x, 1), tu)
    sos = butter(2, [0.1, 0.75], btype="band", fs=5, output="sos")
    return sosfiltfilt(sos, x)


def breathing_regions(img, meta, sel, best):
    """Outline every region the breathing rate is fused from; highlight the best.

    Channels 0..31 are the 4x4 tile grid (even = vertical motion, odd = horizontal),
    32/33 the chest box, 34/35 the head box.
    """
    H, W = img.shape[:2]
    for ch in sel:
        is_best = ch == best
        col = (255, 255, 0) if is_best else (110, 110, 0)
        axis = "|" if ch % 2 == 0 else "-"
        if ch >= 34:                      # head box = the face box
            x, y = meta[0], meta[1]
            w, h = meta[2] - meta[0], meta[3] - meta[1]
            label = "head " + axis
        elif ch >= 32:                    # chest box
            x, y, w, h = meta[26:30]
            label = "chest " + axis
        else:
            t = ch // 2
            x, y = (t % 4) * W // 4, (t // 4) * H // 4
            w, h = W // 4 - 1, H // 4 - 1
            label = "breath " + axis
        if x < 0 or w <= 0:
            continue
        cv2.rectangle(img, (x, y), (x + w, y + h), col, 2 if is_best else 1)
        if is_best:
            cv2.putText(img, label, (x + 3, y + 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4, col, 1)


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
    ap.add_argument("--port", default="auto", help="serial port (default: find the ESP32 by USB id)")
    ap.add_argument("--scale", type=float, default=2.0, help="display scale")
    ap.add_argument("--alpha", type=float, default=60.0, help="magnification factor")
    ap.add_argument("--headless", action="store_true", help="no window; print the device heart rate")
    ap.add_argument("--no-video", action="store_true", help="do not stream video (faster on-device fps)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N seconds")
    ap.add_argument("--record", help="save the raw serial stream (video + samples + HR) to this file")
    ap.add_argument("--pacer", type=float, default=0, help="start the breathing pacer at this rate (/min)")
    ap.add_argument("--pacer-file", default="/tmp/heartcam_pacer",
                    help="write a rate (or 'off') to this file to change the pacer while running")
    args = ap.parse_args()

    dev = Device(args.port, args.record)
    dev.send("v", startup=True)
    if not args.no_video and not args.headless:
        dev.send("s", startup=True)
    dev.start()
    mag = Magnifier(alpha=args.alpha)
    magnify = True
    exposure = 300
    last_t = None
    last_print = 0
    view = None
    pacer_on, pacer_rate, pacer_t0 = args.pacer > 0, args.pacer or 10.0, time.time()
    if pacer_on:
        dev.mark(f"pacer on rate={pacer_rate:.0f}")
    pacer_mtime = 0.0
    start = time.time()
    try:
        while not args.seconds or time.time() - start < args.seconds:
            if args.headless:
                time.sleep(0.2)
                if dev.hr_time > last_print:
                    last_print = dev.hr_time
                    h = dev.hr
                    rr = dev.rr
                    print(f"bpm={h.get('bpm')} raw={h.get('raw')} {fmt_status(h)} face={h.get('face')} "
                          f"det_hit={h.get('det_hit')} fps={h.get('fps')} | breathing={rr.get('br')}/min "
                          f"{rr.get('state')} q={rr.get('q')} snr={rr.get('snr')}", flush=True)
                continue

            # external pacer control: a number (or "off") in the control file
            try:
                mt = os.path.getmtime(args.pacer_file)
                if mt != pacer_mtime:
                    pacer_mtime = mt
                    txt = open(args.pacer_file).read().strip().lower()
                    if txt in ("off", "0"):
                        pacer_on = False
                    elif txt:
                        pacer_rate = float(np.clip(float(txt), 4, 40))
                        pacer_on = True
                        pacer_t0 = time.time()
                    dev.mark(f"pacer {'on' if pacer_on else 'off'} rate={pacer_rate:.0f}")
                    print(f"[viewer] pacer {'on' if pacer_on else 'off'} at {pacer_rate:.0f}/min", flush=True)
            except (OSError, ValueError):
                pass

            if not dev.connected:
                view = None
            if dev.frames:
                t, meta, jpg = dev.frames.popleft()
                frame = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    dt = t - last_t if last_t and 0 < t - last_t < 1 else 0.1
                    last_t = t
                    left = frame.copy()
                    draw_overlay(left, meta)
                    sel = [int(c) for c in dev.rr.get("sel", "").split(",") if c.strip().lstrip("-").isdigit()]
                    breathing_regions(left, meta, sel, int(dev.rr.get("best", -1)))
                    right = mag(frame, dt) if magnify else frame
                    view = cv2.resize(np.hstack([left, right]), None, fx=args.scale, fy=args.scale)
            if view is None:
                view = np.zeros((int(240 * args.scale), int(640 * args.scale), 3), np.uint8)
                msg = "video off" if args.no_video else "waiting for video..."
                if not dev.connected:
                    msg = "board disconnected - waiting for it..."
                cv2.putText(view, msg,
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)

            W = view.shape[1]
            panel = np.zeros((330, W, 3), np.uint8)
            pulse = pos_pulse(list(dev.samples))
            draw_plot(panel, 5, 5, W - 10, 95, pulse, (90, 90, 255), "pulse (POS on device samples, last 10 s)")
            rr = dev.rr
            breath = breathing_wave(list(dev.motion), int(rr.get("best", -1)))
            draw_plot(panel, 5, 195, W - 10, 80, breath, (255, 255, 0), "breathing (best tile displacement, last 30 s)")
            rstate = rr.get("state", "")
            rcol = {"locked": (255, 255, 80), "acquiring": (0, 200, 255)}.get(rstate, (120, 120, 120))
            br = float(rr.get("br", 0) or 0)
            cv2.putText(panel, f"{br:4.1f} /min" if br > 0 else "--.- /min", (10, 318),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, rcol, 3)
            cv2.putText(panel, f"breathing {rstate}  q={rr.get('q', '?')} snr={rr.get('snr', '?')}dB "
                               f"stab={rr.get('stab', '?')} agree={rr.get('agree', '?')} motion={rr.get('motion', '?')} "
                               f"band={rr.get('band', '?')}",
                        (300, 312), cv2.FONT_HERSHEY_SIMPLEX, 0.5, rcol, 1)
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
            if pacer_on:
                phase = ((time.time() - pacer_t0) * pacer_rate / 60.0) % 1.0
                size = 0.5 - 0.5 * np.cos(2 * np.pi * phase)  # grows (inhale) then shrinks (exhale)
                cx, cy = W - 110, 90
                cv2.circle(view, (cx, cy), int(20 + 60 * size), (255, 200, 80), 3)
                label = "breathe IN" if phase < 0.5 else "breathe OUT"
                cv2.putText(view, f"{label}  pacer {pacer_rate:.0f}/min", (cx - 105, cy + 105),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 80), 2)
                cv2.putText(panel, f"pacer {pacer_rate:.0f}/min", (W - 220, 318), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (255, 200, 80), 2)
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
            elif key == ord("p"):
                pacer_on = not pacer_on
                pacer_t0 = time.time()
                dev.mark(f"pacer {'on' if pacer_on else 'off'} rate={pacer_rate:.0f} t={time.time():.3f}")
            elif key in (ord("["), ord("]")):
                pacer_rate = float(np.clip(pacer_rate + (1 if key == ord("]") else -1), 4, 40))
                pacer_t0 = time.time()
                dev.mark(f"pacer {'on' if pacer_on else 'off'} rate={pacer_rate:.0f} t={time.time():.3f}")
            elif key == ord("b"):
                dev.send("b1" if dev.rr.get("band") != "infant" else "b0")
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
