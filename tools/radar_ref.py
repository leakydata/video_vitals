#!/usr/bin/env python3
"""Compare heart_cam with a Seeed MR60BHA2 60 GHz radar (breathing + heart rate).

The radar is an independent reference: point it at the same subject as the
camera, sit still, and this logs both and reports how well they agree.

Both boards are Espressif USB devices, so each port is probed: the radar
speaks TinyFrames (SOF 0x01), heart_cam sends "HR ..." / "RR ..." text.

usage: radar_ref.py [--seconds N] [--csv out.csv] [--radar-port P] [--cam-port P]
"""
import argparse
import re
import struct
import sys
import threading
import time

import serial
import serial.tools.list_ports

T_BREATH, T_HEART = 0x0A14, 0x0A15


def cksum(b):
    c = 0
    for x in b:
        c ^= x
    return ~c & 0xFF


class FrameParser:
    """TinyFrame decoder: SOF 0x01 | ID u16 | LEN u16 | TYPE u16 | HCK | DATA | DCK."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        self.buf += data
        out, b, i = [], self.buf, 0
        while len(b) - i >= 8:
            if b[i] != 0x01 or cksum(b[i:i + 7]) != b[i + 7]:
                i += 1
                continue
            length = (b[i + 3] << 8) | b[i + 4]
            ftype = (b[i + 5] << 8) | b[i + 6]
            if length > 1024:
                i += 1
                continue
            end = i + 8 + length + (1 if length else 0)
            if end > len(b):
                break
            payload = bytes(b[i + 8:i + 8 + length])
            if length and cksum(payload) != b[end - 1]:
                i += 1
                continue
            out.append((ftype, payload))
            i = end
        del b[:i]
        return out


def identify(port, seconds=3.0):
    """Return 'radar', 'cam' or None by listening to the port."""
    try:
        with serial.Serial(port, 115200, timeout=0.2) as s:
            data = b""
            t0 = time.time()
            while time.time() - t0 < seconds:
                data += s.read(4096)
                if re.search(rb"^(HR|RR) ", data, re.M):
                    return "cam"
                if FrameParser().feed(data):
                    return "radar"
    except (serial.SerialException, OSError):
        return None
    return None


def find_ports(radar_port, cam_port):
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid == 0x303A]
    known = {radar_port: "radar", cam_port: "cam"}
    found = {v: k for k, v in known.items() if k}
    for p in ports:
        if p in known:
            continue
        kind = identify(p)
        if kind and kind not in found:
            found[kind] = p
            print(f"[{kind}] {p}")
    return found.get("radar"), found.get("cam")


class Reader(threading.Thread):
    def __init__(self, port, kind):
        super().__init__(daemon=True)
        self.port, self.kind = port, kind
        self.br = self.hr = None      # latest rates
        self.br_t = self.hr_t = 0.0   # when each was last updated (monotonic)
        self.running = True

    def run(self):
        fp = FrameParser()
        buf = b""
        try:
            ser = serial.Serial(self.port, 115200, timeout=0.2)
        except (serial.SerialException, OSError) as e:
            print(f"[{self.kind}] cannot open {self.port}: {e}")
            return
        if self.kind == "cam":
            ser.write(b"\n")  # wake the command parser
        while self.running:
            try:
                data = ser.read(4096)
            except (serial.SerialException, OSError):
                break
            if not data:
                continue
            if self.kind == "radar":
                for ftype, p in fp.feed(data):
                    if ftype == T_BREATH and len(p) >= 4:
                        self.br = struct.unpack("<f", p[:4])[0]
                        self.br_t = time.monotonic()
                    elif ftype == T_HEART and len(p) >= 4:
                        self.hr = struct.unpack("<f", p[:4])[0]
                        self.hr_t = time.monotonic()
            else:
                buf += data
                lines = buf.split(b"\n")
                buf = lines[-1][-4096:]
                for ln in lines[:-1]:
                    m = re.search(rb"^(HR|RR) .*", ln)
                    if not m:
                        continue
                    kv = dict(re.findall(rb"(\w+)=([\w.\-]+)", ln))
                    val = float(kv.get(b"bpm" if ln.startswith(b"HR") else b"br", 0) or 0)
                    state = kv.get(b"state", b"").decode()
                    if ln.startswith(b"HR"):
                        self.hr = val if state == "locked" else None
                        self.hr_t = time.monotonic()
                    else:
                        self.br = val if state == "locked" else None
                        self.br_t = time.monotonic()
        ser.close()
        self.br = self.hr = None   # a reader that has stopped has nothing current to say


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--csv")
    ap.add_argument("--radar-port")
    ap.add_argument("--cam-port")
    a = ap.parse_args()

    radar_port, cam_port = find_ports(a.radar_port, a.cam_port)
    if not radar_port:
        print("no MR60BHA2 radar found (is the bridge plugged in?)")
        return 1
    if not cam_port:
        print("no heart_cam board found")
        return 1

    readers = [Reader(radar_port, "radar"), Reader(cam_port, "cam")]
    for r in readers:
        r.start()
    radar, cam = readers
    out = open(a.csv, "w") if a.csv else None
    if out:
        out.write("t,radar_br,radar_hr,cam_br,cam_hr\n")
    print(f"{'t':>5} {'radar br':>9} {'cam br':>8} {'diff':>6} | {'radar hr':>9} {'cam hr':>8} {'diff':>6}")
    dbr, dhr = [], []
    t0 = time.time()
    try:
        while time.time() - t0 < a.seconds:
            time.sleep(1.0)
            t = time.time() - t0
            # Only pair readings that are both current: a stopped device would
            # otherwise keep contributing its last value to the agreement score.
            FRESH = 5.0
            now = time.monotonic()
            fresh = lambda v, t: v if (v is not None and now - t < FRESH) else None
            rb, rh = fresh(radar.br, radar.br_t), fresh(radar.hr, radar.hr_t)
            cb, ch = fresh(cam.br, cam.br_t), fresh(cam.hr, cam.hr_t)
            if out:
                out.write(f"{t:.1f},{rb or ''},{rh or ''},{cb or ''},{ch or ''}\n")
                out.flush()
            fb = f"{rb - cb:+.1f}" if rb and cb else "-"
            fh = f"{rh - ch:+.1f}" if rh and ch else "-"
            if rb and cb:
                dbr.append(abs(rb - cb))
            if rh and ch:
                dhr.append(abs(rh - ch))
            print(f"{t:5.0f} {rb if rb else float('nan'):9.1f} {cb if cb else float('nan'):8.1f} {fb:>6} | "
                  f"{rh if rh else float('nan'):9.1f} {ch if ch else float('nan'):8.1f} {fh:>6}", flush=True)
    except KeyboardInterrupt:
        pass
    for r in readers:
        r.running = False
    if out:
        out.close()
    if dbr:
        print(f"breathing: {len(dbr)} paired samples, mean |difference| {sum(dbr) / len(dbr):.2f} /min")
    if dhr:
        print(f"heart rate: {len(dhr)} paired samples, mean |difference| {sum(dhr) / len(dhr):.2f} bpm")
    if not dbr and not dhr:
        print("no paired samples (neither device reported a locked rate)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
