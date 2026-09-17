#!/usr/bin/env python3
"""Stream a video as device-like frames (320x240 YUV422/YUYV) to stdout,
for idf/test/resp_replay. The frame is center-cropped to 4:3 first.

usage: video_frames.py clip.mp4 [--target-fps 11.1] [--max-seconds N] | resp_replay
"""
import argparse
import struct
import sys

import cv2
import numpy as np

W, H = 320, 240


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--target-fps", type=float, default=11.1, help="drop frames to mimic the device (0 = all)")
    ap.add_argument("--max-seconds", type=float, default=0)
    ap.add_argument("--crop", help="x,y,w,h in source pixels (before scaling): zoom in on the subject")
    args = ap.parse_args()
    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    out = sys.stdout.buffer
    i, next_keep = 0, 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        t = i / fps
        i += 1
        if args.max_seconds and t > args.max_seconds:
            break
        if args.target_fps:
            if t + 1e-6 < next_keep:
                continue
            next_keep += 1 / args.target_fps
        if args.crop:
            cx, cy, cw, ch = (int(v) for v in args.crop.split(","))
            frame = frame[cy:cy + ch, cx:cx + cw]
        h, w = frame.shape[:2]
        if w * 3 > h * 4:  # crop to 4:3
            cw = h * 4 // 3
            frame = frame[:, (w - cw) // 2:(w - cw) // 2 + cw]
        small = cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)
        yuv = cv2.cvtColor(small, cv2.COLOR_BGR2YUV)  # Y, U, V per pixel
        buf = np.empty((H, W, 2), np.uint8)
        buf[..., 0] = yuv[..., 0]                      # Y for every pixel
        buf[..., 1][:, 0::2] = yuv[..., 1][:, 0::2]    # U from the even pixel of each pair
        buf[..., 1][:, 1::2] = yuv[..., 2][:, 0::2]    # V from the same pair
        out.write(b"FRM1" + struct.pack("<IHH", round(t * 1000), W, H) + buf.tobytes())
    out.flush()


if __name__ == "__main__":
    main()
