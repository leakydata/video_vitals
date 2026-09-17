#!/usr/bin/env python3
"""Turn a face video into the per-frame ROI samples the firmware produces.

Mimics the device: frames are scaled to 320x240 and quantised to RGB565,
landmarks come from a face detector running at ~3 Hz with the same smoothing,
ROI placement, dead-band, skin gate and motion flag as main.cpp. Output is
"S t_ms r g b n r g b n r g b n motion" lines, readable by test/replay.

usage: video_to_samples.py video.mp4 out.txt [--max-seconds N] [--det-every K]
"""
import argparse
import math
import os

import cv2
import numpy as np

W, H = 320, 240
JUMP_MOTION = 0.04
MIN_ROI_PIXELS = 40
FACE_MOTION = 0.35
MODEL = os.path.join(os.path.dirname(__file__), "models", "yunet.onnx")


def rgb565(frame_bgr):
    """Quantise like the sensor: 5/6/5 bits, then expand as the firmware does."""
    b = (frame_bgr[..., 0] >> 3).astype(np.int32) << 3
    g = (frame_bgr[..., 1] >> 2).astype(np.int32) << 2
    r = (frame_bgr[..., 2] >> 3).astype(np.int32) << 3
    return r, g, b


def face_rois(box, eye_l, eye_r, mouth):
    """Same placement as main.cpp face_rois()."""
    dx, dy = eye_r[0] - eye_l[0], eye_r[1] - eye_l[1]
    d = math.hypot(dx, dy)
    if d < 8:
        return []
    mx, my = (eye_l[0] + eye_r[0]) / 2, (eye_l[1] + eye_r[1]) / 2
    em = abs((mouth[0] - mx) * (-dy / d) + (mouth[1] - my) * (dx / d))
    v = min(max(em / 1.1, 0.5 * d), 1.5 * d)
    spec = [
        (mx, my - 0.80 * v, 1.20 * d, 0.55 * v),
        (eye_l[0] - 0.10 * d, eye_l[1] + 0.60 * v, 0.60 * d, 0.45 * v),
        (eye_r[0] + 0.10 * d, eye_r[1] + 0.60 * v, 0.60 * d, 0.45 * v),
    ]
    bw = box[2] - box[0]
    bx0, bx1 = max(0.0, box[0] + 0.05 * bw), min(float(W), box[2] - 0.05 * bw)
    by0, by1 = max(0.0, box[1]), min(float(H), box[3])
    out = []
    for cx, cy, w, h in spec:
        cx0, cx1 = max(cx - w / 2, bx0), min(cx + w / 2, bx1)
        cy0, cy1 = max(cy - h / 2, by0), min(cy + h / 2, by1)
        if max(0, cx1 - cx0) * max(0, cy1 - cy0) < 0.5 * w * h:
            out.append((0, 0, 0, 0))
            continue
        x0, y0 = round(cx0), round(cy0)
        out.append((x0, y0, round(cx1) - x0, round(cy1) - y0))
    return out


def roi_mean(r, g, b, roi, any_colour=False):
    x, y, w, h = roi
    if w < 4 or h < 4:
        return (0.0, 0.0, 0.0, 0)
    R, G, B = r[y:y + h, x:x + w].ravel(), g[y:y + h, x:x + w].ravel(), b[y:y + h, x:x + w].ravel()
    ok = (R <= 244) & (G <= 248) & (B <= 244)
    Y = (77 * R + 150 * G + 29 * B) >> 8
    ok &= Y >= 25
    Cr = ((128 * R - 107 * G - 21 * B) >> 8) + 128
    Cb = ((-43 * R - 85 * G + 128 * B) >> 8) + 128
    skin = ok & (Cr >= 133) & (Cr <= 173) & (Cb >= 77) & (Cb <= 127)
    n, an = int(skin.sum()), int(ok.sum())
    if not any_colour and n >= an * 4 // 10 and n >= 16:
        m = skin
    elif any_colour and an >= 16:
        m = ok
        n = an
    else:
        return (0.0, 0.0, 0.0, 0)
    return (R[m].mean(), G[m].mean(), B[m].mean(), min(n, 65535))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("out")
    ap.add_argument("--max-seconds", type=float, default=0)
    ap.add_argument("--det-every", type=int, default=0, help="run detection every K frames (default: ~3 Hz)")
    ap.add_argument("--start", type=float, default=0, help="skip the first N seconds")
    ap.add_argument("--target-fps", type=float, default=11.1, help="drop frames to mimic the device rate (0 = keep all)")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    eff_fps = min(fps, args.target_fps) if args.target_fps else fps
    det_every = args.det_every or max(1, round(eff_fps / 3))
    next_keep = 0.0
    det = cv2.FaceDetectorYN.create(MODEL, "", (W, H), 0.6)
    if args.start:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start * 1000)

    face = None  # smoothed [x, y, w, h, 10 landmarks]
    last_det_t = -10.0
    prev_c = None
    motion_until = -1.0
    held = None
    gain = [np.ones(3) for _ in range(3)]
    prev = [None] * 3
    jump_until = -1.0
    i = kept = 0
    frames = hits = 0
    with open(args.out, "w") as f:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t = i / fps
            i += 1
            if args.max_seconds and t > args.max_seconds:
                break
            if args.target_fps and t + 1e-6 < next_keep:
                continue
            next_keep = (next_keep if args.target_fps else 0) + (1 / args.target_fps if args.target_fps else 0)
            small = cv2.resize(frame, (W, H), interpolation=cv2.INTER_AREA)
            r, g, b = rgb565(small)

            kept += 1
            if (kept - 1) % det_every == 0:
                frames += 1
                _, faces = det.detect(small)
                if faces is not None and len(faces):
                    hits += 1
                    best = max(faces, key=lambda z: z[2] * z[3])
                    x, y, w, h = best[:4]
                    lm = best[4:14].copy()
                    # YuNet: subject's right eye, left eye, nose, right mouth, left mouth.
                    # Reorder to the MNP convention (image-left eye first).
                    eyes = sorted([(lm[0], lm[1]), (lm[2], lm[3])])
                    mouth = sorted([(lm[6], lm[7]), (lm[8], lm[9])])
                    kp = np.array([*eyes[0], *mouth[0], lm[4], lm[5], *eyes[1], *mouth[1]], float)
                    cur = np.concatenate([[x, y, x + w, y + h], kp])
                    c = ((x + w / 2), (y + h / 2))
                    if prev_c is not None and t - prev_c[2] < 0.8:
                        sc = (0.5 * c[0] + 0.5 * prev_c[0], 0.5 * c[1] + 0.5 * prev_c[1])
                        v = math.hypot(sc[0] - prev_c[0], sc[1] - prev_c[1]) / w / (t - prev_c[2])
                        if v > FACE_MOTION:
                            motion_until = t + 0.4
                        prev_c = (sc[0], sc[1], t)
                    else:
                        prev_c = (c[0], c[1], t)
                    fresh = face is None or t - last_det_t > 2.5 or abs(c[0] - (face[0] + face[2]) / 2) > w * 0.5
                    face = cur if fresh else face + 0.3 * (cur - face)
                    last_det_t = t
                else:
                    prev_c = None

            samples = [(0.0, 0.0, 0.0, 0)] * 3
            if face is not None and t - last_det_t < 2.5:
                rois = face_rois(face[0:4], face[4:6], face[10:12], ((face[6] + face[12]) / 2, (face[7] + face[13]) / 2))
                if rois:
                    moved = held is None or any(
                        abs(a[0] - h_[0]) > 2 or abs(a[1] - h_[1]) > 2 or abs(a[2] - h_[2]) > 3 or abs(a[3] - h_[3]) > 3
                        for a, h_ in zip(rois, held))
                    if moved:
                        for k in range(len(rois)):
                            a_ = roi_mean(r, g, b, held[k]) if held is not None else (0, 0, 0, 0)
                            b_ = roi_mean(r, g, b, rois[k])
                            if a_[3] and b_[3] and min(b_[:3]) > 0:
                                gain[k] = np.clip(gain[k] * np.array(a_[:3]) / np.array(b_[:3]), 0.3, 3.0)
                            else:
                                gain[k] = np.ones(3)
                        held = rois
                    samples = []
                    for k, roi in enumerate(held):
                        m = roi_mean(r, g, b, roi)
                        m = (*(np.array(m[:3]) * gain[k]), m[3])
                        if m[3] < MIN_ROI_PIXELS:
                            m = (0.0, 0.0, 0.0, 0)
                        if m[3] and prev[k] is not None and prev[k][3]:
                            y0, y1 = sum(prev[k][:3]), sum(m[:3])
                            thr = JUMP_MOTION * max(1.0, math.sqrt(200.0 / min(m[3], prev[k][3])))
                            if y0 > 0 and abs(y1 - y0) / y0 > thr:
                                jump_until = t + 0.3
                        prev[k] = m
                        samples.append(m)
            else:
                held = None
                prev = [None] * 3
            motion = int(t < motion_until or t < jump_until)
            f.write("S %d %s %d\n" % (round(t * 1000), " ".join(
                "%.2f %.2f %.2f %d" % s for s in samples), motion))
    print(f"{args.video}: {i} frames @ {fps:.2f} fps, detection hit rate {hits}/{frames}")


if __name__ == "__main__":
    main()
