#!/usr/bin/env python3
"""Render the recorded merge time-lapse into frames and an MP4.

Reads the `.npz` snapshots written by `merge_timelapse_recorder.py` and draws,
per frame, a three-panel view of the same instant:

    leo1 map | leo2 map | candidate merge | accepted merge

The point of showing all three together is that it makes the *merge* legible:
you can watch two separate partial maps grow independently, see the moment the
alignment locks, and then see leo2's geometry land on top of leo1's in the
shared panel. A single merged-map animation cannot show that -- it looks like
one map appearing out of nowhere.

Maps are drawn with hard three-colour quantisation (free / unknown / occupied),
never a grey ramp, for the same reason as the still renders: a ramp turns a
doubled wall into soft shading.

Usage:
  render_timelapse.py <run_dir> [--fps 6] [--width 1400] [--max-frames 200]
"""

import argparse
import glob
import math
import os

import cv2
import numpy as np

FREE = (245, 245, 247)
UNKNOWN = (205, 204, 201)
OCC = (29, 24, 20)
LEO1 = (180, 119, 31)     # BGR of #1f77b4
LEO2 = (10, 113, 232)     # BGR of #e8710a
BG = (255, 255, 255)


def colourise(grid, flip=True):
    """int8 occupancy -> BGR image, row 0 = ymin."""
    if grid is None or grid.size <= 1:
        return None
    img = np.zeros(grid.shape + (3,), dtype=np.uint8)
    img[:] = UNKNOWN
    img[(grid >= 0) & (grid < 50)] = FREE
    img[grid >= 50] = OCC
    # ``np.flipud`` returns a negative-stride view.  OpenCV drawing functions
    # (circle/line) reject that layout on the Humble image's OpenCV 4.5 build,
    # so materialize a contiguous array before trajectories are overlaid.
    return np.ascontiguousarray(np.flipud(img)) if flip else img


def world_to_px(x, y, info, shape):
    ox, oy, res = info
    h = shape[0]
    c = int(round((x - ox) / res))
    r = int(round((y - oy) / res))
    return c, h - 1 - r          # image row 0 is ymax after the flip


def panel(grid, info, size, trail=None, pose=None, colour=LEO1, title=''):
    img = colourise(grid)
    if img is None:
        img = np.full((size[1], size[0], 3), BG, dtype=np.uint8)
    else:
        for pts, col in (trail or []):
            prev = None
            for (x, y) in pts:
                if not (math.isfinite(x) and math.isfinite(y)):
                    prev = None
                    continue
                p = world_to_px(x, y, info, grid.shape)
                if prev is not None:
                    cv2.line(img, prev, p, col, 1, cv2.LINE_AA)
                prev = p
        if pose is not None and all(math.isfinite(v) for v in pose):
            p = world_to_px(pose[0], pose[1], info, grid.shape)
            cv2.circle(img, p, 4, colour, -1, cv2.LINE_AA)
            cv2.circle(img, p, 6, (255, 255, 255), 1, cv2.LINE_AA)
    # letterbox into the panel, preserving aspect
    h, w = img.shape[:2]
    scale = min(size[0] / w, size[1] / h)
    resized = cv2.resize(img, (max(1, int(w * scale)), max(1, int(h * scale))),
                         interpolation=cv2.INTER_NEAREST)
    out = np.full((size[1], size[0], 3), BG, dtype=np.uint8)
    y0 = (size[1] - resized.shape[0]) // 2
    x0 = (size[0] - resized.shape[1]) // 2
    out[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    cv2.putText(out, title, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (60, 60, 60), 1, cv2.LINE_AA)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--fps', type=float, default=6.0)
    ap.add_argument('--width', type=int, default=1400)
    ap.add_argument('--max-frames', type=int, default=200)
    args = ap.parse_args()

    snaps = sorted(glob.glob(os.path.join(args.run_dir, 'timelapse', 'snap*.npz')))
    if not snaps:
        print(f'no snapshots in {args.run_dir}/timelapse')
        return 1
    if len(snaps) > args.max_frames:
        step = len(snaps) / args.max_frames
        snaps = [snaps[int(i * step)] for i in range(args.max_frames)]

    pw = args.width // 4
    ph = int(pw * 0.85)
    out_path = os.path.join(args.run_dir, 'merge_timelapse.mp4')
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'),
                             args.fps, (pw * 4, ph + 40))
    frames_dir = os.path.join(args.run_dir, 'timelapse_frames')
    os.makedirs(frames_dir, exist_ok=True)

    t0 = None
    trail1, trail2 = [], []
    written = 0
    for i, path in enumerate(snaps):
        d = np.load(path)
        t = float(d['t'])
        t0 = t if t0 is None else t0
        p1, p2 = d['p1'], d['p2']
        if math.isfinite(p1[0]):
            trail1.append((float(p1[0]), float(p1[1])))
        if math.isfinite(p2[0]):
            trail2.append((float(p2[0]), float(p2[1])))
        locked = bool(d['locked'])
        tf = d['tf']

        # leo2's trail expressed in leo1's frame, once a transform exists.
        trail2_shared = []
        if math.isfinite(tf[0]):
            c, s = math.cos(tf[2]), math.sin(tf[2])
            trail2_shared = [(tf[0] + c * x - s * y, tf[1] + s * x + c * y)
                             for (x, y) in trail2]

        panels = [
            panel(d['leo1'], d['leo1_info'], (pw, ph),
                  trail=[(trail1, LEO1)], pose=p1, colour=LEO1,
                  title='leo1  /leo1/map'),
            panel(d['leo2'], d['leo2_info'], (pw, ph),
                  trail=[(trail2, LEO2)], pose=p2, colour=LEO2,
                  title='leo2  /leo2/map'),
            panel(d['candidate'], d['candidate_info'], (pw, ph),
                  title='candidate merge  (preview only)'),
            panel(d['shared'], d['shared_info'], (pw, ph),
                  trail=[(trail1, LEO1), (trail2_shared, LEO2)],
                  title='accepted merge  /shared_map'),
        ]
        strip = np.hstack(panels)
        bar = np.full((40, pw * 4, 3), (250, 250, 250), dtype=np.uint8)
        status = ('ALIGNED  leo2 in leo1 frame: '
                  f'({tf[0]:+.2f}, {tf[1]:+.2f}, {math.degrees(tf[2]):+.1f} deg)'
                  if locked and math.isfinite(tf[0])
                  else 'NOT ALIGNED  - rovers have not seen common markers yet')
        colour = (40, 130, 40) if locked else (40, 40, 190)
        cv2.putText(bar, f't = {t - t0:6.0f} s   {status}', (12, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, colour, 1, cv2.LINE_AA)
        frame = np.vstack([strip, bar])
        writer.write(frame)
        if i % max(1, len(snaps) // 60) == 0:
            cv2.imwrite(os.path.join(frames_dir, f'f{i:04d}.jpg'), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 78])
        written += 1
    writer.release()
    size_mb = os.path.getsize(out_path) / 1e6 if os.path.exists(out_path) else 0
    print(f'{out_path}: {written} frames, {size_mb:.1f} MB, {pw*4}x{ph+40}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
