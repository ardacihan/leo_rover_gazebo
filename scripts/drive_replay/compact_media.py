#!/usr/bin/env python3
"""Re-encode a run's dashboard media small enough to embed in one artifact.

Reads <run_dir>/{default,logic}/*.mp4, writes <run_dir>/compact/ with the
same relative names, downscaled and frame-decimated but on the same clock
(fps stays correct relative to playback: dropping every second frame halves
the fps declared to the writer, so currentTime still maps 1:1 to bag time).

    python3 compact_media.py <run_dir> [--width 360] [--drop 2]
"""
import argparse
import sys
from pathlib import Path

import cv2

VIDEOS = ['default/color.mp4', 'default/depth.mp4', 'default/lidar.mp4',
          'logic/map.mp4', 'logic/global_costmap.mp4',
          'logic/local_costmap.mp4']


def recode(src, dst, width, drop):
    cap = cv2.VideoCapture(str(src))
    if not cap.isOpened():
        return False
    fps = cap.get(cv2.CAP_PROP_FPS) or 8.0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if w > width:
        h = int(h * width / w / 2) * 2
        w = width
    else:
        w -= w % 2
        h -= h % 2
    dst.parent.mkdir(parents=True, exist_ok=True)
    out = cv2.VideoWriter(str(dst), cv2.VideoWriter_fourcc(*'avc1'),
                          fps / drop, (w, h))
    if not out.isOpened():
        sys.exit(f'writer failed: {dst}')
    n = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if n % drop == 0:
            out.write(cv2.resize(frame, (w, h)))
        n += 1
    cap.release()
    out.release()
    print(f'{src.name}: {n} frames -> {dst.stat().st_size/1e6:.1f} MB '
          f'({w}x{h} @ {fps/drop:.0f} fps)')
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--width', type=int, default=360)
    ap.add_argument('--drop', type=int, default=2)
    args = ap.parse_args()
    run = Path(args.run_dir)
    for rel in VIDEOS:
        src = run / rel
        if src.exists():
            recode(src, run / 'compact' / rel, args.width, args.drop)


if __name__ == '__main__':
    main()
