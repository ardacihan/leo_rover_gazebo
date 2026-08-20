#!/usr/bin/env python3
"""Contact sheet of every dashboard panel at one timestamp.

    python3 grab_frames.py <run_dir> <t_seconds> <out.png>
"""
import sys
from pathlib import Path

import cv2
import numpy as np

VIDEOS = [('default/color.mp4', 'camera'), ('default/depth.mp4', 'depth'),
          ('default/lidar.mp4', 'lidar/odom'), ('logic/map.mp4', 'slam map'),
          ('logic/global_costmap.mp4', 'global costmap'),
          ('logic/local_costmap.mp4', 'local costmap')]


def main():
    run, t, out = Path(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    tiles = []
    for rel, name in VIDEOS:
        p = run / rel
        if not p.exists():
            continue
        cap = cv2.VideoCapture(str(p))
        fps = cap.get(cv2.CAP_PROP_FPS) or 8.0
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
        ok, frame = cap.read()
        cap.release()
        if not ok:
            continue
        h, w = frame.shape[:2]
        scale = 420 / max(h, w)
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 18), (0, 0, 0), -1)
        cv2.putText(frame, f'{name} t={t:.1f}s', (4, 13),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        tiles.append(frame)
    if not tiles:
        sys.exit('no frames')
    H = max(f.shape[0] for f in tiles)
    W = max(f.shape[1] for f in tiles)
    cols = 3
    rows = (len(tiles) + cols - 1) // cols
    sheet = np.zeros((rows * H, cols * W, 3), np.uint8)
    for i, f in enumerate(tiles):
        r, c = divmod(i, cols)
        sheet[r * H:r * H + f.shape[0], c * W:c * W + f.shape[1]] = f
    cv2.imwrite(out, sheet)
    print('wrote', out)


if __name__ == '__main__':
    main()
