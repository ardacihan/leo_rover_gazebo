#!/usr/bin/env python3
"""Baseline-vs-tuned contact sheet for one run and timestamp.

    python3 cmp_ab.py <run_dir> <t> <out.png>

Rows: SLAM map, global costmap, local costmap. Columns: baseline (frozen
2026-08-20 params), tuned (current config/real). Camera frame on top for
ground truth.
"""
import sys
from pathlib import Path

import cv2
import numpy as np


def frame(path, t, label):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        return None
    fps = cap.get(cv2.CAP_PROP_FPS) or 8.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, int(t * fps))
    ok, f = cap.read()
    cap.release()
    if not ok:
        return None
    s = 430 / max(f.shape[:2])
    f = cv2.resize(f, (int(f.shape[1] * s), int(f.shape[0] * s)))
    cv2.rectangle(f, (0, 0), (f.shape[1], 18), (0, 0, 0), -1)
    cv2.putText(f, label, (4, 13), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (255, 255, 255), 1)
    return f


def main():
    run, t, out = Path(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
    rows = []
    cam = frame(run / 'default/color.mp4', t, f'camera t={t:.0f}s')
    for name in ('map', 'global_costmap', 'local_costmap'):
        pair = []
        for variant, label in (('logic_baseline', 'baseline'),
                               ('logic', 'robust')):
            f = frame(run / variant / f'{name}.mp4', t, f'{label} {name}')
            if f is not None:
                pair.append(f)
        if pair:
            h = max(p.shape[0] for p in pair)
            w = sum(p.shape[1] for p in pair) + 8
            row = np.zeros((h, w, 3), np.uint8)
            x = 0
            for p in pair:
                row[:p.shape[0], x:x + p.shape[1]] = p
                x += p.shape[1] + 8
            rows.append(row)
    if cam is not None:
        rows.insert(0, cam)
    H = sum(r.shape[0] + 8 for r in rows)
    W = max(r.shape[1] for r in rows)
    sheet = np.zeros((H, W, 3), np.uint8)
    y = 0
    for r in rows:
        sheet[y:y + r.shape[0], :r.shape[1]] = r
        y += r.shape[0] + 8
    cv2.imwrite(out, sheet)
    print('wrote', out)


if __name__ == '__main__':
    main()
