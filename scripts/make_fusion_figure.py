#!/usr/bin/env python3
"""Before/after merged-map comparison figure + wall-IoU table.

Usage:
  python make_fusion_figure.py --world <sdf> --before <dir> --after <dir> \
      --out <png> [--title "office_world"]

`before`/`after` are map_fusion.py output dirs (merged.yaml + result.json).
"""

import argparse
import json
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from map_fusion import load_map
from world_ground_truth import rasterize_world, score_map


def panel(ax, m, title, metrics):
    H, W = m.shape
    rgb = np.full((H, W, 3), 0.86, dtype=np.float32)
    rgb[m.grid == 0] = 1.0
    rgb[m.grid == 100] = 0.05
    ext = [m.origin[0], m.origin[0] + W * m.res,
           m.origin[1], m.origin[1] + H * m.res]
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
    ax.set_title(title, fontsize=11)
    txt = (f"wall IoU@10cm: {metrics['iou@2']:.3f}\n"
           f"precision@10cm: {metrics['precision@2']:.3f}\n"
           f"RMSE to true wall: {metrics['rmse_m'] * 100:.1f} cm")
    ax.text(0.02, 0.02, txt, transform=ax.transAxes, fontsize=8.5,
            va='bottom', ha='left',
            bbox=dict(facecolor='white', alpha=0.85, edgecolor='0.6'))
    ax.set_xlabel('x [m]')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True)
    ap.add_argument('--before', required=True)
    ap.add_argument('--after', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--title', default='')
    args = ap.parse_args()

    gt = rasterize_world(args.world, 0.05)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5.5))
    rows = {}
    for ax, d, label in [
            (axes[0], args.before,
             'BEFORE: fixed-offset overwrite merge (old pipeline)'),
            (axes[1], args.after,
             'AFTER: registered + log-odds fused + cleaned')]:
        m = load_map(os.path.join(d, 'merged.yaml'))
        s = score_map(m.grid == 100, m.origin, m.res, gt)
        rows[label] = s
        panel(ax, m, label, s)
    axes[0].set_ylabel('y [m]')
    if args.title:
        fig.suptitle(args.title, fontsize=13)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    with open(os.path.splitext(args.out)[0] + '_metrics.json', 'w') as f:
        json.dump(rows, f, indent=2)
    for label, s in rows.items():
        print(f"{label}: IoU@2={s['iou@2']:.3f} prec@2={s['precision@2']:.3f} "
              f"rec@2={s['recall@2']:.3f} rmse={s['rmse_m']:.3f} m")
    print('wrote', args.out)


if __name__ == '__main__':
    main()
