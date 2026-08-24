#!/usr/bin/env python3
"""Rasterize a world and render it with candidate spawn poses drawn on it.

Two rovers must start in *different rooms*, and a spawn that lands inside a
wall wastes a whole sim run. This renders the ground-truth occupancy with the
proposed spawns and marker positions marked so they can be checked by eye
before the run, and prints the free/occupied verdict for each point.

Usage: pick_spawns.py <world.sdf> <out.png> [--pt x,y,label ...]
"""
import argparse
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

sys.path.insert(0, '/ros2_ws/scripts')
from world_ground_truth import rasterize_world


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('out')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--pt', action='append', default=[],
                    help='x,y,label — drawn and clearance-checked')
    ap.add_argument('--clearance', type=float, default=0.35,
                    help='metres of free space a spawn needs all round')
    args = ap.parse_args()

    gt = rasterize_world(args.world, args.res)
    h, w = gt.occ.shape
    ox, oy = gt.origin

    fig, ax = plt.subplots(figsize=(11, 8), dpi=110)
    ax.imshow(gt.occ, origin='lower', cmap='Greys',
              extent=[ox, ox + w * args.res, oy, oy + h * args.res])

    rad = int(round(args.clearance / args.res))
    for spec in args.pt:
        x, y, label = spec.split(',', 2)
        x, y = float(x), float(y)
        c = int(round((x - ox) / args.res))
        r = int(round((y - oy) / args.res))
        if 0 <= r < h and 0 <= c < w:
            patch = gt.occ[max(0, r - rad):r + rad + 1, max(0, c - rad):c + rad + 1]
            free = not patch.any()
        else:
            free = False
        colour = 'tab:green' if free else 'tab:red'
        ax.plot(x, y, 'o', ms=11, mfc='none', mew=2.2, color=colour)
        ax.annotate(label, (x, y), textcoords='offset points', xytext=(9, 6),
                    fontsize=9, color=colour, weight='bold')
        print(f'{label:22s} ({x:7.2f}, {y:7.2f})  '
              f'{"FREE" if free else "BLOCKED/OUT OF BOUNDS"}')

    ax.set_title(f'{args.world.split("/")[-1]}  bounds={tuple(round(b, 1) for b in gt.bounds)}')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.grid(alpha=0.25, lw=0.4)
    fig.tight_layout()
    fig.savefig(args.out)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
