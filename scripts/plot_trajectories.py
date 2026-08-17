#!/usr/bin/env python3
"""Render a ground-truth / odometry-only / SLAM trajectory plot for a run.

Three paths from `pose_error.csv`, drawn over the rasterised world so the
route can be read against the actual walls. This is the plot that shows what
the robot *did*, as opposed to what it mapped.

Usage:
    python3 plot_trajectories.py <run_dir> <world.sdf> <out.png>
"""

import csv
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                        # noqa: E402

from world_ground_truth import rasterize_world   # noqa: E402

BG = '#0E1116'
GRID_WALL = '#59677A'
GT = '#E6EAF0'
ODOM = '#F85149'
SLAM = '#4C9AFF'


def load(path):
    cols = {k: [] for k in ('gt_x', 'gt_y', 'odom_x', 'odom_y', 'slam_x', 'slam_y')}
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            try:
                vals = {k: float(row[k]) for k in cols}
            except (TypeError, ValueError, KeyError):
                continue
            if any(v != v for v in vals.values()):
                continue
            for k, v in vals.items():
                cols[k].append(v)
    return {k: np.asarray(v) for k, v in cols.items()}


def main():
    run_dir, world, out = sys.argv[1], sys.argv[2], sys.argv[3]
    d = load(f'{run_dir}/pose_error.csv')
    if len(d['gt_x']) < 2:
        raise SystemExit(f'{run_dir}: too few samples')

    fig, ax = plt.subplots(figsize=(7.2, 4.6), dpi=130)
    fig.patch.set_facecolor(BG)
    ax.set_facecolor(BG)

    try:
        gt_grid = rasterize_world(world, res=0.05, z_plane=0.5)
        ys, xs = np.nonzero(gt_grid.occ)
        wx = gt_grid.origin[0] + (xs + 0.5) * gt_grid.res
        wy = gt_grid.origin[1] + (ys + 0.5) * gt_grid.res
        ax.scatter(wx, wy, s=0.35, c=GRID_WALL, marker='s', linewidths=0, alpha=0.85)
    except Exception:
        pass  # husarion has no usable raster; the paths still tell the story

    ax.plot(d['odom_x'], d['odom_y'], color=ODOM, lw=1.3, alpha=0.9,
            label='odometry only', zorder=3)
    ax.plot(d['slam_x'], d['slam_y'], color=SLAM, lw=1.3, alpha=0.9,
            label='SLAM estimate', zorder=4)
    ax.plot(d['gt_x'], d['gt_y'], color=GT, lw=1.9, label='ground truth', zorder=5)
    ax.scatter([d['gt_x'][0]], [d['gt_y'][0]], s=52, facecolor='#3FB950',
               edgecolor=BG, linewidth=1.4, zorder=6, label='start')
    ax.scatter([d['gt_x'][-1]], [d['gt_y'][-1]], s=52, facecolor='#FF5A3C',
               edgecolor=BG, linewidth=1.4, zorder=6, label='end')

    ax.set_aspect('equal', adjustable='datalim')
    ax.tick_params(colors='#8B98A9', labelsize=7)
    for spine in ax.spines.values():
        spine.set_color('#2A323D')
    ax.grid(color='#1E252E', lw=0.6)
    ax.set_axisbelow(True)
    leg = ax.legend(loc='upper right', fontsize=7, framealpha=0.9,
                    facecolor='#161B22', edgecolor='#2A323D', labelcolor='#E6EAF0')
    leg.get_frame().set_linewidth(0.6)

    fig.tight_layout(pad=0.4)
    fig.savefig(out, facecolor=BG)
    print(f'wrote {out}')


if __name__ == '__main__':
    main()
