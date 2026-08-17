#!/usr/bin/env python3
"""Plot ground-truth, wheel-odometry and SLAM trajectories from pose_error.csv.

Usage:
    python3 plot_pose_error.py <pose_error.csv> <out.png> [title]
"""

import csv
import math
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    t, gt, odom, slam = [], [], [], []
    with open(path) as fh:
        for row in csv.DictReader(fh):
            if not row.get('gt_x'):
                continue
            t.append(float(row['t']))
            gt.append((float(row['gt_x']), float(row['gt_y'])))
            odom.append((float(row['odom_x']), float(row['odom_y']))
                        if row.get('odom_x') else (np.nan, np.nan))
            slam.append((float(row['slam_x']), float(row['slam_y']))
                        if row.get('slam_x') else (np.nan, np.nan))
    return (np.asarray(t), np.asarray(gt), np.asarray(odom),
            np.asarray(slam))


def main():
    path, out_png = sys.argv[1], sys.argv[2]
    title = sys.argv[3] if len(sys.argv) > 3 else ''
    t, gt, odom, slam = load(path)
    t = t - t[0]

    fig, axes = plt.subplots(1, 2, figsize=(16, 6.6))

    ax = axes[0]
    ax.plot(gt[:, 0], gt[:, 1], c='0.25', lw=2.4, label='ground truth')
    ax.plot(odom[:, 0], odom[:, 1], c='#d1495b', lw=1.8, ls='--',
            label='wheel odometry only')
    ax.plot(slam[:, 0], slam[:, 1], c='#1b6ac9', lw=1.8,
            label='SLAM estimate')
    ax.plot(gt[0, 0], gt[0, 1], marker='o', ms=9, c='green', zorder=5)
    ax.set_aspect('equal')
    ax.grid(alpha=0.25)
    ax.legend(loc='best')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title('trajectories')

    ax = axes[1]
    odom_err = np.hypot(odom[:, 0] - gt[:, 0], odom[:, 1] - gt[:, 1])
    slam_err = np.hypot(slam[:, 0] - gt[:, 0], slam[:, 1] - gt[:, 1])
    ax.plot(t, odom_err, c='#d1495b', lw=2, label='wheel odometry error')
    ax.plot(t, slam_err, c='#1b6ac9', lw=2, label='SLAM error')
    ax.set_xlabel('time (s)')
    ax.set_ylabel('position error (m)')
    ax.grid(alpha=0.25)
    ax.legend(loc='best')
    ax.set_title('error against ground truth')

    if title:
        fig.suptitle(title, fontsize=15)
        plt.tight_layout(rect=[0, 0, 1, 0.95])
    else:
        plt.tight_layout()
    plt.savefig(out_png, dpi=115)
    print('wrote', out_png)
    finite = np.isfinite(slam_err)
    print(f'odom final {odom_err[-1]:.2f} m, '
          f'SLAM rms {math.sqrt(np.nanmean(slam_err[finite] ** 2)):.2f} m, '
          f'SLAM final {slam_err[finite][-1]:.2f} m')


if __name__ == '__main__':
    main()
