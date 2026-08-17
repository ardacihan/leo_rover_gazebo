#!/usr/bin/env python3
"""Plot a clean sim scan next to degraded versions, for visual review.

The point is to let someone who has watched the physical rover's /scan in RViz
say whether the simulated degradation looks like the real thing. Uses the same
degrade_ranges() the sim runs.

The middle row is the one that matters: it takes a straight wall segment, fits
a line to the clean points, and plots each degraded point's perpendicular
deviation from that line in millimetres. That is exactly the "how fat is the
wall" question, in units you can compare against a real scan.

Usage (inside the container, with ROS sourced):
    python3 /ros2_ws/scripts/plot_scan_realism.py <scan_sample.json> <out.png>
"""

import json
import math
import random
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from sim_realism_scan import degrade_ranges  # noqa: E402

VARIANTS = [
    ('clean sim lidar', 'what the sim has always fed SLAM', None),
    ('sigma 5 mm, 1% dropout', 'a good real unit', dict(
        range_noise=0.005, dropout_rate=0.01)),
    ('sigma 10 mm, 2% dropout', 'moderate', dict(
        range_noise=0.010, dropout_rate=0.02)),
    ('sigma 20 mm, 2% dropout', 'my current default', dict(
        range_noise=0.020, dropout_rate=0.02)),
]


def to_xy(ranges, angle_min, angle_increment):
    pts, angles = [], []
    angle = angle_min
    for r in ranges:
        if r is not None and math.isfinite(r):
            pts.append((r * math.cos(angle), r * math.sin(angle)))
            angles.append(angle)
        angle += angle_increment
    return np.asarray(pts).reshape(-1, 2), np.asarray(angles)


def pick_wall(clean_pts, clean_angles):
    """Find the longest run of collinear points -- a flat wall segment."""
    best = None
    step = clean_angles[1] - clean_angles[0] if len(clean_angles) > 1 else 0.02
    start = 0
    for i in range(1, len(clean_pts) + 1):
        broken = (i == len(clean_pts)
                  or clean_angles[i] - clean_angles[i - 1] > step * 1.6
                  or np.linalg.norm(clean_pts[i] - clean_pts[i - 1]) > 0.35)
        if broken:
            if i - start >= 12:
                seg = clean_pts[start:i]
                span = np.linalg.norm(seg[-1] - seg[0])
                if best is None or span > best[0]:
                    best = (span, start, i)
            start = i
    return best


def deviations(pts, origin, direction):
    normal = np.array([-direction[1], direction[0]])
    rel = pts - origin
    return rel @ normal, rel @ direction


def main():
    sample_path, out_png = sys.argv[1], sys.argv[2]
    with open(sample_path) as fh:
        scans = json.load(fh)
    scan = scans[0]
    a0, da = scan['angle_min'], scan['angle_increment']
    clean = scan['ranges']

    clean_pts, clean_angles = to_xy(clean, a0, da)
    wall = pick_wall(clean_pts, clean_angles)
    if wall is None:
        raise SystemExit('no wall segment found in this scan')
    _, lo, hi = wall
    seg = clean_pts[lo:hi]
    origin = seg.mean(axis=0)
    centred = seg - origin
    direction = np.linalg.svd(centred, full_matrices=False)[2][0]
    lo_angle, hi_angle = clean_angles[lo], clean_angles[hi - 1]

    fig, axes = plt.subplots(3, len(VARIANTS),
                             figsize=(4.9 * len(VARIANTS), 12.6))

    for col, (title, subtitle, kwargs) in enumerate(VARIANTS):
        if kwargs is None:
            ranges = clean
        else:
            ranges, _ = degrade_ranges(clean, a0, da, random.Random(7),
                                       range_max=12.0, self_return=True,
                                       **kwargs)
        pts, angles = to_xy(ranges, a0, da)

        # --- row 0: whole scan ---
        ax = axes[0][col]
        ax.scatter(pts[:, 0], pts[:, 1], s=4, c='#1b6ac9', linewidths=0)
        ax.plot(0, 0, marker='^', ms=12, c='crimson', zorder=5)
        ax.set_title(f'{title}\n({subtitle})', fontsize=12)
        ax.set_aspect('equal')
        ax.set_xlim(-6.5, 6.5)
        ax.set_ylim(-6.5, 6.5)
        ax.grid(alpha=0.25)
        if kwargs is not None:
            arc = np.linspace(math.radians(45), math.radians(82), 40)
            ax.plot(1.1 * np.cos(arc), 1.1 * np.sin(arc), c='darkorange', lw=2.5)
            ax.text(0.2, 1.45, 'camera-bracket\nself-return',
                    color='darkorange', fontsize=9)

        # --- row 1: wall flatness, in millimetres ---
        ax = axes[1][col]
        mask = (angles >= lo_angle) & (angles <= hi_angle)
        perp, along = deviations(pts[mask], origin, direction)
        ax.axhline(0, c='0.5', lw=1)
        ax.scatter(along, perp * 1000.0, s=20, c='#1b6ac9', linewidths=0)
        ax.set_ylim(-90, 90)
        ax.set_xlabel('distance along wall (m)')
        ax.set_ylabel('deviation from the wall (mm)')
        sd = float(np.std(perp * 1000.0)) if len(perp) else 0.0
        ax.set_title(f'flatness of a {np.ptp(along):.1f} m wall segment\n'
                     f'scatter sd = {sd:.1f} mm', fontsize=11)
        ax.grid(alpha=0.25)

        # --- row 2: the same wall in plan view, tight zoom ---
        ax = axes[2][col]
        ax.scatter(along, perp, s=22, c='#1b6ac9', linewidths=0)
        ax.axhline(0, c='crimson', lw=1.2)
        ax.set_ylim(-0.25, 0.25)
        ax.set_aspect(4.0)
        ax.set_xlabel('distance along wall (m)')
        ax.set_ylabel('metres')
        ax.set_title('same wall, plan view (y stretched 4x)', fontsize=11)
        ax.grid(alpha=0.25)

    fig.suptitle('Simulated lidar degradation: does this match your real '
                 'RPLIDAR C1?', fontsize=16)
    plt.tight_layout(rect=[0, 0, 1, 0.975])
    plt.savefig(out_png, dpi=110)
    print('wrote', out_png)


if __name__ == '__main__':
    main()
