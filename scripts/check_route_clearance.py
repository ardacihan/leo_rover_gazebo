#!/usr/bin/env python3
"""Check a scripted_drive route against a world's collision geometry.

scripted_drive.py has no obstacle avoidance by design -- it must follow the
identical trajectory in every run. That makes a route that clips a desk a silent
experiment-killer: the rover wedges against it and the run burns its wall-clock
cap. Run this before trusting a route.

Usage:
    python3 check_route_clearance.py <world.sdf> [route] [--radius 0.28] [--png out.png]
"""

import argparse
import math

import numpy as np
from scipy import ndimage

from scripted_drive import ROUTES
from world_ground_truth import rasterize_world


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('route', nargs='?', default='office_full')
    # Leo Rover is ~0.44 m wide; 0.28 m adds a small margin to the half-width.
    ap.add_argument('--radius', type=float, default=0.28)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--z', type=float, default=0.5)
    ap.add_argument('--step', type=float, default=0.05)
    ap.add_argument('--png')
    args = ap.parse_args()

    gt = rasterize_world(args.world, res=args.res, z_plane=args.z)
    # Distance in metres from every cell to the nearest obstacle.
    clearance = ndimage.distance_transform_edt(~gt.occ) * gt.res

    def clearance_at(x, y):
        col = int(round((x - gt.origin[0]) / gt.res - 0.5))
        row = int(round((y - gt.origin[1]) / gt.res - 0.5))
        if not (0 <= row < clearance.shape[0] and 0 <= col < clearance.shape[1]):
            return float('inf')
        return float(clearance[row, col])

    waypoints = [(0.0, 0.0)] + list(ROUTES[args.route])
    worst = []
    samples = []
    for i in range(len(waypoints) - 1):
        (x0, y0), (x1, y1) = waypoints[i], waypoints[i + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        n = max(int(length / args.step), 1)
        leg_min, leg_at = float('inf'), None
        for k in range(n + 1):
            t = k / n
            x, y = x0 + (x1 - x0) * t, y0 + (y1 - y0) * t
            c = clearance_at(x, y)
            samples.append((x, y, c))
            if c < leg_min:
                leg_min, leg_at = c, (x, y)
        worst.append((leg_min, i, (x0, y0), (x1, y1), leg_at))

    print(f'route {args.route}: {len(waypoints) - 1} legs, '
          f'required clearance {args.radius:.2f} m\n')
    bad = [w for w in worst if w[0] < args.radius]
    for leg_min, i, a, b, at in sorted(worst)[:8]:
        flag = 'BLOCKED' if leg_min < args.radius else 'ok'
        print(f'  [{flag:>7}] leg {i:2d} {a} -> {b}: '
              f'min clearance {leg_min:.2f} m at ({at[0]:.2f}, {at[1]:.2f})')

    total = sum(math.hypot(waypoints[i + 1][0] - waypoints[i][0],
                           waypoints[i + 1][1] - waypoints[i][1])
                for i in range(len(waypoints) - 1))
    print(f'\ntotal path length {total:.1f} m')
    print(f'{len(bad)} blocked leg(s)')

    if args.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        extent = [gt.origin[0], gt.origin[0] + gt.occ.shape[1] * gt.res,
                  gt.origin[1], gt.origin[1] + gt.occ.shape[0] * gt.res]
        plt.figure(figsize=(13, 9))
        plt.imshow(gt.occ, origin='lower', cmap='gray_r', extent=extent,
                   alpha=0.75)
        arr = np.asarray(samples)
        ok = arr[arr[:, 2] >= args.radius]
        no = arr[arr[:, 2] < args.radius]
        plt.plot(ok[:, 0], ok[:, 1], '.', ms=2.5, c='#1b6ac9', label='clear')
        if len(no):
            plt.plot(no[:, 0], no[:, 1], '.', ms=7, c='crimson',
                     label='blocked')
        wp = np.asarray(waypoints)
        plt.plot(wp[:, 0], wp[:, 1], 'o', ms=4, c='darkorange',
                 label='waypoints')
        plt.legend(loc='upper right')
        plt.gca().set_aspect('equal')
        plt.grid(alpha=0.2)
        plt.title(f'{args.route}: {total:.0f} m, {len(bad)} blocked leg(s)')
        plt.tight_layout()
        plt.savefig(args.png, dpi=120)
        print('wrote', args.png)

    raise SystemExit(1 if bad else 0)


if __name__ == '__main__':
    main()
