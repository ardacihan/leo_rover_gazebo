#!/usr/bin/env python3
"""Experiment: can a better polish pull phase2_depot_coordinated under 0.5 m?

Tries, seeded from the same winning mode the benchmark found:
  A. current polish (n=4000)                     -- reference
  B. denser sampling (n=12000), finer last stage
  C. symmetric scoring: average of map2-walls-on-map1 and map1-walls-on-map2
Prints error vs truth for each, on every depot/office pair for context.
"""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

from merge_benchmark import RUNS, load_map, wrap_deg      # noqa: E402
from spawn_poses import relative_offset                   # noqa: E402
import marker_free_matcher as mfm                         # noqa: E402


def polish_sym(g1, i1, g2, i2, dx, dy, yaw, n_pts=6000):
    """Symmetric wall-overlap polish: average of both directions."""
    d1 = mfm._binary_dilate(g1 >= 50, 1)
    d2 = mfm._binary_dilate(g2 >= 50, 1)

    def pts(g, i, n):
        ys, xs = np.nonzero(g >= 50)
        if len(xs) > n:
            pick = np.random.RandomState(0).choice(len(xs), n, replace=False)
            ys, xs = ys[pick], xs[pick]
        return (i[0] + (xs + 0.5) * i[2], i[1] + (ys + 0.5) * i[2])

    p2x, p2y = pts(g2, i2, n_pts)
    p1x, p1y = pts(g1, i1, n_pts)

    def score(tx, ty, th):
        c, s = math.cos(th), math.sin(th)
        qx = tx + c * p2x - s * p2y
        qy = ty + s * p2x + c * p2y
        ci = ((qx - i1[0]) / i1[2]).astype(int)
        ri = ((qy - i1[1]) / i1[2]).astype(int)
        ok = (ci >= 0) & (ci < g1.shape[1]) & (ri >= 0) & (ri < g1.shape[0])
        f = float(d1[ri[ok], ci[ok]].sum()) / len(p2x) if ok.any() else 0.0
        # inverse: map1 walls into map2 frame
        ux = c * (p1x - tx) + s * (p1y - ty)
        uy = -s * (p1x - tx) + c * (p1y - ty)
        ci = ((ux - i2[0]) / i2[2]).astype(int)
        ri = ((uy - i2[1]) / i2[2]).astype(int)
        ok = (ci >= 0) & (ci < g2.shape[1]) & (ri >= 0) & (ri < g2.shape[0])
        g = float(d2[ri[ok], ci[ok]].sum()) / len(p1x) if ok.any() else 0.0
        return 0.5 * (f + g)

    best = (dx, dy, yaw)
    best_s = score(*best)
    stages = ((0.15, math.radians(1.0), 1.05, math.radians(4.0)),
              (0.05, math.radians(0.25), 0.15, math.radians(1.2)),
              (0.015, math.radians(0.08), 0.05, math.radians(0.3)))
    for st, sy, wt, wy in stages:
        cx, cy, cth = best
        for th in np.arange(cth - wy, cth + wy + 1e-9, sy):
            for tx in np.arange(cx - wt, cx + wt + 1e-9, st):
                for ty in np.arange(cy - wt, cy + wt + 1e-9, st):
                    sc = score(tx, ty, th)
                    if sc > best_s:
                        best_s, best = sc, (tx, ty, th)
    return best, best_s


def main():
    root = os.path.join(ROOT, 'reports', 'multirobot_2026-08-23')
    for run, world in RUNS:
        if 'husarion' in run:
            continue
        d = os.path.join(root, run)
        g1, i1 = load_map(os.path.join(d, 'leo1_map'))
        g2, i2 = load_map(os.path.join(d, 'leo2_map'))
        if g1 is None or g2 is None:
            continue
        tx, ty, tyaw = relative_offset(world)
        modes = mfm.candidate_modes(g1, i1, g2, i2)
        best = None
        for scv, mx, my, mth in modes:
            est, hit = mfm.polish(g1, i1, g2, i2, mx, my, mth)
            if est is not None and (best is None or hit > best[0]):
                best = (hit, est, (mx, my, mth))
        if best is None:
            print(f'{run}: no mode')
            continue
        hit, estA, seed = best

        def err(e):
            return (math.hypot(e[0] - tx, e[1] - ty),
                    abs(wrap_deg(math.degrees(e[2] - tyaw))))
        eA = err(estA)
        estB, hitB = mfm.polish(g1, i1, g2, i2, *seed, n_pts=12000)
        eB = err(estB) if estB else (99, 99)
        estC, hitC = polish_sym(g1, i1, g2, i2, *seed)
        eC = err(estC)
        print(f'{run:36s} A(n4k):{eA[0]:5.2f}m/{eA[1]:5.2f}d h={hit:.3f}  '
              f'B(n12k):{eB[0]:5.2f}m/{eB[1]:5.2f}d h={hitB:.3f}  '
              f'C(sym):{eC[0]:5.2f}m/{eC[1]:5.2f}d h={hitC:.3f}')


if __name__ == '__main__':
    main()
