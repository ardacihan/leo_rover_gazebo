#!/usr/bin/env python3
"""Tuning table for marker_free_matcher: every mode vs ground truth.

For each benchmark pair, print the global-search modes with their coarse
score, full-res polished hit, and error against the true transform -- the
evidence for choosing POLISH_HIT_MIN and MARGIN_MIN honestly instead of
fitting them to the pass column.
"""
import math
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

from merge_benchmark import RUNS, load_map, wrap_deg          # noqa: E402
from spawn_poses import relative_offset                       # noqa: E402
import marker_free_matcher as mfm                             # noqa: E402


def main():
    root = os.path.join(ROOT, 'reports', 'multirobot_2026-08-23')
    for run, world in RUNS:
        d = os.path.join(root, run)
        g1, i1 = load_map(os.path.join(d, 'leo1_map'))
        g2, i2 = load_map(os.path.join(d, 'leo2_map'))
        if g1 is None or g2 is None:
            print(f'{run}: maps missing')
            continue
        tx, ty, tyaw = relative_offset(world)
        t0 = time.time()
        modes = mfm.candidate_modes(g1, i1, g2, i2)
        t_glob = time.time() - t0
        print(f'\n== {run} ({world})  truth=({tx:.2f},{ty:.2f},{math.degrees(tyaw):.1f}d)'
              f'  global={t_glob:.1f}s')
        for k, (scv, mx, my, mth) in enumerate(modes):
            t1 = time.time()
            est, hit = mfm.polish(g1, i1, g2, i2, mx, my, mth)
            t_pol = time.time() - t1
            if est is None:
                print(f'  mode{k}: coarse={scv:.3f} polish failed')
                continue
            exy = math.hypot(est[0] - tx, est[1] - ty)
            eyaw = abs(wrap_deg(math.degrees(est[2] - tyaw)))
            tag = 'TRUE' if (exy <= 0.5 and eyaw <= 10) else ('near' if (exy <= 1.0 and eyaw <= 15) else 'WRONG')
            print(f'  mode{k}: coarse={scv:6.3f} hit={hit:.3f} '
                  f'yaw={math.degrees(est[2]):7.1f} t=({est[0]:7.2f},{est[1]:7.2f}) '
                  f'err={exy:5.2f}m/{eyaw:6.2f}d {tag}  ({t_pol:.1f}s)')


if __name__ == '__main__':
    main()
