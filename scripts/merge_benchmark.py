#!/usr/bin/env python3
"""Score any map-merging method against every saved map pair at once.

Twelve two-rover runs on this branch left a pair of per-robot occupancy grids
behind, and the true transform between them is known exactly -- it is the
difference between the two spawn poses, which the launch file itself used. That
makes a benchmark: a candidate matcher can be judged in seconds against real,
drifted, partially-overlapping maps instead of by waiting 25 minutes for a
simulation that answers one question.

Use it that way. Re-simulating to test a merge change is the single most
expensive mistake available on this project.

    # score the built-in grid matcher on everything
    python3 scripts/merge_benchmark.py

    # score your own: a callable taking (grid1, info1, grid2, info2) and
    # returning (dx, dy, yaw_rad) or None
    python3 scripts/merge_benchmark.py --method mymodule:match

Each grid is (H, W) int8 in the usual occupancy convention (-1 unknown,
0..100), and each info is (origin_x, origin_y, resolution).
"""

import argparse
import importlib
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

from render_multirobot_media import read_pgm            # noqa: E402
from spawn_poses import relative_offset                 # noqa: E402

# run directory -> world. The truth comes from spawn_poses.relative_offset,
# so it can never drift from what the simulator actually did.
RUNS = [
    ('phase1_husarion_coordinated_run3', 'husarion_office'),
    ('phase1_husarion_coordinated_run4', 'husarion_office'),
    ('phase2_depot_coordinated', 'depot_world'),
    ('phase2_depot_coordinated_run2', 'depot_world'),
    ('phase2_depot_coordinated_timelapse', 'depot_world'),
    ('phase2_depot_independent', 'depot_world'),
    ('phase2_depot_showcase', 'depot_world'),
    ('phase2_office_coordinated', 'office_world'),
    ('phase4_depot_fixed', 'depot_world'),
    ('phase4_office_fixed', 'office_world'),
]

# Runs where one rover barely moved: kept out of the headline score because a
# map with almost no structure is not a fair test of a matcher, but still
# listed so nobody is tempted to quietly drop the hard cases.
DEGENERATE = {'phase1_husarion_coordinated', 'phase1_husarion_coordinated_run2'}

PASS_XY = 0.50      # metres, the Phase 1 gate from the original goal doc
PASS_YAW = 10.0     # degrees


def load_map(stem):
    """(grid int8, (origin_x, origin_y, res)) from a map_server pgm/yaml pair."""
    pgm, yaml_path = stem + '.pgm', stem + '.yaml'
    if not (os.path.exists(pgm) and os.path.exists(yaml_path)):
        return None, None
    meta = {}
    for line in open(yaml_path):
        if ':' in line and not line.strip().startswith('#'):
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    res = float(meta.get('resolution', 0.05))
    origin = [float(v) for v in meta.get('origin', '[0,0,0]').strip('[]').split(',')]
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))

    img = read_pgm(pgm)
    p = img.astype(np.float32) / 255.0
    p = p if negate else 1.0 - p
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[p <= free_th] = 0
    grid[p >= occ_th] = 100
    grid = np.flipud(grid)          # row 0 = ymin, matching OccupancyGrid
    return grid, (origin[0], origin[1], res)


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def score(estimate, truth):
    if estimate is None:
        return None, None
    dx, dy, yaw = estimate
    return (math.hypot(dx - truth[0], dy - truth[1]),
            abs(wrap_deg(math.degrees(yaw) - math.degrees(truth[2]))))


def load_method(spec):
    if not spec:
        from grid_match import match_grids     # the project's own matcher
        return match_grids
    mod, _, fn = spec.partition(':')
    return getattr(importlib.import_module(mod), fn or 'match')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--method', default='',
                    help='module:function returning (dx, dy, yaw_rad) or None')
    ap.add_argument('--root', default=os.path.join(ROOT, 'reports', 'multirobot_2026-08-23'))
    ap.add_argument('--include-degenerate', action='store_true')
    args = ap.parse_args()

    try:
        method = load_method(args.method)
    except Exception as exc:
        print(f'could not load method {args.method or "grid_match:match_grids"}: {exc}')
        print('write one that takes (grid1, info1, grid2, info2) and returns '
              '(dx, dy, yaw_rad) mapping map2 into map1, or None to abstain.')
        return 2

    rows, passed, attempted, abstained = [], 0, 0, 0
    runs = list(RUNS)
    if args.include_degenerate:
        runs += [(d, 'husarion_office') for d in sorted(DEGENERATE)]

    for run, world in runs:
        d = os.path.join(args.root, run)
        g1, i1 = load_map(os.path.join(d, 'leo1_map'))
        g2, i2 = load_map(os.path.join(d, 'leo2_map'))
        if g1 is None or g2 is None:
            continue
        truth = relative_offset(world)
        if truth is None:
            continue
        t0 = time.time()
        try:
            est = method(g1, i1, g2, i2)
        except Exception as exc:
            est = None
            print(f'  {run}: raised {type(exc).__name__}: {exc}')
        dt = time.time() - t0
        exy, eyaw = score(est, truth)
        if est is None:
            abstained += 1
            rows.append((run, world, None, None, dt))
            continue
        attempted += 1
        good = exy <= PASS_XY and eyaw <= PASS_YAW
        passed += bool(good)
        rows.append((run, world, exy, eyaw, dt))

    print(f'{"run":38s} {"world":16s} {"xy err":>8s} {"yaw err":>9s} {"s":>6s}')
    print('-' * 82)
    for run, world, exy, eyaw, dt in rows:
        if exy is None:
            print(f'{run:38s} {world:16s} {"abstain":>8s} {"-":>9s} {dt:6.1f}')
        else:
            mark = ' ok' if (exy <= PASS_XY and eyaw <= PASS_YAW) else '   '
            print(f'{run:38s} {world:16s} {exy:7.2f}m {eyaw:8.2f}d {dt:6.1f}{mark}')
    total = len(rows)
    print('-' * 82)
    print(f'  {passed}/{attempted} attempted merges within {PASS_XY} m / {PASS_YAW} deg'
          f'   ({abstained} abstained, {total} pairs)')
    print('  An abstention is a PASS in disguise: refusing an ambiguous match is')
    print('  worth far more than a confident wrong one, which is what the grid')
    print('  matcher produced on 4 of 4 runs (90 and 180 degree flips).')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
