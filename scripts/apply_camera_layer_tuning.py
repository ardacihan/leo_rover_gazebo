#!/usr/bin/env python3
"""Narrow the depth camera's costmap band so it cannot box the rover in.

Observation motivating this
---------------------------
Three of eleven autonomous runs stalled: the rover stopped and never restarted
while Nav2 kept issuing goals. In `reports/night/n18_ship_s101` the behaviour
server logged `Collision Ahead` seven times — `BackUp` and `Spin` check the
local costmap before moving and refused, so every recovery aborted and the
rover could not escape.

Re-running the same seed with the camera removed from the costmap
(`ENABLE_VOXEL=false`) gave **zero** `Collision Ahead` aborts and lifted
coverage from 0.519 to 0.866. So the camera layer is part of the deadlock.

Removing the camera is the wrong fix: it is the only sensor that sees a table
crossbar or a chair leg above the 2-D lidar plane. What this script does
instead is narrow *when* it is allowed to mark:

    min_obstacle_height  0.06 -> 0.15   above floor-plane noise; a rover that
                                        pitches over a threshold tips the floor
                                        into a 6 cm band, and a marked floor is
                                        a wall the rover is standing on
    obstacle_max_range   2.5  -> 1.8    depth noise grows with range squared;
                                        marks beyond ~2 m are the least
                                        trustworthy and the least useful
    raytrace_max_range   3.0  (kept)    clearing must out-range marking or a
                                        false mark is never cleared

Usage:
    python3 scripts/apply_camera_layer_tuning.py --profile sim [--revert]
"""

import argparse
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

TUNED = {'min_obstacle_height': 0.15, 'obstacle_max_range': 1.8}
ORIGINAL = {'min_obstacle_height': 0.06, 'obstacle_max_range': 2.5}


def edit(path, values):
    with open(path, encoding='utf-8') as fh:
        lines = fh.readlines()

    # Line-oriented rather than a YAML round-trip: rewriting the file through
    # PyYAML would drop every comment in it, and the comments in nav2.yaml are
    # the record of why each number is what it is.
    in_camera = False
    changed = 0
    for i, line in enumerate(lines):
        stripped = line.strip()
        if stripped == 'camera:':
            in_camera = True
            continue
        if in_camera:
            indent = len(line) - len(line.lstrip())
            if stripped and not stripped.startswith('#') and indent <= 8:
                in_camera = False
                continue
            for key, value in values.items():
                if stripped.startswith(f'{key}:'):
                    pad = ' ' * indent
                    lines[i] = f'{pad}{key}: {value}\n'
                    changed += 1
    if changed:
        with open(path, 'w', encoding='utf-8', newline='\n') as fh:
            fh.writelines(lines)
    return changed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--profile', default='sim', choices=('sim', 'real'))
    ap.add_argument('--revert', action='store_true')
    args = ap.parse_args()

    path = os.path.join(ROOT, 'src', 'leo_nav2_exploration', 'config',
                        args.profile, 'nav2.yaml')
    if not os.path.exists(path):
        print(f'no such file: {path}', file=sys.stderr)
        return 2

    values = ORIGINAL if args.revert else TUNED
    n = edit(path, values)
    print(f'{path}: {n} camera parameter(s) set to '
          f'{"original" if args.revert else "tuned"} values {values}')

    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh)
    for scope in ('local_costmap', 'global_costmap'):
        layer = data.get(scope, {}).get(scope, {}).get('ros__parameters', {}) \
                    .get('obstacle_layer', {})
        if 'camera' in layer:
            cam = layer['camera']
            print(f'  {scope}: min_obstacle_height={cam["min_obstacle_height"]} '
                  f'obstacle_max_range={cam["obstacle_max_range"]} '
                  f'raytrace_max_range={cam["raytrace_max_range"]}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
