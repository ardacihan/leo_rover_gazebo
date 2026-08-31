#!/usr/bin/env python3
"""Verify authored ArUco markers sit flush on real walls.

A marker is a plate GLUED to a wall: its yaw must equal the wall's outward
normal. `validate_aruco_placement.py` can only rasterize authored box worlds;
this checker instead uses a SLAM map from any good run, so it works for mesh
worlds (husarion) too. For every marker it fits the local wall direction from
occupied cells within 0.35 m and measures the facing-vs-normal angle.

    python3 scripts/check_marker_orientation.py <mock_markers.yaml> <map.yaml> \
        [--spawn-x 0 --spawn-y 0 --spawn-yaw 0]

The map must be in a frame anchored at --spawn (leo1's spawn pose for that
world; defaults fit husarion where leo1 spawns at the world origin).
Exit 1 when any marker is >30 deg off flush or has no wall within 0.35 m.
"""

import argparse
import math
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_multirobot_media import read_pgm  # noqa: E402


def load_map(map_yaml):
    origin = [0.0, 0.0]
    res = 0.05
    for line in open(map_yaml):
        if line.startswith('origin'):
            origin = [float(v) for v in
                      line.split('[')[1].split(']')[0].split(',')[:2]]
        if line.startswith('resolution'):
            res = float(line.split(':')[1])
    img = read_pgm(os.path.join(os.path.dirname(map_yaml),
                                os.path.basename(map_yaml).replace('.yaml', '.pgm')))
    return np.flipud(img), origin, res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('markers_yaml')
    ap.add_argument('map_yaml')
    ap.add_argument('--spawn-x', type=float, default=0.0)
    ap.add_argument('--spawn-y', type=float, default=0.0)
    ap.add_argument('--spawn-yaw', type=float, default=0.0)
    ap.add_argument('--max-off-deg', type=float, default=30.0)
    args = ap.parse_args()

    grid, origin, res = load_map(args.map_yaml)
    occ = grid <= 50
    h, w = grid.shape
    data = yaml.safe_load(open(args.markers_yaml))
    c0, s0 = math.cos(-args.spawn_yaw), math.sin(-args.spawn_yaw)

    bad = 0
    for m in data.get('markers', []):
        wx, wy = float(m['x']), float(m['y'])
        yaw = float(m.get('yaw', 0.0))
        # world -> map frame (map anchored at leo1 spawn)
        x = c0 * (wx - args.spawn_x) - s0 * (wy - args.spawn_y)
        y = s0 * (wx - args.spawn_x) + c0 * (wy - args.spawn_y)
        myaw = yaw - args.spawn_yaw
        r, c = int((y - origin[1]) / res), int((x - origin[0]) / res)
        R = int(0.35 / res)
        pts = [(cc * res + origin[0], rr * res + origin[1])
               for rr in range(max(0, r - R), min(h, r + R + 1))
               for cc in range(max(0, c - R), min(w, c + R + 1))
               if occ[rr, cc]]
        if len(pts) < 4:
            print(f"id {m['id']:>2} ({wx:6.2f},{wy:6.2f}): FAIL - no wall "
                  f"within 0.35 m (floating marker)")
            bad += 1
            continue
        P = np.array(pts)
        _, _, vt = np.linalg.svd(P - P.mean(axis=0))
        wall_dir = vt[0]
        normal = math.atan2(wall_dir[0], -wall_dir[1])
        d = (math.degrees(myaw) - math.degrees(normal)) % 180
        d = min(d, 180 - d)
        ok = d <= args.max_off_deg
        print(f"id {m['id']:>2} ({wx:6.2f},{wy:6.2f}): "
              f"{'ok  ' if ok else 'FAIL'} facing is {d:5.1f} deg off the "
              f"wall normal")
        bad += 0 if ok else 1
    if bad:
        print(f"{bad} marker(s) not flush on a wall")
    return 1 if bad else 0


if __name__ == '__main__':
    raise SystemExit(main())
