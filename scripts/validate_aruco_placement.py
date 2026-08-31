#!/usr/bin/env python3
"""Validate that authored ArUco tags are usable by the rover camera.

The marker YAML is the source of truth for both Gazebo spawning and alignment
evaluation. This check rejects the placement mistakes that otherwise fail
silently during a long run: tags floating off a surface, tags outside the
room, outward-facing tags, inconsistent heights, or an unusual tag count.

Usage:
  validate_aruco_placement.py office_world
  validate_aruco_placement.py depot_world --json report.json
"""

import argparse
import json
import math
import os
import sys

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
from world_ground_truth import rasterize_world  # noqa: E402


def _paths(world):
    config = os.path.join(
        ROOT, 'src', 'leo_rover_exploration', 'config',
        f'mock_markers_{world}.yaml')
    sdf = os.path.join(
        ROOT, 'src', 'leo_rover_gazebo', 'worlds', f'{world}.sdf')
    return config, sdf


def _occupied_near(gt, x, y, radius):
    col = int((x - gt.origin[0]) / gt.res)
    row = int((y - gt.origin[1]) / gt.res)
    cells = max(1, int(math.ceil(radius / gt.res)))
    r0, r1 = max(0, row - cells), min(gt.occ.shape[0], row + cells + 1)
    c0, c1 = max(0, col - cells), min(gt.occ.shape[1], col + cells + 1)
    return bool(gt.occ[r0:r1, c0:c1].any())


def _is_free(gt, x, y):
    col = int((x - gt.origin[0]) / gt.res)
    row = int((y - gt.origin[1]) / gt.res)
    if row < 0 or col < 0 or row >= gt.occ.shape[0] or col >= gt.occ.shape[1]:
        return False
    return not bool(gt.occ[row, col])


def validate(world, *, camera_height=0.20, marker_side=0.20,
             min_count=5, max_count=10, height_tolerance=0.01):
    config, sdf = _paths(world)
    errors, warnings = [], []
    if not os.path.exists(config):
        return {'world': world, 'markers': [], 'errors': [
            f'missing marker config: {config}'], 'warnings': []}
    if not os.path.exists(sdf):
        return {'world': world, 'markers': [], 'errors': [
            f'missing local SDF (cannot validate attachment): {sdf}'],
                'warnings': []}
    with open(config) as fh:
        markers = (yaml.safe_load(fh) or {}).get('markers', [])
    if not min_count <= len(markers) <= max_count:
        errors.append(
            f'expected {min_count}-{max_count} markers, found {len(markers)}')
    ids = [int(m['id']) for m in markers]
    if len(ids) != len(set(ids)):
        errors.append('marker IDs are not unique')
    if any(mid < 0 or mid >= 50 for mid in ids):
        errors.append('DICT_4X4_50 supports IDs 0-49 only')

    heights = [float(m.get('z', 0.0)) for m in markers]
    if heights and max(heights) - min(heights) > height_tolerance:
        errors.append(
            f'marker centres are not at one floor height: {sorted(set(heights))}')
    # Camera vertical FOV derived from 60 degree horizontal FOV and 4:3 image.
    vfov = 2.0 * math.atan(math.tan(1.047 / 2.0) * 3.0 / 4.0)
    closest_view = 0.5
    vertical_half_view = math.tan(vfov / 2.0) * closest_view
    if heights and (abs(heights[0] - camera_height) + marker_side / 2.0
                    > vertical_half_view + marker_side / 2.0):
        errors.append('marker height is outside the camera vertical field of view')
    if any(z - marker_side / 2.0 <= 0.0 for z in heights):
        errors.append('a marker intersects the floor')

    gt = rasterize_world(sdf, res=0.025, z_plane=heights[0] if heights else 0.3,
                         margin=0.25)
    details = []
    for marker in markers:
        mid = int(marker['id'])
        x, y = float(marker['x']), float(marker['y'])
        z, yaw = float(marker['z']), float(marker.get('yaw', 0.0))
        attached = _occupied_near(gt, x, y, radius=0.16)
        # yaw is the readable face normal. Points in front of the tag must be
        # inside free room space, not beyond an exterior wall.
        sightline = []
        for distance in (0.25, 0.50, 0.75):
            sightline.append(_is_free(
                gt, x + distance * math.cos(yaw),
                y + distance * math.sin(yaw)))
        inward = all(sightline)
        inside = (gt.bounds[0] <= x <= gt.bounds[1]
                  and gt.bounds[2] <= y <= gt.bounds[3])
        if not attached:
            errors.append(f'id {mid}: not attached to a wall/object surface')
        if not inside:
            errors.append(f'id {mid}: outside room/world bounds')
        if not inward:
            errors.append(
                f'id {mid}: readable face points into/through a wall or outside')
        details.append({'id': mid, 'x': x, 'y': y, 'z': z,
                        'yaw': yaw, 'attached': attached,
                        'inside': inside, 'camera_sightline_clear': inward})
    return {'world': world, 'marker_count': len(markers), 'ids': ids,
            'common_height_m': heights[0] if heights else None,
            'camera_height_m': camera_height, 'markers': details,
            'errors': errors, 'warnings': warnings}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('--camera-height', type=float, default=0.20)
    ap.add_argument('--marker-side', type=float, default=0.20)
    ap.add_argument('--min-count', type=int, default=5)
    ap.add_argument('--max-count', type=int, default=10)
    ap.add_argument('--json')
    args = ap.parse_args()
    report = validate(
        args.world, camera_height=args.camera_height,
        marker_side=args.marker_side, min_count=args.min_count,
        max_count=args.max_count)
    text = json.dumps(report, indent=2, sort_keys=True)
    if args.json:
        with open(args.json, 'w', encoding='utf-8') as fh:
            fh.write(text + '\n')
    print(text)
    return 1 if report['errors'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
