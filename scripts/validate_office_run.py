#!/usr/bin/env python3
"""Validate office corner coverage and bounded exploration effort."""

from __future__ import annotations

import argparse
import ast
import csv
import json
import math
import os

import numpy as np
from PIL import Image


# Structural room corners, inset 0.45 m, expressed in leo1/map. Leo1 starts at
# world (-7, 5) with zero yaw in office_world.
PROBES = {
    'northwest/NW': (-4.55, 2.55), 'northwest/NE': (2.55, 2.55),
    'northwest/SW': (-4.55, -3.35), 'northwest/SE': (2.55, -3.35),
    'north-middle/NW': (3.55, 2.55), 'north-middle/NE': (10.55, 2.55),
    'north-middle/SW': (3.55, -3.35), 'north-middle/SE': (10.55, -3.35),
    'northeast/NW': (11.55, 2.55), 'northeast/NE': (18.55, 2.55),
    'northeast/SW': (11.55, -3.25), 'northeast/SE': (18.55, -3.25),
    'southwest/NW': (-4.55, -6.75), 'southwest/NE': (6.55, -6.75),
    'southwest/SW': (-4.55, -12.55), 'southwest/SE': (6.55, -12.55),
    'southeast/NW': (7.55, -6.75), 'southeast/NE': (18.55, -6.75),
    'southeast/SW': (7.55, -12.55), 'southeast/SE': (18.55, -12.55),
}


def _map(run):
    meta = {}
    with open(os.path.join(run, 'merged_map.yaml')) as fh:
        for line in fh:
            if ':' in line:
                key, value = line.split(':', 1)
                meta[key.strip()] = value.strip()
    image = np.asarray(Image.open(os.path.join(run, 'merged_map.pgm')))
    if image.ndim == 3:
        image = image[..., 0]
    # map_server PGMs put the grid's ymax at image row zero.
    known = np.flipud(image != 205)
    origin = ast.literal_eval(meta['origin'])
    return known, float(origin[0]), float(origin[1]), float(meta['resolution'])


def _path_length(path):
    if not os.path.isfile(path):
        return 0.0
    points = []
    for row in csv.DictReader(open(path)):
        try:
            points.append((float(row['x']), float(row['y'])))
        except (KeyError, TypeError, ValueError):
            pass
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(points, points[1:]))


def validate(run, min_patch=0.60, min_mean=0.90, max_distance=300.0):
    out = {'run': os.path.abspath(run), 'ok': False}
    try:
        known, ox, oy, res = _map(run)
    except (OSError, KeyError, ValueError, SyntaxError) as exc:
        out['reason'] = f'missing or invalid merged map: {exc}'
        return out
    depth = max(1, int(round(0.50 / res)))
    probes = []
    for name, (x, y) in PROBES.items():
        col, row = int((x - ox) / res), int((y - oy) / res)
        inside = 0 <= row < known.shape[0] and 0 <= col < known.shape[1]
        center = bool(known[row, col]) if inside else False
        if inside:
            # Measure the room-facing quadrant from the inset probe. A
            # symmetric patch at an outer corner includes cells behind the
            # exterior wall and perversely rewards lidar leaks / boundary
            # tracing. The suffix is the structural corner, so its opposite
            # direction points into reachable room interior.
            corner = name.rsplit('/', 1)[-1]
            dc = 1 if 'W' in corner else -1
            dr = -1 if 'N' in corner else 1
            c0, c1 = sorted((col, col + dc * depth))
            r0, r1 = sorted((row, row + dr * depth))
            r0, r1 = max(0, r0), min(known.shape[0], r1 + 1)
            c0, c1 = max(0, c0), min(known.shape[1], c1 + 1)
            fraction = float(known[r0:r1, c0:c1].mean())
        else:
            fraction = 0.0
        probes.append({'name': name, 'x': x, 'y': y,
                       'center_known': center, 'patch_known': fraction})
    fractions = [p['patch_known'] for p in probes]
    distances = {robot: _path_length(os.path.join(run, f'traj_{robot}.csv'))
                 for robot in ('leo1', 'leo2')}
    total_distance = sum(distances.values())
    explorer_path = os.path.join(run, 'explorer.log')
    explorer = open(explorer_path, errors='replace').read() \
        if os.path.isfile(explorer_path) else ''
    completed = explorer.count('Exploration finished.') >= 2
    aborted = 'Exploration aborted:' in explorer
    out.update({
        'probes': probes,
        'all_centers_known': all(p['center_known'] for p in probes),
        'min_patch_known': min(fractions),
        'mean_patch_known': float(np.mean(fractions)),
        'distance_m': distances,
        'total_distance_m': total_distance,
        'explorers_completed': completed,
        'explorer_aborted': aborted,
    })
    failures = []
    if not out['all_centers_known']:
        failures.append('one or more room-corner centers remain unknown')
    if out['min_patch_known'] < min_patch:
        failures.append(f"minimum corner patch {out['min_patch_known']:.1%} < {min_patch:.0%}")
    if out['mean_patch_known'] < min_mean:
        failures.append(f"mean corner coverage {out['mean_patch_known']:.1%} < {min_mean:.0%}")
    if total_distance > max_distance:
        failures.append(f'total trajectory {total_distance:.1f} m > {max_distance:.0f} m')
    if not completed or aborted:
        failures.append('both explorers did not complete normally')
    out['ok'] = not failures
    out['reason'] = '; '.join(failures) if failures else 'office exploration validated'
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('runs', nargs='+')
    parser.add_argument('--min-patch', type=float, default=0.60)
    parser.add_argument('--min-mean', type=float, default=0.90)
    parser.add_argument('--max-distance', type=float, default=300.0)
    args = parser.parse_args()
    rows = [validate(r, args.min_patch, args.min_mean, args.max_distance)
            for r in args.runs]
    print(json.dumps(rows if len(rows) > 1 else rows[0], indent=2))
    return 0 if all(row['ok'] for row in rows) else 2


if __name__ == '__main__':
    raise SystemExit(main())
