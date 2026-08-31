#!/usr/bin/env python3
"""Score whether an accepted merge is geometrically overlaid, not just locked.

Loads the two local occupancy maps and the last accepted transform, then
prints residual / geometry_ok. A lock with residual > 0.10 m is a fail.
"""

from __future__ import annotations

import csv
import json
import math
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__),
                                '..', 'src', 'multi_robot_shared_mapping'))
from multi_robot_shared_mapping.geometric_residual import (  # noqa: E402
    geometric_lock_ok, residual_stats)


def _load_grid(run, name):
    yaml_path = os.path.join(run, f'{name}.yaml')
    pgm_path = os.path.join(run, f'{name}.pgm')
    if not os.path.isfile(yaml_path) or not os.path.isfile(pgm_path):
        return None, None
    meta = {}
    for line in open(yaml_path):
        if ':' in line:
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    origin = eval(meta.get('origin', '[0, 0, 0]'))
    res = float(meta['resolution'])
    img = np.array(Image.open(pgm_path))
    if img.ndim == 3:
        img = img[..., 0]
    # PGM: 254 free, 0 occupied, 205 unknown -> occupancy int8.
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[img >= 250] = 0
    grid[img <= 50] = 100
    info = (float(origin[0]), float(origin[1]), res, 0.0)
    return np.flipud(grid), info


def _last_transform(run):
    path = os.path.join(run, 'alignment.csv')
    if not os.path.isfile(path):
        return None
    rows = list(csv.DictReader(open(path)))
    for row in reversed(rows):
        try:
            return (float(row['map_x']), float(row['map_y']),
                    math.radians(float(row['map_yaw_deg'])))
        except (TypeError, ValueError, KeyError):
            try:
                return (float(row['tag_x']), float(row['tag_y']),
                        math.radians(float(row['tag_yaw_deg'])))
            except (TypeError, ValueError, KeyError):
                continue
    return None


def score_run(run):
    g1, i1 = _load_grid(run, 'leo1_map')
    g2, i2 = _load_grid(run, 'leo2_map')
    tf = _last_transform(run)
    out = {'run': run, 'transform': tf, 'ok': False, 'reason': 'missing maps'}
    if g1 is None or g2 is None or tf is None:
        return out
    stats = residual_stats(g1, i1, g2, i2, tf)
    ok, reason = geometric_lock_ok(stats)
    out.update(stats)
    out['ok'] = ok
    out['reason'] = reason
    out['transform'] = [round(v, 4) for v in tf]
    return out


def main():
    runs = sys.argv[1:] or ['.']
    results = [score_run(os.path.abspath(r)) for r in runs]
    print(json.dumps(results if len(results) > 1 else results[0], indent=2))
    if any(not r['ok'] for r in results):
        sys.exit(2)


if __name__ == '__main__':
    main()
