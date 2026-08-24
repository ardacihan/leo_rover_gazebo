#!/usr/bin/env python3
"""Coordinated-vs-independent metrics for one map, from finished run dirs.

    python3 scripts/phase2_metrics.py <world> <run_dir> [<run_dir> ...]

Per run: final known area, time to 90% of final, duplicated coverage (area
both rovers observed — the overlap the shared map is supposed to eliminate),
goals sent/failed. Duplicated coverage is computed from the two saved
per-rover maps with the TRUE spawn transform, so it measures behaviour, not
alignment quality.
"""
import math
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

from render_multirobot_media import read_pgm                    # noqa: E402
from spawn_poses import bounds_in_robot_frame, relative_offset  # noqa: E402


def load_map(stem):
    """Strict trinary loader for map_saver output.

    merge_benchmark.load_map classifies by the yaml thresholds, and the
    saver writes free_thresh 0.25 -- the unknown pixel (205 -> p=0.196)
    lands BELOW that and is read as free. Harmless for wall matching,
    fatal for a known-area metric. Here: pixel 254 = free, 0 = occupied,
    everything else (205) = unknown.
    """
    import numpy as np
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
    img = read_pgm(pgm)
    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[img >= 250] = 0
    grid[img <= 10] = 100
    grid = np.flipud(grid)
    return grid, (origin[0], origin[1], res)


def coverage_series(path):
    """[(t, m2)] from a map_coverage.py log."""
    out = []
    if not os.path.exists(path):
        return out
    for line in open(path, errors='replace'):
        m = re.search(r't=(\d+)s known=([0-9.]+)m2', line)
        if m:
            out.append((int(m.group(1)), float(m.group(2))))
    return out


def time_to_frac(series, frac):
    if not series:
        return None
    final = series[-1][1]
    target = frac * final
    for t, v in series:
        if v >= target:
            return t
    return None


def duplicated_area(d, world):
    """m^2 known by BOTH rovers, under the true transform."""
    g1, i1 = load_map(os.path.join(d, 'leo1_map'))
    g2, i2 = load_map(os.path.join(d, 'leo2_map'))
    if g1 is None or g2 is None:
        return None
    tx, ty, tyaw = relative_offset(world)
    known1 = g1 != -1
    ys, xs = np.nonzero(g2 != -1)
    if xs.size == 0:
        return 0.0
    px = i2[0] + (xs + 0.5) * i2[2]
    py = i2[1] + (ys + 0.5) * i2[2]
    c, s = math.cos(tyaw), math.sin(tyaw)
    qx = tx + c * px - s * py
    qy = ty + s * px + c * py
    ci = ((qx - i1[0]) / i1[2]).astype(int)
    ri = ((qy - i1[1]) / i1[2]).astype(int)
    ok = (ci >= 0) & (ci < g1.shape[1]) & (ri >= 0) & (ri < g1.shape[0])
    # dedup onto map1's grid: several fine map2 cells land on one map1 cell
    seen2 = np.zeros_like(known1)
    seen2[ri[ok], ci[ok]] = True
    both = known1 & seen2
    # clip to the world box (in leo1's frame), same as the coverage metric:
    # 12 m lidar rays leaking through doorways mark phantom free space far
    # outside the building and would swamp the comparison.
    b = bounds_in_robot_frame(world, 'leo1')
    if b:
        xmin, xmax, ymin, ymax = b
        rows, cols = np.indices(both.shape)
        wx = i1[0] + (cols + 0.5) * i1[2]
        wy = i1[1] + (rows + 0.5) * i1[2]
        both &= (wx >= xmin) & (wx <= xmax) & (wy >= ymin) & (wy <= ymax)
    return int(both.sum()) * i1[2] * i1[2]


def goal_stats(d):
    log = os.path.join(d, 'explorer.log')
    sent = failed = 0
    if os.path.exists(log):
        for line in open(log, errors='replace'):
            if 'New frontier goal:' in line:
                sent += 1
            if 'goal failed' in line.lower():
                failed += 1
    return sent, failed


def main():
    world = sys.argv[1]
    print(f'{"run":42s} {"final m2":>9s} {"t90 s":>7s} {"dup m2":>8s} '
          f'{"goals":>6s} {"failed":>7s}')
    print('-' * 82)
    for d in sys.argv[2:]:
        series = coverage_series(os.path.join(d, 'coverage.log'))
        final = series[-1][1] if series else None
        t90 = time_to_frac(series, 0.90)
        dup = duplicated_area(d, world)
        sent, failed = goal_stats(d)
        name = os.path.basename(d.rstrip('/\\'))
        print(f'{name:42s} '
              f'{final if final is not None else float("nan"):9.1f} '
              f'{t90 if t90 is not None else -1:7d} '
              f'{dup if dup is not None else float("nan"):8.1f} '
              f'{sent:6d} {failed:7d}')


if __name__ == '__main__':
    main()
