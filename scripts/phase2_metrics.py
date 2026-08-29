#!/usr/bin/env python3
"""Coordinated-vs-independent metrics for one map, from finished run dirs.

    python3 scripts/phase2_metrics.py <world> <run_dir> [<run_dir> ...]

Per run: final known area, time to 90% of final, duplicated coverage (area
both rovers observed), and goals sent/failed. A genuinely aligned run is
scored from its published shared-map curve. If independent rovers finish
without rendezvous, their two local-map snapshots are unioned using the TRUE
spawn transform for scoring only; that transform is never published at run
time. Duplication is always evaluated from both saved local maps.
"""
import csv
import glob
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
    origin_text = meta.get('origin', '[0,0,0]').strip('[]')
    origin = [float(v) for v in origin_text.split(',')]
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


def truth_union_area(g1, i1, g2, i2, world):
    """Known union area after scoring both local grids in leo1's frame."""
    grids = [(g1, i1, (0.0, 0.0, 0.0)),
             (g2, i2, relative_offset(world))]
    valid = [(grid, info, transform) for grid, info, transform in grids
             if grid is not None and info is not None and grid.shape != (1, 1)]
    if not valid:
        return None

    resolution = min(float(info[2]) for _, info, _ in valid)
    bounds = bounds_in_robot_frame(world, 'leo1')
    if bounds is None:
        # Unknown worlds fall back to the rectangular extent of both grids.
        corners = []
        for grid, info, (tx, ty, yaw) in valid:
            c, s = math.cos(yaw), math.sin(yaw)
            for x in (info[0], info[0] + grid.shape[1] * info[2]):
                for y in (info[1], info[1] + grid.shape[0] * info[2]):
                    corners.append((tx + c * x - s * y,
                                    ty + s * x + c * y))
        bounds = (min(x for x, _ in corners), max(x for x, _ in corners),
                  min(y for _, y in corners), max(y for _, y in corners))
    xmin, xmax, ymin, ymax = bounds
    width = max(1, int(math.ceil((xmax - xmin) / resolution)))
    height = max(1, int(math.ceil((ymax - ymin) / resolution)))
    union = np.zeros((height, width), dtype=bool)

    for grid, info, (tx, ty, yaw) in valid:
        rows, cols = np.nonzero(grid != -1)
        if not rows.size:
            continue
        x = info[0] + (cols + 0.5) * info[2]
        y = info[1] + (rows + 0.5) * info[2]
        c, s = math.cos(yaw), math.sin(yaw)
        qx = tx + c * x - s * y
        qy = ty + s * x + c * y
        ci = np.floor((qx - xmin) / resolution).astype(int)
        ri = np.floor((qy - ymin) / resolution).astype(int)
        ok = (ci >= 0) & (ci < width) & (ri >= 0) & (ri < height)
        union[ri[ok], ci[ok]] = True
    return int(union.sum()) * resolution * resolution


def truth_union_series(run_dir, world):
    """Scoring-only known-union curve from recorded per-rover snapshots."""
    out = []
    pattern = os.path.join(run_dir, 'timelapse', 'snap*.npz')
    for path in sorted(glob.glob(pattern)):
        try:
            with np.load(path) as snap:
                area = truth_union_area(
                    snap['leo1'], snap['leo1_info'],
                    snap['leo2'], snap['leo2_info'], world)
                if area is not None:
                    out.append((int(round(float(snap['t']))), area))
        except (KeyError, OSError, ValueError):
            continue
    return out


def alignment_locked(run_dir):
    """Whether the runtime ever published a vetted shared-frame lock."""
    path = os.path.join(run_dir, 'alignment.csv')
    if not os.path.exists(path):
        return False
    try:
        with open(path, newline='') as handle:
            return any(row.get('locked', '').strip().lower() in ('1', 'true')
                       for row in csv.DictReader(handle))
    except OSError:
        return False


def evaluation_coverage_series(run_dir, world):
    """Published merge when locked; scoring-only local union otherwise."""
    runtime = coverage_series(os.path.join(run_dir, 'coverage.log'))
    if runtime and alignment_locked(run_dir):
        return runtime
    scored_union = truth_union_series(run_dir, world)
    return scored_union or runtime


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
    if len(sys.argv) < 3:
        print(f'usage: {os.path.basename(sys.argv[0])} <world> <run_dir> '
              f'[run_dir ...]', file=sys.stderr)
        raise SystemExit(2)
    world = sys.argv[1]
    # Every metric here is scored against the authored spawn offset, so an
    # unknown world silently poisons the whole table. Fail loudly instead:
    # the old behaviour was a bare TypeError from unpacking None, several
    # frames deep in duplicated_area().
    if relative_offset(world) is None:
        print(f'unknown world {world!r}: no spawn poses in spawn_poses.py. '
              f'The first argument is the world, then the run directories.',
              file=sys.stderr)
        raise SystemExit(2)
    print(f'{"run":42s} {"final m2":>9s} {"t90 s":>7s} {"dup m2":>8s} '
          f'{"goals":>6s} {"failed":>7s}')
    print('-' * 82)
    for d in sys.argv[2:]:
        series = evaluation_coverage_series(d, world)
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
