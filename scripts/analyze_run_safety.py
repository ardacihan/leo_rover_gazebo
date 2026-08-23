#!/usr/bin/env python3
"""Score how *safely* a run drove, independent of which stack drove it.

`eval_map.py` answers "is the map right?". This answers "did the rover behave?"
-- the part the real Leo Rover actually has to survive. Everything is derived
from the ground-truth pose column of ``pose_error.csv`` and the world SDF, so a
run from the original explore_lite stack and one from the Nav2 overlay are
scored by exactly the same rule.

Reported per run:

  min_clearance_m     closest the robot body ever came to real geometry
  contacts            samples with clearance <= --contact (body touching a wall)
  near_misses         samples with clearance <= --near  (scraping distance)
  contact_events      contiguous runs of contact samples, i.e. distinct bumps
  stuck_events        >= --stuck-sec of continuous motion below --stuck-speed
  stuck_frac          fraction of the run spent not moving
  path_len_m          ground-truth distance travelled
  doorway_passes      transitions through a cell whose clearance < --doorway,
                      i.e. how many genuinely narrow gaps it committed to
  explored_extent_m2  area of the convex-ish footprint the rover reached

Usage:
    python3 analyze_run_safety.py <pose_error.csv> <world.sdf> [--json out.json]
"""

import argparse
import csv
import json
import math

import numpy as np
from scipy import ndimage

from world_ground_truth import rasterize_world


def load_gt(path):
    ts, xs, ys = [], [], []
    with open(path, newline='') as fh:
        for row in csv.DictReader(fh):
            try:
                t = float(row['t'])
                x = float(row['gt_x'])
                y = float(row['gt_y'])
            except (TypeError, ValueError, KeyError):
                continue
            if math.isnan(x) or math.isnan(y):
                continue
            ts.append(t)
            xs.append(x)
            ys.append(y)
    return np.asarray(ts), np.asarray(xs), np.asarray(ys)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pose_csv')
    ap.add_argument('world')
    # Leo Rover is a ~0.44 m square; 0.22 m is the half-width, so a body-centre
    # clearance at or below that means the chassis is inside the obstacle.
    ap.add_argument('--contact', type=float, default=0.22)
    ap.add_argument('--near', type=float, default=0.30)
    # A gap the rover can only take deliberately: less than a body width of
    # slack on each side.
    ap.add_argument('--doorway', type=float, default=0.55)
    ap.add_argument('--stuck-speed', type=float, default=0.02)
    ap.add_argument('--stuck-sec', type=float, default=20.0)
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--z', type=float, default=0.5)
    ap.add_argument('--json')
    args = ap.parse_args()

    ts, xs, ys = load_gt(args.pose_csv)
    if len(ts) < 2:
        raise SystemExit(f'{args.pose_csv}: fewer than 2 usable pose samples')

    gt = rasterize_world(args.world, res=args.res, z_plane=args.z)
    clearance = ndimage.distance_transform_edt(~gt.occ) * gt.res
    rows, cols = clearance.shape

    def clearance_at(x, y):
        col = int(round((x - gt.origin[0]) / gt.res - 0.5))
        row = int(round((y - gt.origin[1]) / gt.res - 0.5))
        if not (0 <= row < rows and 0 <= col < cols):
            # Outside the rasterised world: treat as wide open rather than as a
            # phantom collision.
            return float('inf')
        return float(clearance[row, col])

    clr = np.asarray([clearance_at(x, y) for x, y in zip(xs, ys)])
    finite = clr[np.isfinite(clr)]

    steps = np.hypot(np.diff(xs), np.diff(ys))
    dts = np.diff(ts)
    dts[dts <= 0] = np.nan
    speeds = steps / dts

    contact_mask = clr <= args.contact
    # Contiguous contact samples are one bump, not N bumps.
    contact_events = int(np.sum(np.diff(contact_mask.astype(int)) == 1)
                         + (1 if contact_mask.size and contact_mask[0] else 0))

    slow = np.nan_to_num(speeds, nan=0.0) < args.stuck_speed
    stuck_events, stuck_time, run_start = 0, 0.0, None
    for i, is_slow in enumerate(slow):
        if is_slow and run_start is None:
            run_start = i
        elif not is_slow and run_start is not None:
            span = ts[i] - ts[run_start]
            if span >= args.stuck_sec:
                stuck_events += 1
                stuck_time += span
            run_start = None
    if run_start is not None:
        span = ts[-1] - ts[run_start]
        if span >= args.stuck_sec:
            stuck_events += 1
            stuck_time += span

    narrow = clr < args.doorway
    doorway_passes = int(np.sum(np.diff(narrow.astype(int)) == 1))

    duration = float(ts[-1] - ts[0])
    # Area actually visited, as the footprint of the trajectory on the grid.
    visited = set()
    for x, y in zip(xs, ys):
        visited.add((int((x - gt.origin[0]) / gt.res), int((y - gt.origin[1]) / gt.res)))

    result = {
        'samples': int(len(ts)),
        'duration_s': round(duration, 1),
        'path_len_m': round(float(np.nansum(steps)), 2),
        'min_clearance_m': round(float(finite.min()), 3) if finite.size else None,
        'p05_clearance_m': round(float(np.percentile(finite, 5)), 3) if finite.size else None,
        'contacts': int(contact_mask.sum()),
        'contact_events': contact_events,
        'near_misses': int((clr <= args.near).sum()),
        'doorway_passes': doorway_passes,
        'stuck_events': stuck_events,
        'stuck_time_s': round(stuck_time, 1),
        'stuck_frac': round(stuck_time / duration, 3) if duration > 0 else None,
        'mean_speed_mps': round(float(np.nanmean(speeds)), 3),
        'visited_area_m2': round(len(visited) * args.res * args.res, 2),
    }

    for key, value in result.items():
        print(f'{key:22s} {value}')

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(result, fh, indent=2)
        print(f'\nwrote {args.json}')


if __name__ == '__main__':
    main()
