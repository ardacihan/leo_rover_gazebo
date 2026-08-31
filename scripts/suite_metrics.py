#!/usr/bin/env python3
"""Per-run metric extraction for the overnight suite -> one JSON cache.

Everything the paper figures and the analysis page need, computed once:

  team coverage over time   union of both rovers' maps, overlaid with the TRUE
                            spawn offset (ground truth used only to MEASURE, so
                            the number means the same thing in every condition,
                            merged or not)
  duplicated coverage       area both rovers mapped (the work done twice)
  alignment error over time from alignment.csv (vs the true offset)
  distances, finish flags, marker detections, planner failures, bans

Usage: python3 scripts/suite_metrics.py [--out final/suite_metrics.json] [-j 8]
"""

import argparse
import csv
import glob
import json
import math
import os
import re
import sys
from multiprocessing import Pool

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))

RES = 0.1
CONDS = ('single', 'indep', 'c2u', 'c2k', 'tag', 'mfree')


def world_of(run_name, bundle):
    if '_l3_' in run_name:
        return 'small_house_l3'
    if '_l9_' in run_name:
        return 'small_house_l9'
    if '_l15_' in run_name:
        return 'small_house_l15'
    if '_sh_' in run_name:
        return 'small_house'
    if '_office_' in run_name:
        return 'office_world'
    if '_depot_' in run_name:
        return 'depot_world'
    if '_husarion_' in run_name:
        return 'husarion_office'
    return 'office_world'


def geom(world):
    import spawn_poses
    box = spawn_poses.WORLD_BOUNDS[world]
    r1 = spawn_poses.bounds_in_robot_frame(world, 'leo1')
    off = spawn_poses.relative_offset(world)
    return r1, off, (box[1] - box[0]) * (box[3] - box[2])


def coverage_series(run, world):
    """[(t, union, only1, only2, both)] m^2, ground-truth overlay."""
    rect, off, _ = geom(world)
    x0, x1, y0, y1 = rect
    W, H = int((x1 - x0) / RES) + 1, int((y1 - y0) / RES) + 1
    c, s = math.cos(off[2]), math.sin(off[2])
    out = []
    snaps = sorted(glob.glob(os.path.join(run, 'timelapse', 'snap*.npz')))
    for path in snaps[::2]:
        try:
            d = np.load(path)
        except (OSError, ValueError):
            continue
        masks = []
        for key in ('leo1', 'leo2'):
            m = np.zeros((H, W), dtype=bool)
            if key in d.files and d[key].size > 1:
                g, info = d[key], d[f'{key}_info']
                ys, xs = np.nonzero(g >= 0)
                if len(ys):
                    wx = info[0] + (xs + 0.5) * info[2]
                    wy = info[1] + (ys + 0.5) * info[2]
                    if key == 'leo2':
                        wx, wy = off[0] + c * wx - s * wy, off[1] + s * wx + c * wy
                    ci = ((wx - x0) / RES).astype(int)
                    ri = ((wy - y0) / RES).astype(int)
                    ok = (ci >= 0) & (ci < W) & (ri >= 0) & (ri < H)
                    m[ri[ok], ci[ok]] = True
            masks.append(m)
        a, b = masks
        cell = RES * RES
        out.append((float(d['t']), float((a | b).sum()) * cell,
                    float((a & ~b).sum()) * cell, float((b & ~a).sum()) * cell,
                    float((a & b).sum()) * cell))
    if out:
        t0 = out[0][0]
        out = [(row[0] - t0,) + tuple(row[1:]) for row in out]
    return out


def alignment_series(run):
    """[(t, err_xy_m, locked)] and first-lock time, from alignment.csv."""
    path = os.path.join(run, 'alignment.csv')
    if not os.path.exists(path):
        return [], None
    rows, t0, lock_t = [], None, None
    try:
        for r in csv.DictReader(open(path)):
            try:
                t = float(r['t'])
            except (KeyError, ValueError):
                continue
            t0 = t if t0 is None else t0
            locked = str(r.get('locked', '0')).strip() == '1'
            err = None
            try:
                err = float(r['err_xy_m']) if r.get('err_xy_m') else None
            except ValueError:
                err = None
            if locked and lock_t is None:
                lock_t = t - t0
            rows.append((t - t0, err, locked))
    except OSError:
        return [], None
    return rows, lock_t


def traj_len(path):
    if not os.path.exists(path):
        return 0.0
    pts = []
    try:
        for r in csv.DictReader(open(path)):
            pts.append((float(r['x']), float(r['y'])))
    except (OSError, KeyError, ValueError):
        return 0.0
    return sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(pts, pts[1:]))


def count_in(path, pattern):
    if not os.path.exists(path):
        return 0
    rx = re.compile(pattern)
    try:
        return sum(1 for line in open(path, errors='replace') if rx.search(line))
    except OSError:
        return 0


def markers(run, world):
    """(n_detected_true, mean_err_m) against that world's own marker truth."""
    try:
        from render_multirobot_media import markers_for, final_locked_transform
    except ImportError:
        return 0, None
    truth = {i: (x, y) for i, x, y in markers_for(world)}
    if not truth:
        return 0, None
    det = {}
    tf = final_locked_transform(run)
    for robot in ('leo1', 'leo2'):
        p = os.path.join(run, f'aruco_registry_{robot}.json')
        if not os.path.exists(p):
            continue
        try:
            data = json.load(open(p))
        except (OSError, json.JSONDecodeError):
            continue
        for m in data.get('markers', []):
            x, y = float(m['x']), float(m['y'])
            if robot == 'leo2':
                if not tf:
                    continue
                yaw = math.radians(tf[2])
                c, s = math.cos(yaw), math.sin(yaw)
                x, y = tf[0] + c * x - s * y, tf[1] + s * x + c * y
            det.setdefault(int(m['id']), (x, y))
    errs = [math.hypot(det[i][0] - truth[i][0], det[i][1] - truth[i][1])
            for i in det if i in truth]
    return len(errs), (float(np.mean(errs)) if errs else None)


def one(run):
    name = os.path.basename(run)
    bundle = run.split(os.sep)[-3]
    world = world_of(name, bundle)
    cond = name.split('_')[1]
    if cond not in CONDS:
        return None
    series = coverage_series(run, world)
    align, lock_t = alignment_series(run)
    d1 = traj_len(os.path.join(run, 'traj_leo1.csv'))
    d2 = traj_len(os.path.join(run, 'traj_leo2.csv'))
    n_mk, mk_err = markers(run, world)
    exp_log = os.path.join(run, 'explorer.log')
    _, _, footprint = geom(world)
    return dict(
        run=name, bundle=bundle, world=world, cond=cond, path=run,
        series=series, align=align, lock_t=lock_t,
        dist1=d1, dist2=d2, dist=d1 + d2,
        final_union=series[-1][1] if series else None,
        final_dup=series[-1][4] if series else None,
        footprint=footprint,
        n_markers=n_mk, marker_err=mk_err,
        finished=count_in(exp_log, r'Exploration finished\.'),
        aborted=count_in(exp_log, r'Exploration aborted:'),
        bans=count_in(exp_log, r'blacklisting goal|long-banning'),
        planner_fail=count_in(os.path.join(run, 'nav2.log'),
                              r'failed to create plan'),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=os.path.join(ROOT, 'final',
                                                  'suite_metrics.json'))
    ap.add_argument('-j', type=int, default=8)
    args = ap.parse_args()
    runs = sorted(glob.glob(os.path.join(ROOT, 'final', 'bundles', 'st-*',
                                         'runs', 'run_*')))
    runs = [r for r in runs if os.path.isdir(r)]
    print(f'{len(runs)} runs')
    with Pool(args.j) as pool:
        rows = [r for r in pool.map(one, runs) if r]
    json.dump(rows, open(args.out, 'w'))
    print(f'wrote {args.out}: {len(rows)} runs')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
