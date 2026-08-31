#!/usr/bin/env python3
"""Side-by-side comparison videos for the analysis page.

  compare_setups_<map>.mp4  the four team setups on the same building, playing
                            in step on the same clock
  compare_maps.mp4          the same setup (teamwork, start unknown) on all
                            four buildings

Frames come from the recorded map snapshots, so the clock is simulated time:
panel k of every video shows the same moment of its run. A run that finishes
early holds its last frame, with "finished" written on it, instead of going
black - so a short panel means that team was done, not that data is missing.

Usage: python3 scripts/build_combined_videos.py
"""

import glob
import json
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))
from render_timelapse import panel, world_to_px, LEO1, LEO2  # noqa: E402

OUT = os.path.join(ROOT, 'final', 'figures')
PANEL = (620, 470)
LABEL = {'single': 'one robot', 'indep': 'two robots, not talking',
         'c2u': 'teamwork, start unknown', 'c2k': 'teamwork, start known'}
NICE = {'office_world': 'office', 'depot_world': 'depot',
        'small_house': 'house', 'husarion_office': 'cluttered office'}


def frames_for(run, world, want=160):
    """Per-snapshot panels of the team's combined map, plus sim times."""
    import spawn_poses
    off = spawn_poses.relative_offset(world)
    c, s = math.cos(off[2]), math.sin(off[2])
    snaps = sorted(glob.glob(os.path.join(run, 'timelapse', 'snap*.npz')))
    if not snaps:
        return [], []
    step = max(1, len(snaps) // want)
    out, times, trail1, trail2 = [], [], [], []
    t0 = None
    for path in snaps[::step]:
        try:
            d = np.load(path)
        except (OSError, ValueError):
            continue
        t = float(d['t'])
        t0 = t if t0 is None else t0
        p1, p2 = d['p1'], d['p2']
        if math.isfinite(float(p1[0])):
            trail1.append((float(p1[0]), float(p1[1])))
        if math.isfinite(float(p2[0])):
            x, y = float(p2[0]), float(p2[1])
            trail2.append((off[0] + c * x - s * y, off[1] + s * x + c * y))
        # Draw on robot 1's grid; robot 2's map is pasted in using the true
        # offset so every condition is shown the same way, merged or not.
        g1, i1 = d['leo1'], d['leo1_info']
        if g1.size <= 1:
            continue
        grid = np.array(g1, dtype=np.int8, copy=True)
        if 'leo2' in d.files and d['leo2'].size > 1:
            g2, i2 = d['leo2'], d['leo2_info']
            ys, xs = np.nonzero(g2 >= 0)
            wx = i2[0] + (xs + 0.5) * i2[2]
            wy = i2[1] + (ys + 0.5) * i2[2]
            tx, ty = off[0] + c * wx - s * wy, off[1] + s * wx + c * wy
            ci = ((tx - i1[0]) / i1[2]).astype(int)
            ri = ((ty - i1[1]) / i1[2]).astype(int)
            ok = ((ci >= 0) & (ci < grid.shape[1]) &
                  (ri >= 0) & (ri < grid.shape[0]))
            vals = g2[ys[ok], xs[ok]]
            cur = grid[ri[ok], ci[ok]]
            grid[ri[ok], ci[ok]] = np.where(cur < 0, vals,
                                            np.maximum(cur, vals))
        img = panel(grid, i1, PANEL,
                    trail=[(trail1, LEO1), (trail2, LEO2)], title='')
        out.append(img)
        times.append(t - t0)
    return out, times


def tile(panels, labels, times, sub, out_path, fps=8.0):
    """Grid of panels, padded with each run's last frame; one shared clock."""
    n = len(panels)
    cols = 2 if n > 2 else n
    rowsn = int(math.ceil(n / cols))
    length = max(len(p) for p in panels)
    W, H = PANEL[0] * cols, 34 + rowsn * (PANEL[1] + 34)
    w = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'avc1'), fps, (W, H))
    if not w.isOpened():
        raise RuntimeError(f'cannot open writer {out_path}')
    for k in range(length):
        canvas = np.full((H, W, 3), 255, dtype=np.uint8)
        for i, frames in enumerate(panels):
            done = k >= len(frames)
            img = frames[min(k, len(frames) - 1)].copy()
            r, c = divmod(i, cols)
            # header (34) + this row's own label bar (34)
            y0 = 34 + r * (PANEL[1] + 34) + 34
            x0 = c * PANEL[0]
            bar = np.full((34, PANEL[0], 3), (247, 247, 245), dtype=np.uint8)
            txt = labels[i] + (
                f'   t = {times[i][min(k, len(times[i]) - 1)]:.0f} s'
                if times[i] else '')
            if done:
                txt += '   [finished]'
            cv2.putText(bar, txt, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (40, 40, 40) if not done else (120, 120, 120), 1,
                        cv2.LINE_AA)
            canvas[y0 - 34:y0, x0:x0 + PANEL[0]] = bar
            canvas[y0:y0 + PANEL[1], x0:x0 + PANEL[0]] = img
        head = np.full((34, W, 3), (255, 255, 255), dtype=np.uint8)
        cv2.putText(head, sub, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                    (20, 20, 20), 1, cv2.LINE_AA)
        canvas[0:34] = head
        w.write(canvas)
    w.release()
    print(f'{out_path}: {length} frames')


def pick(rows, world, cond):
    """Median-coverage run of that cell, so the video is typical not cherry-picked."""
    rs = [r for r in rows if r['world'] == world and r['cond'] == cond
          and r['final_union'] and r['cap'] == {'office_world': 8,
                                                'depot_world': 7,
                                                'small_house': 10,
                                                'husarion_office': 12}.get(world)]
    if not rs:
        rs = [r for r in rows if r['world'] == world and r['cond'] == cond
              and r['final_union']]
    if not rs:
        return None
    rs.sort(key=lambda r: r['final_union'])
    return rs[len(rs) // 2]


def main():
    os.makedirs(OUT, exist_ok=True)
    rows = json.load(open(os.path.join(ROOT, 'final', 'suite_metrics.json')))
    man = json.load(open(os.path.join(ROOT, 'final',
                                      'suite_2026-08-30_manifest.json')))
    caps = {r['name']: r.get('cap') for r in man['runs']}
    for r in rows:
        r['cap'] = caps.get(r['run'])

    for world in ('office_world', 'small_house'):
        packs, labels, times = [], [], []
        for cond in ('single', 'indep', 'c2u', 'c2k'):
            r = pick(rows, world, cond)
            if not r:
                continue
            f, t = frames_for(r['path'], world)
            if not f:
                continue
            packs.append(f); labels.append(LABEL[cond]); times.append(t)
        if packs:
            tile(packs, labels, times,
                 f'{NICE[world]} - same building, four team setups, same clock',
                 os.path.join(OUT, f'compare_setups_{NICE[world]}.mp4'))

    packs, labels, times = [], [], []
    for world in ('office_world', 'depot_world', 'small_house',
                  'husarion_office'):
        r = pick(rows, world, 'c2u')
        if not r:
            continue
        f, t = frames_for(r['path'], world)
        if not f:
            continue
        packs.append(f); labels.append(NICE[world]); times.append(t)
    if packs:
        tile(packs, labels, times,
             'same team setup (teamwork, start unknown) on all four buildings',
             os.path.join(OUT, 'compare_maps.mp4'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
