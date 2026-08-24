#!/usr/bin/env python3
"""Live wall display for the two-rover merge: watch a directory, re-fuse, render.

The lab demo loop. Each rover runs fully onboard (own SLAM, own EKF, own ArUco
detector, own ROS domain) and periodically lands four files in one laptop
directory (rsync/scp pull loop, see TOMORROW_PLAN.md):

    aruco_registry_<name>.json      <name>_map.pgm / .yaml     (x2 rovers)

This script polls that directory. Before the rovers have >= 2 common markers
it renders both maps side by side with a "waiting for rendezvous" banner; the
moment the registries share enough tags it computes the transform (same
ungated Kabsch as align_registries_offline.py) and renders the fused map with
the recovered transform printed on it. The output PNG + auto-refreshing HTML
make the demo: the two fragments visibly snap into one building when the
rovers have both seen the corridor markers.

Nothing here feeds back into the rovers, so a bad fit can never break the
run -- the display is downstream-only, and every frame is recomputed from
scratch so it self-heals as the registries improve.

Usage:
  live_merge_watch.py <dir> [--interval 10] [--exclude ID...] [--use-best]
                            [--truth X Y YAW_DEG] [--once]
Open <dir>/live.html in a browser (fullscreen it on the projector).
"""
import argparse
import glob
import math
import os
import subprocess
import sys
import time
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from align_registries_offline import (            # noqa: E402
    load_registry, fit, residuals, ang_diff_deg)
from render_multirobot_media import read_map       # noqa: E402

import matplotlib                                  # noqa: E402
matplotlib.use('Agg')
import matplotlib.pyplot as plt                    # noqa: E402
import numpy as np                                 # noqa: E402

HTML = """<!doctype html><meta http-equiv="refresh" content="3">
<title>two-rover shared map</title>
<body style="margin:0;background:#111;display:flex;align-items:center;
justify-content:center;height:100vh">
<img src="live_merged.png?t={t}" style="max-width:100vw;max-height:100vh">
</body>"""


def grid_img(grid):
    img = np.full(grid.shape, 0.82)
    img[grid == 0] = 1.0
    img[grid == 2] = 0.0
    return img


def render_waiting(maps, names, common, out_png, note):
    n = max(1, len(maps))
    fig, axes = plt.subplots(1, n, figsize=(7 * n, 7))
    if n == 1:
        axes = [axes]
    for ax, (grid, ext), name in zip(axes, maps, names):
        ax.imshow(grid_img(grid), cmap='gray', vmin=0, vmax=1,
                  origin='lower', extent=[ext[0], ext[1], ext[2], ext[3]])
        ax.set_title(name)
    fig.suptitle(f'waiting for rendezvous -- common markers: {common}  {note}',
                 fontsize=15)
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def render_merged(stem, est, common, res, out_png, truth=None):
    grid, ext = read_map(stem)
    fig, ax = plt.subplots(figsize=(11, 11 * grid.shape[0] / grid.shape[1]))
    ax.imshow(grid_img(grid), cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=[ext[0], ext[1], ext[2], ext[3]])
    title = (f'SHARED MAP -- rendezvous on tags {common}   '
             f'tf ({est.dx:.2f} m, {est.dy:.2f} m, '
             f'{math.degrees(est.yaw):.1f} deg)   '
             f'mean residual {sum(res) / len(res):.2f} m')
    if truth:
        terr = math.hypot(est.dx - truth[0], est.dy - truth[1])
        yerr = abs(ang_diff_deg(est.yaw, math.radians(truth[2])))
        title += f'   [vs truth {terr:.2f} m / {yerr:.1f} deg]'
    ax.set_title(title, fontsize=12)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def tick(d, args, last_sig):
    regs = sorted(glob.glob(os.path.join(d, 'aruco_registry_*.json')))
    stems = []
    for rp in regs:
        name = os.path.basename(rp).replace('aruco_registry_',
                                            '').replace('.json', '')
        for cand in (os.path.join(d, f'{name}_map'), os.path.join(d, name)):
            if os.path.exists(cand + '.pgm'):
                stems.append((rp, cand, name))
                break
    watched = [p for rp, st, _ in stems for p in (rp, st + '.pgm')]
    sig = tuple((p, os.path.getmtime(p)) for p in watched)
    if sig == last_sig:
        return last_sig                      # nothing new
    out_png = os.path.join(d, 'live_merged.png')

    if len(stems) < 2:
        maps = [read_map(st) for _, st, _ in stems]
        render_waiting([m for m in maps if m[0] is not None],
                       [n for (_, _, n), m in zip(stems, maps)
                        if m[0] is not None],
                       [], out_png, f'({len(stems)}/2 rovers reporting)')
        return sig

    (reg1, stem1, name1), (reg2, stem2, name2) = stems[0], stems[1]
    r1 = load_registry(reg1, args.use_best)
    r2 = load_registry(reg2, args.use_best)
    common = sorted(set(r1) & set(r2) - set(args.exclude))
    if len(common) < 2:
        render_waiting([read_map(stem1), read_map(stem2)], [name1, name2],
                       common, out_png,
                       f'({name1} sees {sorted(r1)}, {name2} sees {sorted(r2)})')
        print(f'[{time.strftime("%H:%M:%S")}] waiting: common={common}')
        return sig

    src = [(r2[i][0], r2[i][1]) for i in common]
    tgt = [(r1[i][0], r1[i][1]) for i in common]
    est = fit(src, tgt)
    res = residuals(est, src, tgt)
    merged_stem = os.path.join(d, 'live_merged')
    subprocess.run(
        [sys.executable, os.path.join(HERE, 'fuse_maps_offline.py'),
         stem1, stem2, merged_stem,
         '--tf', f'{est.dx}', f'{est.dy}', f'{math.degrees(est.yaw)}'],
        check=True, stdout=subprocess.DEVNULL)
    render_merged(merged_stem, est, common, res, out_png, args.truth)
    print(f'[{time.strftime("%H:%M:%S")}] merged: tags={common} '
          f'tf=({est.dx:.2f}, {est.dy:.2f}, {math.degrees(est.yaw):.1f} deg) '
          f'mean_res={sum(res) / len(res):.2f} m')
    return sig


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('dir')
    ap.add_argument('--interval', type=float, default=10.0)
    ap.add_argument('--exclude', nargs='*', type=int, default=[])
    ap.add_argument('--use-best', action='store_true')
    ap.add_argument('--truth', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--once', action='store_true',
                    help='one tick, then exit (for testing)')
    args = ap.parse_args()

    with open(os.path.join(args.dir, 'live.html'), 'w') as fh:
        fh.write(HTML.format(t=0))
    print(f'watching {args.dir} -- open '
          f'{os.path.join(args.dir, "live.html")} in a browser')
    sig = None
    while True:
        try:
            sig = tick(args.dir, args, sig)
        except Exception:
            traceback.print_exc()      # a half-copied file: skip this tick
        if args.once:
            break
        time.sleep(args.interval)
    return 0


if __name__ == '__main__':
    sys.exit(main())
