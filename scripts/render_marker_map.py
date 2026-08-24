#!/usr/bin/env python3
"""Draw where each rover detected each ArUco marker, against ground truth.

Two panels:

* **left** — the map with, per marker, the ground-truth position (star), each
  rover's estimate (circle), and a line joining estimate to truth so the error
  is a length you can see rather than a number in a table. Detection rays are
  drawn faintly from where the rover stood to what it saw, which shows the
  geometry that produced the estimate.
* **right** — error against range, the relationship that decides `max_range`.

This is the figure that separates "the detector is wrong" from "the rover
thought it was somewhere else": a marker seen many times from close range and
still metres out is a pose problem, not a perception one.

Usage:
  render_marker_map.py <run_dir> --world <name> --out <png>
                       [--leo2-tf X Y YAW_DEG] [--spawn1 X Y YAW] [--spawn2 ...]
"""

import argparse
import csv
import json
import math
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                        # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_multirobot_media import read_map, MAP_CMAP, MAP_NORM, markers_for  # noqa: E402

LEO1, LEO2 = '#1f77b4', '#e8710a'
TRUTH = '#8e44ad'


def load_registry(path):
    if not os.path.exists(path):
        return {}
    d = json.load(open(path))
    return {int(m['id']): (float(m['x']), float(m['y']), int(m.get('hits', 0)),
                           float(m.get('best_range_m', float('nan'))))
            for m in d.get('markers', [])}


def load_samples(path):
    out = []
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                out.append((int(row['id']), float(row['range_m']),
                            float(row['map_x']), float(row['map_y'])))
            except (ValueError, KeyError):
                continue
    return out


def to_frame(x, y, tf):
    if tf is None:
        return x, y
    c, s = math.cos(math.radians(tf[2])), math.sin(math.radians(tf[2]))
    return tf[0] + c * x - s * y, tf[1] + s * x + c * y


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--leo2-tf', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--spawn1', nargs=3, type=float, default=[0, 0, 0])
    ap.add_argument('--title', default='')
    args = ap.parse_args()
    run = args.run_dir

    truth = {mid: (x, y) for mid, x, y in markers_for(args.world)}
    reg1 = load_registry(os.path.join(run, 'aruco_registry_leo1.json'))
    reg2 = load_registry(os.path.join(run, 'aruco_registry_leo2.json'))
    s1 = load_samples(os.path.join(run, 'aruco_samples_leo1.csv'))
    s2 = load_samples(os.path.join(run, 'aruco_samples_leo2.csv'))

    # Everything is drawn in leo1's map frame; leo1's own frame is offset from
    # the world by its spawn, so ground truth is shifted to match rather than
    # the maps being shifted to the world.
    sx, sy, syaw = args.spawn1
    def truth_in_leo1(x, y):
        c, s = math.cos(-math.radians(syaw)), math.sin(-math.radians(syaw))
        dx, dy = x - sx, y - sy
        return c * dx - s * dy, s * dx + c * dy

    cls, extent = read_map(os.path.join(run, 'merged_map'))
    if cls is None:
        cls, extent = read_map(os.path.join(run, 'leo1_map'))

    fig, (ax, ax2) = plt.subplots(
        1, 2, figsize=(17, 7.5), dpi=115,
        gridspec_kw={'width_ratios': [1.55, 1]})

    if cls is not None:
        ax.imshow(cls, origin='lower', extent=extent, cmap=MAP_CMAP,
                  norm=MAP_NORM, interpolation='nearest')

    # faint detection rays: where the rover stood -> what it recorded
    for samples, colour, tf in ((s1, LEO1, None), (s2, LEO2, args.leo2_tf)):
        for mid, rng, mx, my in samples[::3]:
            gx, gy = to_frame(mx, my, tf)
            ax.plot([gx], [gy], '.', color=colour, ms=1.4, alpha=0.30)

    rows = []
    for mid in sorted(set(truth) | set(reg1) | set(reg2)):
        if mid in truth:
            tx, ty = truth_in_leo1(*truth[mid])
            ax.plot(tx, ty, '*', color=TRUTH, ms=17, mec='white', mew=0.8,
                    zorder=5)
            ax.annotate(str(mid), (tx, ty), textcoords='offset points',
                        xytext=(8, 7), fontsize=9, color=TRUTH, weight='bold')
        for reg, colour, tf, name in ((reg1, LEO1, None, 'leo1'),
                                      (reg2, LEO2, args.leo2_tf, 'leo2')):
            if mid not in reg:
                continue
            ex, ey, hits, best = reg[mid]
            gx, gy = to_frame(ex, ey, tf)
            ax.plot(gx, gy, 'o', color=colour, ms=8, mfc='none', mew=2,
                    zorder=6)
            if mid in truth:
                ax.plot([tx, gx], [ty, gy], '-', color=colour, lw=1.4,
                        alpha=0.85, zorder=4)
                err = math.hypot(gx - tx, gy - ty)
                rows.append((name, mid, best, err, hits, colour))

    ax.set_title(args.title or 'ArUco detections vs ground truth')
    ax.set_xlabel('x [m]  (leo1 map frame)'); ax.set_ylabel('y [m]')
    ax.grid(alpha=0.2, lw=0.4)
    handles = [
        plt.Line2D([], [], marker='*', ls='', color=TRUTH, ms=14, label='true marker position'),
        plt.Line2D([], [], marker='o', ls='', mfc='none', mew=2, color=LEO1, label='leo1 estimate'),
        plt.Line2D([], [], marker='o', ls='', mfc='none', mew=2, color=LEO2, label='leo2 estimate'),
        plt.Line2D([], [], ls='-', color='#888', label='error'),
    ]
    ax.legend(handles=handles, loc='best', fontsize=9)

    for name, mid, best, err, hits, colour in rows:
        if not math.isfinite(best):
            continue
        ax2.plot(best, err, 'o', color=colour, ms=7 + min(hits, 200) / 40.0,
                 mfc=colour, alpha=0.75)
        ax2.annotate(f'{mid}', (best, err), textcoords='offset points',
                     xytext=(6, 4), fontsize=8, color=colour)
    ax2.axvline(4.5, ls='--', color='#c0392b', lw=1.2)
    ax2.annotate('max_range 4.5 m', (4.5, ax2.get_ylim()[1]),
                 textcoords='offset points', xytext=(-100, -14),
                 fontsize=9, color='#c0392b')
    ax2.axhline(0.5, ls=':', color='#555', lw=1)
    ax2.set_xlabel('closest observed range [m]')
    ax2.set_ylabel('position error [m]')
    ax2.set_title('error vs range  (marker size = sightings)')
    ax2.grid(alpha=0.3, lw=0.5)

    fig.tight_layout()
    fig.savefig(args.out)
    print(f'{args.out}: {len(rows)} marker estimates plotted')
    for name, mid, best, err, hits, _ in sorted(rows, key=lambda r: -r[3])[:6]:
        print(f'  {name} id={mid}: err {err:.2f} m at {best:.2f} m, {hits} hits')


if __name__ == '__main__':
    main()
