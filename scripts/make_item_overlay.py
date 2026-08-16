#!/usr/bin/env python3
"""Overlay found items + ground-truth markers on a (merged) map.

Usage:
  python make_item_overlay.py --map merged.yaml --items items.jsonl \
      --markers mock_markers_office_world.yaml --out overlay.png
      [--title "..."]

Ground-truth markers are drawn as red diamonds; confirmed item estimates as
green circles (annotated with id + which robot confirmed first); unconfirmed
sightings as yellow crosses. Item positions in items.jsonl are already in the
common world frame (item_claims are published there).
"""

import argparse
import json

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import yaml

from map_fusion import load_map


def final_items(jsonl_path):
    """Last claim per (robot) merged: id -> best known record + first
    confirm time/robot."""
    latest = {}          # robot -> items list
    first_confirm = {}   # id -> (t, robot)
    with open(jsonl_path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            if rec.get('topic') != 'item_claims':
                continue
            latest[rec.get('robot', '?')] = rec.get('items', [])
            for it in rec.get('items', []):
                if it.get('confirmed') and it['id'] not in first_confirm \
                        and not it.get('via_peer'):
                    first_confirm[int(it['id'])] = (rec.get('t', 0.0),
                                                    rec.get('robot', '?'))
    merged = {}
    for robot, items in latest.items():
        for it in items:
            mid = int(it['id'])
            keep = merged.get(mid)
            if keep is None or (it.get('confirmed') and not keep.get('confirmed')):
                merged[mid] = {**it, 'robot': robot}
    return merged, first_confirm


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', required=True)
    ap.add_argument('--items', required=True)
    ap.add_argument('--markers', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--title', default='Found items')
    args = ap.parse_args()

    m = load_map(args.map)
    with open(args.markers) as f:
        gt = yaml.safe_load(f)['markers']
    items, first_confirm = final_items(args.items)

    H, W = m.shape
    rgb = np.full((H, W, 3), 0.82, dtype=np.float32)
    rgb[m.grid == 0] = 1.0
    rgb[m.grid == 100] = 0.08
    ext = [m.origin[0], m.origin[0] + W * m.res,
           m.origin[1], m.origin[1] + H * m.res]
    fig, ax = plt.subplots(figsize=(max(7, W / 70), max(5, H / 70)))
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')

    gx = [g['x'] for g in gt]
    gy = [g['y'] for g in gt]
    ax.scatter(gx, gy, marker='D', s=90, facecolors='none',
               edgecolors='tab:red', linewidths=1.6, label='ground truth')
    for g in gt:
        ax.annotate(str(g['id']), (g['x'], g['y']),
                    textcoords='offset points', xytext=(6, 6),
                    fontsize=8, color='tab:red')
    conf = [it for it in items.values() if it.get('confirmed')]
    unconf = [it for it in items.values() if not it.get('confirmed')]
    if conf:
        ax.scatter([i['x'] for i in conf], [i['y'] for i in conf],
                   marker='o', s=45, c='tab:green', label='confirmed')
        for it in conf:
            t_r = first_confirm.get(int(it['id']))
            note = f"{it['id']}"
            if t_r:
                note += f" ({t_r[1]}, t={t_r[0]:.0f}s)"
            ax.annotate(note, (it['x'], it['y']),
                        textcoords='offset points', xytext=(6, -12),
                        fontsize=7, color='tab:green')
    if unconf:
        ax.scatter([i['x'] for i in unconf], [i['y'] for i in unconf],
                   marker='x', s=45, c='goldenrod', label='unconfirmed')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title(args.title)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150)
    print('wrote', args.out,
          f'({len(conf)} confirmed, {len(unconf)} unconfirmed,'
          f' {len(gt)} ground truth)')


if __name__ == '__main__':
    main()
