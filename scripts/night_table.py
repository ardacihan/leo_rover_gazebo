#!/usr/bin/env python3
"""Collect the night's run directories into one comparison table.

    python3 scripts/night_table.py reports/night [--md out.md]

Reads `map_score.json`, `safety_score.json` and `aruco_score.json` from each
subdirectory. Columns are chosen for what actually decides whether a map is
usable: coverage and phantom walls first, then trajectory error, then the
driving behaviour that produced them. `metrics ranked a truncated run highest`
is the trap this table is arranged to avoid -- read `cover` and `path` before
believing a low `phantom`.
"""

import argparse
import json
import os
import sys


def load(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return {}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--md')
    args = ap.parse_args()

    rows = []
    for name in sorted(os.listdir(args.root)):
        d = os.path.join(args.root, name)
        if not os.path.isdir(d):
            continue
        m = load(os.path.join(d, 'map_score.json'))
        s = load(os.path.join(d, 'safety_score.json'))
        a = load(os.path.join(d, 'aruco_score.json'))
        if not m and not s:
            continue
        rows.append({
            'run': name,
            'cover': m.get('free_area_ratio'),
            'phantom': m.get('phantom_frac'),
            'iou': m.get('iou@2_aligned'),
            'wall_rmse': m.get('rmse_m_aligned'),
            'slam_ate': m.get('slam_ate_rmse_m'),
            'odom_ate': m.get('odom_ate_rmse_m'),
            'path': s.get('path_len_m'),
            'stuck': s.get('stuck_frac'),
            'speed': s.get('mean_speed_mps'),
            'doors': s.get('doorway_passes'),
            'contacts': s.get('contacts'),
            'near': s.get('near_misses'),
            'minclr': s.get('min_clearance_m'),
            'dur': s.get('duration_s'),
            'aruco': (f"{a['n_detected_correct']}/{a['n_truth']}"
                      if a.get('n_truth') else None),
        })

    cols = [('run', 28, 's'), ('cover', 6, '.3f'), ('phantom', 7, '.3f'),
            ('iou', 6, '.3f'), ('wall_rmse', 9, '.3f'), ('slam_ate', 8, '.3f'),
            ('odom_ate', 8, '.2f'), ('path', 6, '.1f'), ('stuck', 6, '.2f'),
            ('speed', 6, '.3f'), ('doors', 5, 'd'), ('contacts', 8, 'd'),
            ('near', 4, 'd'), ('minclr', 6, '.3f'), ('dur', 6, '.0f'),
            ('aruco', 6, 's')]

    def fmt(row):
        out = []
        for key, width, kind in cols:
            v = row.get(key)
            if v is None:
                out.append(f'{"-":>{width}}')
            elif kind == 's':
                out.append(f'{str(v):<{width}}' if key == 'run' else f'{str(v):>{width}}')
            else:
                out.append(f'{v:>{width}{kind}}')
        return '  '.join(out)

    header = '  '.join(
        f'{k:<{w}}' if k == 'run' else f'{k:>{w}}' for k, w, _ in cols)
    lines = [header, '-' * len(header)] + [fmt(r) for r in rows]
    print('\n'.join(lines))

    if args.md:
        md = ['| ' + ' | '.join(k for k, _, _ in cols) + ' |',
              '|' + '---|' * len(cols)]
        for r in rows:
            cells = []
            for key, _, kind in cols:
                v = r.get(key)
                cells.append('-' if v is None
                             else (str(v) if kind == 's' else f'{v:{kind}}'))
            md.append('| ' + ' | '.join(cells) + ' |')
        with open(args.md, 'w') as fh:
            fh.write('\n'.join(md) + '\n')
        print(f'\nwrote {args.md}', file=sys.stderr)


if __name__ == '__main__':
    main()
