#!/usr/bin/env python3
"""Build the head-to-head table across every scored exp_run.sh output.

Reads the ``map_score.json`` / ``safety_score.json`` pairs that exp_score.sh
writes and prints one row per run, so the original stack and the Nav2 overlay
can be compared on identical worlds with identical metrics.

Usage:
    python3 exp_compare.py <reports/exp> [--json out.json] [--md out.md]
"""

import argparse
import json
import os


# (json file, key, column header, higher_is_better)
#
# Map agreement is reported *aligned*. slam_toolbox anchors the map frame on the
# first processed scan, which lands wherever the bootstrap jog left the rover,
# so every run carries a rigid map-frame offset that has nothing to do with map
# quality -- on the orig office run it alone moved IoU from 0.373 to 0.580.
# Raw ATE is kept alongside it because that offset inflates ATE the same way.
COLUMNS = [
    ('map_score.json', 'iou@2_aligned', 'IoU', True),
    ('map_score.json', 'phantom_frac', 'phantom', False),
    ('map_score.json', 'rmse_m_aligned', 'rmse m', False),
    ('map_score.json', 'free_area_ratio', 'coverage', True),
    ('map_score.json', 'slam_ate_rmse_m', 'slam ATE', False),
    ('map_score.json', 'odom_ate_rmse_m', 'odom ATE', False),
    ('safety_score.json', 'contact_events', 'bumps', False),
    ('safety_score.json', 'min_clearance_m', 'min clr', True),
    ('safety_score.json', 'doorway_passes', 'doors', True),
    ('safety_score.json', 'stuck_frac', 'stuck', False),
    ('safety_score.json', 'path_len_m', 'path m', True),
]


def load(run_dir):
    data = {}
    for name in ('map_score.json', 'safety_score.json'):
        path = os.path.join(run_dir, name)
        if os.path.isfile(path):
            try:
                with open(path) as fh:
                    data[name] = json.load(fh)
            except json.JSONDecodeError:
                data[name] = {}
        else:
            data[name] = {}
    return data


def fmt(value):
    if value is None:
        return '-'
    if isinstance(value, float):
        return f'{value:.3f}'
    return str(value)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--json')
    ap.add_argument('--md')
    args = ap.parse_args()

    runs = sorted(
        d for d in os.listdir(args.root)
        if os.path.isdir(os.path.join(args.root, d)) and not d.startswith(('_', '.'))
    )

    rows, records = [], {}
    for run in runs:
        data = load(os.path.join(args.root, run))
        if not data['map_score.json'] and not data['safety_score.json']:
            continue
        row = [run]
        record = {}
        for fname, key, _header, _hib in COLUMNS:
            value = data[fname].get(key)
            record[key] = value
            row.append(fmt(value))
        rows.append(row)
        records[run] = record

    if not rows:
        print(f'no scored runs under {args.root}')
        return

    headers = ['run'] + [c[2] for c in COLUMNS]
    widths = [max(len(headers[i]), max(len(r[i]) for r in rows)) for i in range(len(headers))]

    def render(cells):
        return '  '.join(c.ljust(widths[i]) for i, c in enumerate(cells)).rstrip()

    lines = [render(headers), render(['-' * w for w in widths])]
    lines += [render(r) for r in rows]
    table = '\n'.join(lines)
    print(table)

    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(records, fh, indent=2)
        print(f'\nwrote {args.json}')
    if args.md:
        md = ['| ' + ' | '.join(headers) + ' |',
              '|' + '|'.join('---' for _ in headers) + '|']
        md += ['| ' + ' | '.join(r) + ' |' for r in rows]
        with open(args.md, 'w') as fh:
            fh.write('\n'.join(md) + '\n')
        print(f'wrote {args.md}')


if __name__ == '__main__':
    main()
