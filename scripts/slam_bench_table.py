#!/usr/bin/env python3
"""Collect slam_bench metrics.json files into one comparison table.

Usage:
    python3 slam_bench_table.py <bench-root> [--md out.md] [--csv out.csv]
"""

import argparse
import glob
import json
import os

COLUMNS = [
    ('label', 'run', None),
    ('iou@2', 'wall IoU', '{:.3f}'),
    ('precision@2', 'precision', '{:.3f}'),
    ('recall@2', 'recall', '{:.3f}'),
    ('rmse_m', 'wall RMSE m', '{:.3f}'),
    ('phantom_frac', 'phantom', '{:.3f}'),
    ('slam_ate_rmse_m', 'SLAM ATE m', '{:.3f}'),
    ('slam_final_err_m', 'SLAM final m', '{:.3f}'),
    ('odom_final_err_m', 'odom final m', '{:.2f}'),
    ('free_area_ratio', 'coverage', '{:.3f}'),
    ('extent_excess_m', 'extent excess m', None),
]

# Preferred display order; anything else is appended alphabetically.
ORDER = ['A_ideal', 'A_real', 'B_real', 'B_real_selffiltered', 'C_real',
         'C_real_selffiltered', 'C_miscal']


def load(root):
    rows = []
    for path in sorted(glob.glob(os.path.join(root, '*', 'metrics.json'))):
        with open(path) as fh:
            data = json.load(fh)
        data.setdefault('label', os.path.basename(os.path.dirname(path)))
        rows.append(data)
    rows.sort(key=lambda r: (ORDER.index(r['label'])
                             if r['label'] in ORDER else len(ORDER),
                             r['label']))
    return rows


def cell(row, key, fmt):
    value = row.get(key)
    if value is None:
        return '-'
    if isinstance(value, list):
        return ', '.join(f'{v:+.2f}' for v in value)
    if fmt and isinstance(value, (int, float)):
        return fmt.format(value)
    return str(value)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root')
    ap.add_argument('--md')
    ap.add_argument('--csv')
    args = ap.parse_args()

    rows = load(args.root)
    if not rows:
        raise SystemExit(f'no metrics.json under {args.root}')

    headers = [h for _, h, _ in COLUMNS]
    table = [[cell(r, k, f) for k, _, f in COLUMNS] for r in rows]
    widths = [max(len(headers[i]), max(len(r[i]) for r in table))
              for i in range(len(headers))]

    def line(cells):
        return '  '.join(c.ljust(widths[i]) for i, c in enumerate(cells))

    print(line(headers))
    print('  '.join('-' * w for w in widths))
    for r in table:
        print(line(r))

    if args.md:
        with open(args.md, 'w') as fh:
            fh.write('| ' + ' | '.join(headers) + ' |\n')
            fh.write('|' + '|'.join('---' for _ in headers) + '|\n')
            for r in table:
                fh.write('| ' + ' | '.join(r) + ' |\n')
        print('wrote', args.md)
    if args.csv:
        import csv as csvmod
        with open(args.csv, 'w', newline='') as fh:
            writer = csvmod.writer(fh)
            writer.writerow(headers)
            writer.writerows(table)
        print('wrote', args.csv)


if __name__ == '__main__':
    main()
