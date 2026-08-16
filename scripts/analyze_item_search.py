#!/usr/bin/env python3
"""Item-search benchmark analysis: time-to-k-items across conditions.

Reads the items.jsonl logs written by item_recorder.py for every run dir
given, computes when the UNION of confirmed items across robots reached k
(k = 1..N) in sim time, and emits a comparison figure + summary table.

Usage:
  python analyze_item_search.py --out reports/item_search_collab \
      label1=path/to/run1 label2=path/to/run2 ...

Runs whose label shares a prefix before ':' are averaged as one condition,
e.g.  "2coord:seedA=..." "2coord:seedB=..." -> condition "2coord".
"""

import argparse
import json
import os
from collections import defaultdict

import numpy as np


def load_confirm_times(jsonl_path):
    """First sim time each item id was CONFIRMED by any robot."""
    confirmed = {}
    t_end = 0.0
    with open(jsonl_path) as f:
        for line in f:
            try:
                rec = json.loads(line)
            except ValueError:
                continue
            t = rec.get('t', 0.0)
            t_end = max(t_end, t)
            if rec.get('topic') != 'item_claims':
                continue
            for it in rec.get('items', []):
                if it.get('confirmed') and it['id'] not in confirmed:
                    confirmed[int(it['id'])] = t
    return confirmed, t_end


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='label=rundir (label "cond:seed")')
    ap.add_argument('--out', required=True)
    ap.add_argument('--n-items', type=int, default=8)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    per_run = {}
    for spec in args.runs:
        label, path = spec.split('=', 1)
        jl = os.path.join(path, 'items.jsonl')
        if not os.path.exists(jl):
            print(f'WARN: no items.jsonl in {path}, skipping')
            continue
        confirmed, t_end = load_confirm_times(jl)
        times = sorted(confirmed.values())
        per_run[label] = {'confirm_times': times, 'ids': sorted(confirmed),
                          'n_found': len(times), 't_end': t_end,
                          'path': path}
        print(f'{label}: {len(times)}/{args.n_items} items, '
              f'times={[round(t) for t in times]}, log end t={t_end:.0f}s')

    conditions = defaultdict(list)
    for label, r in per_run.items():
        conditions[label.split(':')[0]].append(r)

    summary = {}
    for cond, runs in conditions.items():
        rows = []
        for k in range(1, args.n_items + 1):
            ts = [r['confirm_times'][k - 1] if len(r['confirm_times']) >= k
                  else None for r in runs]
            done = [t for t in ts if t is not None]
            rows.append({'k': k, 'times': ts,
                         'mean': float(np.mean(done)) if done else None,
                         'n_reached': len(done), 'n_runs': len(runs)})
        summary[cond] = {
            'runs': [{'n_found': r['n_found'], 't_end': r['t_end'],
                      'path': r['path'],
                      'confirm_times': r['confirm_times']} for r in runs],
            'time_to_k': rows,
        }
    with open(os.path.join(args.out, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    # ---- figure: step curves items-found vs sim time, one line per run ----
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'1robot': '#59a1d8', '2indep': '#e8a33d', '2coord': '#4fae7a'}
    fig, ax = plt.subplots(figsize=(8, 5))
    seen_conds = set()
    for label, r in sorted(per_run.items()):
        cond = label.split(':')[0]
        c = colors.get(cond, 'gray')
        ts = [0] + r['confirm_times'] + [r['t_end']]
        ks = list(range(len(r['confirm_times']) + 1)) + [r['n_found']]
        ax.step(ts, ks, where='post', color=c, alpha=0.85,
                lw=2 if cond not in seen_conds else 1.2,
                label=cond if cond not in seen_conds else None)
        seen_conds.add(cond)
    ax.axhline(args.n_items, color='k', ls=':', lw=0.8)
    ax.set_xlabel('sim time [s]')
    ax.set_ylabel('confirmed items (union across robots)')
    ax.set_yticks(range(args.n_items + 1))
    ax.set_title('Item search: time to confirmed items')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'time_to_items.png'), dpi=150)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
