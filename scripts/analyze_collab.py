#!/usr/bin/env python3
"""Analyze collaborative-exploration runs and emit comparison figures.

Discovers run directories named  <root>/<world>_<condition>/  where condition
is one of: single, independent, coordinated. Each dir is expected to hold a
map_coverage.py log (coverage.log), optionally a merged map (merged_map.pgm/
.yaml or map.pgm/.yaml) and per-robot trajectories (traj.csv).

Outputs (into <root>/figures/):
  coverage_vs_time_<world>.png   coverage curves, one per condition
  maps_<world>.png               final maps + trajectory overlays
  summary_bars.png               final area + time-to-target by condition
  summary.json / summary.md      metric tables

Usage: python3 analyze_collab.py [root=reports/collab]
"""

import glob
import json
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

COV = re.compile(r't=(\d+)s known=([\d.]+)m2 free=([\d.]+)m2 occ=([\d.]+)m2')
CONDITIONS = ['single', 'independent', 'coordinated']
COLORS = {'single': '#4C72B0', 'independent': '#DD8452', 'coordinated': '#55A868'}
LABELS = {'single': '1 robot (baseline)',
          'independent': '2 robots (uncoordinated)',
          'coordinated': '2 robots (coordinated)'}
ROBOT_COLORS = {'leo1': '#C44E52', 'leo2': '#8172B3'}


def parse_coverage(path):
    t, known = [], []
    if not os.path.exists(path):
        return np.array([]), np.array([])
    with open(path) as f:
        for line in f:
            m = COV.search(line)
            if m:
                t.append(int(m.group(1)))
                known.append(float(m.group(2)))
    t, known = np.array(t, float), np.array(known, float)
    if len(t):
        t = t - t[0]                       # align each run to its own start
    return t, known


def time_to_target(t, known, target):
    """First aligned time at which coverage >= target, or None."""
    for ti, ki in zip(t, known):
        if ki >= target:
            return float(ti)
    return None


def load_map(run_dir):
    for base in ('merged_map', 'map'):
        pgm = os.path.join(run_dir, base + '.pgm')
        yml = os.path.join(run_dir, base + '.yaml')
        if os.path.exists(pgm) and os.path.exists(yml):
            img = plt.imread(pgm)
            res, ox, oy = 0.05, 0.0, 0.0
            with open(yml) as f:
                for line in f:
                    if line.startswith('resolution'):
                        res = float(line.split(':')[1])
                    if line.startswith('origin'):
                        vals = re.findall(r'-?[\d.]+', line.split(':', 1)[1])
                        ox, oy = float(vals[0]), float(vals[1])
            return img, res, ox, oy
    return None, None, None, None


def load_traj(run_dir):
    path = os.path.join(run_dir, 'traj.csv')
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        next(f, None)
        for line in f:
            parts = line.strip().split(',')
            if len(parts) != 4:
                continue
            _, r, x, y = parts
            out.setdefault(r, []).append((float(x), float(y)))
    return {r: np.array(v) for r, v in out.items()}


def discover(root):
    """world -> {condition: run_dir}."""
    runs = {}
    for d in sorted(glob.glob(os.path.join(root, '*_*'))):
        if not os.path.isdir(d):
            continue
        name = os.path.basename(d)
        for cond in CONDITIONS:
            if name.endswith('_' + cond):
                world = name[: -(len(cond) + 1)]
                runs.setdefault(world, {})[cond] = d
    return runs


def plot_coverage(world, conds, fig_dir):
    fig, ax = plt.subplots(figsize=(8, 4.8))
    any_data = False
    for cond in CONDITIONS:
        if cond not in conds:
            continue
        t, known = parse_coverage(os.path.join(conds[cond], 'coverage.log'))
        if not len(t):
            continue
        any_data = True
        ax.plot(t, known, label=LABELS[cond], color=COLORS[cond], linewidth=2.2)
    if not any_data:
        plt.close(fig)
        return
    ax.set_xlabel('time since exploration start [s]')
    ax.set_ylabel('mapped area  [m²]')
    ax.set_title(f'Collaborative exploration — {world}')
    ax.legend(frameon=False)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(fig_dir, f'coverage_vs_time_{world}.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('wrote', out)


def plot_maps(world, conds, fig_dir):
    present = [c for c in CONDITIONS if c in conds]
    if not present:
        return
    fig, axes = plt.subplots(1, len(present),
                             figsize=(5.2 * len(present), 5.0), squeeze=False)
    drew = False
    for ax, cond in zip(axes[0], present):
        img, res, ox, oy = load_map(conds[cond])
        ax.set_title(LABELS[cond], fontsize=11)
        ax.set_xticks([]); ax.set_yticks([])
        if img is None:
            ax.text(0.5, 0.5, 'no map', ha='center', va='center',
                    transform=ax.transAxes)
            continue
        drew = True
        h = img.shape[0]
        ax.imshow(img, cmap='gray', origin='upper')
        traj = load_traj(conds[cond])
        for r, pts in traj.items():
            if not len(pts):
                continue
            px = (pts[:, 0] - ox) / res
            py = h - (pts[:, 1] - oy) / res
            ax.plot(px, py, color=ROBOT_COLORS.get(r, '#333'),
                    linewidth=1.6, label=r)
            ax.scatter(px[0], py[0], c='lime', s=30, zorder=5)
        if traj:
            ax.legend(loc='lower right', fontsize=8, frameon=True)
    if drew:
        fig.suptitle(f'Final maps + rover trajectories — {world}', y=1.02)
        fig.tight_layout()
        out = os.path.join(fig_dir, f'maps_{world}.png')
        fig.savefig(out, dpi=130, bbox_inches='tight')
        print('wrote', out)
    plt.close(fig)


def separation_series(run_dir):
    """From traj.csv return (times, distances) between leo1 and leo2."""
    traj_by_t = {}
    path = os.path.join(run_dir, 'traj.csv')
    if not os.path.exists(path):
        return np.array([]), np.array([])
    with open(path) as f:
        next(f, None)
        for line in f:
            p = line.strip().split(',')
            if len(p) != 4:
                continue
            t, r, x, y = float(p[0]), p[1], float(p[2]), float(p[3])
            traj_by_t.setdefault(t, {})[r] = (x, y)
    ts, ds = [], []
    for t in sorted(traj_by_t):
        d = traj_by_t[t]
        if 'leo1' in d and 'leo2' in d:
            ts.append(t)
            ds.append(np.hypot(d['leo1'][0] - d['leo2'][0],
                               d['leo1'][1] - d['leo2'][1]))
    ts = np.array(ts)
    if len(ts):
        ts = ts - ts[0]
    return ts, np.array(ds)


def plot_separation(world, conds, fig_dir):
    fig, ax = plt.subplots(figsize=(8, 4.2))
    drew = False
    means = {}
    for cond in ('independent', 'coordinated'):
        if cond not in conds:
            continue
        ts, ds = separation_series(conds[cond])
        if not len(ts):
            continue
        drew = True
        means[cond] = float(np.mean(ds))
        ax.plot(ts, ds, label=f'{LABELS[cond]} (mean {means[cond]:.1f} m)',
                color=COLORS[cond], linewidth=1.8)
    if not drew:
        plt.close(fig)
        return means
    ax.set_xlabel('time since exploration start [s]')
    ax.set_ylabel('inter-rover distance [m]')
    ax.set_title(f'Spatial separation between rovers — {world}')
    ax.legend(frameon=False)
    ax.grid(alpha=0.3)
    fig.tight_layout()
    out = os.path.join(fig_dir, f'separation_{world}.png')
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print('wrote', out)
    return means


def main():
    root = sys.argv[1] if len(sys.argv) > 1 else 'reports/collab'
    fig_dir = os.path.join(root, 'figures')
    os.makedirs(fig_dir, exist_ok=True)
    runs = discover(root)
    if not runs:
        print('no runs found under', root)
        return

    summary = {}
    for world, conds in sorted(runs.items()):
        plot_coverage(world, conds, fig_dir)
        plot_maps(world, conds, fig_dir)
        sep_means = plot_separation(world, conds, fig_dir)
        # common absolute target = 90% of the best final area across conditions
        finals = {}
        curves = {}
        for cond, d in conds.items():
            t, k = parse_coverage(os.path.join(d, 'coverage.log'))
            if len(k):
                finals[cond] = float(k[-1])
                curves[cond] = (t, k)
        if not finals:
            continue
        target = 0.90 * max(finals.values())
        summary[world] = {}
        for cond, (t, k) in curves.items():
            summary[world][cond] = {
                'final_area_m2': round(float(k[-1]), 1),
                'duration_s': int(t[-1]),
                'time_to_90pct_common_s': time_to_target(t, k, target),
                'common_target_m2': round(target, 1),
                'mean_separation_m': round(sep_means.get(cond), 2)
                if sep_means.get(cond) is not None else None,
            }

    with open(os.path.join(fig_dir, 'summary.json'), 'w') as f:
        json.dump(summary, f, indent=2)

    # summary markdown + speedup
    lines = ['# Collaborative exploration summary', '',
             '90% target = 90% of best final mapped area per world.', '']
    for world, cw in sorted(summary.items()):
        lines.append(f'## {world}')
        lines.append('| condition | final area m² | duration s | '
                     'time→90% target s |')
        lines.append('|---|---|---|---|')
        for cond in CONDITIONS:
            if cond in cw:
                c = cw[cond]
                lines.append(f'| {LABELS[cond]} | {c["final_area_m2"]} | '
                             f'{c["duration_s"]} | '
                             f'{c["time_to_90pct_common_s"]} |')
        # speedup coordinated vs single & vs independent
        base = cw.get('single', {}).get('time_to_90pct_common_s')
        coord = cw.get('coordinated', {}).get('time_to_90pct_common_s')
        indep = cw.get('independent', {}).get('time_to_90pct_common_s')
        if base and coord:
            lines.append(f'\n*Coordinated reaches 90% target '
                         f'{base / coord:.2f}× faster than single robot.*')
        if indep and coord:
            lines.append(f'*Coordinated is {indep / coord:.2f}× faster than '
                         f'uncoordinated 2-robot.*')
        lines.append('')
    with open(os.path.join(fig_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines))

    # aggregate bar chart: time-to-90% target by world/condition
    worlds = sorted(summary.keys())
    if worlds:
        fig, ax = plt.subplots(figsize=(1.8 * len(worlds) + 3, 4.8))
        width = 0.26
        xs = np.arange(len(worlds))
        for j, cond in enumerate(CONDITIONS):
            vals = [summary[w].get(cond, {}).get('time_to_90pct_common_s')
                    or 0 for w in worlds]
            ax.bar(xs + (j - 1) * width, vals, width,
                   label=LABELS[cond], color=COLORS[cond])
        ax.set_xticks(xs)
        ax.set_xticklabels(worlds, rotation=15)
        ax.set_ylabel('time to 90% target [s]')
        ax.set_title('Time to reach 90% coverage target (lower = better)')
        ax.legend(frameon=False)
        ax.grid(alpha=0.3, axis='y')
        fig.tight_layout()
        out = os.path.join(fig_dir, 'summary_bars.png')
        fig.savefig(out, dpi=130)
        plt.close(fig)
        print('wrote', out)

    print('\n=== summary ===')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
