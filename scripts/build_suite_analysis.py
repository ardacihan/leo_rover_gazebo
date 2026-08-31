#!/usr/bin/env python3
"""Cross-condition analysis page for the overnight suite (Study A).

Compares single / independent / coordinated-unknown / coordinated-known on
every map:

  * team coverage vs sim time (mean line + min-max band per condition),
  * time to 80% of the map's best achieved coverage (bar + per-run dots),
  * total distance driven (bar + per-run dots).

Team coverage metric: the union of both rovers' own maps, overlaid with the
TRUE spawn offset - ground truth used for measurement only, so the metric is
identical and fair across conditions whether or not the run's own alignment
ever merged. Union is counted on a 0.1 m grid inside the world footprint.

Writes final/bundles/st-analysis/{index.html, assets/*, bundle.json} and
caches per-run series in assets/cache.json (delete to force recompute).

Usage: python3 scripts/build_suite_analysis.py
"""

import glob
import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from build_final_dashboard import (  # noqa: E402
    PAGE_CSS, world_geometry, parse_traj, traj_length, build_root_index)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'final', 'bundles', 'st-analysis')
ASSETS = os.path.join(OUT, 'assets')
CACHE = os.path.join(ASSETS, 'cache.json')

CONDS = ['single', 'indep', 'c2u', 'c2k']
LABEL = {'single': 'single robot', 'indep': '2 independent',
         'c2u': '2 coordinated (unknown start)',
         'c2k': '2 coordinated (known start)'}
COLOUR = {'single': '#2a78d6', 'indep': '#eb6834',
          'c2u': '#1baf7a', 'c2k': '#eda100'}
MAPS = {'office': 'office_world', 'depot': 'depot_world',
        'small': 'small_house', 'husarion': 'husarion_office'}
INK2 = '#52514e'
RES = 0.1  # union-grid cell, metres


def gt_offset(world):
    launch = os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch')
    if launch not in sys.path:
        sys.path.insert(0, launch)
    import spawn_poses
    return spawn_poses.relative_offset(world)


def union_series(run, world):
    """[(sim_t, union_m2)] for one run, ground-truth overlay metric."""
    rect, _ = world_geometry(world)
    x0, x1, y0, y1 = rect['leo1']
    W = int((x1 - x0) / RES) + 1
    H = int((y1 - y0) / RES) + 1
    off = gt_offset(world)
    c, s = math.cos(off[2]), math.sin(off[2])
    out = []
    snaps = sorted(glob.glob(os.path.join(run, 'timelapse', 'snap*.npz')))
    for path in snaps[::2]:
        try:
            d = np.load(path)
            grid = np.zeros((H, W), dtype=bool)
            for key, tf in (('leo1', None), ('leo2', (off[0], off[1], c, s))):
                if key not in d.files:
                    continue
                g = d[key]
                if g.size <= 1:
                    continue
                info = d[f'{key}_info']
                ys, xs = np.nonzero(g >= 0)
                if not len(ys):
                    continue
                wx = info[0] + (xs + 0.5) * info[2]
                wy = info[1] + (ys + 0.5) * info[2]
                if tf is not None:
                    ox, oy, cc, ss = tf
                    wx, wy = ox + cc * wx - ss * wy, oy + ss * wx + cc * wy
                ci = ((wx - x0) / RES).astype(int)
                ri = ((wy - y0) / RES).astype(int)
                ok = (ci >= 0) & (ci < W) & (ri >= 0) & (ri < H)
                grid[ri[ok], ci[ok]] = True
            out.append((float(d['t']), float(grid.sum()) * RES * RES))
        except (OSError, KeyError, ValueError):
            continue
    if out:
        t0 = out[0][0]
        out = [(t - t0, a) for t, a in out]
    return out


def run_metrics(run, world):
    series = union_series(run, world)
    d1 = traj_length(parse_traj(os.path.join(run, 'traj_leo1.csv')))
    d2 = traj_length(parse_traj(os.path.join(run, 'traj_leo2.csv')))
    return {'series': series, 'dist_m': d1 + d2,
            'final_m2': series[-1][1] if series else None}


def collect():
    if os.path.exists(CACHE):
        return json.load(open(CACHE))
    data = {}
    for short, world in MAPS.items():
        bundle = os.path.join(ROOT, 'final', 'bundles', f'st-coord-{short}')
        for run in sorted(glob.glob(os.path.join(bundle, 'runs', 'run_*'))):
            name = os.path.basename(run)
            cond = name.split('_')[1]
            if cond not in CONDS:
                continue
            print(f'  {short}/{name}', flush=True)
            data.setdefault(short, {}).setdefault(cond, []).append(
                run_metrics(run, world))
    os.makedirs(ASSETS, exist_ok=True)
    json.dump(data, open(CACHE, 'w'))
    return data


def t_to_frac(series, target_m2):
    for t, a in series:
        if a >= target_m2:
            return t
    return None


def chart_coverage(short, per_cond, out_png):
    fig, ax = plt.subplots(figsize=(8.8, 4.6))
    tmax = 0
    for cond in CONDS:
        runs = per_cond.get(cond, [])
        if not runs:
            continue
        # resample every run to a common time base
        end = max(r['series'][-1][0] for r in runs if r['series'])
        base = np.arange(0, end + 1, 10.0)
        tmax = max(tmax, end)
        curves = []
        for r in runs:
            if not r['series']:
                continue
            ts = [p[0] for p in r['series']]
            ys = [p[1] for p in r['series']]
            curves.append(np.interp(base, ts, ys))
        if not curves:
            continue
        arr = np.vstack(curves)
        ax.plot(base / 60, arr.mean(axis=0), color=COLOUR[cond], lw=2,
                label=f'{LABEL[cond]} (n={len(curves)})')
        ax.fill_between(base / 60, arr.min(axis=0), arr.max(axis=0),
                        color=COLOUR[cond], alpha=0.13, lw=0)
    ax.set_xlabel('simulated minutes')
    ax.set_ylabel('team-known area (m²)')
    ax.set_xlim(0, tmax / 60)
    ax.set_ylim(bottom=0)
    ax.legend(loc='lower right', fontsize=9)
    ax.set_title(f'{MAPS[short]} — coverage by condition '
                 f'(line = mean, band = min–max)', fontsize=11,
                 color=INK2, loc='left')
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def chart_bars(short, per_cond, value_fn, ylabel, title, out_png,
               lower_better=True):
    fig, ax = plt.subplots(figsize=(8.8, 4.0))
    xs, labels = [], []
    for i, cond in enumerate(CONDS):
        vals = [v for v in (value_fn(r) for r in per_cond.get(cond, []))
                if v is not None]
        labels.append(LABEL[cond].replace(' (', '\n('))
        if not vals:
            continue
        ax.bar(i, float(np.mean(vals)), width=0.62, color=COLOUR[cond],
               alpha=0.85)
        ax.scatter([i + 0.02] * len(vals), vals, color='#0b0b0b', s=16,
                   zorder=3)
        xs.append(i)
    ax.set_xticks(range(len(CONDS)))
    ax.set_xticklabels(labels, fontsize=8.5)
    ax.set_ylabel(ylabel)
    ax.set_title(f'{MAPS[short]} — {title}', fontsize=11, color=INK2,
                 loc='left')
    fig.tight_layout()
    fig.savefig(out_png, dpi=110)
    plt.close(fig)


def main():
    os.makedirs(ASSETS, exist_ok=True)
    print('collecting per-run metrics (cached after first pass)...')
    data = collect()

    sections = []
    for short in MAPS:
        per_cond = data.get(short, {})
        if not per_cond:
            continue
        best = max((r['final_m2'] or 0) for rs in per_cond.values()
                   for r in rs)
        target = 0.8 * best
        chart_coverage(short, per_cond,
                       os.path.join(ASSETS, f'{short}_coverage.png'))
        chart_bars(short, per_cond,
                   lambda r, tg=target: (lambda t: t / 60 if t else None)(
                       t_to_frac(r['series'], tg)),
                   'minutes to 80% coverage',
                   'time to 80% of best achieved coverage (dots = runs)',
                   os.path.join(ASSETS, f'{short}_t80.png'))
        chart_bars(short, per_cond, lambda r: r['dist_m'],
                   'metres driven (team total)',
                   'distance driven (dots = runs)',
                   os.path.join(ASSETS, f'{short}_dist.png'))
        n = sum(len(v) for v in per_cond.values())
        sections.append(f'''
<h2>{MAPS[short]} ({n} runs)</h2>
<div><img class="fig" loading="lazy" src="assets/{short}_coverage.png"></div>
<div class="grid2">
<div><img class="fig" loading="lazy" src="assets/{short}_t80.png"></div>
<div><img class="fig" loading="lazy" src="assets/{short}_dist.png"></div>
</div>''')

    body = f'''
<title>Suite Analysis — coordination</title>
<style>{PAGE_CSS}</style>
<div class="wrap">
<p class="back"><a href="../../index.html">← all sessions</a></p>
<h1>Suite analysis — single vs independent vs coordinated</h1>
<p class="sub">Cross-condition comparison from the overnight suite
(Study&nbsp;A). <b>Team coverage</b> is the union of both robots' own maps
overlaid with the true spawn offset — ground truth is used only to
<i>measure</i>, never by the algorithms, so the metric is identical for every
condition whether or not a run's own alignment merged. <b>Time to
completion</b> is the simulated time to reach 80% of the best coverage any
run achieved on that map (well-defined even for runs stopped at the time
cap). Bars are means; black dots are individual runs.</p>
{''.join(sections)}
<div class="card"><p class="note">Conditions: <b>single robot</b> — one
rover; <b>2 independent</b> — two rovers, no coordination; <b>2 coordinated
(unknown start)</b> — coordination with the relative pose estimated by
hybrid alignment; <b>2 coordinated (known start)</b> — the true relative
pose given at t=0. Same spawns and odometry seeds in every run. Raw data:
final/bundles/st-coord-*/runs.</p></div>
</div>'''
    open(os.path.join(OUT, 'index.html'), 'w').write(body)
    from datetime import datetime
    json.dump({'title': 'Suite analysis — coordination comparison',
               'note': 'cross-condition plots: coverage, time to 80%, distance',
               'started': datetime.now().isoformat(),
               'runs': []},
              open(os.path.join(OUT, 'bundle.json'), 'w'), indent=1)
    build_root_index(os.path.join(ROOT, 'final'))
    print('built', os.path.join(OUT, 'index.html'))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
