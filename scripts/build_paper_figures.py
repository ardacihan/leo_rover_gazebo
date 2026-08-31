#!/usr/bin/env python3
"""The eight paper figures, from final/suite_metrics.json.

  fig1 coverage over time, four exploration modes, 95% confidence bands
  fig2 representative paths: independent vs coordinated vs known-start
  fig3 T90 boxplots per method across all four maps
  fig4 duplicated coverage (work done twice) per method
  fig5 alignment error over time: map-only / tag-only / hybrid
  fig6 landmark count (3 / 9 / 15): merge reliability, merge time, explore time
  fig7 method x map heatmap: success rate and speed
  fig8 the cost of not knowing where the other robot started

Writes final/figures/*.png (+ figures.json with the numbers behind them).
"""

import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'scripts'))
OUT = os.path.join(ROOT, 'final', 'figures')

C = {'single': '#2a78d6', 'indep': '#eb6834', 'c2u': '#1baf7a',
     'c2k': '#eda100', 'tag': '#e87ba4', 'mfree': '#4a3aa7'}
L = {'single': 'one robot', 'indep': 'two robots, not talking',
     'c2u': 'two robots, teamwork (start unknown)',
     'c2k': 'two robots, teamwork (start known)',
     'tag': 'markers only', 'mfree': 'map shapes only'}
MAPS = ['office_world', 'depot_world', 'small_house', 'husarion_office']
NICE = {'office_world': 'office', 'depot_world': 'depot',
        'small_house': 'house', 'husarion_office': 'cluttered office'}
INK, INK2, MUTED, GRID = '#0b0b0b', '#52514e', '#898781', '#e1e0d9'

plt.rcParams.update({
    'figure.facecolor': '#fcfcfb', 'axes.facecolor': '#fcfcfb',
    'axes.edgecolor': '#c3c2b7', 'axes.labelcolor': INK2,
    'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.7,
    'xtick.color': MUTED, 'ytick.color': MUTED, 'text.color': INK,
    'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False,
})


def load():
    rows = json.load(open(os.path.join(ROOT, 'final', 'suite_metrics.json')))
    man = json.load(open(os.path.join(ROOT, 'final',
                                      'suite_2026-08-30_manifest.json')))
    caps = {r['name']: r.get('cap') for r in man['runs']}
    for r in rows:
        r['cap'] = caps.get(r['run'])
    return rows


# Runs were launched in phases with different exploration time limits (5-12
# min). Comparing "time to finish" across conditions is only fair between
# runs that had the SAME time limit, so every cross-condition figure uses the
# largest matched-limit set for each building.
CORE_CAP = {'office_world': 8, 'depot_world': 7, 'small_house': 10,
            'husarion_office': 12}


def core(rows, world=None, cond=None):
    out = [r for r in rows if r['cap'] == CORE_CAP.get(r['world'])]
    if world:
        out = [r for r in out if r['world'] == world]
    if cond:
        out = [r for r in out if r['cond'] == cond]
    return out


def target_area(rows, world):
    """90% of the typical (median) final map size for this building."""
    vals = [r['final_union'] for r in core(rows, world) if r['final_union']]
    return 0.9 * float(np.median(vals)) if vals else None


def sel(rows, **kw):
    out = rows
    for k, v in kw.items():
        out = [r for r in out if r[k] == v] if not isinstance(v, (list, tuple)) \
            else [r for r in out if r[k] in v]
    return out


def best_union(rows, world):
    vals = [r['final_union'] for r in sel(rows, world=world)
            if r['final_union']]
    return max(vals) if vals else None


def t_reach(series, target):
    for row in series:
        if row[1] >= target:
            return row[0] / 60.0
    return None


def mean_ci(arr):
    """mean and 95% CI half-width (t-free normal approx; n noted in caption)."""
    a = np.asarray(arr, dtype=float)
    m = np.nanmean(a, axis=0)
    n = np.sum(~np.isnan(a), axis=0)
    sd = np.nanstd(a, axis=0, ddof=1) if a.shape[0] > 1 else np.zeros_like(m)
    with np.errstate(invalid='ignore', divide='ignore'):
        ci = 1.96 * sd / np.sqrt(np.maximum(n, 1))
    return m, ci


# ------------------------------------------------------------------ fig 1
def fig1(rows):
    fig, axes = plt.subplots(2, 2, figsize=(12.5, 8.2))
    for ax, world in zip(axes.ravel(), MAPS):
        for cond in ('single', 'indep', 'c2u', 'c2k'):
            rs = [r for r in core(rows, world, cond) if r['series']]
            if not rs:
                continue
            # A run that finished early keeps the map it built, so hold
            # its last value (np.interp clamps) rather than dropping it.
            end = float(np.median([r['series'][-1][0] for r in rs]))
            base = np.arange(0, end + 1, 10.0)
            curves = np.vstack([
                np.interp(base, [p[0] for p in r['series']],
                          [p[1] for p in r['series']]) for r in rs])
            m, ci = mean_ci(curves)
            ax.plot(base / 60, m, color=C[cond], lw=2,
                    label=f'{L[cond]}  (n={len(rs)})')
            ax.fill_between(base / 60, m - ci, m + ci, color=C[cond],
                            alpha=0.18, lw=0)
        ax.set_title(NICE[world], fontsize=11, color=INK2, loc='left')
        ax.set_xlabel('minutes')
        ax.set_ylabel('area mapped by the team (m²)')
        ax.set_ylim(bottom=0)
        ax.legend(fontsize=7.5, loc='lower right')
    fig.suptitle('How fast the map gets filled in, by team setup '
                 '(line = average, shaded = 95% confidence)',
                 fontsize=12, x=0.02, ha='left', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    fig.savefig(os.path.join(OUT, 'fig1_coverage_curves.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 2
def fig2(rows):
    from render_multirobot_media import read_pgm
    panels = [('single', 'one robot'), ('indep', 'two robots, not talking'),
              ('c2u', 'teamwork, start unknown'),
              ('c2k', 'teamwork, start known')]
    fig, axes = plt.subplots(1, 4, figsize=(17, 4.6))
    for ax, (cond, title) in zip(axes, panels):
        rs = [r for r in sel(rows, world='office_world', cond=cond)
              if r['final_union']]
        if not rs:
            ax.axis('off')
            continue
        rs.sort(key=lambda r: -r['final_union'])
        run = rs[len(rs) // 2]['path']          # median-ish representative
        # Always robot 1's own map as the backdrop: a merged map carries the
        # merge's own error, which would make the four panels look different
        # for a reason that has nothing to do with the paths being compared.
        stem = 'leo1_map'
        ypath = os.path.join(run, f'{stem}.yaml')
        origin, res = [0, 0], 0.05
        for line in open(ypath):
            if line.startswith('origin'):
                origin = [float(v) for v in
                          line.split('[')[1].split(']')[0].split(',')[:2]]
            if line.startswith('resolution'):
                res = float(line.split(':')[1])
        img = read_pgm(os.path.join(run, f'{stem}.pgm'))
        h, w = img.shape
        ax.imshow(np.flipud(img), cmap='gray', vmin=0, vmax=254, origin='lower',
                  extent=[origin[0], origin[0] + w * res,
                          origin[1], origin[1] + h * res])
        import csv as _csv
        for rob, col in (('leo1', '#2a78d6'), ('leo2', '#eb6834')):
            p = os.path.join(run, f'traj_{rob}.csv')
            if not os.path.exists(p):
                continue
            xs, ys = [], []
            for r in _csv.DictReader(open(p)):
                try:
                    xs.append(float(r['x'])); ys.append(float(r['y']))
                except (KeyError, ValueError):
                    pass
            if not xs:
                continue
            if rob == 'leo2' and cond != 'single':
                import spawn_poses
                sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo',
                                                'launch'))
                off = spawn_poses.relative_offset('office_world')
                c, s = math.cos(off[2]), math.sin(off[2])
                xs, ys = ([off[0] + c * x - s * y for x, y in zip(xs, ys)],
                          [off[1] + s * x + c * y for x, y in zip(xs, ys)])
            ax.plot(xs, ys, color=col, lw=1.3, alpha=0.95)
        ax.set_title(title, fontsize=10.5, color=INK2)
        ax.set_xticks([]); ax.set_yticks([]); ax.grid(False)
    fig.suptitle('Where the robots actually drove (office map, one typical run '
                 'each) — blue = robot 1, orange = robot 2',
                 fontsize=12, x=0.02, ha='left', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.93))
    fig.savefig(os.path.join(OUT, 'fig2_trajectories.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 3
def fig3(rows, stats):
    fig, ax = plt.subplots(figsize=(12.5, 5.0))
    conds = ('single', 'indep', 'c2u', 'c2k')
    width, pos, ticks, labels = 0.19, [], [], []
    for gi, world in enumerate(MAPS):
        tgt = target_area(rows, world)
        for ci_, cond in enumerate(conds):
            rs = core(rows, world, cond)
            vals = [t for t in (t_reach(r['series'], tgt) for r in rs)
                    if t is not None]
            censored = len(rs) - len(vals)
            x = gi + (ci_ - 1.5) * width
            if vals:
                bp = ax.boxplot([vals], positions=[x], widths=width * 0.85,
                                patch_artist=True, showfliers=False,
                                medianprops=dict(color=INK, lw=1.4))
                bp['boxes'][0].set(facecolor=C[cond], alpha=0.75,
                                   edgecolor='#c3c2b7')
                ax.scatter([x] * len(vals), vals, s=12, color=INK, zorder=3,
                           alpha=0.75)
            if censored:
                ax.text(x, ax.get_ylim()[1] * 0.02, f'{censored}✗',
                        ha='center', fontsize=7.5, color='#d03b3b')
            stats.setdefault('t90', {}).setdefault(world, {})[cond] = dict(
                n=len(rs), reached=len(vals),
                median=float(np.median(vals)) if vals else None,
                mean=float(np.mean(vals)) if vals else None)
        ticks.append(gi); labels.append(NICE[world])
    ax.set_xticks(ticks); ax.set_xticklabels(labels)
    ax.set_ylabel('minutes to map 90% of the typical final map')
    handles = [plt.Line2D([], [], color=C[c], lw=6, label=L[c]) for c in conds]
    ax.legend(handles=handles, fontsize=8.5, loc='upper left')
    ax.set_title('Time to finish the job (lower is better). Only runs with '
                 'the same time limit are compared. Dots are single runs; '
                 'a red ✗ count is runs that never got there.',
                 fontsize=11, color=INK2, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig3_t90_box.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 4
def fig4(rows, stats):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))
    conds = ('indep', 'c2u', 'c2k')
    width = 0.26
    for gi, world in enumerate(MAPS):
        for ci_, cond in enumerate(conds):
            rs = [r for r in core(rows, world, cond)
                  if r['final_dup'] is not None]
            if not rs:
                continue
            dup = [r['final_dup'] for r in rs]
            frac = [100.0 * r['final_dup'] / r['final_union'] for r in rs
                    if r['final_union']]
            x = gi + (ci_ - 1) * width
            ax1.bar(x, np.mean(dup), width * 0.9, color=C[cond], alpha=0.85)
            ax1.scatter([x] * len(dup), dup, s=10, color=INK, zorder=3)
            ax2.bar(x, np.mean(frac), width * 0.9, color=C[cond], alpha=0.85)
            ax2.scatter([x] * len(frac), frac, s=10, color=INK, zorder=3)
            stats.setdefault('dup', {}).setdefault(world, {})[cond] = dict(
                mean_m2=float(np.mean(dup)), mean_pct=float(np.mean(frac)))
    for ax, ylab in ((ax1, 'area both robots mapped (m²)'),
                     (ax2, 'share of the map covered twice (%)')):
        ax.set_xticks(range(len(MAPS)))
        ax.set_xticklabels([NICE[m] for m in MAPS])
        ax.set_ylabel(ylab)
    handles = [plt.Line2D([], [], color=C[c], lw=6, label=L[c]) for c in conds]
    ax1.legend(handles=handles, fontsize=8, loc='upper left')
    fig.suptitle('Wasted effort: how much of the building both robots mapped '
                 'separately', fontsize=12, x=0.02, ha='left', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(os.path.join(OUT, 'fig4_overlap.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 5
def fig5(rows, stats):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6),
                                   gridspec_kw={'width_ratios': [2, 1]})
    worlds = ('office_world', 'depot_world', 'small_house')
    for cond in ('mfree', 'tag', 'c2u'):
        # Only runs that lasted at least the 6-minute window, so "did it
        # ever merge" means the same thing for every method.
        rs = [r for r in rows if r['cond'] == cond and r['world'] in worlds
              and r['align'] and (r['cap'] or 0) >= 6]
        if not rs:
            continue
        base = np.arange(0, 6 * 60, 15.0)
        curves = []
        for r in rs:
            ts = [a[0] for a in r['align'] if a[1] is not None]
            es = [a[1] for a in r['align'] if a[1] is not None]
            if len(ts) < 2:
                continue
            curves.append(np.interp(base, ts, es, left=np.nan, right=np.nan))
            # Every run drawn faintly: averaging runs that start estimating at
            # different moments invents spikes that no single run ever had.
            ax1.plot(base / 60, curves[-1], color=C[cond], lw=0.8, alpha=0.25)
        if not curves:
            continue
        med = np.nanmedian(np.vstack(curves), axis=0)
        lab = {'c2u': 'both together (hybrid)', 'tag': 'markers only',
               'mfree': 'map shapes only'}[cond]
        ax1.plot(base / 60, med, color=C[cond], lw=2.4,
                 label=f'{lab}  (n={len(curves)}, median)')
        locked = [r for r in rs
                  if r.get('lock_t') is not None and r['lock_t'] <= 6 * 60]
        stats.setdefault('align', {})[cond] = dict(
            n=len(rs), locked=len(locked),
            lock_rate=100.0 * len(locked) / len(rs),
            mean_lock_min=float(np.mean([r['lock_t'] / 60 for r in locked]))
            if locked else None)
    ax1.axhline(0.5, color=MUTED, ls='--', lw=1)
    ax1.annotate('half a metre', (0.15, 0.52), color=MUTED, fontsize=8)
    ax1.set_xlabel('minutes'); ax1.set_ylabel('how far off the merge is (m)')
    ax1.set_ylim(0, 3); ax1.legend(fontsize=8.5)
    ax1.set_title('Getting the two maps lined up (thin lines = single runs)',
                  fontsize=11, color=INK2, loc='left')
    order = ['mfree', 'tag', 'c2u']
    vals = [stats['align'][c]['lock_rate'] for c in order if c in stats.get('align', {})]
    ax2.bar(range(len(vals)), vals, color=[C[c] for c in order], alpha=0.85)
    ax2.set_xticks(range(len(order)))
    ax2.set_xticklabels(['map\nshapes', 'markers', 'both'], fontsize=9)
    ax2.set_ylabel('runs merged within 6 minutes (%)'); ax2.set_ylim(0, 100)
    ax2.set_title('Did it merge at all?', fontsize=11, color=INK2, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig5_alignment.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 6
def fig6(rows, stats):
    variants = [('small_house_l3', '3 markers'), ('small_house_l9', '9 markers'),
                ('small_house_l15', '15 markers')]
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.3))
    xs = range(len(variants))
    rate, lock, expl, seen = [], [], [], []
    for world, _ in variants:
        rs = [r for r in sel(rows, world=world) if r['cap'] == 10]
        lk = [r['lock_t'] / 60 for r in rs if r.get('lock_t') is not None]
        rate.append(100.0 * len(lk) / len(rs) if rs else 0)
        lock.append(np.mean(lk) if lk else np.nan)
        vals = [r['final_union'] for r in rs if r['final_union']]
        tgt = 0.9 * float(np.median(vals)) if vals else None
        t = [x for x in (t_reach(r['series'], tgt) for r in rs) if x]
        expl.append(np.mean(t) if t else np.nan)
        seen.append(np.mean([r['n_markers'] for r in rs]) if rs else 0)
        stats.setdefault('lmk', {})[world] = dict(
            n=len(rs), lock_rate=rate[-1],
            mean_lock_min=float(lock[-1]) if not np.isnan(lock[-1]) else None,
            mean_t90_min=float(expl[-1]) if not np.isnan(expl[-1]) else None,
            mean_markers_found=float(seen[-1]))
    for ax, vals, lab, colr in (
            (axes[0], rate, 'runs that merged (%)', '#1baf7a'),
            (axes[1], lock, 'minutes until the maps merged', '#2a78d6'),
            (axes[2], expl, 'minutes to map 90%', '#eb6834')):
        ax.bar(xs, vals, 0.55, color=colr, alpha=0.85)
        ax.set_xticks(list(xs))
        ax.set_xticklabels([v[1] for v in variants])
        ax.set_ylabel(lab)
        for x, v in zip(xs, vals):
            if not (isinstance(v, float) and np.isnan(v)):
                ax.text(x, v, f'{v:.0f}' if v > 5 else f'{v:.1f}',
                        ha='center', va='bottom', fontsize=9, color=INK2)
    axes[0].set_ylim(0, 105)
    fig.suptitle('More markers on the walls = the two maps join up more often '
                 'and sooner (house map)', fontsize=12, x=0.02, ha='left',
                 color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.9))
    fig.savefig(os.path.join(OUT, 'fig6_landmarks.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 7
def fig7(rows, stats):
    conds = ('single', 'indep', 'c2u', 'c2k')
    succ = np.full((len(conds), len(MAPS)), np.nan)
    speed = np.full((len(conds), len(MAPS)), np.nan)
    for j, world in enumerate(MAPS):
        tgt = target_area(rows, world)
        for i, cond in enumerate(conds):
            rs = core(rows, world, cond)
            if not rs:
                continue
            good = [r for r in rs
                    if r['final_union'] and r['final_union'] >= tgt
                    and not r['aborted']]
            succ[i, j] = 100.0 * len(good) / len(rs)
            t = [x for x in (t_reach(r['series'], tgt) for r in rs) if x]
            speed[i, j] = np.mean(t) if t else np.nan
            stats.setdefault('grid', {}).setdefault(world, {})[cond] = dict(
                n=len(rs), success_pct=succ[i, j],
                mean_t90=None if np.isnan(speed[i, j]) else float(speed[i, j]))
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13.5, 4.4))
    for ax, data, cmap, title, fmt in (
            (ax1, succ, 'Greens', 'How often it worked (% of runs that mapped '
             '90%+ without a robot giving up)', '{:.0f}%'),
            (ax2, speed, 'Blues_r', 'How long it took (minutes to 90%)',
             '{:.1f}')):
        im = ax.imshow(data, cmap=cmap, aspect='auto')
        ax.set_xticks(range(len(MAPS)))
        ax.set_xticklabels([NICE[m] for m in MAPS], fontsize=9)
        ax.set_yticks(range(len(conds)))
        ax.set_yticklabels([L[c] for c in conds], fontsize=8.5)
        ax.grid(False)
        for i in range(len(conds)):
            for j in range(len(MAPS)):
                v = data[i, j]
                ax.text(j, i, '—' if np.isnan(v) else fmt.format(v),
                        ha='center', va='center', fontsize=9,
                        color='#0b0b0b')
        ax.set_title(title, fontsize=10, color=INK2, loc='left')
        fig.colorbar(im, ax=ax, shrink=0.85)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig7_heatmap.png'), dpi=130)
    plt.close(fig)


# ------------------------------------------------------------------ fig 8
def fig8(rows, stats):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.6))
    gaps, labels, t_u, t_k = [], [], [], []
    for world in MAPS:
        tgt = target_area(rows, world)
        tu = [x for x in (t_reach(r['series'], tgt)
                          for r in core(rows, world, 'c2u')) if x]
        tk = [x for x in (t_reach(r['series'], tgt)
                          for r in core(rows, world, 'c2k')) if x]
        if not tu or not tk:
            continue
        labels.append(NICE[world])
        t_u.append(np.mean(tu)); t_k.append(np.mean(tk))
        gaps.append(100.0 * (np.mean(tu) - np.mean(tk)) / np.mean(tu))
        stats.setdefault('oracle', {})[world] = dict(
            t90_unknown=float(np.mean(tu)), t90_known=float(np.mean(tk)),
            lost_pct=float(gaps[-1]),
            mean_lock_min=float(np.mean([r['lock_t'] / 60 for r in
                                         core(rows, world, 'c2u')
                                         if r.get('lock_t')] or [0])))
    x = np.arange(len(labels))
    ax1.bar(x - 0.19, t_u, 0.36, color=C['c2u'], alpha=0.85,
            label='start unknown (has to work it out)')
    ax1.bar(x + 0.19, t_k, 0.36, color=C['c2k'], alpha=0.85,
            label='start known (told at the beginning)')
    ax1.set_xticks(x); ax1.set_xticklabels(labels)
    ax1.set_ylabel('minutes to map 90%'); ax1.legend(fontsize=8.5)
    ax1.set_title('Same teamwork, only difference is knowing where the other '
                  'robot started', fontsize=10.5, color=INK2, loc='left')
    ax2.bar(x, gaps, 0.5, color='#d03b3b', alpha=0.8)
    for xi, g in zip(x, gaps):
        ax2.text(xi, g, f'{g:.0f}%', ha='center', va='bottom', fontsize=10,
                 color=INK2)
    ax2.set_xticks(x); ax2.set_xticklabels(labels)
    ax2.set_ylabel('extra time spent (%)')
    ax2.set_title('Time lost purely because the maps had to be lined up first',
                  fontsize=10.5, color=INK2, loc='left')
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, 'fig8_oracle_gap.png'), dpi=130)
    plt.close(fig)


def main():
    os.makedirs(OUT, exist_ok=True)
    sys.path.insert(0, os.path.join(ROOT, 'src', 'leo_rover_gazebo', 'launch'))
    rows = load()
    stats = {}
    fig1(rows); print('fig1')
    fig2(rows); print('fig2')
    fig3(rows, stats); print('fig3')
    fig4(rows, stats); print('fig4')
    fig5(rows, stats); print('fig5')
    fig6(rows, stats); print('fig6')
    fig7(rows, stats); print('fig7')
    fig8(rows, stats); print('fig8')
    json.dump(stats, open(os.path.join(OUT, 'figures.json'), 'w'), indent=1)
    print('wrote', OUT)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
