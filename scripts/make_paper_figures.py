#!/usr/bin/env python3
"""Regenerate every report figure as a clean, LaTeX-ready image.

No suptitles, no per-panel titles, no annotation text-boxes, no card chrome --
just axes, data, legends and tight margins. Multi-panel figures are also split
into individual panels so each can be dropped into a paper as its own \\includegraphics.

Sources (all confirmed present in this repo):
  two-robot   : reports/collab_final/<world>_{single,independent,coordinated}/{coverage.log,traj.csv,*map*.pgm/.yaml}
  single-robot: reports/final_runs/<world>/{coverage.log,status.log,exploration_final.png}
                reports/pr{1,2,3,4}/{coverage.log,office_coverage.log}

Output: report_package/paper_figures/{two_robot,single_robot}/*.png
Run:    python scripts/make_paper_figures.py
"""

import os
import re
import shutil
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / 'reports'
OUT = ROOT / 'report_package' / 'paper_figures'
OUT_TWO = OUT / 'two_robot'
OUT_ONE = OUT / 'single_robot'

DPI = 200

# ---------- shared style ----------------------------------------------------

plt.rcParams.update({
    'font.size': 11,
    'axes.titlesize': 11,
    'axes.labelsize': 11,
    'legend.fontsize': 10,
    'xtick.labelsize': 10,
    'ytick.labelsize': 10,
    'savefig.bbox': 'tight',
    'savefig.pad_inches': 0.02,
    'figure.dpi': DPI,
})


def clean_axes(ax):
    ax.grid(alpha=0.2, linewidth=0.8)
    ax.spines[['top', 'right']].set_visible(False)


def save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=DPI)
    plt.close(fig)
    print('wrote', path.relative_to(ROOT))


# ---------- TWO-ROBOT: coverage / separation / maps -------------------------

COV = re.compile(r't=(\d+)s known=([\d.]+)m2')
CONDS = ['single', 'independent', 'coordinated']
COLORS = {'single': '#4C72B0', 'independent': '#DD8452', 'coordinated': '#55A868'}
LABELS = {'single': '1 robot (baseline)',
          'independent': '2 robots (uncoordinated)',
          'coordinated': '2 robots (coordinated)'}
ROBOT_COLORS = {'leo1': '#C44E52', 'leo2': '#8172B3'}
COLLAB = REPORTS / 'collab_final'


def parse_coverage(path):
    t, known = [], []
    if not os.path.exists(path):
        return np.array([]), np.array([])
    for m in COV.finditer(Path(path).read_text(errors='replace')):
        t.append(int(m.group(1)))
        known.append(float(m.group(2)))
    t, known = np.array(t, float), np.array(known, float)
    if len(t):
        t = t - t[0]
    return t, known


def load_map(run_dir):
    for base in ('merged_map', 'map'):
        pgm = run_dir / (base + '.pgm')
        yml = run_dir / (base + '.yaml')
        if pgm.exists() and yml.exists():
            img = plt.imread(str(pgm))
            res, ox, oy = 0.05, 0.0, 0.0
            for line in yml.read_text().splitlines():
                if line.startswith('resolution'):
                    res = float(line.split(':')[1])
                if line.startswith('origin'):
                    vals = re.findall(r'-?[\d.]+', line.split(':', 1)[1])
                    ox, oy = float(vals[0]), float(vals[1])
            return img, res, ox, oy
    return None, None, None, None


def load_traj(run_dir):
    path = run_dir / 'traj.csv'
    out = {}
    if not path.exists():
        return out
    lines = path.read_text().splitlines()[1:]
    for line in lines:
        p = line.strip().split(',')
        if len(p) != 4:
            continue
        _, r, x, y = p
        out.setdefault(r, []).append((float(x), float(y)))
    return {r: np.array(v) for r, v in out.items()}


def separation_series(run_dir):
    by_t = {}
    path = run_dir / 'traj.csv'
    if not path.exists():
        return np.array([]), np.array([])
    for line in path.read_text().splitlines()[1:]:
        p = line.strip().split(',')
        if len(p) != 4:
            continue
        t, r, x, y = float(p[0]), p[1], float(p[2]), float(p[3])
        by_t.setdefault(t, {})[r] = (x, y)
    ts, ds = [], []
    for t in sorted(by_t):
        d = by_t[t]
        if 'leo1' in d and 'leo2' in d:
            ts.append(t)
            ds.append(np.hypot(d['leo1'][0] - d['leo2'][0],
                               d['leo1'][1] - d['leo2'][1]))
    ts = np.array(ts)
    if len(ts):
        ts = ts - ts[0]
    return ts, np.array(ds)


def two_robot_coverage(world):
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    drew = False
    for cond in CONDS:
        t, k = parse_coverage(COLLAB / f'{world}_{cond}' / 'coverage.log')
        if not len(t):
            continue
        drew = True
        ax.plot(t, k, label=LABELS[cond], color=COLORS[cond], linewidth=2.2)
    if not drew:
        plt.close(fig)
        return
    ax.set_xlabel('time since exploration start [s]')
    ax.set_ylabel('mapped area  [m$^2$]')
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    clean_axes(ax)
    save(fig, OUT_TWO / f'coverage_{world}.png')


def two_robot_separation(world):
    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    drew = False
    for cond in ('independent', 'coordinated'):
        ts, ds = separation_series(COLLAB / f'{world}_{cond}')
        if not len(ts):
            continue
        drew = True
        ax.plot(ts, ds, color=COLORS[cond], linewidth=1.9,
                label=f'{LABELS[cond]} (mean {np.mean(ds):.1f} m)')
    if not drew:
        plt.close(fig)
        return
    ax.set_xlabel('time since exploration start [s]')
    ax.set_ylabel('inter-rover distance [m]')
    ax.set_ylim(bottom=0)
    ax.legend(frameon=False)
    clean_axes(ax)
    save(fig, OUT_TWO / f'separation_{world}.png')


def two_robot_map_panel(world, cond):
    run_dir = COLLAB / f'{world}_{cond}'
    img, res, ox, oy = load_map(run_dir)
    if img is None:
        return
    fig, ax = plt.subplots(figsize=(5.0, 5.0))
    ax.imshow(img, cmap='gray', origin='upper')
    ax.set_xticks([])
    ax.set_yticks([])
    for sp in ax.spines.values():
        sp.set_visible(False)
    h = img.shape[0]
    traj = load_traj(run_dir)
    for r, pts in traj.items():
        if not len(pts):
            continue
        px = (pts[:, 0] - ox) / res
        py = h - (pts[:, 1] - oy) / res
        ax.plot(px, py, color=ROBOT_COLORS.get(r, '#333'),
                linewidth=1.6, label=r)
        ax.scatter(px[0], py[0], c='lime', s=30, zorder=5)
    if traj:
        ax.legend(loc='lower right', fontsize=9, frameon=True)
    save(fig, OUT_TWO / f'maps_{world}_{cond}.png')


# ---------- SINGLE-ROBOT: verification (engine) coverage --------------------

FINAL = REPORTS / 'final_runs'
EBLUE, EAQUA, EYELLOW = '#2a78d6', '#1baf7a', '#eda100'
INK, INK2 = '#0b0b0b', '#52514e'
ENGINE_WORLDS = [('office_world', EBLUE), ('leo_world', EAQUA),
                 ('depot_world', EYELLOW)]


def eng_coverage(path, t0_area=None):
    samples = [(int(m.group(1)), float(m.group(2)))
               for m in COV.finditer(Path(path).read_text(errors='replace'))]
    if t0_area is not None:
        idx = next((i for i, (_, a) in enumerate(samples) if a > t0_area), 0)
        t0 = samples[idx][0]
        samples = [(t - t0, a) for t, a in samples[idx:]]
    return [(t / 60.0, a) for t, a in samples]


def camera_curve(status_log):
    pts = []
    text = Path(status_log).read_text(errors='replace')
    for m in re.finditer(r'"sim_time": ([\d.]+), "state": "[a-z]+", '
                         r'"frontiers": \d+, "coverage_m2": [\d.]+, '
                         r'"camera_coverage": ([\d.]+)', text):
        pts.append((float(m.group(1)) / 60.0, float(m.group(2)) * 100.0))
    return pts


def verify_lidar_growth():
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for name, color in ENGINE_WORLDS:
        x, y = zip(*eng_coverage(FINAL / name / 'coverage.log'))
        ax.plot(x, y, color=color, linewidth=2)
        ax.annotate(name, (x[-1], y[-1]), textcoords='offset points',
                    xytext=(-4, 8), ha='right', fontsize=9.5, color=INK)
    ax.set_xlabel('sim time (minutes)')
    ax.set_ylabel('known map area (m$^2$)')
    clean_axes(ax)
    save(fig, OUT_ONE / 'verify_lidar_growth.png')


def verify_camera_coverage():
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    for name, color in ENGINE_WORLDS:
        cam = [(t, p) for t, p in camera_curve(FINAL / name / 'status.log')
               if t > 2]
        if not cam:
            continue
        cx, cy = zip(*cam)
        ax.plot(cx, cy, color=color, linewidth=2)
        dy = {'depot_world': -16, 'leo_world': 10, 'office_world': 10}[name]
        ax.annotate(name, (cx[-1], cy[-1]), textcoords='offset points',
                    xytext=(-4, dy), ha='right', fontsize=9.5, color=INK)
    ax.set_xlabel('sim time (minutes)')
    ax.set_ylabel('wall cells observed (%)')
    ax.set_ylim(0, 100)
    clean_axes(ax)
    save(fig, OUT_ONE / 'verify_camera_coverage.png')


# ---------- SINGLE-ROBOT: PR strategy comparison ----------------------------

PR_COLORS = {
    'PR1 hardened frontier': '#1565c0',
    'PR2 RPP controller': '#ef6c00',
    'PR3 escape recovery': '#2e7d32',
    'PR4 frontier + camera sweep': '#6a1b9a',
}


def pr_coverage(path, max_minutes=None):
    samples = []
    for m in COV.finditer(Path(path).read_text(errors='replace')):
        minute = int(m.group(1)) / 60.0
        if max_minutes is None or minute <= max_minutes:
            samples.append((minute, float(m.group(2))))
    return samples


def pr_panel(runs, out_name, xmax):
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    for label, samples in runs:
        x, y = zip(*samples)
        ax.plot(x, y, linewidth=2.5, label=label, color=PR_COLORS[label])
        ax.scatter(x[-1], y[-1], s=28, color=PR_COLORS[label], zorder=3)
    ax.set_xlabel('simulation time (minutes)')
    ax.set_ylabel('known map area (m$^2$)')
    ax.set_xlim(0, xmax)
    ax.set_ylim(bottom=0)
    ax.legend(loc='lower right', frameon=False)
    clean_axes(ax)
    save(fig, OUT_ONE / out_name)


def pr_comparison():
    pr_panel(
        [('PR1 hardened frontier', pr_coverage(REPORTS / 'pr1' / 'coverage.log')),
         ('PR4 frontier + camera sweep', pr_coverage(REPORTS / 'pr4' / 'coverage.log'))],
        'pr_coverage_leo_world.png', xmax=80)
    pr_panel(
        [('PR2 RPP controller',
          pr_coverage(REPORTS / 'pr2' / 'coverage.log', max_minutes=20)),
         ('PR3 escape recovery',
          pr_coverage(REPORTS / 'pr3' / 'office_coverage.log'))],
        'pr_coverage_office_world.png', xmax=20)


# ---------- collect already-clean final map images --------------------------

def collect_maps():
    OUT_ONE.mkdir(parents=True, exist_ok=True)
    single_maps = {
        'map_office_world.png': FINAL / 'office_world' / 'exploration_final.png',
        'map_leo_world.png': FINAL / 'leo_world' / 'exploration_final.png',
        'map_depot_world.png': FINAL / 'depot_world' / 'exploration_final.png',
        'map_depot_aliased_slam.png':
            FINAL / 'depot_world_aliased_slam' / 'exploration_final.png',
    }
    for dst, src in single_maps.items():
        if src.exists():
            shutil.copy(src, OUT_ONE / dst)
            print('copied', (OUT_ONE / dst).relative_to(ROOT))
    # two-robot merged final maps already ship clean in report_package
    two_src = ROOT / 'report_package' / 'two_robot' / 'final_maps'
    if two_src.exists():
        OUT_TWO.mkdir(parents=True, exist_ok=True)
        for png in two_src.glob('*.png'):
            shutil.copy(png, OUT_TWO / f'finalmap_{png.name}')
            print('copied', (OUT_TWO / f'finalmap_{png.name}').relative_to(ROOT))


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    for world in ('office_world', 'depot_world'):
        two_robot_coverage(world)
        two_robot_separation(world)
        for cond in CONDS:
            two_robot_map_panel(world, cond)
    verify_lidar_growth()
    verify_camera_coverage()
    pr_comparison()
    collect_maps()
    print('\nAll clean figures in', OUT.relative_to(ROOT))


if __name__ == '__main__':
    main()
