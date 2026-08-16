#!/usr/bin/env python3
"""Build the exploration-engine report figures from the July 2026 runs.

Inputs (produced by scripts/auto_explore_run.sh):
  reports/comparison_run/{explore_lite,custom}/coverage.log
  reports/final_runs/{office_world,leo_world,depot_world}/{coverage,status,explorer}.log

Outputs -> reports/engine_report/:
  explore_lite_vs_custom.png   head-to-head on leo_world
  final_runs_coverage.png      lidar area + camera coverage, three worlds
  final_maps_board.png         final map card collage
"""

import json
import re
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'reports' / 'engine_report'
COV_RE = re.compile(r't=(\d+)s known=([\d.]+)m2')

# dataviz reference palette (validated, fixed slot order)
BLUE, AQUA, YELLOW = '#2a78d6', '#1baf7a', '#eda100'
INK, INK2 = '#0b0b0b', '#52514e'


def coverage(path, t0_area=None):
    """[(t_min, m2)] from a coverage log; optionally re-zero t at the first
    sample whose area exceeds t0_area (aligns runs on explorer start)."""
    samples = [(int(m.group(1)), float(m.group(2)))
               for m in COV_RE.finditer(Path(path).read_text(errors='replace'))]
    if t0_area is not None:
        idx = next((i for i, (_, a) in enumerate(samples) if a > t0_area), 0)
        t0 = samples[idx][0]
        samples = [(t - t0, a) for t, a in samples[idx:]]
    return [(t / 60.0, a) for t, a in samples]


def camera_curve(status_log):
    """[(t_min, pct)] camera coverage from a status log."""
    pts = []
    text = Path(status_log).read_text(errors='replace')
    for m in re.finditer(r'"sim_time": ([\d.]+), "state": "[a-z]+", '
                         r'"frontiers": \d+, "coverage_m2": [\d.]+, '
                         r'"camera_coverage": ([\d.]+)', text):
        pts.append((float(m.group(1)) / 60.0, float(m.group(2)) * 100.0))
    return pts


def style(ax):
    ax.grid(alpha=0.18, linewidth=0.8)
    ax.spines[['top', 'right']].set_visible(False)
    ax.spines[['left', 'bottom']].set_color('#c9c8c2')
    ax.tick_params(colors=INK2, labelsize=9)
    ax.xaxis.label.set_color(INK2)
    ax.yaxis.label.set_color(INK2)
    ax.title.set_color(INK)


def head_to_head():
    custom = coverage(ROOT / 'reports/comparison_run/custom/coverage.log', 30)
    lite = coverage(ROOT / 'reports/comparison_run/explore_lite/coverage.log', 30)

    fig, ax = plt.subplots(figsize=(9, 5.2), dpi=150)
    for samples, color, label in ((custom, BLUE, 'custom frontier explorer'),
                                  (lite, AQUA, 'explore_lite')):
        x, y = zip(*samples)
        ax.plot(x, y, color=color, linewidth=2)
        ax.scatter(x[-1], y[-1], s=24, color=color, zorder=3)
        ax.annotate(label, (x[-1], y[-1]),
                    textcoords='offset points', xytext=(-6, 10),
                    ha='right', fontsize=9.5, color=INK)
    ax.set_xlabel('minutes since explorer start (sim time)')
    ax.set_ylabel('known map area (m²)')
    ax.set_title('leo_world head-to-head: identical sim, SLAM, Nav2 and start jog',
                 fontsize=12, fontweight='bold')
    ax.text(0.99, 0.42,
            'Both reach full coverage (~390 m² reachable) in ~6 min.\n'
            'The custom plateau reads ~5% high: a late wall double-edge\n'
            'inflates "known" cells. Custom then terminates, returns to\n'
            'start and saves the map; explore_lite (this run) stopped at\n'
            '12.5 min. Without the start jog explore_lite livelocks on an\n'
            'instantly-reached first goal (archived evidence).',
            transform=ax.transAxes, ha='right', va='top', fontsize=8.5,
            color=INK2, linespacing=1.4)
    style(ax)
    fig.tight_layout()
    fig.savefig(OUT / 'explore_lite_vs_custom.png')
    plt.close(fig)


def final_runs():
    runs = [
        ('office_world', BLUE), ('leo_world', AQUA), ('depot_world', YELLOW),
    ]
    fig, axes = plt.subplots(1, 2, figsize=(13, 5), dpi=150)
    for name, color in runs:
        base = ROOT / 'reports/final_runs' / name
        x, y = zip(*coverage(base / 'coverage.log'))
        axes[0].plot(x, y, color=color, linewidth=2)
        axes[0].annotate(name, (x[-1], y[-1]), textcoords='offset points',
                         xytext=(-4, 8), ha='right', fontsize=9.5, color=INK)
        # skip the first 2 min: with a handful of wall cells the ratio is noise
        cam = [(t, p) for t, p in camera_curve(base / 'status.log') if t > 2]
        if cam:
            cx, cy = zip(*cam)
            axes[1].plot(cx, cy, color=color, linewidth=2)
            dy = {'depot_world': -16, 'leo_world': 10, 'office_world': 10}[name]
            axes[1].annotate(name, (cx[-1], cy[-1]), textcoords='offset points',
                             xytext=(-4, dy), ha='right', fontsize=9.5, color=INK)
    axes[0].set_title('lidar map growth', fontsize=11, fontweight='bold')
    axes[0].set_ylabel('known map area (m²)')
    axes[1].set_title('camera wall coverage (sweep phase)', fontsize=11,
                      fontweight='bold')
    axes[1].set_ylabel('wall cells observed (%)')
    axes[1].set_ylim(0, 100)
    for ax in axes:
        ax.set_xlabel('sim time (minutes)')
        style(ax)
    fig.suptitle('Item-search runs on three worlds - frontier phase, camera sweep, return',
                 fontsize=13, fontweight='bold', color=INK)
    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(OUT / 'final_runs_coverage.png')
    plt.close(fig)


def font(size, bold=False):
    name = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(name, size)
    except OSError:
        return ImageFont.load_default()


def maps_board(cards, out_name, title, subtitle):
    rows = (len(cards) + 1) // 2
    margin, gap, header = 50, 30, 110
    width = 1480
    card_h = 520
    card_w = (width - 2 * margin - gap) // 2
    height = header + margin + rows * card_h + (rows - 1) * gap
    canvas = Image.new('RGB', (width, height), '#fcfcfb')
    draw = ImageDraw.Draw(canvas)
    draw.text((margin, 28), title, fill=INK, font=font(32, bold=True))
    draw.text((margin, 72), subtitle, fill=INK2, font=font(18))
    for i, (path, name, caption) in enumerate(cards):
        col, row = i % 2, i // 2
        x = margin + col * (card_w + gap)
        y = header + row * (card_h + gap)
        draw.rounded_rectangle((x, y, x + card_w, y + card_h), radius=14,
                               fill='white', outline='#d8dee6', width=2)
        box = (card_w - 32, card_h - 104)
        img = Image.open(path).convert('RGB')
        img = ImageOps.contain(img, box, Image.Resampling.LANCZOS)
        canvas.paste(img, (x + (card_w - img.width) // 2,
                           y + 16 + (box[1] - img.height) // 2))
        draw.text((x + 20, y + card_h - 76), name, fill=INK,
                  font=font(21, bold=True))
        draw.text((x + 20, y + card_h - 44), caption, fill=INK2, font=font(15))
    canvas.save(OUT / out_name)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    head_to_head()
    final_runs()
    maps_board(
        [
            (ROOT / 'reports/final_runs/office_world/exploration_final.png',
             'office_world (new, corridor + 5 offices)',
             '24x16 m | 8/8 items | 87% camera coverage | clean walls'),
            (ROOT / 'reports/final_runs/leo_world/exploration_final.png',
             'leo_world (benchmark arena)',
             '20x20 m | 6/6 items | 90% camera coverage | clean walls'),
            (ROOT / 'reports/final_runs/depot_world/exploration_final.png',
             'depot_world (new, small rooms)',
             '14x14 m | 4/4 items | clean walls'),
            (ROOT / 'reports/final_runs/depot_world_aliased_slam/exploration_final.png',
             'depot_world v1 - SLAM failure case study',
             'aliased corridor geometry -> wrong loop closure (kept as evidence)'),
        ],
        'final_maps_board.png',
        'Item-search verification - final maps, July 5 2026',
        'red: rover trail | blue: final pose | all runs headless, '
        'full stack (Gazebo + slam_toolbox + Nav2 + frontier/sweep explorer)',
    )
    print(f'wrote {OUT}')


if __name__ == '__main__':
    main()
