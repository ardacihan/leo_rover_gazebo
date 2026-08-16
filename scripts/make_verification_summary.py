#!/usr/bin/env python3
"""Build teammate-facing comparison artifacts from recorded sim runs."""

import re
from pathlib import Path

import matplotlib.pyplot as plt
from PIL import Image, ImageDraw, ImageFont, ImageOps


ROOT = Path(__file__).resolve().parents[1]
REPORTS = ROOT / 'reports'
OUT = REPORTS / 'comparison'
SAMPLE_RE = re.compile(r't=(\d+)s known=([\d.]+)m2')


def coverage(path, max_minutes=None):
    samples = []
    for match in SAMPLE_RE.finditer(path.read_text(errors='replace')):
        minute = int(match.group(1)) / 60.0
        if max_minutes is None or minute <= max_minutes:
            samples.append((minute, float(match.group(2))))
    return samples


def plot_coverage():
    runs = {
        'PR1 hardened frontier': coverage(
            REPORTS / 'pr1' / 'coverage.log'),
        'PR4 frontier + camera sweep': coverage(
            REPORTS / 'pr4' / 'coverage.log'),
        'PR2 RPP controller': coverage(
            REPORTS / 'pr2' / 'coverage.log', max_minutes=20),
        'PR3 escape recovery': coverage(
            REPORTS / 'pr3' / 'office_coverage.log'),
    }
    leo_panel = ['PR1 hardened frontier', 'PR4 frontier + camera sweep']
    office_panel = ['PR2 RPP controller', 'PR3 escape recovery']

    ghost_fix = REPORTS / 'pr3' / 'ghost_fix_run' / 'coverage.log'
    if ghost_fix.exists():
        runs['Ghost long-ban fix (full office run)'] = coverage(ghost_fix)
        office_panel.append('Ghost long-ban fix (full office run)')
    item_fixed = (REPORTS / 'pr4' / 'item_search_fixed' / 'coverage.log')
    if item_fixed.exists():
        runs['Item search rerun (fixed code)'] = coverage(item_fixed)
        leo_panel.append('Item search rerun (fixed code)')

    fig, axes = plt.subplots(1, 2, figsize=(16, 6.8), dpi=140)
    panels = [
        (
            axes[0],
            tuple(leo_panel),
            'leo_world: lidar map growth',
            80,
        ),
        (
            axes[1],
            tuple(office_panel),
            'husarion_office: adversarial geometry',
            max(20, int(max((s[0] for s in runs.get(
                'Ghost long-ban fix (full office run)', [(20, 0)])),
                default=20)) + 2),
        ),
    ]
    colors = {
        'PR1 hardened frontier': '#1565c0',
        'PR2 RPP controller': '#ef6c00',
        'PR3 escape recovery': '#2e7d32',
        'PR4 frontier + camera sweep': '#6a1b9a',
        'Ghost long-ban fix (full office run)': '#c62828',
        'Item search rerun (fixed code)': '#00838f',
    }

    for ax, labels, title, xmax in panels:
        for label in labels:
            samples = runs[label]
            x, y = zip(*samples)
            ax.plot(x, y, linewidth=2.5, label=label, color=colors[label])
            ax.scatter(x[-1], y[-1], s=28, color=colors[label], zorder=3)
        ax.set_title(title, fontweight='bold')
        ax.set_xlabel('Simulation time (minutes)')
        ax.set_ylabel('Known map area (m^2)')
        ax.set_xlim(0, xmax)
        ax.set_ylim(bottom=0)
        ax.grid(alpha=0.25)
        ax.legend(loc='lower right')

    axes[0].text(
        0.02,
        0.95,
        'PR4 lidar coverage plateaus early; the remaining time is the\n'
        'camera wall sweep that raises camera coverage from 38% to 90%.',
        transform=axes[0].transAxes,
        va='top',
        fontsize=9,
        bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': '#cccccc'},
    )
    axes[1].text(
        0.02,
        0.95,
        'PR2 is clipped to 20 min; its pre-PR3 pathological soak continued\n'
        'for 9.8 h. PR3 escaped seven consecutive real planner rejects.',
        transform=axes[1].transAxes,
        va='top',
        fontsize=9,
        bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': '#cccccc'},
    )

    fig.suptitle(
        'Exploration strategy comparison - recorded simulation runs',
        fontsize=16,
        fontweight='bold',
    )
    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(OUT / 'coverage_comparison.png', bbox_inches='tight')
    plt.close(fig)


def font(size, bold=False):
    family = 'DejaVuSans-Bold.ttf' if bold else 'DejaVuSans.ttf'
    try:
        return ImageFont.truetype(family, size)
    except OSError:
        return ImageFont.load_default()


def plot_maps():
    cards = [
        (
            REPORTS / 'pr1' / 'exploration_final.png',
            'PR1 - Hardened frontier',
            'leo_world | 390.8 m^2 | ~99% reachable area',
        ),
        (
            REPORTS / 'pr2' / 'exploration_final.png',
            'PR2 - RPP + rotation shim',
            'husarion_office | faster normal driving',
        ),
        (
            REPORTS / 'pr3' / 'office_escape_final.png',
            'PR3 - Last-resort escape',
            'husarion_office | 7 rejects -> reverse -> new goal',
        ),
        (
            REPORTS / 'pr4' / 'exploration_final.png',
            'PR4 - Camera wall sweep',
            'leo_world | 90% camera coverage | 6/6 items',
        ),
    ]
    ghost_fix_png = (REPORTS / 'pr3' / 'ghost_fix_run'
                     / 'exploration_final.png')
    if ghost_fix_png.exists():
        cards.append((
            ghost_fix_png,
            'PR3 follow-up - Ghost long-ban fix',
            'husarion_office | full run terminates + saves map',
        ))
    item_fixed_png = (REPORTS / 'pr4' / 'item_search_fixed'
                      / 'exploration_final.png')
    if item_fixed_png.exists():
        cards.append((
            item_fixed_png,
            'Final algorithm - item search rerun',
            'leo_world | fixed code | full map + items',
        ))

    rows = (len(cards) + 1) // 2
    margin, gap, header = 55, 35, 120
    width = 1500
    card_h_fixed = (1160 - header - 55 - 35) // 2
    height = header + margin + rows * card_h_fixed + (rows - 1) * gap
    card_w = (width - 2 * margin - gap) // 2
    card_h = card_h_fixed
    canvas = Image.new('RGB', (width, height), '#f5f7fa')
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (margin, 32),
        'Leo Rover exploration milestone - verification board',
        fill='#17212b',
        font=font(34, bold=True),
    )
    draw.text(
        (margin, 77),
        'Recorded June 11-12, 2026 | red line: rover trail | blue: final pose',
        fill='#52606d',
        font=font(20),
    )

    for index, (path, title, subtitle) in enumerate(cards):
        col, row = index % 2, index // 2
        x = margin + col * (card_w + gap)
        y = header + row * (card_h + gap)
        draw.rounded_rectangle(
            (x, y, x + card_w, y + card_h),
            radius=18,
            fill='white',
            outline='#d8dee6',
            width=2,
        )
        image_box = (card_w - 36, card_h - 112)
        image = Image.open(path).convert('RGB')
        image = ImageOps.contain(image, image_box, Image.Resampling.LANCZOS)
        image_x = x + (card_w - image.width) // 2
        image_y = y + 18 + (image_box[1] - image.height) // 2
        canvas.paste(image, (image_x, image_y))
        draw.text(
            (x + 22, y + card_h - 82),
            title,
            fill='#17212b',
            font=font(23, bold=True),
        )
        draw.text(
            (x + 22, y + card_h - 47),
            subtitle,
            fill='#52606d',
            font=font(17),
        )

    canvas.save(OUT / 'final_maps_comparison.png')


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    plot_coverage()
    plot_maps()
    print(f'Wrote comparison artifacts to {OUT}')


if __name__ == '__main__':
    main()
