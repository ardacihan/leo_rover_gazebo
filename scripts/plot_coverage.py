#!/usr/bin/env python3
"""Plot a coverage curve from a map_coverage.py log.

Usage: python3 plot_coverage.py <coverage.log> <out.png> [label]
Lines look like:
    coverage: t=150s known=288.2m2 free=279.0m2 occ=9.2m2 (400x399)
"""

import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

LINE = re.compile(
    r't=(\d+)s known=([\d.]+)m2 free=([\d.]+)m2 occ=([\d.]+)m2')


def parse(path):
    t, known, free, occ = [], [], [], []
    with open(path) as f:
        for line in f:
            m = LINE.search(line)
            if m:
                t.append(int(m.group(1)))
                known.append(float(m.group(2)))
                free.append(float(m.group(3)))
                occ.append(float(m.group(4)))
    return t, known, free, occ


def main():
    log, out = sys.argv[1], sys.argv[2]
    label = sys.argv[3] if len(sys.argv) > 3 else 'run'
    t, known, free, occ = parse(log)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.plot(t, known, label='known area', linewidth=2)
    ax.plot(t, free, label='free', linestyle='--')
    ax.plot(t, occ, label='occupied', linestyle=':')
    ax.set_xlabel('sim time [s]')
    ax.set_ylabel('area [m²]')
    ax.set_title(f'Map coverage over time — {label}')
    ax.legend()
    ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    print(f'wrote {out} ({len(t)} samples, final known={known[-1] if known else 0} m2)')


if __name__ == '__main__':
    main()
