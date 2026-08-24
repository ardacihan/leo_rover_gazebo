#!/usr/bin/env python3
"""Render the full media set for one two-rover run, for the eyes-on review.

A coverage number cannot see a doubled wall, a seam, or a map that has drifted
outside the building, so every run gets pictures and the pictures decide. This
produces, in the run directory:

    merged_map.png      the shared occupancy grid
    leo1_map.png        per-robot inputs to the merge, at the same scale
    leo2_map.png
    traj_overlay.png    both trajectories + marker ground truth on the merge
    coverage.png        coverage vs sim time
    alignment.png       alignment error and confidence vs sim time

Maps are drawn with **hard three-colour quantisation**, not a smooth grey
ramp: free / unknown / occupied and nothing in between. A ramp renders a
half-confident cell as light grey, which is exactly how a doubled wall or a
speckle field hides -- they look like soft shading instead of the two
separate walls they are.

Usage:
    render_multirobot_media.py <run_dir> [--world office_world] [--title ...]
"""

import argparse
import csv
import math
import os
import re
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt                      # noqa: E402
from matplotlib.colors import ListedColormap, BoundaryNorm   # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

FREE, UNKNOWN, OCC = '#f7f7f5', '#c9ccd1', '#14181d'
MAP_CMAP = ListedColormap([FREE, UNKNOWN, OCC])
MAP_NORM = BoundaryNorm([-0.5, 0.5, 1.5, 2.5], MAP_CMAP.N)
LEO1, LEO2 = '#1f77b4', '#e8710a'
COVERAGE_LINE = re.compile(
    r't=(\d+)s known=([\d.]+)m2 free=([\d.]+)m2 occ=([\d.]+)m2')


# --------------------------------------------------------------------- maps

def read_pgm(path):
    """Binary (P5) PGM -> uint8 array. map_saver_cli writes only P5."""
    with open(path, 'rb') as fh:
        data = fh.read()
    fields, pos = [], 0
    while len(fields) < 4:
        if data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos) + 1
            continue
        if data[pos:pos + 1].isspace():
            pos += 1
            continue
        end = pos
        while not data[end:end + 1].isspace():
            end += 1
        fields.append(data[pos:end])
        pos = end
    pos += 1
    magic, width, height, maxval = fields
    if magic != b'P5':
        raise ValueError(f'{path}: expected P5, got {magic!r}')
    width, height = int(width), int(height)
    arr = np.frombuffer(data[pos:pos + width * height], dtype=np.uint8)
    return arr.reshape(height, width)


def read_map(stem):
    """(classes, extent) for a map_saver pair, or (None, None) if absent.

    classes: 0 free, 1 unknown, 2 occupied, row 0 = ymin.
    """
    pgm, yaml_path = stem + '.pgm', stem + '.yaml'
    if not (os.path.exists(pgm) and os.path.exists(yaml_path)):
        return None, None
    meta = {}
    for line in open(yaml_path):
        if ':' in line and not line.strip().startswith('#'):
            k, v = line.split(':', 1)
            meta[k.strip()] = v.strip()
    res = float(meta.get('resolution', 0.05))
    origin = [float(v) for v in
              meta.get('origin', '[0,0,0]').strip('[]').split(',')]
    occ_th = float(meta.get('occupied_thresh', 0.65))
    free_th = float(meta.get('free_thresh', 0.25))
    negate = int(meta.get('negate', 0))

    img = read_pgm(pgm)
    # map_server convention: p_occupied = (255 - pixel) / 255 unless negated.
    p = img.astype(np.float32) / 255.0
    p = p if negate else 1.0 - p
    cls = np.full(img.shape, 1, dtype=np.uint8)
    cls[p <= free_th] = 0
    cls[p >= occ_th] = 2
    cls = np.flipud(cls)          # PGM row 0 is the TOP row (ymax)
    h, w = cls.shape
    extent = [origin[0], origin[0] + w * res, origin[1], origin[1] + h * res]
    return cls, extent


def draw_map(ax, cls, extent, title):
    if cls is None:
        ax.text(0.5, 0.5, 'map not saved', ha='center', va='center',
                transform=ax.transAxes, color='#c0392b')
        ax.set_title(title)
        return
    ax.imshow(cls, origin='lower', extent=extent, cmap=MAP_CMAP, norm=MAP_NORM,
              interpolation='nearest')
    occ = int((cls == 2).sum())
    known = int((cls != 1).sum())
    ax.set_title(f'{title}\n{known} known cells, {occ} occupied')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.grid(alpha=0.2, lw=0.4)


def save_map(stem, out, title):
    cls, extent = read_map(stem)
    if cls is None:
        return False
    fig, ax = plt.subplots(figsize=(10, 8), dpi=120)
    draw_map(ax, cls, extent, title)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


# -------------------------------------------------------------- trajectories

def read_traj(path):
    out = {}
    if not os.path.exists(path):
        return out
    with open(path) as fh:
        for row in csv.DictReader(fh):
            try:
                out.setdefault(row['robot'], []).append(
                    (float(row['t']), float(row['x']), float(row['y'])))
            except (ValueError, KeyError):
                continue
    return out


def markers_for(world):
    try:
        share = os.path.join(os.path.dirname(os.path.dirname(
            os.path.abspath(__file__))), 'src', 'leo_rover_exploration',
            'config', f'mock_markers_{world}.yaml')
        if not os.path.exists(share):
            return []
        out = []
        for line in open(share):
            m = re.search(r'id:\s*(\d+).*?x:\s*(-?[\d.]+).*?y:\s*(-?[\d.]+)', line)
            if m:
                out.append((int(m.group(1)), float(m.group(2)), float(m.group(3))))
        return out
    except OSError:
        return []


def _apply_tf(pts, tf):
    x0, y0, yaw = tf[0], tf[1], math.radians(tf[2])
    c, s = math.cos(yaw), math.sin(yaw)
    return [(t_, x0 + c * x - s * y, y0 + s * x + c * y) for t_, x, y in pts]


def save_overlay(run, out, world, title, leo2_tf=None):
    """The night's headline image: merged map + both paths + marker truth."""
    cls, extent = read_map(os.path.join(run, 'merged_map'))
    if cls is None:
        cls, extent = read_map(os.path.join(run, 'leo1_map'))
    traj = read_traj(os.path.join(run, 'traj.csv'))
    # When alignment never locks there is no leo1/map -> leo2/base_link, so the
    # shared-frame recorder silently drops that rover. Fall back per rover to
    # its own-frame track, which is always recorded. Both are anchored on the
    # rover's spawn, so they only coincide with the shared frame for leo1 --
    # the overlay says so in the legend rather than pretending otherwise.
    own_frame = set()
    transformed = set()
    for name in ('leo1', 'leo2'):
        alt = read_traj(os.path.join(run, f'traj_{name}.csv')).get(name)
        if not alt:
            continue
        # The shared-frame recorder can only log a rover once the alignment TF
        # exists, so leo2's track there starts mid-run and understates what it
        # actually explored. Its own-frame track covers the whole run; with a
        # recovered transform it can be shown in leo1's frame properly.
        if name == 'leo2' and leo2_tf is not None:
            traj[name] = _apply_tf(alt, leo2_tf)
            transformed.add(name)
        elif not traj.get(name):
            traj[name] = alt
            own_frame.add(name)
    if cls is None and not traj:
        return False

    fig, ax = plt.subplots(figsize=(12, 9), dpi=120)
    if cls is not None:
        ax.imshow(cls, origin='lower', extent=extent, cmap=MAP_CMAP,
                  norm=MAP_NORM, interpolation='nearest')
    for name, colour in (('leo1', LEO1), ('leo2', LEO2)):
        pts = traj.get(name, [])
        if not pts:
            continue
        xs = [p[1] for p in pts]; ys = [p[2] for p in pts]
        if name in own_frame:
            suffix = f' [{name}/map frame - unaligned]'
        elif name in transformed:
            suffix = ' [own track, mapped through the recovered transform]'
        else:
            suffix = ''
        ax.plot(xs, ys, '-', color=colour, lw=1.8, alpha=0.9,
                label=f'{name} ({len(pts)} samples){suffix}')
        ax.plot(xs[0], ys[0], 'o', color=colour, ms=10, mfc='white', mew=2.2)
        ax.plot(xs[-1], ys[-1], 's', color=colour, ms=9)

    # Marker ground truth is in leo1's *world* frame; leo1/map is anchored on
    # leo1's spawn, so this only lines up when leo1 started at the origin.
    # Drawn regardless, as a scale and orientation reference.
    for mid, mx, my in markers_for(world):
        ax.plot(mx, my, '*', color='#8e44ad', ms=13, mec='white', mew=0.7)
        ax.annotate(str(mid), (mx, my), textcoords='offset points',
                    xytext=(6, 5), fontsize=8, color='#8e44ad', weight='bold')

    ax.set_title(title)
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.legend(loc='best', fontsize=9)
    ax.grid(alpha=0.2, lw=0.4)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


# ------------------------------------------------------------------ curves

def _coverage_series(path):
    ts, known = [], []
    if not os.path.exists(path):
        return ts, known
    for line in open(path, errors='ignore'):
        m = COVERAGE_LINE.search(line)
        if m:
            ts.append(int(m.group(1))); known.append(float(m.group(2)))
    return ts, known


def save_coverage(run, out, title):
    """Merged coverage plus each rover's own, on one axis.

    The merged curve is empty whenever alignment never locked -- /shared_map
    has a publisher but never publishes -- and a run that fails to align must
    still show what each rover mapped. Drawing all three also makes the
    coordination story legible: two curves of very different height mean one
    rover did all the work.
    """
    series = []
    for name, fname, colour in (
            ('merged /shared_map', 'coverage.log', '#2c7fb8'),
            ('leo1 /leo1/map', 'coverage_leo1.log', LEO1),
            ('leo2 /leo2/map', 'coverage_leo2.log', LEO2)):
        ts, known = _coverage_series(os.path.join(run, fname))
        if ts:
            series.append((name, ts, known, colour))
    if not series:
        return False

    t0 = min(s[1][0] for s in series)
    fig, ax = plt.subplots(figsize=(9, 5), dpi=120)
    bits = []
    for name, ts, known, colour in series:
        merged = 'merged' in name
        ax.plot([(t - t0) / 60.0 for t in ts], known,
                '-' if merged else '--', color=colour,
                lw=2.4 if merged else 1.8,
                label=f'{name} (final {known[-1]:.1f} m$^2$)')
        bits.append(f'{name.split()[0]} {known[-1]:.1f}')
    ax.set_xlabel('sim time [min]'); ax.set_ylabel('known area [m$^2$]')
    ax.set_title(f'{title}' + chr(10) + f'final: {", ".join(bits)} m$^2$'
                 ' (clipped to the world bounds)')
    ax.legend(loc='upper left', fontsize=9)
    ax.grid(alpha=0.3, lw=0.5)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True



def save_alignment(run, out, title):
    path = os.path.join(run, 'alignment.csv')
    if not os.path.exists(path):
        return False
    rows = list(csv.DictReader(open(path)))
    if not rows:
        return False

    def col(name, conv=float):
        vals = []
        for r in rows:
            v = r.get(name, '')
            vals.append(conv(v) if v not in ('', None) else float('nan'))
        return np.array(vals, dtype=float)

    t = col('t'); t = (t - t[0]) / 60.0
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7), dpi=120, sharex=True)
    ax1.plot(t, col('err_xy_m'), '-', color='#c0392b', lw=2, label='|error| xy [m]')
    ax1.axhline(0.5, ls='--', lw=1, color='#c0392b', alpha=0.6,
                label='0.5 m gate')
    ax1b = ax1.twinx()
    ax1b.plot(t, np.abs(col('err_yaw_deg')), '-', color='#8e44ad', lw=1.6,
              label='|error| yaw [deg]')
    ax1b.axhline(10.0, ls='--', lw=1, color='#8e44ad', alpha=0.6)
    ax1b.set_ylabel('yaw error [deg]', color='#8e44ad')
    ax1.set_ylabel('xy error [m]', color='#c0392b')
    ax1.set_title(title)
    ax1.grid(alpha=0.3, lw=0.5)
    ax1.legend(loc='upper right', fontsize=8)

    ax2.plot(t, col('tag_conf'), '-', color='#2c7fb8', lw=1.8, label='tag confidence')
    ax2.plot(t, col('map_conf'), '-', color='#27ae60', lw=1.8, label='accepted confidence')
    ax2.plot(t, col('locked'), '-', color='#111', lw=1.2, alpha=0.7, label='TF locked')
    ax2b = ax2.twinx()
    ax2b.plot(t, col('n_common'), ':', color='#e8710a', lw=1.8,
              label='common landmarks')
    ax2b.set_ylabel('common landmarks', color='#e8710a')
    ax2.set_xlabel('sim time [min]'); ax2.set_ylabel('confidence')
    ax2.set_ylim(-0.05, 1.05)
    ax2.grid(alpha=0.3, lw=0.5)
    ax2.legend(loc='upper left', fontsize=8)
    fig.tight_layout(); fig.savefig(out); plt.close(fig)
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir')
    ap.add_argument('--world', default='')
    ap.add_argument('--title', default='')
    ap.add_argument('--leo2-tf', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW_DEG'),
                    help="recovered leo2/map pose in leo1/map; maps leo2's "
                         'full own-frame track into the shared frame')
    args = ap.parse_args()
    run = args.run_dir
    tag = args.title or os.path.basename(run.rstrip('/\\'))

    made = []
    for stem, out, title in (
            ('merged_map', 'merged_map.png', f'{tag} - merged /shared_map'),
            ('leo1_map', 'leo1_map.png', f'{tag} - leo1 /leo1/map'),
            ('leo2_map', 'leo2_map.png', f'{tag} - leo2 /leo2/map')):
        if save_map(os.path.join(run, stem), os.path.join(run, out), title):
            made.append(out)
    if save_overlay(run, os.path.join(run, 'traj_overlay.png'), args.world,
                    f'{tag} - merged map, both trajectories, marker truth',
                    leo2_tf=args.leo2_tf):
        made.append('traj_overlay.png')
    if save_coverage(run, os.path.join(run, 'coverage.png'),
                     f'{tag} - merged coverage'):
        made.append('coverage.png')
    if save_alignment(run, os.path.join(run, 'alignment.png'),
                      f'{tag} - leo2->leo1 alignment vs ground truth'):
        made.append('alignment.png')

    print(f'{run}: wrote {len(made)} figures: {", ".join(made) or "none"}')
    missing = {'merged_map.png', 'leo1_map.png', 'leo2_map.png',
               'traj_overlay.png', 'coverage.png', 'alignment.png'} - set(made)
    if missing:
        print(f'  MISSING: {", ".join(sorted(missing))}')


if __name__ == '__main__':
    main()
