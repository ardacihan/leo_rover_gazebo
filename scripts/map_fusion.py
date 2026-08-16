#!/usr/bin/env python3
"""Offline multi-robot occupancy-map fusion with registration + cleanup.

Replaces the naive fixed-offset overwrite merge with:
  1. registration  — refine each moving map's pose vs the reference map by
     correlative matching on occupied cells (multi-resolution grid search over
     a likelihood field), seeded by the known spawn offsets ("exact" prior)
     or a deliberately coarse prior (±1 m, ±15°) for the relaxed-assumption
     experiment ("coarse").
  2. fusion        — per-cell log-odds sum (probabilistic), or legacy
     max-overwrite ("naive") for before/after comparison.
  3. cleanup       — remove small isolated occupied components, fill small
     unknown speckle inside free space, clip to world bounds.
  4. scoring       — wall IoU / RMSE vs the world SDF ground truth
     (world_ground_truth.py).

Usage (host or container, pure numpy/scipy):
  python map_fusion.py --map maps/leo1_map.yaml --pose 0,0,0 \
                       --map maps/leo2_map.yaml --pose 1.5,0,0 \
                       --world src/leo_rover_gazebo/worlds/office_world.sdf \
                       --register exact --out out_dir
"""

import argparse
import json
import math
import os

import numpy as np
from scipy import ndimage

OCC, FREE, UNK = 100, 0, -1
# Log-odds vote weights. A free cell within 1 cell of the SAME map's own wall
# is that wall's sub-cell edge, not confident open space - it votes weakly so
# it cannot veto the other map's occupied vote (which would punch holes along
# every slightly-misaligned wall), yet alone it still classifies as free.
L_OCC, L_FREE, L_FREE_WEAK = 2.0, 1.4, 0.6
OCC_T, FREE_T = 1.3, -0.55        # classify occ if >= OCC_T, free if <= FREE_T
# resulting truth table: occ alone 2.0=occ; occ+occ 4.0=occ; occ+weak 1.4=occ;
# occ+strong-free 0.6=unknown (true drift conflict -> cleaned later);
# weak alone -0.6=free; strong alone -1.4=free.


# ---------------------------------------------------------------- loading

class GridMap:
    """Trinary occupancy grid + world pose of its own frame in metres."""

    def __init__(self, grid, origin, res, name=''):
        self.grid = grid          # int8 (H, W), row 0 = ymin; -1/0/100
        self.origin = origin      # (x, y) of cell (0,0) corner IN OWN FRAME
        self.res = res
        self.name = name

    @property
    def shape(self):
        return self.grid.shape


def load_map(yaml_path, name=''):
    import re
    meta = {}
    with open(yaml_path) as f:
        for line in f:
            m = re.match(r'(\w+):\s*(.+)', line.strip())
            if m:
                meta[m.group(1)] = m.group(2)
    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')[:2]]
    pgm = os.path.join(os.path.dirname(yaml_path), meta['image'])
    with open(pgm, 'rb') as f:
        magic = f.readline().strip()
        assert magic == b'P5', f'unsupported pgm {magic}'
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = [int(t) for t in line.split()]
        maxval = int(f.readline())
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
    # pgm row 0 = TOP (ymax); flip so row 0 = ymin like OccupancyGrid.
    img = np.flipud(img)
    grid = np.full(img.shape, UNK, dtype=np.int8)
    grid[img <= int(0.35 * maxval)] = OCC     # dark = occupied (negate: 0)
    grid[img >= int(0.9 * maxval)] = FREE     # light = free
    return GridMap(grid, tuple(origin), res, name or os.path.basename(yaml_path))


# ---------------------------------------------------------------- registration

def _occupied_points(m, cap=15000):
    r, c = np.nonzero(m.grid == OCC)
    x = m.origin[0] + (c + 0.5) * m.res
    y = m.origin[1] + (r + 0.5) * m.res
    pts = np.stack([x, y], axis=1)
    if len(pts) > cap:
        pts = pts[np.random.default_rng(0).choice(len(pts), cap, replace=False)]
    return pts


def _likelihood_field(ref, pose, sigma_cells):
    """Blurred occupancy of the reference map, plus a sampler for world pts."""
    occ = (ref.grid == OCC).astype(np.float32)
    field = ndimage.gaussian_filter(occ, sigma_cells)
    peak = field.max()
    if peak > 0:
        field /= peak
    tx, ty, th = pose
    ct, st = math.cos(-th), math.sin(-th)

    def sample(pts):
        # world -> ref frame
        px = pts[:, 0] - tx
        py = pts[:, 1] - ty
        rx = px * ct - py * st
        ry = px * st + py * ct
        ci = ((rx - ref.origin[0]) / ref.res - 0.5)
        ri = ((ry - ref.origin[1]) / ref.res - 0.5)
        ci_n = np.round(ci).astype(int)
        ri_n = np.round(ri).astype(int)
        ok = ((ci_n >= 0) & (ci_n < field.shape[1]) &
              (ri_n >= 0) & (ri_n < field.shape[0]))
        vals = np.zeros(len(pts), dtype=np.float32)
        vals[ok] = field[ri_n[ok], ci_n[ok]]
        return vals

    return sample


def register(moving, ref, seed_pose, ref_pose=(0, 0, 0),
             window=(0.4, 0.4, math.radians(4)), verbose=True):
    """Find pose (tx,ty,theta) of `moving`'s frame in the merged frame that
    best aligns its occupied cells with `ref` (already placed at ref_pose).

    Multi-resolution correlative search seeded at seed_pose, bounded by
    `window` (±dx, ±dy, ±dtheta). Returns (pose, score)."""
    pts0 = _occupied_points(moving)
    if len(pts0) < 50:
        return seed_pose, 0.0

    def transformed(pose):
        tx, ty, th = pose
        ct, st = math.cos(th), math.sin(th)
        x = pts0[:, 0] * ct - pts0[:, 1] * st + tx
        y = pts0[:, 0] * st + pts0[:, 1] * ct + ty
        return np.stack([x, y], axis=1)

    levels = [
        # (sigma_cells, step_xy, step_th, use window)
        (4.0, max(0.10, window[0] / 6), max(math.radians(1.0), window[2] / 8), window),
        (2.0, 0.025, math.radians(0.4), (0.12, 0.12, math.radians(1.5))),
        (1.2, 0.010, math.radians(0.15), (0.035, 0.035, math.radians(0.5))),
    ]
    best = tuple(seed_pose)
    best_s = -1.0
    for sigma, sxy, sth, win in levels:
        sample = _likelihood_field(ref, ref_pose, sigma)
        cx, cy, cth = best
        dxs = np.arange(-win[0], win[0] + 1e-9, sxy)
        dys = np.arange(-win[1], win[1] + 1e-9, sxy)
        dths = np.arange(-win[2], win[2] + 1e-9, sth)
        best_s = -1.0
        for dth in dths:
            th = cth + dth
            ct, st = math.cos(th), math.sin(th)
            rx = pts0[:, 0] * ct - pts0[:, 1] * st
            ry = pts0[:, 0] * st + pts0[:, 1] * ct
            for dx in dxs:
                for dy in dys:
                    pts = np.stack([rx + cx + dx, ry + cy + dy], axis=1)
                    s = float(sample(pts).mean())
                    if s > best_s:
                        best_s, best = s, (cx + dx, cy + dy, th)
        if verbose:
            print(f'  [register {moving.name}] level sigma={sigma}: '
                  f'pose=({best[0]:.3f},{best[1]:.3f},{math.degrees(best[2]):.2f}deg)'
                  f' score={best_s:.4f}')
    return best, best_s


# ---------------------------------------------------------------- fusion

def fuse(maps_poses, mode='logodds', res=None):
    """Fuse [(GridMap, pose), ...] into one GridMap in the merged frame.

    mode 'logodds': per-cell sum of votes;  'naive': max() overwrite (legacy
    compositor behaviour) for the before/after comparison."""
    res = res or maps_poses[0][0].res
    # Merged extent = transformed corners of every map.
    xs, ys = [], []
    for m, (tx, ty, th) in maps_poses:
        H, W = m.shape
        corners = np.array([
            [m.origin[0], m.origin[1]],
            [m.origin[0] + W * m.res, m.origin[1]],
            [m.origin[0], m.origin[1] + H * m.res],
            [m.origin[0] + W * m.res, m.origin[1] + H * m.res]])
        ct, st = math.cos(th), math.sin(th)
        x = corners[:, 0] * ct - corners[:, 1] * st + tx
        y = corners[:, 0] * st + corners[:, 1] * ct + ty
        xs += list(x)
        ys += list(y)
    ox, oy = min(xs), min(ys)
    W = int(math.ceil((max(xs) - ox) / res))
    H = int(math.ceil((max(ys) - oy) / res))
    # Merged cell centres, inverse-warped into each map (no holes).
    mx = ox + (np.arange(W) + 0.5) * res
    my = oy + (np.arange(H) + 0.5) * res
    MX, MY = np.meshgrid(mx, my)
    logodds = np.zeros((H, W), dtype=np.float32)
    naive = np.full((H, W), UNK, dtype=np.int16)
    for m, (tx, ty, th) in maps_poses:
        ct, st = math.cos(-th), math.sin(-th)
        px = MX - tx
        py = MY - ty
        rx = px * ct - py * st
        ry = px * st + py * ct
        ci = np.round((rx - m.origin[0]) / m.res - 0.5).astype(int)
        ri = np.round((ry - m.origin[1]) / m.res - 0.5).astype(int)
        ok = ((ci >= 0) & (ci < m.shape[1]) & (ri >= 0) & (ri < m.shape[0]))
        occ_d = ndimage.binary_dilation(m.grid == OCC,
                                        structure=np.ones((3, 3)))
        # per-cell vote of this map: +L_OCC / -L_FREE / -L_FREE_WEAK / 0
        vote = np.where(
            m.grid == OCC, np.float32(L_OCC),
            np.where((m.grid == FREE) & occ_d, np.float32(-L_FREE_WEAK),
                     np.where(m.grid == FREE, np.float32(-L_FREE),
                              np.float32(0.0))))
        vgrid = np.zeros((H, W), dtype=np.float32)
        vgrid[ok] = vote[ri[ok], ci[ok]]
        logodds += vgrid
        # naive mode keeps the raw values (legacy overwrite behaviour)
        raw = np.full((H, W), UNK, dtype=np.int16)
        raw[ok] = m.grid[ri[ok], ci[ok]]
        np.maximum(naive, raw, out=naive)
    if mode == 'naive':
        grid = naive.astype(np.int8)
    else:
        grid = np.full((H, W), UNK, dtype=np.int8)
        grid[logodds >= OCC_T] = OCC
        grid[logodds <= FREE_T] = FREE
        # occ + strong-free disagreement lands between the thresholds and
        # stays unknown; the speckle filter / free-fill in clean() resolves it.
    return GridMap(grid, (ox, oy), res, 'merged'), logodds


# ---------------------------------------------------------------- cleanup

def clean(m, bounds=None, min_occ_cells=6, max_unk_hole=40, margin=0.3):
    """Despeckle + clip. bounds = (xmin, xmax, ymin, ymax) world shell."""
    g = m.grid.copy()
    # 1. clip to world bounds (+margin): outside -> unknown.
    if bounds is not None:
        xmin, xmax, ymin, ymax = bounds
        H, W = g.shape
        cx = m.origin[0] + (np.arange(W) + 0.5) * m.res
        cy = m.origin[1] + (np.arange(H) + 0.5) * m.res
        out_c = (cx < xmin - margin) | (cx > xmax + margin)
        out_r = (cy < ymin - margin) | (cy > ymax + margin)
        g[out_r, :] = UNK
        g[:, out_c] = UNK
    # 2. remove small isolated occupied components.
    occ = g == OCC
    lab, n = ndimage.label(occ, structure=np.ones((3, 3)))
    if n:
        sizes = ndimage.sum_labels(np.ones_like(lab), lab, index=np.arange(1, n + 1))
        small = np.isin(lab, np.nonzero(sizes < min_occ_cells)[0] + 1)
        # speckle sitting in free space becomes free; elsewhere unknown
        free_near = ndimage.binary_dilation(g == FREE, iterations=2)
        g[small & free_near] = FREE
        g[small & ~free_near] = UNK
    # 3. fill small unknown holes fully surrounded by free.
    unk = g == UNK
    lab, n = ndimage.label(unk, structure=np.ones((3, 3)))
    if n:
        sizes = ndimage.sum_labels(np.ones_like(lab), lab, index=np.arange(1, n + 1))
        for idx in (np.nonzero(sizes <= max_unk_hole)[0] + 1):
            comp = lab == idx
            border = ndimage.binary_dilation(comp) & ~comp
            vals = g[border]
            if len(vals) and (vals == FREE).all():
                g[comp] = FREE
    # 4. crop to the known region (tidy extents).
    known = g != UNK
    if known.any():
        rows = np.nonzero(known.any(axis=1))[0]
        cols = np.nonzero(known.any(axis=0))[0]
        r0, r1 = max(rows[0] - 4, 0), min(rows[-1] + 5, g.shape[0])
        c0, c1 = max(cols[0] - 4, 0), min(cols[-1] + 5, g.shape[1])
        g = g[r0:r1, c0:c1]
        origin = (m.origin[0] + c0 * m.res, m.origin[1] + r0 * m.res)
    else:
        origin = m.origin
    return GridMap(g, origin, m.res, m.name + '_clean')


# ---------------------------------------------------------------- output

def save_map(m, stem):
    img = np.full(m.shape, 205, dtype=np.uint8)
    img[m.grid == FREE] = 254
    img[m.grid == OCC] = 0
    img = np.flipud(img)
    with open(stem + '.pgm', 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (m.shape[1], m.shape[0]))
        f.write(img.tobytes())
    with open(stem + '.yaml', 'w') as f:
        f.write(f'image: {os.path.basename(stem)}.pgm\nmode: trinary\n'
                f'resolution: {m.res}\n'
                f'origin: [{m.origin[0]:.3f}, {m.origin[1]:.3f}, 0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')


def render_png(m, path, title=None, gt=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    H, W = m.shape
    rgb = np.full((H, W, 3), 0.82, dtype=np.float32)   # unknown grey
    rgb[m.grid == FREE] = 1.0
    rgb[m.grid == OCC] = 0.05
    fig, ax = plt.subplots(figsize=(max(6, W / 80), max(4.5, H / 80)))
    ext = [m.origin[0], m.origin[0] + W * m.res,
           m.origin[1], m.origin[1] + H * m.res]
    ax.imshow(rgb, origin='lower', extent=ext, interpolation='nearest')
    if gt is not None:
        from world_ground_truth import _resample_mask_to
        gtm = _resample_mask_to(gt, m.origin, m.res, m.shape)
        yy, xx = np.nonzero(gtm)
        ax.scatter(m.origin[0] + (xx + 0.5) * m.res,
                   m.origin[1] + (yy + 0.5) * m.res,
                   s=0.15, c='tab:red', alpha=0.25, linewidths=0)
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    if title:
        ax.set_title(title)
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ---------------------------------------------------------------- CLI

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--map', action='append', required=True, dest='maps')
    ap.add_argument('--pose', action='append', required=True,
                    help='tx,ty,theta_deg of this map frame (spawn offset)')
    ap.add_argument('--world', help='SDF for ground truth + bound clipping')
    ap.add_argument('--register', choices=['none', 'exact', 'coarse'],
                    default='exact')
    ap.add_argument('--fuse', choices=['logodds', 'naive'], default='logodds')
    ap.add_argument('--no-clean', action='store_true')
    ap.add_argument('--z', type=float, default=0.5, help='lidar plane height')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    maps = [load_map(p, f'm{i}') for i, p in enumerate(args.maps)]
    poses = []
    for s in args.pose:
        v = [float(t) for t in s.split(',')]
        poses.append((v[0], v[1], math.radians(v[2] if len(v) > 2 else 0.0)))

    gt = bounds = None
    if args.world:
        from world_ground_truth import rasterize_world, score_map
        gt = rasterize_world(args.world, maps[0].res, args.z)
        bounds = gt.bounds

    result = {'seed_poses': [list(p) for p in poses], 'register': args.register,
              'fuse': args.fuse}
    if args.register != 'none' and len(maps) > 1:
        win = ((0.4, 0.4, math.radians(4)) if args.register == 'exact'
               else (1.2, 1.2, math.radians(16)))
        refined = [poses[0]]
        for m, seed in zip(maps[1:], poses[1:]):
            pose, score = register(m, maps[0], seed, poses[0], win)
            refined.append(pose)
            print(f'{m.name}: seed=({seed[0]:.2f},{seed[1]:.2f},'
                  f'{math.degrees(seed[2]):.1f}) -> refined=({pose[0]:.3f},'
                  f'{pose[1]:.3f},{math.degrees(pose[2]):.2f}deg) score={score:.4f}')
        result['refined_poses'] = [list(p) for p in refined]
        result['correction'] = [
            [r[0] - s[0], r[1] - s[1], math.degrees(r[2] - s[2])]
            for r, s in zip(refined, poses)]
        poses = refined

    merged, _ = fuse(list(zip(maps, poses)), mode=args.fuse)
    if not args.no_clean:
        merged = clean(merged, bounds)
    save_map(merged, os.path.join(args.out, 'merged'))
    render_png(merged, os.path.join(args.out, 'merged.png'),
               title=os.path.basename(args.out), gt=gt)
    if gt is not None:
        from world_ground_truth import score_map
        result['metrics'] = score_map(merged.grid == OCC, merged.origin,
                                      merged.res, gt)
        print(json.dumps(result['metrics'], indent=2))
    with open(os.path.join(args.out, 'result.json'), 'w') as f:
        json.dump(result, f, indent=2)
    print('wrote', args.out)


if __name__ == '__main__':
    main()
