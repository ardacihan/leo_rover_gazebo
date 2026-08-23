#!/usr/bin/env python3
"""Rasterize a Gazebo world SDF into a ground-truth occupancy mask + score maps.

The lidar scans a horizontal plane ~0.5 m above the floor, so the ground truth
is the footprint of every *collision* shape whose z-span crosses that plane
(visual-only props like the mock ArUco tiles sit below it and never appear in
the SLAM map).

Usage:
    python world_ground_truth.py <world.sdf> [--res 0.05] [--z 0.5] [--png out.png]

Also importable:  gt = rasterize_world(sdf_path, res)   -> GroundTruth
                  metrics = score_map(occ_mask, origin, res, gt)
"""

import argparse
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage


@dataclass
class GroundTruth:
    occ: np.ndarray          # bool (H, W), row 0 = ymin (grid convention)
    origin: tuple            # (x, y) of cell (0, 0) corner, world metres
    res: float
    bounds: tuple            # (xmin, xmax, ymin, ymax) of the outer shell
    shapes: list = field(default_factory=list)   # for debugging


def _parse_pose(text):
    v = [float(t) for t in (text or '0 0 0 0 0 0').split()]
    while len(v) < 6:
        v.append(0.0)
    return v  # x y z roll pitch yaw


def parse_world_shapes(sdf_path, z_plane=0.5):
    """Return list of shapes crossing z_plane: ('box', cx, cy, sx, sy, yaw)
    or ('cyl', cx, cy, radius)."""
    root = ET.parse(sdf_path).getroot()
    shapes = []
    for model in root.iter('model'):
        mpose = _parse_pose(model.findtext('pose'))
        for link in model.findall('link'):
            lpose = _parse_pose(link.findtext('pose'))
            for coll in link.findall('collision'):
                geom = coll.find('geometry')
                if geom is None:
                    continue
                cpose = _parse_pose(coll.findtext('pose'))
                # Compose poses (yaw-only worlds; ignore roll/pitch which are
                # zero for every static prop in ours).
                yaw = mpose[5] + lpose[5] + cpose[5]
                cx = mpose[0] + lpose[0] + cpose[0]
                cy = mpose[1] + lpose[1] + cpose[1]
                cz = mpose[2] + lpose[2] + cpose[2]
                box = geom.find('box')
                cyl = geom.find('cylinder')
                if box is not None:
                    sx, sy, sz = [float(t) for t in box.findtext('size').split()]
                    if abs(cz - z_plane) <= sz / 2:
                        shapes.append(('box', cx, cy, sx, sy, yaw))
                elif cyl is not None:
                    r = float(cyl.findtext('radius'))
                    h = float(cyl.findtext('length'))
                    if abs(cz - z_plane) <= h / 2:
                        shapes.append(('cyl', cx, cy, r))
    return shapes


def rasterize_world(sdf_path, res=0.05, z_plane=0.5, margin=0.5):
    shapes = parse_world_shapes(sdf_path, z_plane)
    if not shapes:
        raise ValueError(f'no collision shapes cross z={z_plane} in {sdf_path}')
    # Outer shell bounds = extent of all shapes.
    xs, ys = [], []
    for s in shapes:
        if s[0] == 'box':
            _, cx, cy, sx, sy, yaw = s
            c, si = math.cos(yaw), math.sin(yaw)
            ex = abs(sx / 2 * c) + abs(sy / 2 * si)
            ey = abs(sx / 2 * si) + abs(sy / 2 * c)
            xs += [cx - ex, cx + ex]
            ys += [cy - ey, cy + ey]
        else:
            _, cx, cy, r = s
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    ox, oy = xmin - margin, ymin - margin
    W = int(round((xmax - xmin + 2 * margin) / res))
    H = int(round((ymax - ymin + 2 * margin) / res))
    occ = np.zeros((H, W), dtype=bool)
    # Cell-center coordinate grids.
    cxs = ox + (np.arange(W) + 0.5) * res
    cys = oy + (np.arange(H) + 0.5) * res
    X, Y = np.meshgrid(cxs, cys)
    for s in shapes:
        if s[0] == 'box':
            _, cx, cy, sx, sy, yaw = s
            dx, dy = X - cx, Y - cy
            u = dx * math.cos(-yaw) - dy * math.sin(-yaw)
            v = dx * math.sin(-yaw) + dy * math.cos(-yaw)
            occ |= (np.abs(u) <= sx / 2) & (np.abs(v) <= sy / 2)
        else:
            _, cx, cy, r = s
            occ |= (X - cx) ** 2 + (Y - cy) ** 2 <= r ** 2
    return GroundTruth(occ=occ, origin=(ox, oy), res=res,
                       bounds=(xmin, xmax, ymin, ymax), shapes=shapes)


def _resample_mask_to(gt, origin, res, shape):
    """Ground-truth occupancy resampled onto a map's grid (nearest cell)."""
    H, W = shape
    cxs = origin[0] + (np.arange(W) + 0.5) * res
    cys = origin[1] + (np.arange(H) + 0.5) * res
    ci = np.round((cxs - gt.origin[0]) / gt.res - 0.5).astype(int)
    ri = np.round((cys - gt.origin[1]) / gt.res - 0.5).astype(int)
    valid_c = (ci >= 0) & (ci < gt.occ.shape[1])
    valid_r = (ri >= 0) & (ri < gt.occ.shape[0])
    out = np.zeros((H, W), dtype=bool)
    rr = ri[valid_r][:, None]
    cc = ci[valid_c][None, :]
    out[np.ix_(valid_r, valid_c)] = gt.occ[rr, cc]
    return out


def score_map(occ_mask, origin, res, gt, tol_cells=(1, 2)):
    """Score a map's occupied mask against ground truth.

    Returns dict with wall IoU at each tolerance (cells), precision/recall,
    and RMSE (m) of occupied cells to the nearest true wall.
    """
    gt_on_map = _resample_mask_to(gt, origin, res, occ_mask.shape)
    out = {'n_occ': int(occ_mask.sum()), 'n_gt': int(gt_on_map.sum())}
    if not occ_mask.any() or not gt_on_map.any():
        return out
    # Distance (cells) from every cell to nearest GT wall / nearest map wall.
    d_to_gt = ndimage.distance_transform_edt(~gt_on_map)
    d_to_map = ndimage.distance_transform_edt(~occ_mask)
    d_occ = d_to_gt[occ_mask]
    out['rmse_m'] = float(np.sqrt(np.mean(np.minimum(d_occ, 20) ** 2))) * res
    out['mean_dist_m'] = float(np.mean(np.minimum(d_occ, 20))) * res
    for t in tol_cells:
        tp = int((d_occ <= t).sum())
        fp = out['n_occ'] - tp
        fn = int((d_to_map[gt_on_map] > t).sum())
        out[f'iou@{t}'] = tp / max(tp + fp + fn, 1)
        out[f'precision@{t}'] = tp / max(out['n_occ'], 1)
        out[f'recall@{t}'] = (out['n_gt'] - fn) / max(out['n_gt'], 1)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world')
    ap.add_argument('--res', type=float, default=0.05)
    ap.add_argument('--z', type=float, default=0.5)
    ap.add_argument('--png')
    args = ap.parse_args()
    gt = rasterize_world(args.world, args.res, args.z)
    print(f'shapes: {len(gt.shapes)}  grid: {gt.occ.shape[1]}x{gt.occ.shape[0]}'
          f'  bounds: {gt.bounds}  occupied cells: {int(gt.occ.sum())}')
    if args.png:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        plt.figure(figsize=(8, 6))
        plt.imshow(gt.occ, origin='lower', cmap='gray_r',
                   extent=[gt.origin[0], gt.origin[0] + gt.occ.shape[1] * gt.res,
                           gt.origin[1], gt.origin[1] + gt.occ.shape[0] * gt.res])
        plt.title(f'{args.world} ground truth (z={args.z} m)')
        plt.tight_layout()
        plt.savefig(args.png, dpi=150)
        print('wrote', args.png)


if __name__ == '__main__':
    main()
