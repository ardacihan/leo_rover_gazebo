#!/usr/bin/env python3
"""Score a saved SLAM map against a Gazebo world and flag drift artefacts.

Reads a map_saver_cli ``.yaml``/``.pgm`` pair, rasterises the world SDF at the
lidar plane, and reports both agreement metrics and the specific failure
signatures that show up when SLAM drifts:

  wall IoU / precision / recall  -- geometric agreement at 1 and 2 cell tolerance
  rmse_m                         -- mean distance from a mapped wall to a true one
  phantom_frac                   -- fraction of mapped wall cells further than
                                    ``--phantom-cells`` from any true wall. Ghost
                                    copies of a corridor (the classic skid-steer
                                    yaw-drift signature) land here.
  known_area_m2 vs free_area_m2  -- a map whose free area exceeds the world's
                                    reachable area has been stretched by drift
  bbox                           -- mapped extent against the world extent; a
                                    map larger than the world is drift, not
                                    coverage

With ``--pose-csv`` (from pose_error_recorder.py) it also reports absolute
trajectory error for the SLAM and odometry-only estimates.

Usage:
    python3 eval_map.py <map.yaml> <world.sdf> [--z 0.5] [--png out.png] \
        [--pose-csv pose_error.csv] [--json out.json]
"""

import argparse
import csv
import json
import math
import os
import re

import numpy as np
from scipy import ndimage

from world_ground_truth import rasterize_world, score_map, _resample_mask_to

# The grey nav2's map_saver reserves for "not yet observed".
UNKNOWN_PGM_VALUE = 205


def read_pgm(path):
    """Minimal binary (P5) PGM reader; map_saver_cli always writes P5."""
    with open(path, 'rb') as fh:
        data = fh.read()
    if not data.startswith(b'P5'):
        raise ValueError(f'{path} is not a binary PGM')
    # Header: P5, width, height, maxval -- comments (#...) may appear between.
    fields, pos = [], 2
    while len(fields) < 3:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b'#':
            while pos < len(data) and data[pos:pos + 1] not in (b'\n', b'\r'):
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        fields.append(int(data[start:pos]))
    pos += 1  # single whitespace after maxval
    width, height, maxval = fields
    dtype = np.uint8 if maxval < 256 else '>u2'
    pixels = np.frombuffer(data, dtype=dtype, count=width * height, offset=pos)
    return pixels.reshape(height, width)


def read_map(yaml_path):
    """Return (occupied_mask, free_mask, unknown_mask, origin, res).

    Masks use grid convention (row 0 = ymin), matching world_ground_truth.
    """
    with open(yaml_path) as fh:
        text = fh.read()

    def field(name, default=None):
        m = re.search(rf'^{name}\s*:\s*(.+)$', text, re.M)
        return m.group(1).strip() if m else default

    image = field('image')
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), image)
    res = float(field('resolution'))
    origin = [float(v) for v in
              field('origin').strip('[] ').replace(',', ' ').split()][:2]
    negate = int(field('negate', '0'))
    occ_th = float(field('occupied_thresh', '0.65'))
    free_th = float(field('free_thresh', '0.25'))

    pixels = read_pgm(image)
    # PGM row 0 is the TOP of the image = ymax. Flip to grid convention.
    pixels = pixels[::-1, :]
    # map_saver_cli writes exactly three values in trinary mode: 0 occupied,
    # 254 free, 205 unknown. 205 must be matched explicitly -- putting it
    # through the thresholds gives p = 50/255 = 0.196, which is below the
    # default free_thresh of 0.25 and would silently count all unexplored
    # space as mapped free space.
    unknown = pixels == UNKNOWN_PGM_VALUE
    p = (pixels / 255.0 if negate else (255.0 - pixels) / 255.0).astype(np.float64)
    occupied = (p > occ_th) & ~unknown
    free = (p < free_th) & ~unknown
    unknown = ~occupied & ~free
    return occupied, free, unknown, (origin[0], origin[1]), res


def best_rigid_alignment(occ, origin, res, gt_on_map, tol_cells=2,
                         max_shift_m=0.7, shift_step_m=0.05,
                         max_rot_deg=3.0, rot_step_deg=0.5):
    """Find the (dx, dy, dtheta) that best registers the map onto the truth.

    A SLAM map is only defined up to the frame it anchored on: slam_toolbox
    fixes `map` at the pose of the first scan, so two otherwise identical runs
    can sit 10-15 cm apart. At a 10 cm scoring tolerance that global offset
    dominates the score and would rank configurations by anchoring luck rather
    than by map quality. So report the score after registration as well.

    Scored on the fraction of mapped wall cells landing within tolerance of a
    true wall, which needs only one distance transform of the ground truth.
    """
    if not occ.any() or not gt_on_map.any():
        return (0.0, 0.0, 0.0)
    d_to_gt = ndimage.distance_transform_edt(~gt_on_map)
    rows, cols = np.nonzero(occ)
    # Rotate about the centroid of the mapped walls.
    cy, cx = rows.mean(), cols.mean()
    ry, rx = rows - cy, cols - cx
    H, W = occ.shape
    shift_cells = int(round(max_shift_m / res))
    step_cells = max(int(round(shift_step_m / res)), 1)
    offsets = range(-shift_cells, shift_cells + 1, step_cells)
    angles = np.arange(-max_rot_deg, max_rot_deg + 1e-9, rot_step_deg)

    best = (-1.0, 0, 0, 0.0)
    for deg in angles:
        t = math.radians(deg)
        ct, st = math.cos(t), math.sin(t)
        base_r = cy + ry * ct - rx * st
        base_c = cx + ry * st + rx * ct
        for dr in offsets:
            rr = np.round(base_r + dr).astype(int)
            valid_r = (rr >= 0) & (rr < H)
            for dc in offsets:
                cc = np.round(base_c + dc).astype(int)
                ok = valid_r & (cc >= 0) & (cc < W)
                if not ok.any():
                    continue
                hits = (d_to_gt[rr[ok], cc[ok]] <= tol_cells).sum()
                score = hits / len(rr)
                if score > best[0]:
                    best = (score, dr, dc, deg)
    _, dr, dc, deg = best
    return (dc * res, dr * res, deg)


def apply_rigid(occ, dx_m, dy_m, deg, res):
    """Resample an occupancy mask under a rigid transform (nearest cell)."""
    if dx_m == 0.0 and dy_m == 0.0 and deg == 0.0:
        return occ
    rows, cols = np.nonzero(occ)
    if not len(rows):
        return occ
    cy, cx = rows.mean(), cols.mean()
    t = math.radians(deg)
    ct, st = math.cos(t), math.sin(t)
    ry, rx = rows - cy, cols - cx
    rr = np.round(cy + ry * ct - rx * st + dy_m / res).astype(int)
    cc = np.round(cx + ry * st + rx * ct + dx_m / res).astype(int)
    H, W = occ.shape
    ok = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
    out = np.zeros_like(occ)
    out[rr[ok], cc[ok]] = True
    return out


def evaluate(map_yaml, world_sdf, z_plane=0.5, phantom_cells=3):
    occ, free, unknown, origin, res = read_map(map_yaml)
    gt = rasterize_world(world_sdf, res=res, z_plane=z_plane)

    result = score_map(occ, origin, res, gt)
    gt_on_map = _resample_mask_to(gt, origin, res, occ.shape)

    # Registered score: the same metrics after removing the global anchor
    # offset, so runs are compared on shape rather than on where slam_toolbox
    # happened to pin the map frame.
    dx, dy, deg = best_rigid_alignment(occ, origin, res, gt_on_map)
    occ_aligned = apply_rigid(occ, dx, dy, deg, res)
    aligned = score_map(occ_aligned, origin, res, gt)
    result['align_dx_m'] = dx
    result['align_dy_m'] = dy
    result['align_deg'] = deg
    result['align_shift_m'] = float(math.hypot(dx, dy))
    for key in ('iou@1', 'iou@2', 'precision@2', 'recall@2', 'rmse_m'):
        if key in aligned:
            result[f'{key}_aligned'] = aligned[key]

    # Phantom walls: mapped occupancy far from any true wall (after
    # registration, so a global offset is not counted as phantom structure).
    if occ.any() and gt_on_map.any():
        d_to_gt = ndimage.distance_transform_edt(~gt_on_map)
        result['phantom_frac_raw'] = float(
            (d_to_gt[occ] > phantom_cells).sum()) / float(occ.sum())
        result['phantom_frac'] = float(
            (d_to_gt[occ_aligned] > phantom_cells).sum()) / float(
                max(occ_aligned.sum(), 1))
    else:
        result['phantom_frac'] = float('nan')

    cell_area = res * res
    result['occ_area_m2'] = float(occ.sum()) * cell_area
    result['free_area_m2'] = float(free.sum()) * cell_area
    result['known_area_m2'] = float((occ | free).sum()) * cell_area
    result['resolution'] = res

    # Reachable (interior free) ground truth: everything not occupied that the
    # outer shell encloses. Flood from the border of the GT grid inwards; what
    # is NOT reached and NOT occupied is the interior.
    outside = np.zeros_like(gt.occ)
    outside[0, :] = outside[-1, :] = True
    outside[:, 0] = outside[:, -1] = True
    outside = ndimage.binary_propagation(outside, mask=~gt.occ)
    interior = ~gt.occ & ~outside
    result['gt_free_area_m2'] = float(interior.sum()) * gt.res * gt.res
    result['free_area_ratio'] = (result['free_area_m2']
                                 / max(result['gt_free_area_m2'], 1e-9))

    # Extents. A mapped bbox wider than the world means the map was stretched.
    rows, cols = np.nonzero(occ | free)
    if len(rows):
        result['map_bbox'] = [
            float(origin[0] + cols.min() * res),
            float(origin[0] + (cols.max() + 1) * res),
            float(origin[1] + rows.min() * res),
            float(origin[1] + (rows.max() + 1) * res)]
        result['map_extent_m'] = [
            result['map_bbox'][1] - result['map_bbox'][0],
            result['map_bbox'][3] - result['map_bbox'][2]]
    result['world_bbox'] = [float(v) for v in gt.bounds]
    result['world_extent_m'] = [gt.bounds[1] - gt.bounds[0],
                                gt.bounds[3] - gt.bounds[2]]
    if 'map_extent_m' in result:
        result['extent_excess_m'] = [
            result['map_extent_m'][0] - result['world_extent_m'][0],
            result['map_extent_m'][1] - result['world_extent_m'][1]]
    return result, occ, free, gt, origin, res


def trajectory_error(csv_path):
    """Absolute trajectory error of the SLAM and odometry-only estimates."""
    slam_err, odom_err, final = [], [], {}
    with open(csv_path) as fh:
        for row in csv.DictReader(fh):
            try:
                gx, gy = float(row['gt_x']), float(row['gt_y'])
            except (ValueError, KeyError):
                continue
            if row.get('slam_x'):
                slam_err.append(math.hypot(float(row['slam_x']) - gx,
                                           float(row['slam_y']) - gy))
                final['slam_final_err_m'] = slam_err[-1]
            if row.get('odom_x'):
                odom_err.append(math.hypot(float(row['odom_x']) - gx,
                                           float(row['odom_y']) - gy))
                final['odom_final_err_m'] = odom_err[-1]
    out = dict(final)
    for name, series in (('slam', slam_err), ('odom', odom_err)):
        if series:
            arr = np.asarray(series)
            out[f'{name}_ate_rmse_m'] = float(np.sqrt(np.mean(arr ** 2)))
            out[f'{name}_ate_max_m'] = float(arr.max())
    return out


def render(png, occ, free, gt, origin, res, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    H, W = occ.shape
    extent = [origin[0], origin[0] + W * res, origin[1], origin[1] + H * res]
    rgb = np.ones((H, W, 3))
    rgb[free] = (0.85, 0.85, 0.85)
    rgb[occ] = (0.0, 0.0, 0.0)

    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    axes[0].imshow(rgb, origin='lower', extent=extent)
    axes[0].set_title(f'{title}\nSLAM map')
    gt_extent = [gt.origin[0], gt.origin[0] + gt.occ.shape[1] * gt.res,
                 gt.origin[1], gt.origin[1] + gt.occ.shape[0] * gt.res]
    axes[1].imshow(gt.occ, origin='lower', cmap='gray_r', extent=gt_extent,
                   alpha=0.35)
    ys, xs = np.nonzero(occ)
    axes[1].scatter(origin[0] + (xs + 0.5) * res, origin[1] + (ys + 0.5) * res,
                    s=0.4, c='crimson', linewidths=0)
    axes[1].set_title('map walls (red) over world truth (grey)')
    for ax in axes:
        ax.set_aspect('equal')
        ax.grid(alpha=0.2)
    plt.tight_layout()
    plt.savefig(png, dpi=130)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('map_yaml')
    ap.add_argument('world_sdf')
    ap.add_argument('--z', type=float, default=0.5)
    ap.add_argument('--phantom-cells', type=int, default=3)
    ap.add_argument('--png')
    ap.add_argument('--pose-csv')
    ap.add_argument('--json')
    ap.add_argument('--label', default='')
    args = ap.parse_args()

    result, occ, free, gt, origin, res = evaluate(
        args.map_yaml, args.world_sdf, args.z, args.phantom_cells)
    if args.pose_csv and os.path.exists(args.pose_csv):
        result.update(trajectory_error(args.pose_csv))
    result['label'] = args.label or os.path.basename(
        os.path.dirname(os.path.abspath(args.map_yaml)))

    order = ['label', 'iou@2_aligned', 'precision@2_aligned',
             'recall@2_aligned', 'rmse_m_aligned', 'phantom_frac',
             'align_shift_m', 'align_deg', 'iou@2', 'phantom_frac_raw',
             'rmse_m', 'slam_ate_rmse_m', 'slam_final_err_m',
             'odom_ate_rmse_m', 'odom_final_err_m', 'free_area_m2',
             'gt_free_area_m2', 'free_area_ratio', 'known_area_m2',
             'map_extent_m', 'world_extent_m', 'extent_excess_m']
    for key in order:
        if key in result:
            value = result[key]
            if isinstance(value, float):
                print(f'{key:22s} {value:.4f}')
            elif isinstance(value, list):
                print(f'{key:22s} ' + ', '.join(f'{v:.2f}' for v in value))
            else:
                print(f'{key:22s} {value}')

    if args.png:
        render(args.png, occ, free, gt, origin, res, result['label'])
        print('wrote', args.png)
    if args.json:
        with open(args.json, 'w') as fh:
            json.dump(result, fh, indent=2)
        print('wrote', args.json)


if __name__ == '__main__':
    main()
