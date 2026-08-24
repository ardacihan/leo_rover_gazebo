#!/usr/bin/env python3
"""Offline two-rover map merge: registries -> transform -> fused map + PNG.

This is the deployment path for the lab. Each rover maps on its own (its own
ROS domain, no cross-rover DDS, no namespacing) and produces two artifacts:
its saved map (map_saver on its local /map) and its ArUco landmark registry
(aruco_registry_<name>.json). This script does the rest on the laptop, in
about a second, and can be re-run as often as needed:

  1. match the marker ids both rovers confirmed,
  2. plain Kabsch fit over the common landmarks (no residual gating -- the
     live aligner's gates rejected a 0.61 m fit on depot 2026-08-24 and kept
     publishing a stale 19-degree-wrong one; offline, a human looks at the
     picture instead),
  3. fuse the two saved maps under that transform (via fuse_maps_offline.py),
  4. render a PNG for the eye test.

Also prints a leave-one-out table so a single bad landmark can be spotted and
excluded with --exclude.

Usage (a run directory laid out like the sim reports):
  align_registries_offline.py <run_dir> [--truth X Y YAW_DEG] [--exclude ID...]
Or explicit paths:
  align_registries_offline.py --reg1 A.json --reg2 B.json \
      --map1 leo1_map --map2 leo2_map --out merged [--truth X Y YAW_DEG]

Convention: robot 1 is the reference; the printed transform is the pose of
robot 2's map frame expressed in robot 1's map frame -- the same --tf that
fuse_maps_offline.py takes.
"""
import argparse
import glob
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, 'src', 'multi_robot_shared_mapping'))

from multi_robot_shared_mapping.tag_map_alignment import (   # noqa: E402
    estimate_2d_transform)


def load_registry(path, use_best=False):
    with open(path) as fh:
        data = json.load(fh)
    px, py = ('best_x', 'best_y') if use_best else ('x', 'y')
    return {m['id']: (m[px], m[py], m.get('hits', 0))
            for m in data.get('markers', [])}


def fit(src_pts, tgt_pts):
    """Ungated Kabsch: report residuals, never reject."""
    return estimate_2d_transform(
        src_pts, tgt_pts, min_tags=2,
        max_mean_error=1e9, max_point_error=1e9,
        use_orientation=False)


def residuals(est, src_pts, tgt_pts):
    c, s = math.cos(est.yaw), math.sin(est.yaw)
    out = []
    for (sx, sy), (tx, ty) in zip(src_pts, tgt_pts):
        mx = est.dx + c * sx - s * sy
        my = est.dy + s * sx + c * sy
        out.append(math.hypot(mx - tx, my - ty))
    return out


def ang_diff_deg(a_rad, b_rad):
    d = math.degrees(a_rad - b_rad)
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def refine_transform(map1_stem, map2_stem, est, trans_win=0.5, yaw_win_deg=4.0):
    """Local grid-correlation polish of a tag-seeded transform.

    This is deliberately NOT global map matching. Global scan/grid matching on
    rectilinear, self-similar rooms has confidently produced 180-, 90- and
    65-degree flips on this project (depot twice, husarion once) -- the SE(2)
    objective is multimodal and a wrong optimum can score as well as the right
    one. Seeded by the tag estimate and confined to a +/-0.5 m / +/-4 deg
    trust region, the same wall-pattern signal becomes safe: within that
    window the true alignment is the only good optimum, and the search can
    only tighten the residual tag error, never flip the map.

    Score = fraction of map2's occupied cells landing on (1-cell-dilated)
    occupied cells of map1. Coarse-to-fine, ~2 s.
    """
    import numpy as np
    from render_multirobot_media import read_map
    a, ea = read_map(map1_stem)
    b, eb = read_map(map2_stem)
    if a is None or b is None:
        return None, None
    OCC = 2
    resa = (ea[1] - ea[0]) / a.shape[1]
    resb = (eb[1] - eb[0]) / b.shape[1]
    occ = (a == OCC)
    d = occ.copy()
    d[1:, :] |= occ[:-1, :]; d[:-1, :] |= occ[1:, :]
    d[:, 1:] |= occ[:, :-1]; d[:, :-1] |= occ[:, 1:]
    d[1:, 1:] |= occ[:-1, :-1]; d[:-1, :-1] |= occ[1:, 1:]
    d[1:, :-1] |= occ[:-1, 1:]; d[:-1, 1:] |= occ[1:, :-1]
    ys, xs = np.nonzero(b == OCC)
    if len(xs) < 50:
        return None, None
    if len(xs) > 6000:
        pick = np.random.RandomState(0).choice(len(xs), 6000, replace=False)
        ys, xs = ys[pick], xs[pick]
    px = eb[0] + (xs + 0.5) * resb
    py = eb[2] + (ys + 0.5) * resb

    def score(dx, dy, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        qx = dx + c * px - s * py
        qy = dy + s * px + c * py
        ci = ((qx - ea[0]) / resa).astype(int)
        ri = ((qy - ea[2]) / resa).astype(int)
        ok = (ci >= 0) & (ci < a.shape[1]) & (ri >= 0) & (ri < a.shape[0])
        if not ok.any():
            return 0.0
        return float(d[ri[ok], ci[ok]].sum()) / len(px)

    best = (est.dx, est.dy, est.yaw)
    base_s = best_s = score(*best)
    stages = ((0.10, math.radians(1.0), trans_win, math.radians(yaw_win_deg)),
              (0.025, math.radians(0.25), 0.12, math.radians(1.2)))
    for step_t, step_y, wt, wy in stages:
        cx, cy, cyaw = best
        for yw in np.arange(cyaw - wy, cyaw + wy + 1e-9, step_y):
            for tx in np.arange(cx - wt, cx + wt + 1e-9, step_t):
                for ty in np.arange(cy - wt, cy + wt + 1e-9, step_t):
                    s = score(tx, ty, yw)
                    if s > best_s:
                        best_s, best = s, (tx, ty, yw)
    import types
    ref = types.SimpleNamespace(dx=best[0], dy=best[1], yaw=best[2])
    return ref, (base_s, best_s)


def combined_landmarks(r1, r2, est):
    """All landmarks in robot 1's frame: robot 2's discoveries transformed in.

    For a tag both saw, keep robot 1's position (its frame is the reference).
    Returns {id: (x, y, who)}.
    """
    c, s = math.cos(est.yaw), math.sin(est.yaw)
    out = {}
    for i, (x, y, _) in r2.items():
        out[i] = (est.dx + c * x - s * y, est.dy + s * x + c * y, 'rov2')
    for i, (x, y, _) in r1.items():
        who = 'both' if i in r2 else 'rov1'
        out[i] = (x, y, who)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('run_dir', nargs='?',
                    help='directory with aruco_registry_*.json + saved maps')
    ap.add_argument('--reg1'); ap.add_argument('--reg2')
    ap.add_argument('--map1'); ap.add_argument('--map2')
    ap.add_argument('--out', default=None)
    ap.add_argument('--truth', nargs=3, type=float, default=None,
                    metavar=('X', 'Y', 'YAW_DEG'))
    ap.add_argument('--exclude', nargs='*', type=int, default=[],
                    help='marker ids to leave out of the fit')
    ap.add_argument('--use-best', action='store_true',
                    help='use each landmark closest-range observation '
                         'instead of the filtered position')
    ap.add_argument('--no-fuse', action='store_true',
                    help='only compute and print the transform')
    ap.add_argument('--refine', action='store_true',
                    help='polish the tag transform by local grid correlation '
                         '(+/-0.5 m, +/-4 deg trust region around it)')
    args = ap.parse_args()

    reg1, reg2, map1, map2 = args.reg1, args.reg2, args.map1, args.map2
    out = args.out
    if args.run_dir:
        d = args.run_dir
        regs = sorted(glob.glob(os.path.join(d, 'aruco_registry_*.json')))
        if len(regs) < 2 and not (reg1 and reg2):
            print(f'need two aruco_registry_*.json in {d}, found {len(regs)}')
            return 1
        reg1 = reg1 or regs[0]
        reg2 = reg2 or regs[1]

        def find_map(reg_path):
            name = os.path.basename(reg_path)
            name = name.replace('aruco_registry_', '').replace('.json', '')
            for cand in (os.path.join(d, f'{name}_map'),
                         os.path.join(d, f'{name}')):
                if os.path.exists(cand + '.pgm'):
                    return cand
            return None
        map1 = map1 or find_map(reg1)
        map2 = map2 or find_map(reg2)
        out = out or os.path.join(d, 'offline_merged')

    r1 = load_registry(reg1, args.use_best)
    r2 = load_registry(reg2, args.use_best)
    common = sorted(set(r1) & set(r2) - set(args.exclude))
    print(f'reference: {os.path.basename(reg1)}   aligned: '
          f'{os.path.basename(reg2)}')
    print(f'landmarks: ref={sorted(r1)} other={sorted(r2)} '
          f'common(used)={common}'
          + (f' excluded={args.exclude}' if args.exclude else ''))
    if len(common) < 2:
        print('FAIL: need >= 2 common landmarks for a transform. '
              'Drive both rovers past the same markers.')
        return 1

    src = [(r2[i][0], r2[i][1]) for i in common]   # robot 2 frame
    tgt = [(r1[i][0], r1[i][1]) for i in common]   # robot 1 frame
    est = fit(src, tgt)
    res = residuals(est, src, tgt)

    print(f'\ntransform (robot2 map -> robot1 map): '
          f'x={est.dx:.3f}  y={est.dy:.3f}  yaw={math.degrees(est.yaw):.2f} deg')
    print('per-tag residuals:')
    for i, r in zip(common, res):
        print(f'  tag {i:2d}: {r:.3f} m   (hits {r1[i][2]}/{r2[i][2]})')
    print(f'  mean {sum(res) / len(res):.3f} m  max {max(res):.3f} m')

    if len(common) >= 3:
        print('leave-one-out (a big drop in mean residual = that tag is bad):')
        for skip in common:
            keep = [i for i in common if i != skip]
            e = fit([(r2[i][0], r2[i][1]) for i in keep],
                    [(r1[i][0], r1[i][1]) for i in keep])
            rr = residuals(e, [(r2[i][0], r2[i][1]) for i in keep],
                           [(r1[i][0], r1[i][1]) for i in keep])
            line = (f'  without {skip:2d}: mean {sum(rr) / len(rr):.3f} m  '
                    f'yaw {math.degrees(e.yaw):7.2f} deg')
            if args.truth:
                terr = math.hypot(e.dx - args.truth[0], e.dy - args.truth[1])
                yerr = abs(ang_diff_deg(e.yaw, math.radians(args.truth[2])))
                line += f'   vs truth {terr:.2f} m / {yerr:.1f} deg'
            print(line)

    def report_truth(e, label):
        terr = math.hypot(e.dx - args.truth[0], e.dy - args.truth[1])
        yerr = abs(ang_diff_deg(e.yaw, math.radians(args.truth[2])))
        verdict = 'PASS' if terr <= 0.5 and yerr <= 10 else 'FAIL'
        print(f'{label} vs ground truth ({args.truth[0]}, {args.truth[1]}, '
              f'{args.truth[2]} deg): {terr:.3f} m / {yerr:.2f} deg  '
              f'[{verdict} @ 0.5 m / 10 deg]')

    if args.truth:
        print()
        report_truth(est, 'tags   ')

    have_maps = (map1 and map2 and os.path.exists(map1 + '.pgm')
                 and os.path.exists(map2 + '.pgm'))
    if args.refine and have_maps:
        ref, sc = refine_transform(map1, map2, est)
        if ref is not None:
            print(f'refined (grid correlation around tag estimate): '
                  f'x={ref.dx:.3f}  y={ref.dy:.3f}  '
                  f'yaw={math.degrees(ref.yaw):.2f} deg   '
                  f'(overlap {sc[0]:.3f} -> {sc[1]:.3f})')
            if args.truth:
                report_truth(ref, 'refined')
            est = ref
    elif args.refine:
        print('(--refine needs both saved maps)')

    lm = combined_landmarks(r1, r2, est)
    print('\ncombined landmark map (robot 1 frame):')
    for i in sorted(lm):
        x, y, who = lm[i]
        print(f'  tag {i:2d}: ({x:7.2f}, {y:7.2f})   seen by {who}')

    if args.no_fuse:
        return 0
    if not have_maps:
        print('\n(no saved map pair found -- skipping fusion; '
              'pass --map1/--map2)')
        return 0

    cmd = [sys.executable, os.path.join(HERE, 'fuse_maps_offline.py'),
           map1, map2, out,
           '--tf', f'{est.dx}', f'{est.dy}', f'{math.degrees(est.yaw)}']
    print()
    subprocess.run(cmd, check=True)

    # PNG for the eye test.
    from render_multirobot_media import read_map
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np
    grid, ext = read_map(out)
    img = np.full(grid.shape, 0.82)
    img[grid == 0] = 1.0    # free
    img[grid == 2] = 0.0    # occupied
    fig, ax = plt.subplots(figsize=(10, 10 * grid.shape[0] / grid.shape[1]))
    ax.imshow(img, cmap='gray', vmin=0, vmax=1, origin='lower',
              extent=[ext[0], ext[1], ext[2], ext[3]])
    colors = {'both': 'tab:green', 'rov1': 'tab:blue', 'rov2': 'tab:orange'}
    for i, (x, y, who) in sorted(lm.items()):
        ax.plot(x, y, 'o', ms=9, mfc='none', mew=2, color=colors[who])
        ax.annotate(str(i), (x, y), textcoords='offset points',
                    xytext=(7, 7), fontsize=9, color=colors[who])
    for who, col in colors.items():
        ax.plot([], [], 'o', mfc='none', mew=2, color=col, label=who)
    ax.legend(loc='lower right', fontsize=8)
    ax.set_title(f'offline merge: tags {common}, '
                 f'tf ({est.dx:.2f}, {est.dy:.2f}, '
                 f'{math.degrees(est.yaw):.1f} deg)')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(out + '.png', dpi=130)
    print(f'{out}.png written')
    return 0


if __name__ == '__main__':
    sys.exit(main())
