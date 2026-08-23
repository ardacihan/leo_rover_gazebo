#!/usr/bin/env python3
"""Synthetic validation of map_fusion.register().

Splits a real single-robot SLAM map into two overlapping halves, re-expresses
the second half in a frame displaced by a KNOWN ground-truth transform, then
checks that registration recovers that transform:

  * "exact" prior: seed = truth + small error (<=0.3 m, <=3 deg) - the known-
    spawn scenario of the live system (seed error = per-robot SLAM drift).
  * "coarse" prior: seed = truth + large error (<=1 m, <=15 deg) - the relaxed
    assumption of OVERNIGHT_GOAL Priority 3.

Usage: python validate_registration.py <map.yaml> <out.json> [n_trials]
"""

import json
import math
import sys

import numpy as np
from scipy import ndimage

from map_fusion import GridMap, load_map, register, UNK


def resample_to_frame(src, pose, origin, shape):
    """Express `src` (own frame == world here) in a frame whose pose in the
    world is `pose`: cell centres of the new grid are mapped world = R p + t,
    sampled from src. Returns GridMap whose own-frame origin is `origin`."""
    tx, ty, th = pose
    H, W = shape
    cx = origin[0] + (np.arange(W) + 0.5) * src.res
    cy = origin[1] + (np.arange(H) + 0.5) * src.res
    PX, PY = np.meshgrid(cx, cy)
    wx = PX * math.cos(th) - PY * math.sin(th) + tx
    wy = PX * math.sin(th) + PY * math.cos(th) + ty
    ci = np.round((wx - src.origin[0]) / src.res - 0.5).astype(int)
    ri = np.round((wy - src.origin[1]) / src.res - 0.5).astype(int)
    ok = ((ci >= 0) & (ci < src.shape[1]) & (ri >= 0) & (ri < src.shape[0]))
    g = np.full(shape, UNK, dtype=np.int8)
    g[ok] = src.grid[ri[ok], ci[ok]]
    return GridMap(g, origin, src.res, 'half2_synth')


def main():
    map_yaml = sys.argv[1]
    out_json = sys.argv[2]
    n_trials = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    full = load_map(map_yaml, 'full')
    H, W = full.shape
    # Reference = left 62% of the map (own frame == world frame).
    a = GridMap(full.grid[:, :int(W * 0.62)].copy(), full.origin, full.res, 'A')
    # Second half = right 62%, with 24% overlap in the middle.
    c0 = int(W * 0.38)
    b_world = GridMap(full.grid[:, c0:].copy(),
                      (full.origin[0] + c0 * full.res, full.origin[1]),
                      full.res, 'B')

    rng = np.random.default_rng(42)
    results = {'exact': [], 'coarse': []}
    for mode, terr, aerr, win in [
            ('exact', 0.3, 3.0, (0.4, 0.4, math.radians(4))),
            ('coarse', 1.0, 15.0, (1.2, 1.2, math.radians(16)))]:
        for k in range(n_trials):
            # True pose of B's frame in the world.
            true = (float(rng.uniform(-0.2, 0.2)),
                    float(rng.uniform(-0.2, 0.2)),
                    float(rng.uniform(-math.radians(3), math.radians(3))))
            b = resample_to_frame(b_world, true, b_world.origin, b_world.shape)
            # But wait: resample_to_frame maps new-frame coords through pose,
            # i.e. the generated grid holds src content seen from a frame at
            # `true`; registering b against a should recover exactly `true`.
            seed = (true[0] + float(rng.uniform(-terr, terr)),
                    true[1] + float(rng.uniform(-terr, terr)),
                    true[2] + math.radians(float(rng.uniform(-aerr, aerr))))
            rec, score = register(b, a, seed, (0, 0, 0), win, verbose=False)
            err_t = math.hypot(rec[0] - true[0], rec[1] - true[1])
            err_a = abs(math.degrees(rec[2] - true[2]))
            seed_t = math.hypot(seed[0] - true[0], seed[1] - true[1])
            seed_a = abs(math.degrees(seed[2] - true[2]))
            results[mode].append({
                'true': [true[0], true[1], math.degrees(true[2])],
                'seed_err_m': seed_t, 'seed_err_deg': seed_a,
                'recovered_err_m': err_t, 'recovered_err_deg': err_a,
                'score': score})
            print(f'{mode} #{k}: seed off by {seed_t:.3f} m/{seed_a:.1f} deg '
                  f'-> recovered off by {err_t:.3f} m/{err_a:.2f} deg')
    for mode in list(results):
        errs = [r['recovered_err_m'] for r in results[mode]]
        angs = [r['recovered_err_deg'] for r in results[mode]]
        results[mode + '_summary'] = {
            'mean_err_m': float(np.mean(errs)), 'max_err_m': float(np.max(errs)),
            'mean_err_deg': float(np.mean(angs)), 'max_err_deg': float(np.max(angs))}
        print(mode, 'summary:', results[mode + '_summary'])
    with open(out_json, 'w') as f:
        json.dump(results, f, indent=2)


if __name__ == '__main__':
    main()
