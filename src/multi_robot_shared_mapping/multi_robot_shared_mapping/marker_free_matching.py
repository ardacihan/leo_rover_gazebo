"""Marker-free occupancy-grid merge: global search + margin abstention.

The naive baseline (`_baseline_matcher.py`) fails 0/10 for two reasons this
module is built around:

  * it returns the best-scoring rotation with no ambiguity test, so in the
    rectilinear depot/office worlds a 90/180-degree flip that genuinely scores
    well wins silently;
  * its translation search is capped at +/-6 m while the husarion pairs are
    ~13.8 m apart, so the right answer is not even in its search space.

Stage 1 -- global FFT search. Per yaw hypothesis (2-degree sweep over the full
circle) rasterize map2 onto a coarse grid and score every translation at once
by cross-correlation. The score is computed on the overlap region only:

    S(t) = (occ-hits - 0.6*occ2-on-free1 - 0.4*free2-on-occ1) / occ2-in-known1

Hypotheses that barely overlap map1's known space are invalid -- scoring cells
the other map has never seen rewards hypotheses that hide their disagreement
in unknown space. Top translation peaks per yaw survive (with non-max
suppression), then all candidates are deduped into distinct *modes*: two
candidates belong to the same mode iff within 12 deg AND 2 m.

Stage 2 -- polish and decide. The coarse score is too blunt to separate truth
from a symmetric flip (measured 0.38 vs 0.35 on depot pairs), so the decision
uses the full-resolution score instead: each of the top modes is polished
coarse-to-fine (same scoring as align_registries_offline.refine_transform,
which reaches ~99% wall overlap when correctly seeded) and the *polished*
wall-overlap fractions are compared. Commit only when the best mode clears an
absolute floor AND beats every other mode by a relative margin. In a
symmetric room the flipped mode polishes nearly as high, the margin
collapses, and the matcher abstains instead of merging confidently wrong.

Benchmark:  python3 scripts/merge_benchmark.py --method marker_free_matcher:match
Tuning knobs are module constants; MFM_DEBUG=1 prints the mode table.
`candidate_modes()` + `polish()` are exposed for the tuning script.
"""
import math
import os

import numpy as np

# --- tuning constants -------------------------------------------------------
COARSE = 0.25            # m/cell for the global FFT stage
DILATE = 2               # coarse cells of wall dilation (+/-0.5 m tolerance)
YAW_STEP_DEG = 2.0
W_OCC_ON_FREE = 0.6      # penalty: map2 wall lands on map1 free space
W_FREE_ON_OCC = 0.4      # penalty: map2 free space swallows a map1 wall
MIN_OVERLAP_FRAC = 0.25  # occ2 cells that must land in map1's known space.
                         # Tried 0.10 to rescue partial-overlap husarion:
                         # it floods the candidate list with tiny-overlap
                         # hypotheses scoring near-perfect on a fragment and
                         # the true modes get crowded out -> 10/10 abstain.
MIN_OVERLAP_CELLS = 40
PEAKS_PER_YAW = 4        # translation peaks kept per yaw (NMS below)
NMS_M = 2.0              # metres masked around an accepted peak
MODE_YAW_DEG = 12.0      # closer than this in yaw AND...
MODE_TRANS_M = 2.0       # ...translation = same mode
N_MODES = 30             # modes polished at full resolution: the coarse score
                         # is only a candidate generator (measured: it ranked
                         # the true depot mode 3rd-9th and once below 20th),
                         # the polished hit is the discriminative signal
                         # (true modes 0.87-0.998 vs best wrong mode <= 0.34)
POLISH_HIT_MIN = 0.80    # polished wall-overlap floor -> abstain. 0.55 let a
                         # drift-poisoned office pair commit 1.9 m wrong at
                         # hit 0.764; every measured true mode scored >= 0.87
MARGIN_MIN = 0.30        # (best - runner) / best on polished hits -> abstain
                         # (measured: true-mode margins >= 0.6, flips ~ 0)

DEBUG = bool(os.environ.get('MFM_DEBUG'))


def _binary_dilate(m, it):
    out = m.copy()
    for _ in range(it):
        p = out.copy()
        p[1:, :] |= out[:-1, :]; p[:-1, :] |= out[1:, :]
        p[:, 1:] |= out[:, :-1]; p[:, :-1] |= out[:, 1:]
        out = p
    return out


def _world_points(grid, info, val_test):
    ys, xs = np.nonzero(val_test(grid))
    ox, oy, res = info
    return np.stack([ox + (xs + 0.5) * res, oy + (ys + 0.5) * res], 1)


def _raster(points, lo, cell, shape):
    idx = np.floor((points - lo) / cell).astype(int)
    ok = ((idx[:, 0] >= 0) & (idx[:, 0] < shape[1]) &
          (idx[:, 1] >= 0) & (idx[:, 1] < shape[0]))
    a = np.zeros(shape, np.float32)
    a[idx[ok, 1], idx[ok, 0]] = 1.0
    return a


def candidate_modes(g1, i1, g2, i2):
    """Global search. Returns modes sorted by coarse score:
    [(coarse_score, dx, dy, yaw_rad), ...]  (world-frame map2->map1)."""
    occ1p = _world_points(g1, i1, lambda g: g >= 50)
    free1p = _world_points(g1, i1, lambda g: g == 0)
    occ2p = _world_points(g2, i2, lambda g: g >= 50)
    free2p = _world_points(g2, i2, lambda g: g == 0)
    if len(occ1p) < 50 or len(occ2p) < 50:
        return []
    if len(free2p) > 60000:
        free2p = free2p[:: len(free2p) // 60000]

    all1 = np.vstack([occ1p, free1p]) if len(free1p) else occ1p
    lo1 = all1.min(0) - 1.0
    hi1 = all1.max(0) + 1.0
    n1 = np.ceil((hi1 - lo1) / COARSE).astype(int) + 1
    shape1 = (int(n1[1]), int(n1[0]))

    occ1 = _raster(occ1p, lo1, COARSE, shape1)
    free1 = _raster(free1p, lo1, COARSE, shape1)
    occ1d = _binary_dilate(occ1 > 0, DILATE).astype(np.float32)
    known1 = np.maximum(occ1d, free1)

    c2 = occ2p.mean(0)
    q_occ = occ2p - c2
    q_free = free2p - c2
    r2 = float(np.abs(q_occ).max() + 1.0)
    n2 = int(np.ceil(2 * r2 / COARSE)) + 1

    H = shape1[0] + n2 + 2
    W = shape1[1] + n2 + 2
    F_num = np.fft.rfft2(occ1d - W_OCC_ON_FREE * free1, s=(H, W))
    F_pen = np.fft.rfft2(-W_FREE_ON_OCC * occ1, s=(H, W))
    F_kno = np.fft.rfft2(known1, s=(H, W))

    nms_cells = max(1, int(round(NMS_M / COARSE)))
    lo2 = np.array([-r2, -r2])
    shape2 = (n2, n2)
    cands = []
    for deg in np.arange(0.0, 360.0, YAW_STEP_DEG):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        R = np.array([[c, -s], [s, c]])
        a_occ = _raster(q_occ @ R.T, lo2, COARSE, shape2)
        a_free = _raster(q_free @ R.T, lo2, COARSE, shape2)
        n_occ2 = a_occ.sum()
        if n_occ2 < 30:
            continue
        Fo = np.conj(np.fft.rfft2(a_occ, s=(H, W)))
        Ff = np.conj(np.fft.rfft2(a_free, s=(H, W)))
        num = (np.fft.irfft2(F_num * Fo, s=(H, W)) +
               np.fft.irfft2(F_pen * Ff, s=(H, W)))
        kno = np.fft.irfft2(F_kno * Fo, s=(H, W))
        valid = kno >= max(MIN_OVERLAP_FRAC * n_occ2, MIN_OVERLAP_CELLS)
        if not valid.any():
            continue
        sc = np.where(valid, num / np.maximum(kno, 1.0), -1e9)
        for _ in range(PEAKS_PER_YAW):
            flat = int(np.argmax(sc))
            dyc, dxc = np.unravel_index(flat, sc.shape)
            best = float(sc[dyc, dxc])
            if best <= -1e8:
                break
            sdx = dxc if dxc <= W // 2 else dxc - W
            sdy = dyc if dyc <= H // 2 else dyc - H
            t = lo1 + np.array([sdx, sdy]) * COARSE - lo2
            tx = t[0] - (R[0, 0] * c2[0] + R[0, 1] * c2[1])
            ty = t[1] - (R[1, 0] * c2[0] + R[1, 1] * c2[1])
            cands.append((best, tx, ty, th))
            r0, r1 = max(0, dyc - nms_cells), min(H, dyc + nms_cells + 1)
            c0, c1 = max(0, dxc - nms_cells), min(W, dxc + nms_cells + 1)
            sc[r0:r1, c0:c1] = -1e9

    cands.sort(key=lambda r: -r[0])
    modes = []
    for scv, tx, ty, th in cands:
        dup = False
        for _, mx, my, mth in modes:
            dyaw = abs((math.degrees(th - mth) + 180) % 360 - 180)
            if dyaw < MODE_YAW_DEG and math.hypot(tx - mx, ty - my) < MODE_TRANS_M:
                dup = True
                break
        if not dup:
            modes.append((scv, tx, ty, th))
        if len(modes) >= N_MODES:
            break
    return modes


def polish(g1, i1, g2, i2, dx, dy, yaw, n_pts=4000):
    """Full-res coarse-to-fine local polish.

    The hit fraction is normalized by ALL of map2's wall points, not by the
    in-overlap subset. Overlap-only normalization was tried and it is the
    trap the goal doc warns about: a hypothesis that hides most of map2 in
    map1's unknown space gets judged on a tiny matching fragment and scores
    0.8+, flooding the mode list (measured: 10/10 abstain, true modes
    crowded out). The cost is that genuinely partial-overlap pairs
    (husarion) cap below the commit floor and abstain -- acceptable.

    Returns ((dx, dy, yaw), hit_fraction) or (None, 0.0)."""
    occ1 = g1 >= 50
    d = _binary_dilate(occ1, 1)
    ys, xs = np.nonzero(g2 >= 50)
    if len(xs) < 50:
        return None, 0.0
    if len(xs) > n_pts:
        pick = np.random.RandomState(0).choice(len(xs), n_pts, replace=False)
        ys, xs = ys[pick], xs[pick]
    px = i2[0] + (xs + 0.5) * i2[2]
    py = i2[1] + (ys + 0.5) * i2[2]
    ox, oy, res = i1
    h, w = g1.shape

    def score(tx, ty, th):
        c, s = math.cos(th), math.sin(th)
        qx = tx + c * px - s * py
        qy = ty + s * px + c * py
        ci = ((qx - ox) / res).astype(int)
        ri = ((qy - oy) / res).astype(int)
        ok = (ci >= 0) & (ci < w) & (ri >= 0) & (ri < h)
        if not ok.any():
            return 0.0
        return float(d[ri[ok], ci[ok]].sum()) / len(px)

    best = (dx, dy, yaw)
    best_s = score(*best)
    # stage-1 window must exceed the coarse stage's worst-case error: 0.25 m
    # cells plus real inter-map SLAM drift (measured ~1.9 m on the office
    # pair), so a slightly-off coarse peak can slide onto the true optimum.
    stages = ((0.15, math.radians(1.0), 1.05, math.radians(4.0)),
              (0.05, math.radians(0.25), 0.15, math.radians(1.2)),
              (0.02, math.radians(0.10), 0.06, math.radians(0.4)))
    for st, sy, wt, wy in stages:
        cx, cy, cth = best
        for th in np.arange(cth - wy, cth + wy + 1e-9, sy):
            for tx in np.arange(cx - wt, cx + wt + 1e-9, st):
                for ty in np.arange(cy - wt, cy + wt + 1e-9, st):
                    sc = score(tx, ty, th)
                    if sc > best_s:
                        best_s, best = sc, (tx, ty, th)
    return best, best_s


def match(g1, i1, g2, i2):
    est, _ = match_diag(g1, i1, g2, i2)
    return est


def match_diag(g1, i1, g2, i2):
    """Like match(), but also returns a diagnostics dict:
    {reason, best_hit, margin, n_modes, runner_hit} (fields None when n/a)."""
    diag = {'reason': '', 'best_hit': None, 'margin': None,
            'n_modes': 0, 'runner_hit': None}
    modes = candidate_modes(g1, i1, g2, i2)
    diag['n_modes'] = len(modes)
    if not modes:
        diag['reason'] = 'no candidate modes (too little overlap or structure)'
        return None, diag
    polished = []
    for scv, tx, ty, th in modes:
        est, hit = polish(g1, i1, g2, i2, tx, ty, th)
        if est is not None:
            polished.append((hit, est, scv))
    if not polished:
        diag['reason'] = 'no mode survived polish'
        return None, diag
    polished.sort(key=lambda r: -r[0])
    best_hit, best_est, _ = polished[0]
    diag['best_hit'] = best_hit
    if DEBUG:
        for hit, est, scv in polished[:6]:
            print(f'    mode yaw={math.degrees(est[2]):7.1f} '
                  f't=({est[0]:7.2f},{est[1]:7.2f}) coarse={scv:.3f} hit={hit:.3f}')
    if best_hit < POLISH_HIT_MIN:
        diag['reason'] = f'best polished hit {best_hit:.3f} < {POLISH_HIT_MIN}'
        if DEBUG:
            print(f'    abstain: {diag["reason"]}')
        return None, diag
    # runner-up = best polished mode genuinely distinct from the winner
    # (several coarse peaks can slide into the same optimum during polish)
    runner_hit = None
    for hit, est, _ in polished[1:]:
        dyaw = abs((math.degrees(est[2] - best_est[2]) + 180) % 360 - 180)
        dt = math.hypot(est[0] - best_est[0], est[1] - best_est[1])
        if dyaw > MODE_YAW_DEG or dt > MODE_TRANS_M:
            runner_hit = hit
            break
    diag['runner_hit'] = runner_hit
    if runner_hit is not None:
        margin = (best_hit - runner_hit) / max(best_hit, 1e-6)
        diag['margin'] = margin
        if DEBUG:
            print(f'    margin={margin:.3f} (runner hit={runner_hit:.3f})')
        if margin < MARGIN_MIN:
            diag['reason'] = (f'ambiguous: margin {margin:.3f} < {MARGIN_MIN} '
                              f'(runner-up hit {runner_hit:.3f})')
            return None, diag
    diag['reason'] = 'committed'
    return best_est, diag
