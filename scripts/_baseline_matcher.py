"""Naive correlative matcher -- the baseline the overnight work must beat.

Deliberately simple and deliberately over-confident: it returns its best
scoring rotation with no ambiguity test, which is exactly how the existing
grid aligner produces 90 and 180 degree flips in rectilinear rooms.
"""
import math
import numpy as np


def match(g1, i1, g2, i2, angles=None, res_scale=4):
    a1 = (g1 >= 50); a2 = (g2 >= 50)
    if a1.sum() < 50 or a2.sum() < 50:
        return None
    p1 = np.argwhere(a1)[:, ::-1].astype(float) * i1[2] + np.array(i1[:2])
    p2 = np.argwhere(a2)[:, ::-1].astype(float) * i2[2] + np.array(i2[:2])
    if len(p1) > 3000: p1 = p1[::len(p1)//3000]
    if len(p2) > 3000: p2 = p2[::len(p2)//3000]
    cell = i1[2] * res_scale
    lo = np.minimum(p1.min(0), p2.min(0)) - 5.0
    hi = np.maximum(p1.max(0), p2.max(0)) + 5.0
    shape = np.ceil((hi - lo) / cell).astype(int) + 1
    occ = np.zeros(shape[::-1], dtype=bool)
    idx = ((p1 - lo) / cell).astype(int)
    occ[idx[:, 1], idx[:, 0]] = True
    best = None
    for deg in (angles if angles is not None else range(0, 360, 5)):
        th = math.radians(deg)
        c, s = math.cos(th), math.sin(th)
        r2 = np.stack([c * p2[:, 0] - s * p2[:, 1], s * p2[:, 0] + c * p2[:, 1]], 1)
        for dx in np.arange(-6, 6.01, cell * 2):
            for dy in np.arange(-6, 6.01, cell * 2):
                q = ((r2 + (dx, dy) - lo) / cell).astype(int)
                ok = ((q[:, 0] >= 0) & (q[:, 0] < shape[0]) &
                      (q[:, 1] >= 0) & (q[:, 1] < shape[1]))
                if not ok.any(): continue
                sc = occ[q[ok, 1], q[ok, 0]].sum()
                if best is None or sc > best[0]:
                    best = (sc, dx, dy, th)
    if best is None:
        return None
    return (best[1], best[2], best[3])
