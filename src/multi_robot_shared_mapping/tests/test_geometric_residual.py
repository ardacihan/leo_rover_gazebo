"""A confident-but-shifted overlay must not count as a successful lock."""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from multi_robot_shared_mapping.geometric_residual import (
    geometric_lock_ok, residual_polish, residual_stats)
from multi_robot_shared_mapping.grid_registration import local_refine


def _l_walls(shift_xy=(0.0, 0.0)):
    grid = np.full((80, 80), -1, dtype=np.int8)
    grid[10:70, 15] = 100
    grid[55, 15:65] = 100
    info = (shift_xy[0], shift_xy[1], 0.05, 0.0)
    return grid, info


def test_quarter_metre_offset_fails_geometric_lock():
    target, t_info = _l_walls()
    source, s_info = _l_walls()
    shifted = (0.25, -0.04, math.radians(1.0))
    stats = residual_stats(target, t_info, source, s_info, shifted)
    ok, reason = geometric_lock_ok(stats)
    assert stats['median_m'] > 0.15
    assert not ok
    assert "residual" in reason or "hit" in reason


def test_identity_passes_geometric_lock():
    target, info = _l_walls()
    stats = residual_stats(target, info, target, info, (0.0, 0.0, 0.0))
    ok, reason = geometric_lock_ok(stats)
    assert ok
    assert reason == "geometry aligned"
    assert stats['median_m'] < 0.04


def test_residual_polish_closes_visible_offset():
    target, info = _l_walls()
    seed = (0.22, -0.16, math.radians(1.5))
    before = residual_stats(target, info, target, info, seed)
    polished, after = residual_polish(target, info, target, info, seed)
    assert after['median_m'] < before['median_m'] - 0.08
    assert math.hypot(polished[0], polished[1]) < 0.06
    ok, _ = geometric_lock_ok(after)
    assert ok


def test_local_refine_then_lock_ok_on_offset_seed():
    target, info = _l_walls()
    seed = (0.22, -0.16, math.radians(1.5))
    refined, metrics = local_refine(target, info, target, info, seed,
                                    max_points=1000)
    assert metrics['median_m'] < 0.08
    ok, _ = geometric_lock_ok(metrics)
    assert ok
    assert math.hypot(refined[0], refined[1]) < 0.06
