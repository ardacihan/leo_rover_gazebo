import math

import numpy as np

from multi_robot_shared_mapping.grid_registration import (
    cell_centers, invert_transform, local_refine, registration_hits)


def test_cell_centers_respect_rotated_grid_origin():
    grid = np.array([[100, -1], [-1, -1]], dtype=np.int8)
    points = cell_centers(grid, (2.0, 3.0, 1.0, math.pi / 2),
                          lambda g: g >= 50)
    assert np.allclose(points, [[1.5, 3.5]])


def test_transform_inverse_round_trip():
    t = (2.0, -1.0, 0.4)
    inv = invert_transform(t)
    x, y = 3.0, 2.0
    c, s = math.cos(t[2]), math.sin(t[2])
    q = (t[0] + c * x - s * y, t[1] + s * x + c * y)
    ci, si = math.cos(inv[2]), math.sin(inv[2])
    p = (inv[0] + ci * q[0] - si * q[1],
         inv[1] + si * q[0] + ci * q[1])
    assert np.allclose(p, (x, y))


def test_local_refinement_removes_visible_grid_offset():
    target = np.full((80, 80), -1, dtype=np.int8)
    target[10:70, 15] = 100
    target[55, 15:65] = 100
    source = target.copy()
    info = (0.0, 0.0, 0.05, 0.0)
    seed = (0.22, -0.16, math.radians(1.5))
    before = registration_hits(target, info, source, info, seed)['wall_hit']
    refined, metrics = local_refine(target, info, source, info, seed,
                                    max_points=1000)
    assert metrics['wall_hit'] > before + 0.5
    assert math.hypot(refined[0], refined[1]) < 0.06
    assert abs(math.degrees(refined[2])) < 0.3
