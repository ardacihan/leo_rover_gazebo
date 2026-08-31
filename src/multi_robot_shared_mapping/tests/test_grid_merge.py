import math

import numpy as np

from multi_robot_shared_mapping.grid_merge import merge_grids


def test_merge_resamples_different_resolutions_without_holes():
    coarse = np.zeros((2, 2), dtype=np.int16)
    merged, info = merge_grids([(coarse, (0.0, 0.0, 1.0, 0.0), None),
                                (np.zeros((1, 1), dtype=np.int16),
                                 (5.0, 0.0, 0.5, 0.0), None)])
    assert info[2] == 0.5
    assert np.all(merged[:, :4] == 0)


def test_rotated_origin_and_source_transform_land_on_same_wall():
    target = np.full((6, 6), -1, dtype=np.int16)
    target[2, 3] = 100
    # The source center is (1.5, 2.5) after its rotated origin, then the
    # source->target transform places it on target cell center (3.5, 2.5).
    source = np.array([[100]], dtype=np.int16)
    merged, _ = merge_grids([
        (target, (0.0, 0.0, 1.0, 0.0), None),
        (source, (2.0, 2.0, 1.0, math.pi / 2), (2.0, 0.0, 0.0)),
    ])
    assert int((merged >= 50).sum()) == 1
