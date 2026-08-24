"""The shared-map frontier mask: peer-covered space is covered space.

Pure-logic tests for FrontierExplorer._peer_known_mask -- invoked unbound on a
stub, no rclpy node needed. The contract under test:

* unknown cells of the own map that the merged map knows -> masked;
* everything else (shared-unknown, out of shared bounds) -> untouched;
* stale shared map or missing alignment offset -> None (own-map behaviour).
"""
import math
import types

import numpy as np

from leo_rover_exploration.frontier_explorer import FrontierExplorer, UNKNOWN

RES = 0.1


def grid_msg(grid, ox=0.0, oy=0.0):
    h, w = grid.shape
    info = types.SimpleNamespace(
        height=h, width=w, resolution=RES,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=ox, y=oy)))
    return types.SimpleNamespace(info=info, data=grid.flatten().tolist())


class _Logger:
    def info(self, *a, **k):
        pass


class Stub:
    shared_map_max_age = 20.0

    def __init__(self, shared_msg, shared_time=95.0, offset=(0.0, 0.0, 0.0)):
        self.shared_map_msg = shared_msg
        self._shared_map_time = shared_time
        self._offset = offset
        self._shared_mask_active = False
        self._shared_masked_cells = 0

    def _now_sec(self):
        return 100.0

    def _get_common_offset(self):
        return self._offset

    def get_logger(self):
        return _Logger()


def own_info(ox=0.0, oy=0.0):
    return grid_msg(np.zeros((6, 6), np.int8), ox, oy).info


def call(stub, info, unknown):
    return FrontierExplorer._peer_known_mask(stub, info, unknown)


def test_masks_only_cells_the_shared_map_knows():
    shared = np.full((6, 6), UNKNOWN, np.int8)
    shared[2, 2] = 0        # free in merged map
    shared[3, 3] = 100      # wall in merged map
    unknown = np.zeros((6, 6), bool)
    unknown[2, 2] = unknown[3, 3] = unknown[4, 4] = True
    mask = call(Stub(grid_msg(shared)), own_info(), unknown)
    assert mask is not None
    assert mask[2, 2] and mask[3, 3]
    assert not mask[4, 4]           # shared map is unknown there too
    assert mask.sum() == 2


def test_stale_shared_map_disables_masking():
    shared = np.zeros((6, 6), np.int8)
    unknown = np.ones((6, 6), bool)
    stub = Stub(grid_msg(shared), shared_time=100.0 - 21.0)
    assert call(stub, own_info(), unknown) is None


def test_missing_offset_disables_masking():
    shared = np.zeros((6, 6), np.int8)
    unknown = np.ones((6, 6), bool)
    stub = Stub(grid_msg(shared), offset=None)
    assert call(stub, own_info(), unknown) is None


def test_out_of_shared_bounds_cells_stay_unmasked():
    shared = np.zeros((2, 2), np.int8)      # tiny merged map near origin
    unknown = np.zeros((6, 6), bool)
    unknown[0, 0] = True                    # inside shared extent
    unknown[5, 5] = True                    # far outside it
    mask = call(Stub(grid_msg(shared)), own_info(), unknown)
    assert mask is not None
    assert mask[0, 0]
    assert not mask[5, 5]


def test_rotated_offset_maps_cells_correctly():
    # Own frame rotated 180 deg and shifted: own cell (r=0,c=0) at world
    # (0.05, 0.05) lands at (1.0, 1.0) - (0.05, 0.05) = (0.95, 0.95) in the
    # common frame, which is shared cell (r=9, c=9) of a 10x10 grid.
    shared = np.full((10, 10), UNKNOWN, np.int8)
    shared[9, 9] = 0
    unknown = np.zeros((6, 6), bool)
    unknown[0, 0] = True
    unknown[1, 1] = True    # lands at (0.85, 0.85) -> shared (8,8): unknown
    stub = Stub(grid_msg(shared), offset=(1.0, 1.0, math.pi))
    mask = call(stub, own_info(), unknown)
    assert mask is not None
    assert mask[0, 0]
    assert not mask[1, 1]
