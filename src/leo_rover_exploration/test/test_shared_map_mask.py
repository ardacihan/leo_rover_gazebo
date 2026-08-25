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


def grid_msg(grid, ox=0.0, oy=0.0, frame='common', origin_yaw=0.0):
    h, w = grid.shape
    info = types.SimpleNamespace(
        height=h, width=w, resolution=RES,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=ox, y=oy),
            orientation=types.SimpleNamespace(
                x=0.0, y=0.0, z=math.sin(origin_yaw / 2.0),
                w=math.cos(origin_yaw / 2.0))))
    return types.SimpleNamespace(
        info=info, data=grid.flatten().tolist(),
        header=types.SimpleNamespace(frame_id=frame))


class _Logger:
    def info(self, *a, **k):
        pass


class Stub:
    shared_map_max_age = 20.0
    map_frame = 'own/map'
    common_frame = 'common'

    def __init__(self, shared_msg, shared_time=95.0, offset=(0.0, 0.0, 0.0)):
        self.shared_map_msg = shared_msg
        self._shared_map_time = shared_time
        self._offset = offset
        self._shared_mask_active = False
        self._shared_map_available = False
        self._shared_masked_cells = 0
        self._shared_alignment_required = False
        self._shared_alignment_locked = True

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


def test_prelock_central_fallback_cannot_mask_frontiers():
    shared = np.zeros((6, 6), np.int8)
    unknown = np.ones((6, 6), bool)
    stub = Stub(grid_msg(shared))
    stub._shared_alignment_required = True
    stub._shared_alignment_locked = False
    assert call(stub, own_info(), unknown) is None
    assert not stub._shared_map_available


def test_missing_offset_disables_masking():
    shared = np.zeros((6, 6), np.int8)
    unknown = np.ones((6, 6), bool)
    stub = Stub(grid_msg(shared), offset=None)
    assert call(stub, own_info(), unknown) is None


def test_own_frame_shared_map_needs_no_offset():
    # A per-rover merger (Phase 3) publishes in the rover's own map frame;
    # the mask must work with NO alignment TF at all (offset None).
    shared = np.zeros((6, 6), np.int8)          # fully known merged map
    unknown = np.zeros((6, 6), bool)
    unknown[2, 2] = True
    stub = Stub(grid_msg(shared, frame='own/map'), offset=None)
    mask = call(stub, own_info(), unknown)
    assert mask is not None
    assert mask[2, 2]


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


def test_rotated_grid_origins_are_used_for_shared_lookup():
    # The own cell centre is (-0.05, 0.05) after a +90 degree origin pose.
    # The shared grid also has a +90 degree origin, so this is shared (0,0).
    shared = np.full((4, 4), UNKNOWN, np.int8)
    shared[0, 0] = 0
    unknown = np.zeros((2, 2), bool)
    unknown[0, 0] = True
    own = grid_msg(
        np.zeros((2, 2), np.int8), origin_yaw=math.pi / 2).info
    msg = grid_msg(shared, origin_yaw=math.pi / 2)
    mask = call(Stub(msg), own, unknown)
    assert mask[0, 0]


def test_active_goal_is_redundant_when_shared_mask_removes_its_frontier():
    clusters = [
        {'goal': (1.0, 1.0)},
        {'goal': (4.0, 4.0)},
    ]
    assert FrontierExplorer._goal_has_matching_frontier(
        (1.2, 1.1), clusters, 0.5)
    assert not FrontierExplorer._goal_has_matching_frontier(
        (2.5, 2.5), clusters, 0.5)


def test_marker_frontier_bonus_prefers_unknown_near_confirmed_landmark():
    stub = types.SimpleNamespace(
        landmarks={7: (2.0, 2.0)},
        marker_frontier_radius=4.0,
        marker_frontier_bonus=18.0,
    )
    near = FrontierExplorer._marker_bonus(
        stub, {'goal': (3.0, 2.0), 'size_m': 1.0})
    far = FrontierExplorer._marker_bonus(
        stub, {'goal': (7.0, 2.0), 'size_m': 1.0})
    assert near == 13.5
    assert far == 0.0
