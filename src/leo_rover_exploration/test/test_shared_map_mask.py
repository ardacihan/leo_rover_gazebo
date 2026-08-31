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


def grid_msg(grid, ox=0.0, oy=0.0, frame='common'):
    h, w = grid.shape
    info = types.SimpleNamespace(
        height=h, width=w, resolution=RES,
        origin=types.SimpleNamespace(
            position=types.SimpleNamespace(x=ox, y=oy)))
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
    alignment_locked = True

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


def test_masks_only_cells_the_shared_map_knows_solidly():
    # A SOLID known block in the merged map counts as covered; a cell the
    # merged map does not know stays unmasked.
    shared = np.full((12, 12), UNKNOWN, np.int8)
    shared[0:9, 0:9] = 0    # solidly known block (free)
    shared[4, 4] = 100      # a wall inside it
    unknown = np.zeros((12, 12), bool)
    unknown[3, 3] = unknown[4, 4] = unknown[11, 11] = True
    mask = call(Stub(grid_msg(shared)), own_info(), unknown)
    assert mask is not None
    assert mask[3, 3] and mask[4, 4]
    assert not mask[11, 11]         # shared map is unknown there
    assert mask.sum() == 2


def test_patchy_glimpses_do_not_mask():
    # Sparse wall fragments (a peer's lidar seeing a room through a window)
    # must NOT count as coverage: treating them as covered erased every
    # frontier into the husarion corner rooms and exploration finished with
    # the rooms never entered.
    shared = np.full((10, 10), UNKNOWN, np.int8)
    shared[2, 2] = 100      # isolated fragments
    shared[5, 7] = 100
    unknown = np.ones((10, 10), bool)
    mask = call(Stub(grid_msg(shared)), own_info(), unknown)
    assert mask is not None
    assert mask.sum() == 0


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
    # common frame, which is shared cell (r=9, c=9) of a 10x10 grid. The
    # shared map is solidly known except one hole at (8,8).
    shared = np.zeros((10, 10), np.int8)
    shared[8, 8] = UNKNOWN
    unknown = np.zeros((6, 6), bool)
    unknown[0, 0] = True
    unknown[1, 1] = True    # lands at (0.85, 0.85) -> shared (8,8): unknown
    stub = Stub(grid_msg(shared), offset=(1.0, 1.0, math.pi))
    mask = call(stub, own_info(), unknown)
    assert mask is not None
    assert mask[0, 0]
    assert not mask[1, 1]


class CandidateStub:
    _shared_mask_active = False
    alignment_locked = False
    alignment_ambiguity = 0.0
    candidate_max_age = 20.0
    candidate_guidance_min_confidence = 0.15

    def __init__(self, robot, peer_grid, transform, confidence):
        self.robot_name = robot
        self.candidate_map_msg = grid_msg(peer_grid, frame='peer/map')
        self._candidate_map_time = 95.0
        self.candidate_transform = transform
        self._candidate_transform_time = 95.0
        self.alignment_confidence = confidence

    def _now_sec(self):
        return 100.0


def candidate(stub, xy):
    return FrontierExplorer._candidate_redundancy(stub, xy)


def test_soft_candidate_guidance_is_zero_below_confidence_floor():
    peer = np.zeros((20, 20), np.int8)
    stub = CandidateStub('leo2', peer, (1.0, 1.0, math.pi), 0.10)
    assert candidate(stub, (0.05, 0.05)) == 0.0


def test_ambiguous_candidate_does_not_steer_exploration():
    peer = np.zeros((20, 20), np.int8)
    stub = CandidateStub('leo2', peer, (1.0, 1.0, math.pi), 0.9)
    stub.alignment_ambiguity = 0.95
    assert candidate(stub, (0.05, 0.05)) == 0.0


def test_soft_candidate_guidance_maps_leo2_point_into_leo1_peer_map():
    peer = np.zeros((20, 20), np.int8)
    stub = CandidateStub('leo2', peer, (1.0, 1.0, math.pi), 0.8)
    assert candidate(stub, (0.05, 0.05)) > 0.75


def test_soft_candidate_guidance_inverts_transform_for_leo1():
    peer = np.zeros((20, 20), np.int8)
    stub = CandidateStub('leo1', peer, (1.0, 1.0, math.pi), 0.8)
    assert candidate(stub, (0.95, 0.95)) > 0.75


def test_accepted_shared_mask_disables_candidate_guidance():
    peer = np.zeros((20, 20), np.int8)
    stub = CandidateStub('leo2', peer, (0.0, 0.0, 0.0), 0.9)
    stub._shared_mask_active = True
    assert candidate(stub, (0.5, 0.5)) == 0.0
