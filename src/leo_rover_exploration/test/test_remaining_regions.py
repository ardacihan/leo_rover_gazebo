"""Unexplored rooms must stay visible after the doorway frontier vanishes."""

import numpy as np

from leo_rover_exploration.remaining_regions import (
    remaining_room_area_m2, remaining_unknown_regions)


def _office_stub():
    # 0.10 m cells. A mapped corridor on the left, an unknown 3x3 m room
    # opening through a one-cell doorway.
    h, w = 80, 80
    unknown = np.zeros((h, w), dtype=bool)
    free = np.zeros((h, w), dtype=bool)
    free[20:50, 5:25] = True
    unknown[22:52, 30:60] = True
    free[34:36, 25:31] = True          # doorway
    return unknown, free


def test_remaining_room_is_reported_when_doorway_frontier_is_gone():
    unknown, free = _office_stub()
    rooms = remaining_unknown_regions(unknown, free, 0.10, (0.0, 0.0),
                                      min_area_m2=2.0)
    assert len(rooms) == 1
    assert rooms[0]['area_m2'] > 8.0
    # Goal sits on the free side of the doorway, not in unknown space.
    gx, gy = rooms[0]['goal']
    col = int((gx - 0.0) / 0.10)
    row = int((gy - 0.0) / 0.10)
    assert free[row, col]


def test_sealed_unknown_blob_is_not_a_remaining_room():
    unknown = np.zeros((40, 40), dtype=bool)
    free = np.zeros((40, 40), dtype=bool)
    unknown[10:20, 10:20] = True
    free[30:35, 30:35] = True
    rooms = remaining_unknown_regions(unknown, free, 0.10, (0.0, 0.0),
                                      min_area_m2=0.5)
    assert rooms == []


def test_tiny_unknown_pocket_is_ignored():
    unknown, free = _office_stub()
    unknown[22:52, 30:60] = False
    unknown[24:26, 32:34] = True
    free[25, 31] = True
    rooms = remaining_unknown_regions(unknown, free, 0.10, (0.0, 0.0),
                                      min_area_m2=2.0)
    assert rooms == []


def test_remaining_area_sums_reachable_rooms():
    unknown, free = _office_stub()
    rooms = remaining_unknown_regions(unknown, free, 0.10, (0.0, 0.0),
                                      min_area_m2=2.0)
    assert remaining_room_area_m2(rooms) == rooms[0]['area_m2']


def test_unknown_mostly_at_world_edge_is_exterior_not_a_room():
    # The bulk of this blob lies in the edge band outside the 6x6 world:
    # that is the exterior halo behind the outer wall.
    unknown = np.zeros((80, 80), dtype=bool)
    free = np.zeros((80, 80), dtype=bool)
    unknown[50:79, 50:79] = True
    free[48:50, 52:56] = True
    rooms = remaining_unknown_regions(
        unknown, free, 0.10, (0.0, 0.0), min_area_m2=2.0,
        world_bounds=(0.0, 6.0, 0.0, 6.0), bounds_margin_m=0.4)
    assert rooms == []


def test_perimeter_room_touching_world_edge_survives():
    # World bounds are wall centrelines, so a real perimeter room's unknown
    # reaches within centimetres of the bound. Touching the edge band must
    # not retire the room - only being mostly inside the band does.
    unknown = np.zeros((80, 80), dtype=bool)
    free = np.zeros((80, 80), dtype=bool)
    unknown[20:79, 20:70] = True       # reaches y = 7.9 of an 8 m world
    free[35:37, 15:21] = True
    rooms = remaining_unknown_regions(
        unknown, free, 0.10, (0.0, 0.0), min_area_m2=2.0,
        world_bounds=(0.0, 8.0, 0.0, 8.0), bounds_margin_m=0.4)
    assert len(rooms) == 1
