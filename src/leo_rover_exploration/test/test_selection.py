import math

from leo_rover_exploration.observations import distinct_view
from leo_rover_exploration.selection import sweep_candidate_metrics


def test_nearby_cluster_wins_when_information_is_equal():
    near = sweep_candidate_metrics((0, 0), 0, (2, 0), (3, 0), 20)
    far = sweep_candidate_metrics((0, 0), 0, (12, 0), (13, 0), 20)
    assert near['utility'] > far['utility']


def test_large_remote_gain_can_still_win():
    near = sweep_candidate_metrics((0, 0), 0, (2, 0), (3, 0), 5)
    far = sweep_candidate_metrics((0, 0), 0, (8, 0), (9, 0), 80)
    assert far['utility'] > near['utility']


def test_heading_change_is_part_of_motion_cost():
    straight = sweep_candidate_metrics((0, 0), 0, (2, 0), (3, 0), 20)
    behind = sweep_candidate_metrics((0, 0), math.pi, (2, 0), (3, 0), 20)
    assert straight['utility'] > behind['utility']


def test_adjacent_detection_frames_are_not_independent():
    first = (10.0, 1.0, 2.0, 0.0)
    adjacent = (10.5, 1.01, 2.01, 0.01)
    assert not distinct_view(first, adjacent, 0.5, 5.0, 0.35)


def test_time_motion_or_heading_can_make_a_view_independent():
    first = (10.0, 1.0, 2.0, 0.0)
    assert distinct_view(first, (15.0, 1.0, 2.0, 0.0), 0.5, 5.0, 0.35)
    assert distinct_view(first, (10.5, 1.6, 2.0, 0.0), 0.5, 5.0, 0.35)
    assert distinct_view(first, (10.5, 1.0, 2.0, 0.4), 0.5, 5.0, 0.35)
