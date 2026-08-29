"""A distant room has to be able to beat a nearby scrap.

husarion_office, office_final_check: the run ended with a 7.55 m frontier at
(12.53, -1.24) -- the doorway region of floor_6, a 20 m2 room -- untouched,
while leo1 spent its last 390 s gaining 3.7 m2 from leftovers near (6.4, -10.6).

Under the linear utility that is not a bug in the search, it is arithmetic:
the reward was the frontier's own boundary length, so the room was worth 7.55
against a distance weighted 3.0, and it could only ever win from within 3.7 m.
The ratio utility scores unknown area per metre driven instead, so yield
decides rather than proximity.
"""
import math

from leo_rover_exploration.coordination import (
    base_utility, coordinated_allocation)

# Straight off the end of office_final_check: leo1 at (6.4, -10.6), the
# floor_6 doorway frontier it never took, and the kind of sliver it kept
# taking instead while gaining 3.7 m2 in 390 s.
ROVER = (6.4, -10.6)
ROOM = {'goal': (12.53, -1.24), 'size_m': 7.55, 'gain': 7.55}
SLIVER = {'goal': (5.4, -11.2), 'size_m': 0.55, 'gain': 0.55}


def util(f, mode, robot=ROVER):
    return base_utility(f['gain'], robot, f['goal'], 3.0, 1.0, mode, 1.0)


def test_linear_utility_cannot_reach_the_far_room():
    """The defect. Distance is weighted 3.0 and the room is 11.2 m away, so
    it scores -26 against the sliver's -3, and no amount of the room being
    bigger changes that: it could only win from inside 3.7 m."""
    assert util(ROOM, 'linear') < util(SLIVER, 'linear')
    assert util(ROOM, 'linear') < -25.0


def test_ratio_utility_prefers_the_room_over_a_sliver():
    """The fix is not that the room always wins -- a genuinely rich patch
    underfoot still should -- but that a nearly exhausted one stops beating
    a whole unexplored room."""
    assert util(ROOM, 'ratio') > util(SLIVER, 'ratio')


def test_ratio_still_takes_a_rich_patch_underfoot_first():
    """Greedy-by-yield is still greedy, and that is correct: 2 m2 one metre
    away really is a better use of the next metre than 7.55 eleven away."""
    rich_and_near = {'goal': (5.4, -11.2), 'gain': 2.0}
    assert util(rich_and_near, 'ratio') > util(ROOM, 'ratio')


def test_ratio_is_gain_per_metre():
    d = math.hypot(ROOM['goal'][0] - ROVER[0], ROOM['goal'][1] - ROVER[1])
    assert util(ROOM, 'ratio') == ROOM['gain'] / d


def test_ratio_still_prefers_the_nearer_of_two_equal_regions():
    near = {'goal': (7.4, -10.6), 'gain': 10.0}
    far = {'goal': (16.4, -10.6), 'gain': 10.0}
    assert util(near, 'ratio') > util(far, 'ratio')


def test_ratio_still_prefers_the_richer_of_two_equidistant_regions():
    poor = {'goal': (6.4, -5.6), 'gain': 3.0}
    rich = {'goal': (11.4, -10.6), 'gain': 12.0}
    assert util(rich, 'ratio') > util(poor, 'ratio')


def test_min_travel_cost_stops_a_frontier_underfoot_scoring_infinity():
    underfoot = {'goal': (6.4, -10.6), 'gain': 1.0}
    assert util(underfoot, 'ratio') == 1.0        # gain / max(0, 1.0)
    assert math.isfinite(util(underfoot, 'ratio'))


def test_linear_mode_is_unchanged():
    """The old behaviour stays available and identical."""
    assert util(SLIVER, 'linear') == 1.0 * SLIVER['gain'] - 3.0 * math.hypot(
        SLIVER['goal'][0] - ROVER[0], SLIVER['goal'][1] - ROVER[1])


def test_allocation_honours_the_utility_mode():
    robots = [('leo1', ROVER)]
    frontiers = [ROOM, SLIVER]
    linear = coordinated_allocation(robots, frontiers, utility_mode='linear')
    ratio = coordinated_allocation(robots, frontiers, utility_mode='ratio')
    assert linear['leo1'] == SLIVER['goal']
    assert ratio['leo1'] == ROOM['goal']


def test_allocation_defaults_to_linear_for_existing_callers():
    got = coordinated_allocation([('leo1', ROVER)], [ROOM, SLIVER])
    assert got['leo1'] == SLIVER['goal']


def test_explicit_gain_beats_size_m_when_both_are_present():
    """The information gain is what the explorer computes; size_m is only the
    fallback for callers that never measured one."""
    robots = [('leo1', (0.0, 0.0))]
    wide_but_shallow = {'goal': (1.0, 0.0), 'size_m': 30.0, 'gain': 1.0}
    narrow_but_deep = {'goal': (1.0, 1.0), 'size_m': 1.0, 'gain': 30.0}
    got = coordinated_allocation(robots, [wide_but_shallow, narrow_but_deep],
                                 utility_mode='ratio')
    assert got['leo1'] == narrow_but_deep['goal']


def test_two_rovers_still_get_different_frontiers():
    """The ratio mode must not break the fan-apart property."""
    robots = [('leo1', (0.0, 0.0)), ('leo2', (10.0, 0.0))]
    frontiers = [{'goal': (1.0, 0.0), 'gain': 8.0},
                 {'goal': (11.0, 0.0), 'gain': 8.0}]
    got = coordinated_allocation(robots, frontiers, utility_mode='ratio')
    assert got['leo1'] != got['leo2']
    assert got['leo1'] is not None and got['leo2'] is not None
