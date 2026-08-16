"""Unit tests for the distributed coordinated frontier allocation policy."""

from leo_rover_exploration.coordination import (
    coordinated_allocation, independent_choice)


def f(x, y, size=1.0):
    return {'goal': (x, y), 'size_m': size}


def test_two_robots_take_nearest_frontier():
    robots = [('leo1', (0.0, 0.0)), ('leo2', (10.0, 0.0))]
    frontiers = [f(1.0, 0.0), f(9.0, 0.0)]
    a = coordinated_allocation(robots, frontiers)
    assert a['leo1'] == (1.0, 0.0)
    assert a['leo2'] == (9.0, 0.0)


def test_more_robots_than_frontiers_leaves_one_idle():
    robots = [('leo1', (0.0, 0.0)), ('leo2', (0.2, 0.0))]
    a = coordinated_allocation(robots, [f(1.0, 0.0)])
    assigned = [v for v in a.values() if v is not None]
    assert assigned == [(1.0, 0.0)]
    assert list(a.values()).count(None) == 1


def test_proximity_discount_fans_robots_apart():
    # Both rovers start at the origin. There is a rich cluster of frontiers to
    # the east and a lone frontier to the west.
    robots = [('leo1', (0.0, 0.0)), ('leo2', (0.0, 0.0))]
    frontiers = [f(5.0, 0.0, 3), f(5.4, 0.0, 3), f(5.0, 0.4, 3), f(-5.0, 0.0, 1)]

    # No discount: both rovers pile into the eastern cluster.
    greedy = coordinated_allocation(robots, frontiers, discount_strength=0.0)
    assert greedy['leo1'][0] > 0 and greedy['leo2'][0] > 0

    # With the discount, the second rover is pushed to the western frontier.
    fanned = coordinated_allocation(robots, frontiers,
                                    discount_radius=3.0, discount_strength=1.0)
    xs = sorted(v[0] for v in fanned.values())
    assert xs[0] < 0 < xs[1], f"expected a split, got {fanned}"


def test_independent_choice_is_redundant():
    # Two rovers running the uncoordinated baseline pick the same frontier.
    frontiers = [f(5.0, 0.0, 3), f(-5.0, 0.0, 1)]
    c1 = independent_choice((0.0, 0.0), frontiers)
    c2 = independent_choice((0.1, 0.0), frontiers)
    assert c1 == c2 == (5.0, 0.0)


def test_committed_peer_goal_is_respected():
    # leo2 has already committed to the eastern frontier; leo1 should take the
    # western one even though east has higher raw gain.
    robots = [('leo1', (0.0, 0.0)), ('leo2', (0.0, 0.0))]
    frontiers = [f(5.0, 0.0, 5), f(-5.0, 0.0, 1)]
    a = coordinated_allocation(robots, frontiers,
                               committed={'leo2': (5.0, 0.0)})
    assert a['leo2'] == (5.0, 0.0)
    assert a['leo1'] == (-5.0, 0.0)


def test_deterministic():
    robots = [('leo1', (0.0, 0.0)), ('leo2', (3.0, 3.0))]
    frontiers = [f(1.0, 1.0, 2), f(2.0, 2.0, 2), f(4.0, 4.0, 2)]
    a1 = coordinated_allocation(robots, frontiers)
    a2 = coordinated_allocation(robots, frontiers)
    assert a1 == a2
