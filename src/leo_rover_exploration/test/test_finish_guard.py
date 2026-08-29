"""The explorer must not retire while a frontier is only briefly banned.

FrontierExplorer._temporarily_suppressed decides whether "nothing eligible"
means "map finished" or "wait for a ban to lapse". One Nav2 abort bans its
goal for blacklist_ttl (90 s) and one planner rejection for skip_ttl (20 s),
while the finish timer is idle_cycles_to_finish x planner_period = 12 s.
Without this guard a rover declares the world explored seconds after a single
failed goal -- and STATE_DONE is terminal, so it never plans again.

Pure-logic tests: the methods are invoked unbound on a stub, no rclpy node.
"""
from leo_rover_exploration.frontier_explorer import FrontierExplorer


class Stub:
    min_frontier_size = 0.5
    blacklist_radius = 0.6
    blacklist_growth = 0.9
    blacklist_max_radius = 3.0
    blacklist_max_strikes = 3

    _entry_radius = FrontierExplorer._entry_radius
    _temporarily_suppressed = FrontierExplorer._temporarily_suppressed

    def __init__(self, now=1000.0):
        self._now = now
        self.blacklist = []
        self.skip_list = []

    def _now_sec(self):
        return self._now


def cluster(x, y, size=2.0):
    return {'size_m': size, 'goal': (x, y), 'centroid': (x, y)}


def test_frontier_under_a_short_ban_holds_exploration_open():
    s = Stub()
    s.blacklist = [{'pos': (2.0, -8.0), 'expires': s._now + 90.0, 'strikes': 1}]
    held = s._temporarily_suppressed([cluster(2.0, -8.0)])
    assert [c['goal'] for c in held] == [(2.0, -8.0)]


def test_a_planner_skip_also_holds_exploration_open():
    s = Stub()
    s.skip_list = [{'pos': (2.9, -6.7), 'expires': s._now + 20.0, 'strikes': 1}]
    assert len(s._temporarily_suppressed([cluster(2.9, -6.7, size=18.9)])) == 1


def test_confirmed_unreachable_frontier_does_not_hold():
    """A ban that reached max strikes is the give-up verdict of the strike
    ladder; honouring it is what lets a walled-off gap end the run."""
    s = Stub()
    s.blacklist = [{'pos': (2.0, -8.0), 'expires': s._now + 900.0,
                    'strikes': 3}]
    assert s._temporarily_suppressed([cluster(2.0, -8.0)]) == []


def test_expired_ban_is_not_suppression():
    s = Stub()
    s.blacklist = [{'pos': (2.0, -8.0), 'expires': s._now - 1.0, 'strikes': 1}]
    assert s._temporarily_suppressed([cluster(2.0, -8.0)]) == []


def test_unbanned_frontier_is_not_held():
    s = Stub()
    assert s._temporarily_suppressed([cluster(2.0, -8.0)]) == []


def test_frontier_below_the_size_floor_never_holds():
    s = Stub()
    s.blacklist = [{'pos': (2.0, -8.0), 'expires': s._now + 90.0, 'strikes': 1}]
    assert s._temporarily_suppressed([cluster(2.0, -8.0, size=0.2)]) == []


def test_a_frontier_out_of_ban_radius_is_not_held():
    s = Stub()
    s.blacklist = [{'pos': (2.0, -8.0), 'expires': s._now + 90.0, 'strikes': 1}]
    assert s._temporarily_suppressed([cluster(9.0, -8.0)]) == []


def test_regression_office_independent_leo1_2026_08_29():
    """leo1 announced "Exploration finished" at 15.8 m2 with an 18.90 m
    frontier cluster at (2.91, -6.70) still in its own map -- the goal it had
    skipped for "no path" seconds earlier. That frontier must hold the run
    open rather than end it.
    """
    s = Stub()
    s.skip_list = [{'pos': (2.89, -6.67), 'expires': s._now + 20.0,
                    'strikes': 1}]
    held = s._temporarily_suppressed([cluster(2.91, -6.70, size=18.90)])
    assert len(held) == 1


def test_mixed_bans_hold_only_the_recoverable_frontier():
    s = Stub()
    s.blacklist = [
        {'pos': (2.0, -8.0), 'expires': s._now + 90.0, 'strikes': 1},
        {'pos': (9.0, -2.0), 'expires': s._now + 900.0, 'strikes': 3},
    ]
    held = s._temporarily_suppressed(
        [cluster(2.0, -8.0), cluster(9.0, -2.0)])
    assert [c['goal'] for c in held] == [(2.0, -8.0)]
