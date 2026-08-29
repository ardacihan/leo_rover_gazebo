"""A pinned rover must recover, and must never be reported as finished.

husarion_office, 2026-08-29: leo1 drove into the notch west of the floor edge,
sat at (0.06, -6.65) moving 0.74 m in 200 s while every frontier failed, and
then logged "Exploration finished" with 30.1 m2 mapped. The run harness counts
that string, so a wedged rover was being recorded as a completed exploration.

The contract:
  * _immobilised() separates "map done" from "I cannot move";
  * _worthwhile() reports frontiers that remain regardless of bans;
  * _backtrack_step() aims at a pose the rover already held;
  * a backtrack failure never blacklists that pose.
"""
from collections import deque

from leo_rover_exploration.frontier_explorer import (
    GOAL_BACKTRACK, FrontierExplorer)


class Stub:
    min_frontier_size = 0.5
    backtrack_min_distance = 1.5
    backtrack_min_age = 30.0
    wedge_escapes = 3

    _immobilised = FrontierExplorer._immobilised
    _worthwhile = FrontierExplorer._worthwhile
    _backtrack_step = FrontierExplorer._backtrack_step

    def __init__(self, now=1000.0, robot=(0.06, -6.65)):
        self._now = now
        self._robot = robot
        self.pos_history = deque(maxlen=64)
        self.sent = []
        self.logged = []

    def _now_sec(self):
        return self._now

    def _robot_xy(self):
        return self._robot

    def _send_goal(self, xy, score, robot, kind):
        self.sent.append((xy, kind))

    def get_logger(self):
        stub = self

        class _L:
            def warn(self, msg, **k):
                stub.logged.append(msg)

            info = warn
            error = warn
        return _L()


def track(stub, points, step=2.0):
    t = stub._now - step * len(points)
    for x, y in points:
        stub.pos_history.append((t, x, y))
        t += step


def cluster(x, y, size=2.0):
    return {'size_m': size, 'goal': (x, y), 'centroid': (x, y)}


def test_pinned_rover_reads_as_immobilised():
    s = Stub()
    track(s, [(0.06, -6.65)] * 30)
    assert s._immobilised()


def test_driving_rover_does_not_read_as_immobilised():
    s = Stub()
    track(s, [(0.5 * i, -6.0) for i in range(30)])
    assert not s._immobilised()


def test_too_few_samples_is_not_immobilised():
    """A rover that just started must never be called stuck."""
    s = Stub()
    track(s, [(0.06, -6.65)] * 4)
    assert not s._immobilised()


def test_worthwhile_ignores_bans_but_honours_the_size_floor():
    s = Stub()
    got = s._worthwhile([cluster(2, -8), cluster(3, -9, size=0.1)])
    assert [c['goal'] for c in got] == [(2, -8)]


def test_backtrack_aims_at_a_pose_the_rover_held():
    s = Stub()
    # drove east then got pinned back at the notch
    track(s, [(4.0, -6.6)] * 5 + [(2.0, -6.6)] * 5 + [(0.06, -6.65)] * 20)
    assert s._backtrack_step()
    (target, kind) = s.sent[0]
    assert kind == GOAL_BACKTRACK
    assert target == (4.0, -6.6)     # furthest old pose, not the nearest


def test_backtrack_refuses_when_every_sample_is_too_close():
    s = Stub()
    track(s, [(0.06, -6.65)] * 30)
    assert not s._backtrack_step()
    assert s.sent == []


def test_backtrack_refuses_recent_samples():
    """Poses from the last few seconds are inside the wedge, not out of it."""
    s = Stub()
    s.pos_history.append((s._now - 2.0, 5.0, -6.6))
    assert not s._backtrack_step()


def test_backtrack_needs_a_pose_history():
    assert not Stub()._backtrack_step()
