"""A rover that never left its spawn has not finished exploring.

husarion_office, 2026-08-29 (run D): leo2 spawned inside lethal costmap cost.
Every compute_path_to_pose failed -- for goals 0.86 m away -- so it never
reached a single frontier. Two things then went wrong:

  * each warmup nudge cleared skips_since_dispatch, so the counter never
    reached escape_after_skips and the stronger escape never fired;
  * once its one frontier hit the 3-strike long ban, "no eligible frontier"
    was read as a finished map: 6.1 m2 mapped, 1.1 m travelled, and the run
    recorded finished=1/2.

The escape counter must only reset when the escape actually moved the rover,
and completion must require having reached at least one goal.
"""
import math

from leo_rover_exploration.frontier_explorer import FrontierExplorer


class EscapeStub:
    """Just enough of the node to drive _escape_tick's tail."""
    escape_after_skips = 6

    _escape_tick = FrontierExplorer._escape_tick

    def __init__(self, start, end, skips=5):
        self.escape_ticks = 1
        self.escape_count = 1
        self._escape_from = start
        self._last_xy = end
        self.skips_since_dispatch = skips
        self.skip_list = []
        self.cleared = 0
        self.published = []

    # collaborators
    class _Pub:
        def __init__(self, outer):
            self.outer = outer

        def publish(self, msg):
            self.outer.published.append(msg)

    @property
    def cmd_vel_pub(self):
        return EscapeStub._Pub(self)

    def _clear_costmaps(self):
        self.cleared += 1

    def _now_sec(self):
        return 1000.0


def test_escape_that_moved_the_rover_clears_the_skip_count():
    s = EscapeStub(start=(0.0, 0.0), end=(0.8, 0.0))
    s._escape_tick()
    assert s.skips_since_dispatch == 0


def test_escape_that_freed_nothing_keeps_the_skip_count():
    """This is the leo2 case: nudges that achieve nothing must not reset the
    ladder, or escape_after_skips can never be reached."""
    s = EscapeStub(start=(0.0, 0.0), end=(0.05, 0.0))
    s._escape_tick()
    assert s.skips_since_dispatch == 5


def test_a_fresh_burst_anchors_on_its_first_tick():
    """The anchor is captured when the burst starts, so the final tick can
    tell whether the whole burst achieved any motion."""
    s = EscapeStub(start=None, end=(3.0, 1.0))
    s.escape_ticks = 5          # a real burst, not its last tick
    s._escape_tick()
    assert s._escape_from == (3.0, 1.0)
    assert s.escape_ticks == 4
    assert s.skips_since_dispatch == 5   # untouched until the burst ends


def test_escape_resets_its_anchor_for_the_next_burst():
    s = EscapeStub(start=(0.0, 0.0), end=(0.05, 0.0))
    s._escape_tick()
    assert s._escape_from is None


def test_the_move_threshold_is_twenty_centimetres():
    just_under = EscapeStub(start=(0.0, 0.0), end=(0.19, 0.0))
    just_under._escape_tick()
    assert just_under.skips_since_dispatch == 5
    just_over = EscapeStub(start=(0.0, 0.0), end=(0.21, 0.0))
    just_over._escape_tick()
    assert just_over.skips_since_dispatch == 0
    assert math.isclose(0.21, 0.21)
