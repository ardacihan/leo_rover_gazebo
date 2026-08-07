import pathlib
import sys
import unittest


SCRIPTS = pathlib.Path(__file__).parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from exploration_policy import (  # noqa: E402
    choose_escape_action,
    choose_turn_direction,
    robust_clearance,
    turn_clearances,
)


class ExplorationPolicyTests(unittest.TestCase):
    def test_sparse_thin_returns_are_filtered(self):
        values = [0.4] * 5 + [1.8] * 20
        self.assertEqual(robust_clearance(values, 8), 1.8)

    def test_dense_wall_is_not_filtered(self):
        values = [0.7] * 14 + [2.0] * 20
        self.assertEqual(robust_clearance(values, 8), 0.7)

    def test_turn_choice_accounts_for_opposite_rear_sweep(self):
        left, right = turn_clearances(
            left=1.2, right=2.0, rear_left=0.3, rear_right=1.0
        )
        self.assertEqual(choose_turn_direction(left, right), 1.0)

    def test_reverse_when_both_turns_are_blocked(self):
        action, direction = choose_escape_action(
            0.3, 0.32, 0.35, 1.1, 0.75, 0, 2
        )
        self.assertEqual((action, direction), ("reverse", 0.0))

    def test_stop_when_rear_is_blocked(self):
        action, _ = choose_escape_action(
            0.3, 0.32, 0.35, 0.5, 0.75, 0, 2
        )
        self.assertEqual(action, "stop")

    def test_stop_after_reverse_attempt_limit(self):
        action, _ = choose_escape_action(
            0.3, 0.32, 0.35, 1.1, 0.75, 2, 2
        )
        self.assertEqual(action, "stop")


if __name__ == "__main__":
    unittest.main()
