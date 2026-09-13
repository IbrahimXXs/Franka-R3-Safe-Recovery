"""Meaningful checks for the belief quantifier and candidate rejection logic."""

import unittest
from research.planner import Candidate, best_common_recovery, select_candidate


class PlannerMathTests(unittest.TestCase):
    def test_no_hidden_state_oracle(self):
        # Each hidden state has a perfect but mutually incompatible recovery.
        # A planner without identifying observations can only succeed with p=0.5.
        _, score = best_common_recovery({"left": (1.0, 0.0), "right": (0.0, 1.0)}, (0.5, 0.5))
        self.assertEqual(score, 0.5)

    def test_unsafe_candidate_path_is_rejected_despite_recoverable_endpoint(self):
        action = Candidate("bad_path", 10.0, (0.2, 0.2), {"retreat": (1.0, 1.0)})
        self.assertIsNone(select_candidate([action], (0.5, 0.5)))

    def test_empty_or_unsupported_recovery_is_not_a_safe_hold(self):
        self.assertIsNone(select_candidate([], (1.0,)))
        self.assertIsNone(select_candidate([Candidate("none", 1.0, (1.0,), {})], (1.0,)))

    def test_less_progress_can_preserve_recovery(self):
        actions = [Candidate("deep", 2, (1.0,), {"retreat": (0.1,)}),
                   Candidate("shallow", 1, (1.0,), {"retreat": (0.99,)})]
        self.assertEqual(select_candidate(actions, (1.0,))["candidate"], "shallow")

    def test_invalid_prediction_fails_closed(self):
        with self.assertRaises(ValueError):
            select_candidate([Candidate("nan", 1, (float("nan"),), {"retreat": (1.0,)})], (1.0,))


if __name__ == "__main__":
    unittest.main()
