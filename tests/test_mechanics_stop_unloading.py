import unittest

from simulation.analyze_mechanics_stop_unloading import (
    chronological_change, executed_indices, fixed_window, terminal_phase_indices, window_change,
)


class MechanicsStopUnloadingTests(unittest.TestCase):
    def test_guard_before_first_step_has_no_stop_or_unloading(self):
        copied = [{'phase': 'stop', 'recovery_time_s': 0., 'time_s': 10.,
                   'normal_load_n': 12., 'wrist_force_n': 5.}]
        indices = executed_indices(copied, 'stop')
        self.assertEqual(indices, [])
        self.assertFalse(fixed_window(copied, indices, 1/240)['observed'])
        self.assertIsNone(chronological_change(copied[0], None))

    def test_fixed_tail_window_does_not_choose_earlier_peak_or_copied_row(self):
        rows = [{'phase': 'stop', 'time_s': i/240, 'recovery_time_s': i/240,
                 'normal_load_n': 100. if i < 30 else 2., 'wrist_force_n': 1.}
                for i in range(61)]
        indices = executed_indices(rows, 'stop')
        self.assertEqual(len(indices), 60)
        tail = fixed_window(rows, indices, 1/240)
        self.assertTrue(tail['observed'])
        self.assertEqual(tail['start_sample_index'], 37)
        self.assertEqual(tail['end_sample_index'], 60)
        self.assertEqual(tail['mean_normal_load_n'], 2.)
        self.assertAlmostEqual(tail['support_s'], .1)
        self.assertAlmostEqual(tail['first_last_timestamp_span_s'], 23/240)
        first = fixed_window(rows, indices, 1/240, at_end=False)
        self.assertEqual(first['start_sample_index'], 1)
        self.assertEqual(first['mean_normal_load_n'], 100.)

    def test_windows_reject_time_gaps_and_do_not_fill_missing_signal_with_zero(self):
        rows = [{'phase': 'retreat', 'time_s': (i+1)/100, 'normal_load_n': None}
                for i in range(10)]
        window = fixed_window(rows, list(range(10)), .01)
        self.assertTrue(window['observed'])
        self.assertIsNone(window['mean_normal_load_n'])
        rows[5]['time_s'] = .061
        self.assertFalse(fixed_window(rows, list(range(10)), .01)['observed'])
        self.assertIsNone(window_change(window, {'observed': False}))

    def test_changes_use_corresponding_values_and_zero_baseline_has_no_ratio(self):
        result = chronological_change({'normal_load_n': 2., 'wrist_force_n': 0.,
                                       'tip_x_mm': .1, 'tip_y_mm': .2},
                                      {'normal_load_n': 1., 'wrist_force_n': .1,
                                       'tip_x_mm': .4, 'tip_y_mm': .6})
        self.assertEqual(result['delta_normal_load_n'], -1.)
        self.assertEqual(result['fractional_change_normal_load_n'], -.5)
        self.assertIsNone(result['fractional_change_wrist_force_n'])
        self.assertAlmostEqual(result['tip_lateral_motion_mm'], .5)

    def test_reference_tail_excludes_initialization_and_previous_phase(self):
        rows = [{'phase': 'approach', 'reference_step': 0},
                {'phase': 'approach', 'reference_step': 1}]
        self.assertEqual(terminal_phase_indices(rows), [1])
        rows += [{'phase': 'hold', 'reference_step': 2}]
        self.assertEqual(terminal_phase_indices(rows), [2])


if __name__ == '__main__':
    unittest.main()
