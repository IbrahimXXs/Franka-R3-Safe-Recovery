import csv
import json
from pathlib import Path
import tempfile
import unittest

from simulation.analyze_mechanics_states import (
    export_states, mean100ms_selection, parse_cell, peak_selection, state_row,
)


class MechanicsStateExportTests(unittest.TestCase):
    def test_parse_false_and_null_without_inventing_zero(self):
        self.assertIs(parse_cell('False'), False)
        self.assertIs(parse_cell('true'), True)
        self.assertIsNone(parse_cell(''))
        self.assertIsNone(parse_cell('null'))
        self.assertEqual(parse_cell('0.0'), 0.)
        self.assertEqual(parse_cell('retreat'), 'retreat')
        for value in ('inf', '-Infinity', 'NaN'):
            with self.assertRaises(ValueError):
                parse_cell(value)

    def test_retreat_selection_excludes_stop_and_window_gaps(self):
        rows = [{'phase': 'stop', 'time_s': 0., 'recovery_time_s': 0., 'wrist_force_n': 100.}]
        rows += [{'phase': 'retreat', 'time_s': i*.01, 'recovery_time_s': i*.01,
                  'wrist_force_n': 2.} for i in range(1, 11)]
        peak = peak_selection(rows, 'wrist_force_n', 'retreat')
        self.assertEqual(peak['value_n'], 2.)
        mean = mean100ms_selection(rows, .01)
        self.assertEqual(mean['index'], 10)
        self.assertEqual(mean['window_start_sample_index'], 1)
        self.assertEqual(mean['value_n'], 2.)
        self.assertAlmostEqual(mean['window_support_s'], .1)
        self.assertIsNone(mean100ms_selection(rows[:-1], .01))
        rows[5]['time_s'] = .051
        self.assertIsNone(mean100ms_selection(rows, .01))

    def test_masks_keep_guide_length_distinct_from_unobserved_contact_span(self):
        sample = {'geometry_overlap_estimate_valid': True, 'effective_guide_span_estimate_mm': 10.5,
                  'wall_both_sides_loaded': False, 'wall_centroid_axial_span_mm': 9.,
                  'normal_force_world_z_n': None, 'friction_force_world_z_n': -.4}
        row = state_row({}, [sample], {'index': 0}, 'reference_terminal', 'insertion.csv',
                        'reference', 'reference')
        self.assertEqual(row['geometric_guide_length_Lg_mm'], 10.5)
        self.assertIsNone(row['observed_contact_separation_ell_mm'])
        self.assertIsNone(row['contact_normal_axial_resistance_n'])
        self.assertEqual(row['contact_friction_axial_resistance_n'], .4)

    def test_copied_stop_is_observed_but_did_not_execute(self):
        rows = [{'phase': 'stop', 'recovery_time_s': 0., 'normal_load_n': 9.9}]
        row = state_row({}, rows, {'index': 0}, 'stop_end', 'final_retreat.csv',
                        'straight', 'stop', dt=1/240)
        self.assertIs(row['state_observed'], True)
        self.assertIs(row['phase_executed'], False)
        self.assertIs(row['sample_is_copied_recovery_start'], True)
        self.assertEqual(row['phase_executed_sample_count'], 0)
        self.assertEqual(row['phase_observed_duration_s'], 0.)
        self.assertEqual(row['normal_load_n'], 9.9)

    def test_stop_duration_counts_only_executed_steps_through_selection(self):
        rows = [{'phase': 'stop', 'recovery_time_s': i/240} for i in range(61)]
        rows += [{'phase': 'retreat', 'recovery_time_s': 61/240}]
        row = state_row({}, rows, {'index': 60}, 'stop_end', 'final_retreat.csv',
                        'straight', 'stop', dt=1/240)
        self.assertIs(row['phase_executed'], True)
        self.assertIs(row['sample_is_copied_recovery_start'], False)
        self.assertEqual(row['phase_executed_sample_count'], 60)
        self.assertAlmostEqual(row['phase_observed_duration_s'], .25)
        first_retreat = state_row({}, rows, {'index': 61}, 'retreat_wrist_peak',
                                  'final_retreat.csv', 'straight', 'retreat', dt=1/240)
        self.assertEqual(first_retreat['phase_executed_sample_count'], 1)
        self.assertAlmostEqual(first_retreat['phase_observed_duration_s'], 1/240)

    def test_reference_initialization_has_no_trajectory_step_support(self):
        rows = [{'phase': 'approach', 'reference_step': i, 'time_s': i/240}
                for i in range(3)]
        initial = state_row({}, rows, {'index': 0}, 'reference_terminal', 'insertion.csv',
                            'reference', 'reference', dt=1/240)
        self.assertIs(initial['state_observed'], True)
        self.assertIs(initial['phase_executed'], False)
        self.assertIs(initial['sample_is_reference_initialization'], True)
        self.assertIs(initial['sample_is_copied_recovery_start'], False)
        self.assertEqual(initial['phase_observed_duration_s'], 0.)
        actual = state_row({}, rows, {'index': 2}, 'reference_terminal', 'insertion.csv',
                           'reference', 'reference', dt=1/240)
        self.assertEqual(actual['phase_executed_sample_count'], 2)
        self.assertAlmostEqual(actual['phase_observed_duration_s'], 2/240)

    def test_export_reads_only_complete_attempts_and_marks_missing_profiles(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            study = root/'study'; study.mkdir()
            case = study/'finished'; case.mkdir()
            (case/'insertion.csv').write_text(
                'time_s,phase,depth_gate_passed,wrist_force_n,normal_load_n\n'
                '0,insert,False,0.1,0\n'
                '0.01,settle,True,0.2,0\n')
            manifest = {'physics_hz': 100, 'status': 'running',
                        'case_plan': {'cases': [{}, {}]},
                        'attempts': [{'case_id': 'finished', 'folder': 'finished', 'status': 'complete'},
                                     {'case_id': 'still_running', 'folder': 'unreadable', 'status': 'running'}]}
            (study/'study.json').write_text(json.dumps(manifest))
            output = root/'review'
            summary = export_states(study, output)
            self.assertEqual(summary['completed_attempts'], 1)
            self.assertEqual(summary['state_rows'], 11)
            self.assertEqual(summary['observed_state_rows'], 3)
            self.assertFalse(summary['all_planned_cases_included'])
            with (output/'mechanics_states.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            gate = next(row for row in rows if row['state'] == 'depth_gate')
            self.assertEqual(gate['depth_gate_passed'], 'true')
            self.assertEqual(gate['sample_index'], '1')
            missing = next(row for row in rows if row['policy'] == 'realign')
            self.assertEqual(missing['state_observed'], 'false')
            self.assertEqual(missing['phase_executed'], '')
            self.assertEqual(missing['phase_observed_duration_s'], '')
            self.assertEqual(missing['wrist_force_n'], '')
            self.assertEqual(missing['missing_reason'], 'policy_profile_unavailable')


if __name__ == '__main__':
    unittest.main()
