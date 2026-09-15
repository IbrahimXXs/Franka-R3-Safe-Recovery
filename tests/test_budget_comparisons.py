"""Behavior tests for closed, common-prefix budget comparison evidence."""
import copy
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_budget import make_plan, make_angle6_plan
from research.forge_protocol import protocol
from simulation.analyze_budget_comparisons import analyze_study, compare_entries, write_review
from tests.test_forge_protocol import sample


def observations(count=4, *, recovery=False):
    result = []
    for i in range(count):
        row = sample(i)
        row.update(time_s=i*.01, phase='retreat' if recovery else 'settle',
                   reference_step=10 if recovery else i,
                   command_depth_mm=18., command_pitch_deg=0.,
                   command_offset_x_mm=0., command_offset_y_mm=0.,
                   command_withdrawal_mm=i*.01 if recovery else 0.,
                   command_withdrawal_speed_mm_s=1. if recovery else 0.)
        result.append(row)
    return result


def entry(condition_index=2, *, reference_count=4, recovery_count=4):
    condition = make_plan()['conditions'][condition_index]
    p = asdict(protocol()); p['force_budget_n'] = condition['force_budget_n']
    return dict(condition=condition, status='complete',
                reference_rows=observations(reference_count),
                recovery_rows=observations(recovery_count,recovery=True),
                record=dict(schema='Forge-budget-branch-v1', status='complete',
                    condition=condition, policy='straight', sources={},
                    protocol=p.copy(), effective_protocol=p.copy(),
                    reference=dict(metrics=dict(reference_complete=True, numerically_valid=True,
                        grasp_retained=True, depth_gate_passed=True, depth_gate_step=1,
                        termination_reason='completed', terminal_depth_mm=18.,
                        max_wrist_force_n=1., max_normal_load_n=2.)),
                    recovery=dict(metrics=dict(reason='cleared', recovery_censored=False,
                        numerically_valid=True, grasp_retained=True, max_wrist_force_n=1.)),
                    outcome=dict(outcome='clear_safe', task_safe=True,recovery_safe=True)))


class BudgetComparisonsTests(unittest.TestCase):
    def test_angle_extension_keeps_all_planned_comparisons_without_opening_active_files(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            plan = make_angle6_plan()
            manifest = dict(schema=plan['schema'], case_plan=plan, sources={}, status='running',
                            attempts=[dict(condition_id=plan['conditions'][-1]['condition_id'],
                                           policy='straight', status='running', folder='not-closed')])
            (directory/'study.json').write_text(json.dumps(manifest))
            result = analyze_study(directory)
            self.assertEqual(result['study_id'], 'Forge-budget-angle6-v1')
            self.assertEqual(result['planned_straight_conditions'], 6)
            self.assertEqual(result['closed_straight_conditions'], 0)
            self.assertEqual([c['b']['tilt_amplitude_deg'] for c in result['comparisons']], [2.,3.,4.,5.,6.])
            self.assertTrue(all(c['a']['tilt_amplitude_deg'] == 1.5 and
                                c['status'] == 'pending_or_incomplete' and
                                c['observed_prefixes_matched'] is None for c in result['comparisons']))

    def test_angle_comparison_uses_actual_gate_and_keeps_later_failures(self):
        a,b = entry(0),entry(6)
        b['reference_rows'][2]['tip_x_mm'] += 1.
        a['reference_rows'][1]['tilt_triggered'] = False
        b['reference_rows'][1]['tilt_triggered'] = True
        b['record']['reference']['metrics'].update(reference_complete=False,
            grasp_retained=False, termination_reason='grasp_lost')
        b['record']['outcome'].update(outcome='reference_grasp_limit',task_safe=False,recovery_safe=None)
        result = compare_entries(a,b,'angle_depth_gate_prefix')
        self.assertEqual(result['compared_samples_a'],2)
        self.assertEqual(result['compared_samples_b'],2)
        self.assertTrue(result['matching_evidence_eligible'])
        self.assertFalse(result['prefix_audit']['replay_prefix_equal'])
        self.assertEqual(result['first_recorded_value_difference'],
                         dict(index=1,fields={'tilt_triggered':dict(a=False,b=True)}))
        self.assertFalse(result['b']['reference_complete'])
        self.assertEqual(result['b']['outcome'],'reference_grasp_limit')

    def test_different_gate_timing_is_not_hidden_by_common_prefix_trimming(self):
        a,b = entry(0),entry(1)
        b['record']['reference']['metrics']['depth_gate_step'] = 2
        result = compare_entries(a,b,'angle_depth_gate_prefix')
        self.assertEqual((result['compared_samples_a'],result['compared_samples_b']),(2,3))
        self.assertFalse(result['observed_prefixes_matched'])
        b['record']['reference']['metrics']['depth_gate_step'] = 50
        self.assertEqual(compare_entries(a,b,'angle_depth_gate_prefix')['status'],'depth_gate_unavailable')
        b['record']['reference']['metrics']['depth_gate_passed'] = False
        self.assertEqual(compare_entries(a,b,'angle_depth_gate_prefix')['status'],'depth_gate_not_reached')

    def test_budget_reference_compares_only_shared_observations(self):
        a,b = entry(7,reference_count=2),entry(2)
        a['reference_rows'][1]['wrist_force_n'] = b['reference_rows'][1]['wrist_force_n'] = 3.2
        a['record']['reference']['metrics'].update(reference_complete=False,termination_reason='operational_budget_exceeded')
        result = compare_entries(a,b,'budget_reference_prefix')
        self.assertEqual((result['samples_a'],result['samples_b'],result['compared_samples_b']),(2,4,2))
        self.assertTrue(result['observed_prefixes_matched'])
        self.assertTrue(result['shorter_stopped_at_first_budget_crossing'])
        self.assertTrue(result['crossing_counterpart']['wrist_force_equal'])
        self.assertFalse(result['unobserved_future_imputed'])
        self.assertFalse(result['a']['reference_complete'])
        self.assertTrue(result['b']['reference_complete'])

    def test_recovery_extra_commands_are_checked_even_if_physical_states_match(self):
        a,b = entry(2,recovery_count=2),entry(3)
        b['recovery_rows'][1]['command_withdrawal_mm'] += .001
        result = compare_entries(a,b,'budget_recovery_prefix')
        self.assertTrue(result['prefix_audit']['replay_prefix_matched'])
        self.assertFalse(result['observed_prefixes_matched'])
        self.assertEqual(result['first_additional_command_mismatch']['field'],'command_withdrawal_mm')
        self.assertFalse(result['complete_recovery_equality_claimed'])

    def test_recovery_crossing_preserves_equality_boundary_and_counterpart(self):
        a,b = entry(2,recovery_count=3),entry(3,recovery_count=5)
        for branch in (a,b):
            branch['recovery_rows'][1]['wrist_force_n'] = 4.
            branch['recovery_rows'][2]['wrist_force_n'] = 4.2
        a['record']['recovery']['metrics'].update(reason='operational_budget_exceeded',recovery_censored=True)
        result = compare_entries(a,b,'budget_recovery_prefix')
        self.assertTrue(result['matching_evidence_eligible'])
        self.assertTrue(result['shorter_stopped_at_first_budget_crossing'])
        self.assertEqual(result['crossing_counterpart']['index'],2)
        self.assertEqual(result['crossing_counterpart']['longer']['wrist_force_n'],4.2)
        self.assertEqual(result['compared_samples_b'],3)
        self.assertFalse(result['complete_recovery_equality_claimed'])
        # An earlier crossing followed by another executed sample is not a correct first-crossing stop.
        a['recovery_rows'][0]['wrist_force_n'] = 4.1
        self.assertFalse(compare_entries(a,b,'budget_recovery_prefix')['shorter_stopped_at_first_budget_crossing'])

    def test_complete_reference_and_full_prefix_match_required_for_recovery_evidence(self):
        a,b = entry(2),entry(3)
        b['reference_rows'][0]['tip_x_mm'] += 1.
        result = compare_entries(a,b,'budget_recovery_prefix')
        self.assertTrue(result['observed_prefixes_matched'])
        self.assertFalse(result['matching_evidence_eligible'])
        b['record']['reference']['metrics']['reference_complete'] = False
        result = compare_entries(a,b,'budget_recovery_prefix')
        self.assertEqual(result['status'],'reference_incomplete_recovery_not_compared')
        self.assertIsNone(result['observed_prefixes_matched'])

    def test_numerical_and_configuration_flags_remain_separate_from_state_matching(self):
        a,b = entry(2),entry(3)
        for branch in (a,b):branch['reference_rows'][1]['min_separation_mm'] = -2.
        b['record']['seed'] = 999
        result = compare_entries(a,b,'budget_reference_prefix')
        self.assertTrue(result['observed_prefixes_matched'])
        self.assertFalse(result['compared_prefixes_numerically_valid'])
        self.assertFalse(result['matching_evidence_eligible'])
        self.assertIn('seed',result['configuration_differences'])

    def test_depth_and_friction_are_descriptive(self):
        for kind,index in (('depth_description',4),('friction_description',5)):
            with self.subTest(kind=kind):
                a,b = entry(index),entry(2)
                a['reference_rows'][0]['tip_x_mm'] += 1.
                result = compare_entries(a,b,kind)
                self.assertEqual(result['status'],'descriptive_only')
                self.assertIsNone(result['observed_prefixes_matched'])

    def test_closed_sha_validation_and_running_branch_is_never_opened(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)/'study'; root.mkdir()
            a = entry(0); closed = root/'closed'; closed.mkdir()
            record = copy.deepcopy(a['record']); record['artifact_sha256'] = {}
            for segment in ('reference','recovery'):
                filename = segment+'.csv'; path = closed/filename
                with path.open('w',newline='') as stream:
                    rows = a[segment+'_rows']; writer = csv.DictWriter(stream,fieldnames=list(rows[0]))
                    writer.writeheader(); writer.writerows(rows)
                record[segment]['trajectory'] = filename
                record['artifact_sha256'][filename] = hashlib.sha256(path.read_bytes()).hexdigest()
            record_path = closed/'run.json'; record_path.write_text(json.dumps(record))
            plan = make_plan()
            manifest = dict(schema=plan['schema'],case_plan=plan,sources={},status='running',attempts=[
                dict(condition_id=a['condition']['condition_id'],policy='straight',status='complete',folder='closed',
                     run_sha256=hashlib.sha256(record_path.read_bytes()).hexdigest()),
                dict(condition_id=plan['conditions'][1]['condition_id'],policy='straight',status='running',
                     folder='intentionally_absent')])
            (root/'study.json').write_text(json.dumps(manifest))
            result = analyze_study(root)
            self.assertEqual(result['closed_straight_conditions'],1)
            self.assertEqual(len(result['comparisons']),9)
            self.assertTrue(all(c['status']=='pending_or_incomplete' for c in result['comparisons']))
            destination = Path(temp)/'review'; write_review(result,destination)
            with (destination/'parameter_comparisons.csv').open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))),9)
            with self.assertRaisesRegex(ValueError,'outside'):
                write_review(result,root/'review')
            with (closed/'reference.csv').open('a') as stream:stream.write('tampered\n')
            with self.assertRaisesRegex(ValueError,'SHA mismatch'):
                analyze_study(root)


if __name__ == '__main__':unittest.main()
