import copy
import hashlib
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from research.forge_budget import (SCHEMA, ANGLE6_STUDY_ID, classify, evidence,
                                   load_plan, make_plan, make_angle6_plan, reference_eligible)
from research.forge_mechanics_plan import MechanicsState, make_case


def sample(index, force=0., torque=0., phase='retreat', dt=.05):
    return dict(time_s=10. + index * dt, physics_time_s=10. + index * dt,
                recovery_time_s=index * dt, phase=phase, wrist_force_n=force,
                wrist_torque_nm=torque, depth_mm=18. - index * .1,
                grasp_slip_mm=.01, grasp_slip_deg=.02)


def reference(**updates):
    result = dict(reference_complete=True, numerically_valid=True, grasp_retained=True,
                  reference_within_budget=True, depth_gate_passed=True,
                  force_budget_exceeded=False, torque_budget_exceeded=False)
    result.update(updates)
    return result


def recovery(**updates):
    result = dict(numerically_valid=True, grasp_retained=True,
                  force_budget_exceeded=False, torque_budget_exceeded=False,
                  label_eligible=True, safe_recovery=True, cleared=True,
                  recovery_censored=False, reason='cleared')
    result.update(updates)
    return result


def record(ref=None, rec=None):
    return dict(reference={'metrics': reference() if ref is None else ref},
                recovery={'metrics': recovery() if rec is None else rec})


class BudgetPlanTests(unittest.TestCase):
    def test_original_plan_json_bytes_remain_unchanged(self):
        path = Path(__file__).resolve().parents[1]/'experiments/forge_budget_v1.json'
        expected_sha = 'a5a5904a717538dd4c8bc196c79572fed9acc91c5525d33d186441a5a8c285b0'
        generated = (json.dumps(make_plan(),indent=2)+'\n').encode()
        self.assertEqual(hashlib.sha256(generated).hexdigest(),expected_sha)
        self.assertEqual(path.read_bytes(),generated)
        self.assertNotIn('study_id',make_plan())
        self.assertEqual(load_plan(path),make_plan())

    def test_bounded_conditions_order_and_identity(self):
        plan = make_plan()
        self.assertEqual(plan['schema'], SCHEMA)
        self.assertEqual(plan['policies'], ['straight', 'realign'])
        expected = [(18., (1., 1.), 0., 4.), (18., (1., 1.), 1., 4.),
                    (18., (1., 1.), 1.5, 4.), (18., (1., 1.), 1.5, 5.),
                    (12., (1., 1.), 1.5, 4.), (18., (.5, .5), 1.5, 4.),
                    (18., (1., 1.), 2., 4.), (18., (1., 1.), 1.5, 3.)]
        self.assertEqual(len(plan['conditions']), len(expected))
        ids = []
        for condition, (depth, pair, angle, budget) in zip(plan['conditions'], expected):
            case = make_case(depth, pair, angle)
            self.assertEqual(condition['case'], case)
            self.assertEqual(condition['force_budget_n'], budget)
            self.assertEqual(condition['torque_budget_nm'], 1.)
            self.assertEqual(condition['physics_hz'], 240)
            self.assertEqual(condition['seed'], 20260913)
            self.assertEqual(condition['condition_id'], f'{case["case_id"]}__b{int(budget):02d}')
            ids.append(condition['condition_id'])
        self.assertEqual(len(ids), len(set(ids)))
        self.assertEqual(plan['budget_definition']['termination_comparison'], '>')
        self.assertTrue(plan['budget_definition']['equality_allowed'])

    def test_exact_plan_roundtrip_and_mutation_rejection(self):
        with TemporaryDirectory() as directory:
            path = Path(directory)/'plan.json'
            plan = make_plan(); path.write_text(json.dumps(plan))
            self.assertEqual(load_plan(path), plan)
            changes = []
            for key, value in [('force_budget_n', 20.), ('physics_hz', True), ('seed', 1)]:
                changed = copy.deepcopy(plan); changed['conditions'][0][key] = value; changes.append(changed)
            changed = copy.deepcopy(plan); changed['conditions'].reverse(); changes.append(changed)
            changed = copy.deepcopy(plan); changed['conditions'][0]['case']['hole_static_friction'] = .75; changes.append(changed)
            changed = copy.deepcopy(plan); changed['extra'] = True; changes.append(changed)
            for changed in changes:
                path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):load_plan(path)
            path.write_text('{"schema":"Forge-budget-v1","schema":"Forge-budget-v1"}')
            with self.assertRaisesRegex(ValueError, 'Duplicate'):load_plan(path)

    def test_plan_calls_do_not_share_mutable_cases(self):
        first = make_plan(); first['conditions'][0]['case']['target_depth_mm'] = 6.
        self.assertEqual(make_plan()['conditions'][0]['case']['target_depth_mm'], 18.)


class BudgetAngle6PlanTests(unittest.TestCase):
    def test_exact_matrix_keeps_physics_and_uses_unique_command_angle_ids(self):
        original = make_plan(); plan = make_angle6_plan()
        path=Path(__file__).resolve().parents[1]/'experiments/forge_budget_angle6_v1.json'
        self.assertEqual(path.read_text(),json.dumps(plan,indent=2)+'\n')
        self.assertEqual(load_plan(path),plan)
        self.assertEqual(plan['schema'],SCHEMA)
        self.assertEqual(plan['study_id'],ANGLE6_STUDY_ID)
        self.assertEqual(plan['policies'],original['policies'])
        self.assertEqual(plan['budget_definition'],original['budget_definition'])
        self.assertEqual([c['case']['tilt_amplitude_deg'] for c in plan['conditions']],[1.5,2.,3.,4.,5.,6.])
        anchor = original['conditions'][2]
        self.assertEqual(plan['conditions'][0],anchor)
        identities = [c['condition_id'] for c in plan['conditions']]
        self.assertEqual(len(set(identities)),6)
        self.assertEqual(identities[-1],'m_d18_fs100_fd100_a06__b04')
        for condition in plan['conditions']:
            for key in ('force_budget_n','torque_budget_nm','physics_hz','seed'):
                self.assertEqual(condition[key],anchor[key])
            changed = {'case_id','tilt_amplitude_deg','final_pitch_deg','path_group_id'}
            self.assertEqual({k:v for k,v in condition['case'].items() if k not in changed},
                             {k:v for k,v in anchor['case'].items() if k not in changed})
        self.assertEqual([c['role'] for c in plan['conditions']],['candidate']+['angle_extension']*5)

    def test_selection_provenance_is_explicit_and_bound_to_original_plan(self):
        plan = make_angle6_plan(); provenance = plan['selection_provenance']
        self.assertEqual(provenance['maximum_command_tilt_deg'],6.)
        self.assertEqual(provenance['anchor_condition_id'],make_plan()['conditions'][2]['condition_id'])
        self.assertEqual(provenance['source_plan_canonical_sha256'],
                         'dd4ca9c20b16ebdaa54c513f06f16f9d202bed63f5484137537ea4c3e1115690')
        self.assertFalse(provenance['independent_random_replications'])
        self.assertIn('allowlist',provenance['parameter_validation_change'])
        plan['conditions'][0]['case']['target_depth_mm'] = 6.
        plan['budget_definition']['equality_allowed'] = False
        self.assertEqual(make_angle6_plan()['conditions'][0]['case']['target_depth_mm'],18.)
        self.assertEqual(make_plan()['conditions'][2]['case']['target_depth_mm'],18.)
        self.assertTrue(make_plan()['budget_definition']['equality_allowed'])

    def test_roundtrip_and_reject_modified_schema_provenance_order_range_and_types(self):
        with TemporaryDirectory() as directory:
            path=Path(directory)/'plan.json'; plan=make_angle6_plan()
            path.write_text(json.dumps(plan)); self.assertEqual(load_plan(path),plan)
            mutations=[]
            for key,value in [('schema','other'),('study_id','Forge-budget-v1')]:
                modified=copy.deepcopy(plan);modified[key]=value;mutations.append(modified)
            modified=copy.deepcopy(plan);del modified['selection_provenance'];mutations.append(modified)
            modified=copy.deepcopy(plan);modified['selection_provenance']['maximum_command_tilt_deg']=7.;mutations.append(modified)
            modified=copy.deepcopy(plan);modified['conditions'].reverse();mutations.append(modified)
            modified=copy.deepcopy(plan);modified['conditions'].append(copy.deepcopy(modified['conditions'][0]));mutations.append(modified)
            for value in (6,True,'6.0',6.1,7.,-6.,float('nan'),float('inf')):
                modified=copy.deepcopy(plan);modified['conditions'][-1]['case']['tilt_amplitude_deg']=value;mutations.append(modified)
            for key,value in [('force_budget_n',4),('physics_hz',240.),('seed',20260913.)]:
                modified=copy.deepcopy(plan);modified['conditions'][0][key]=value;mutations.append(modified)
            for modified in mutations:
                path.write_text(json.dumps(modified))
                with self.subTest(modified=modified):
                    with self.assertRaises(ValueError):load_plan(path)
            # Duplicate keys are invalid even if their repeated values agree.
            raw=json.dumps(plan).replace('"maximum_command_tilt_deg": 6.0',
                '"maximum_command_tilt_deg": 6.0, "maximum_command_tilt_deg": 6.0')
            path.write_text(raw)
            with self.assertRaisesRegex(ValueError,'Duplicate'):load_plan(path)

    def test_six_degree_state_uses_existing_depth_gate_and_time_ramp(self):
        state=MechanicsState(make_angle6_plan()['conditions'][-1]['case'])
        start=state.insertion_end_time_s
        self.assertEqual(state.command(start)['pitch_deg'],0.)
        state.observe(start,18.,physics_step=2272)
        self.assertFalse(state.depth_gate_passed)
        gate=start+.2;state.observe(gate,18.,physics_step=2320)
        self.assertTrue(state.depth_gate_passed)
        self.assertEqual(state.command(gate)['pitch_deg'],0.)
        middle=state.command(gate+.5);end=state.command(gate+1.)
        self.assertEqual(middle['phase'],'tilt');self.assertEqual(middle['pitch_deg'],3.)
        self.assertEqual(end['phase'],'hold');self.assertEqual(end['pitch_deg'],6.)
        self.assertEqual(end['depth_mm'],18.)
        for angle in (6.1,7.,-6.,True,'6',float('nan'),float('inf')):
            with self.subTest(angle=angle):
                with self.assertRaises(ValueError):make_case(18.,(1.,1.),angle)


class BudgetEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.protocol = SimpleNamespace(force_budget_n=4., torque_budget_nm=1.)

    def test_equality_is_allowed_and_next_float_crosses(self):
        equal = evidence([sample(0, 4., 1.)], self.protocol, .05, 'recovery')
        self.assertFalse(equal['budget_exceeded'])
        self.assertIsNone(equal['first_budget_crossing'])
        self.assertEqual(equal['force_margin_n'], 0.)
        rows = [sample(0, 4., 1.), sample(1, math.nextafter(4., math.inf), 1.)]
        result = evidence(rows, self.protocol, .05, 'recovery')
        crossing = result['first_budget_crossing']
        self.assertTrue(result['force_budget_exceeded'])
        self.assertFalse(result['torque_budget_exceeded'])
        self.assertEqual(crossing['index'], 1)
        self.assertTrue(crossing['executed_step'])
        self.assertGreater(crossing['force_overshoot_n'], 0.)
        self.assertLess(result['force_margin_n'], 0.)

    def test_initial_excess_is_observed_but_never_executed(self):
        result = evidence([sample(0, 5., 1.2, 'stop')], self.protocol, .05, 'recovery')
        self.assertEqual(result['sample_count'], 1)
        self.assertEqual(result['executed_step_count'], 0)
        self.assertEqual(result['executed_duration_s'], 0.)
        crossing = result['first_budget_crossing']
        self.assertTrue(crossing['initial_state_already_over_budget'])
        self.assertFalse(crossing['executed_step'])
        self.assertEqual(crossing['force_overshoot_n'], 1.)
        self.assertAlmostEqual(crossing['torque_overshoot_nm'], .2)
        self.assertFalse(result['phase_metrics']['stop']['phase_executed'])
        self.assertEqual(result['phase_metrics']['stop']['max_wrist_force_n'], 5.)
        self.assertIsNone(result['phase_metrics']['stop']['peak_wrist_force_mean100ms_n'])

    def test_first_crossing_retains_both_channels_and_state(self):
        rows = [sample(0), sample(1, 3., 1.1), sample(2, 6., 0.)]
        result = evidence(rows, self.protocol, .05, 'recovery')
        self.assertEqual(result['first_budget_crossing']['index'], 1)
        self.assertEqual(result['first_budget_crossing']['depth_mm'], 17.9)
        self.assertEqual(result['first_budget_crossing']['grasp_slip_mm'], .01)
        self.assertEqual(result['max_wrist_force_n'], 6.)
        self.assertTrue(result['force_budget_exceeded'])
        self.assertTrue(result['torque_budget_exceeded'])
        self.assertFalse(result['first_budget_crossing']['force_budget_exceeded'])

    def test_windows_exclude_initial_and_break_at_phase_and_time_gaps(self):
        rows = [sample(0, 100., phase='stop'), sample(1, 2., phase='stop'),
                sample(2, 4., phase='stop'), sample(3, 8.), sample(4, 10.)]
        rows[4]['recovery_time_s'] = .3
        result = evidence(rows, self.protocol, .05, 'recovery')
        stop, retreat = result['phase_metrics']['stop'], result['phase_metrics']['retreat']
        self.assertEqual(stop['peak_wrist_force_mean100ms_n'], 3.)
        self.assertEqual(stop['complete_force_mean100ms_windows'], 1)
        self.assertEqual(stop['executed_step_count'], 2)
        self.assertIsNone(retreat['peak_wrist_force_mean100ms_n'])
        rows[4]['recovery_time_s'] = .2
        self.assertEqual(evidence(rows, self.protocol, .05, 'recovery')['phase_metrics']['retreat']['peak_wrist_force_mean100ms_n'], 9.)

    def test_reference_windows_use_reference_clock(self):
        rows = [sample(0), sample(1, 1.), sample(2, 3.)]
        for row in rows:row['recovery_time_s'] = 0.
        result = evidence(rows, vars(self.protocol), .05, 'reference')
        self.assertEqual(result['phase_metrics']['retreat']['peak_wrist_force_mean100ms_n'], 2.)

    def test_missing_nonfinite_and_negative_signals_cannot_certify_budget(self):
        for bad in (None, float('nan'), float('inf'), -1., True):
            rows = [sample(0), sample(1)]; rows[1]['wrist_force_n'] = bad
            result = evidence(rows, self.protocol, .05, 'recovery')
            self.assertFalse(result['budget_signals_finite'])
            self.assertEqual(result['budget_signals_invalid_indices'], [1])
            self.assertIsNone(result['phase_metrics']['retreat']['peak_wrist_force_mean100ms_n'])
        empty = evidence([], self.protocol, .05)
        self.assertEqual(empty['executed_step_count'], 0)
        self.assertFalse(empty['budget_signals_finite'])
        self.assertIsNone(empty['max_wrist_force_n'])

    def test_invalid_settings_rejected(self):
        for dt in (0., -1., float('nan'), True):
            with self.assertRaises(ValueError):evidence([], self.protocol, dt)
        for budget in (0., -1., float('inf'), True):
            with self.assertRaises(ValueError):evidence([], dict(force_budget_n=budget, torque_budget_nm=1.), .05)
        with self.assertRaises(ValueError):evidence([], self.protocol, .05, 'retreat')


class BudgetClassificationTests(unittest.TestCase):
    def test_reference_requires_every_acceptance_gate(self):
        self.assertTrue(reference_eligible(reference()))
        for key in ('reference_complete', 'numerically_valid', 'grasp_retained',
                    'reference_within_budget', 'depth_gate_passed'):
            for value in (False, None, 1):
                self.assertFalse(reference_eligible(reference(**{key: value})))
        self.assertFalse(reference_eligible(reference(force_budget_exceeded=True)))
        self.assertFalse(reference_eligible(reference(budget_signals_finite=False)))

    def test_known_complete_clearance_is_safe(self):
        result = classify(record())
        self.assertEqual(result['outcome'], 'clear_safe')
        self.assertIs(result['recovery_safe'], True)
        self.assertIs(result['task_safe'], True)

    def test_reference_budget_or_grasp_failure_does_not_label_unexecuted_recovery(self):
        for updates, outcome in [(dict(force_budget_exceeded=True, reference_within_budget=False), 'reference_budget_exceeded'),
                                 (dict(grasp_retained=False), 'reference_grasp_limit')]:
            value = dict(reference={'metrics': reference(reference_complete=False, **updates)}, recovery=None)
            result = classify(value)
            self.assertEqual(result['outcome'], outcome)
            self.assertIsNone(result['recovery_safe'])
            self.assertIs(result['task_safe'], False)

    def test_recovery_crossing_has_priority_over_clearance_and_preserves_grasp_flag(self):
        result = classify(record(rec=recovery(force_budget_exceeded=True, grasp_retained=False)))
        self.assertEqual(result['outcome'], 'recovery_budget_exceeded')
        self.assertIs(result['task_safe'], False)
        self.assertIs(result['recovery_safe'], False)
        self.assertIs(result['flags']['recovery_grasp_retained'], False)
        result = classify(record(rec=recovery(grasp_retained=False)))
        self.assertEqual(result['outcome'], 'recovery_grasp_limit')

    def test_numerical_unknown_overrides_coincident_force_and_grasp_failures(self):
        for where in ('reference', 'recovery'):
            value = record()
            value[where]['metrics'].update(numerically_valid=False, force_budget_exceeded=True, grasp_retained=False)
            result = classify(value)
            self.assertEqual(result['outcome'], 'numerical_unknown')
            self.assertIsNone(result['task_safe'])
            self.assertIsNone(result['recovery_safe'])
            self.assertIs(result['flags'][where+'_force_budget_exceeded'], True)
        result = classify(record(rec=recovery(budget_signals_finite=False)))
        self.assertEqual(result['outcome'], 'numerical_unknown')

    def test_censored_timeout_and_missing_recovery_are_unknown(self):
        for updates in (dict(recovery_censored=True), dict(cleared=False),
                        dict(reason='time_budget_exhausted'), dict(label_eligible=False)):
            result = classify(record(rec=recovery(**updates)))
            self.assertEqual(result['outcome'], 'timeout_unknown')
            self.assertIsNone(result['task_safe'])
            self.assertIsNone(result['recovery_safe'])
        value = record(); value['recovery'] = None
        self.assertEqual(classify(value)['outcome'], 'timeout_unknown')
        value = record(ref=reference(reference_complete=False))
        self.assertEqual(classify(value)['outcome'], 'timeout_unknown')


if __name__ == '__main__':
    unittest.main()
