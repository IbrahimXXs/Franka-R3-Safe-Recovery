import copy
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_mechanics_plan import (
    MechanicsState, case_for_slot, load_plan, make_case, make_plan, subset_plan,
)


def gated_state(angle=2., depth=6.):
    state = MechanicsState(make_case(depth, (.5, .5), angle))
    end = state.insertion_end_time_s
    state.observe(end, depth, physics_step=100)
    state.observe(end + .1, depth, physics_step=112)
    state.observe(end + .2, depth, physics_step=124)
    return state


class MechanicsPlanTests(unittest.TestCase):
    def test_matrix_uses_true_effective_pairs_and_all_controls_precede_tilts(self):
        plan = make_plan()
        cases = plan['cases']
        self.assertEqual(len(cases), 24)
        self.assertEqual(len({case['case_id'] for case in cases}), 24)
        self.assertTrue(all(case['tilt_amplitude_deg'] == 0 for case in cases[:12]))
        self.assertTrue(all(case['tilt_amplitude_deg'] == 2 for case in cases[12:]))
        self.assertEqual({case['target_depth_mm'] for case in cases}, {6., 12., 18.})
        self.assertEqual({(case['pair_static_friction'], case['pair_dynamic_friction']) for case in cases},
                         {(.5, .5), (.75, .75), (1., 1.), (1., .5)})
        for case in cases:
            self.assertEqual(case['radial_clearance_mm'], .2)
            self.assertEqual(case['hole_depth_mm'], 25.)
            for kind in ('static', 'dynamic'):
                self.assertEqual(case[f'peg_{kind}_friction'], .75)
                self.assertEqual((case[f'peg_{kind}_friction'] + case[f'hole_{kind}_friction']) / 2,
                                 case[f'pair_{kind}_friction'])
            self.assertAlmostEqual((case['target_depth_mm'] + 10.) / case['insertion_duration_s'], 3.75)
        for control, tilted in zip(cases[:12], cases[12:]):
            self.assertEqual(control['split_group_id'], tilted['split_group_id'])

    def test_canonical_plan_and_subsets_validate_and_cannot_enable_retries(self):
        root = Path(__file__).resolve().parents[1]
        plan = make_plan()
        self.assertEqual(load_plan(root / 'experiments/forge_mechanics_pilot.json'), plan)
        subset = subset_plan(plan, effective_friction_pair=(1., .5))
        self.assertEqual(len(subset['cases']), 6)
        self.assertEqual(subset['cases'][0]['source_slot'], 3)
        self.assertEqual([c['slot'] for c in subset['cases']], list(range(6)))
        selected = subset_plan(plan, cases=[plan['cases'][13], plan['cases'][1]])
        self.assertEqual([c['source_slot'] for c in selected['cases']], [1, 13])
        self.assertEqual(case_for_slot(subset, 0), subset['cases'][0])
        with self.assertRaisesRegex(ValueError, 'retries'):
            case_for_slot(subset, 0, 1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'plan.json'
            path.write_text(json.dumps(subset))
            self.assertEqual(load_plan(path), subset)
            for key, value in (('hole_static_friction', .75), ('settle_dwell_s', .1),
                               ('target_depth_mm', 20.), ('insertion_duration_s', 8.)):
                changed = copy.deepcopy(subset)
                changed['cases'][0][key] = value
                path.write_text(json.dumps(changed))
                with self.assertRaises(ValueError):
                    load_plan(path)
            subset['source_plan']['sha256'] = 'wrong'
            path.write_text(json.dumps(subset))
            with self.assertRaises(ValueError):
                load_plan(path)

    def test_insertion_schedule_is_aligned_and_depth_gate_uses_settle_observations(self):
        case = make_case(12., (.75, .75), 2.)
        state = MechanicsState(case)
        self.assertEqual(state.command(1.)['phase'], 'approach')
        self.assertEqual(state.command(1.)['depth_mm'], -10.)
        midpoint = 2. + case['insertion_duration_s'] / 2.
        self.assertEqual(state.command(midpoint)['phase'], 'insert')
        self.assertAlmostEqual(state.command(midpoint)['depth_mm'], 1.)
        self.assertEqual(state.command(midpoint)['pitch_deg'], 0.)
        end = state.insertion_end_time_s
        state.observe(end - .2, 12.)
        self.assertTrue(state.depth_attained)
        self.assertFalse(state.depth_gate_passed)
        state.observe(end, 12.)
        state.observe(end + .1, 12.)
        self.assertFalse(state.depth_gate_passed)
        applied = state.command(end + .2)
        self.assertEqual(applied['phase'], 'settle')
        self.assertEqual(applied['pitch_deg'], 0.)
        state.observe(end + .2, 12., physics_step=123)
        self.assertTrue(state.depth_gate_passed)
        self.assertEqual(state.depth_gate_step, 123)
        following = state.command(end + .2 + 1. / 120.)
        self.assertEqual(following['phase'], 'tilt')
        self.assertGreater(following['pitch_deg'], 0.)
        self.assertEqual(following['depth_mm'], 12.)

    def test_settle_requires_contiguous_tolerance_and_does_not_count_overshoot(self):
        state = MechanicsState(make_case(6., (1., 1.), 2.))
        end = state.insertion_end_time_s
        state.observe(end, 6.)
        state.observe(end + .1, 6.11)
        self.assertIsNone(state.settle_window_start_time_s)
        state.observe(end + .2, 5.9)
        state.observe(end + .3, 6.1)
        self.assertFalse(state.depth_gate_passed)
        state.observe(end + .4, 6.)
        self.assertTrue(state.depth_gate_passed)
        self.assertAlmostEqual(state.depth_gate_time_s, end + .4)

    def test_tilt_and_hold_have_fixed_depth_and_complete_only_after_observation(self):
        state = gated_state(depth=18.)
        start = state.depth_gate_time_s
        for elapsed, expected_pitch in ((0., 0.), (.25, .3125), (.5, 1.), (1., 2.), (2., 2.)):
            command = state.command(start + elapsed)
            self.assertEqual(command['depth_mm'], 18.)
            self.assertAlmostEqual(command['pitch_deg'], expected_pitch)
        state.observe(start + .5, 17.8)
        self.assertFalse(state.done)
        state.observe(start + 1., 17.8)
        self.assertTrue(state.tilt_ramp_completed)
        state.observe(start + 1.9, 17.8)
        self.assertFalse(state.command(start + 2.)['done'])
        state.observe(start + 2., 17.8)
        self.assertTrue(state.done)
        self.assertEqual(state.termination_reason, 'reference_complete')

    def test_zero_tilt_control_uses_same_sequence_timing(self):
        control, tilted = gated_state(0.), gated_state(2.)
        self.assertEqual(control.depth_gate_time_s, tilted.depth_gate_time_s)
        for state in (control, tilted):
            start = state.depth_gate_time_s
            self.assertEqual(state.command(start + .5)['phase'], 'tilt')
            self.assertEqual(state.command(start + 1.5)['phase'], 'hold')
            state.observe(start + 2., 6.)
            self.assertTrue(state.done)
            self.assertTrue(state.tilt_ramp_completed)
        self.assertEqual(control.command(control.depth_gate_time_s + 2.)['pitch_deg'], 0.)
        self.assertFalse(control.as_dict()['tilt_triggered'])
        self.assertTrue(control.as_dict()['tilt_sequence_started'])
        self.assertTrue(tilted.as_dict()['tilt_triggered'])

    def test_timeout_never_applies_tilt_and_boundary_dwell_can_pass(self):
        state = MechanicsState(make_case(6., (.5, .5), 2.))
        state.observe(state.insertion_end_time_s, 6.2)
        self.assertTrue(state.depth_attained)
        state.observe(state.settle_deadline_s, 6.2)
        self.assertTrue(state.done)
        self.assertFalse(state.depth_gate_passed)
        self.assertEqual(state.termination_reason, 'depth_gate_timeout')
        self.assertEqual(state.command(state.settle_deadline_s)['pitch_deg'], 0.)
        valid = MechanicsState(make_case(6., (.5, .5), 2.))
        valid.observe(valid.settle_deadline_s - .2, 6.)
        valid.observe(valid.settle_deadline_s - .1, 6.)
        valid.observe(valid.settle_deadline_s, 6.)
        self.assertTrue(valid.depth_gate_passed)
        self.assertFalse(valid.done)

    def test_numerical_or_grasp_failure_terminates_even_after_tilt_gate(self):
        for phase_time in (0.5, 1.5):
            for field, reason in (('numerically_valid', 'numerical_invalid'), ('grasp_retained', 'grasp_lost')):
                state = gated_state()
                time = state.depth_gate_time_s + phase_time
                state.observe(time, 6., **{field: False})
                self.assertTrue(state.done)
                self.assertEqual(state.termination_reason, reason)
        state = MechanicsState(make_case(6., (.5, .5), 2.))
        state.observe(2.5, -9., numerically_valid=False)
        self.assertFalse(state.depth_gate_passed)
        self.assertTrue(state.done)

    def test_replay_state_is_independent_and_observations_are_monotonic(self):
        case = make_case(6., (.5, .5), 2.)
        first, replay = MechanicsState(case), MechanicsState(case)
        end = first.insertion_end_time_s
        samples = [(0., -10.), (2., -10.), (end, 6.), (end + .1, 6.),
                   (end + .2, 6.), (end + .7, 6.), (end + 2.2, 6.)]
        for step, (time, depth) in enumerate(samples):
            self.assertEqual(first.command(time), replay.command(time))
            self.assertEqual(first.observe(time, depth, physics_step=step),
                             replay.observe(time, depth, physics_step=step))
        self.assertEqual(first.as_dict(), replay.as_dict())
        with self.assertRaisesRegex(ValueError, 'monotonic'):
            first.observe(0., -10.)
        with self.assertRaises(ValueError):
            MechanicsState(dict(case, settle_dwell_s=0.))


if __name__ == '__main__':
    unittest.main()
