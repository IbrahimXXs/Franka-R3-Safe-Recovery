import copy
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_gap_study import (
    TiltTriggerState, case_for_slot, command_misalignment, load_plan,
    make_case, make_plan, subset_plan,
)


class GapStudyTests(unittest.TestCase):
    def test_matrix_has_unique_cases_and_reuses_pilot_without_id_changes(self):
        pilot = make_plan(True)
        full = make_plan(False)
        self.assertEqual(len(pilot['cases']), 23)
        self.assertEqual(len(full['cases']), 125)
        self.assertEqual(full['cases'][:23], pilot['cases'])
        self.assertEqual(len({c['case_id'] for c in full['cases']}), 125)
        self.assertEqual([c['hole_diameter_mm'] for c in pilot['cases'][:5]],
                         [9., 8.786, 8.586, 8.386, 8.186])
        for c in full['cases']:
            self.assertAlmostEqual((c['hole_diameter_mm'] - c['peg_diameter_mm']) / 2.,
                                   c['radial_clearance_mm'])
        self.assertEqual({c['tilt_onset_mm'] for c in full['cases'][5:]}, {6., 10., 14.})
        self.assertEqual({c['final_pitch_deg'] for c in full['cases'][5:]},
                         {-.5, .5, -1., 1., -2., 2., -4., 4.})

    def test_first_actual_crossing_arms_once_and_stall_does_not_stop_ramp(self):
        case = make_case(.1, .5, 2.)
        state = TiltTriggerState()
        self.assertEqual(state.update(case, 0., 15., in_insertion=False)['pitch_deg'], 0.)
        self.assertFalse(state.triggered)
        self.assertEqual(state.update(case, 1., 9.99)['pitch_deg'], 0.)
        self.assertEqual(state.update(case, 2., 10.01)['pitch_deg'], 0.)
        self.assertEqual(state.trigger_depth_mm, 10.01)
        self.assertEqual(state.update(case, 2.5, 10.01)['pitch_deg'], 1.)
        self.assertEqual(state.update(case, 3., 9.)['pitch_deg'], 2.)
        self.assertTrue(state.ramp_completed)
        self.assertEqual(state.trigger_time_s, 2.)
        self.assertEqual(state.ramp_complete_time_s, 3.)
        self.assertEqual(state.update(case, 30., 0.)['pitch_deg'], 2.)
        self.assertEqual(state.as_dict()['tilt_trigger_depth_mm'], 10.01)

    def test_unreached_and_aligned_cases_never_trigger(self):
        for case in (make_case(.2), make_case(.2, .7, .5)):
            state = TiltTriggerState()
            for t in (0., 5., 10.):
                self.assertEqual(state.update(case, t, 13.99)['pitch_deg'], 0.)
            self.assertFalse(state.triggered)
            self.assertFalse(state.ramp_completed)

    def test_ramp_is_smooth_signed_bounded_and_replay_has_independent_state(self):
        case = make_case(.1, .3, 4., -1)
        self.assertEqual(command_misalignment(case)['pitch_deg'], 0.)
        self.assertAlmostEqual(command_misalignment(case, .25)['pitch_deg'], -.625)
        self.assertEqual(command_misalignment(case, -.01)['pitch_deg'], 0.)
        self.assertEqual(command_misalignment(case, 100.)['pitch_deg'], -4.)
        first, replay = TiltTriggerState(), TiltTriggerState()
        for t, depth in ((0., 0.), (1., 6.), (1.25, 6.), (2.1, 6.)):
            self.assertEqual(first.update(case, t, depth), replay.update(case, t, depth))
        self.assertEqual(first.as_dict(), replay.as_dict())
        with self.assertRaisesRegex(ValueError, 'monotonic'):
            first.update(case, 0., 0.)

    def test_retry_retains_physics_and_grouping(self):
        plan = make_plan(True)
        reference = copy.deepcopy(plan)
        retry = case_for_slot(plan, 8, 2)
        self.assertEqual(retry['trajectory_id'], f'{retry["case_id"]}_try02')
        self.assertEqual({k: v for k, v in retry.items() if k not in ('retry', 'trajectory_id')},
                         {k: v for k, v in plan['cases'][8].items() if k not in ('retry', 'trajectory_id')})
        self.assertEqual(plan, reference)

    def test_geometry_subsets_preserve_identity_and_validate_independently(self):
        plan = make_plan(True)
        subset = subset_plan(plan, radial_clearance_mm=.1)
        expected = [c for c in plan['cases'] if c['radial_clearance_mm'] == .1]
        self.assertEqual(len(subset['cases']), 7)
        for slot, (got, original) in enumerate(zip(subset['cases'], expected)):
            self.assertEqual(got, dict(original, slot=slot))
            self.assertEqual(got['source_slot'], original['slot'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'subset.json'
            path.write_text(json.dumps(subset))
            self.assertEqual(load_plan(path), subset)
            subset['source_plan']['sha256'] = 'wrong'
            path.write_text(json.dumps(subset))
            with self.assertRaises(ValueError):
                load_plan(path)

    def test_post_step_observation_then_next_command_has_one_step_ramp_progress(self):
        case = make_case(.1, .3, 2.)
        state = TiltTriggerState()
        state.update(case, 1., 6.01, physics_step=120)
        next_command = state.command(case, 1. + 1. / 120.)
        self.assertGreater(next_command['pitch_deg'], 0.)
        self.assertEqual(state.as_dict()['tilt_trigger_step'], 120)
        self.assertEqual(state.as_dict()['tilt_ramp_fraction'], 0.)

    def test_saved_plans_and_validation_reject_changed_geometry_or_timing(self):
        root = Path(__file__).resolve().parents[1]
        for pilot, name in ((True, 'pilot'), (False, 'full')):
            self.assertEqual(load_plan(root / f'experiments/forge_gap_{name}.json'), make_plan(pilot))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'plan.json'
            for key, value in (('hole_diameter_mm', 8.1), ('tilt_onset_mm', 9.),
                               ('tilt_ramp_duration_s', 2.), ('radial_clearance_mm', .05)):
                plan = make_plan(True)
                plan['cases'][5][key] = value
                path.write_text(json.dumps(plan))
                with self.assertRaises(ValueError):
                    load_plan(path)
            plan = make_plan(True)
            plan['schema'] = 'Forge-Phase2B-depth-drift-v1'
            path.write_text(json.dumps(plan))
            with self.assertRaisesRegex(ValueError, 'schema'):
                load_plan(path)


if __name__ == '__main__':
    unittest.main()
