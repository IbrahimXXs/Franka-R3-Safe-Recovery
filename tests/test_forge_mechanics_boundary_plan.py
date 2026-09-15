"""Boundary-design compatibility checks; no simulator imports or execution."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_mechanics_plan import (
    MechanicsState, case_for_slot, load_plan, make_boundary_plan, make_case, make_plan, subset_plan,
)


LEGACY_PLAN_SHA256 = 'ee42bb7c28c1fa160fca7c2e4b3ffc70c99dbf236a09999477d42f1432fa1e33'


class MechanicsBoundaryPlanTests(unittest.TestCase):
    def test_original_24_case_plan_keeps_exact_structure_and_serialized_bytes(self):
        root = Path(__file__).resolve().parents[1]
        saved = (root / 'experiments/forge_mechanics_pilot.json').read_bytes()
        generated = (json.dumps(make_plan(), indent=2) + '\n').encode()
        self.assertEqual(generated, saved)
        self.assertEqual(hashlib.sha256(generated).hexdigest(), LEGACY_PLAN_SHA256)
        self.assertEqual(load_plan(root / 'experiments/forge_mechanics_pilot.json'), make_plan())
        self.assertEqual(len(make_plan()['cases']), 24)
        self.assertNotIn('design', make_plan())
        self.assertEqual({c['tilt_amplitude_deg'] for c in make_plan()['cases']}, {0., 2.})

    def test_boundary_has_control_then_unique_one_and_one_point_five_degree_cases(self):
        plan = make_boundary_plan()
        self.assertEqual(plan['design'], 'boundary_angles_v1')
        self.assertEqual(plan['attempts_per_case'], 1)
        self.assertEqual(plan['retries_per_case'], 0)
        self.assertEqual([c['tilt_amplitude_deg'] for c in plan['cases']], [0., 1., 1.5])
        self.assertEqual([c['case_id'] for c in plan['cases']], [
            'm_d18_fs100_fd100_a00', 'm_d18_fs100_fd100_a01', 'm_d18_fs100_fd100_a1p5'])
        self.assertEqual({c['target_depth_mm'] for c in plan['cases']}, {18.})
        self.assertEqual({(c['pair_static_friction'], c['pair_dynamic_friction']) for c in plan['cases']}, {(1., 1.)})
        self.assertEqual(len({c['split_group_id'] for c in plan['cases']}), 1)
        self.assertEqual(make_case(18., (1., 1.), 2.)['case_id'], 'm_d18_fs100_fd100_a02')
        for slot, case in enumerate(plan['cases']):
            self.assertEqual(case_for_slot(plan, slot), case)
        with self.assertRaisesRegex(ValueError, 'retries'):
            case_for_slot(plan, 1, 1)

    def test_boundary_load_rejects_missing_design_unknown_design_and_changed_condition(self):
        root = Path(__file__).resolve().parents[1]
        self.assertEqual(load_plan(root / 'experiments/forge_mechanics_boundary.json'), make_boundary_plan())
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'plan.json'
            invalid = []
            missing = make_boundary_plan(); missing.pop('design'); invalid.append(missing)
            unknown = make_boundary_plan(); unknown['design'] = 'boundary_angles_v2'; invalid.append(unknown)
            main = make_plan(); main['design'] = 'boundary_angles_v1'; invalid.append(main)
            for key, value in (('tilt_amplitude_deg', 2.), ('target_depth_mm', 12.),
                               ('pair_dynamic_friction', .5), ('tilt_ramp_duration_s', .5)):
                plan = make_boundary_plan(); plan['cases'][1][key] = value; invalid.append(plan)
            for value in invalid:
                path.write_text(json.dumps(value))
                with self.assertRaises(ValueError):
                    load_plan(path)

    def test_boundary_subsets_preserve_correct_base_provenance_and_source_slots(self):
        plan = make_boundary_plan()
        subset = subset_plan(plan, cases=[plan['cases'][2], plan['cases'][0]])
        self.assertEqual(subset['design'], 'boundary_angles_v1')
        self.assertEqual(subset['source_plan']['design'], 'boundary_angles_v1')
        self.assertEqual(subset['source_plan']['case_count'], 3)
        self.assertEqual([c['slot'] for c in subset['cases']], [0, 1])
        self.assertEqual([c['source_slot'] for c in subset['cases']], [0, 2])
        self.assertNotEqual(subset['source_plan']['sha256'],
                            subset_plan(make_plan(), effective_friction_pair=(1., 1.))['source_plan']['sha256'])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'subset.json'
            path.write_text(json.dumps(subset))
            self.assertEqual(load_plan(path), subset)
            changed = copy.deepcopy(subset); changed['source_plan']['case_count'] = 24
            path.write_text(json.dumps(changed))
            with self.assertRaises(ValueError):
                load_plan(path)

    def test_new_angles_use_same_actual_depth_gate_and_next_step_tilt_schedule(self):
        for angle in (1., 1.5):
            state = MechanicsState(make_case(18., (1., 1.), angle))
            end = state.insertion_end_time_s
            state.observe(end, 18.)
            state.observe(end + .1, 18.)
            applied = state.command(end + .2)
            self.assertEqual(applied['pitch_deg'], 0.)
            self.assertEqual(applied['phase'], 'settle')
            state.observe(end + .2, 18., physics_step=123)
            self.assertTrue(state.depth_gate_passed)
            self.assertEqual(state.depth_gate_step, 123)
            self.assertGreater(state.command(end + .2 + 1 / 240.)['pitch_deg'], 0.)
            self.assertAlmostEqual(state.command(end + .7)['pitch_deg'], angle / 2.)
            self.assertEqual(state.command(end + 1.2)['depth_mm'], 18.)
            self.assertAlmostEqual(state.command(end + 1.2)['pitch_deg'], angle)
            state.observe(end + 2.2, 18.)
            self.assertTrue(state.done)
            self.assertEqual(state.termination_reason, 'reference_complete')

    def test_new_angles_do_not_bypass_depth_or_grasp_gates(self):
        depth = MechanicsState(make_case(18., (1., 1.), 1.5))
        depth.observe(depth.settle_deadline_s, 17.7)
        self.assertEqual(depth.termination_reason, 'depth_gate_timeout')
        self.assertFalse(depth.depth_gate_passed)
        self.assertEqual(depth.command(depth.settle_deadline_s)['pitch_deg'], 0.)
        grasp = MechanicsState(make_case(18., (1., 1.), 1.))
        end = grasp.insertion_end_time_s
        grasp.observe(end, 18.)
        grasp.observe(end + .2, 18.)
        grasp.observe(end + .5, 18., grasp_retained=False)
        self.assertTrue(grasp.done)
        self.assertEqual(grasp.termination_reason, 'grasp_lost')


if __name__ == '__main__':
    unittest.main()
