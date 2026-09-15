import copy
import json
import unittest

from research.forge_mechanics_protocol import (
    aligned_control_passed, compare_prefixes, reference_quality, validate_resume,
)
from research.forge_protocol import protocol
from tests.test_forge_protocol import sample


def prefix():
    rows = [sample(i) for i in range(3)]
    for i, row in enumerate(rows):
        row.update(command_pitch_deg=0., command_offset_x_mm=0.,
                   command_offset_y_mm=0., reference_step=i)
    return rows


class MechanicsProtocolTests(unittest.TestCase):
    def test_matching_endpoint_does_not_hide_different_history(self):
        original = prefix(); replay = copy.deepcopy(original)
        replay[1]['tip_x_mm'] += .01
        result = compare_prefixes(original, replay, protocol())
        self.assertTrue(result['replay_matched'])
        self.assertFalse(result['replay_prefix_matched'])
        self.assertEqual(result['replay_prefix_first_mismatch_index'], 1)

    def test_phase_command_or_truncation_rejects_identical_physical_states(self):
        original = prefix()
        for key, value in (('phase', 'hold'), ('command_pitch_deg', 2.)):
            replay = copy.deepcopy(original); replay[1][key] = value
            self.assertFalse(compare_prefixes(original, replay, protocol())['replay_prefix_matched'])
        self.assertFalse(compare_prefixes(original, original[:-1], protocol())['replay_prefix_matched'])

    def test_record_equality_is_stricter_than_physical_matching(self):
        original = prefix(); replay = copy.deepcopy(original)
        replay[1]['tip_x_mm'] += .00001
        result = compare_prefixes(original, replay, protocol())
        self.assertTrue(result['replay_prefix_matched'])
        self.assertFalse(result['replay_prefix_equal'])

    def test_depth_alone_cannot_pass_control_gate(self):
        attempt = dict(status='complete', metrics=dict(reference_complete=True,
            numerically_valid=True, grasp_retained=True, reference_within_budget=True,
            depth_attained=True), final_retreat=dict(safe_recovery=True))
        self.assertTrue(aligned_control_passed(attempt))
        for key in attempt['metrics']:
            changed = copy.deepcopy(attempt); changed['metrics'][key] = False
            self.assertFalse(aligned_control_passed(changed))
        for recovery in (False, None):
            changed = copy.deepcopy(attempt); changed['final_retreat']['safe_recovery'] = recovery
            self.assertFalse(aligned_control_passed(changed))

    def test_numerical_invalidity_and_grasp_loss_remain_separate(self):
        rows = prefix(); rows[1]['grasp_slip_mm'] = .2
        result = reference_quality(rows, protocol(), 'grasp_lost')
        self.assertTrue(result['numerically_valid'])
        self.assertFalse(result['grasp_retained'])
        self.assertFalse(result['reference_complete'])
        rows[1]['grasp_slip_mm'] = 0.; rows[1]['min_separation_mm'] = -.2
        result = reference_quality(rows, protocol(), 'numerical_invalid')
        self.assertFalse(result['numerically_valid'])
        self.assertTrue(result['grasp_retained'])

    def test_resume_rejects_changed_physics_protocol_or_sources(self):
        saved = dict(study='Forge-mechanics-v1', physics_hz=240, seed=1,
                     case_plan={'cases': [1]}, protocol={'force_budget_n': 20, 'checkpoints_mm': (5., 10.)},
                     sources={'source.py': 'hash'}, radial_clearance_mm=.2,
                     recovery_policy_definition='fixed',
                     simulation_options={'stock_buffers': False, 'device': 'cuda:0'},
                     recovery_motion={'requested_speed_mm_s': 5.})
        validate_resume(saved, copy.deepcopy(saved))
        validate_resume(json.loads(json.dumps(saved)), saved)
        for key in saved:
            changed = copy.deepcopy(saved); changed[key] = None
            with self.assertRaisesRegex(ValueError, key):
                validate_resume(saved, changed)


if __name__ == '__main__':
    unittest.main()
