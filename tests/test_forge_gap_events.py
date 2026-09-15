"""Cross-module invariants for observed events, screening, and collection."""
import copy
from dataclasses import replace
import unittest

from research.forge_collection import accepted_slots, next_case
from research.forge_events import event_checkpoints, recovery_termination
from research.forge_gap_study import TiltTriggerState, make_case, make_plan, subset_plan
from research.forge_protocol import protocol
from research.phase2_protocol import recovery_label


def observed_reference():
    """Depth reaches 6 mm then stalls; time-based tilt still completes."""
    case = make_case(.1, .3, 2.)
    state = TiltTriggerState()
    rows = []
    for step in range(21):
        elapsed = step / 10
        command = state.command(case, elapsed)
        depth = float(min(step, 6))
        phase = 'initial' if step == 0 else 'insert'
        state.update(case, elapsed, depth, phase == 'insert', physics_step=step)
        rows.append(dict(
            time_s=elapsed, reference_step=step, phase=phase, depth_mm=depth,
            command_depth_mm=float(step), command_pitch_deg=command['pitch_deg'],
            min_separation_mm=0., force_norm_n=1., torque_norm_nm=.1,
            normal_load_n=1., wrist_force_n=2., wrist_torque_nm=.1,
            **state.as_dict(),
        ))
    return case, rows


class GapEventTests(unittest.TestCase):
    def test_events_refer_to_completed_causal_states_not_future_ramp_commands(self):
        _, rows = observed_reference()
        original = copy.deepcopy(rows)
        events = {cp['checkpoint_kind']: cp for cp in event_checkpoints(rows, protocol(), .1)}
        self.assertEqual(events['pre_tilt']['step'], 6)
        self.assertEqual(events['pre_tilt']['state']['command_pitch_deg'], 0.)
        self.assertEqual(events['first_stall']['step'], 11)
        self.assertEqual(events['ramp_complete']['step'], 16)
        self.assertEqual(events['ramp_complete']['state']['command_pitch_deg'], 2.)
        self.assertEqual(events['terminal']['step'], 20)
        self.assertEqual(len({cp['checkpoint_id'] for cp in events.values()}), 4)
        self.assertEqual({cp['depth_mm'] for cp in events.values()}, {6.})
        self.assertEqual(rows, original)
        rows[-1]['force_norm_n'] = 400.
        later = {cp['checkpoint_kind']: cp for cp in event_checkpoints(rows, protocol(), .1)}
        self.assertEqual(events['first_stall'], later['first_stall'])

    def test_effective_tight_clearance_screen_propagates_only_forward(self):
        _, rows = observed_reference()
        rows[8]['min_separation_mm'] = -.03
        tight = replace(protocol(), effective_radial_clearance_mm=.1)
        events = {cp['checkpoint_kind']: cp for cp in event_checkpoints(rows, tight, .1)}
        self.assertTrue(events['pre_tilt']['prefix_numerically_valid'])
        for kind in ('first_stall', 'ramp_complete', 'terminal'):
            self.assertFalse(events[kind]['prefix_numerically_valid'])
            self.assertEqual(events[kind]['prefix_max_penetration_mm'], .03)
        default = event_checkpoints(rows, protocol(), .1)
        self.assertTrue(all(cp['prefix_numerically_valid'] for cp in default))

    def test_unrequested_events_remain_observed_and_unreached_events_remain_unknown(self):
        _, rows = observed_reference()
        events = event_checkpoints(rows[:9], protocol(), .1, probe_events=('first_stall', 'terminal'))
        cp = {row['checkpoint_kind']: row for row in events}
        self.assertTrue(cp['pre_tilt']['reached'])
        self.assertFalse(cp['pre_tilt']['probe_requested'])
        self.assertFalse(cp['ramp_complete']['reached'])
        self.assertFalse(cp['first_stall']['reached'])
        for kind in ('ramp_complete', 'first_stall'):
            self.assertIsNone(cp[kind]['depth_mm'])
            self.assertNotIn('state', cp[kind])
            self.assertIsNone(recovery_label(cp[kind]['probes'], cp[kind]['reached'],
                                            cp[kind]['prefix_numerically_valid'])[0])
        self.assertTrue(cp['terminal']['reached'])
        self.assertEqual(cp['terminal']['step'], 8)

    def test_geometry_subset_collection_does_not_replace_unknown_recovery_labels(self):
        plan = subset_plan(make_plan(True), radial_clearance_mm=.1)
        p = replace(protocol(), target_valid=len(plan['cases']))
        first = next_case(p, [], plan)
        events = event_checkpoints(observed_reference()[1][:5], p, .1)
        attempt = dict(first, status='complete', metrics={'numerically_valid': True}, checkpoints=events)
        self.assertEqual(accepted_slots([attempt]), {0})
        second = next_case(p, [attempt], plan)
        self.assertEqual(second['case_id'], plan['cases'][1]['case_id'])
        self.assertEqual(second['slot'], 1)
        self.assertEqual(second['source_slot'], plan['cases'][1]['source_slot'])
        attempt['metrics']['numerically_valid'] = False
        retry = next_case(p, [attempt], plan)
        self.assertEqual(retry['case_id'], first['case_id'])
        self.assertEqual(retry['retry'], 1)

    def test_only_completed_clearance_has_uncensored_recorded_motion(self):
        self.assertFalse(recovery_termination('cleared')['recovery_censored'])
        for reason in ('operational_budget_exceeded', 'grasp_retention_limit',
                       'numerical_screen_failed', 'time_budget_exhausted'):
            self.assertTrue(recovery_termination(reason)['recovery_censored'])


if __name__ == '__main__':
    unittest.main()
