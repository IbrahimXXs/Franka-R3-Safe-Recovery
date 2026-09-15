"""Scientific screening boundaries, not simulation/controller implementation tests."""
from copy import deepcopy
import json

from simulation.analyze_mechanics_budget_sweep import (
    COMPLETE, EXCEEDED, analyze_case, analyze_studies, first_crossing, separation_intervals,
)


def row(force, time=0., phase='insert', torque=.01):
    return dict(wrist_force_n=force, wrist_torque_nm=torque, time_s=time, phase=phase,
                depth_mm=18., command_depth_mm=18.)


def attempt():
    recovery = dict(label_eligible=True, safe_recovery=True, numerically_valid=True,
                    grasp_retained=True, cleared=True, recovery_censored=False, reason='cleared')
    return dict(status='complete', metrics=dict(numerically_valid=True, grasp_retained=True,
                reference_complete=True, reference_protocol_completed=True),
                final_retreat=deepcopy(recovery), probes=[dict(policy='realign',
                replay_matched=True, replay_prefix_matched=True, replay_prefix_numerically_valid=True,
                replay_prefix_equal=False, **recovery)])


def test_reference_crossing_prevents_reachable_recovery_even_if_both_later_clear():
    result = analyze_case(attempt(), [row(4.1)], [row(.2, phase='retreat')], [row(.1, phase='retreat')], 4.)
    assert not result['reference']['admissible']
    assert result['reference']['state'] == 'reference_force_budget_exceeded'
    assert result['straight']['state'] == result['realign']['state'] == 'reference_not_admissible'
    assert result['pair_state'] == 'reference_not_admissible'


def test_first_raw_strict_crossing_and_whole_recovery_include_stop():
    samples = [row(4., phase='stop'), row(4.2, .01, 'stop'), row(.1, 2., 'retreat')]
    assert first_crossing(samples, 'wrist_force_n', 4.)['sample_index'] == 1
    result = analyze_case(attempt(), [row(3.)], samples, [row(3.5, phase='stop'), row(.1, 2., 'retreat')], 4.)
    assert result['straight']['state'] == EXCEEDED
    assert result['straight']['first_force_crossing']['phase'] == 'stop'
    assert result['straight']['phase_first_force_crossings']['retreat'] is None
    assert result['straight']['source_cleared'] is True
    assert result['pair_state'] == 'historical_straight_exceeded_realign_completed'


def test_censored_without_force_crossing_is_unknown_not_success():
    source = attempt()
    source['final_retreat'].update(cleared=False, recovery_censored=True, reason='timeout')
    result = analyze_case(source, [row(.1)], [row(.3, phase='retreat')], [row(.2, phase='retreat')], 4.)
    assert result['straight']['state'] == 'unknown_right_censored'
    assert result['realign']['state'] == COMPLETE
    assert result['pair_state'] == 'unknown_or_other_termination'


def test_missing_or_numerically_invalid_signal_cannot_pass():
    source = attempt()
    result = analyze_case(source, [row(.1)], [row(None, phase='retreat')], [], 4.)
    assert result['straight']['state'] == 'unknown_recovery_signals_missing'
    source['metrics']['numerically_valid'] = False
    result = analyze_case(source, [row(10.)], [row(.1)], [row(.1)], 4.)
    assert result['reference']['state'] == 'unknown_reference_numerically_invalid'
    assert result['reference']['first_force_crossing'] is not None


def test_replay_mismatch_cannot_become_paired_success_and_source_is_unchanged():
    source = attempt()
    source['probes'][0]['replay_prefix_matched'] = False
    original = deepcopy(source)
    result = analyze_case(source, [row(.1)], [row(4.5, phase='retreat')], [row(.1, phase='retreat')], 4.)
    assert result['straight']['state'] == EXCEEDED
    assert result['realign']['state'] == 'unknown_replay_unmatched_or_invalid'
    assert result['pair_state'] == 'unknown_replay_unmatched_or_invalid'
    assert source == original


def test_separation_window_accounts_for_reference_and_realign_whole_peak():
    source = attempt()
    straight = [row(3.75, phase='stop'), row(4.58, 1., 'retreat')]
    realign = [row(3.75, phase='stop'), row(2.28, 1., 'realign'), row(1.15, 3., 'retreat')]
    intervals = separation_intervals(source, [row(3.56)], straight, realign, 1.)
    assert len(intervals) == 1
    assert intervals[0]['lower_inclusive_n'] == 3.75
    assert intervals[0]['upper_exclusive_n'] == 4.58
    assert separation_intervals(source, [row(13.15)], straight, realign, 1.) == []


def test_earlier_torque_crossing_is_not_called_first_force_failure():
    result = analyze_case(attempt(), [row(.1)],
                          [row(.1, phase='stop', torque=1.1), row(5., 1., 'retreat')],
                          [row(.1)], 4.)
    assert result['straight']['state'] == 'historical_recovery_torque_exceedance'
    assert result['straight']['first_force_crossing']['sample_index'] == 1
    assert result['straight']['lower_budget_history_truncates_at_sample'] == 0


def test_grasp_gate_does_not_become_a_withdrawal_force_negative():
    source = attempt()
    source['metrics'].update(grasp_retained=False, reference_complete=False)
    source['final_retreat'].update(grasp_retained=False, cleared=False, recovery_censored=True)
    result = analyze_case(source, [row(2.)], [row(1., phase='stop')], [], 8.)
    assert result['reference']['state'] == 'reference_grasp_gate_failed'
    assert result['straight']['state'] == 'reference_not_admissible'


def test_pending_case_remains_in_denominator_without_opening_live_files(tmp_path):
    case = dict(case_id='pending', target_depth_mm=18, pair_static_friction=1.,
                pair_dynamic_friction=1., tilt_amplitude_deg=1.5)
    manifest = dict(study='Forge-mechanics-v1', status='running', physics_hz=240,
                    protocol=dict(force_budget_n=20., torque_budget_nm=1.),
                    case_plan=dict(cases=[case]), attempts=[dict(**case, status='running', folder='not_written')])
    (tmp_path/'study.json').write_text(json.dumps(manifest))
    result = analyze_studies([tmp_path], [4.])
    assert len(result['entries']) == 1
    assert result['entries'][0]['reference']['state'] == 'unknown_reference_not_complete'
    assert result['entries'][0]['reference']['peak_wrist_force_n'] is None
    assert result['input_files'] == []
