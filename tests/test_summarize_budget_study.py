"""Evidence distinctions for online budget summary; no simulator dependency."""
from copy import deepcopy
from dataclasses import asdict
import json

from research.forge_budget import make_plan, make_angle6_plan
from research.forge_protocol import protocol
from simulation.summarize_budget_study import branch_row, pair_rows, summarize_study, withdrawal_plot_data, write_summary
from tests.test_forge_mechanics_protocol import prefix


def record(condition, policy, *, outcome='clear_safe', peak=3.):
    clear = outcome == 'clear_safe'
    return dict(condition=condition, policy=policy, effective_protocol=asdict(protocol()),
                reference=dict(metrics=dict(reference_complete=True, numerically_valid=True, grasp_retained=True),
                               evidence=dict(max_wrist_force_n=2., phase_metrics={})),
                recovery=dict(metrics=dict(numerically_valid=True, grasp_retained=True, cleared=clear, recovery_censored=not clear,
                                           reason='cleared' if clear else 'operational_budget_exceeded'),
                              evidence=dict(max_wrist_force_n=peak, phase_metrics={
                                  'retreat':dict(phase_executed=True, max_wrist_force_n=peak,
                                                 peak_wrist_force_mean100ms_n=1.)})),
                outcome=dict(outcome=outcome, reference_eligible=True, recovery_safe=clear, task_safe=clear))


def pair_fixture():
    condition = make_plan()['conditions'][2]
    a, b = record(condition, 'straight', outcome='recovery_budget_exceeded', peak=4.5), record(condition, 'realign')
    branches = [branch_row(condition, policy, dict(status='complete'), run)
                for policy, run in [('straight', a), ('realign', b)]]
    return condition, branches, [a, b], [prefix(), prefix()]


def test_matched_budget_abort_is_paired_but_peak_difference_is_unknown():
    condition, branches, records, rows = pair_fixture()
    result, audit = pair_rows(condition, branches, records, rows)
    assert result['pair_state'] == 'paired_straight_overbudget_realign_safe'
    assert audit['replay_prefix_matched'] is True
    assert branches[0]['recovery_peak_is_lower_bound'] is True
    assert result['paired_costs_complete'] is False
    assert result['straight_minus_realign_recovery_observed_peak_wrist_force_n'] is None


def test_endpoint_equality_does_not_hide_prefix_difference_or_erase_branch_outcomes():
    condition, branches, records, rows = pair_fixture()
    rows[1][1]['tip_x_mm'] += .01
    result, audit = pair_rows(condition, branches, records, rows)
    assert audit['replay_matched'] is True
    assert result['pair_state'] == 'prefix_mismatch_unknown'
    assert branches[0]['outcome'] == 'recovery_budget_exceeded'
    assert branches[1]['outcome'] == 'clear_safe'


def test_equal_seed_and_prefix_do_not_hide_changed_protocol():
    condition, branches, records, rows = pair_fixture()
    records[1]['effective_protocol']['force_budget_n'] = 3.
    result, audit = pair_rows(condition, branches, records, rows)
    assert audit['replay_prefix_matched'] is True
    assert result['pair_state'] == 'configuration_mismatch_unknown'


def test_bitwise_equality_not_required_and_complete_costs_can_be_compared():
    condition = make_plan()['conditions'][2]
    records = [record(condition, 'straight', peak=3.5), record(condition, 'realign', peak=3.)]
    branches = [branch_row(condition, p, dict(status='complete'), r) for p, r in zip(['straight','realign'],records)]
    rows = [prefix(), prefix()]; rows[1][1]['tip_x_mm'] += .00001
    result, audit = pair_rows(condition, branches, records, rows)
    assert result['pair_state'] == 'paired_both_clear_safe'
    assert audit['replay_prefix_equal'] is False
    assert result['paired_costs_complete'] is True
    assert result['straight_minus_realign_recovery_observed_peak_wrist_force_n'] == .5


def test_skipped_alternative_has_no_recovery_label_or_peak():
    condition, branches, records, rows = pair_fixture()
    branches[1] = branch_row(condition, 'realign', dict(status='not_tested', reason='reference_not_eligible'))
    result, audit = pair_rows(condition, branches, [records[0], None], [rows[0], []])
    assert result['pair_state'] == 'alternative_not_tested_reference_ineligible'
    assert branches[1]['recovery_safe'] is None
    assert branches[1]['recovery_observed_peak_wrist_force_n'] is None
    assert audit is None


def test_simultaneous_budget_and_grasp_events_are_not_a_clean_policy_pair():
    condition, branches, records, rows = pair_fixture()
    records[0]['recovery']['metrics']['grasp_retained'] = False
    branches[0] = branch_row(condition, 'straight', dict(status='complete'), records[0])
    result, audit = pair_rows(condition, branches, records, rows)
    assert result['pair_state'] == 'paired_mixed_constraints'
    assert result['paired_recovery_evidence_eligible'] is False
    assert branches[0]['outcome'] == 'recovery_budget_exceeded'
    assert branches[0]['recovery_grasp_retained'] is False


def test_pending_study_exports_all_planned_denominators_without_live_reads(tmp_path):
    source=tmp_path/'live'; source.mkdir()
    plan=make_plan()
    manifest=dict(schema='Forge-budget-v1', status='running', case_plan=plan, sources={},
                  attempts=[dict(condition_id=plan['conditions'][0]['condition_id'], policy='straight',
                                 status='running', folder='does-not-exist')])
    (source/'study.json').write_text(json.dumps(manifest))
    result=summarize_study(source)
    assert len(result['cases'])==8 and len(result['branches'])==16 and len(result['paired'])==8
    assert result['summary']['counts']['closed_branches']==0
    assert result['summary']['input_files']==[]
    assert all(r['recovery_safe'] is None for r in result['branches'])
    write_summary(result,tmp_path/'review')
    assert (tmp_path/'review/branches.csv').exists()
    assert (tmp_path/'review/schema.json').exists()
    for condition, case_row in zip(plan['conditions'], result['cases']):
        assert case_row['split_group_id'] == condition['case']['split_group_id']
        assert case_row['path_group_id'] == condition['case']['path_group_id']


def test_shared_path_groups_survive_budget_and_policy_variants():
    plan = make_plan()
    conditions = [c for c in plan['conditions'] if c['case']['target_depth_mm']==18
                  and c['case']['tilt_amplitude_deg']==1.5 and c['case']['pair_static_friction']==1.]
    rows = [branch_row(c, p) for c in conditions for p in ('straight', 'realign')]
    assert len(rows) == 6
    assert {r['force_budget_n'] for r in rows} == {3., 4., 5.}
    assert len({r['condition_id'] for r in rows}) == 3
    assert len({r['path_group_id'] for r in rows}) == 1
    assert len({r['split_group_id'] for r in rows}) == 1
    assert all(r['recovery_safe'] is None for r in rows)


def test_withdrawal_plot_uses_recorded_command_despite_copied_initial_target():
    rows = [dict(recovery_time_s=0., depth_mm=18.228, command_depth_mm=18., command_withdrawal_mm=0.),
            dict(recovery_time_s=.004, depth_mm=18.229, command_depth_mm=18.228, command_withdrawal_mm=0.),
            dict(recovery_time_s=.5, depth_mm=18.2, command_depth_mm=17.228, command_withdrawal_mm=1.)]
    data = withdrawal_plot_data(18.228, rows)
    assert data['commanded_mm'] == [0., 0., 1.]
    assert data['actual_mm'][0] == 0.
    assert data['actual_mm'][1] < 0.
    del rows[0]['command_withdrawal_mm']
    assert withdrawal_plot_data(18.228, rows)['commanded_mm'][0] is None


def test_high_angle_fields_distinguish_planned_command_and_actual_reach():
    condition=make_angle6_plan()['conditions'][-1]
    run=record(condition,'straight')
    run['reference']['metrics'].update(terminal_command_tilt_deg=1.8, terminal_actual_tilt_deg=1.3,
                                       max_actual_tilt_deg=1.35, tilt_ramp_completed=False)
    row=branch_row(condition,'straight',dict(status='complete'),run)
    assert row['tilt_amplitude_deg']==6.
    assert row['tilt_ramp_duration_s']==1.
    assert row['planned_peak_command_tilt_rate_deg_s']==9.
    assert row['terminal_command_tilt_deg']==1.8 and row['terminal_actual_tilt_deg']==1.3
    assert row['max_actual_tilt_deg']==1.35 and row['tilt_ramp_completed'] is False


def test_second_plan_summary_has_six_conditions_and_twelve_missing_branches(tmp_path):
    source=tmp_path/'live';source.mkdir()
    plan=make_angle6_plan()
    (source/'study.json').write_text(json.dumps(dict(schema='Forge-budget-v1',status='running',
        case_plan=plan,sources={},attempts=[])))
    result=summarize_study(source)
    assert len(result['cases'])==6 and len(result['branches'])==12
    assert [r['planned_peak_command_tilt_rate_deg_s'] for r in result['cases']]==[2.25,3.,4.5,6.,7.5,9.]
    assert all(r['straight_terminal_command_tilt_deg'] is None for r in result['cases'])
