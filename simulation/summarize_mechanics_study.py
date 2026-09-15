"""Read-only tables for the 24-case fixed-depth / friction / tilt study.

The default CLI prints summary JSON; --output-dir exports to a separate review
directory. Partial manifests preserve pending and excluded planned cases.
Neither a numerical reject nor a missing replay becomes a mechanical failure.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:sys.path.insert(0, str(ROOT))
from research.forge_mechanics_metrics import FACTOR_FIELDS


OVERVIEW_FIELDS = (
    'case_id', 'target_depth_mm', 'terminal_depth_mm', 'pair_static_friction', 'pair_dynamic_friction',
    'tilt_amplitude_deg', 'terminal_actual_tilt_deg', 'reference_state', 'reference_reason',
    'numerically_valid', 'max_normal_load_n', 'reference_peak_wrist_force_n', 'max_grasp_slip_mm',
    'straight_numerically_valid', 'straight_retreat_observed', 'straight_reason', 'straight_safe_recovery',
    'straight_retreat_peak_wrist_force_n', 'straight_retreat_peak_wrist_force_mean100ms_n',
    'straight_max_wrist_force_n', 'paired_state', 'paired_costs_complete',
    'matched_realign_retreat_peak_wrist_force_n', 'matched_realign_max_wrist_force_n',
    'matched_realign_realign_peak_wrist_force_n',
)


def overview_rows(cases):
    """A small projection of the full case table, without reinterpreting labels."""
    result = []
    for case in cases:
        row = {key:case.get(key) for key in OVERVIEW_FIELDS}
        row['reference_reason'] = (case.get('reference_termination_reason') or case.get('termination_reason')
                                   or case.get('exclusion_reason'))
        for key in OVERVIEW_FIELDS:
            if key.startswith('matched_realign_'):
                row[key] = case.get(key.removeprefix('matched_')) if case.get('paired_results_known') is True else None
        result.append(row)
    return result


def _flat(value, prefix=''):
    return {prefix+key: item for key, item in value.items() if not isinstance(item, (dict, list))}


def _known_boolean(value):
    return type(value) is bool


def _pair_state(attempt, realign):
    metrics = attempt.get('metrics') or {}
    straight = attempt.get('final_retreat') or {}
    if attempt.get('status') != 'complete':return 'reference_incomplete'
    if metrics.get('numerically_valid') is not True:return 'reference_numerically_invalid'
    if metrics.get('reference_complete') is False:return 'reference_protocol_incomplete'
    if not straight:return 'continuous_straight_unobserved'
    if straight.get('numerically_valid') is not True:return 'continuous_straight_numerically_invalid'
    if not realign:return 'realign_not_tested'
    if realign.get('reason') == 'reference_not_eligible':return 'realign_not_tested_reference_ineligible'
    if realign.get('replay_matched') is not True:return 'replay_endpoint_unmatched'
    if realign.get('replay_prefix_matched') is not True:return 'replay_prefix_unmatched'
    if realign.get('numerically_valid') is not True:return 'realign_numerically_invalid'
    if not (straight.get('label_eligible') and realign.get('label_eligible')):
        return 'recovery_label_ineligible'
    if not (_known_boolean(straight.get('safe_recovery')) and _known_boolean(realign.get('safe_recovery'))):
        return 'recovery_outcome_unknown'
    return 'matched_results_known'


def _reference_state(attempt):
    metrics = attempt.get('metrics') or {}
    if attempt.get('status') != 'complete':return 'incomplete'
    if metrics.get('numerically_valid') is not True:return 'numerically_invalid'
    if metrics.get('grasp_retained') is False:return 'grasp_failed'
    if metrics.get('reference_within_budget') is False:return 'operational_budget_exceeded'
    if metrics.get('aligned_entry_reached') is not True:return 'aligned_depth_gate_not_reached'
    if metrics.get('reference_protocol_completed') is not True or metrics.get('reference_complete') is False:return 'tilt_sequence_incomplete'
    return 'valid_protocol_completed'


def _factor_key(row):
    return tuple(row.get(key) for key in ('target_depth_mm', 'pair_static_friction', 'pair_dynamic_friction'))


def _aggregate(rows):
    completed = [r for r in rows if r.get('attempt_status') == 'complete']
    valid = [r for r in completed if r.get('numerically_valid') is True]
    paired = [r for r in rows if r.get('paired_results_known')]
    counts = {}
    for r in rows:counts[r['reference_state']] = counts.get(r['reference_state'], 0)+1
    pair_counts = {}
    for r in rows:pair_counts[r['paired_state']] = pair_counts.get(r['paired_state'], 0)+1
    return dict(planned_cases=len(rows), completed_cases=len(completed), numerically_valid_completed_cases=len(valid),
                reference_state_counts=counts, pair_state_counts=pair_counts,
                valid_depth_gates_passed=sum(r.get('aligned_entry_reached') is True for r in valid),
                valid_protocol_completed=sum(r.get('reference_state') == 'valid_protocol_completed' for r in valid),
                valid_continuous_straight_safe=sum(r.get('straight_numerically_valid') is True and
                                                 r.get('straight_safe_recovery') is True for r in valid),
                matched_policy_pairs=len(paired),
                matched_complete_cost_pairs=sum(r.get('paired_costs_complete') is True for r in paired),
                matched_both_policies_safe=sum(r.get('straight_safe_recovery') is True and
                                              r.get('realign_safe_recovery') is True for r in paired),
                matched_straight_failed_realign_safe=sum(r.get('straight_safe_recovery') is False and
                                                        r.get('realign_safe_recovery') is True for r in paired))


def summarize_manifest(manifest):
    """Return case/attempt/probe/control tables without reading or writing files."""
    if manifest.get('study', manifest.get('schema')) != 'Forge-mechanics-v1':
        raise ValueError('Expected Forge-mechanics-v1')
    plan = manifest.get('case_plan') or manifest.get('plan') or {}
    planned = plan.get('cases') if isinstance(plan, dict) else plan
    if not planned:planned = manifest.get('cases') or []
    if not planned:raise ValueError('A mechanics summary needs the frozen planned cases')
    by_id = {case['case_id']:case for case in planned}
    skipped = {}
    for entry in manifest.get('skipped_cases', []):
        if isinstance(entry, dict):skipped[entry['case_id']] = entry.get('reason', 'excluded')
        elif isinstance(entry, (list, tuple)):skipped[entry[0]] = entry[1]
        else:skipped[entry] = 'excluded'
    attempts = []; probes = []; latest = {}
    for attempt in manifest.get('attempts', []):
        case_id = attempt['case_id']
        if case_id not in by_id:raise ValueError(f'Unplanned case {case_id}')
        case = {**by_id[case_id], **attempt}
        metrics = attempt.get('metrics') or {}
        realign = next((p for p in attempt.get('probes', []) if p.get('policy') == 'realign'), {})
        straight = attempt.get('final_retreat') or {}
        row = {key:case.get(key) for key in FACTOR_FIELDS}
        row.update(trajectory_id=attempt.get('trajectory_id'), folder=attempt.get('folder'),
                   attempt_status=attempt.get('status'), reference_state=_reference_state(attempt),
                   **_flat(metrics))
        row.update(straight_origin='original_reference_continuation' if straight else None,
                   realign_origin='independent_reference_replay' if realign else None,
                   **_flat(straight, 'straight_'), **_flat(realign, 'realign_'))
        row['paired_state'] = _pair_state(attempt, realign)
        row['paired_results_known'] = row['paired_state'] == 'matched_results_known'
        row['paired_costs_complete'] = bool(row['paired_results_known'] and straight.get('cleared') is True and
                                           realign.get('cleared') is True and not straight.get('recovery_censored') and
                                           not realign.get('recovery_censored'))
        for key in ('max_wrist_force_n', 'retreat_peak_wrist_force_n', 'retreat_peak_wrist_force_mean100ms_n'):
            a, b = straight.get(key), realign.get(key)
            row[f'paired_recorded_{key}_reduction_n'] = a-b if row['paired_results_known'] and a is not None and b is not None else None
        latest[case_id] = row;attempts.append(row)
        for policy, probe, origin in [('straight',straight,'original_reference_continuation'),
                                      ('realign',realign,'independent_reference_replay')]:
            item = {key:row.get(key) for key in FACTOR_FIELDS}
            item.update(trajectory_id=attempt.get('trajectory_id'), folder=attempt.get('folder'), policy=policy,
                        origin=origin, probe_present=bool(probe), reference_state=row['reference_state'],
                        parent_numerically_valid=metrics.get('numerically_valid'), paired_state=row['paired_state'])
            item.update(_flat(probe))
            for source in ('replay_errors', 'replay_prefix_errors', 'replay_prefix_max_errors'):
                item.update(_flat(probe.get(source) or {}, source+'_'))
            probes.append(item)
    cases = []
    for case in planned:
        if case['case_id'] in latest:row = dict(latest[case['case_id']])
        else:
            row = {key:case.get(key) for key in FACTOR_FIELDS}
            reason = skipped.get(case['case_id'])
            row.update(attempt_status='excluded' if reason else 'not_started',
                       reference_state='excluded' if reason else 'not_started', exclusion_reason=reason,
                       paired_results_known=False, paired_costs_complete=False,
                       paired_state='excluded' if reason else 'not_started')
        cases.append(row)
    baselines = {_factor_key(row):row for row in cases if row.get('tilt_amplitude_deg') == 0}
    effects = []
    compare_keys = ('tilt_terminal_depth_loss_mm', 'tilt_max_depth_loss_mm',
                    'straight_retreat_peak_wrist_force_n', 'straight_retreat_peak_wrist_force_mean100ms_n',
                    'straight_retreat_peak_contact_normal_axial_resistance_n',
                    'straight_retreat_peak_contact_friction_axial_resistance_n')
    for row in cases:
        if not row.get('tilt_amplitude_deg'):continue
        base = baselines.get(_factor_key(row), {})
        eligible = row['reference_state'] == 'valid_protocol_completed' and base.get('reference_state') == 'valid_protocol_completed'
        effect = {key:row.get(key) for key in FACTOR_FIELDS}
        effect.update(baseline_case_id=base.get('case_id'), case_reference_state=row['reference_state'],
                      baseline_reference_state=base.get('reference_state'), reference_comparison_eligible=eligible,
                      completed_safe_withdrawal_comparison=bool(eligible and row.get('straight_safe_recovery') is True and
                                                               base.get('straight_safe_recovery') is True))
        for key in compare_keys:
            value, control = row.get(key), base.get(key)
            allowed = effect['completed_safe_withdrawal_comparison'] if key.startswith('straight_') else eligible
            effect[key] = value;effect['baseline_'+key] = control
            effect['difference_'+key] = value-control if allowed and value is not None and control is not None else None
        effects.append(effect)
    summary = dict(schema='Forge-mechanics-review-v1', source_status=manifest.get('status'),
                   recorded_attempts=len(attempts), elapsed_wall_seconds=manifest.get('elapsed_wall_seconds'),
                   counts=_aggregate(cases),
                   interpretation={
                       'unit':'latest attempt per frozen planned condition; retries are not independent experiments',
                       'entry':'aligned target-depth dwell is separate from tilt-window depth loss and numerical/grasp acceptance',
                       'pair':'continuous straight is compared with realign only when endpoint and full-prefix tolerances match and both policy outcomes are eligible; bitwise prefix equality is diagnostic only',
                       'cost':'recorded peaks from censored motions are partial; paired_costs_complete identifies two completed clearances',
                       'controls':'theta=0 control shares target depth and effective static/dynamic friction pair; differences are descriptive, not significance tests',
                       'missing':'absent measurements remain null, never zero; excluded cases remain in the planned denominator'},
                   by_target_depth=[dict(target_depth_mm=depth, **_aggregate([r for r in cases if r.get('target_depth_mm')==depth]))
                                    for depth in sorted({r['target_depth_mm'] for r in cases})])
    return dict(summary=summary, cases=cases, overview=overview_rows(cases), attempts=attempts,
                probes=probes, control_comparisons=effects)


def write_review(result, output_dir):
    directory = Path(output_dir);directory.mkdir(parents=True, exist_ok=True)
    (directory/'summary.json').write_text(json.dumps(result['summary'], indent=2, allow_nan=False)+'\n')
    for key, name in (('cases','cases.csv'), ('overview','mechanics_overview.csv'), ('attempts','attempts.csv'), ('probes','recovery_probes.csv'),
                      ('control_comparisons','control_comparisons.csv')):
        rows = result[key]
        if not rows:continue
        with (directory/name).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
            writer.writeheader();writer.writerows(rows)
    overview_schema = dict(schema='Forge-mechanics-overview-v1', rows=len(result['cases']), columns=list(OVERVIEW_FIELDS),
                           source_study=result['summary'].get('source_study'),
                           source_manifest_sha256=result['summary'].get('source_manifest_sha256'),
                           field_rules={
                               'identity':'One row for every planned condition; field names/units match cases.csv unless noted.',
                               'reference_reason':'reference_termination_reason, falling back to termination_reason or exclusion_reason when absent.',
                               'matched_realign_*':'Corresponding realign_* value only when paired_results_known is true; otherwise null.',
                               'max_normal_load_n':'Time maximum of the per-frame sum of contact-normal magnitudes during the reference; neither a time-integrated load nor axial withdrawal force.',
                               '*_max_wrist_force_n':'Maximum over all recorded recovery rows, including the copied t=0 reference endpoint, stop and alignment. If no recovery step executed, this is only the initial snapshot, not an executed recovery peak. *_retreat_peak_* covers actual retreat only.',
                               'matched_realign_realign_peak_wrist_force_n':'The recenter + align phase peak; included so a lower retreat peak is not mistaken for lower whole-recovery cost.',
                               'paired_costs_complete':'False means costs are not two complete clearances; a known failed/censored result may still retain recorded partial peaks.',
                               'null':'Missing/unobserved or excluded comparison; never zero. True/False are source labels, not inferred from blank cells.'})
    (directory/'mechanics_overview.schema.json').write_text(json.dumps(overview_schema, indent=2, allow_nan=False)+'\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--plots', action='store_true', help='Write standalone PNG/PDF figures with tables (requires --output-dir and matplotlib)')
    args = parser.parse_args()
    if args.plots and not args.output_dir:
        parser.error('--plots requires --output-dir')
    source = args.study_dir.resolve()
    manifest_bytes = (source/'study.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    result = summarize_manifest(manifest)
    result['summary']['source_study'] = str(source)
    result['summary']['source_manifest_sha256'] = hashlib.sha256(manifest_bytes).hexdigest()
    if args.output_dir:
        destination = args.output_dir.resolve()
        if destination == source or source in destination.parents:
            parser.error('--output-dir must be outside the source study; original outputs are read-only')
        write_review(result, destination)
        (destination/'source_study_snapshot.json').write_bytes(manifest_bytes)
        if args.plots:
            from simulation.plot_mechanics_study import write_plots
            artifacts = write_plots(result, destination)
            sources = dict(source_study=str(source), source_status=manifest.get('status'),
                           source_manifest_snapshot='source_study_snapshot.json',
                           source_manifest_sha256=result['summary']['source_manifest_sha256'],
                           source_case_ids=[row['case_id'] for row in result['cases']],
                           source_trajectory_ids=[row.get('trajectory_id') for row in result['attempts']],
                           source_simulation_hashes=manifest.get('sources'),
                           review_source_sha256={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest()
                                                 for name in ('simulation/summarize_mechanics_study.py', 'simulation/plot_mechanics_study.py')},
                           artifacts=artifacts,
                           interpretation='A read-only snapshot; pending, excluded, invalid and censored results are not plotted as zero. Descriptive, not significance testing.')
            (destination/'plot_sources.json').write_text(json.dumps(sources, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result['summary'], indent=2, allow_nan=False))


if __name__=='__main__':main()
