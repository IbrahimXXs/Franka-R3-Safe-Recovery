"""Read a running or finished gap-study batch without changing its outputs.

By default print compact JSON.  ``--output-dir`` writes review tables and,
with ``--plots``, the existing aggregate plots to a separate destination. Run
the latter in the project's conda environment (matplotlib is optional for
the read-only JSON/CSV summary). The source batch is never written.
"""
import argparse
import csv
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


FACTORS = ('case_id', 'radial_clearance_mm', 'effective_radial_clearance_mm',
           'hole_diameter_mm', 'peg_diameter_mm', 'tilt_onset_fraction',
           'tilt_onset_mm', 'tilt_amplitude_deg', 'tilt_sign', 'tilt_axis')
INSERTION = ('insertion_success', 'stalled', 'numerically_valid', 'grasp_retained',
             'max_depth', 'max_penetration', 'max_wrist_force_n', 'max_wrist_torque_nm',
             'tilt_applicable', 'tilt_triggered', 'tilt_ramp_completed',
             'max_actual_tilt_deg', 'max_command_tilt_deg', 'terminal_actual_tilt_deg',
             'terminal_command_tilt_deg', 'tilt_trigger_actual_depth_mm',
             'tilt_completion_actual_tilt_deg', 'tilt_completion_actual_depth_mm')
POLICY = ('label_eligible', 'replay_matched', 'safe_recovery', 'numerically_valid',
          'grasp_retained', 'cleared', 'reason', 'termination_reason', 'termination_phase',
          'recovery_censored', 'max_wrist_force_n', 'max_wrist_torque_nm',
          'stop_peak_wrist_force_n', 'realign_peak_wrist_force_n',
          'retreat_peak_wrist_force_n', 'retreat_peak_wrist_force_mean100ms_n',
          'retreat_peak_wrist_world_z_abs_n', 'retreat_peak_contact_downward_resistance_n',
          'retreat_actual_withdrawal_mm', 'retreat_commanded_withdrawal_mm',
          'retreat_contact_resistive_work_j', 'duration_s')


def _event_status(case, checkpoint, reference_complete):
    """Separate absent experimental events from unresolved recovery evidence."""
    kind = checkpoint.get('checkpoint_kind', 'fixed_depth')
    if kind in ('pre_tilt', 'ramp_complete') and case.get('tilt_amplitude_deg') == 0:
        return 'not_applicable_aligned_control'
    if not checkpoint.get('reached'):
        return 'event_not_observed'
    if not reference_complete:
        return 'reference_incomplete'
    if not checkpoint.get('prefix_numerically_valid'):
        return 'numerically_invalid_prefix'
    if not checkpoint.get('probes'):
        return ('event_only_no_probe_requested' if checkpoint.get('probe_requested') is False
                else 'recovery_evidence_unresolved')
    label = checkpoint.get('Y_R_tested')
    return {1: 'safe_policy_observed', 0: 'tested_policies_failed',
            None: 'recovery_evidence_unresolved'}[label]


def _counts(rows, checkpoints):
    complete = [row for row in rows if row.get('reference_complete')]
    trusted = [row for row in complete if row.get('numerically_valid') is True]
    continuation = [row for row in complete if row.get('continuation_straight_present')]
    valid_continuation = [row for row in continuation if row.get('numerically_valid') is True
                          and row.get('continuation_straight_numerically_valid') is True]
    continuation_costs = [row.get('continuation_straight_retreat_peak_wrist_force_n') for row in continuation]
    valid_continuation_costs = [row.get('continuation_straight_retreat_peak_wrist_force_n') for row in valid_continuation]
    continuation_whole_costs = [row.get('continuation_straight_max_wrist_force_n') for row in continuation]
    valid_continuation_whole_costs = [row.get('continuation_straight_max_wrist_force_n') for row in valid_continuation]
    # A later invalid contact does not invalidate a previously screened prefix
    # and its independently matched, valid recovery. Keep that evidence while
    # also exposing the stricter whole-reference subset.
    eligible = [row for row in checkpoints if row.get('parent_complete') and row.get('reached')
                and row.get('prefix_numerically_valid') is True]
    whole_valid = [row for row in eligible if row.get('parent_numerically_valid') is True]
    kinds = {}
    for row in checkpoints:
        key = row['event_status']
        kinds[key] = kinds.get(key, 0) + 1
    paired = [row for row in eligible if row.get('straight_label_eligible') and row.get('realign_label_eligible')]
    whole_pairs = [row for row in paired if row.get('parent_numerically_valid') is True]
    costs = [row.get('straight_retreat_peak_wrist_force_n') for row in eligible if row.get('straight_label_eligible')]
    whole_costs = [row.get('straight_retreat_peak_wrist_force_n') for row in whole_valid if row.get('straight_label_eligible')]
    return dict(planned_cases=len(rows), complete_cases=len(complete),
                valid_complete_cases=len(trusted),
                successful_insertions=sum(row.get('insertion_success') is True for row in complete),
                stalled_insertions=sum(row.get('stalled') is True for row in complete),
                trusted_successful_insertions=sum(row.get('insertion_success') is True for row in trusted),
                trusted_stalled_insertions=sum(row.get('stalled') is True for row in trusted),
                completed_perturbation_cases=sum(row.get('tilt_amplitude_deg', 0) > 0 for row in complete),
                triggered_perturbations=sum(row.get('tilt_triggered') is True for row in complete),
                command_ramps_completed=sum(row.get('tilt_ramp_completed') is True for row in complete),
                checkpoint_event_status_counts=kinds,
                prefix_valid_reached_checkpoints=len(eligible),
                whole_reference_valid_reached_checkpoints=len(whole_valid),
                both_policy_pairs_eligible=len(paired),
                whole_reference_valid_both_policy_pairs_eligible=len(whole_pairs),
                direct_failed_realign_safe=sum(row.get('straight_safe_recovery') is False and
                                              row.get('realign_safe_recovery') is True for row in paired),
                whole_reference_valid_direct_failed_realign_safe=sum(row.get('straight_safe_recovery') is False and
                                                                     row.get('realign_safe_recovery') is True for row in whole_pairs),
                maximum_recorded_direct_retreat_wrist_n=max([x for x in costs if x is not None], default=None),
                whole_reference_valid_maximum_recorded_direct_retreat_wrist_n=max([x for x in whole_costs if x is not None], default=None),
                continuation_straight_observed_complete_references=len(continuation),
                continuation_straight_cleared=sum(row.get('continuation_straight_cleared') is True for row in continuation),
                continuation_straight_valid_reference_and_recovery=len(valid_continuation),
                continuation_straight_valid_safe_recoveries=sum(row.get('continuation_straight_safe_recovery') is True for row in valid_continuation),
                continuation_straight_censored=sum(row.get('continuation_straight_recovery_censored') is True for row in continuation),
                continuation_straight_maximum_recorded_retreat_wrist_n=max([x for x in continuation_costs if x is not None], default=None),
                continuation_straight_valid_maximum_recorded_retreat_wrist_n=max([x for x in valid_continuation_costs if x is not None], default=None),
                continuation_straight_maximum_recorded_whole_recovery_wrist_n=max([x for x in continuation_whole_costs if x is not None], default=None),
                continuation_straight_valid_maximum_recorded_whole_recovery_wrist_n=max([x for x in valid_continuation_whole_costs if x is not None], default=None),
                censored_probes=sum(row.get(f'{policy}_recovery_censored') is True for row in checkpoints
                                    for policy in ('straight', 'realign')))


def summarize_batch(directory):
    directory = Path(directory).resolve()
    batch = json.loads((directory / 'batch.json').read_text())
    plan = json.loads((directory / 'plan.json').read_text())
    planned = {case['case_id']: case for case in plan['cases']}
    attempts = []; checkpoints = []; latest = {}; child_directories = []
    for group in batch['groups']:
        source = directory / group['folder']
        child_directories.append(str(source))
        path = source / 'study.json'
        if not path.exists():
            continue
        manifest = json.loads(path.read_text())
        for attempt in manifest['attempts']:
            case_id = attempt['case_id']
            if case_id not in planned:
                raise ValueError(f'Unplanned child case: {case_id}')
            factors = {key: attempt.get(key, planned[case_id].get(key)) for key in FACTORS}
            metrics = attempt.get('metrics', {})
            complete = attempt.get('status') == 'complete'
            row = dict(factors, source_study=group['folder'], trajectory_id=attempt['trajectory_id'],
                       attempt_status=attempt.get('status'), reference_complete=complete,
                       **{key: metrics.get(key) for key in INSERTION})
            continuation = attempt.get('final_retreat') or {}
            row['continuation_straight_present'] = bool(continuation)
            row['continuation_straight_origin'] = 'original_reference_continuation_without_replay' if continuation else None
            # This is valuable direct-withdrawal evidence even if an independent
            # replay cannot be matched. It never fills a missing paired policy.
            row.update({f'continuation_straight_{key}': continuation.get(key) for key in POLICY
                        if key not in ('replay_matched', 'label_eligible')})
            amplitude = factors['tilt_amplitude_deg']
            actual = row['terminal_actual_tilt_deg']
            row['terminal_actual_fraction_of_target'] = actual / amplitude if amplitude and actual is not None else None
            attempts.append(row); latest[case_id] = row
            for cp in attempt.get('checkpoints', []):
                state = cp.get('state') or {}
                item = dict(row, checkpoint_id=cp.get('checkpoint_id'), checkpoint_kind=cp.get('checkpoint_kind'),
                            reached=cp.get('reached'), event_status=_event_status(attempt, cp, complete),
                            parent_complete=complete, parent_numerically_valid=metrics.get('numerically_valid'),
                            prefix_numerically_valid=cp.get('prefix_numerically_valid'),
                            probe_requested=cp.get('probe_requested'),
                            checkpoint_depth_mm=state.get('depth_mm'), checkpoint_actual_tilt_deg=state.get('tilt_deg'),
                            checkpoint_command_tilt_deg=state.get('command_tilt_deg'),
                            checkpoint_wrist_force_n=state.get('wrist_force_n'),
                            Y_R_tested=cp.get('Y_R_tested'), label_reason=cp.get('label_reason'))
                probes = {p['policy']: p for p in cp.get('probes', [])}
                for policy in ('straight', 'realign'):
                    probe = probes.get(policy, {})
                    item[f'{policy}_probe_present'] = bool(probe)
                    item.update({f'{policy}_{key}': probe.get(key) for key in POLICY})
                checkpoints.append(item)
    cases = []
    for case_id, case in planned.items():
        cases.append(latest.get(case_id) or dict({key: case.get(key) for key in FACTORS},
                                                attempt_status='not_started', reference_complete=False))
    current = {(row['source_study'], row['trajectory_id']) for row in latest.values()}
    current_checkpoints = [row for row in checkpoints if (row['source_study'], row['trajectory_id']) in current]
    by_gap = []
    for gap in sorted({case['radial_clearance_mm'] for case in cases}, reverse=True):
        subset = [row for row in cases if row['radial_clearance_mm'] == gap]
        cps = [row for row in current_checkpoints if row['radial_clearance_mm'] == gap]
        by_gap.append(dict(radial_clearance_mm=gap, **_counts(subset, cps)))
    summary = dict(schema='Forge-gap-review-v1', source_batch=str(directory), source_status=batch['status'],
                   interpretation='latest attempt per planned case; absent events are not recovery failures',
                   count_definitions={
                       'successful_insertions_and_stalled_insertions': 'all complete recorded flags, including invalid references; not exclusively physical outcomes',
                       'trusted_insertion_counts': 'complete references passing the numerical screen; this does not establish solver convergence',
                       'default_recovery_counts': 'complete attempts with reached, numerically valid prefixes and policy-level label eligibility; later whole-reference invalidity does not erase an earlier valid matched probe',
                       'whole_reference_valid_recovery_counts': 'stricter subset also requiring the complete reference to pass numerical screening',
                       'continuation_straight': 'straight withdrawal executed immediately after the original reference, without a reset/replay; separate from matched checkpoint policy pairs and never used to fill an unknown replay',
                       'continuation_straight_valid_counts': 'completed reference plus numerically valid reference and continuous recovery; valid_safe also requires the recorded safe_recovery result',
                       'source_labels': 'Y_R_tested and label_reason are copied unchanged; event_status is a separate review classification'},
                   counts=_counts(cases, current_checkpoints), by_gap=by_gap)
    return dict(summary=summary, cases=cases, attempts=attempts, checkpoints=checkpoints,
                case_plan=plan, child_directories=child_directories)


def _table(path, rows):
    if not rows:
        return
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
        writer.writeheader(); writer.writerows(rows)


def export_review(result, directory, plots=False):
    directory = Path(directory).resolve()
    source = Path(result['summary']['source_batch']).resolve()
    if directory == source or source in directory.parents:
        raise ValueError('Review destination must be outside the source batch; source outputs are read-only')
    directory.mkdir(parents=True, exist_ok=True)
    (directory / 'summary.json').write_text(json.dumps(result['summary'], indent=2, allow_nan=False) + '\n')
    for key, name in [('cases', 'case_summary.csv'), ('attempts', 'attempt_summary.csv'),
                      ('checkpoints', 'checkpoint_summary.csv')]:
        _table(directory / name, result[key])
    if plots:
        from simulation.phase2_report import write_gap_batch_report
        write_gap_batch_report(directory, result['case_plan'], result['child_directories'],
                               status=result['summary']['source_status'])
    return directory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--batch-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--plots', action='store_true')
    args = parser.parse_args()
    if args.plots and args.output_dir is None:
        parser.error('--plots requires a separate --output-dir')
    result = summarize_batch(args.batch_dir)
    if args.output_dir is not None:
        export_review(result, args.output_dir, args.plots)
    print(json.dumps(result['summary'], indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
