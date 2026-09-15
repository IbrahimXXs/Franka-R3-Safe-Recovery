#!/usr/bin/env python3
"""Export representative measured states from completed mechanics attempts.

This offline command imports no simulator and does not modify study outputs.
Example:
  python3 simulation/analyze_mechanics_states.py \
      --study-dir outputs/Forge-Mechanics-Matrix-20260915 \
      --output-dir outputs/Forge-Mechanics-Matrix-20260915-review
"""
import argparse
from collections import deque
import csv
import hashlib
import io
import json
import math
from pathlib import Path


FACTOR_FIELDS = (
    'case_id', 'trajectory_id', 'folder', 'sample_role', 'target_depth_mm',
    'tilt_amplitude_deg', 'tilt_axis', 'radial_clearance_mm',
    'effective_radial_clearance_mm', 'hole_depth_mm', 'hole_type',
    'pair_static_friction', 'pair_dynamic_friction', 'hole_static_friction',
    'hole_dynamic_friction', 'peg_static_friction', 'peg_dynamic_friction',
    'friction_combine_mode',
)
SAMPLE_FIELDS = (
    'phase', 'time_s', 'physics_time_s', 'recovery_time_s', 'reference_step',
    'depth_mm', 'tilt_deg', 'tip_x_mm', 'tip_y_mm',
    'command_depth_mm', 'command_tilt_deg', 'command_pitch_deg',
    'command_offset_x_mm', 'command_offset_y_mm',
    'depth_gate_passed', 'depth_gate_time_s', 'depth_gate_step',
    'geometry_overlap_estimate_valid', 'effective_guide_span_estimate_mm',
    'geometry_overlap_estimate_mm', 'geometry_wall_signed_violation_estimate_mm',
    'wall_pos_loaded', 'wall_neg_loaded', 'wall_both_sides_loaded',
    'wall_pos_normal_load_n', 'wall_neg_normal_load_n',
    'wall_other_normal_load_n', 'wall_excluded_normal_load_n',
    'wall_centroid_axial_span_mm', 'contact_axial_span_valid', 'contact_axial_span_mm',
    'normal_load_n', 'contact_count', 'friction_contact_count', 'wall_contact_count',
    'wrist_force_n', 'wrist_torque_nm', 'force_norm_n', 'torque_norm_nm',
    'normal_force_norm_n', 'friction_force_norm_n',
    'min_separation_mm', 'grasp_slip_mm', 'grasp_slip_deg',
    'fx', 'fy', 'fz', 'taux', 'tauy', 'tauz',
) + tuple(f'wall_{side}_centroid_{axis}_mm' for side in ('pos', 'neg') for axis in 'xyz') \
  + tuple(f'wall_{side}_normal_force_socket_{axis}_n' for side in ('pos', 'neg') for axis in 'xyz') \
  + tuple(f'{component}_force_world_{axis}_n' for component in ('normal', 'friction') for axis in 'xyz') \
  + tuple(f'{component}_torque_world_{axis}_nm' for component in ('normal', 'friction') for axis in 'xyz') \
  + tuple(f'wrist_force_world_{i}' for i in range(3)) \
  + tuple(f'wrist_torque_about_peg_base_world_{i}' for i in range(3))
STATE_FIELDS = (
    'policy', 'policy_origin', 'state', 'state_observed', 'missing_reason',
    'phase_executed', 'phase_executed_sample_count', 'phase_observed_duration_s',
    'sample_is_copied_recovery_start', 'sample_is_reference_initialization',
    'source_csv', 'sample_index', 'selection_phase', 'selection_metric', 'selection_value_n',
    'window_sample_count', 'window_support_s', 'window_start_sample_index',
    'window_start_time_s', 'window_end_time_s', 'window_first_last_timestamp_span_s',
    'reference_complete', 'reference_numerically_valid', 'reference_grasp_retained',
    'policy_label_eligible', 'policy_replay_matched', 'policy_safe_recovery',
    'policy_termination_reason',
    'geometric_guide_length_Lg_mm', 'observed_contact_separation_ell_mm',
    'contact_normal_axial_resistance_n', 'contact_friction_axial_resistance_n',
)


def parse_cell(value):
    """Retain real booleans/nulls; reject non-finite numbers instead of hiding them."""
    if value is None:
        return None
    value = value.strip()
    if value.lower() in ('', 'none', 'null'):
        return None
    if value.lower() in ('true', 'false'):
        return value.lower() == 'true'
    try:
        number = float(value)
    except ValueError:
        return value
    if not math.isfinite(number):
        raise ValueError(f'Non-finite CSV value: {value!r}')
    return number


def number(row, key):
    value = row.get(key)
    return float(value) if type(value) in (int, float) and math.isfinite(value) else None


def load_profile(path):
    data = path.read_bytes()
    if not data.endswith(b'\n'):
        raise ValueError(f'Incomplete CSV record: {path}')
    rows = []
    for index, raw in enumerate(csv.DictReader(io.StringIO(data.decode()))):
        if None in raw:
            raise ValueError(f'Unexpected extra CSV fields at row {index}: {path}')
        try:
            rows.append({key: parse_cell(value) for key, value in raw.items()})
        except ValueError as error:
            raise ValueError(f'{path}, sample {index}: {error}') from error
    return rows, hashlib.sha256(data).hexdigest()


def peak_selection(rows, field, phase=None):
    candidates = [(index, row) for index, row in enumerate(rows)
                  if (phase is None or row.get('phase') == phase) and number(row, field) is not None]
    if not candidates:
        return None
    index, row = max(candidates, key=lambda item: number(item[1], field))
    return {'index': index, 'metric': field, 'value_n': number(row, field)}


def mean100ms_selection(rows, dt, phase='retreat'):
    """Select the endpoint of a complete same-phase window, never across a gap."""
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('Physics timestep must be positive and finite')
    count = max(1, math.ceil(.1/dt-1e-9))
    window = deque()
    best = None
    previous_time = None
    for index, row in enumerate(rows):
        time = number(row, 'time_s')
        value = number(row, 'wrist_force_n')
        if (row.get('phase') != phase or time is None or value is None
                or row.get('recovery_time_s') == 0):
            window.clear()
            previous_time = None
            continue
        if previous_time is not None and not math.isclose(time-previous_time, dt, abs_tol=1e-7, rel_tol=1e-5):
            window.clear()
        previous_time = time
        window.append((index, time, value))
        if len(window) > count:
            window.popleft()
        if len(window) == count:
            mean = sum(item[2] for item in window)/count
            if best is None or mean > best['value_n']:
                best = {'index': index, 'metric': 'wrist_force_mean100ms_n', 'value_n': mean,
                        'window_sample_count': count, 'window_support_s': count*dt,
                        'window_start_sample_index': window[0][0],
                        'window_start_time_s': window[0][1], 'window_end_time_s': time,
                        'window_first_last_timestamp_span_s': time-window[0][1]}
    return best


def phase_execution(rows, index, policy, dt=None):
    """Observed simulation-step support in this phase through the selected state.

    Recovery row zero can be a copied pre-recovery observation. It represents
    no executed hold, although its pose and force are valid observations.
    """
    phase = rows[index].get('phase')
    first = index
    while first > 0 and rows[first-1].get('phase') == phase:
        first -= 1
    copied = lambda row: policy != 'reference' and number(row, 'recovery_time_s') == 0.
    initial = lambda row: policy == 'reference' and number(row, 'reference_step') == 0.
    count = sum(not (copied(row) or initial(row)) for row in rows[first:index+1])
    if dt is not None and (not math.isfinite(dt) or dt <= 0):
        raise ValueError('Physics timestep must be positive and finite')
    return {'phase_executed': count > 0, 'phase_executed_sample_count': count,
            'phase_observed_duration_s': count*dt if dt is not None else (0. if count == 0 else None),
            'sample_is_copied_recovery_start': copied(rows[index]),
            'sample_is_reference_initialization': initial(rows[index])}


def state_row(attempt, rows, selection, state, source, policy, phase, missing_reason=None, policy_metadata=None, dt=None):
    metadata = policy_metadata or {}
    result = {key: attempt.get(key) for key in FACTOR_FIELDS}
    result.update({key: None for key in STATE_FIELDS+SAMPLE_FIELDS})
    result.update(policy=policy,
                  policy_origin='reference' if policy == 'reference' else (
                      'original_continuous' if policy == 'straight' else 'replayed_terminal'),
                  state=state, source_csv=str(source), selection_phase=phase,
                  state_observed=selection is not None,
                  missing_reason=missing_reason if selection is None else None,
                  reference_complete=attempt.get('metrics', {}).get('reference_complete'),
                  reference_numerically_valid=attempt.get('metrics', {}).get('numerically_valid'),
                  reference_grasp_retained=attempt.get('metrics', {}).get('grasp_retained'),
                  policy_label_eligible=metadata.get('label_eligible'),
                  policy_replay_matched=metadata.get('replay_matched'),
                  policy_safe_recovery=metadata.get('safe_recovery'),
                  policy_termination_reason=metadata.get('reason'))
    if selection is None:
        return result
    sample = rows[selection['index']]
    result.update({key: sample.get(key) for key in SAMPLE_FIELDS})
    result.update(phase_execution(rows, selection['index'], policy, dt))
    result.update(sample_index=selection['index'], selection_metric=selection.get('metric'),
                  selection_value_n=selection.get('value_n'))
    for key in ('window_sample_count', 'window_support_s', 'window_start_sample_index',
                'window_start_time_s', 'window_end_time_s', 'window_first_last_timestamp_span_s'):
        result[key] = selection.get(key)
    result['geometric_guide_length_Lg_mm'] = number(sample, 'effective_guide_span_estimate_mm') \
        if sample.get('geometry_overlap_estimate_valid') is True else None
    result['observed_contact_separation_ell_mm'] = number(sample, 'wall_centroid_axial_span_mm') \
        if sample.get('wall_both_sides_loaded') is True else None
    for kind in ('normal', 'friction'):
        axial = number(sample, f'{kind}_force_world_z_n')
        result[f'contact_{kind}_axial_resistance_n'] = max(0., -axial) if axial is not None else None
    return result


def extract_attempt(attempt, study_dir, dt):
    folder = study_dir/attempt['folder']
    reference_path = folder/'insertion.csv'
    reference, digest = load_profile(reference_path)
    hashes = {str(reference_path.resolve()): digest}
    gate = next((index for index, row in enumerate(reference) if row.get('depth_gate_passed') is True), None)
    selectors = (
        ('depth_gate', {'index': gate} if gate is not None else None, 'first_depth_gate_passed', 'depth_gate_not_observed'),
        ('reference_terminal', {'index': len(reference)-1} if reference else None, 'reference', 'empty_reference'),
        ('reference_normal_load_peak', peak_selection(reference, 'normal_load_n'), 'reference', 'normal_load_missing'),
    )
    output = [state_row(attempt, reference, selected, name, reference_path, 'reference', phase, reason, dt=dt)
              for name, selected, phase, reason in selectors]
    policies = [('straight', folder/'final_retreat.csv', attempt.get('final_retreat', {}))]
    realign = next((probe for probe in attempt.get('probes', []) if probe.get('policy') == 'realign'), {})
    policies.append(('realign', folder/'terminal_realign_recovery.csv', realign))
    for policy, path, metadata in policies:
        if path.exists():
            rows, digest = load_profile(path)
            hashes[str(path.resolve())] = digest
        else:
            rows = []
        missing_file = None if path.exists() else 'policy_profile_unavailable'
        stop_indices = [index for index, row in enumerate(rows) if row.get('phase') == 'stop']
        selectors = (
            ('stop_end', {'index': stop_indices[-1]} if stop_indices else None, 'stop', 'stop_phase_not_observed'),
            ('retreat_wrist_peak', peak_selection(rows, 'wrist_force_n', 'retreat'), 'retreat', 'retreat_force_not_observed'),
            ('retreat_wrist_mean100ms_peak_end', mean100ms_selection(rows, dt), 'retreat', 'complete_100ms_retreat_window_unavailable'),
            ('retreat_normal_load_peak', peak_selection(rows, 'normal_load_n', 'retreat'), 'retreat', 'retreat_normal_load_not_observed'),
        )
        output.extend(state_row(attempt, rows, selected, name, path, policy, phase,
                                missing_file or reason, metadata, dt=dt)
                      for name, selected, phase, reason in selectors)
    return output, hashes


def export_states(study_dir, output_dir):
    study_dir, output_dir = Path(study_dir).resolve(), Path(output_dir).resolve()
    # This command must not overwrite any file inside a live study directory.
    if output_dir == study_dir or study_dir in output_dir.parents:
        raise ValueError('Choose an output directory outside the simulation study')
    study_bytes = (study_dir/'study.json').read_bytes()
    study = json.loads(study_bytes)
    hz = float(study['physics_hz'])
    if not math.isfinite(hz) or hz <= 0:
        raise ValueError('Invalid recorded physics rate')
    complete = [a for a in study.get('attempts', []) if a.get('status') == 'complete']
    records, hashes = [], {}
    for attempt in complete:
        selected, sources = extract_attempt(attempt, study_dir, 1/hz)
        records.extend(selected)
        hashes.update(sources)
    for path, digest in hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Completed profile changed during extraction: {path}')
    if any(type(value) in (int, float) and not math.isfinite(value)
           for record in records for value in record.values()):
        raise ValueError('State export contains a non-finite numeric value')
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir/'mechanics_states.csv'
    fields = FACTOR_FIELDS+STATE_FIELDS+SAMPLE_FIELDS
    with destination.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for record in records:
            writer.writerow({key: ('true' if value else 'false') if type(value) is bool else value
                             for key, value in record.items()})
    summary = {
        'schema': 'mechanics_state_export_v2', 'study_dir': str(study_dir),
        'study_status_at_read': study.get('status'),
        'study_json_sha256_at_read': hashlib.sha256(study_bytes).hexdigest(),
        'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        'completed_attempts': len(complete), 'completed_case_ids': [a['case_id'] for a in complete],
        'planned_cases': len(study.get('case_plan', {}).get('cases', [])),
        'all_planned_cases_included': len({a['case_id'] for a in complete}) == len(study.get('case_plan', {}).get('cases', [])),
        'state_rows': len(records), 'observed_state_rows': sum(r['state_observed'] for r in records),
        'physics_hz': hz, 'csv_path': str(destination), 'input_profile_hashes': hashes,
        'csv_boolean_encoding': 'true / false; empty field means null, not false or zero',
        'state_selection': {
            'depth_gate': 'first reference row with authoritative depth_gate_passed=true',
            'reference_terminal': 'last saved reference state, even if protocol failed; quality labels remain separate',
            'reference_normal_load_peak': 'maximum normal_load_n anywhere in reference; first sample wins a tie',
            'stop_end': 'last saved stop-phase observation, including a copied recovery t=0 row if the guard prevented every STOP step; inspect phase_executed and phase_observed_duration_s',
            'retreat_wrist_peak': 'maximum wrist force within retreat phase only',
            'retreat_wrist_mean100ms_peak_end': 'instantaneous state at the end of the greatest complete 100ms wrist-force window; selection_value_n stores the window mean',
            'retreat_normal_load_peak': 'maximum normal_load_n within retreat phase only',
        },
        'phase_execution_definitions': {
            'phase_executed': 'whether at least one non-copied simulation-step observation exists in the contiguous actual sample phase through this selected state; null for an unobserved selected state',
            'phase_executed_sample_count': 'number of such observations up to and including this state; excludes non-reference recovery_time_s=0 copied rows and reference_step=0 initialization rows',
            'phase_observed_duration_s': 'phase_executed_sample_count / physics_hz: recorded simulation-step support through the selected state, not a guarantee of phase completion; missing samples do not count as observed time',
            'sample_is_copied_recovery_start': 'true only for a selected non-reference observation with recovery_time_s=0; an observation with no new physics step',
            'sample_is_reference_initialization': 'true only for a selected reference observation with reference_step=0; prepare result before the trajectory executes its first physics step',
        },
        'length_definitions': {
            'geometric_guide_length_Lg_mm': 'alias of valid effective_guide_span_estimate_mm: actual pose, full-radius shaft axis segment clipped to straight wall; approximate geometry, not a measured contact distance',
            'observed_contact_separation_ell_mm': 'alias of wall_centroid_axial_span_mm only when both loaded straight-wall side flags pass; normal-load weighted region centers, not uniquely identified contact pairs',
            'contact_axial_span_mm': 'separate extrema span of classified normal points; inspect contact_axial_span_valid; coincident points may give zero',
        },
        'limitations': [
            'Only attempts marked complete at manifest read are included; invalid or failed completed attempts are retained with their labels.',
            'Missing events/policies/windows produce unobserved rows with null measurements, never invented zeros.',
            'state_observed alone does not imply a policy phase executed. A guard can leave only a copied STOP t=0 observation, with phase_executed=false and duration zero.',
            'The 100ms calculation uses ceil(0.1/dt) contiguous retreat samples and the same N*dt support convention as the study metrics; first-to-last timestamp span is separately recorded.',
            'Signed world-Z force components are fixture-on-peg; axial withdrawal resistance is max(0,-Z). Independent component peaks must not be added.',
            'Command columns are preserved exactly as logged; a policy final orientation target is not necessarily its interpolated within-step command.',
            'Lg and ell have distinct validity masks and meanings. No geometric compatibility bound or material deformation is inferred.',
        ],
    }
    (output_dir/'mechanics_states.schema.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    summary = export_states(args.study_dir, args.output_dir)
    print(json.dumps({key: summary[key] for key in ('completed_attempts', 'state_rows', 'observed_state_rows', 'csv_path')}, indent=2))


if __name__ == '__main__':
    main()
