"""Read-only comparison of independent mechanics runs at different rates.

Only completed attempts' trajectories are opened. This is a sensitivity audit:
the custom controller updates every physics step, and the independent resets
need not produce identical initial states. It is not a pure dt-convergence test.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from simulation.summarize_mechanics_study import summarize_manifest


# Compare every other planned field, including newly added physical parameters.
ORCHESTRATION_FIELDS = {
    'case_id', 'family', 'sample_role', 'split_group_id', 'path_group_id',
    'slot', 'source_slot', 'retry', 'trajectory_id', 'folder', 'status',
}
EXPECTED_RATE_CONFIG_PATHS = {
    'sim.dt', 'sim.render_interval', 'decimation', 'ft_smoothing_factor',
}
STUDY_LOCAL_CONFIG_PATHS = {
    'task.fixed_asset.spawn.usd_path', 'task.fixed_asset_cfg.usd_path',
}
RESULT_FIELDS = (
    'reference_state', 'reference_termination_reason', 'numerically_valid',
    'reference_protocol_completed', 'grasp_retained', 'reference_within_budget',
    'terminal_depth_mm', 'terminal_actual_tilt_deg', 'terminal_command_tilt_deg',
    'tilt_terminal_depth_loss_mm', 'max_normal_load_n', 'reference_peak_wrist_force_n',
    'reference_peak_wrist_force_mean100ms_within_phase_n',
    'reference_peak_normal_load_mean100ms_within_phase_n',
    'insert_peak_wrist_force_n', 'insert_peak_wrist_force_mean100ms_n',
    'insert_peak_normal_load_n', 'insert_peak_normal_load_mean100ms_n',
    'hold_peak_normal_load_mean100ms_n', 'hold_peak_wrist_force_mean100ms_n',
    'max_grasp_slip_mm', 'max_grasp_slip_deg', 'max_penetration_mm',
    'straight_retreat_observed', 'straight_reason', 'straight_numerically_valid',
    'straight_grasp_retained', 'straight_label_eligible', 'straight_safe_recovery',
    'straight_cleared', 'straight_recovery_censored', 'straight_duration_s',
    'straight_stop_peak_wrist_force_n', 'straight_retreat_peak_wrist_force_n',
    'straight_retreat_peak_wrist_force_mean100ms_n', 'straight_max_wrist_force_n',
    'straight_retreat_peak_contact_downward_resistance_n',
    'straight_retreat_peak_contact_downward_resistance_mean100ms_n',
    'straight_retreat_peak_contact_normal_axial_resistance_n',
    'straight_retreat_peak_contact_friction_axial_resistance_n',
    'straight_retreat_max_depth_tracking_error_mm', 'straight_max_penetration',
    'paired_state', 'paired_results_known', 'paired_costs_complete',
    'realign_reason', 'realign_replay_matched', 'realign_replay_prefix_matched',
    'realign_replay_prefix_equal', 'realign_numerically_valid',
)
MATCHED_FIELDS = (
    'safe_recovery', 'cleared', 'recovery_censored', 'duration_s',
    'max_wrist_force_n', 'stop_peak_wrist_force_n', 'realign_peak_wrist_force_n',
    'retreat_peak_wrist_force_n', 'retreat_peak_wrist_force_mean100ms_n',
)
INITIAL_FIELDS = (
    'phase', 'time_s', 'physics_time_s', 'depth_mm', 'tip_x_mm', 'tip_y_mm',
    'tilt_deg', 'command_depth_mm', 'command_tilt_deg', 'grasp_slip_mm',
    'grasp_slip_deg', 'wrist_force_n', 'wrist_torque_nm', 'normal_load_n',
    'qw', 'qx', 'qy', 'qz', 'vx', 'vy', 'vz', 'omegax', 'omegay', 'omegaz',
    *(f'joint{i}_rad' for i in range(1, 8)),
    *(f'joint_velocity{i}_rad_s' for i in range(1, 8)),
    *(f'hand_pose_{i}' for i in range(7)),
    *(f'grasp_position_{i}' for i in range(3)),
    *(f'grasp_quat_{i}' for i in range(4)),
)


def _hash(data):
    return hashlib.sha256(data).hexdigest()


def _differences(a, b, path=''):
    if isinstance(a, dict) and isinstance(b, dict):
        result = []
        for key in sorted(set(a) | set(b)):
            item_path = f'{path}.{key}' if path else key
            if key not in a or key not in b:
                result.append(dict(path=item_path, a=a.get(key), b=b.get(key),
                                   missing_in='a' if key not in a else 'b'))
            else:
                result.extend(_differences(a[key], b[key], item_path))
        return result
    return [] if a == b else [dict(path=path, a=a, b=b)]


def _value(value):
    if value in (None, ''):
        return None
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError):
        return value


def _vector(row, keys):
    values = [row.get(key) for key in keys]
    return values if all(type(v) in (int, float) for v in values) else None


def _delta(a, b, keys, scale=1., maximum=False):
    av, bv = _vector(a, keys), _vector(b, keys)
    if av is None or bv is None:
        return None
    diffs = [(y-x)*scale for x, y in zip(av, bv)]
    return max(map(abs, diffs)) if maximum else math.sqrt(sum(d*d for d in diffs))


def _quat_delta(a, b, keys):
    av, bv = _vector(a, keys), _vector(b, keys)
    if av is None or bv is None:
        return None
    norm = math.sqrt(sum(x*x for x in av)*sum(x*x for x in bv))
    if not norm:
        return None
    return math.degrees(2*math.acos(min(1., abs(sum(x*y for x, y in zip(av, bv))/norm))))


def initial_differences(a, b):
    if a is None or b is None:
        return None
    result = {f'delta_{key}': b[key]-a[key] if type(a.get(key)) in (float, int)
              and type(b.get(key)) in (float, int) else None for key in
              ('depth_mm', 'tip_x_mm', 'tip_y_mm', 'tilt_deg', 'wrist_force_n', 'grasp_slip_mm')}
    result.update(
        tip_position_distance_mm=_delta(a, b, ('tip_x_mm', 'tip_y_mm', 'depth_mm')),
        peg_orientation_distance_deg=_quat_delta(a, b, ('qw', 'qx', 'qy', 'qz')),
        hand_position_distance_mm=_delta(a, b, [f'hand_pose_{i}' for i in range(3)], 1000),
        hand_orientation_distance_deg=_quat_delta(a, b, [f'hand_pose_{i}' for i in range(3, 7)]),
        joint_position_max_difference_rad=_delta(a, b, [f'joint{i}_rad' for i in range(1, 8)], maximum=True),
        joint_velocity_max_difference_rad_s=_delta(a, b, [f'joint_velocity{i}_rad_s' for i in range(1, 8)], maximum=True),
        peg_linear_velocity_distance_m_s=_delta(a, b, ('vx', 'vy', 'vz')),
        peg_angular_velocity_distance_rad_s=_delta(a, b, ('omegax', 'omegay', 'omegaz')),
        grasp_position_distance_mm=_delta(a, b, [f'grasp_position_{i}' for i in range(3)], 1000),
        grasp_orientation_distance_deg=_quat_delta(a, b, [f'grasp_quat_{i}' for i in range(4)]),
    )
    return result


def load_study(directory):
    directory = Path(directory).resolve()
    manifest_bytes = (directory/'study.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest.get('study') != 'Forge-mechanics-v1':
        raise ValueError(f'Not a mechanics study: {directory}')
    config_bytes = (directory/'config.json').read_bytes()
    config = json.loads(config_bytes)
    plans = manifest['case_plan']['cases']
    if len({p['case_id'] for p in plans}) != len(plans):
        raise ValueError(f'Duplicate planned case id: {directory}')
    complete = {}
    for attempt in manifest.get('attempts', []):
        if attempt.get('status') == 'complete':
            if attempt['case_id'] in complete:
                raise ValueError(f'Multiple complete attempts: {directory}/{attempt["case_id"]}')
            complete[attempt['case_id']] = attempt
    sources = []
    for relative, declared in manifest.get('sources', {}).items():
        path = (directory/'source'/relative).resolve()
        if directory/'source' not in path.parents:
            raise ValueError(f'Invalid archived source path: {relative}')
        actual = _hash(path.read_bytes()) if path.is_file() else None
        sources.append(dict(path=relative, declared_sha256=declared, actual_sha256=actual,
                            verified=actual == declared))
    summary = summarize_manifest({**manifest, 'attempts':list(complete.values())})
    rows = {r['case_id']:r for r in summary['cases']}
    latest = {a['case_id']:a for a in manifest.get('attempts', [])}
    return dict(directory=directory, manifest=manifest, config=config, complete=complete,
                plans={p['case_id']:p for p in plans}, rows=rows, latest=latest,
                provenance=dict(path=str(directory), status=manifest.get('status'),
                                study_sha256=_hash(manifest_bytes), config_sha256=_hash(config_bytes),
                                source_archives=sources, all_source_archives_verified=bool(sources) and
                                all(s['verified'] for s in sources)))


def completed_case(study, case_id, files):
    attempt = study['complete'].get(case_id)
    if attempt is None:
        planned_row = study['rows'][case_id]
        status = study['latest'].get(case_id, {}).get('status', planned_row['attempt_status'])
        return dict(status=status, measurements=None, initial_state=None, terminal_state=None,
                    runtime_material=None, trajectory_id=None, input_files=[])
    folder = (study['directory']/attempt['folder']).resolve()
    if study['directory'] not in folder.parents:
        raise ValueError(f'Attempt folder outside study: {folder}')
    trajectory_path = folder/'trajectory.json'
    trajectory_bytes = trajectory_path.read_bytes()
    trajectory = json.loads(trajectory_bytes)
    if trajectory != attempt:
        raise ValueError(f'Completed trajectory disagrees with study snapshot: {trajectory_path}')
    insertion_path = folder/'insertion.csv'
    insertion_bytes = insertion_path.read_bytes()
    stream = csv.DictReader(io.StringIO(insertion_bytes.decode()))
    first = next(stream, None)
    if first is None:
        raise ValueError(f'Completed reference CSV is empty: {insertion_path}')
    last = first
    for last in stream:
        pass
    initial = {key:_value(first.get(key)) for key in INITIAL_FIELDS}
    terminal = {key:_value(last.get(key)) for key in
                ('depth_mm', 'tip_x_mm', 'tip_y_mm', 'tilt_deg', 'command_depth_mm', 'command_tilt_deg',
                 'qw', 'qx', 'qy', 'qz', 'phase', 'time_s')}
    row = study['rows'][case_id]
    measurements = {key:row.get(key) for key in RESULT_FIELDS}
    for quantity in ('wrist_force', 'normal_load'):
        values = [row.get(f'{phase}_peak_{quantity}_mean100ms_n')
                  for phase in ('approach', 'insert', 'settle', 'tilt', 'hold')]
        values = [v for v in values if type(v) in (float, int) and math.isfinite(v)]
        measurements[f'reference_peak_{quantity}_mean100ms_within_phase_n'] = max(values) if values else None
    for key in MATCHED_FIELDS:
        measurements['matched_realign_'+key] = row.get('realign_'+key) if row['paired_results_known'] else None
    owned_files = [dict(path=str(path), sha256=_hash(data)) for path, data in
                   ((trajectory_path, trajectory_bytes), (insertion_path, insertion_bytes))]
    files.extend(owned_files)
    return dict(status='complete', measurements=measurements, initial_state=initial, terminal_state=terminal,
                runtime_material=attempt.get('material'), effective_radial_clearance_mm=attempt.get('effective_radial_clearance_mm'),
                trajectory_id=attempt.get('trajectory_id'), input_files=owned_files)


def _rates(study):
    hz = study['manifest']['physics_hz']
    config = study['config']
    return dict(physics_hz=hz, custom_closed_loop_control_hz=hz, force_sampling_hz=hz,
                sim_dt_s=config['sim']['dt'], configured_env_decimation=config['decimation'],
                configured_env_action_hz=hz/config['decimation'], render_interval=config['sim']['render_interval'],
                ft_smoothing_factor=config['ft_smoothing_factor'],
                explanation='ForgeBackend.tick regenerates controls and records each physics step; env decimation is not the custom controller update period.')


def compare_studies(directory_a, directory_b):
    a, b = load_study(directory_a), load_study(directory_b)
    common = sorted(set(a['plans']) & set(b['plans']))
    audits = {}
    for key in ('protocol', 'effective_protocol', 'seed', 'recovery_motion', 'recovery_policy_definition',
                'simulation_options', 'geometry', 'sources', 'acceptance_rule', 'budget_signal'):
        differences = _differences(a['manifest'].get(key), b['manifest'].get(key))
        audits[key] = dict(equal=not differences, differences=differences)
    config_differences = _differences(a['config'], b['config'])
    for item in config_differences:
        item['category'] = ('rate_derived' if item['path'] in EXPECTED_RATE_CONFIG_PATHS else
                            'study_local_geometry_path' if item['path'] in STUDY_LOCAL_CONFIG_PATHS else 'other')
    cases = []; files = []
    for case_id in common:
        pa = {k:v for k, v in a['plans'][case_id].items() if k not in ORCHESTRATION_FIELDS}
        pb = {k:v for k, v in b['plans'][case_id].items() if k not in ORCHESTRATION_FIELDS}
        differences = _differences(pa, pb)
        ca, cb = completed_case(a, case_id, files), completed_case(b, case_id, files)
        both_complete = ca['status'] == cb['status'] == 'complete'
        case = dict(case_id=case_id, physical_parameters_a=pa, physical_parameters_b=pb,
                    physical_plan_equal=not differences, physical_plan_differences=differences,
                    both_complete=both_complete, a=ca, b=cb,
                    initial_state_differences_b_minus_a=initial_differences(ca['initial_state'], cb['initial_state']),
                    runtime_material_comparison=dict(equal=ca['runtime_material'] == cb['runtime_material'],
                        differences=_differences(ca['runtime_material'], cb['runtime_material'])) if both_complete else None,
                    effective_gap_equal=ca.get('effective_radial_clearance_mm') == cb.get('effective_radial_clearance_mm') if both_complete else None)
        cases.append(case)
    return dict(schema='Forge-mechanics-rate-comparison-v1',
                study_a={**a['provenance'], 'rates':_rates(a)}, study_b={**b['provenance'], 'rates':_rates(b)},
                common_planned_cases=len(common), both_completed_cases=sum(c['both_complete'] for c in cases),
                only_in_a=sorted(set(a['plans'])-set(b['plans'])), only_in_b=sorted(set(b['plans'])-set(a['plans'])),
                configuration_audit=audits, runtime_config_differences=config_differences,
                ignored_plan_fields=sorted(ORCHESTRATION_FIELDS), cases=cases,
                interpretation={
                    'scope':'Independent-run rate sensitivity, not pure dt convergence or a matched cross-rate causal comparison. Physics, closed-loop control and sample rates change together; reset states can also differ.',
                    'initial_state':'First saved reference sample after reset, before approach; differences are b minus a, distances are nonnegative. Cross-rate prefixes are not matched or relabeled.',
                    'normal_load':'max_normal_load_n is the maximum sum of contact-normal magnitudes; not axial pull force and not wrist force.',
                    'geometry':'max_penetration_mm and straight_max_penetration are contact separation-based numerical overlap diagnostics in mm, not plastic deformation; nominal mesh envelope overlap is a separate quantity.',
                    'whole_peak':'straight_max_wrist_force_n and matched_realign_max_wrist_force_n include copied reference t=0, STOP and all executed phases; retreat fields cover retreat only.',
                    'means':'100 ms values are maxima of full within-phase averaging windows, not single-frame peaks. reference_*_mean100ms_within_phase_n takes the maximum of the approach/insert/settle/tilt/hold phase maxima, never averaging across phase boundaries.',
                    'policy':'matched_realign_* is populated only for within-run endpoint + full-prefix matched, numerically valid, label-eligible policy outcomes. Realign recenters XY and aligns orientation. paired_costs_complete separately marks two completed uncensored clearances.',
                    'grasp':'The existing grasp gate is a relative-slip threshold, not proof the object fell out.',
                    'pending':'Only complete attempts are opened. Other case measurements and initial differences remain null; null never denotes zero or a failed withdrawal.',
                    'source_audit':'Archived source files are hashed without execution; configuration equality is reported independently from case results.'},
                provenance=dict(exporter=str(Path(__file__).resolve()), exporter_sha256=_hash(Path(__file__).read_bytes()),
                                summary_helper_sha256=_hash((ROOT/'simulation/summarize_mechanics_study.py').read_bytes()),
                                completed_input_files=files))


def write_comparison(result, output_dir):
    output_dir = Path(output_dir).resolve()
    for name in ('study_a', 'study_b'):
        source = Path(result[name]['path'])
        if source == output_dir or source in output_dir.parents:
            raise ValueError('Output directory must be outside both input studies')
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir/'comparison.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    rows = []
    for case in result['cases']:
        row = dict(case_id=case['case_id'], physical_plan_equal=case['physical_plan_equal'], both_complete=case['both_complete'])
        for key in ('target_depth_mm', 'pair_static_friction', 'pair_dynamic_friction', 'tilt_amplitude_deg'):
            for side in ('a', 'b'):
                row[f'{side}_{key}'] = case[f'physical_parameters_{side}'].get(key)
        for side in ('a', 'b'):
            entry = case[side]
            row[f'{side}_physics_and_control_hz'] = result[f'study_{side}']['rates']['physics_hz']
            row[f'{side}_status'] = entry['status']
            measurements = entry['measurements'] or {}
            for key in (*RESULT_FIELDS, *('matched_realign_'+key for key in MATCHED_FIELDS)):
                row[f'{side}_{key}'] = measurements.get(key)
        # Fixed columns exist even in a fully pending comparison.
        initial_keys = initial_differences({}, {}).keys()
        differences = case['initial_state_differences_b_minus_a'] or {}
        row.update({'initial_'+key:differences.get(key) for key in initial_keys})
        rows.append(row)
    with (output_dir/'comparison.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['case_id'])
        writer.writeheader(); writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-a', required=True, type=Path, help='Reference study, e.g. 240 Hz boundary')
    parser.add_argument('--study-b', required=True, type=Path, help='Comparison study, e.g. 480 Hz boundary')
    parser.add_argument('--output-dir', required=True, type=Path, help='Separate review directory')
    args = parser.parse_args()
    result = compare_studies(args.study_a, args.study_b)
    write_comparison(result, args.output_dir)
    print(json.dumps(dict(common_planned_cases=result['common_planned_cases'],
                          both_completed_cases=result['both_completed_cases'],
                          all_common_physical_plans_equal=all(c['physical_plan_equal'] for c in result['cases']),
                          configuration_equal={k:v['equal'] for k, v in result['configuration_audit'].items()},
                          output_dir=str(args.output_dir.resolve())), indent=2))


if __name__ == '__main__':
    main()
