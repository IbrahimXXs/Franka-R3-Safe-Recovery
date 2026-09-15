#!/usr/bin/env python3
"""Audit chronological STOP unloading from completed tilted mechanics cases.

No simulator is imported. Outputs must be outside the live study directory.
This measures what happened during the hold, not a counterfactual no-STOP run.
"""
import argparse
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import shlex

try:
    from .analyze_mechanics_states import load_profile, number, peak_selection, phase_execution
    from .analyze_mechanics_contact_regions import region_measurements
except ImportError:
    from analyze_mechanics_states import load_profile, number, peak_selection, phase_execution
    from analyze_mechanics_contact_regions import region_measurements


SIGNALS = ('normal_load_n', 'wrist_force_n', 'depth_mm', 'tilt_deg',
           'tip_x_mm', 'tip_y_mm', 'grasp_slip_mm', 'grasp_slip_deg',
           'normal_force_world_z_n', 'friction_force_world_z_n')
STATE_SIGNALS = SIGNALS + ('time_s', 'physics_time_s', 'recovery_time_s',
    'min_separation_mm', 'effective_guide_span_estimate_mm',
    'geometry_wall_signed_violation_estimate_mm', 'command_depth_mm',
    'command_tilt_deg', 'command_pitch_deg', 'phase')
REGION_SIGNALS = ('all_loaded_normal_load_n', 'all_loaded_normal_z_span_mm',
    'force_supported_normal_z_span_mm', 'inner_straight_wall_normal_load_n',
    'upper_rim_band_normal_load_n', 'entrance_chamfer_region_normal_load_n',
    'other_normal_load_n', 'wall_rim_pos_normal_load_n', 'wall_rim_neg_normal_load_n',
    'wall_rim_pos_centroid_x_mm', 'wall_rim_neg_centroid_x_mm',
    'wall_rim_pos_centroid_z_mm', 'wall_rim_neg_centroid_z_mm',
    'wall_rim_both_sides_loaded', 'wall_rim_centroid_axial_span_mm')
FRACTIONAL_CHANGE_MIN_BASELINE_N = .01


def executed_indices(rows, phase):
    return [i for i, row in enumerate(rows) if row.get('phase') == phase
            and number(row, 'recovery_time_s') is not None
            and number(row, 'recovery_time_s') > 0.]


def fixed_window(rows, indices, dt, at_end=True):
    """A fixed chronological 100 ms window, never optimized for a peak."""
    count = max(1, math.ceil(.1/dt-1e-9))
    if len(indices) < count:
        return {'observed': False, 'reason': 'fewer_than_100ms_actual_samples'}
    selected = indices[-count:] if at_end else indices[:count]
    times = [number(rows[i], 'time_s') for i in selected]
    if (any(t is None for t in times)
            or any(b != a+1 for a, b in zip(selected, selected[1:]))
            or any(not math.isclose(b-a, dt, abs_tol=1e-7, rel_tol=1e-5)
                   for a, b in zip(times, times[1:]))
            or len({rows[i].get('phase') for i in selected}) != 1):
        return {'observed': False, 'reason': 'window_crosses_gap_or_phase_boundary'}
    result = {'observed': True, 'sample_count': count, 'support_s': count*dt,
              'start_sample_index': selected[0], 'end_sample_index': selected[-1],
              'start_time_s': times[0], 'end_time_s': times[-1],
              'first_last_timestamp_span_s': times[-1]-times[0]}
    for field in SIGNALS:
        values = [number(rows[i], field) for i in selected]
        result['mean_'+field] = sum(values)/count if all(v is not None for v in values) else None
    loads = [number(rows[i], 'normal_load_n') for i in selected]
    result['loaded_sample_fraction_at_0p01n'] = (
        sum(v >= .01 for v in loads)/count if all(v is not None for v in loads) else None)
    return result


def chronological_change(start, end):
    """Changes between corresponding chronological observations, never peaks."""
    if start is None or end is None:
        return None
    result = {}
    for field in SIGNALS:
        a, b = number(start, field), number(end, field)
        result['delta_'+field] = b-a if a is not None and b is not None else None
        if field in ('normal_load_n', 'wrist_force_n'):
            result['fractional_change_'+field] = ((b-a)/a if a is not None
                and a >= FRACTIONAL_CHANGE_MIN_BASELINE_N and b is not None else None)
    dx, dy = result['delta_tip_x_mm'], result['delta_tip_y_mm']
    result['tip_lateral_motion_mm'] = math.hypot(dx, dy) if dx is not None and dy is not None else None
    return result


def window_change(start, end):
    if not start.get('observed') or not end.get('observed'):
        return None
    return chronological_change({key: start.get('mean_'+key) for key in SIGNALS},
                                {key: end.get('mean_'+key) for key in SIGNALS})


def read_selected_raw(path, rows, indices, radius, hashes):
    raw_path = path.with_name(path.stem+'_contacts.jsonl.gz')
    data = raw_path.read_bytes()
    hashes[str(raw_path)] = hashlib.sha256(data).hexdigest()
    decoded = gzip.decompress(data)  # Verify complete gzip EOF/CRC, even for early selections.
    output = {}
    line_count = 0
    for index, line in enumerate(decoded.splitlines()):
        line_count += 1
        if index not in indices:
            continue
        raw = json.loads(line)
        row = rows[index]
        for field in ('time_s', 'physics_time_s'):
            if not math.isclose(float(raw[field]), float(row[field]), abs_tol=1e-8, rel_tol=0):
                raise ValueError(f'Raw/CSV {field} mismatch: {raw_path}:{index}')
        if raw['phase'] != row['phase']:
            raise ValueError(f'Raw/CSV phase mismatch: {raw_path}:{index}')
        regions = region_measurements(raw['contacts']['normal_contacts'], radius)
        output[index] = {key: regions[key] for key in REGION_SIGNALS}
        output[index]['raw_normal_sum_minus_csv_n'] = regions['all_loaded_normal_load_n']-row['normal_load_n']
    if line_count != len(rows) or any(index not in output for index in indices):
        raise ValueError(f'Raw/CSV sample count or selected index mismatch: {raw_path}')
    return output


def event(rows, index, raw, path, policy, dt):
    if index is None:
        return None
    row = rows[index]
    result = {field: row.get(field) for field in STATE_SIGNALS}
    result.update(source_csv=str(path), sample_index=index,
                  **phase_execution(rows, index, policy, dt), **raw[index])
    return result


def terminal_phase_indices(rows):
    if not rows:
        return []
    end = len(rows)-1
    start = end
    while start > 0 and rows[start-1].get('phase') == rows[end].get('phase'):
        start -= 1
    return [i for i in range(start, end+1) if number(rows[i], 'reference_step') != 0]


def export_audit(study_dir, output_dir):
    study_dir, output_dir = Path(study_dir).resolve(), Path(output_dir).resolve()
    if output_dir == study_dir or study_dir in output_dir.parents:
        raise ValueError('Write the audit outside the simulation study')
    manifest_bytes = (study_dir/'study.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    scene_bytes = (study_dir/'scene.json').read_bytes()
    scene = json.loads(scene_bytes)
    radius = float(scene['measured_bore_diameter_mm'])/2000
    dt = 1/float(manifest['physics_hz'])
    expected_stop = float(manifest['protocol']['stop_duration_s'])
    attempts = [a for a in manifest['attempts'] if a.get('status') == 'complete'
                and float(a.get('tilt_amplitude_deg') or 0) != 0]
    hashes, cases, flat, cross_depth = {}, [], [], []
    for attempt in attempts:
        folder = study_dir/attempt['folder']
        ref_path = folder/'insertion.csv'
        ref, hashes[str(ref_path)] = load_profile(ref_path)
        ref_peak = peak_selection(ref, 'normal_load_n')['index']
        ref_raw = read_selected_raw(ref_path, ref, {len(ref)-1, ref_peak}, radius, hashes)
        metadata = {key: attempt.get(key) for key in ('case_id', 'trajectory_id', 'folder',
            'target_depth_mm', 'tilt_amplitude_deg', 'pair_static_friction', 'pair_dynamic_friction')}
        metrics = attempt.get('metrics', {})
        metadata.update(reference_complete=metrics.get('reference_complete'),
                        reference_numerically_valid=metrics.get('numerically_valid'),
                        reference_grasp_retained=metrics.get('grasp_retained'),
                        reference_termination_reason=metrics.get('termination_reason'))
        ref_end = event(ref, len(ref)-1, ref_raw, ref_path, 'reference', dt)
        ref_max = event(ref, ref_peak, ref_raw, ref_path, 'reference', dt)
        ref_window = fixed_window(ref, terminal_phase_indices(ref), dt)
        case = {**metadata, 'reference_terminal': ref_end, 'reference_normal_peak': ref_max,
                'reference_terminal_last100ms': ref_window, 'policies': []}
        for name, value in [('reference_terminal', ref_end), ('reference_normal_peak', ref_max)]:
            cross_depth.append({**metadata, 'state': name, **value})
        probe = next((p for p in attempt.get('probes', []) if p.get('policy') == 'realign'), {})
        policies = [('straight', folder/'final_retreat.csv', attempt.get('final_retreat', {})),
                    ('realign', folder/'terminal_realign_recovery.csv', probe)]
        for policy, path, policy_metadata in policies:
            result = {'policy': policy, 'reason': policy_metadata.get('reason'),
                      'replay_matched': policy_metadata.get('replay_matched'),
                      'safe_recovery': policy_metadata.get('safe_recovery'),
                      'profile_available': path.exists()}
            scalar = {**metadata, 'policy': policy, **{key: result[key] for key in
                      ('reason', 'replay_matched', 'safe_recovery', 'profile_available')}}
            if not path.exists():
                case['policies'].append(result)
                flat.append(scalar)
                continue
            rows, hashes[str(path)] = load_profile(path)
            stops, retreats = executed_indices(rows, 'stop'), executed_indices(rows, 'retreat')
            realigns = executed_indices(rows, 'realign')
            loaded = [i for i in retreats if number(rows[i], 'normal_load_n') is not None
                      and number(rows[i], 'normal_load_n') >= .01]
            selectors = {'recovery_start': 0 if rows else None,
                         'stop_end': stops[-1] if stops else None,
                         'realign_end': realigns[-1] if realigns else None,
                         'retreat_first': retreats[0] if retreats else None,
                         'retreat_first_loaded_at_0p01n': loaded[0] if loaded else None,
                         'retreat_normal_peak': max(retreats, key=lambda i: rows[i]['normal_load_n']) if retreats else None,
                         'retreat_wrist_peak': max(retreats, key=lambda i: rows[i]['wrist_force_n']) if retreats else None}
            selected_raw = read_selected_raw(path, rows, {i for i in selectors.values() if i is not None}, radius, hashes)
            events = {name: event(rows, index, selected_raw, path, policy, dt) for name, index in selectors.items()}
            stop_window = fixed_window(rows, stops, dt)
            retreat_window = fixed_window(rows, retreats, dt, at_end=False)
            # For a replayed policy use its own pre-recovery prefix, not the original branch.
            before_window = ref_window
            before_source = str(ref_path)
            if policy == 'realign':
                prefix_path = folder/'terminal_realign_prefix.csv'
                if prefix_path.exists():
                    prefix, hashes[str(prefix_path)] = load_profile(prefix_path)
                    before_window = fixed_window(prefix, terminal_phase_indices(prefix), dt)
                    before_source = str(prefix_path)
                else:
                    before_window = {'observed': False, 'reason': 'replay_prefix_unavailable'}
                    before_source = None
            changes = chronological_change(events['recovery_start'], events['stop_end'])
            tail_changes = window_change(before_window, stop_window)
            result.update(events=events, stop_executed=bool(stops), stop_step_count=len(stops),
                          stop_observed_duration_s=len(stops)*dt,
                          full_stop_observed=len(stops)*dt >= expected_stop-dt/2,
                          retreat_executed=bool(retreats), before_stop_last100ms=before_window,
                          before_stop_source_csv=before_source, stop_last100ms=stop_window,
                          retreat_first100ms=retreat_window,
                          recovery_start_to_stop_end=changes,
                          before_stop_to_stop_tail_mean_change=tail_changes,
                          reference_to_recovery_start=chronological_change(ref_end, events['recovery_start']))
            if events['retreat_first_loaded_at_0p01n'] and events['stop_end']:
                first_load = events['retreat_first_loaded_at_0p01n']
                result['stop_end_to_first_loaded_retreat'] = chronological_change(events['stop_end'], first_load)
                result['time_after_stop_until_first_loaded_retreat_s'] = first_load['time_s']-events['stop_end']['time_s']
            for field in ('stop_executed', 'stop_step_count', 'stop_observed_duration_s',
                          'full_stop_observed', 'retreat_executed'):
                scalar[field] = result[field]
            for event_name in ('recovery_start', 'stop_end', 'retreat_first', 'retreat_first_loaded_at_0p01n'):
                observation = events[event_name] or {}
                for field in ('normal_load_n', 'wrist_force_n', 'depth_mm', 'tilt_deg',
                              'tip_x_mm', 'tip_y_mm', 'grasp_slip_mm',
                              'upper_rim_band_normal_load_n', 'inner_straight_wall_normal_load_n',
                              'wall_rim_centroid_axial_span_mm', 'wall_rim_both_sides_loaded'):
                    scalar[event_name+'_'+field] = observation.get(field)
            for window_name, values in [('before_stop_last100ms', before_window),
                                        ('stop_last100ms', stop_window), ('retreat_first100ms', retreat_window)]:
                for field in ('mean_normal_load_n', 'mean_wrist_force_n', 'loaded_sample_fraction_at_0p01n'):
                    scalar[window_name+'_'+field] = values.get(field)
            for change_name, values in [('stop_endpoint_change', changes), ('stop_tail_mean_change', tail_changes)]:
                for field in ('delta_normal_load_n', 'delta_wrist_force_n',
                              'fractional_change_normal_load_n', 'fractional_change_wrist_force_n',
                              'delta_depth_mm', 'delta_tilt_deg', 'tip_lateral_motion_mm'):
                    scalar[change_name+'_'+field] = (values or {}).get(field)
            case['policies'].append(result)
            flat.append(scalar)
        cases.append(case)
    for path, digest in hashes.items():
        if hashlib.sha256(Path(path).read_bytes()).hexdigest() != digest:
            raise RuntimeError(f'Completed input changed during audit: {path}')
    groups = {}
    for case in cases:
        pair = (case['pair_static_friction'], case['pair_dynamic_friction'])
        groups.setdefault(pair, []).append(case)
    comparable = [{'pair_static_friction': pair[0], 'pair_dynamic_friction': pair[1],
                   'case_ids_in_depth_order': [c['case_id'] for c in sorted(group, key=lambda c: c['target_depth_mm'])]}
                  for pair, group in groups.items() if len({c['target_depth_mm'] for c in group}) >= 2]
    output_dir.mkdir(parents=True, exist_ok=True)
    report = {'schema': 'mechanics_stop_unloading_v1', 'study_dir': str(study_dir),
              'study_status_at_read': manifest['status'],
              'completed_attempts_at_read': sum(a.get('status') == 'complete' for a in manifest['attempts']),
              'completed_tilted_attempts_reviewed': len(cases),
              'case_ids': [c['case_id'] for c in cases],
              'study_json_sha256_at_read': hashlib.sha256(manifest_bytes).hexdigest(),
              'scene_json_sha256': hashlib.sha256(scene_bytes).hexdigest(),
              'analyzer_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              'helper_sha256': {str(Path(f.__code__.co_filename).resolve()): hashlib.sha256(Path(f.__code__.co_filename).read_bytes()).hexdigest()
                                for f in (load_profile, region_measurements)},
              'input_hashes': hashes, 'physics_hz': 1/dt, 'expected_stop_duration_s': expected_stop,
              'methods': [
                  'Only attempts marked complete at manifest read, with nonzero planned tilt, are audited.',
                  'Actual recovery steps require phase match and recovery_time_s > 0. The copied t=0 observation does not execute STOP.',
                  'Endpoint changes compare each recovery branch initial observation with its actual STOP end, including observed pose change.',
                  'Fractional force changes are reported only when the earlier value is at least .01 N, avoiding large ratios of near-zero loads; absolute changes remain available. This reporting threshold is not a calibrated sensor noise floor.',
                  'Tail changes compare the last contiguous same-phase 100 ms before recovery with the last actual 100 ms of STOP. Replayed policies use their own prefix.',
                  '100 ms is N*dt support, with N=ceil(.1/dt); first-to-last timestamp span is (N-1)*dt. Windows are fixed in time, not maximum windows.',
                  'Contact reloading is first actual retreat observation with summed normal load >= .01 N; this is a declared observation threshold, not a physical onset guarantee.',
                  'Raw region analysis retains normal points >1e-6 N; force-supported span uses individual points >=.01 N; both wall+rim side sums must each reach .01 N.',
                  'All normal regions and wall+rim centroids are estimates from independent normal points, not identified mechanical contact pairs.',
              ],
              'limitations': [
                  'STOP commands the attained hand pose rather than preserving the previous reference target; removing controller tracking error can alter loads while peg angle barely changes.',
                  'A chronological force decrease is observed unloading during this protocol. Without a no-STOP counterfactual it cannot establish the isolated causal effect or statistical significance of STOP.',
                  'Unrelated phase maxima are never divided to claim unloading. Summed contact normal load is not wrist resultant force.',
                  'Skipped STOP/retreat leaves null endpoint/window changes, not zero force. Grasp threshold termination is not proof that the peg dropped or was irrecoverably jammed.',
                  'The same planned friction/depth comparison may have different attained angles, offsets and guard outcomes. This is not a fixed-moment or fixed-angle mechanical experiment.',
                  'Raw/pose within-step solver timing has not been directly guaranteed by the contact API; no global sample shift is applied.',
              ], 'same_friction_multiple_depth_groups': comparable,
              'cross_depth_reference_states': sorted(cross_depth, key=lambda c: (c['pair_static_friction'], c['pair_dynamic_friction'], c['target_depth_mm'], c['state'])),
              'cases': cases}
    (output_dir/'mechanics_stop_unloading.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    fields = list(dict.fromkeys(key for row in flat for key in row))
    with (output_dir/'mechanics_stop_unloading.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows({key: ('true' if value else 'false') if type(value) is bool else value
                         for key, value in row.items()} for row in flat)
    explanation = f'''# STOP 前后载荷离线审计

本次读取时主矩阵有 {report['completed_attempts_at_read']} 个完成案例；本表只分析其中 {len(cases)} 个倾斜案例。运行中案例不读取。重新运行同一命令即可更新。

```bash
python3 simulation/analyze_mechanics_stop_unloading.py --study-dir {shlex.quote(str(study_dir))} --output-dir {shlex.quote(str(output_dir))}
```

CSV 每行是一项恢复策略。JSON 保存相应端点、固定 100 ms 窗口、原始接触分区和输入哈希。空值表示没有可用观测，不能读作零载荷。

`stop_executed=false` 表示没有新的 STOP 物理步，通常只保存了恢复 t=0 的参考末端复制行。`full_stop_observed=true` 才说明记录到设定的 {expected_stop:g} s STOP。

`stop_endpoint_change` 比较同一恢复分支的 t=0 与真实 STOP 末端；`stop_tail_mean_change` 比较恢复前最后 100 ms 与 STOP 最后 100 ms。负值表示后者降低。这些是时序变化，不是相隔时刻峰值的比值。短时零接触可能伴随脉冲，应同时看窗口均值与载荷占空比。

STOP 将控制目标切换为当时测得的手部位姿，可能释放此前的跟踪误差。即使销角度几乎不变，接触载荷也可以改变。这里没有无 STOP 对照，不能将下降单独归因为保持动作或声称统计显著。

同摩擦不同深度的参考末端和法向载荷峰位置见 JSON 的 `cross_depth_reference_states`。需要同时比较实际角度、偏移、夹持滑移和终止原因；上缘与反侧内壁的受载区跨度不是唯一识别出的“两点接触距离”。
'''
    (output_dir/'mechanics_stop_unloading.md').write_text(explanation)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    result = export_audit(args.study_dir, args.output_dir)
    print(json.dumps({key: result[key] for key in ('completed_attempts_at_read',
        'completed_tilted_attempts_reviewed', 'case_ids', 'same_friction_multiple_depth_groups')}, indent=2))


if __name__ == '__main__':
    main()
