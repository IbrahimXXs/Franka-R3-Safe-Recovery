"""Read-only consistency audit of completed mechanics output.

This validates saved evidence, not physical realism or the research hypothesis.
Pending/interrupted attempts are counted but their actively written files are
never opened. --deep additionally checks CSV/contact streams and motion data.
Only the standard library is used; archived simulation code is never executed.
"""
import argparse
from collections import Counter
import csv
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


class Audit:
    def __init__(self):
        self.checks = 0
        self.errors = []
        self.files = []
        self.streams = []

    def check(self, okay, code, location, detail=''):
        self.checks += 1
        if not okay:
            self.errors.append(dict(code=code, location=str(location), detail=str(detail)))
        return bool(okay)

    def close(self, actual, expected, code, location, tolerance=1e-6):
        return self.check(_finite(actual) and _finite(expected) and
                          math.isclose(float(actual), float(expected), rel_tol=1e-6, abs_tol=tolerance),
                          code, location, f'actual={actual!r}, expected={expected!r}')

    def file(self, path):
        if not self.check(path.is_file(), 'missing_file', path):
            return False
        self.files.append(dict(path=str(path), sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        return True


def _finite(value):
    if isinstance(value, bool) or value is None:
        return False
    try:
        return math.isfinite(float(value))
    except (ValueError, TypeError):
        return False


def _triples(value):
    if isinstance(value, list) and len(value) == 3 and all(_finite(x) for x in value):
        return [value]
    if isinstance(value, list):
        return [row for child in value for row in _triples(child)]
    return []


def _factor_key(case):
    return tuple(case.get(key) for key in ('target_depth_mm', 'pair_static_friction', 'pair_dynamic_friction'))


def _material(audit, attempt):
    where = attempt['case_id']
    state = attempt.get('material') or {}
    runtime = state.get('runtime_materials') or {}
    shapes = {name: _triples(runtime.get(name)) for name in ('hole', 'peg', 'robot')}
    for name in shapes:
        audit.check(bool(shapes[name]), 'material_runtime_missing', where, name)
        if name != 'robot':
            audit.check(len(shapes[name]) == 1, 'material_shape_count', where, name)
    bindings = state.get('bindings') or {}
    for index, kind in enumerate(('static', 'dynamic')):
        target = attempt[f'pair_{kind}_friction']
        audit.close(state.get(f'requested_pair_{kind}_friction'), target, 'material_requested_pair', where)
        audit.close(state.get(f'effective_pair_{kind}_friction'), target, 'material_effective_pair', where)
        audit.close(attempt.get(f'peg_{kind}_friction'), .75, 'peg_plan_friction', where)
        audit.close(attempt.get(f'hole_{kind}_friction'), 2*target-.75, 'hole_plan_friction', where)
        for name in ('hole', 'peg', 'robot'):
            expected = 2*target-.75 if name == 'hole' else .75
            for shape in shapes[name]:
                audit.close(shape[index], expected, 'material_runtime_friction', where, tolerance=1e-6)
            if name != 'robot':
                audit.close(bindings.get(name, {}).get(f'usd_{kind}_friction'), expected, 'material_usd_friction', where)
        if shapes['hole'] and shapes['peg']:
            audit.close((shapes['hole'][0][index]+shapes['peg'][0][index])/2,
                        target, 'material_average_readback', where)
    for name, rows in shapes.items():
        for shape in rows:
            audit.close(shape[2], 0., 'material_restitution', f'{where}/{name}')
    audit.check(attempt.get('friction_combine_mode') == 'average', 'material_plan_mode', where)
    for name in ('hole', 'peg'):
        binding = bindings.get(name, {})
        audit.check(binding.get('resolved_friction_combine_mode') == 'average', 'material_resolved_mode', f'{where}/{name}')
        audit.check(binding.get('bound_before_physics_initialization') is True, 'material_binding_timing', f'{where}/{name}')
    audit.check(bool(bindings.get('hole', {}).get('material_path')) and
                bindings.get('hole', {}).get('material_path') != bindings.get('peg', {}).get('material_path'),
                'material_independent_bindings', where)


def _recovery_labels(audit, attempt, recovery, policy):
    where = f"{attempt['case_id']}/{policy}"
    if not audit.check(bool(recovery), 'missing_recovery_record', where):
        return
    numerical = recovery.get('numerically_valid')
    eligible = recovery.get('label_eligible')
    safe = recovery.get('safe_recovery')
    reason = recovery.get('reason', recovery.get('termination_reason'))
    reference_valid = (attempt.get('metrics') or {}).get('numerically_valid') is True
    matching = True if policy == 'straight' else (recovery.get('replay_matched') is True and
                                                 recovery.get('replay_prefix_matched') is True and
                                                 recovery.get('replay_prefix_numerically_valid') is True)
    can_label = reference_valid and numerical is True and matching and reason != 'numerical_guard'
    audit.check(not eligible or can_label, 'ineligible_recovery_labeled', where)
    audit.check((type(safe) is bool) == (eligible is True), 'unknown_changed_to_boolean', where)
    if not can_label:
        audit.check(safe is None and eligible is False, 'unknown_recovery_became_negative', where)
    if safe is True:
        audit.check(recovery.get('cleared') is True and recovery.get('grasp_retained') is True and
                    recovery.get('force_budget_exceeded') is False and recovery.get('torque_budget_exceeded') is False,
                    'safe_recovery_conditions', where)
        audit.check(reason == 'cleared' and recovery.get('recovery_censored') is False, 'safe_recovery_termination', where)
    if recovery.get('replay_prefix_matched') is True:
        audit.check(recovery.get('replay_matched') is True, 'prefix_match_endpoint_conflict', where)
        audit.check(recovery.get('replay_prefix_first_mismatch_index') is None, 'prefix_match_index_conflict', where)
    if reason in ('replay_mismatch', 'invalid_replay_prefix', 'reference_not_eligible'):
        audit.check(safe is None and eligible is False, 'untested_policy_has_outcome', where)
    if reason == 'reference_not_eligible':
        metrics = attempt.get('metrics') or {}
        reference_eligible = all(metrics.get(key) is True for key in
                                 ('numerically_valid', 'reference_complete', 'grasp_retained', 'reference_within_budget'))
        audit.check(not reference_eligible, 'eligible_reference_marked_untested', where)
    if reason == 'invalid_replay_prefix':
        audit.check(recovery.get('replay_prefix_numerically_valid') is False, 'invalid_prefix_reason_conflict', where)


def _csv_rows(audit, path):
    if not audit.file(path):
        return []
    with path.open(newline='') as stream:
        rows = list(csv.DictReader(stream))
    audit.check(bool(rows), 'empty_csv', path)
    return rows


def _stream(audit, folder, stem, dt, attempt, recovery=None, fixed_speed=False, protocol=None):
    """Compare independently saved records at every sample, including zeros."""
    path = folder/f'{stem}.csv'
    rows = _csv_rows(audit, path)
    contact_path = folder/f'{stem}_contacts.jsonl.gz'
    if not rows or not audit.file(contact_path):
        return rows
    with gzip.open(contact_path, 'rt') as stream:
        contacts = [json.loads(line) for line in stream]
    audit.check(len(rows) == len(contacts), 'contact_csv_length', path,
                f'csv={len(rows)}, contact={len(contacts)}')
    audit.streams.append(dict(path=str(path), csv_samples=len(rows), contact_samples=len(contacts)))
    t0 = float(rows[0]['time_s'])
    physics0 = float(rows[0]['physics_time_s'])
    is_recovery = recovery is not None
    for i, (row, raw) in enumerate(zip(rows, contacts)):
        where = f'{path}:sample={i}'
        t = float(row['time_s'])
        physics = float(row['physics_time_s'])
        audit.close(t-t0, i*dt, 'sample_clock', where, tolerance=1e-5)
        audit.close(physics-physics0, i*dt, 'physics_clock', where, tolerance=1e-5)
        audit.close(raw.get('time_s'), t, 'contact_clock', where, tolerance=1e-8)
        audit.close(raw.get('physics_time_s'), physics, 'contact_physics_clock', where, tolerance=1e-8)
        audit.close(raw.get('recording_time_s'), t-t0, 'contact_recording_clock', where, tolerance=1e-8)
        audit.check(row['phase'] == raw.get('phase'), 'contact_phase', where)
        snapshot = raw.get('contacts') or {}
        audit.close(snapshot.get('time_s'), t, 'snapshot_clock', where, tolerance=1e-8)
        audit.close(snapshot.get('physics_time_s'), physics, 'snapshot_physics_clock', where, tolerance=1e-8)
        # Recovery sample zero is the preceding physical contact snapshot, now
        # recorded as stop. Its inner phase intentionally remains hold/insert.
        if not (is_recovery and i == 0):
            audit.check(snapshot.get('phase') == row['phase'], 'snapshot_phase', where)
        for kind in ('static', 'dynamic'):
            audit.close(row.get(f'effective_pair_{kind}_friction'), attempt[f'pair_{kind}_friction'], 'sample_material_pair', where)
        for axis in 'xyz':
            total = float(row[f'f{axis}'])
            normal = float(row[f'normal_force_world_{axis}_n'])
            friction = float(row[f'friction_force_world_{axis}_n'])
            audit.close(normal+friction, total, 'force_component_sum', where, tolerance=2e-5)
        for kind in ('normal', 'friction'):
            points = snapshot.get(kind+'_contacts', [])
            count = snapshot.get(kind+'_contact_count_all')
            audit.check(type(count) is int and count >= len(points), 'raw_contact_count', where, kind)
            # Raw exports omit loads <= 1e-6 N; permit the bounded omitted
            # total plus float32 sum roundoff, not arbitrary force residuals.
            tolerance = max(0, (count or 0)-len(points))*1e-6+2e-5
            for axis_index, axis in enumerate('xyz'):
                total = sum(point['normal_load_n']*point['normal_world'][axis_index] if kind == 'normal'
                            else point['force_world_n'][axis_index] for point in points)
                audit.close(row.get(f'{kind}_force_world_{axis}_n'), total, 'raw_contact_force_sum', where, tolerance=tolerance)
        if is_recovery:
            audit.close(row.get('recovery_time_s'), t-t0, 'recovery_clock', where, tolerance=1e-5)
        if is_recovery and fixed_speed:
            _motion_sample(audit, row, rows[0], recovery, protocol, where, i)
    return rows


def _motion_sample(audit, row, first, recovery, protocol, where, index):
    """Check displacement against the analytic integral of the stated velocity.

    Kept independent of WithdrawalProfile and the frozen runner imports.
    """
    meta = recovery.get('recovery_motion') or {}
    distance = max(0., float(first['depth_mm'])+10.)
    speed, ramp = 5., .5
    if 0 < distance < speed*ramp:
        ramp = math.sqrt(distance*.1)
        speed = 10*ramp
    duration = distance/speed+ramp if distance else 0.
    if index == 0:
        audit.close(meta.get('distance_mm'), distance, 'motion_distance', where)
        audit.close(meta.get('requested_speed_mm_s'), 5., 'motion_requested_speed', where)
        audit.close(meta.get('requested_ramp_s'), .5, 'motion_requested_ramp', where)
        audit.close(meta.get('duration_s'), duration, 'motion_duration', where)
        audit.close(meta.get('max_speed_mm_s'), speed if distance else 0., 'motion_max_speed', where)
    start = protocol['stop_duration_s'] + (protocol['realign_duration_s'] if recovery.get('policy') == 'realign' else 0.)
    elapsed = float(row['recovery_time_s'])-start
    if elapsed <= 0 or not distance:
        position = velocity = 0.
    elif elapsed >= duration:
        position, velocity = distance, 0.
    else:
        tail = min(elapsed, duration-elapsed)
        if tail < ramp:
            u = tail/ramp
            velocity = speed*(3*u*u-2*u*u*u)
            endpoint_distance = speed*ramp*(u**3-u**4/2)
            position = endpoint_distance if elapsed <= duration/2 else distance-endpoint_distance
        else:
            velocity = speed
            position = speed*(elapsed-ramp/2)
    audit.close(row.get('command_withdrawal_mm'), position, 'motion_position', where, tolerance=1e-7)
    audit.close(row.get('command_withdrawal_speed_mm_s'), velocity, 'motion_velocity', where, tolerance=1e-7)
    t = float(row['recovery_time_s'])
    expected_phase = ('stop' if t <= protocol['stop_duration_s'] else 'realign' if t <= start else
                      'retreat' if elapsed <= duration else 'clear_hold')
    audit.check(row['phase'] == expected_phase, 'motion_phase', where)
    if index:
        audit.close(row.get('command_depth_mm'), float(first['depth_mm'])-position, 'motion_command_depth', where)


def _prefix(audit, original, replay, probe, where, protocol):
    audit.check(probe.get('replay_sample_count') == len(replay), 'replay_sample_count', where)
    if 'reference_sample_count' in probe:
        audit.check(probe['reference_sample_count'] == len(original), 'reference_sample_count', where)
    if probe.get('replay_prefix_matched') is True:
        audit.check(len(original) == len(replay), 'matched_prefix_length', where)
        for i, (a, b) in enumerate(zip(original, replay)):
            for key in ('phase', 'command_depth_mm', 'command_pitch_deg', 'command_offset_x_mm', 'command_offset_y_mm', 'reference_step'):
                audit.check(a.get(key) == b.get(key), 'matched_prefix_command', f'{where}:sample={i}', key)
            audit.close(a.get('time_s'), b.get('time_s'), 'matched_prefix_clock', f'{where}:sample={i}', tolerance=1e-5)
    if probe.get('replay_prefix_equal') is True:
        ignored = {'time_s', 'physics_time_s'}
        equal = len(original) == len(replay) and all(
            all(a.get(key) == b.get(key) for key in (set(a)|set(b))-ignored) for a, b in zip(original, replay))
        audit.check(equal, 'bitwise_prefix_equality_claim', where)
    if original and replay:
        from research.forge_mechanics_protocol import compare_prefixes
        check = compare_prefixes(_typed_rows(original), _typed_rows(replay), SimpleNamespace(**protocol))
        for key in ('replay_matched', 'replay_prefix_matched', 'replay_prefix_equal', 'replay_prefix_first_mismatch_index'):
            audit.check(probe.get(key) == check[key], 'recomputed_replay_metadata', where, key)


def _typed_rows(rows):
    result = []
    for raw in rows:
        row = {}
        for key, value in raw.items():
            if value == '':row[key] = None
            elif value == 'True':row[key] = True
            elif value == 'False':row[key] = False
            else:
                try:row[key] = float(value)
                except (TypeError, ValueError):row[key] = value
        result.append(row)
    return result


def control_prefix_checks(plan, completed, reference_rows, protocol, *, deep, excluded=()):
    """Analysis-only pairing audit; mismatch does not invalidate the dataset."""
    from research.forge_mechanics_protocol import compare_prefixes
    controls = {_factor_key(case):case for case in plan if case.get('tilt_amplitude_deg') == 0}
    done = {attempt['case_id']:attempt for attempt in completed}
    results = []
    for case in plan:
        if not case.get('tilt_amplitude_deg'):
            continue
        control = controls.get(_factor_key(case), {})
        control_id, case_id = control.get('case_id'), case['case_id']
        item = dict(case_id=case_id, control_case_id=control_id, target_depth_mm=case['target_depth_mm'],
                    pair_static_friction=case['pair_static_friction'], pair_dynamic_friction=case['pair_dynamic_friction'],
                    analysis_prefix_matched=None, exact_prefix_equal=None)
        if case_id in excluded:
            item['status'] = 'tilted_case_excluded'
        elif case_id not in done or control_id not in done:
            item['status'] = 'awaiting_two_completed_references'
        elif not deep:
            item['status'] = 'not_checked_requires_deep'
        else:
            gates = [(done[identity].get('metrics') or {}).get('depth_gate_step') for identity in (control_id, case_id)]
            item.update(control_depth_gate_step=gates[0], tilted_depth_gate_step=gates[1])
            if any(not _finite(gate) or float(gate) < 0 for gate in gates):
                item['status'] = 'depth_gate_not_reached'
            elif not all(reference_rows.get(identity) for identity in (control_id, case_id)):
                item['status'] = 'reference_rows_unavailable'
            else:
                try:
                    prefixes = [[row for row in _typed_rows(reference_rows[identity])
                                 if row['reference_step'] <= gate] for identity, gate in zip((control_id, case_id), gates)]
                    check = compare_prefixes(*prefixes, SimpleNamespace(**protocol))
                    item.update(status='checked', analysis_prefix_matched=check['replay_prefix_matched'],
                                exact_prefix_equal=check['replay_prefix_equal'],
                                control_sample_count=len(prefixes[0]), tilted_sample_count=len(prefixes[1]),
                                checks=check,
                                interpretation='Pre-tilt physical/command prefixes match within configured tolerances; exact equality is diagnostic.'
                                if check['replay_prefix_matched'] else
                                'Pre-tilt prefixes differ; factor comparisons are descriptive and must not be called strict paired causal effects.')
                except (ValueError, KeyError, TypeError, ZeroDivisionError) as error:
                    item.update(status='analysis_check_unavailable', detail=repr(error))
        results.append(item)
    return results


def validate_study(study_dir, *, deep=False):
    """Validate one manifest snapshot and only its closed completed files."""
    directory = Path(study_dir).resolve()
    manifest_bytes = (directory/'study.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    audit = Audit()
    audit.check(manifest.get('study') == 'Forge-mechanics-v1', 'study_schema', directory)
    plan = (manifest.get('case_plan') or {}).get('cases', [])
    ids = [c.get('case_id') for c in plan]
    audit.check(bool(plan) and None not in ids and len(set(ids)) == len(ids), 'plan_unique_cases', directory)
    by_id = {c['case_id']: c for c in plan}
    attempts = manifest.get('attempts', [])
    completed = [a for a in attempts if a.get('status') == 'complete']
    counts = Counter(a.get('case_id') for a in completed)
    audit.check(all(n == 1 for n in counts.values()), 'duplicate_completed_condition', directory, dict(counts))
    audit.check(len({a.get('folder') for a in completed}) == len(completed), 'duplicate_completed_folder', directory)
    done = {a['case_id']: a for a in completed}
    for attempt in attempts:
        audit.check(attempt.get('case_id') in by_id, 'unplanned_attempt', attempt.get('case_id'))
    for relative, expected in (manifest.get('sources') or {}).items():
        path = (directory/'source'/relative).resolve()
        if not audit.check(directory/'source' in path.parents, 'archive_path_outside_source', relative):
            continue
        if audit.file(path):
            audit.check(audit.files[-1]['sha256'] == expected, 'archive_source_sha256', relative)
    audit.check(bool(manifest.get('sources')), 'source_manifest_missing', directory)
    skipped = manifest.get('skipped_cases', [])
    audit.check(len({s['case_id'] for s in skipped}) == len(skipped), 'duplicate_exclusion', directory)
    for skip in skipped:
        case = by_id.get(skip['case_id'], {})
        control = done.get(skip.get('control_case_id'), {})
        audit.check(bool(case) and skip['case_id'] not in done and case.get('tilt_amplitude_deg', 0) > 0,
                    'invalid_excluded_condition', skip['case_id'])
        audit.check(bool(control) and control.get('tilt_amplitude_deg') == 0 and _factor_key(case) == _factor_key(control),
                    'exclusion_control_relationship', skip['case_id'])
        m = control.get('metrics') or {}
        passed = (all(m.get(k) is True for k in ('reference_complete', 'numerically_valid', 'grasp_retained',
                                                'reference_within_budget', 'depth_attained')) and
                  (control.get('final_retreat') or {}).get('safe_recovery') is True)
        audit.check(skip.get('reason') == 'aligned_control_failed' and not passed, 'exclusion_control_not_failed', skip['case_id'])
    motion = manifest.get('recovery_motion')
    fixed_speed = bool(motion)
    if fixed_speed:
        audit.close(motion.get('requested_speed_mm_s'), 5., 'study_withdrawal_speed', directory)
        audit.close(motion.get('ramp_s'), .5, 'study_withdrawal_ramp', directory)
        audit.check(motion.get('inherited_retreat_duration_s_used') is False, 'study_fixed_duration_disabled', directory)
    reference_rows = {}
    for attempt in completed:
        where = attempt['case_id']
        if where not in by_id:
            continue
        folder = (directory/attempt['folder']).resolve()
        if not audit.check(directory in folder.parents, 'attempt_folder_outside_study', where):
            continue
        for key, expected in by_id.get(where, {}).items():
            audit.check(attempt.get(key) == expected, 'attempt_plan_metadata', where, key)
        _material(audit, attempt)
        audit.close(attempt.get('effective_radial_clearance_mm'), (manifest.get('geometry') or {}).get('effective_radial_clearance_mm'),
                    'attempt_effective_gap', where)
        if audit.file(folder/'trajectory.json'):
            audit.check(json.loads((folder/'trajectory.json').read_text()) == attempt, 'trajectory_manifest_agreement', where)
        straight = attempt.get('final_retreat') or {}
        _recovery_labels(audit, attempt, straight, 'straight')
        probes = attempt.get('probes') or []
        audit.check(len(probes) == 1 and probes[0].get('policy') == 'realign', 'realign_probe_record', where)
        for probe in probes:
            _recovery_labels(audit, attempt, probe, 'realign')
        if not deep:
            continue
        try:
            dt = 1/manifest['physics_hz']
            protocol = manifest.get('effective_protocol') or manifest['protocol']
            original = _stream(audit, folder, 'insertion', dt, attempt)
            reference_rows[where] = original
            _stream(audit, folder, 'final_retreat', dt, attempt, dict(straight, policy='straight'), fixed_speed, protocol)
            for probe in probes:
                if probe.get('replay_matched') is not None:
                    replay = _stream(audit, folder, 'terminal_realign_prefix', dt, attempt)
                    _prefix(audit, original, replay, probe, where, protocol)
                if 'numerically_valid' in probe:
                    _stream(audit, folder, 'terminal_realign_recovery', dt, attempt, probe, fixed_speed, protocol)
        except (KeyError, ValueError, TypeError, OSError) as error:
            audit.check(False, 'deep_data_exception', where, repr(error))
    control_checks = control_prefix_checks(plan, completed, reference_rows,
                                          manifest.get('effective_protocol') or manifest.get('protocol', {}),
                                          deep=deep, excluded={s['case_id'] for s in skipped})
    return dict(schema='Forge-mechanics-output-validation-v1', source_study=str(directory),
                source_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(), source_status=manifest.get('status'),
                passed=not audit.errors, deep=deep, planned_cases=len(plan), completed_cases_checked=len(completed),
                skipped_cases_checked=len(skipped), incomplete_attempts_not_opened=len(attempts)-len(completed),
                pending_cases=len(set(ids)-set(done)-{s['case_id'] for s in skipped}),
                check_count=audit.checks, error_count=len(audit.errors), errors=audit.errors,
                checked_files=audit.files, checked_streams=audit.streams,
                control_prefix_checks=control_checks,
                control_prefix_interpretation='Analysis-only checks from initial sample through each reference depth gate. Mismatch limits causal pairing; it does not invalidate observations or change recovery-policy labels.',
                scope='Serialized evidence consistency for completed attempts only; not a certification of physics, sensor calibration, or the research hypothesis.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--deep', action='store_true')
    args = parser.parse_args()
    source = args.study_dir.resolve()
    if args.output_dir:
        destination = args.output_dir.resolve()
        if destination == source or source in destination.parents:
            parser.error('--output-dir must be outside the source study')
    result = validate_study(source, deep=args.deep)
    if args.output_dir:
        destination.mkdir(parents=True, exist_ok=True)
        (destination/'validation.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key:value for key, value in result.items() if key not in ('checked_files', 'checked_streams')}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
