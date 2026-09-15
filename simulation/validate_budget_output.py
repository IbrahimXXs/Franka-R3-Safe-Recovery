"""Read-only audit of closed prospective budget branches; no simulator imports.

--deep checks every CSV/contact sample and recomputes observations, budget
events, state gates, recovery labels and independent-process prefix matches.
Archived code is hashed, never executed. Pending files are never opened.
"""
import argparse
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from research.forge_budget import load_plan, evidence, classify, reference_eligible
from research.forge_events import recovery_termination
from research.forge_mechanics_metrics import mechanics_reference_metrics, mechanics_recovery_metrics
from research.forge_mechanics_plan import MechanicsState
from research.forge_mechanics_protocol import reference_quality, compare_prefixes
from research.forge_protocol import load_protocol, screened, retained, within_budget, clear, recovery_summary
from simulation.validate_mechanics_output import Audit, _material, _stream, _typed_rows


def _json(path, data=None):
    def unique(pairs):
        result = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(f'Duplicate JSON key: {key}')
            result[key] = value
        return result
    def invalid(value):
        raise ValueError(f'Non-finite JSON value: {value}')
    return json.loads(Path(path).read_bytes() if data is None else data,
                      object_pairs_hook=unique, parse_constant=invalid)


def _canonical(value):
    return json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':'))


def _same(actual, expected):
    """Tolerant numerical recomputation; booleans/null remain type-sensitive."""
    if isinstance(expected, bool) or expected is None:
        return type(actual) is type(expected) and actual == expected
    if isinstance(expected, (int, float)):
        return type(actual) in (int, float) and math.isfinite(actual) and math.isclose(
            actual, expected, rel_tol=1e-8, abs_tol=1e-8)
    if isinstance(expected, dict):
        return isinstance(actual, dict) and set(actual) == set(expected) and all(
            _same(actual[key], value) for key, value in expected.items())
    if isinstance(expected, list):
        return isinstance(actual, list) and len(actual) == len(expected) and all(map(_same, actual, expected))
    return type(actual) is type(expected) and actual == expected


def _subset(audit, actual, expected, code, where):
    bad = [key for key, value in expected.items() if key not in actual or not _same(actual[key], value)]
    audit.check(not bad, code, where, ', '.join(bad[:20]))


def _inside(audit, root, relative, code):
    path = (root/relative).resolve()
    return path if audit.check(root.resolve() in path.parents, code, relative) else None


def _hashed(audit, path, expected, code):
    if audit.file(path):
        audit.check(audit.files[-1]['sha256'] == expected, code, path)


def _sources(audit, folder, sources):
    audit.check(isinstance(sources, dict) and bool(sources), 'source_manifest_missing', folder)
    for relative, digest in (sources or {}).items():
        path = _inside(audit, folder/'source', relative, 'source_path_outside_archive')
        if path is not None:
            _hashed(audit, path, digest, 'source_sha256')


def budget_checks(audit, rows, saved, metrics, protocol, dt, segment, where):
    """Recompute raw thresholds and forbid samples after the first crossing."""
    result = evidence(rows, protocol, dt, segment)
    audit.check(_same(saved, result), 'recomputed_budget_evidence', where)
    audit.check(result['budget_signals_finite'], 'budget_signal_not_finite', where)
    _subset(audit, metrics, {key: result[key] for key in (
        'force_budget_exceeded', 'torque_budget_exceeded', 'max_wrist_force_n',
        'max_wrist_torque_nm', 'budget_signals_finite')}, 'budget_metrics_disagree', where)
    crossing = result['first_budget_crossing']
    if crossing is not None:
        audit.check(crossing['index'] == len(rows)-1, 'steps_after_budget_crossing', where)
    for i, row in enumerate(rows):
        for signal, indices in (('wrist_force_n', range(3)), ('wrist_torque_nm', range(3, 6))):
            values = [row.get(f'wrist_raw_child_{j}') for j in indices]
            if all(type(value) in (int, float) and math.isfinite(value) for value in values):
                audit.close(row.get(signal), math.sqrt(sum(value*value for value in values)),
                            'raw_wrist_norm', f'{where}:sample={i}', tolerance=2e-5)
            else:
                audit.check(False, 'raw_wrist_components_missing', f'{where}:sample={i}', signal)
    return result


def _guarded(row):
    return row['force_norm_n'] > 500 or row['min_separation_mm'] < -1.


def reference_checks(audit, rows, saved, case, protocol, dt, where):
    state = MechanicsState(case)
    audit.close(rows[0]['time_s'], 0., 'reference_initial_clock', where)
    audit.close(rows[0]['physics_time_s'], 0., 'reference_initial_physics_clock', where)
    for i, row in enumerate(rows):
        location = f'{where}:sample={i}'
        audit.close(row.get('reference_step'), i, 'reference_step_index', location)
        if i:
            previous = rows[i-1]
            audit.check(not (_guarded(previous) or not within_budget(previous, protocol) or state.done),
                        'reference_step_after_termination', location)
            command = state.command(i*dt)
            _subset(audit, row, dict(phase=command['phase'], command_depth_mm=command['depth_mm'],
                                    command_pitch_deg=command['pitch_deg'],
                                    command_tilt_deg=abs(command['pitch_deg']),
                                    command_offset_x_mm=0., command_offset_y_mm=0.),
                    'reference_command', location)
        else:
            audit.check(row.get('phase') == 'initial', 'reference_initial_phase', where)
        state.observe(i*dt, row['depth_mm'], screened(row, protocol), retained(row), physics_step=i)
        _subset(audit, row, state.as_dict(), 'reference_state', location)
    last = rows[-1]
    if _guarded(last): reason = 'numerical_guard'
    elif not within_budget(last, protocol): reason = 'operational_budget_exceeded'
    elif state.done: reason = 'completed' if state.termination_reason == 'reference_complete' else state.termination_reason
    else:
        reason = 'reference_time_limit'
        audit.close(len(rows)-1, round(30./dt), 'reference_timeout_length', where)
    recomputed = {**mechanics_reference_metrics(rows, case, dt), **state.as_dict(),
                  **reference_quality(rows, protocol, reason)}
    recomputed['budget_signals_finite'] = evidence(rows, protocol, dt)['budget_signals_finite']
    _subset(audit, saved, recomputed, 'recomputed_reference_metrics', where)
    return recomputed


def recovery_checks(audit, rows, reference, saved, protocol, dt, where):
    overrides = {'phase', 'recovery_time_s', 'command_withdrawal_mm', 'command_withdrawal_speed_mm_s'}
    _subset(audit, rows[0], {k:v for k,v in reference[-1].items() if k not in overrides},
            'recovery_initial_copy', where)
    audit.close(rows[0].get('recovery_time_s'), 0., 'recovery_initial_clock', where)
    audit.check(rows[0].get('phase') == 'stop', 'recovery_initial_phase', where)
    dwell = math.ceil(.2/dt-1e-9)
    def terminal(prefix):
        row = prefix[-1]
        if _guarded(row): return 'numerical_guard'
        if not screened(row, protocol): return 'numerical_screen_failed'
        if not within_budget(row, protocol): return 'operational_budget_exceeded'
        if not retained(row): return 'grasp_retention_limit'
        if len(prefix) >= dwell and all(clear(r) for r in prefix[-dwell:]): return 'cleared'
        return None
    for i in range(1, len(rows)):
        audit.check(terminal(rows[:i]) is None, 'recovery_step_after_termination', f'{where}:sample={i}')
    reason = terminal(rows) or 'time_budget_exhausted'
    recomputed = recovery_summary(rows, protocol, dt, prefix_valid=True)
    recomputed.update(mechanics_recovery_metrics(rows, dt), reason=reason,
                      **recovery_termination(reason), termination_phase=rows[-1]['phase'])
    if reason == 'numerical_guard':
        recomputed.update(numerically_valid=False, label_eligible=False, safe_recovery=None)
    recomputed['budget_signals_finite'] = evidence(rows, protocol, dt, 'recovery')['budget_signals_finite']
    _subset(audit, saved, recomputed, 'recomputed_recovery_metrics', where)
    return recomputed


def _pairs(conditions, records, rows, deep):
    result = []
    for condition in conditions:
        identity = condition['condition_id']
        a, b = (records.get((identity, policy)) for policy in ('straight', 'realign'))
        item = dict(condition_id=identity, full_reference_prefix_matched=None,
                    paired_policy_comparison_eligible=False,
                    straight_actual_outcome=a.get('outcome') if a else None,
                    realign_actual_outcome=b.get('outcome') if b else None)
        if not a or not b:
            item['status'] = 'two_completed_branches_unavailable'
        elif not all(reference_eligible(x['reference']['metrics']) for x in (a,b)):
            item['status'] = 'reference_not_eligible_for_pairing'
        elif not deep:
            item['status'] = 'requires_deep_prefix_check'
        elif not all(rows.get((identity, policy)) for policy in ('straight', 'realign')):
            item['status'] = 'reference_rows_unavailable'
        else:
            try:
                check = compare_prefixes(rows[(identity,'straight')], rows[(identity,'realign')],
                                         SimpleNamespace(**a['effective_protocol']))
                item.update(status='checked', checks=check,
                            full_reference_prefix_matched=check['replay_prefix_matched'],
                            paired_policy_comparison_eligible=check['replay_prefix_matched'])
            except (KeyError, ValueError, TypeError, ZeroDivisionError) as error:
                item.update(status='comparison_unavailable', detail=repr(error))
        result.append(item)
    return result


def validate_study(study_dir, *, deep=False):
    directory = Path(study_dir).resolve()
    manifest_bytes = (directory/'study.json').read_bytes()
    manifest = _json(directory/'study.json', manifest_bytes)
    audit = Audit()
    audit.check(manifest.get('schema') == 'Forge-budget-v1', 'study_schema', directory)
    plan = manifest.get('case_plan') or {}
    if audit.file(directory/'case_plan.json'):
        try:
            # The loader accepts only the exact approved canonical plans,
            # including numeric types, order and selection provenance.
            expected_plan = load_plan(directory/'case_plan.json')
            audit.check(True, 'canonical_plan', directory)
            audit.check(_canonical(expected_plan) == _canonical(plan), 'plan_file_agreement', directory)
        except (ValueError, TypeError, OSError) as error:
            audit.check(False, 'canonical_plan', directory, repr(error))
    _hashed(audit, directory/'base_protocol.json', manifest.get('base_protocol_sha256'), 'base_protocol_sha256')
    base_protocol = load_protocol(directory/'base_protocol.json')
    _sources(audit, directory, manifest.get('sources'))
    # The exact schema uses these archived sources; require the complete set.
    from simulation.run_budget_study import SOURCE_FILES
    audit.check(set(manifest.get('sources', {})) == set(SOURCE_FILES), 'source_manifest_members', directory)
    conditions = plan.get('conditions', [])
    by_id = {c['condition_id']:c for c in conditions}
    attempts = manifest.get('attempts', [])
    keys = [(a.get('condition_id'), a.get('policy')) for a in attempts]
    audit.check(len(set(keys)) == len(keys), 'duplicate_branch', directory)
    planned_keys = {(c['condition_id'], p) for c in conditions for p in ('straight','realign')}
    audit.check(set(keys) <= planned_keys, 'unplanned_branch', directory)
    closed = [a for a in attempts if a.get('status') == 'complete']
    audit.check(len({a.get('folder') for a in closed}) == len(closed), 'duplicate_branch_folder', directory)
    if manifest.get('status') == 'complete':
        audit.check(set(keys) == planned_keys and all(a.get('status') in ('complete','not_tested') for a in attempts),
                    'complete_study_missing_results', directory)
    records, references, observed = {}, {}, []
    for attempt in closed:
        key = (attempt.get('condition_id'), attempt.get('policy'))
        where = '/'.join(str(x) for x in key)
        if key not in planned_keys: continue
        condition = by_id[key[0]]; case = condition['case']
        folder = _inside(audit, directory, attempt['folder'], 'branch_outside_study')
        if folder is None: continue
        try:
            _hashed(audit, folder/'run.json', attempt.get('run_sha256'), 'run_sha256')
            record = _json(folder/'run.json'); records[key] = record
            audit.check(record.get('schema') == 'Forge-budget-branch-v1' and record.get('status') == 'complete',
                        'branch_schema_status', where)
            audit.check(_same(record.get('condition'), condition) and record.get('policy') == key[1], 'branch_plan', where)
            audit.check(_same(record.get('outcome'), attempt.get('outcome')), 'branch_manifest_outcome', where)
            audit.check(record.get('sources') == manifest.get('sources'), 'branch_sources_agreement', where)
            _sources(audit, folder, record.get('sources'))
            audit.check(_same(record.get('budget_definition'), plan.get('budget_definition')), 'branch_budget_definition', where)
            p = replace(base_protocol, seed=condition['seed'], force_budget_n=condition['force_budget_n'],
                        torque_budget_nm=condition['torque_budget_nm'])
            audit.check(_canonical(record.get('protocol')) == _canonical(asdict(p)), 'branch_base_protocol', where)
            audit.close(record.get('physics_hz'), condition['physics_hz'], 'branch_physics_rate', where)
            audit.check(record.get('seed') == condition['seed'], 'branch_seed', where)
            recovery = record.get('recovery')
            required = {'config.json','scene.json','insertion.csv','insertion_contacts.jsonl.gz'}
            if recovery is not None: required |= {'recovery.csv','recovery_contacts.jsonl.gz'}
            hashes = record.get('artifact_sha256') or {}
            audit.check(set(hashes) == required, 'artifact_manifest_members', where)
            for name in required:
                _hashed(audit, folder/name, hashes.get(name), 'artifact_sha256')
            config, scene = _json(folder/'config.json'), _json(folder/'scene.json')
            gap = scene['conservative_radial_clearance_mm']
            p = replace(p, effective_radial_clearance_mm=gap).validate()
            audit.check(_canonical(record.get('effective_protocol')) == _canonical(asdict(p)), 'effective_protocol', where)
            audit.close(config['sim']['dt'], 1/condition['physics_hz'], 'config_physics_dt', where)
            audit.check(record.get('geometry',{}).get('socket_mesh_sha256') == scene.get('socket_mesh_sha256'), 'geometry_hash', where)
            audit.close(record.get('geometry',{}).get('effective_radial_clearance_mm'), gap, 'geometry_effective_gap', where)
            _material(audit, dict(case, material=record.get('material')))
            _material(audit, dict(case, material=record.get('material_after_run')))
            audit.check(record.get('preparation',{}).get('budget_active_during_prepare') is False,
                        'preparation_budget_scope', where)
            response = record.get('termination_response') or {}
            audit.check(response.get('command_stream_ended') is True and type(response.get('post_trigger_physics_steps')) is int
                        and response.get('post_trigger_physics_steps') == 0
                        and response.get('hardware_braking_validated') is False, 'termination_response_scope', where)
            metrics = (record.get('reference') or {}).get('metrics') or {}
            eligible = reference_eligible(metrics)
            audit.check((recovery is not None) == eligible, 'recovery_eligibility', where)
            if recovery is None:
                audit.check(not (folder/'recovery.csv').exists() and not (folder/'recovery_contacts.jsonl.gz').exists(),
                            'unexecuted_recovery_artifacts', where)
            audit.check(_same(record.get('outcome'), classify(record)), 'recomputed_classification', where)
            item = dict(condition_id=key[0], policy=key[1], outcome=record.get('outcome'),
                        recovery_present=recovery is not None, first_reference_budget_crossing=None,
                        first_recovery_budget_crossing=None)
            if deep:
                dt = 1/condition['physics_hz']
                for segment, stem in (('reference','insertion'), ('recovery','recovery')):
                    segment_record = record.get(segment)
                    if segment_record is None: continue
                    audit.check(segment_record.get('trajectory') == stem+'.csv' and
                                segment_record.get('contacts') == stem+'_contacts.jsonl.gz', 'segment_paths', where)
                    m = segment_record['metrics']
                    raw = _stream(audit, folder, stem, dt, case,
                                  dict(m, policy=key[1]) if segment == 'recovery' else None,
                                  fixed_speed=segment == 'recovery', protocol=asdict(p))
                    rows = _typed_rows(raw)
                    if not rows: continue
                    e = budget_checks(audit, rows, segment_record.get('evidence'), m, p, dt, segment, where+'/'+segment)
                    item['first_'+segment+'_budget_crossing'] = e['first_budget_crossing']
                    if segment == 'reference':
                        references[key] = rows
                        rebuilt_ref = reference_checks(audit, rows, m, case, p, dt, where)
                        if e['budget_exceeded']:
                            audit.check(recovery is None, 'recovery_after_reference_budget_crossing', where)
                        rebuilt = dict(record, reference=dict(segment_record, metrics=rebuilt_ref))
                    else:
                        rebuilt_rec = recovery_checks(audit, rows, references[key], m, p, dt, where)
                        rebuilt['recovery'] = dict(segment_record, metrics=rebuilt_rec)
                audit.check(_same(record.get('outcome'), classify(rebuilt)), 'raw_recomputed_classification', where)
            observed.append(item)
        except (KeyError, ValueError, TypeError, OSError, ArithmeticError) as error:
            audit.check(False, 'branch_data_exception', where, repr(error))
    not_tested = [a for a in attempts if a.get('status') == 'not_tested']
    for attempt in not_tested:
        key = (attempt.get('condition_id'), 'straight')
        original = records.get(key)
        audit.check(attempt.get('policy') == 'realign' and original is not None and
                    not reference_eligible(original['reference']['metrics']), 'not_tested_prerequisite', attempt.get('condition_id'))
        audit.check(attempt.get('reason') == 'reference_not_eligible_in_straight_branch' and
                    all(attempt.get(k) is None for k in ('folder','run_sha256','outcome','recovery')),
                    'not_tested_has_no_policy_result', attempt.get('condition_id'))
        original_attempt = next((a for a in closed if (a.get('condition_id'),a.get('policy')) == key), {})
        audit.check(attempt.get('source_run') == original_attempt.get('folder'), 'not_tested_source_branch', attempt.get('condition_id'))
    analysis_files = ('simulation/validate_budget_output.py', 'simulation/validate_mechanics_output.py',
                      'research/forge_budget.py', 'research/forge_mechanics_metrics.py',
                      'research/forge_mechanics_plan.py', 'research/forge_mechanics_protocol.py',
                      'research/forge_protocol.py', 'research/phase2_protocol.py',
                      'research/forge_gap_metrics.py', 'research/forge_events.py')
    return dict(schema='Forge-budget-output-validation-v1', source_study=str(directory),
                source_manifest_sha256=hashlib.sha256(manifest_bytes).hexdigest(),
                analysis_sources={relative:hashlib.sha256((ROOT/relative).read_bytes()).hexdigest()
                                  for relative in analysis_files},
                source_status=manifest.get('status'), passed=not audit.errors, deep=deep,
                planned_conditions=len(conditions), planned_policy_branches=len(planned_keys),
                completed_branches_checked=len(closed), not_tested_branches_checked=len(not_tested),
                incomplete_branches_not_opened=len(attempts)-len(closed)-len(not_tested),
                unrecorded_branches=len(planned_keys-set(keys)),
                check_count=audit.checks, error_count=len(audit.errors), errors=audit.errors,
                checked_files=audit.files, checked_streams=audit.streams, actual_branch_results=observed,
                independent_policy_pair_checks=_pairs(conditions, records, references, deep),
                scope='Closed saved evidence only. A successful audit is not physical calibration, a braking guarantee, or proof of paired causal effects. Prefix mismatch never changes a branch observation.')


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
        (destination/'budget_validation.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({key:result[key] for key in ('passed','deep','completed_branches_checked',
                                               'not_tested_branches_checked','check_count','error_count','errors')}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
