"""Read-only parameter evidence from closed straight branches of a budget study.

The output keeps planned comparisons pending until their inputs close. Budget
comparisons inspect only the common observed prefix; a longer branch never
supplies unexecuted samples to the shorter branch. No simulator is imported.
"""
import argparse
import csv
import hashlib
import io
import itertools
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from research.forge_mechanics_protocol import compare_prefixes
from research.forge_protocol import screened


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _read(path, files, expected=None):
    data = path.read_bytes(); digest = _sha(data)
    if expected is not None and digest != expected:
        raise ValueError(f'Closed input SHA mismatch: {path}')
    files[str(path)] = digest
    return data


def _rows(path, files, expected):
    data = _read(path, files, expected)
    if not data.endswith(b'\n'):
        raise ValueError(f'Incomplete closed CSV: {path}')
    result = []
    for raw in csv.DictReader(io.StringIO(data.decode())):
        if None in raw or any(value is None for value in raw.values()):
            raise ValueError(f'Malformed CSV row: {path}')
        row = {}
        for key, value in raw.items():
            if value == '':row[key] = None
            elif value in ('True', 'False'):row[key] = value == 'True'
            else:
                try:number = float(value)
                except ValueError:row[key] = value
                else:
                    if not math.isfinite(number):raise ValueError(f'Nonfinite CSV field: {path}/{key}')
                    row[key] = number
        result.append(row)
    if not result:raise ValueError(f'Empty closed CSV: {path}')
    return result


def _inside(parent, relative):
    path = (parent/relative).resolve()
    if parent not in path.parents:
        raise ValueError(f'Input escapes its source directory: {path}')
    return path


def _load_straight(directory, manifest, files):
    conditions = manifest['case_plan']['conditions']
    known = {c['condition_id']:c for c in conditions}
    if len(known) != len(conditions):raise ValueError('Duplicate planned condition')
    attempts = {}
    for attempt in manifest.get('attempts', []):
        if attempt['condition_id'] not in known:raise ValueError('Unplanned branch')
        key = attempt['condition_id'], attempt['policy']
        if key in attempts:raise ValueError('Duplicate branch invocation')
        attempts[key] = attempt
    entries = {}
    for identity, condition in known.items():
        attempt = attempts.get((identity, 'straight'), {})
        entry = dict(condition=condition, status=attempt.get('status', 'not_started'), record=None,
                     reference_rows=[], recovery_rows=[])
        # In particular, do not open an active process's run.json or CSV.
        if entry['status'] == 'complete':
            folder = _inside(directory, attempt['folder'])
            path = folder/'run.json'
            record = json.loads(_read(path, files, attempt['run_sha256']))
            if (record.get('schema') != 'Forge-budget-branch-v1' or record.get('status') != 'complete'
                    or record.get('condition') != condition or record.get('policy') != 'straight'
                    or record.get('sources') != manifest['sources']):
                raise ValueError(f'Closed branch does not match frozen study: {path}')
            entry['record'] = record
            for segment in ('reference', 'recovery'):
                if record.get(segment):
                    name = record[segment]['trajectory']
                    entry[segment+'_rows'] = _rows(_inside(folder, name), files, record['artifact_sha256'][name])
        entries[identity] = entry
    return entries


def _description(entry):
    condition = entry['condition']; case = condition['case']; record = entry['record'] or {}
    reference = (record.get('reference') or {}).get('metrics') or {}
    recovery = (record.get('recovery') or {}).get('metrics') or {}
    outcome = record.get('outcome') or {}
    return dict(condition_id=condition['condition_id'], status=entry['status'],
                target_depth_mm=case['target_depth_mm'], tilt_amplitude_deg=case['tilt_amplitude_deg'],
                pair_static_friction=case['pair_static_friction'], pair_dynamic_friction=case['pair_dynamic_friction'],
                force_budget_n=condition['force_budget_n'], reference_complete=reference.get('reference_complete'),
                reference_numerically_valid=reference.get('numerically_valid'),
                reference_grasp_retained=reference.get('grasp_retained'),
                reference_within_budget=reference.get('reference_within_budget'),
                reference_force_budget_exceeded=reference.get('force_budget_exceeded'),
                reference_torque_budget_exceeded=reference.get('torque_budget_exceeded'),
                depth_gate_passed=reference.get('depth_gate_passed'), depth_gate_step=reference.get('depth_gate_step'),
                terminal_depth_mm=reference.get('terminal_depth_mm'), terminal_actual_tilt_deg=reference.get('terminal_actual_tilt_deg'),
                reference_peak_wrist_force_n=reference.get('max_wrist_force_n'),
                reference_peak_normal_load_n=reference.get('max_normal_load_n'),
                reference_reason=reference.get('termination_reason', reference.get('reference_termination_reason')),
                recovery_reason=recovery.get('reason'), recovery_censored=recovery.get('recovery_censored'),
                recovery_numerically_valid=recovery.get('numerically_valid'), recovery_grasp_retained=recovery.get('grasp_retained'),
                recovery_force_budget_exceeded=recovery.get('force_budget_exceeded'),
                recovery_torque_budget_exceeded=recovery.get('torque_budget_exceeded'),
                recovery_peak_wrist_force_n=recovery.get('max_wrist_force_n'),
                retreat_peak_wrist_force_n=recovery.get('retreat_peak_wrist_force_n'),
                retreat_peak_wrist_force_mean100ms_n=recovery.get('retreat_peak_wrist_force_mean100ms_n'),
                outcome=outcome.get('outcome'), task_safe=outcome.get('task_safe'), recovery_safe=outcome.get('recovery_safe'))


def _configuration_differences(a, b):
    differences = []
    for key in ('sources', 'geometry', 'material', 'physics_hz', 'seed', 'preparation',
                'recovery_motion', 'budget_definition'):
        if a.get(key) != b.get(key):differences.append(key)
    for key in ('protocol', 'effective_protocol'):
        aa = {k:v for k,v in a[key].items() if k != 'force_budget_n'}
        bb = {k:v for k,v in b[key].items() if k != 'force_budget_n'}
        if aa != bb:differences.append(key+'_excluding_force_budget_n')
    return differences


def _prefix_audit(a, b, aa, bb, extra_commands=()):
    """Use the production physical matcher, plus commands it does not cover."""
    p = SimpleNamespace(**a['record']['effective_protocol'])
    q = SimpleNamespace(**b['record']['effective_protocol'])
    result = dict(observed_prefixes_matched=None, prefix_audit=None, prefix_audit_error=None,
                  configuration_differences=_configuration_differences(a['record'], b['record']),
                  additional_commands_equal=None, first_additional_command_mismatch=None,
                  first_recorded_value_difference=None,
                  compared_prefixes_numerically_valid=None)
    try:
        check = compare_prefixes(aa, bb, p)
        stored_difference = next((dict(index=i, fields={key:dict(a=x.get(key),b=y.get(key))
            for key in sorted(set(x)|set(y)) if key not in ('time_s','physics_time_s') and x.get(key) != y.get(key)})
            for i,(x,y) in enumerate(zip(aa,bb))
            if any(x.get(key) != y.get(key) for key in set(x)|set(y) if key not in ('time_s','physics_time_s'))), None)
        first = next((dict(index=i, field=key, a=x.get(key), b=y.get(key))
                      for i,(x,y) in enumerate(zip(aa,bb)) for key in extra_commands
                      if key not in x or key not in y or x[key] is None or y[key] is None or x[key] != y[key]), None)
        commands_equal = len(aa) == len(bb) and first is None
        valid = all(screened(row, proto) and row['force_norm_n'] <= 500
                    and row['min_separation_mm'] >= -1.
                    for rows,proto in ((aa,p),(bb,q)) for row in rows)
        result.update(prefix_audit=check, additional_commands_equal=commands_equal,
                      first_recorded_value_difference=stored_difference,
                      first_additional_command_mismatch=first,
                      compared_prefixes_numerically_valid=valid,
                      observed_prefixes_matched=bool(check['replay_prefix_matched'] and commands_equal))
    except (ValueError, KeyError, TypeError, ZeroDivisionError) as error:
        result['prefix_audit_error'] = f'{type(error).__name__}: {error}'
    result['matching_evidence_eligible'] = bool(result['observed_prefixes_matched']
        and result['compared_prefixes_numerically_valid'] and not result['configuration_differences'])
    return result


def _crossing_at_shorter_end(a, b, segment):
    aa, bb = a[segment+'_rows'], b[segment+'_rows']
    result = dict(shorter_condition_id=None, shorter_stopped_at_first_budget_crossing=None,
                  crossing_counterpart=None)
    if len(aa) == len(bb) or not aa or not bb:return result
    short, long = (a,b) if len(aa) < len(bb) else (b,a)
    rows = short[segment+'_rows']; other = long[segment+'_rows']
    protocol = short['record']['effective_protocol']
    index = next((i for i,row in enumerate(rows) if row['wrist_force_n'] > protocol['force_budget_n']
                  or row['wrist_torque_nm'] > protocol['torque_budget_nm']), None)
    metrics = short['record'][segment]['metrics']
    reason = metrics.get('reason', metrics.get('termination_reason', metrics.get('reference_termination_reason')))
    result.update(shorter_condition_id=short['condition']['condition_id'],
                  shorter_stopped_at_first_budget_crossing=index == len(rows)-1 and reason == 'operational_budget_exceeded')
    if index is not None:
        keys = ('time_s', 'recovery_time_s', 'phase', 'wrist_force_n', 'wrist_torque_nm',
                'depth_mm', 'command_depth_mm', 'command_withdrawal_mm', 'command_withdrawal_speed_mm_s')
        result['crossing_counterpart'] = dict(index=index,
            shorter={k:rows[index].get(k) for k in keys},
            longer={k:other[index].get(k) for k in keys},
            wrist_force_equal=rows[index]['wrist_force_n'] == other[index]['wrist_force_n'],
            wrist_torque_equal=rows[index]['wrist_torque_nm'] == other[index]['wrist_torque_nm'],
            longer_condition_id=long['condition']['condition_id'],
            interpretation='Observed counterpart only. No later sample is assigned to the shorter branch.')
    return result


def compare_entries(a, b, kind):
    item = dict(comparison_id=f'{kind}__{a["condition"]["condition_id"]}__{b["condition"]["condition_id"]}',
                kind=kind, policy='straight', a=_description(a), b=_description(b),
                status='pending_or_incomplete', observed_prefixes_matched=None,
                scope=None, samples_a=None, samples_b=None, compared_samples_a=None, compared_samples_b=None,
                complete_recovery_equality_claimed=False, unobserved_future_imputed=False)
    if not a['record'] or not b['record']:return item
    if kind in ('depth_description', 'friction_description'):
        item.update(status='descriptive_only', scope='actual terminal measurements and outcomes; no strict initial-state or causal equality claim')
        return item
    segment = 'recovery' if kind == 'budget_recovery_prefix' else 'reference'
    aa, bb = a[segment+'_rows'], b[segment+'_rows']
    item.update(samples_a=len(aa), samples_b=len(bb))
    if kind == 'angle_depth_gate_prefix':
        item['scope'] = 'each complete observed pre-tilt prefix through its own depth-gate step'
        gates = [entry['record']['reference']['metrics'].get('depth_gate_step') for entry in (a,b)]
        if any(entry['record']['reference']['metrics'].get('depth_gate_passed') is not True for entry in (a,b)):
            item['status'] = 'depth_gate_not_reached'; return item
        if any(type(gate) not in (int,float) or not math.isfinite(gate) or gate < 0 or gate != int(gate) for gate in gates):
            item['status'] = 'depth_gate_unavailable'; return item
        if any(not any(r.get('reference_step') == gate for r in rows) for rows,gate in zip((aa,bb),gates)):
            item['status'] = 'depth_gate_unavailable'; return item
        aa, bb = ([r for r in rows if r['reference_step'] <= gate] for rows,gate in zip((aa,bb),gates))
    else:
        item['scope'] = 'common observed prefix only; longer unexecuted future is not evidence for the shorter branch'
        if segment == 'recovery':
            if not all(entry['record']['reference']['metrics'].get('reference_complete') is True for entry in (a,b)):
                item['status'] = 'reference_incomplete_recovery_not_compared'; return item
            if not aa or not bb:
                item['status'] = 'recovery_unobserved'; return item
            ref_check = _prefix_audit(a, b, a['reference_rows'], b['reference_rows'])
            item['full_reference_prefix_audit'] = ref_check
        item.update(_crossing_at_shorter_end(a,b,segment))
        count = min(len(aa),len(bb)); aa,bb = aa[:count],bb[:count]
    item.update(compared_samples_a=len(aa), compared_samples_b=len(bb))
    if not aa or not bb:
        item['status'] = 'prefix_unobserved'; return item
    commands = ('command_withdrawal_mm', 'command_withdrawal_speed_mm_s', 'recovery_time_s') if segment == 'recovery' else ()
    item.update(_prefix_audit(a,b,aa,bb,commands))
    if segment == 'recovery':
        item['matching_evidence_eligible'] &= item['full_reference_prefix_audit']['matching_evidence_eligible']
    item['status'] = ('audit_unknown' if item['prefix_audit_error'] else 'observed_prefix_matched'
                      if item['observed_prefixes_matched'] else 'observed_prefix_mismatch')
    return item


def analyze_study(study_dir):
    directory = Path(study_dir).resolve(); files = {}
    data = _read(directory/'study.json', files)
    manifest = json.loads(data)
    if manifest.get('schema') != 'Forge-budget-v1':raise ValueError('Expected Forge-budget-v1 study')
    entries = _load_straight(directory, manifest, files)

    def choose(depth=18., friction=(1.,1.), angle=1.5, budget=4.):
        found = [e for e in entries.values() if e['condition']['force_budget_n'] == budget
                 and e['condition']['case']['target_depth_mm'] == depth
                 and e['condition']['case']['tilt_amplitude_deg'] == angle
                 and (e['condition']['case']['pair_static_friction'],e['condition']['case']['pair_dynamic_friction']) == friction]
        if len(found) != 1:raise ValueError('Required bounded comparison condition is missing or duplicated')
        return found[0]

    if manifest['case_plan'].get('study_id') == 'Forge-budget-angle6-v1':
        comparisons = [compare_entries(choose(angle=1.5), choose(angle=angle),
                                       'angle_depth_gate_prefix') for angle in (2.,3.,4.,5.,6.)]
    else:
        comparisons = [compare_entries(choose(angle=0.),choose(angle=angle),'angle_depth_gate_prefix') for angle in (1.,1.5,2.)]
        comparisons += [compare_entries(choose(budget=a),choose(budget=b),'budget_reference_prefix')
                        for a,b in itertools.combinations((3.,4.,5.),2)]
        comparisons += [compare_entries(choose(budget=4.),choose(budget=5.),'budget_recovery_prefix'),
                        compare_entries(choose(depth=12.),choose(depth=18.),'depth_description'),
                        compare_entries(choose(friction=(.5,.5)),choose(friction=(1.,1.)),'friction_description')]
    # Closed inputs must remain unchanged even when the manifest advances.
    for name,digest in files.items():
        if name != str(directory/'study.json') and _sha(Path(name).read_bytes()) != digest:
            raise ValueError(f'Closed input changed during analysis: {name}')
    return dict(schema='Forge-budget-parameter-comparisons-v1', source_study=str(directory),
                study_id=manifest['case_plan'].get('study_id', 'Forge-budget-original-v1'),
                source_status=manifest.get('status'), source_study_sha256=_sha(data),
                planned_straight_conditions=len(entries), closed_straight_conditions=sum(e['record'] is not None for e in entries.values()),
                branch_statuses=[dict(condition_id=k,status=e['status']) for k,e in entries.items()],
                comparisons=comparisons, input_files=[dict(path=k,sha256=v) for k,v in files.items()],
                analyzer_sha256=_sha(Path(__file__).read_bytes()),
                matching_helper_sha256=_sha((ROOT/'research/forge_mechanics_protocol.py').read_bytes()),
                interpretation=[
                    'Budget and grasp terminations remain observed outcomes, not discarded conditions.',
                    'Budget comparisons use only the common observed prefix; matching it never claims complete recovery equality.',
                    'Counterpart values come from their own budget branch; no unexecuted future is filled into a stopped branch.',
                    'Depth and friction contrasts are descriptive actual-state comparisons, not strict initial-state causal pairs.',
                    'Physical tolerance matching does not certify unobserved solver state; validity and configuration flags remain separate.',
                    'Exact stored-row equality also includes phase-state metadata; first_recorded_value_difference is diagnostic, not an extra physical-match criterion.',
                    'Only closed straight branches at the captured manifest snapshot are opened. Pending comparisons remain explicit.'])


def write_review(result, output_dir):
    directory = Path(output_dir).resolve(); source = Path(result['source_study'])
    if directory == source or source in directory.parents:
        raise ValueError('Review directory must be outside the source study')
    directory.mkdir(parents=True,exist_ok=True)
    (directory/'parameter_comparisons.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    flat = []
    for item in result['comparisons']:
        row = {k:v for k,v in item.items() if k not in ('a','b')}
        for side in ('a','b'):row.update({side+'_'+k:v for k,v in item[side].items()})
        flat.append(row)
    with (directory/'parameter_comparisons.csv').open('w',newline='') as stream:
        writer = csv.DictWriter(stream,fieldnames=list(dict.fromkeys(k for row in flat for k in row)))
        writer.writeheader()
        writer.writerows({k:json.dumps(v,allow_nan=False) if isinstance(v,(dict,list)) else v for k,v in row.items()} for row in flat)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path)
    args = parser.parse_args()
    result = analyze_study(args.study_dir)
    destination = args.output_dir or args.study_dir.with_name(args.study_dir.name+'-review')
    write_review(result,destination)
    print(json.dumps(dict(closed_straight_conditions=result['closed_straight_conditions'],
                         comparisons=len(result['comparisons']),output_dir=str(destination.resolve())),indent=2))


if __name__ == '__main__':main()
