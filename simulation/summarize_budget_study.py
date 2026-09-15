"""Read-only summaries of prospective, independently executed budget branches.

No simulator modules are imported. Pending branches retain explicit missing
values. Observed branch outcomes are distinct from full-prefix matched policy
comparisons; truncated force peaks cannot serve as completed motion costs.
"""
import argparse
from collections import Counter
import csv
import hashlib
import io
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from research.forge_mechanics_protocol import compare_prefixes


POLICIES = ('straight', 'realign')
PAIRED_COST_FIELDS = ('recovery_observed_peak_wrist_force_n', 'retreat_observed_peak_wrist_force_n',
                      'retreat_peak_wrist_force_mean100ms_n')


def _sha(data):
    return hashlib.sha256(data).hexdigest()


def _parse(value):
    if value in ('', None):
        return None
    if value in ('True', 'False'):
        return value == 'True'
    try:
        number = float(value)
        return number if math.isfinite(number) else None
    except (ValueError, TypeError):
        return value


def read_rows(path, expected_sha=None):
    data = path.read_bytes()
    digest = _sha(data)
    if expected_sha is not None and digest != expected_sha:
        raise ValueError(f'Artifact SHA mismatch: {path}')
    rows = [{k:_parse(v) for k, v in row.items()} for row in csv.DictReader(io.StringIO(data.decode()))]
    return rows, dict(path=str(path), sha256=digest)


def _closed_cost(record):
    recovery = (record.get('recovery') or {}).get('metrics') or {}
    return (recovery.get('numerically_valid') is True and recovery.get('cleared') is True
            and recovery.get('recovery_censored') is False)


def branch_row(condition, policy, attempt=None, record=None):
    case = condition['case']
    row = dict(condition_id=condition['condition_id'], case_id=case['case_id'], role=condition['role'],
               split_group_id=case['split_group_id'], path_group_id=case['path_group_id'],
               target_depth_mm=case['target_depth_mm'], pair_static_friction=case['pair_static_friction'],
               pair_dynamic_friction=case['pair_dynamic_friction'], tilt_amplitude_deg=case['tilt_amplitude_deg'],
               tilt_ramp_duration_s=case['tilt_ramp_duration_s'],
               planned_peak_command_tilt_rate_deg_s=1.5*case['tilt_amplitude_deg']/case['tilt_ramp_duration_s'],
               force_budget_n=condition['force_budget_n'], torque_budget_nm=condition['torque_budget_nm'],
               physics_hz=condition['physics_hz'], policy=policy,
               status=(attempt or {}).get('status', 'not_started'), folder=(attempt or {}).get('folder'),
               reason=(attempt or {}).get('reason'), outcome=None, reference_eligible=None,
               recovery_safe=None, task_safe=None, recovery_observed=None,
               reference_numerically_valid=None, reference_grasp_retained=None,
               reference_complete=None, reference_observed_peak_wrist_force_n=None,
               reference_peak_is_lower_bound=None,
               terminal_depth_mm=None, terminal_actual_tilt_deg=None, terminal_command_tilt_deg=None,
               max_actual_tilt_deg=None, tilt_ramp_completed=None, reference_max_normal_load_n=None,
               reference_max_grasp_slip_mm=None, reference_max_penetration_mm=None,
               recovery_numerically_valid=None, recovery_grasp_retained=None, recovery_cleared=None, recovery_censored=None,
               recovery_peak_is_lower_bound=None, recovery_observed_peak_wrist_force_n=None,
               observed_task_peak_wrist_force_n=None, reference_reason=None, recovery_reason=None,
               first_budget_crossing_segment=None, first_budget_crossing_phase=None,
               first_budget_crossing_time_s=None, first_budget_crossing_recovery_time_s=None,
               first_budget_crossing_wrist_force_n=None, first_budget_crossing_wrist_torque_nm=None,
               force_overshoot_n=None, torque_overshoot_nm=None)
    for segment in ('reference', 'recovery'):
        for flag in ('force_budget_exceeded', 'torque_budget_exceeded', 'budget_signals_finite'):
            row[segment+'_'+flag] = None
    for phase in ('stop', 'realign', 'retreat', 'clear_hold'):
        row.update({phase+'_executed':None, phase+'_observed_peak_wrist_force_n':None,
                    phase+'_peak_wrist_force_mean100ms_n':None})
    if record is None:
        return row
    reference = record.get('reference') or {}
    recovery = record.get('recovery') or {}
    rm, re = reference.get('metrics') or {}, reference.get('evidence') or {}
    cm, ce = recovery.get('metrics') or {}, recovery.get('evidence') or {}
    outcome = record['outcome']
    row.update({k:outcome.get(k) for k in ('outcome', 'reference_eligible', 'recovery_safe', 'task_safe')})
    row.update(status='complete', reason=outcome.get('outcome'), recovery_observed=bool(recovery),
               reference_numerically_valid=rm.get('numerically_valid'), reference_grasp_retained=rm.get('grasp_retained'),
               reference_complete=rm.get('reference_complete'), reference_observed_peak_wrist_force_n=re.get('max_wrist_force_n'),
               reference_peak_is_lower_bound=(rm.get('reference_complete') is not True)
                   if rm.get('numerically_valid') is True else None,
               terminal_depth_mm=rm.get('terminal_depth_mm'), terminal_actual_tilt_deg=rm.get('terminal_actual_tilt_deg'),
               terminal_command_tilt_deg=rm.get('terminal_command_tilt_deg'),
               max_actual_tilt_deg=rm.get('max_actual_tilt_deg'), tilt_ramp_completed=rm.get('tilt_ramp_completed'),
               reference_max_normal_load_n=rm.get('max_normal_load_n'), reference_max_grasp_slip_mm=rm.get('max_grasp_slip_mm'),
               reference_max_penetration_mm=rm.get('max_penetration_mm'),
               recovery_numerically_valid=cm.get('numerically_valid'), recovery_cleared=cm.get('cleared'),
               recovery_grasp_retained=cm.get('grasp_retained'),
               recovery_censored=cm.get('recovery_censored'),
               recovery_peak_is_lower_bound=not _closed_cost(record)
                   if recovery and cm.get('numerically_valid') is True else None,
               recovery_observed_peak_wrist_force_n=ce.get('max_wrist_force_n'),
               reference_reason=rm.get('termination_reason', rm.get('reference_termination_reason')),
               recovery_reason=cm.get('reason', cm.get('termination_reason')))
    peaks = [v for v in (re.get('max_wrist_force_n'), ce.get('max_wrist_force_n')) if type(v) in (float, int)]
    row['observed_task_peak_wrist_force_n'] = max(peaks) if peaks else None
    for segment, evidence in (('reference', re), ('recovery', ce)):
        for flag in ('force_budget_exceeded', 'torque_budget_exceeded', 'budget_signals_finite'):
            row[segment+'_'+flag] = evidence.get(flag)
    for segment, evidence in (('reference', re), ('recovery', ce)):
        crossing = evidence.get('first_budget_crossing')
        if crossing:
            row.update(first_budget_crossing_segment=segment, first_budget_crossing_phase=crossing.get('phase'),
                       first_budget_crossing_time_s=crossing.get('time_s'),
                       first_budget_crossing_recovery_time_s=crossing.get('recovery_time_s'),
                       first_budget_crossing_wrist_force_n=crossing.get('wrist_force_n'),
                       first_budget_crossing_wrist_torque_nm=crossing.get('wrist_torque_nm'),
                       force_overshoot_n=crossing.get('force_overshoot_n'),
                       torque_overshoot_nm=crossing.get('torque_overshoot_nm'))
            break
    for phase in ('stop', 'realign', 'retreat', 'clear_hold'):
        metrics = (ce.get('phase_metrics') or {}).get(phase) or {}
        row[phase+'_executed'] = metrics.get('phase_executed', False) if recovery else None
        row[phase+'_observed_peak_wrist_force_n'] = metrics.get('max_wrist_force_n')
        row[phase+'_peak_wrist_force_mean100ms_n'] = metrics.get('peak_wrist_force_mean100ms_n')
    return row


def pair_rows(condition, branches, records, reference_rows):
    straight, realign = branches
    audit = None; config_differences = []; error = None
    both_complete = all(r['status'] == 'complete' for r in branches)
    if both_complete:
        a, b = records
        for key in ('condition', 'protocol', 'effective_protocol', 'physics_hz', 'seed', 'recovery_motion',
                    'sources', 'geometry', 'material', 'budget_definition', 'preparation'):
            if a.get(key) != b.get(key):
                config_differences.append(key)
        try:
            audit = compare_prefixes(reference_rows[0], reference_rows[1],
                                     SimpleNamespace(**a['effective_protocol']))
        except (ValueError, KeyError, TypeError, ZeroDivisionError) as exc:
            error = f'{type(exc).__name__}: {exc}'
    reference_eligible = both_complete and all(r['reference_eligible'] is True for r in branches)
    matched = bool(reference_eligible and not config_differences and audit and
                   audit.get('replay_matched') and audit.get('replay_prefix_matched'))
    recovery_valid = both_complete and all(r['recovery_numerically_valid'] is True for r in branches)
    grasp_retained = both_complete and all(r['recovery_grasp_retained'] is True for r in branches)
    mixed_constraints = any(r['outcome'] == 'recovery_budget_exceeded' and
                            r['recovery_grasp_retained'] is False for r in branches)
    if any(r['status'] == 'not_tested' for r in branches):
        state = 'alternative_not_tested_reference_ineligible'
    elif not both_complete:
        state = 'pending_or_incomplete'
    elif not reference_eligible:
        state = 'reference_ineligible'
    elif config_differences:
        state = 'configuration_mismatch_unknown'
    elif error or audit is None:
        state = 'prefix_audit_unknown'
    elif not audit['replay_matched']:
        state = 'endpoint_mismatch_unknown'
    elif not audit['replay_prefix_matched']:
        state = 'prefix_mismatch_unknown'
    elif not recovery_valid:
        state = 'paired_recovery_numerical_unknown'
    elif mixed_constraints:
        state = 'paired_mixed_constraints'
    elif not grasp_retained:
        state = 'paired_grasp_constraint_or_unknown'
    elif straight['outcome'] == 'recovery_budget_exceeded' and realign['outcome'] == 'clear_safe':
        state = 'paired_straight_overbudget_realign_safe'
    elif straight['outcome'] == 'clear_safe' and realign['outcome'] == 'recovery_budget_exceeded':
        state = 'paired_straight_safe_realign_overbudget'
    elif all(r['outcome'] == 'clear_safe' for r in branches):
        state = 'paired_both_clear_safe'
    elif all(r['outcome'] == 'recovery_budget_exceeded' for r in branches):
        state = 'paired_both_overbudget'
    else:
        state = 'paired_other_or_unknown_outcome'
    complete_costs = bool(matched and recovery_valid and grasp_retained and all(_closed_cost(r) for r in records) and
                          all(r['outcome'] == 'clear_safe' for r in branches))
    row = dict(condition_id=condition['condition_id'], pair_state=state,
               both_branches_complete=both_complete, both_references_eligible=reference_eligible,
               observed_prefixes_matched=matched, paired_costs_complete=complete_costs,
               paired_recovery_evidence_eligible=bool(matched and recovery_valid and grasp_retained),
               configuration_differences=config_differences, prefix_audit_error=error,
               replay_matched=(audit or {}).get('replay_matched'),
               replay_prefix_matched=(audit or {}).get('replay_prefix_matched'),
               replay_prefix_equal=(audit or {}).get('replay_prefix_equal'),
               replay_prefix_first_mismatch_index=(audit or {}).get('replay_prefix_first_mismatch_index'),
               straight_outcome=straight['outcome'], realign_outcome=realign['outcome'])
    for field in PAIRED_COST_FIELDS:
        a, b = straight[field], realign[field]
        row['straight_minus_realign_'+field] = a-b if complete_costs and a is not None and b is not None else None
    return row, audit


def summarize_study(study_dir):
    directory = Path(study_dir).resolve()
    data = (directory/'study.json').read_bytes()
    manifest = json.loads(data)
    if manifest.get('schema') != 'Forge-budget-v1':
        raise ValueError('Expected Forge-budget-v1 root study')
    plan = manifest['case_plan']; conditions = plan['conditions']; policies = plan['policies']
    if policies != list(POLICIES):
        raise ValueError('Expected straight and realign policies in canonical order')
    if len({c['condition_id'] for c in conditions}) != len(conditions):
        raise ValueError('Duplicate planned condition')
    attempts = {}
    for attempt in manifest.get('attempts', []):
        key = (attempt['condition_id'], attempt['policy'])
        if key in attempts:
            raise ValueError(f'Duplicate branch invocation: {key}')
        if key[0] not in {c['condition_id'] for c in conditions} or key[1] not in policies:
            raise ValueError(f'Unplanned branch: {key}')
        attempts[key] = attempt
    branches = []; cases = []; pairs = []; audits = []; files = []; plot_records = {}
    for condition in conditions:
        condition_branches = []; condition_records = []; prefixes = []
        for policy in policies:
            key = (condition['condition_id'], policy)
            attempt = attempts.get(key); record = None; rows = []; recovery_rows = []
            if attempt and attempt['status'] == 'complete':
                folder = (directory/attempt['folder']).resolve()
                if directory not in folder.parents:
                    raise ValueError(f'Branch outside study: {folder}')
                run_path = folder/'run.json'; run_data = run_path.read_bytes()
                if _sha(run_data) != attempt['run_sha256']:
                    raise ValueError(f'Closed branch SHA mismatch: {run_path}')
                record = json.loads(run_data)
                if record.get('schema') != 'Forge-budget-branch-v1' or record.get('status') != 'complete':
                    raise ValueError(f'Expected closed budget branch: {run_path}')
                if record['condition'] != condition or record['policy'] != policy or record['sources'] != manifest['sources']:
                    raise ValueError(f'Closed branch differs from frozen plan/source: {run_path}')
                files.append(dict(path=str(run_path), sha256=_sha(run_data)))
                name = record['reference']['trajectory']
                rows, file = read_rows(folder/name, record['artifact_sha256'][name]); files.append(file)
                if record.get('recovery'):
                    name = record['recovery']['trajectory']
                    recovery_rows, file = read_rows(folder/name, record['artifact_sha256'][name]); files.append(file)
                if condition.get('role') == 'candidate':
                    plot_records[policy] = dict(reference=rows, recovery=recovery_rows, record=record)
            row = branch_row(condition, policy, attempt, record)
            condition_branches.append(row); condition_records.append(record); prefixes.append(rows); branches.append(row)
        pair, audit = pair_rows(condition, condition_branches, condition_records, prefixes)
        pairs.append(pair); audits.append(dict(condition_id=condition['condition_id'], audit=audit))
        case_row = {key:condition_branches[0][key] for key in
                    ('condition_id', 'case_id', 'role', 'split_group_id', 'path_group_id', 'target_depth_mm', 'pair_static_friction',
                     'pair_dynamic_friction', 'tilt_amplitude_deg', 'tilt_ramp_duration_s',
                     'planned_peak_command_tilt_rate_deg_s', 'force_budget_n', 'torque_budget_nm', 'physics_hz')}
        case_row.update({key:value for key, value in pair.items() if key != 'condition_id'})
        for policy, row in zip(policies, condition_branches):
            case_row.update({policy+'_'+key:row[key] for key in
                ('status', 'outcome', 'reference_eligible', 'recovery_safe', 'task_safe',
                 'terminal_depth_mm', 'terminal_actual_tilt_deg', 'terminal_command_tilt_deg',
                 'max_actual_tilt_deg', 'tilt_ramp_completed',
                 'reference_observed_peak_wrist_force_n', 'recovery_observed_peak_wrist_force_n',
                 'recovery_peak_is_lower_bound', 'retreat_observed_peak_wrist_force_n',
                 'retreat_peak_wrist_force_mean100ms_n', 'stop_observed_peak_wrist_force_n',
                 'realign_observed_peak_wrist_force_n', 'first_budget_crossing_segment', 'first_budget_crossing_phase')})
        cases.append(case_row)
    summary = dict(schema='Forge-budget-review-v1', source_study=str(directory), source_status=manifest.get('status'),
                   study_id=plan.get('study_id', 'Forge-budget-original-v1'),
                   selection_provenance=manifest.get('selection_provenance'),
                   source_study_sha256=_sha(data), elapsed_wall_seconds=manifest.get('elapsed_wall_seconds'),
                   counts=dict(planned_conditions=len(conditions), planned_branches=len(conditions)*len(policies),
                       closed_branches=sum(r['status']=='complete' for r in branches),
                       branch_status_counts=dict(Counter(r['status'] for r in branches)),
                       observed_branch_outcome_counts=dict(Counter(r['outcome'] for r in branches if r['status']=='complete')),
                       pair_state_counts=dict(Counter(r['pair_state'] for r in pairs)),
                       paired_complete_costs=sum(r['paired_costs_complete'] for r in pairs)),
                   budget_definition=plan['budget_definition'],
                   interpretation={
                       'branch':'Each policy is an original continuous reference + recovery in its own process. Its source outcome remains visible even when the two policy references do not match.',
                       'pair':'Full sampled reference prefixes and endpoint physical tolerances must match; equal seed alone is insufficient. Both recovery records must also be numerically valid with grasp retained for clean budget-policy labels. Concurrent budget/grasp events are mixed_constraints. Bitwise prefix equality is diagnostic. Matching observed fields does not certify hidden solver-state identity.',
                       'censor':'Observed recovery peaks include the initial snapshot and every executed phase. After budget-triggered termination, they are lower bounds on the unobserved complete-motion peak. Cost differences are supplied only for two matched, uncensored safe clearances.',
                       'not_tested':'The canonical driver skips the alternative policy if the straight branch reference is ineligible. The planned branch remains in the denominator with null recovery labels.',
                       'budget':'Raw sampled wrist norm, strict > threshold, including complete reference and recovery. A threshold-crossing sample may overshoot; shutdown after detection is not validated hardware braking.',
                       'realign':'XY recentering plus orientation alignment; it is not an angle-only intervention.',
                       'unknown':'Numerical rejection, mismatch, missing and incomplete branches are explicit unknown/unpaired states, not recovery failures.',
                       'selection':'Conditions and budgets were selected from prior historical trajectories. These are exploratory online tests, not a blinded holdout or independent stochastic replications.'},
                   prefix_audits=audits, input_files=files, exporter_sha256=_sha(Path(__file__).read_bytes()),
                   matching_helper_sha256=_sha((ROOT/'research/forge_mechanics_protocol.py').read_bytes()))
    summary['interpretation'].update(
        dataset_split='Keep shared physical paths/references together across train/validation/test. B=3/4/5 N variants and both policies reuse the same planned path (path_group_id); split_group_id also groups the related depth/friction family. Do not randomly split individual rows or time samples across these groups.',
        budget_as_input='force_budget_n and torque_budget_nm are condition inputs to safety/recoverability. The same state/path at different budgets can legitimately have different labels; omitting the budget would create apparent contradictions. Matching and outcome eligibility remain separate from the planned group IDs.')
    return dict(summary=summary, cases=cases, branches=branches, paired=pairs,
                plot_records=plot_records, source_manifest=manifest)


def write_summary(result, output_dir):
    directory = Path(output_dir).resolve(); source = Path(result['summary']['source_study'])
    if source == directory or source in directory.parents:
        raise ValueError('Output directory must be outside the source study')
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'summary.json').write_text(json.dumps(result['summary'], indent=2, allow_nan=False)+'\n')
    for name in ('cases', 'branches', 'paired'):
        rows = result[name]
        with (directory/f'{name}.csv').open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['condition_id'])
            writer.writeheader()
            writer.writerows({k:json.dumps(v) if isinstance(v, (dict, list)) else v for k, v in row.items()} for row in rows)
    (directory/'source_study_snapshot.json').write_text(json.dumps(result['source_manifest'], indent=2, allow_nan=False)+'\n')
    schema = dict(schema='Forge-budget-review-columns-v1',
        tables={name:dict(row_unit=unit, columns=list(result[name][0]) if result[name] else [])
                for name, unit in [('cases','One row per planned condition, combining two policy branches.'),
                                   ('branches','One row per planned condition and policy, including not-tested and pending branches.'),
                                   ('paired','One row per planned condition; observed-prefix matching is separate from branch outcomes.')]},
        core_columns={
            'condition_id':'Includes the force budget; identifies a planned condition, not an independent physical path.',
            'case_id':'Physical case ID from the canonical case plan; shared by budget variants.',
            'split_group_id':'Original case.split_group_id. Related depth/friction cases share this split group; keep the group together when partitioning a learning dataset.',
            'path_group_id':'Original case.path_group_id. B3/B4/B5 and the two policies can share the same planned physical/reference path; never randomly split those rows across train/test. This metadata alone does not certify observed-prefix matching.',
            'force_budget_n':'Raw sampled wrist-force norm threshold in N; a model condition input. Strict > triggers termination; equality is allowed.',
            'torque_budget_nm':'Raw sampled wrist-torque norm threshold in N m; a model condition input.',
            'target_depth_mm':'Commanded depth target in mm.',
            'tilt_amplitude_deg':'Commanded tilt amplitude in degrees.',
            'tilt_ramp_duration_s':'Planned smoothstep command-ramp duration in seconds; not the duration actually completed before termination.',
            'planned_peak_command_tilt_rate_deg_s':'Theoretical peak command angle rate in degrees/s: 1.5 * tilt_amplitude_deg / tilt_ramp_duration_s for the existing cubic smoothstep. It is not measured peg angular velocity and may never be reached before termination. With a fixed 1 s ramp, increasing amplitude also increases commanded angular speed.',
            'terminal_command_tilt_deg':'Last actually issued command amplitude in degrees; it can be below the planned tilt_amplitude_deg if the reference terminates early.',
            'terminal_actual_tilt_deg':'Measured terminal peg tilt in degrees, independently of command completion.',
            'max_actual_tilt_deg':'Maximum measured tilt during the source tilt/hold window; null if that window was never observed.',
            'tilt_ramp_completed':'Source command schedule reached the end of its ramp. This does not mean the physical peg reached the commanded target angle.',
            'pair_static_friction / pair_dynamic_friction':'Dimensionless effective pair coefficients requested by the canonical condition.',
            'reference_eligible':'Source branch eligibility of the full reference for starting recovery; null when unobserved.',
            'recovery_safe / task_safe':'Original branch labels. Null means unobserved or unknown, never false by default.',
            '*_observed_peak_wrist_force_n':'Observed raw wrist-force maximum in N over the named segment/phase. Whole recovery includes the initial snapshot, STOP, REALIGN and RETREAT; a phase can contain only a copied initial observation, so also inspect *_executed.',
            '*_peak_wrist_force_mean100ms_n':'Maximum complete contiguous same-phase 100 ms window mean in N; null if a full window is unavailable. Not the budget decision signal.',
            '*_peak_is_lower_bound':'True for a numerically valid incomplete/censored segment: the observed peak is a lower bound on an unobserved completed-motion peak. False for an observed complete segment; null when unobserved or numerically invalid. Do not interpret it as an estimate of the eventual peak.',
            'first_budget_crossing_*':'First recorded crossing, preserving segment, phase and actual force/torque; time_s follows the saved task clock and recovery_time_s starts at the recovery initial snapshot. Overshoot is retained, not clipped.',
            'observed_prefixes_matched':'Both full references are eligible and pass observed-state endpoint/prefix tolerances under matching configuration. This is not inferred from seed or grouping IDs.',
            'paired_recovery_evidence_eligible':'In addition to matched references, both recoveries are numerically valid and retain grasp. A concurrent budget/grasp event is mixed_constraints.',
            'paired_costs_complete':'Both matched branches completed uncensored safe clearance; only then are straight_minus_realign cost differences populated.',
            'straight_minus_realign_*':'N for force costs; null for unmatched, missing, invalid or censored comparisons.'},
        null_rule='JSON null and empty CSV cells mean unobserved, unknown or ineligible comparison; never substitute zero or a failure label.',
        unit_suffixes={'_n':'N','_nm':'N m','_mm':'mm','_deg':'degrees','_s':'seconds','_hz':'Hz'},
        dataset_split=result['summary']['interpretation']['dataset_split'],
        budget_as_input=result['summary']['interpretation']['budget_as_input'],
        source_study_sha256=result['summary']['source_study_sha256'])
    (directory/'schema.json').write_text(json.dumps(schema, indent=2, allow_nan=False)+'\n')


def _mean100(rows, hz):
    count = max(1, math.ceil(.1*hz-1e-9)); window = []; values = []; previous = None
    for index, row in enumerate(rows):
        time = row.get('recovery_time_s', row['time_s'])
        contiguous = (previous is not None and row['phase'] == previous['phase'] and
                       math.isclose(time-previous.get('recovery_time_s', previous['time_s']), 1/hz, abs_tol=1e-7))
        if not contiguous or index == 0:
            window = []
        value = row.get('wrist_force_n')
        if type(value) not in (int, float) or not math.isfinite(value):
            window = []
        elif index > 0:
            window.append(value)
        if len(window) > count:
            window.pop(0)
        values.append(sum(window)/count if len(window)==count else math.nan)
        previous = row
    return values


def _shade(ax, rows, times):
    colors = dict(stop='#dedede', realign='#e0d4ef', retreat='#d6e8f5', insert='#e6f0e0',
                  approach='#f1f1f1', settle='#f6ecd6', tilt='#e0d4ef', hold='#d6e8f5')
    start = 0
    for i in range(1, len(rows)+1):
        if i == len(rows) or rows[i]['phase'] != rows[start]['phase']:
            phase = rows[start]['phase']
            ax.axvspan(times[start], times[i-1], color=colors.get(phase, '#ffffff'), alpha=.55, zorder=0)
            if phase in ('stop', 'realign', 'retreat'):
                ax.text((times[start]+times[i-1])/2, .98, phase.upper(), transform=ax.get_xaxis_transform(),
                        ha='center', va='top', fontsize=7)
            start = i


def withdrawal_plot_data(reference_endpoint_depth_mm, recovery_rows):
    """Use recorded withdrawal commands; t=0 command_depth may be a copied target.

    Missing command_withdrawal_mm stays missing rather than being reconstructed
    from the copied reference command depth. Actual withdrawal is measured from
    the reference endpoint, so it also starts at zero in a continuous branch.
    """
    return dict(time_s=[row.get('recovery_time_s') for row in recovery_rows],
                commanded_mm=[row.get('command_withdrawal_mm') for row in recovery_rows],
                actual_mm=[reference_endpoint_depth_mm-row['depth_mm']
                           if type(row.get('depth_mm')) in (int, float) else None for row in recovery_rows])


def write_plots(result, output_dir):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    directory = Path(output_dir)
    conditions = result['cases']; branches = {(r['condition_id'], r['policy']):r for r in result['branches']}
    style = {'clear_safe':('#bee1c4', 'Clear / safe'),
             'recovery_budget_exceeded':('#f1b5af', 'Recovery budget exceeded'),
             'reference_budget_exceeded':('#f2d2a1', 'Reference budget exceeded'),
             'reference_grasp_limit':('#dcc4e6', 'Reference grasp gate'),
             'recovery_grasp_limit':('#d3b1d6', 'Recovery grasp gate'),
             'numerical_unknown':('#c6d6eb', 'Numerical unknown'),
             'timeout_unknown':('#c6d6eb', 'Incomplete / unknown')}
    fig, ax = plt.subplots(figsize=(12.5, max(5.5, .62*len(conditions)+1.5)))
    for i, case in enumerate(conditions):
        for j, policy in enumerate(POLICIES):
            row = branches[(case['condition_id'], policy)]
            color, label = style.get(row['outcome'], ('#ededed', row['status'].replace('_', ' ')))
            ax.add_patch(Rectangle((j-.48, i-.43), .96, .86, color=color))
            ax.text(j, i, label, ha='center', va='center', fontsize=9)
    labels = [f"{c['role']} | d={c['target_depth_mm']:g} mm, mu={c['pair_static_friction']:g}/{c['pair_dynamic_friction']:g}, "
              f"a={c['tilt_amplitude_deg']:g} deg, B={c['force_budget_n']:g} N" for c in conditions]
    ax.set(xticks=[0,1], xticklabels=['Continuous straight', 'Continuous recenter + align'],
           yticks=range(len(conditions)), yticklabels=labels, xlim=(-.5,1.5), ylim=(len(conditions)-.5,-.5))
    ax.xaxis.tick_top(); ax.tick_params(length=0); ax.spines[:].set_visible(False)
    ax.set_title('Online sampled-budget branch outcomes\nIndependent process per policy; paired conclusions require the separate prefix audit', pad=35)
    fig.tight_layout()
    for suffix in ('png','pdf'):
        fig.savefig(directory/f'budget_outcomes.{suffix}', dpi=180, bbox_inches='tight')
    plt.close(fig)
    candidate = next((c for c in conditions if c['role']=='candidate'), None)
    if candidate:
        fig, axes = plt.subplots(3,2,figsize=(13,10), constrained_layout=True)
        recovery_force_values = [row['wrist_force_n'] for payload in result['plot_records'].values()
                                 for row in payload['recovery']
                                 if type(row.get('wrist_force_n')) in (int, float)]
        recovery_force_top = 1.12*max([candidate['force_budget_n'], *recovery_force_values])
        for col, policy in enumerate(POLICIES):
            payload = result['plot_records'].get(policy)
            branch = branches[(candidate['condition_id'], policy)]
            axes[0,col].set_title(('Straight' if policy=='straight' else 'Recenter + align')+' | '+(branch['outcome'] or branch['status']), fontsize=11)
            for ax in axes[:,col]:
                ax.grid(alpha=.2); ax.tick_params(labelsize=9)
            if not payload:
                for ax in axes[:,col]:
                    ax.text(.5,.5,'No closed branch\n'+branch['status'],ha='center',va='center',transform=ax.transAxes)
                    ax.set_xticks([]);ax.set_yticks([])
                continue
            hz=payload['record']['physics_hz']; ref=payload['reference']; rec=payload['recovery']; budget=candidate['force_budget_n']
            t=[r['time_s']-ref[0]['time_s'] for r in ref]
            ax=axes[0,col]; _shade(ax,ref,t)
            ax.plot(t,[r['wrist_force_n'] for r in ref],label='Raw wrist norm',lw=.8,color='#4c6b91')
            ax.plot(t,_mean100(ref,hz),label='100 ms mean',lw=1.4,color='#d68222')
            ax.axhline(budget,color='#b53031',ls='--',lw=1,label=f'{budget:g} N budget')
            ax.set(xlabel='Reference time (s)',ylabel='Reference wrist force (N)'); ax.legend(fontsize=8,loc='upper left')
            if not rec:
                for ax in axes[1:,col]:
                    ax.text(.5,.5,'Recovery not executed\n'+str(branch['reference_reason']),ha='center',va='center',transform=ax.transAxes)
                continue
            t=[r['recovery_time_s'] for r in rec]
            for ax in axes[1:,col]:_shade(ax,rec,t)
            ax=axes[1,col]
            ax.set_ylim(0, recovery_force_top)
            ax.plot(t,[r['wrist_force_n'] for r in rec],lw=1,color='#4c6b91',label='Raw wrist norm')
            ax.plot(t,_mean100(rec,hz),lw=1.5,color='#d68222',label='100 ms mean')
            ax.axhline(budget,color='#b53031',ls='--',lw=1)
            ax.set(xlabel='Recovery time (s)',ylabel='Recovery wrist force (N)')
            if branch['recovery_censored']:
                ax.plot(t[-1],rec[-1]['wrist_force_n'],'x',color='#b53031',ms=8)
                ax.text(.98,.8,'Stopped / right-censored',ha='right',transform=ax.transAxes,fontsize=8,color='#b53031')
            motion=withdrawal_plot_data(ref[-1]['depth_mm'],rec); ax=axes[2,col]
            ax.plot(motion['time_s'],motion['commanded_mm'],ls='--',label='Commanded withdrawal',color='#7b6297')
            ax.plot(motion['time_s'],motion['actual_mm'],label='Actual withdrawal',color='#3a8064')
            ax.set(xlabel='Recovery time (s)',ylabel='Withdrawal from reference endpoint (mm)')
            ax.legend(fontsize=8,loc='best')
        fig.suptitle('Main candidate: 18 mm, friction 1/1, tilt 1.5 deg, budget 4 N\n'
                     +'Pair: '+candidate['pair_state'].replace('_',' ')+'\n'
                     +'Recovery force axes share a scale; time ranges end at clearance or sampled termination',fontsize=12)
        for suffix in ('png','pdf'):
            fig.savefig(directory/f'budget_candidate_4n_profiles.{suffix}',dpi=180,bbox_inches='tight')
        plt.close(fig)
    (directory/'plot_sources.json').write_text(json.dumps(dict(source_study_sha256=result['summary']['source_study_sha256'],
              exporter_sha256=result['summary']['exporter_sha256'], input_files=result['summary']['input_files']),indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir',required=True,type=Path)
    parser.add_argument('--output-dir',required=True,type=Path)
    parser.add_argument('--plots',action='store_true')
    args=parser.parse_args(); result=summarize_study(args.study_dir); write_summary(result,args.output_dir)
    if args.plots:write_plots(result,args.output_dir)
    print(json.dumps(result['summary']['counts'],indent=2))


if __name__=='__main__':main()
