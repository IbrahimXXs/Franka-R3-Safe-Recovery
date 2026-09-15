"""Offline historical screening of wrist-force budgets; never online labels.

The lower budget is applied to the whole saved reference plus recovery. A
reference crossing makes its later recovery endpoint inadmissible. A recovery
crossing truncates the counterfactual record; later historical clearance cannot
make that lower-budget motion a success. Missing/censored records stay unknown.
"""
import argparse
from collections import Counter
import csv
import hashlib
import io
import json
import math
from pathlib import Path


DEFAULT_BUDGETS = (2., 3., 4., 5., 6., 8., 10.)
ROW_FIELDS = ('phase', 'time_s', 'physics_time_s', 'reference_step',
              'wrist_force_n', 'wrist_torque_nm', 'depth_mm', 'command_depth_mm',
              'tilt_deg', 'command_tilt_deg', 'grasp_slip_mm', 'grasp_slip_deg')
COMPLETE = 'historical_complete_within_budget'
EXCEEDED = 'historical_recovery_force_exceedance'


def _num(value):
    try:
        result = float(value)
        return result if math.isfinite(result) else None
    except (TypeError, ValueError):
        return None


def first_crossing(rows, key, budget, phase=None):
    """Strict > is the existing within_budget <= convention; no interpolation."""
    for index, row in enumerate(rows):
        value = _num(row.get(key))
        if (phase is None or row.get('phase') == phase) and value is not None and value > budget:
            start = _num(rows[0].get('time_s'))
            time = _num(row.get('time_s'))
            return dict(sample_index=index, signal=key, threshold=budget,
                        recording_time_s=time-start if start is not None and time is not None else None,
                        **{k:row.get(k) for k in ROW_FIELDS})
    return None


def _peak(rows, key):
    values = [_num(row.get(key)) for row in rows]
    values = [v for v in values if v is not None]
    return max(values) if values else None


def _signals_complete(rows):
    return bool(rows) and all(_num(r.get(k)) is not None for r in rows
                             for k in ('wrist_force_n', 'wrist_torque_nm'))


def reference_screen(attempt, rows, budget, torque_budget):
    metrics = attempt.get('metrics') or {}
    crossing = first_crossing(rows, 'wrist_force_n', budget)
    torque = first_crossing(rows, 'wrist_torque_nm', torque_budget)
    if attempt.get('status') != 'complete':
        state = 'unknown_reference_not_complete'
    elif not _signals_complete(rows):
        state = 'unknown_reference_signals_missing'
    elif metrics.get('numerically_valid') is not True:
        state = 'unknown_reference_numerically_invalid'
    elif crossing and (not torque or crossing['sample_index'] <= torque['sample_index']):
        state = 'reference_force_budget_exceeded'
    elif torque:
        state = 'reference_torque_budget_exceeded'
    elif metrics.get('grasp_retained') is not True:
        state = 'reference_grasp_gate_failed'
    elif metrics.get('reference_complete') is not True or metrics.get('reference_protocol_completed') is not True:
        state = 'reference_protocol_incomplete'
    else:
        state = 'historical_reference_admissible'
    return dict(state=state, admissible=state == 'historical_reference_admissible',
                first_force_crossing=crossing, first_torque_crossing=torque,
                peak_wrist_force_n=_peak(rows, 'wrist_force_n'),
                source_numerically_valid=metrics.get('numerically_valid'),
                source_grasp_retained=metrics.get('grasp_retained'),
                source_reference_complete=metrics.get('reference_complete'),
                source_reason=metrics.get('reference_termination_reason', metrics.get('termination_reason')))


def recovery_screen(source, rows, reference, budget, torque_budget, *, matched=True):
    force = first_crossing(rows, 'wrist_force_n', budget)
    torque = first_crossing(rows, 'wrist_torque_nm', torque_budget)
    if not reference['admissible']:
        state = 'reference_not_admissible'
    elif not matched:
        state = 'unknown_replay_unmatched_or_invalid'
    elif not source or not rows:
        state = 'unknown_recovery_not_observed'
    elif not _signals_complete(rows):
        state = 'unknown_recovery_signals_missing'
    elif source.get('numerically_valid') is not True:
        state = 'unknown_recovery_numerically_invalid'
    elif source.get('label_eligible') is not True:
        state = 'unknown_source_recovery_label_ineligible'
    elif force and (not torque or force['sample_index'] <= torque['sample_index']):
        state = EXCEEDED
    elif torque:
        state = 'historical_recovery_torque_exceedance'
    elif source.get('grasp_retained') is not True:
        state = 'historical_other_termination_grasp_gate'
    elif source.get('cleared') is True and source.get('recovery_censored') is False:
        state = COMPLETE
    else:
        state = 'unknown_right_censored'
    return dict(state=state, first_force_crossing=force, first_torque_crossing=torque,
                phase_first_force_crossings={phase:first_crossing(rows, 'wrist_force_n', budget, phase)
                                             for phase in ('stop', 'realign', 'retreat', 'clear_hold')},
                peak_wrist_force_n=_peak(rows, 'wrist_force_n'),
                source_reason=source.get('reason', source.get('termination_reason')),
                source_numerically_valid=source.get('numerically_valid'),
                source_grasp_retained=source.get('grasp_retained'),
                source_cleared=source.get('cleared'), source_recovery_censored=source.get('recovery_censored'),
                source_safe_recovery=source.get('safe_recovery'),
                historical_recovery_rows=len(rows),
                lower_budget_history_truncates_at_sample=force['sample_index'] if state == EXCEEDED else
                                                        torque['sample_index'] if state == 'historical_recovery_torque_exceedance' else None)


def analyze_case(attempt, reference_rows, straight_rows, realign_rows, budget, torque_budget=1.):
    ref = reference_screen(attempt, reference_rows, budget, torque_budget)
    straight_source = attempt.get('final_retreat') or {}
    realign_source = next((p for p in attempt.get('probes', []) if p.get('policy') == 'realign'), {})
    matched = all(realign_source.get(k) is True for k in
                  ('replay_matched', 'replay_prefix_matched', 'replay_prefix_numerically_valid'))
    straight = recovery_screen(straight_source, straight_rows, ref, budget, torque_budget)
    realign = recovery_screen(realign_source, realign_rows, ref, budget, torque_budget, matched=matched)
    states = (straight['state'], realign['state'])
    if not ref['admissible']:
        pair_state = 'reference_not_admissible'
    elif not matched:
        pair_state = 'unknown_replay_unmatched_or_invalid'
    elif states == (EXCEEDED, COMPLETE):
        pair_state = 'historical_straight_exceeded_realign_completed'
    elif states == (COMPLETE, EXCEEDED):
        pair_state = 'historical_straight_completed_realign_exceeded'
    elif states == (COMPLETE, COMPLETE):
        pair_state = 'historical_both_completed_within_budget'
    elif states == (EXCEEDED, EXCEEDED):
        pair_state = 'historical_both_force_exceeded'
    else:
        pair_state = 'unknown_or_other_termination'
    return dict(force_budget_n=budget, torque_budget_nm=torque_budget, reference=ref,
                straight=straight, realign=realign, source_replay_eligible=matched,
                source_replay_prefix_equal=realign_source.get('replay_prefix_equal'), pair_state=pair_state)


def _read_rows(path, files):
    data = path.read_bytes()
    files.append(dict(path=str(path), sha256=hashlib.sha256(data).hexdigest()))
    rows = []
    for row in csv.DictReader(io.StringIO(data.decode())):
        rows.append({k:row.get(k) if k == 'phase' else _num(row.get(k)) for k in ROW_FIELDS})
    return rows


def _flat_entry(entry):
    row = {k:entry[k] for k in ('study', 'physics_hz', 'case_id', 'target_depth_mm', 'pair_static_friction',
                               'pair_dynamic_friction', 'tilt_amplitude_deg', 'force_budget_n', 'torque_budget_nm')}
    row.update(reference_state=entry['reference']['state'], reference_admissible=entry['reference']['admissible'],
               reference_peak_wrist_force_n=entry['reference']['peak_wrist_force_n'],
               source_reference_reason=entry['reference']['source_reason'],
               source_replay_eligible=entry['source_replay_eligible'], pair_state=entry['pair_state'])
    for name in ('reference', 'straight', 'realign'):
        detail = entry[name]
        row[name+'_state'] = detail['state']
        row[name+'_peak_wrist_force_n'] = detail['peak_wrist_force_n']
        crossing = detail['first_force_crossing'] or {}
        for key in ('sample_index', 'phase', 'time_s', 'recording_time_s', 'wrist_force_n', 'depth_mm', 'command_depth_mm'):
            row[f'{name}_first_force_crossing_{key}'] = crossing.get(key)
        if name != 'reference':
            row[name+'_source_recovery_censored'] = detail['source_recovery_censored']
            row[name+'_source_reason'] = detail['source_reason']
            for phase, event in detail['phase_first_force_crossings'].items():
                row[f'{name}_{phase}_first_force_crossing_time_s'] = (event or {}).get('time_s')
    return row


def separation_intervals(attempt, reference_rows, straight_rows, realign_rows, torque_budget):
    # A very high force screen still enforces source numerical/grasp/clearance,
    # torque and matching requirements. This does not mutate the source labels.
    screen = analyze_case(attempt, reference_rows, straight_rows, realign_rows, 1e100, torque_budget)
    if screen['pair_state'] != 'historical_both_completed_within_budget':
        return []
    r = screen['reference']['peak_wrist_force_n']
    s = screen['straight']['peak_wrist_force_n']
    a = screen['realign']['peak_wrist_force_n']
    result = []
    for passing, failing, low, high in [('realign', 'straight', max(r, a), s),
                                       ('straight', 'realign', max(r, s), a)]:
        if low < high:
            result.append(dict(within_budget_policy=passing, exceeding_policy=failing,
                               lower_inclusive_n=low, upper_exclusive_n=high, width_n=high-low))
    return result


def analyze_studies(study_dirs, budgets):
    entries = []; intervals = []; studies = []; files = []
    for source in study_dirs:
        directory = Path(source).resolve()
        data = (directory/'study.json').read_bytes()
        manifest = json.loads(data)
        if manifest.get('study') != 'Forge-mechanics-v1':
            raise ValueError(f'Expected mechanics study: {directory}')
        studies.append(dict(path=str(directory), status=manifest.get('status'),
                            study_sha256=hashlib.sha256(data).hexdigest(),
                            source_force_budget_n=manifest['protocol']['force_budget_n'],
                            source_torque_budget_nm=manifest['protocol']['torque_budget_nm']))
        planned = manifest['case_plan']['cases']
        if len({c['case_id'] for c in planned}) != len(planned):
            raise ValueError(f'Duplicate planned cases: {directory}')
        by_id = {}; complete = {}
        for attempt in manifest.get('attempts', []):
            by_id[attempt['case_id']] = attempt
            if attempt.get('status') == 'complete':
                if attempt['case_id'] in complete:
                    raise ValueError(f'Multiple complete attempts: {directory}/{attempt["case_id"]}')
                complete[attempt['case_id']] = attempt
        for case in planned:
            attempt = complete.get(case['case_id'], by_id.get(case['case_id'], {**case, 'status':'not_started'}))
            streams = {'reference':[], 'straight':[], 'realign':[]}
            if attempt.get('status') == 'complete':
                folder = (directory/attempt['folder']).resolve()
                if directory not in folder.parents:
                    raise ValueError(f'Attempt outside source study: {folder}')
                trajectory_data = (folder/'trajectory.json').read_bytes()
                if json.loads(trajectory_data) != attempt:
                    raise ValueError(f'Completed trajectory disagrees with study: {folder}')
                files.append(dict(path=str(folder/'trajectory.json'), sha256=hashlib.sha256(trajectory_data).hexdigest()))
                streams['reference'] = _read_rows(folder/'insertion.csv', files)
                for policy, filename in [('straight', 'final_retreat.csv'), ('realign', 'terminal_realign_recovery.csv')]:
                    path = folder/filename
                    if path.is_file():
                        streams[policy] = _read_rows(path, files)
            identity = dict(study=directory.name, physics_hz=manifest['physics_hz'],
                            **{k:case[k] for k in ('case_id', 'target_depth_mm', 'pair_static_friction',
                                                   'pair_dynamic_friction', 'tilt_amplitude_deg')})
            torque_budget = manifest['protocol']['torque_budget_nm']
            for budget in budgets:
                entries.append({**identity, **analyze_case(attempt, streams['reference'], streams['straight'],
                                                           streams['realign'], budget, torque_budget)})
            for interval in separation_intervals(attempt, streams['reference'], streams['straight'], streams['realign'], torque_budget):
                intervals.append({**identity, **interval})
    counts = []
    for study in studies:
        name = Path(study['path']).name
        for budget in budgets:
            subset = [e for e in entries if e['study'] == name and e['force_budget_n'] == budget]
            counts.append(dict(study=name, force_budget_n=budget, planned_conditions=len(subset),
                               reference_states=dict(Counter(e['reference']['state'] for e in subset)),
                               reference_admissible=sum(e['reference']['admissible'] for e in subset),
                               straight_states=dict(Counter(e['straight']['state'] for e in subset)),
                               realign_states=dict(Counter(e['realign']['state'] for e in subset)),
                               pair_states=dict(Counter(e['pair_state'] for e in subset))))
    return dict(schema='Forge-mechanics-offline-historical-budget-sweep-v1', studies=studies,
                budgets_n=list(budgets), case_budget_rows=len(entries), entries=entries, counts=counts,
                historical_separation_intervals=intervals,
                interpretation={
                    'status':'Offline historical re-evaluation only; no new online trials or source labels. A lower-budget controller would interrupt the recorded path, so later samples are not its counterfactual continuation.',
                    'signal':'Raw wrist-force Euclidean norm at every saved sample; strict force > budget is the existing exceedance rule. A 100 ms mean is not used for budget decisions.',
                    'event_time':'time_s is the saved reference-relative experiment clock; recording_time_s subtracts the first row of the corresponding reference or recovery CSV. An observed threshold crossing can overshoot the threshold in one step; a sampled abort threshold is not a hard continuous-time force cap.',
                    'scope':'Budget applies to the entire saved reference from the first post-reset sample plus the chosen recovery, including copied t=0, STOP, recenter+align, RETREAT and clearance hold if present.',
                    'reference':'A reference force crossing makes its later historical recovery starting point inadmissible under that budget; recovery phase crossings may still be listed as descriptive history, not reachable outcomes.',
                    'torque':'The source torque budget is retained separately. Numerical invalidity, missing signals, grasp gates and incomplete reference protocols exclude an admissible reference.',
                    'censor':'A recovery force crossing is an observed historical exceedance; later clearance is discarded for the proposed lower budget. A record that ends without a crossing and without uncensored clearance is unknown, never a success.',
                    'policy':'A paired comparison requires within-run endpoint and whole-prefix tolerance matching plus prefix numerical validity. Bitwise prefix equality is diagnostic. Realign recenters XY and aligns orientation.',
                    'interval':'The exact historical separation interval includes the reference and whole recovery maxima, not retreat peaks alone. Its lower bound is inclusive; upper bound is exclusive. Intervals selected from these traces are post hoc candidates for new online tests.',
                    'replication':'Study and case IDs remain separate. Reused conditions or identical traces across study folders are not independent replicates. No statistical significance or rate robustness is inferred.',
                    'missing':'Pending cases are included in the planned denominator, but their live CSVs are never opened. Missing and unobserved fields remain null.'},
                input_files=files, exporter_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def write_outputs(result, output_dir):
    directory = Path(output_dir).resolve()
    for study in result['studies']:
        source = Path(study['path'])
        if directory == source or source in directory.parents:
            raise ValueError('Output directory must be outside input studies')
    directory.mkdir(parents=True, exist_ok=True)
    (directory/'budget_sweep.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    for filename, rows in [('budget_sweep.csv', [_flat_entry(e) for e in result['entries']]),
                           ('historical_separation_intervals.csv', result['historical_separation_intervals'])]:
        with (directory/filename).open('w', newline='') as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]) if rows else ['case_id'])
            writer.writeheader(); writer.writerows(rows)
    lines = ['# 历史轨迹的离线力预算重评', '',
             '**这不是低预算闭环实验结果。** 未修改旧轨迹或旧标签；所有阈值均为事后分析候选。', '',
             '按保存的每步原始腕力合力判断 `force > budget`；力预算同时覆盖参考插入与整个恢复过程。',
             '参考阶段先超限时，原恢复起点不可视为该低预算下可达；恢复超限后的旧轨迹即使最终退出，也不能据此判成功。',
             '无超限但未完成退出的右删失记录仍为未知。扭矩预算沿用各来源设置，配对必须满足各自重放匹配。', '',
             '## 各预算下的历史筛选', '',
             '| 来源 | 预算 N | 计划条件 | 参考可接纳 | 直退完成且未超限 | 直退恢复超限 | 匹配直退超限／回正完成 |',
             '|---|---:|---:|---:|---:|---:|---:|']
    for count in result['counts']:
        lines.append(f"| {count['study']} | {count['force_budget_n']:g} | {count['planned_conditions']} | {count['reference_admissible']} | "
                     f"{count['straight_states'].get(COMPLETE, 0)} | {count['straight_states'].get(EXCEEDED, 0)} | "
                     f"{count['pair_states'].get('historical_straight_exceeded_realign_completed', 0)} |")
    lines += ['', '## 匹配历史轨迹的策略分离窗口', '',
              '下界同时覆盖参考和通过策略的全程峰值；上界为另一策略的全程峰值。区间左闭右开。', '',
              '| 来源／案例 | 历史通过策略 | 历史超限策略 | 预算区间 N |', '|---|---|---|---|']
    for item in result['historical_separation_intervals']:
        lines.append(f"| {item['study']} / {item['case_id']} | {item['within_budget_policy']} | {item['exceeding_policy']} | "
                     f"[{item['lower_inclusive_n']:.6f}, {item['upper_exclusive_n']:.6f}) |")
    lines += ['', '完整逐案例、首次超限采样及 STOP/REALIGN/RETREAT 事件见 [budget_sweep.csv](budget_sweep.csv)；',
              '未知原因、原始来源标签、输入 SHA 与解释见 [budget_sweep.json](budget_sweep.json)。', '',
              '相同条件在不同来源目录中可能复用同一物理轨迹，不能将全部行数当作独立重复次数。',
              '不同频率改变控制与采样频率，且初态可不同；本表不能证明频率稳健性。', '']
    (directory/'budget_sweep_zh.md').write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir', type=Path, required=True, action='append')
    parser.add_argument('--budgets', type=float, nargs='+', default=list(DEFAULT_BUDGETS))
    parser.add_argument('--output-dir', type=Path, required=True)
    args = parser.parse_args()
    budgets = sorted(set(args.budgets))
    if any(not math.isfinite(b) or b <= 0 for b in budgets):
        parser.error('Budgets must be finite and positive')
    result = analyze_studies(args.study_dir, budgets)
    write_outputs(result, args.output_dir)
    print(json.dumps(dict(case_budget_rows=result['case_budget_rows'],
                          separation_intervals=result['historical_separation_intervals'],
                          output_dir=str(args.output_dir.resolve())), indent=2))


if __name__ == '__main__':
    main()
