"""Read saved references and classify generic tracking stalls by location.

This standalone audit starts no simulation and changes no study, raw recording,
protocol, or label. It writes only the requested review directory. The original
predicate is reconstructed from each saved protocol and timestep, then a
separate in-hole predicate requires the entire trailing window to have depth>0.
"""
import argparse
from collections import Counter
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REVIEW_DIR = Path(__file__).resolve().parent


def read_rows(path):
    with path.open(newline='') as stream:
        return [{key: value if key == 'phase' else value == 'True' if value in ('True', 'False')
                 else float(value) if value else None for key, value in row.items()}
                for row in csv.DictReader(stream)]


def derived_events(rows, protocol, hz):
    """Keep original event transitions and independently gate the sample mask.

    Filtering only the original event list would miss a stall that begins before
    entry and stays active across entry. Applying the location gate to every
    sample preserves the later first fully-in-hole confirmation in that case.
    """
    window = max(1, round(protocol['stall_window_s'] * hz))
    original, in_hole = [], []
    previous_original = previous_in_hole = False
    for index, row in enumerate(rows):
        active = fully_in_hole = False
        if index >= window:
            past = rows[index - window]
            active = (row['phase'] == 'insert' and past['phase'] == 'insert'
                      and row['command_depth_mm'] - past['command_depth_mm'] >= protocol['stall_command_progress_mm']
                      and row['depth_mm'] - past['depth_mm'] < protocol['stall_progress_mm'])
            fully_in_hole = active and min(r['depth_mm'] for r in rows[index - window:index + 1]) > 0.
        if active and not previous_original:
            original.append(event_record(rows, index, window))
        if fully_in_hole and not previous_in_hole:
            in_hole.append(event_record(rows, index, window))
        previous_original, previous_in_hole = active, fully_in_hole
    return original, in_hole


def event_record(rows, index, window):
    row, past = rows[index], rows[index - window]
    history = rows[index - window:index + 1]
    low, high = min(r['depth_mm'] for r in history), max(r['depth_mm'] for r in history)
    location = 'fully_in_hole' if low > 0 else 'before_entry' if high < 0 else 'entry_transition'
    normal_peak = max(r['normal_load_n'] for r in history)
    contact_peak = max(r['contact_count'] for r in history)
    lowest = min(r['lowest_peg_z_above_mouth_mm'] for r in history)
    no_contact = normal_peak == 0 and contact_peak == 0
    classification = ('pre_entry_no_contact_tracking_stall' if location == 'before_entry' and lowest > 0 and no_contact
                      else 'in_hole_tracking_stall' if location == 'fully_in_hole'
                      else 'entry_or_contact_geometry_review')
    fields = ('time_s', 'phase', 'depth_mm', 'command_depth_mm', 'normal_load_n', 'contact_count',
              'force_norm_n', 'wrist_force_n', 'wrist_torque_nm', 'tilt_deg', 'command_tilt_deg',
              'lowest_peg_z_above_mouth_mm', 'reference_step')
    return dict(index=index, raw_confirmation={key: row.get(key) for key in fields},
                classification=classification, window_location=location,
                window_start_index=index - window, window_start_time_s=past['time_s'],
                window_depth_min_mm=low, window_depth_max_mm=high,
                actual_window_progress_mm=row['depth_mm'] - past['depth_mm'],
                command_window_progress_mm=row['command_depth_mm'] - past['command_depth_mm'],
                window_max_normal_load_n=normal_peak, window_max_contact_count=contact_peak,
                window_min_lowest_peg_z_above_mouth_mm=lowest,
                contact_observed_in_window=not no_contact)


def audit_study(path, completed_only):
    content = path.read_bytes()
    study = json.loads(content)
    result = dict(study=str(path.relative_to(ROOT)) if path.is_relative_to(ROOT) else str(path),
                  study_sha256=hashlib.sha256(content).hexdigest(),
                  physics_hz=study['physics_hz'], study_status_at_read=study['status'], attempts=[])
    for attempt in study['attempts']:
        if completed_only and attempt.get('status') != 'complete':
            continue
        source = path.parent / attempt['folder'] / 'insertion.csv'
        if not source.is_file():
            continue
        rows = read_rows(source)
        original, in_hole = derived_events(rows, study['protocol'], study['physics_hz'])
        checkpoints = attempt.get('checkpoints', [])
        for event in [*original, *in_hole]:
            event['matching_recorded_checkpoints'] = [dict(
                checkpoint_kind=cp.get('checkpoint_kind'), checkpoint_id=cp.get('checkpoint_id'),
                Y_R_tested=cp.get('Y_R_tested'),
                probes=[{key: probe.get(key) for key in ('policy', 'label_eligible', 'replay_matched', 'safe_recovery', 'reason')}
                        for probe in cp.get('probes', [])])
                for cp in checkpoints if cp.get('reached') and cp.get('step') == event['raw_confirmation']['reference_step']]
        result['attempts'].append(dict(
            trajectory_id=attempt['trajectory_id'], case_id=attempt.get('case_id'),
            status=attempt['status'], parent_numerically_valid=attempt.get('metrics', {}).get('numerically_valid'),
            raw_csv=str(source.relative_to(ROOT)) if source.is_relative_to(ROOT) else str(source),
            raw_csv_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            generic_stall_events=original, derived_full_window_in_hole_events=in_hole,
            first_stall_scope='generic_command_tracking_stall',
            raw_first_stall_location=original[0]['window_location'] if original else 'no_generic_stall',
            raw_first_stall_is_first_in_hole_event=bool(original and in_hole and original[0]['index'] == in_hole[0]['index']),
            first_in_hole_has_recorded_recovery_checkpoint=bool(in_hole and in_hole[0]['matching_recorded_checkpoints']),
            first_in_hole_has_recorded_recovery_probe=bool(in_hole and any(cp['probes'] for cp in in_hole[0]['matching_recorded_checkpoints'])),
        ))
    return result


def summarize(studies):
    attempts = [a for study in studies for a in study['attempts']]
    classes = Counter(a['raw_first_stall_location'] for a in attempts)
    counts = dict(completed_reference_attempts=sum(a['status'] == 'complete' for a in attempts),
                  distinct_cases=len({a['case_id'] for a in attempts}),
                  numerically_valid_attempts=sum(a['parent_numerically_valid'] is True for a in attempts),
                  numerically_invalid_attempts=sum(a['parent_numerically_valid'] is False for a in attempts),
                  raw_first_stall_locations=dict(classes),
                  valid_attempts_with_in_hole_stall=sum(a['parent_numerically_valid'] is True and bool(a['derived_full_window_in_hole_events']) for a in attempts),
                  invalid_attempts_with_in_hole_stall=sum(a['parent_numerically_valid'] is False and bool(a['derived_full_window_in_hole_events']) for a in attempts))
    return counts


def write_review(pilot_root, diagnostic_path, output):
    studies = [audit_study(path.resolve(), completed_only=True) for path in sorted(pilot_root.glob('gap_*/study.json'))]
    diagnostic = audit_study(diagnostic_path.resolve(), completed_only=False)
    summary = summarize(studies)
    value = dict(
        schema='offline-stall-location-audit-v1', generated_utc=datetime.now(timezone.utc).isoformat(),
        audit_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        interpretation='Original first_stall is generic command tracking; in-hole tracking stall is not itself proof of jamming.',
        derivation='Apply the saved trailing progress predicate; independently require every depth in its trailing window to be >0 mm.',
        prohibition='Do not inherit a recovery label or force cost from an earlier free-space checkpoint for a later in-hole event. Derived events without a matching recorded recovery checkpoint have unknown recovery outcomes.',
        pilot_summary=summary, pilot_studies=studies, diagnostic=diagnostic,
    )
    output.mkdir(parents=True, exist_ok=True)
    (output / 'stall_location_audit.json').write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    lines = ['# 停滞事件位置审计（离线）', '',
             '**未修改实验、原始数据或标签，未启动仿真。** `first_stall` 的原始语义是命令跟踪停滞；它没有要求插销已进孔，也没有要求存在接触。', '',
             '## 120 Hz 首批实验', '',
             f"- {summary['completed_reference_attempts']} 条完整尝试，覆盖 {summary['distinct_cases']} 个计划案例；{summary['numerically_valid_attempts']} 条数值有效，{summary['numerically_invalid_attempts']} 条数值无效。",
             f"- 首个事件位置：{summary['raw_first_stall_locations']}。没有进孔前的首个停滞事件。",
             f"- 孔内停滞涉及 {summary['valid_attempts_with_in_hole_stall']} 条数值有效尝试和 {summary['invalid_attempts_with_in_hole_stall']} 条数值无效尝试；后者不能作为可信卡死证据。", '',
             '## 240 Hz 诊断', '',
             '| 轨迹 | 事件 | 时间 [s] | 深度 [mm] | 接触法向载荷 [N] | 腕部力 [N] | 命令倾角 [°] | 位置分类 |',
             '| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |']
    for attempt in diagnostic['attempts']:
        for ordinal, event in enumerate(attempt['generic_stall_events'], 1):
            row = event['raw_confirmation']
            lines.append(f"| {attempt['trajectory_id']} | {ordinal} | {row['time_s']:.6f} | {row['depth_mm']:.6f} | {row['normal_load_n']:.6f} | {row['wrist_force_n']:.6f} | {row['command_tilt_deg']:.3f} | {event['classification']} |")
    for attempt in diagnostic['attempts']:
        if attempt['derived_full_window_in_hole_events'] and not attempt['first_in_hole_has_recorded_recovery_probe']:
            lines += ['', f"`{attempt['trajectory_id']}` 的首次孔内停滞**没有对应的已记录恢复探针**，因此该时刻的恢复结果未知。"]
    lines += ['', '第一个事件出现在孔口上方、接触载荷为零且扰动尚未施加，属于进孔前的跟踪迟滞。后续孔内事件应单独分析。原 `first_stall` 恢复分支对应第一个事件，不能把它的安全标签或力峰值转移给后续孔内事件。', '',
              '## 最小后处理修正', '',
              '1. 保留原 `first_stall` 与所有标签；增加事件位置分类和 `first_stall_scope=generic_command_tracking_stall`。',
              '2. 从原始采样重新计算同一停滞谓词，再要求完整确认窗口内 `depth_mm > 0`，得到独立的首次孔内停滞。不能仅过滤原事件列表，否则会漏掉从孔外持续到孔内的停滞。',
              '3. 单独记录是否观察到接触、命令/实际窗口进度、数值有效性和确切 `reference_step`。孔内跟踪停滞仍不等于已证明卡死。',
              '4. 只通过相同参考采样点关联已记录的恢复探针；无对应探针时，孔内恢复结果保持未知。终止状态探针不能替代较早的孔内停滞探针。', '',
              '## 建议测试点', '',
              '- 孔外零接触迟滞后发生独立孔内停滞：保留两个事件，首次孔内事件不能继承早期标签。',
              '- 单次连续停滞跨过孔口：深度门控必须逐采样计算，而不是只过滤原上升沿。',
              '- 确认窗口中任一点深度等于或小于零：不能归为完整窗口孔内停滞。',
              '- 数值无效轨迹仍保留诊断位置，但不给可信卡死结论。',
              '- 无事件、未完成参考和缺失恢复探针分别保留，不把缺失当成安全或失败。', '',
              f'复现：`python3 {Path(__file__).resolve().relative_to(ROOT)}`。脚本只读取原始实验输出，写入本审阅目录；JSON 保存输入文件哈希、全部逐轨迹事件以及检查点关联。', '']
    (output / 'stall_location_audit.md').write_text('\n'.join(lines))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot-root', type=Path, default=ROOT / 'outputs/Forge-GapTilt-Pilot-20260915')
    parser.add_argument('--diagnostic-study', type=Path, default=ROOT / 'outputs/Forge-GapTilt-Diagnostic-240Hz-20260915/study.json')
    parser.add_argument('--output-dir', type=Path, default=REVIEW_DIR)
    args = parser.parse_args()
    write_review(args.pilot_root, args.diagnostic_study, args.output_dir)


if __name__ == '__main__':
    main()
