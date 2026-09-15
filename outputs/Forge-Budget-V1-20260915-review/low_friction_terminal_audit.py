"""Read closed CSVs to audit the terminal low-friction reference force rise.

Run from any directory. This writes only the sibling review JSON, never a study
artifact. A 100 ms window means 24 consecutive 240 Hz observations, consistently
with the study's sample-count moving mean, not an interpolated signal integral.
"""
import argparse
import csv
import hashlib
import io
import json
import math
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_STUDY = HERE.with_name(HERE.name.removesuffix('-review'))
LOW = 'm_d18_fs050_fd050_a1p5__b04'
HIGH = 'm_d18_fs100_fd100_a1p5__b04'
FIELDS = ('time_s', 'physics_time_s', 'reference_step', 'phase', 'command_depth_mm',
          'command_pitch_deg', 'depth_mm', 'tilt_deg', 'wrist_force_n', 'wrist_torque_nm',
          'normal_load_n', 'force_norm_n', 'normal_force_world_z_n',
          'friction_force_world_z_n', 'grasp_slip_mm', 'grasp_slip_deg', 'min_separation_mm')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_verified(path, files, expected=None):
    data = path.read_bytes(); sha = digest(data)
    if expected is not None and sha != expected:
        raise ValueError(f'Closed-input SHA mismatch: {path}')
    files[str(path)] = sha
    return data


def parse_rows(data):
    if not data.endswith(b'\n'):
        raise ValueError('Closed CSV lacks its final newline')
    result = []
    for source in csv.DictReader(io.StringIO(data.decode())):
        if None in source or any(value is None for value in source.values()):
            raise ValueError('Malformed closed CSV row')
        row = {}
        for key in FIELDS:
            value = source[key]
            if key == 'phase':row[key] = value
            else:
                row[key] = float(value)
                if not math.isfinite(row[key]):raise ValueError(f'Nonfinite {key}')
        result.append(row)
    if not result:raise ValueError('No observations')
    return result


def window(rows, count):
    if len(rows) != count:return None
    forces = [r['wrist_force_n'] for r in rows]
    return dict(sample_count=count, first_reference_step=rows[0]['reference_step'],
                last_reference_step=rows[-1]['reference_step'], start_time_s=rows[0]['time_s'],
                end_time_s=rows[-1]['time_s'], endpoint_time_span_s=rows[-1]['time_s']-rows[0]['time_s'],
                mean_wrist_force_n=sum(forces)/count, max_wrist_force_n=max(forces),
                min_wrist_force_n=min(forces))


def hold_statistics(rows, count):
    holds = [r for r in rows if r['phase'] == 'hold']
    if not holds:return dict(sample_count=0, peak_raw_wrist_force_n=None, peak_mean100ms_n=None)
    if any(b['reference_step'] != a['reference_step']+1 for a,b in zip(holds,holds[1:])):
        raise ValueError('HOLD observations are not contiguous')
    windows = [window(holds[i-count+1:i+1],count) for i in range(count-1,len(holds))]
    highest = max(windows,key=lambda w:w['mean_wrist_force_n']) if windows else None
    return dict(sample_count=len(holds), first_reference_step=holds[0]['reference_step'],
                last_reference_step=holds[-1]['reference_step'],
                peak_raw_wrist_force_n=max(r['wrist_force_n'] for r in holds),
                peak_mean100ms_n=highest['mean_wrist_force_n'] if highest else None,
                complete_mean100ms_windows=len(windows), peak_mean100ms_window=highest)


def analyze(study):
    study = Path(study).resolve(); files = {}
    raw = read_verified(study/'study.json',files); manifest = json.loads(raw)
    if manifest['schema'] != 'Forge-budget-v1':raise ValueError('Unexpected study schema')
    records = {}; samples = {}; attempts = {}
    for identity in (LOW,HIGH):
        matches = [a for a in manifest['attempts'] if a['condition_id'] == identity and a['policy'] == 'straight']
        if len(matches) != 1 or matches[0]['status'] != 'complete':
            raise ValueError(f'Requires one closed straight branch: {identity}')
        attempt = matches[0]; attempts[identity] = attempt
        folder = (study/attempt['folder']).resolve()
        if study not in folder.parents:raise ValueError('Branch escapes study directory')
        record = json.loads(read_verified(folder/'run.json',files,attempt['run_sha256']))
        if (record['status'] != 'complete' or record['policy'] != 'straight'
                or record['condition']['condition_id'] != identity or record['sources'] != manifest['sources']):
            raise ValueError('Closed branch identity/source mismatch')
        filename = record['reference']['trajectory']; path = (folder/filename).resolve()
        if folder not in path.parents:raise ValueError('CSV escapes branch directory')
        records[identity] = record
        samples[identity] = parse_rows(read_verified(path,files,record['artifact_sha256'][filename]))
    low, high = records[LOW],records[HIGH]; rows = samples[LOW]; last,previous = rows[-1],rows[-2]
    hz = low['physics_hz']; count = math.ceil(.1*hz-1e-9)
    if hz != 240 or high['physics_hz'] != hz:raise ValueError('This bounded audit expects 240 Hz')
    for identity in (LOW,HIGH):
        if any(b['reference_step'] != a['reference_step']+1 for a,b in zip(samples[identity],samples[identity][1:])):
            raise ValueError('Reference steps are not contiguous')
    crossings = [i for i,r in enumerate(rows)
                 if r['wrist_force_n'] > low['effective_protocol']['force_budget_n']
                 or r['wrist_torque_nm'] > low['effective_protocol']['torque_budget_nm']]
    saved_crossing = low['reference']['evidence']['first_budget_crossing']
    if crossings != [len(rows)-1] or saved_crossing['index'] != crossings[0]:
        raise ValueError('Expected the only budget crossing at the last observed reference sample')
    if low['reference']['metrics']['termination_reason'] != 'operational_budget_exceeded' or low['recovery'] is not None:
        raise ValueError('Expected reference-budget termination with unobserved recovery')
    if any(r['phase'] != 'hold' for r in rows[-count-1:]):raise ValueError('Terminal windows cross a phase boundary')
    high_common = [r for r in samples[HIGH] if r['reference_step'] <= last['reference_step']]
    if high_common[-1]['reference_step'] != last['reference_step']:raise ValueError('No high-friction counterpart step')
    low_hold = hold_statistics(rows,count); high_hold = hold_statistics(samples[HIGH],count)
    for record,stat in ((low,low_hold),(high,high_hold)):
        saved = record['reference']['metrics']['hold_peak_wrist_force_mean100ms_n']
        if not math.isclose(saved,stat['peak_mean100ms_n'],rel_tol=1e-12,abs_tol=1e-12):
            raise ValueError('Recomputed HOLD moving mean disagrees with saved metrics')
    flags = ('reference_complete','numerically_valid','grasp_retained','reference_within_budget',
             'force_budget_exceeded','torque_budget_exceeded','budget_signals_finite','termination_reason')
    result = dict(schema='Forge-budget-low-friction-terminal-audit-v1', source_study=str(study),
        source_study_sha256=digest(raw), input_files=[dict(path=k,sha256=v) for k,v in files.items()],
        analyzer_sha256=digest(Path(__file__).read_bytes()),
        window_definition=dict(support_s=.1,physics_hz=hz,consecutive_sample_count=count,
            mean='arithmetic mean of raw wrist-force norms; complete same-phase windows only',
            endpoint_time_span_s=(count-1)/hz,observed_budget_signal='raw sample, not the moving mean'),
        low_friction=dict(condition=low['condition'],
            outcome=low['outcome'],reference_flags={k:low['reference']['metrics'].get(k) for k in flags},
            depth_gate_step=low['reference']['metrics']['depth_gate_step'],
            depth_gate_time_s=low['reference']['metrics']['depth_gate_time_s'],
            first_budget_crossing=saved_crossing,reference_sample_count=len(rows),
            reference_executed_step_count=len(rows)-1,last_six_observations=rows[-6:],
            terminal_increment_n=last['wrist_force_n']-previous['wrist_force_n'],
            previous_sample_force_n=previous['wrist_force_n'],
            last_complete_100ms_window=window(rows[-count:],count),
            previous_complete_100ms_window_excluding_trigger=window(rows[-count-1:-1],count),
            preceding_23_observations_of_terminal_window=window(rows[-count:-1],count-1),
            terminal_sample_contribution_to_last_window_mean_n=last['wrist_force_n']/count,
            observed_hold=low_hold,
            max_reference_force_before_trigger_n=max(r['wrist_force_n'] for r in rows[:-1]),
            numerical_screen_penetration_limit_mm=low['effective_protocol']['effective_radial_clearance_mm']*low['effective_protocol']['penetration_fraction'],
            terminal_penetration_mm=max(0.,-last['min_separation_mm']),
            observed_budget_crossing_indices=crossings,post_crossing_observed_samples=0,
            recovery_executed=False,subsequent_force_decay_observed=False,
            termination_response=low['termination_response']),
        high_friction_descriptive_control=dict(condition=high['condition'],outcome=high['outcome'],
            reference_flags={k:high['reference']['metrics'].get(k) for k in flags},
            depth_gate_step=high['reference']['metrics']['depth_gate_step'],
            depth_gate_time_s=high['reference']['metrics']['depth_gate_time_s'],
            reference_sample_count=len(samples[HIGH]),last_observation=samples[HIGH][-1],
            complete_observed_hold=high_hold,
            observed_hold_through_low_friction_terminal_step=hold_statistics(high_common,count),
            same_reference_step_observation=high_common[-1],
            terminal_100ms_at_same_reference_step=window(high_common[-count:],count),
            strict_matched_mechanical_states_claimed=False),
        descriptive_timing_difference=dict(
            low_minus_high_depth_gate_step=low['reference']['metrics']['depth_gate_step']-high['reference']['metrics']['depth_gate_step'],
            low_minus_high_hold_first_step=low_hold['first_reference_step']-high_hold['first_reference_step'],
            interpretation='A shared reference-step index is a shared elapsed observation time, not equal phase age or equal physical/contact state.'),
        interpretation_zh=[
            '低摩擦路径在 HOLD 末帧出现原始腕力突然增大并触发预算中止；可以称末帧突升或尖峰样升高。',
            '触发后未继续执行，未观测回落，不能证明这是短暂尖峰、孤立单帧脉冲或稳态持续高力。',
            '100 ms 均值与逐帧原始峰值是不同统计量；均值较低不改变原始信号严格大于 4 N 的超限判定。',
            '法向载荷总和与穿透筛查量也在末帧增大；数值筛查通过仅指满足既定阈值，不能认证该瞬变的实物真实性。',
            '该路径在参考 HOLD 中止，未执行撤出；不是低摩擦导致高拔出力或不可恢复卡死的证据。',
            'μ=1 路径仅作本次参数组合的描述性对照；摩擦改变后的实际深度、接触状态和轨迹不同，不能从一次结果推出摩擦与峰值的单调关系。',
            '高摩擦路径超出低摩擦终止步的观测只属该独立路径，不填入低摩擦路径的未观测未来。'])
    for name,sha in files.items():
        if name != str(study/'study.json') and digest(Path(name).read_bytes()) != sha:
            raise ValueError(f'Closed input changed during audit: {name}')
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--study-dir',type=Path,default=DEFAULT_STUDY)
    parser.add_argument('--output',type=Path,default=HERE/'low_friction_terminal_audit.json')
    args = parser.parse_args(); source=args.study_dir.resolve(); output=args.output.resolve()
    if source in output.parents:raise ValueError('Audit output must be outside the source study')
    result=analyze(source);output.write_text(json.dumps(result,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output),terminal_force_n=result['low_friction']['first_budget_crossing']['wrist_force_n'],
                         last_window=result['low_friction']['last_complete_100ms_window'],
                         previous_window=result['low_friction']['previous_complete_100ms_window_excluding_trigger']),indent=2))


if __name__ == '__main__':main()
