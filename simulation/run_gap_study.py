"""Run one fixed-geometry simulator process at a time; gate perturbations on aligned controls.

This coordinator needs only standard Python. Simulation children use forge_study.sh.
"""
import argparse
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.forge_gap_study import load_plan, subset_plan

TERMINAL_STUDY_STATUSES = ('complete', 'target_not_met')


def save(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def process_snapshot(pid):
    """Read Linux process identity without signalling or attaching to a process.

    PID alone is not identity: boot ID and kernel start ticks distinguish a
    reused PID. The command may legitimately change when bash execs conda.
    """
    try:
        proc = Path('/proc') / str(pid)
        stat = (proc / 'stat').read_text()
        fields = stat[stat.rfind(')') + 2:].split()
        if fields[0] in ('Z', 'X'):
            return None
        ticks = int(fields[19])
        boot_unix = next(int(line.split()[1]) for line in Path('/proc/stat').read_text().splitlines()
                         if line.startswith('btime '))
        command = [part.decode(errors='replace') for part in (proc / 'cmdline').read_bytes().split(b'\0') if part]
        return dict(pid=int(pid), process_group=int(fields[2]), start_time_ticks=ticks,
                    started_unix_s=boot_unix + ticks / os.sysconf('SC_CLK_TCK'),
                    boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(), command=command)
    except (FileNotFoundError, ProcessLookupError):
        return None


def process_snapshots():
    """Include surviving children after their original group leader has exited."""
    result = []
    for path in Path('/proc').iterdir():
        if not path.name.isdigit():
            continue
        try:
            snapshot = process_snapshot(int(path.name))
        except PermissionError:
            # Other users' inaccessible processes cannot be this user's child.
            continue
        if snapshot is not None:
            result.append(snapshot)
    return result


def execution_command_matches(record, observed):
    """Recognize both the original wrapper and its exec/conda/Python forms."""
    expected = record.get('command', [])
    if len(expected) < 3:
        return False
    suffix = expected[2:]
    launchers = {str(ROOT / 'forge_study.sh'), str(ROOT / 'simulation/launch_forge_study.py')}
    return (any(part in launchers for part in observed)
            and len(observed) >= len(suffix) and observed[-len(suffix):] == suffix)


def reconcile_running_executions(directory, manifest):
    """Refuse overlapping live children; retire stale records without killing PIDs.

    Older batches have only PID/command/wall time. For those, require the exact
    study command plus a process start no earlier than the recorded launch
    (allowing two seconds for /proc boot-time rounding). New records also retain
    boot ID and kernel start ticks. Surviving members of the launched process
    group remain protected even after the original conda process exits.
    """
    running = [record for record in manifest.get('executions', []) if record.get('status') == 'running']
    snapshots = process_snapshots() if running else []
    blockers = []
    for record in running:
        identity = record.get('process_identity')
        launched_pid = record.get('pid')
        matches = []
        for observed in snapshots:
            if not execution_command_matches(record, observed['command']):
                continue
            if launched_pid is not None and observed['pid'] != launched_pid and observed['process_group'] != launched_pid:
                continue
            if identity:
                if observed['boot_id'] != identity['boot_id']:
                    continue
                if observed['pid'] == launched_pid:
                    if observed['start_time_ticks'] != identity['start_time_ticks']:
                        continue
                elif observed['start_time_ticks'] < identity['start_time_ticks']:
                    continue
            elif observed['started_unix_s'] < record.get('started_unix_s', float('inf')) - 2.:
                continue
            matches.append(observed['pid'])
        if matches:
            blockers.append(f"{record['group']} (live PID(s) {', '.join(map(str, matches))}; log {record.get('log', 'unknown')})")
        else:
            record.update(status='interrupted', interrupted_unix_s=time.time(),
                          interruption_reason='Recorded simulator process exited, or its PID identity changed')
    if blockers:
        raise RuntimeError('Cannot resume: a previously launched simulator is still running: '
                           + '; '.join(blockers)
                           + '. Wait for it to finish before resuming. No process was signalled.')
    # Also protect a child writer whose coordinator died before saving its PID,
    # or whose execution record was already finalized during interrupted cleanup.
    for group in manifest['groups']:
        child_lock = directory / group['folder'] / '.run.lock'
        if child_lock.is_file():
            with child_lock.open('r') as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    raise RuntimeError(f"Cannot resume: {group['folder']} still holds its simulator run lock. "
                                       'Wait for the active child to finish. No process was signalled.') from None


def baseline_status(study):
    if not study.is_file():
        return 'not_run'
    manifest = json.loads(study.read_text())
    attempts = [a for a in manifest['attempts'] if a.get('slot') == 0 and a.get('status') == 'complete']
    if not attempts:
        return 'not_completed'
    a = attempts[-1]
    m = a.get('metrics', {})
    if not m.get('numerically_valid'):
        return 'numerical_review'
    if not m.get('grasp_retained') or not m.get('insertion_success'):
        return 'aligned_insertion_review'
    if m.get('force_budget_exceeded') or m.get('torque_budget_exceeded'):
        return 'aligned_budget_review'
    if not a.get('final_retreat', {}).get('safe_recovery'):
        return 'aligned_retreat_review'
    return 'passed'


def merge_tables(directory, groups):
    for name in ('trajectories.csv', 'checkpoints.csv', 'recovery_probes.csv', 'comparison.csv'):
        rows = []
        for group in groups:
            path = directory / group['folder'] / name
            if path.is_file():
                with path.open(newline='') as stream:
                    rows.extend(dict(row, source_study=group['folder']) for row in csv.DictReader(stream))
        if rows:
            with (directory / name).open('w', newline='') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for row in rows for k in row)))
                writer.writeheader()
                writer.writerows(rows)


def batch_status(manifest, phase):
    """Distinguish resumable work from exhausted attempts and gated cases."""
    groups = manifest['groups']
    pending_controls = [g for g in groups if g['baseline_status'] in ('not_run', 'not_completed')
                        and g.get('study_status') not in TERMINAL_STUDY_STATUSES]
    pending_perturbations = [g for g in groups if g['baseline_status'] == 'passed'
                             and g.get('study_status') not in TERMINAL_STUDY_STATUSES]
    if pending_controls or (phase == 'all' and pending_perturbations):
        return 'paused'
    if any(g['baseline_status'] == 'passed' and g.get('study_status') == 'target_not_met' for g in groups):
        # In controls-only mode an unfinished approved group still has intended
        # work for a subsequent all-phase run.
        if phase == 'all' or not pending_perturbations:
            return 'target_not_met'
    if any(g['baseline_status'] != 'passed' for g in groups):
        return 'review_required'
    if manifest['valid_references'] == manifest['planned_references']:
        return 'complete'
    if phase == 'controls' and pending_perturbations:
        return 'controls_complete'
    # A terminal child whose counts disagree with the plan needs review; never
    # report it as complete or repeatedly launch a process with no remaining work.
    return 'review_required'


def refresh(directory, manifest):
    total_complete = total_valid = 0
    lines = ['# Gap / insertion-tilt pilot', '',
             'Nominal clearance is one-sided radial clearance. Trigger depths are 6/10/14 mm.',
             'Commanded tilt ramps smoothly for 1 s after the first actual insertion-depth crossing.',
             'Numerical acceptance does not certify physical convergence. Missing probes remain unknown.', '',
             '| Radial gap [mm] | Complete references | Numerically screened | Aligned control | Study status |',
             '| ---: | ---: | ---: | --- | --- |']
    for group in manifest['groups']:
        path = directory / group['folder'] / 'study.json'
        group['baseline_status'] = baseline_status(path)
        if path.is_file():
            child = json.loads(path.read_text())
            complete = [a for a in child['attempts'] if a['status'] == 'complete']
            valid = {a['slot'] for a in complete if a.get('metrics', {}).get('numerically_valid')}
            group.update(complete_references=len(complete), valid_references=len(valid),
                         study_status=child['status'])
            total_complete += len(complete)
            total_valid += len(valid)
        lines.append(f"| {group['radial_clearance_mm']:g} | {group.get('complete_references', 0)} | "
                     f"{group.get('valid_references', 0)} | {group['baseline_status']} | {group.get('study_status', 'not_run')} |")
    manifest.update(complete_references=total_complete, valid_references=total_valid,
                    exhausted_groups=[g['folder'] for g in manifest['groups'] if g.get('study_status') == 'target_not_met'],
                    review_required_groups=[g['folder'] for g in manifest['groups']
                                            if g['baseline_status'] not in ('passed', 'not_run', 'not_completed')])
    lines += ['', f"Batch status: **{manifest['status']}**.",
              f"{total_valid} / {manifest['planned_references']} planned references passed numerical screening.",
              '', '[Reference outcomes](trajectories.csv) · [Recovery tests](recovery_probes.csv) · [Paired comparisons](comparison.csv)',
              '', 'Each child directory contains its frozen plan provenance, scene audit, raw traces, and report.']
    if manifest['exhausted_groups']:
        lines += ['', 'Attempt budgets exhausted in: ' + ', '.join(manifest['exhausted_groups'])
                  + '. These groups are terminal and will not launch again on resume; numerical rejects remain in the data.']
    if manifest['review_required_groups']:
        lines += ['', 'Aligned-control review required in: ' + ', '.join(manifest['review_required_groups'])
                  + '. Their perturbations remain gated.']
    if manifest.get('error'):
        lines += ['', 'Execution error: ' + manifest['error']]
    (directory / 'report.md').write_text('\n'.join(lines) + '\n')
    merge_tables(directory, manifest['groups'])
    save(directory / 'batch.json', manifest)


def execute_group(args, directory, group, stage, manifest):
    study_dir = directory / group['folder']
    command = ['bash', str(ROOT / 'forge_study.sh'), '--headless', '--mode', 'collect',
               '--case-plan', str(directory / group['plan']), '--output-dir', str(study_dir),
               '--radial-clearance-mm', str(group['radial_clearance_mm'])]
    if args.physics_hz is not None:
        command += ['--physics-hz', str(args.physics_hz)]
    if (study_dir / 'study.json').exists():
        command += ['--resume']
    if stage == 'controls':
        command += ['--max-new-attempts', '1']
    elif args.max_new_attempts is not None:
        command += ['--max-new-attempts', str(args.max_new_attempts)]
    log = directory / 'logs' / f"{group['folder']}_{stage}_{len(manifest['executions']):03d}.log"
    record = dict(group=group['folder'], stage=stage, command=command,
                  log=str(log.relative_to(directory)), status='running', started_unix_s=time.time())
    manifest['executions'].append(record)
    refresh(directory, manifest)
    print(f"Starting {group['folder']} {stage}; log: {log}", flush=True)
    start = time.monotonic()
    with log.open('w') as stream:
        process = subprocess.Popen(command, cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        record['pid'] = process.pid
        identity = process_snapshot(process.pid)
        if identity is not None:
            record['process_identity'] = {key: identity[key] for key in ('boot_id', 'start_time_ticks', 'started_unix_s')}
        save(directory / 'batch.json', manifest)
        try:
            code = process.wait()
        except BaseException:
            import signal
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=30)
            raise
    record.update(exit_code=code, status='complete' if code == 0 else 'error',
                  elapsed_wall_seconds=time.monotonic() - start)
    refresh(directory, manifest)
    print(f"Finished {group['folder']}: {record['elapsed_wall_seconds']:.1f}s, "
          f"baseline={group['baseline_status']}, exit={code}", flush=True)
    if code:
        raise RuntimeError(f"Simulator exited {code}; see {log}")


def run(args):
    plan = load_plan(args.case_plan)
    directory = args.output_dir.resolve()
    fingerprint = hashlib.sha256(args.case_plan.read_bytes()).hexdigest()
    directory.mkdir(parents=True, exist_ok=True)
    with (directory / '.batch.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if (directory / 'batch.json').exists():
            if not args.resume:
                raise ValueError('Existing batch requires --resume')
            manifest = json.loads((directory / 'batch.json').read_text())
            if manifest['plan_sha256'] != fingerprint or manifest.get('physics_hz') != args.physics_hz:
                raise ValueError('Cannot resume with a changed plan or physics setting')
            reconcile_running_executions(directory, manifest)
            manifest.pop('error', None)
        else:
            if args.resume:
                raise ValueError('No batch exists to resume')
            (directory / 'plans').mkdir()
            (directory / 'logs').mkdir()
            save(directory / 'plan.json', plan)
            manifest = dict(schema='Forge-gap-batch-v1', status='prepared', plan_sha256=fingerprint,
                            physics_hz=args.physics_hz, planned_references=len(plan['cases']),
                            groups=[], executions=[])
            for gap in sorted({c['radial_clearance_mm'] for c in plan['cases']}, reverse=True):
                folder = f'gap_{round(gap * 1000):04d}'
                path = Path('plans') / f'{folder}.json'
                save(directory / path, subset_plan(plan, radial_clearance_mm=gap))
                manifest['groups'].append(dict(radial_clearance_mm=gap, folder=folder, plan=str(path)))
        refresh(directory, manifest)
        if args.prepare_only:
            print(f"Prepared {manifest['planned_references']} cases in {len(manifest['groups'])} geometry groups: {directory}")
            return
        try:
            manifest['status'] = 'running'
            refresh(directory, manifest)
            # First validate the aligned control of every gap before any tilted run.
            for group in manifest['groups']:
                if (group['baseline_status'] in ('not_run', 'not_completed')
                        and group.get('study_status') not in TERMINAL_STUDY_STATUSES):
                    execute_group(args, directory, group, 'controls', manifest)
            if args.phase == 'all':
                for group in manifest['groups']:
                    if group['baseline_status'] != 'passed':
                        print(f"Review required at {group['folder']}: {group['baseline_status']}; perturbations paused.", flush=True)
                        continue
                    if group.get('study_status') not in TERMINAL_STUDY_STATUSES:
                        execute_group(args, directory, group, 'perturbations', manifest)
            manifest['status'] = batch_status(manifest, args.phase)
        except BaseException as error:
            manifest['status'] = 'paused' if isinstance(error, KeyboardInterrupt) else 'error'
            manifest['error'] = str(error)
            raise
        finally:
            refresh(directory, manifest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-plan', type=Path, default=ROOT / 'experiments/forge_gap_pilot.json')
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--phase', choices=('controls', 'all'), default='all')
    parser.add_argument('--physics-hz', type=int, choices=(120, 240, 480, 960))
    parser.add_argument('--max-new-attempts', type=int)
    parser.add_argument('--prepare-only', action='store_true')
    parser.add_argument('--resume', action='store_true')
    args = parser.parse_args()
    if args.max_new_attempts is not None and args.max_new_attempts < 1:
        parser.error('--max-new-attempts must be positive')
    run(args)


if __name__ == '__main__':
    main()
