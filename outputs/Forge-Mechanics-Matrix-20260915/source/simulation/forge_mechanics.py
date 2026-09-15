"""Depth-controlled, material-audited mechanics collection; simulation only."""
from contextlib import contextmanager
from dataclasses import asdict, replace
import fcntl
import gzip
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.utils.math import quat_slerp
from forge_backend import resolve_physics_rate
from forge_experiment import Stream, guarded, recover, save
from forge_mechanics_backend import MechanicsBench
from research.forge_protocol import load_protocol, radial_clearance, retained, screened, within_budget, recovery_summary
from research.forge_events import recovery_termination
from research.forge_mechanics_plan import SCHEMA, MechanicsState, load_plan
from research.forge_mechanics_metrics import mechanics_reference_metrics, mechanics_recovery_metrics
from research.forge_mechanics_protocol import reference_quality, compare_prefixes, aligned_control_passed, control_key, validate_resume


SOURCE_PATHS = tuple(ROOT / p for p in (
    'simulation/forge_mechanics.py', 'simulation/launch_forge_mechanics.py',
    'simulation/forge_mechanics_backend.py', 'research/forge_mechanics_contacts.py',
    'research/forge_mechanics_plan.py', 'research/forge_mechanics_metrics.py',
    'research/forge_mechanics_protocol.py', 'simulation/forge_backend.py',
    'simulation/forge_experiment.py', 'simulation/contact.py', 'research/forge_protocol.py',
    'research/phase2_protocol.py', 'research/forge_gap_metrics.py', 'research/forge_geometry.py',
    'research/forge_events.py', 'research/future_stall.py', 'research/forge_gap_study.py',
    'research/forge_collection.py', 'research/phase2b.py',
))


@contextmanager
def contact_recording(bench, path, initial=None):
    """Pair each observation with raw contacts by its physical timestamp.

    Normal contacts and friction anchors remain separate streams. Recovery
    CSV time starts at zero while its bench clock continues from the reference.
    """
    original = bench.observe
    offset = initial['time_s'] if initial is not None else 0.
    with gzip.open(path, 'wt', compresslevel=1) as stream:
        def record(row):
            value = dict(time_s=row['time_s'], physics_time_s=row['physics_time_s'],
                         recording_time_s=row['time_s']-offset, phase=row['phase'],
                         contacts=bench.contact_snapshot())
            stream.write(json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n')
        def observe(*args, **kwargs):
            row = original(*args, **kwargs)
            record(row)
            return row
        bench.observe = observe
        try:
            if initial is not None:
                record(dict(initial, phase='stop'))
            yield
        finally:
            bench.observe = original


def reference(bench, case, seed, folder, basename='insertion', stop_step=None):
    state = MechanicsState(case)
    p = bench.protocol
    stream = Stream(folder/f'{basename}.csv')
    reason = 'reference_time_limit'
    try:
        with contact_recording(bench, folder/f'{basename}_contacts.jsonl.gz'):
            first = bench.prepare(seed)
            start_p = bench.env.fingertip_midpoint_pos.clone()
            start_q = bench.env.fingertip_midpoint_quat.clone()
            above_p, above_q = bench.target(-10.)
            state.observe(0., first['depth_mm'], screened(first, p), retained(first), physics_step=0)
            first.update(state.as_dict(), command_pitch_deg=0., command_tilt_deg=0.,
                         command_offset_x_mm=0., command_offset_y_mm=0., reference_step=0)
            stream.add(first)
            maximum_steps = round(30./bench.dt) if stop_step is None else stop_step
            for step in range(1, maximum_steps+1):
                previous = stream.rows[-1]
                if guarded(previous):
                    reason = 'numerical_guard'; break
                if not within_budget(previous, p):
                    reason = 'operational_budget_exceeded'; break
                if state.done:
                    reason = state.termination_reason; break
                t = step*bench.dt
                command = state.command(t)
                if command['done']:
                    reason = state.termination_reason; break
                phase = command['phase']; depth = command['depth_mm']
                if phase == 'approach':
                    b = command['approach_fraction']
                    hand_p = start_p+b*(above_p-start_p)
                    hand_q = quat_slerp(start_q[0], above_q[0].clone(), b).unsqueeze(0)
                else:
                    hand_p, hand_q = bench.target(depth, pitch_deg=command['pitch_deg'])
                bench.tick(hand_p, hand_q)
                row = bench.observe(phase, depth)
                state.observe(t, row['depth_mm'], screened(row, p), retained(row), physics_step=step)
                row.update(state.as_dict(), command_pitch_deg=command['pitch_deg'],
                           command_tilt_deg=abs(command['pitch_deg']), command_offset_x_mm=0.,
                           command_offset_y_mm=0., reference_step=step)
                if abs(row['time_s']-t) > 1e-5:
                    raise RuntimeError('Mechanics reference clock discontinuity')
                stream.add(row)
            else:
                reason = state.termination_reason if state.done else 'replay_stop' if stop_step is not None else 'reference_time_limit'
            if reason == 'reference_complete':
                reason = 'completed'
    finally:
        stream.close()
    return stream.rows, reason, state.as_dict()


def recovery(bench, rows, policy, folder, basename, prefix_valid=True):
    with contact_recording(bench, folder/f'{basename}_contacts.jsonl.gz', initial=rows[-1]):
        samples, reason = recover(bench, rows[-1], policy, bench.protocol, folder/f'{basename}.csv')
    metrics = recovery_summary(samples, bench.protocol, bench.dt, prefix_valid=prefix_valid)
    metrics.update(mechanics_recovery_metrics(samples, bench.dt), reason=reason,
                   **recovery_termination(reason), termination_phase=samples[-1]['phase'])
    return metrics


def _material(bench, case):
    return bench.set_case_material(case['pair_static_friction'], case['pair_dynamic_friction'])


def run(args, app):
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=args.resume)
    with (directory/'.run.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('Another simulator is writing this mechanics study') from None
        return _run(args, app, directory)


def _run(args, app, directory):
    start = time.monotonic()
    plan = load_plan(args.case_plan)
    p = replace(load_protocol(args.config), seed=args.seed).validate()
    rate = resolve_physics_rate(args)
    sources = {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in SOURCE_PATHS}
    manifest = dict(study=SCHEMA, status='running', physics_hz=args.physics_hz,
                    simulation_options=dict(stock_buffers=args.stock_buffers, device=args.device),
                    physics_rate_selection=rate, seed=args.seed, case_plan=plan,
                    protocol=asdict(p), radial_clearance_mm=.2, sources=sources,
                    recovery_policy_definition='Original continuous straight vs replayed terminal XY-recenter-and-upright realign; both begin with 0.25 s attained-pose stop.',
                    attempts=[], skipped_cases=[], validation_status='provisional',
                    budget_signal='raw wrist force and torque norms',
                    acceptance_rule='One completed attempt per physical condition; failures and unknowns remain. All-prefix physical-state matching is required for policy comparison.')
    if args.resume:
        saved = json.loads((directory/'study.json').read_text())
        validate_resume(saved, manifest)
        manifest = saved
        for attempt in manifest['attempts']:
            if attempt['status'] == 'running':
                attempt['status'] = 'interrupted'
        manifest.pop('error', None)
        manifest['resume_count'] = manifest.get('resume_count', 0)+1
    else:
        for path in SOURCE_PATHS:
            target = directory/'source'/path.relative_to(ROOT)
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(path.read_bytes())
        save(directory/'case_plan.json', plan)
    previous_wall = manifest.get('elapsed_wall_seconds', 0.)
    manifest['status'] = 'running'
    save(directory/'study.json', manifest)
    bench = None
    try:
        bench = MechanicsBench(args, app)
        p = replace(p, effective_radial_clearance_mm=bench.scene_info['conservative_radial_clearance_mm']).validate()
        bench.protocol = p
        geometry = dict(socket_mesh_sha256=bench.scene_info['socket_mesh_sha256'],
                        effective_radial_clearance_mm=radial_clearance(p))
        if args.resume and manifest.get('geometry') != geometry:
            raise ValueError('Runtime geometry differs from frozen study')
        manifest['geometry'] = geometry
        manifest['effective_protocol'] = asdict(p)
        save(directory/'config.json', bench.cfg.to_dict())
        save(directory/'scene.json', bench.scene_info)
        # Warm up the fresh scene with baseline material before recorded resets.
        bench.set_case_material(.75, .75)
        bench.prepare(args.seed)
        count = 0
        for case in plan['cases']:
            completed = {a['case_id']: a for a in manifest['attempts'] if a['status'] == 'complete'}
            skipped = {a['case_id'] for a in manifest['skipped_cases']}
            if case['case_id'] in completed or case['case_id'] in skipped:
                continue
            if args.phase == 'controls' and case['tilt_amplitude_deg'] != 0.:
                continue
            if args.case_ids and case['case_id'] not in args.case_ids:
                continue
            if case['tilt_amplitude_deg'] != 0.:
                control = next((a for a in completed.values() if a['tilt_amplitude_deg'] == 0.
                                and control_key(a) == control_key(case)), None)
                if control is None:
                    continue  # Unexecuted control is a pending dependency, not a failed case.
                if not aligned_control_passed(control):
                    manifest['skipped_cases'].append(dict(case_id=case['case_id'],
                        reason='aligned_control_failed', control_case_id=control['case_id']))
                    save(directory/'study.json', manifest)
                    continue
            case_start = time.monotonic()
            material = _material(bench, case)
            folder = directory/case['trajectory_id']; index = 0
            while folder.exists():
                index += 1; folder = directory/f'{case["trajectory_id"]}_resume{index}'
            folder.mkdir()
            attempt = dict(case, folder=folder.name, status='running',
                           effective_radial_clearance_mm=radial_clearance(p), material=material, probes=[])
            manifest['attempts'].append(attempt)
            save(directory/'study.json', manifest)
            print(f'Mechanics: {case["case_id"]}', flush=True)
            rows, reason, state = reference(bench, case, args.seed, folder)
            attempt['metrics'] = {**mechanics_reference_metrics(rows, case, bench.dt), **state,
                                  **reference_quality(rows, p, reason)}
            save(directory/'study.json', manifest)
            attempt['final_retreat'] = recovery(bench, rows, 'straight', folder, 'final_retreat',
                                                prefix_valid=attempt['metrics']['numerically_valid'])
            save(directory/'study.json', manifest)
            probe = dict(policy='realign', replay_matched=None, replay_prefix_matched=None,
                         replay_prefix_equal=None, label_eligible=False, safe_recovery=None,
                         reason='reference_not_eligible')
            if (attempt['metrics']['numerically_valid'] and attempt['metrics']['reference_complete']
                    and attempt['metrics']['grasp_retained'] and attempt['metrics']['reference_within_budget']):
                _material(bench, case)
                prefix, prefix_reason, _ = reference(bench, case, args.seed, folder,
                    basename='terminal_realign_prefix', stop_step=rows[-1]['reference_step'])
                probe.update(compare_prefixes(rows, prefix, p))
                prefix_valid = all(screened(r, p) for r in prefix) and prefix_reason in ('completed', 'replay_stop')
                probe['replay_prefix_numerically_valid'] = prefix_valid
                if probe['replay_matched'] and probe['replay_prefix_matched'] and prefix_valid:
                    probe.update(recovery(bench, prefix, 'realign', folder, 'terminal_realign_recovery', prefix_valid=True))
                else:
                    probe['reason'] = 'invalid_replay_prefix' if not prefix_valid else 'replay_mismatch'
            attempt['probes'].append(probe)
            attempt['status'] = 'complete'
            attempt['elapsed_wall_seconds'] = time.monotonic()-case_start
            save(folder/'trajectory.json', attempt)
            manifest['elapsed_wall_seconds'] = previous_wall+time.monotonic()-start
            save(directory/'study.json', manifest)
            print(f'  reference={reason}; valid={attempt["metrics"]["numerically_valid"]}; '
                  f'straight={attempt["final_retreat"]["reason"]}; realign={probe["reason"]}; '
                  f'retreat_peak={attempt["final_retreat"].get("retreat_peak_wrist_force_n")}', flush=True)
            count += 1
            if args.max_new_attempts is not None and count >= args.max_new_attempts:
                break
        completed = {a['case_id'] for a in manifest['attempts'] if a['status'] == 'complete'}
        skipped = {a['case_id'] for a in manifest['skipped_cases']}
        if len(completed | skipped) == len(plan['cases']):
            invalid = any(not a.get('metrics', {}).get('numerically_valid') for a in manifest['attempts'] if a['status'] == 'complete')
            manifest['status'] = 'complete_with_exclusions' if invalid or skipped else 'complete'
        else:
            manifest['status'] = 'paused'
    except BaseException as error:
        manifest['status'] = 'paused' if isinstance(error, KeyboardInterrupt) else 'error'
        manifest['error'] = str(error)
        for attempt in manifest['attempts']:
            if attempt['status'] == 'running':
                attempt['status'] = 'interrupted'
        raise
    finally:
        manifest['elapsed_wall_seconds'] = previous_wall+time.monotonic()-start
        save(directory/'study.json', manifest)
        if bench is not None:
            bench.close()
    print(f'Mechanics study: {manifest["status"]}; {directory}', flush=True)
