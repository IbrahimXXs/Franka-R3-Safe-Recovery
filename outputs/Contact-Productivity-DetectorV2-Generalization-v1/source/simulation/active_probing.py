"""Separate bounded active identification pilot, reusing unchanged FORGE physics."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import shutil
import time
import torch
from isaaclab.utils.math import quat_apply, quat_mul, quat_conjugate
from forge_backend import Bench, resolve_physics_rate
from forge_experiment import reference, characterize, save, Stream
from research.forge_protocol import load_protocol, match, screened, retained, within_budget
from research.active_probing import schedule, waveform, analyze


def observed(bench, phase, depth, t=0., fraction=0., displacement=0.):
    r = bench.observe(phase, depth)
    peg = bench.env._held_asset.data
    for i, axis in enumerate('xyz'):
        r[f'peg_world_{axis}_m'] = peg.root_pos_w[0,i].item()
    # Logged vx/vy/vz are COM velocities; also expose actual peg-origin velocity.
    v = peg.root_com_lin_vel_w[0]-torch.cross(peg.root_com_ang_vel_w[0],
                                            peg.root_com_pos_w[0]-peg.root_pos_w[0],dim=0)
    r.update({f'peg_base_v{axis}_m_s':v[i].item() for i,axis in enumerate('xyz')})
    p = bench.protocol
    r.update(probe_time_s=t, probe_fraction=fraction, probe_displacement_si=displacement,
             force_budget_exceeded=r['wrist_force_n']>p.force_budget_n,
             torque_budget_exceeded=r['wrist_torque_nm']>p.torque_budget_nm,
             numerically_valid=screened(r,p), grasp_retained=retained(r))
    return r


def safe(r, p):
    return screened(r,p) and retained(r) and within_budget(r,p)


def nominal(bench, state):
    return bench.target(state['command_depth_mm'], x_mm=state['command_offset_x_mm'],
                        y_mm=state['command_offset_y_mm'], roll_deg=state['command_roll_deg'],
                        pitch_deg=state['command_pitch_deg'])


def pause(bench, hp, hq, depth, path):
    stream = Stream(path)
    try:
        for step in range(round(.75/bench.dt)+1):
            if step: bench.tick(hp,hq)
            r=observed(bench,'prepare_hold',depth,step*bench.dt)
            stream.add(r)
            if not safe(r,bench.protocol): break
    finally: stream.close()
    return stream.rows


def probe_target(bench, hp, hq, axis, amount):
    pp=hp+quat_apply(hq,bench.grasp_p)
    pq=quat_mul(hq,bench.grasp_q)
    if axis in ('x','y'): pp=pp.clone(); pp[:,0 if axis=='x' else 1]+=amount
    elif axis in ('roll','pitch'):
        angle=torch.tensor(amount/2,device=bench.device)
        dq=torch.zeros_like(pq);dq[:,0]=torch.cos(angle)
        dq[:,1 if axis=='roll' else 2]=torch.sin(angle)
        pq=quat_mul(dq,pq)  # spatial world-axis rotation about the peg base
    hand_q=quat_mul(pq,quat_conjugate(bench.grasp_q))
    return pp-quat_apply(hand_q,bench.grasp_p),hand_q


def execute(bench, hp, hq, start, spec, path, eligible):
    stream=Stream(path);reason='completed'
    try:
        stream.add(observed(bench,'before',start['command_depth_mm']))
        if not eligible: return stream.rows,'preparation_rejected'
        for step in range(1,round(1.5/bench.dt)+1):
            if not safe(stream.rows[-1],bench.protocol): reason='safety_stop';break
            t=step*bench.dt;phase,fraction=waveform(t)
            displacement=spec['sign']*spec['amplitude_si']*fraction
            p,q=probe_target(bench,hp,hq,spec['axis'],displacement)
            bench.tick(p,q)
            row=observed(bench,phase,start['command_depth_mm'],t,fraction,displacement)
            stream.add(row)
            if not safe(row,bench.protocol): reason='safety_stop';break
    finally: stream.close()
    return stream.rows,reason


def run(args, app):
    root=Path(__file__).resolve().parents[1]
    directory=args.output_dir.resolve();directory.mkdir(parents=True,exist_ok=False)
    start=time.monotonic();p=load_protocol(args.config)
    args.seed=p.seed;rate=resolve_physics_rate(args)
    selections=json.loads(args.plan.read_text())
    specs=schedule(args.repeats)
    manifest=dict(schema='Active-Probing-Pilot-v1',status='running',physics_hz=args.physics_hz,
        physics_rate_selection=rate,repeats=args.repeats,seed=p.seed,protocol=asdict(p),schedule=specs,cases=[],
        design=dict(pause_s=.75,before_s=.25,outward_s=.25,dwell_s=.25,return_s=.25,after_s=.5,
                    analysis_tail_s=.125,translation_m=.0001,rotation_deg=.2,order_seed=20260915,
                    baseline_multiplier=3.,repeat_cosine=.8,repeat_relative_error=.5,off_axis_ratio=.5,
                    motion_floor_m=1e-6,rotation_floor_deg=.002,norm_length_m=.01))
    old={str(f.resolve()):(f.stat().st_size,f.stat().st_mtime_ns) for f in (root/'outputs').rglob('*')
         if f.is_file() and directory not in f.resolve().parents}
    source_dir=directory/'source';source_dir.mkdir()
    sources=[Path(__file__),root/'simulation/launch_active_probing.py',root/'research/active_probing.py',
             root/'simulation/forge_backend.py',root/'simulation/forge_experiment.py',root/'simulation/contact.py',
             root/'research/forge_protocol.py',root/'research/phase2_protocol.py',root/'research/phase2b.py',
             root/'research/contact_response.py',args.plan.resolve(),args.config.resolve()]
    manifest['source_hashes']={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    for f in sources:
        dst=source_dir/f.relative_to(root);dst.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dst)
    save(directory/'pilot.json',manifest)
    bench=None
    try:
        bench=Bench(args,app);bench.protocol=p
        save(directory/'scene.json',bench.scene_info)
        save(directory/'config.json',bench.cfg.to_dict())
        # Match the existing FORGE pipeline's initial solver/grasp priming.
        bench.prepare(p.seed)
        manifest['initial_solver_priming'] = True
        for selection in selections['cases']:
            source=root/selection['study'];archive=json.loads((source/'study.json').read_text())
            original=next(a for a in archive['attempts'] if a['trajectory_id']==selection['trajectory_id'])
            case={k:v for k,v in original.items() if k not in ('checkpoints','metrics','final_retreat','status','folder')}
            case_id=selection['case_id'];cd=directory/case_id;cd.mkdir()
            print(f'[active pilot] Preparing local reference: {case_id}',flush=True)
            rows,cps,aborted=reference(bench,case,p.seed,cd/'insertion.csv',[])
            metrics=characterize(rows,bench,p,aborted);state=rows[-1]
            hp,hq=nominal(bench,state)
            held=pause(bench,hp,hq,state['command_depth_mm'],cd/'nominal_hold.csv')
            anchor=held[-1]
            result=dict(case_id=case_id,case=case,source_study=selection['study'],source_trajectory=selection['trajectory_id'],
                source_study_sha256=hashlib.sha256((source/'study.json').read_bytes()).hexdigest(),
                archived_metrics=original['metrics'],metrics=metrics,state=state,paused_state=anchor,
                wrench_anchor_world_m=[anchor[f'peg_world_{a}_m'] for a in 'xyz'],probes=[])
            manifest['cases'].append(result);save(directory/'pilot.json',manifest)
            print(f"[active pilot] Local {case_id}: success={metrics['insertion_success']} stalled={metrics['stalled']} depth={metrics['max_depth']:.3f} mm normal={metrics['max_normal_load']:.3f} N",flush=True)
            reference_safe=all(safe(r,p) for r in rows+held) and not aborted
            for spec in specs:
                pd=cd/spec['probe_id'];pd.mkdir()
                print(f"[active pilot] {case_id} {spec['probe_id']} ({len(result['probes'])+1}/{len(specs)})",flush=True)
                prefix,_,aborted=reference(bench,case,p.seed,pd/'prefix.csv',[],stop_step=state['reference_step'])
                prefix_match,errors=match(state,prefix[-1],p)
                hp,hq=nominal(bench,state)
                held=pause(bench,hp,hq,prefix[-1]['command_depth_mm'],pd/'preparation.csv')
                paused_match,paused_errors=match(anchor,held[-1],p)
                prefix_safe=all(safe(r,p) for r in prefix+held) and not aborted
                eligible=bool(reference_safe and prefix_safe and prefix_match and paused_match)
                samples,reason=execute(bench,hp,hq,prefix[-1],spec,pd/'probe.csv',eligible)
                # Check full end state as well as the tail-mean return errors in analysis.
                return_match,return_errors=match(samples[round(.25/bench.dt)] if len(samples)>round(.25/bench.dt) else samples[0],samples[-1],p)
                record=dict(**spec,log=str((pd/'probe.csv').relative_to(directory)),reason=reason,
                    prefix_matched=prefix_match,paused_matched=paused_match,prefix_safe=prefix_safe,
                    prefix_errors=errors,paused_errors=paused_errors,return_matched=return_match,return_errors=return_errors,
                    completed=reason=='completed',eligible=bool(eligible and reason=='completed' and all(safe(r,p) for r in samples)),
                    numerically_valid=all(screened(r,p) for r in samples),grasp_retained=all(retained(r) for r in samples),
                    force_budget_exceeded=any(not within_budget(r,p) and r['force_budget_exceeded'] for r in samples),
                    torque_budget_exceeded=any(r['torque_budget_exceeded'] for r in samples),
                    max_normal_load_n=max(r['normal_load_n'] for r in samples),max_penetration_mm=max(0.,-min(r['min_separation_mm'] for r in samples)),
                    max_grasp_slip_mm=max(r['grasp_slip_mm'] for r in samples),max_grasp_slip_deg=max(r['grasp_slip_deg'] for r in samples))
                result['probes'].append(record);save(directory/'pilot.json',manifest)
                print(f"[active pilot] {reason}; eligible={record['eligible']} prefix={prefix_match} paused={paused_match}",flush=True)
        manifest['status']='complete'
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));raise
    finally:
        manifest['elapsed_seconds']=time.monotonic()-start
        manifest['preexisting_outputs_checked']=len(old)
        manifest['preexisting_outputs_changed']=[f for f,stat in old.items() if not Path(f).is_file() or (Path(f).stat().st_size,Path(f).stat().st_mtime_ns)!=stat]
        save(directory/'pilot.json',manifest)
        if bench is not None:bench.close()
    print('[active pilot] Analysis:',analyze(directory),flush=True)
