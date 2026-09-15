"""Verified peg unloading in a new namespace, with unchanged nominal/fixed baselines."""
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import random
import shutil
import time
from isaaclab.utils.math import quat_slerp
from forge_backend import Bench, resolve_physics_rate
from forge_experiment import Stream, save
from productivity_control import execute as execute_previous, safety_reason
from research.forge_protocol import load_protocol
from research.phase2b import command_misalignment
from research.productivity_control import Design, Detector, blend, clock_for_depth, recent_signal, stalled_now
from research.forge_protocol import screened, retained
from research.productivity_unloading import Unloader, UnloadingDesign, POLICIES


def execute_verified(bench, case, frozen, unloading, path):
    policy="productivity"
    p=bench.protocol; design=Design(**frozen['design']); hz=round(1/bench.dt);dt=bench.dt
    first=bench.prepare(p.seed)
    initial=bench.env.fingertip_midpoint_pos.clone(); iq=bench.env.fingertip_midpoint_quat.clone()
    progress=-10.; clock=0.; segment=0; interventions=0; events=[]
    mode='insert'; mode_start=0.; anchor_p=anchor_q=None; anchor_depth=0.
    unloader=None;retract_command=0.
    join_p=join_q=None; join_depth=0.;success_dwell=0;reason='time_budget_exhausted'
    detector=Detector(policy,frozen['eta_threshold'],frozen['force_threshold_n'],design)
    stream=Stream(path)
    command=command_misalignment(case,progress)
    def target(depth):
        return bench.target(depth,x_mm=command['offset_x_mm'],y_mm=command['offset_y_mm'],
                            roll_deg=command['roll_deg'],pitch_deg=command['pitch_deg'])
    above,aq=target(-10.)
    def fields(row, step, hp, hq):
        row.update(step=step,segment=segment,intervention_count=interventions,nominal_clock_s=clock,
            nominal_offset_x_mm=command['offset_x_mm'],nominal_offset_y_mm=command['offset_y_mm'],
            nominal_roll_deg=command['roll_deg'],nominal_pitch_deg=command['pitch_deg'],ramp_progress_depth_mm=progress,
            check_performed=False,eta_valid=False,eta_raw=None,actual_progress_mm=None,command_progress_mm=None,
            trigger_consecutive=detector.count,soft_trigger=False,intervention_started=False,
            stalled_now=False,unload_command_mm=retract_command,
            unload_actual_mm=anchor_depth-row['depth_mm'] if unloader is not None else None,
            unloading_verified=False,unloading_failed=False,force_budget_exceeded=row['wrist_force_n']>p.force_budget_n,
            torque_budget_exceeded=row['wrist_torque_nm']>p.torque_budget_nm,
            numerically_valid=screened(row,p),grasp_retained=retained(row))
        for i in range(3):row[f'command_hand_position_{i}']=hp[0,i].item()
        for i in range(4):row[f'command_hand_quaternion_{i}']=hq[0,i].item()
        return row
    stream.add(fields(first,0,initial,iq))
    try:
        for step in range(1,round((2+design.insertion_budget_s)*hz)+1):
            if safety_reason(stream.rows[-1],p):reason=safety_reason(stream.rows[-1],p);break
            t=step*dt
            progress=max(progress,stream.rows[-1]['depth_mm'])
            command=command_misalignment(case,progress)
            if t<=2.+1e-9:
                phase='approach';depth=-10.;b=blend(t/2.)
                hp=initial+b*(above-initial);hq=quat_slerp(iq[0],aq[0].clone(),b).unsqueeze(0)
            elif mode=='intervene' and unloader.status=='running':
                phase,retract_command=unloader.target(t)
                hp=anchor_p.clone();hp[:,2]+=retract_command/1000.;hq=anchor_q
                depth=anchor_depth-retract_command
            else:
                if mode=='intervene':
                    last=stream.rows[-1]
                    if unloader.status!='verified':raise RuntimeError('Retry without verified actual unloading')
                    events[-1].update(retract_end_time_s=last['time_s'],retract_end_actual_depth_mm=last['depth_mm'],
                        actual_retraction_mm=anchor_depth-last['depth_mm'],verified_before_retry=True)
                    join_p=bench.env.fingertip_midpoint_pos.clone();join_q=bench.env.fingertip_midpoint_quat.clone()
                    join_depth=last['depth_mm'];clock=clock_for_depth(join_depth,case['insertion_duration_s'])
                    mode='rejoin';mode_start=t-dt;segment+=1;detector.reset()
                clock+=dt;depth=-10.+30.*blend(clock/case['insertion_duration_s'])
                hp,hq=target(depth);phase='insert' if clock<=case['insertion_duration_s']+1e-9 else 'hold'
                if mode=='rejoin':
                    b=blend((t-mode_start)/design.rejoin_s)
                    hp=join_p+b*(hp-join_p);hq=quat_slerp(join_q[0],hq[0].clone(),b).unsqueeze(0)
                    depth=join_depth+b*(depth-join_depth);phase='rejoin'
                    if t-mode_start>=design.rejoin_s-1e-9:
                        mode='insert';events[-1]['retry_started_s']=t
            bench.tick(hp,hq);r=fields(bench.observe(phase,depth),step,hp,hq)
            if abs(r['time_s']-t)>1e-5:raise RuntimeError('Control physics clock discontinuity')
            # Every tick is screened before success or any soft intervention can be accepted.
            unsafe=safety_reason(r,p)
            recovery_failure=False
            if mode=='intervene':
                status=unloader.observe(t,r['depth_mm'],not unsafe)
                event=events[-1]
                event.update(max_commanded_retract_mm=unloader.command_mm,max_actual_retract_mm=unloader.max_actual_mm,
                    actual_retract_at_last_sample_mm=anchor_depth-r['depth_mm'],unloading_status=status,
                    first_crossing_time_s=unloader.first_crossing_time,
                    time_to_0_5_mm_s=unloader.first_crossing_time-unloader.t0 if unloader.first_crossing_time is not None else None,
                    verification_time_s=unloader.verified_time,hold_rebounds=unloader.rebounds)
                if status in ('verified','safety_stop','budget_exhausted'):
                    event.update(after_time_s=t,after_depth_mm=r['depth_mm'],
                        after_wrist_force_n=r['wrist_force_n'],after_wrist_torque_nm=r['wrist_torque_nm'],
                        after_normal_load_n=r['normal_load_n'],after_contact_force_n=r['force_norm_n'])
                if status=='verified':r['unloading_verified']=True
                if status in ('safety_stop','budget_exhausted'):
                    r['unloading_failed']=True;recovery_failure=True
            r['stalled_now']=stalled_now(stream.rows+[r],hz,p)
            success_dwell=success_dwell+1 if phase in ('insert','hold') and r['depth_mm']>=p.success_depth_mm else 0
            if step%round(design.check_s*hz)==0:
                signal=recent_signal(stream.rows+[r],hz,case['ramp_onset_mm'],design)
                fired=detector.check(t,signal)
                r.update(check_performed=True,eta_valid=signal is not None,trigger_consecutive=detector.count,soft_trigger=fired)
                if signal:r.update({k:signal[k] for k in ('eta_raw','actual_progress_mm','command_progress_mm')})
                if fired and not unsafe and success_dwell<round(p.success_dwell_s*hz) and interventions<design.max_interventions:
                    interventions+=1;mode='intervene';mode_start=t
                    anchor_p=bench.env.fingertip_midpoint_pos.clone();anchor_q=bench.env.fingertip_midpoint_quat.clone()
                    anchor_depth=r['depth_mm']
                    unloader=Unloader(t,anchor_depth,design.stop_s,unloading);retract_command=0.
                    events.append(dict(intervention=interventions,trigger_time_s=t,trigger_depth_mm=anchor_depth,
                        trigger_command_depth_mm=depth,eta_raw=signal['eta_raw'],wrist_force_n=r['wrist_force_n'],
                        wrist_torque_nm=r['wrist_torque_nm'],normal_load_n=r['normal_load_n'],
                        unloading_status='running',max_commanded_retract_mm=0.,max_actual_retract_mm=0.,
                        previous_stall=any(x['stalled_now'] for x in stream.rows+[r])))
                    r.update(intervention_started=True,intervention_count=interventions,unload_command_mm=0.,unload_actual_mm=0.);detector.reset()
            stream.add(r)
            if unsafe:reason=unsafe;break
            if recovery_failure:reason='unloading_budget_exhausted';break
            if success_dwell>=round(p.success_dwell_s*hz):reason='insertion_success';break
    finally:stream.close()
    rows=stream.rows
    if events and events[-1]['unloading_status']=='running':
        events[-1].update(unloading_status='episode_time_budget_exhausted',after_time_s=rows[-1]['time_s'],
            after_depth_mm=rows[-1]['depth_mm'],after_wrist_force_n=rows[-1]['wrist_force_n'],
            after_wrist_torque_nm=rows[-1]['wrist_torque_nm'],after_normal_load_n=rows[-1]['normal_load_n'],
            after_contact_force_n=rows[-1]['force_norm_n'])
        if reason=='time_budget_exhausted':reason='unloading_episode_time_budget_exhausted'
    first_stall=next((r['time_s'] for r in rows if r['stalled_now']),None)
    maximum=lambda k:max(r[k] for r in rows)
    metrics=dict(policy='verified',insertion_success=reason=='insertion_success',stalled=first_stall is not None,
        stall_episodes=sum(r['stalled_now'] and (i==0 or not rows[i-1]['stalled_now']) for i,r in enumerate(rows)),
        stalled_time_s=sum(r['stalled_now'] for r in rows)*dt,first_stall_s=first_stall,
        max_wrist_force_n=maximum('wrist_force_n'),max_wrist_torque_nm=maximum('wrist_torque_nm'),
        max_normal_load_n=maximum('normal_load_n'),max_contact_force_n=maximum('force_norm_n'),
        max_penetration_mm=max(0.,-min(r['min_separation_mm'] for r in rows)),
        max_grasp_slip_mm=maximum('grasp_slip_mm'),max_grasp_slip_deg=maximum('grasp_slip_deg'),
        force_budget_exceeded=any(r['force_budget_exceeded'] for r in rows),
        torque_budget_exceeded=any(r['torque_budget_exceeded'] for r in rows),
        grasp_retained=all(r['grasp_retained'] for r in rows),numerically_valid=all(r['numerically_valid'] for r in rows),
        intervention_count=interventions,insertion_time_s=max(0.,rows[-1]['time_s']-2.),
        time_to_success_s=max(0.,rows[-1]['time_s']-2.) if reason=='insertion_success' else None,
        final_depth_mm=rows[-1]['depth_mm'],max_depth_mm=maximum('depth_mm'),
        active_insertion_time_s=sum(r['phase']=='insert' for r in rows)*dt,
        intervention_time_s=sum(r['phase'] in ('stop','unload_retract','unload_hold','rejoin') for r in rows)*dt,
        outcome=reason,events=events,rows=len(rows))
    return metrics


def run(args,app):
    root=Path(__file__).resolve().parents[1];directory=args.output_dir.resolve()
    plan=json.loads(args.plan.read_text());previous=root/plan['previous_experiment']
    prior=json.loads((previous/'experiment.json').read_text())
    frozen=json.loads(args.calibration.read_text());old_cal=json.loads((previous/'calibration.json').read_text())
    if frozen!=old_cal:raise ValueError('The previous detector calibration must remain exactly unchanged')
    if frozen['design']!=asdict(Design()):raise ValueError('The previous control design changed')
    unchanged=('research/productivity_control.py','simulation/productivity_control.py','simulation/forge_backend.py',
               'simulation/contact.py','research/forge_protocol.py','research/phase2_protocol.py','research/phase2b.py','experiments/forge_phase2.json')
    for f in unchanged:
        if hashlib.sha256((root/f).read_bytes()).hexdigest()!=prior['source_sha256'][f]:raise ValueError(f'Frozen source changed: {f}')
    case_plan_path=root/plan['case_plan'];case_plan=json.loads(case_plan_path.read_text())
    selections=[{k:c[k] for k in ('case_id','study','trajectory_id')} for c in prior['cases']]
    if case_plan['cases']!=selections:raise ValueError('Use the same frozen case selections')
    unloading=UnloadingDesign(**{k:plan[k] for k in asdict(UnloadingDesign())}).validate()
    p=load_protocol(args.config);args.seed=p.seed;rate=resolve_physics_rate(args)
    if json.loads(args.config.read_text())!=json.loads((previous/'source/experiments/forge_phase2.json').read_text()):
        raise ValueError('The protocol and safety limits must remain unchanged')
    directory.mkdir(parents=True,exist_ok=False)
    old={str(f.resolve()):(f.stat().st_size,f.stat().st_mtime_ns) for f in (root/'outputs').rglob('*')
         if f.is_file() and directory not in f.resolve().parents}
    cases=prior['cases'];schedule=[];rng=random.Random(plan['order_seed'])
    for repeat in range(args.repeats):
        ordering=cases.copy();rng.shuffle(ordering)
        for c in ordering:
            policies=list(POLICIES);rng.shuffle(policies)
            for policy in policies:schedule.append(dict(case_id=c['case_id'],repeat=repeat,policy=policy))
    manifest=dict(schema=plan['schema'],status='running',physics_hz=args.physics_hz,physics_rate_selection=rate,
        seed=p.seed,protocol=asdict(p),design=frozen['design'],unloading_design=asdict(unloading),cases=cases,
        schedule=schedule,runs=[],previous_experiment=plan['previous_experiment'],
        detector_unchanged=True,calibration_unchanged=True,baseline_implementation='unmodified simulation/productivity_control.execute',
        seed_note='Same preparation seed and strict prefix match as previous pilot; repeats test numerical reproducibility, not independent geometries.')
    save(directory/'calibration.json',frozen);save(directory/'experiment.json',manifest)
    sources=[Path(__file__),root/'simulation/launch_productivity_unloading.py',root/'research/productivity_unloading.py',
        root/'research/productivity_unloading_report.py',root/'research/productivity_control_report.py',
        *(root/f for f in unchanged),root/'simulation/forge_experiment.py',root/'productivity_unloading.sh',
        root/'tests/test_productivity_unloading.py',root/'docs/productivity_unloading.md',args.plan.resolve(),args.calibration.resolve(),case_plan_path]
    manifest['source_sha256']={str(f.relative_to(root)):hashlib.sha256(f.read_bytes()).hexdigest() for f in sources}
    for f in sources:
        dest=directory/'source'/f.relative_to(root);dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(f,dest)
    bench=None;start=time.monotonic()
    try:
        bench=Bench(args,app);bench.protocol=p
        save(directory/'scene.json',bench.scene_info);save(directory/'config.json',bench.cfg.to_dict())
        for filename in ('scene.json','config.json'):
            if json.loads((directory/filename).read_text())!=json.loads((previous/filename).read_text()):
                raise ValueError(f'Physics/scene drift compared with previous pilot: {filename}')
        manifest['exact_previous_scene_and_config_match']=True
        bench.prepare(p.seed);manifest['initial_solver_priming']=True
        for i,spec in enumerate(schedule):
            c=next(c for c in cases if c['case_id']==spec['case_id'])
            rid=f"{spec['case_id']}/repeat{spec['repeat']}/{spec['policy']}";rd=directory/rid;rd.mkdir(parents=True)
            print(f'[verified unloading] {i+1}/{len(schedule)} {rid}',flush=True)
            if spec['policy']=='verified':metrics=execute_verified(bench,c['case'],frozen,unloading,rd/'trajectory.csv')
            else:metrics=execute_previous(bench,c['case'],'productivity' if spec['policy']=='fixed' else 'nominal',frozen,rd/'trajectory.csv')
            events=metrics.pop('events');save(rd/'events.json',events)
            result=dict(**spec,run_id=rid,log=rid+'/trajectory.csv',**{k:v for k,v in metrics.items() if k!='policy'})
            manifest['runs'].append(result);save(rd/'metrics.json',result);save(directory/'experiment.json',manifest)
            print(f"[verified unloading] {metrics['outcome']}; depth={metrics['final_depth_mm']:.3f} stall={metrics['stalled']} interventions={metrics['intervention_count']} verified={sum(e.get('unloading_status')=='verified' for e in events)} wrist={metrics['max_wrist_force_n']:.3f} N load={metrics['max_normal_load_n']:.3f} N",flush=True)
        manifest['status']='complete'
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));raise
    finally:
        manifest['elapsed_seconds']=time.monotonic()-start
        manifest['preexisting_outputs_checked']=len(old)
        manifest['preexisting_outputs_changed']=[f for f,stat in old.items() if not Path(f).is_file() or (Path(f).stat().st_size,Path(f).stat().st_mtime_ns)!=stat]
        save(directory/'experiment.json',manifest)
        if bench is not None:bench.close()
    from research.productivity_unloading_report import analyze
    analyze(directory)
