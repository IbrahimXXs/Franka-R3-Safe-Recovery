"""Controlled FORGE references, matched-prefix recovery probes and pilot reports."""
import csv
import json
import hashlib
import time
from dataclasses import asdict,replace
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import torch
from isaaclab.utils.math import quat_apply,quat_mul,quat_conjugate,quat_slerp
from research.phase2_protocol import insertion_metrics,recovery_label,sample_slot
from research.forge_protocol import load_protocol,pilot_cases,retained,screened,within_budget,clear,match,recovery_summary,RADIAL_CLEARANCE_MM
from forge_backend import Bench
from phase2_report import write_phase2_report


def save(path,data):
    tmp=path.with_suffix(path.suffix+'.tmp')
    tmp.write_text(json.dumps(data,indent=2,allow_nan=False,default=str)+'\n');tmp.replace(path)


def blend(u):
    u=min(1.,max(0.,u));return u*u*(3-2*u)


class Stream:
    def __init__(self,path):self.file=path.open('w');self.writer=None;self.rows=[]
    def add(self,row):
        if self.writer is None:
            self.writer=csv.DictWriter(self.file,fieldnames=list(row));self.writer.writeheader()
        self.writer.writerow(row);self.rows.append(row)
    def close(self):self.file.close()


def guarded(row):return row['force_norm_n']>500 or row['min_separation_mm'] < -1.


def reference(bench,case,seed,path,checkpoints,stop_step=None):
    p=bench.protocol;first=bench.prepare(seed)
    initial=bench.env.fingertip_midpoint_pos.clone();iq=bench.env.fingertip_midpoint_quat.clone()
    def target(d):return bench.target(d,case['offset_x_mm'],case['offset_y_mm'],case['roll_deg'],case['pitch_deg'])
    above,aq=target(-10.)
    cps={d:dict(depth_mm=d,reached=False,prefix_numerically_valid=False,probes=[]) for d in checkpoints}
    duration=case['insertion_duration_s'];count=round((3.+duration)/bench.dt)
    stream=Stream(path);aborted=False;prefix_valid=True
    maximum_f=maximum_t=maximum_load=maximum_pen=0.
    try:
        first.update(command_tilt_deg=0.,reference_step=0);stream.add(first)
        for step in range(1,(stop_step or count)+1):
            t=step*bench.dt
            if t<=2:
                phase='approach';depth=-10.;b=blend(t/2)
                hp=initial+b*(above-initial);hq=quat_slerp(iq[0],aq[0].clone(),b).unsqueeze(0)
            elif t<=2+duration:
                phase='insert';depth=-10+30*blend((t-2)/duration);hp,hq=target(depth)
            else:phase='hold';depth=20.;hp,hq=target(depth)
            bench.tick(hp,hq);r=bench.observe(phase,depth)
            r.update(command_tilt_deg=(case['roll_deg']**2+case['pitch_deg']**2)**.5,reference_step=step)
            stream.add(r)
            if abs(r['time_s']-t)>1e-5:raise RuntimeError('Reference physics clock discontinuity')
            prefix_valid &= screened(r,p)
            maximum_f=max(maximum_f,r['force_norm_n']);maximum_t=max(maximum_t,r['torque_norm_nm'])
            maximum_load=max(maximum_load,r['normal_load_n']);maximum_pen=max(maximum_pen,-r['min_separation_mm'])
            if phase in ('insert','hold'):
                for d,cp in cps.items():
                    if not cp['reached'] and r['depth_mm']>=d:
                        cp.update(reached=True,state=r.copy(),step=step,prefix_numerically_valid=bool(prefix_valid),
                            prefix_max_force_n=maximum_f,prefix_max_torque_nm=maximum_t,
                            prefix_max_normal_load_n=maximum_load,prefix_max_penetration_mm=maximum_pen)
            if guarded(r):aborted=True;break
    finally:stream.close()
    return stream.rows,list(cps.values()),aborted


def recover(bench,start,policy,p,path):
    e=bench.env;start_clock=e.last_update_timestamp
    hand_p=e.fingertip_midpoint_pos.clone();hand_q=e.fingertip_midpoint_quat.clone()
    peg_p=e.held_pos.clone();peg_q=e.held_quat.clone()
    gp=quat_apply(quat_conjugate(hand_q),peg_p-hand_p);gq=quat_mul(quat_conjugate(hand_q),peg_q)
    center=peg_p.clone();center[:,:2]=e.fixed_pos_obs_frame[:,:2];aligned_q=bench.peg_q.clone()
    retreat_start=p.stop_duration_s+(p.realign_duration_s if policy=='realign' else 0.)
    length=retreat_start+p.retreat_duration_s+p.clear_hold_s;rise=max(0.,start['depth_mm']+10.)/1000
    stream=Stream(path);reason='time_budget_exhausted'
    stream.add({**start,'phase':'stop','recovery_time_s':0.})
    try:
        for step in range(1,round(length/bench.dt)+1):
            prev=stream.rows[-1]
            if not screened(prev,p):reason='numerical_screen_failed';break
            if not within_budget(prev,p):reason='operational_budget_exceeded';break
            if not retained(prev):reason='grasp_retention_limit';break
            t=step*bench.dt
            if t<=p.stop_duration_s:
                hp,hq=hand_p,hand_q;phase='stop';depth=start['depth_mm']
            elif t<=retreat_start:
                b=blend((t-p.stop_duration_s)/p.realign_duration_s);pp=peg_p+b*(center-peg_p)
                pq=quat_slerp(peg_q[0],aligned_q[0].clone(),b).unsqueeze(0)
                hq=quat_mul(pq,quat_conjugate(gq));hp=pp-quat_apply(hq,gp);phase='realign';depth=start['depth_mm']
            else:
                b=blend((t-retreat_start)/p.retreat_duration_s)
                pp=(center if policy=='realign' else peg_p).clone();pp[:,2]+=b*rise
                pq=aligned_q if policy=='realign' else peg_q
                hq=quat_mul(pq,quat_conjugate(gq));hp=pp-quat_apply(hq,gp)
                phase='retreat' if t<=retreat_start+p.retreat_duration_s else 'clear_hold';depth=start['depth_mm']-b*rise*1000
            bench.tick(hp,hq);row=bench.observe(phase,depth)
            row.update(command_tilt_deg=0. if policy=='realign' else start['tilt_deg'],reference_step=start['reference_step'],recovery_time_s=t)
            if abs(e.last_update_timestamp-start_clock-t)>1e-5:raise RuntimeError('Recovery physics clock discontinuity')
            stream.add(row)
            if guarded(row):reason='numerical_guard';break
            dwell=round(.2/bench.dt)
            if len(stream.rows)>=dwell and all(clear(r) for r in stream.rows[-dwell:]):reason='cleared';break
    finally:stream.close()
    return stream.rows,reason


def characterize(rows,bench,p,aborted):
    active=[r for r in rows if r['phase'] in ('insert','hold')]
    metrics=insertion_metrics(active or rows,bench.dt,p,RADIAL_CLEARANCE_MM,aborted)
    metrics['numerically_valid']=not aborted and all(screened(r,p) for r in rows)
    metrics['outcome_trustworthy']=metrics['numerically_valid']
    metrics['force_budget_exceeded']=any(r['wrist_force_n']>p.force_budget_n for r in rows)
    metrics['torque_budget_exceeded']=any(r['wrist_torque_nm']>p.torque_budget_nm for r in rows)
    metrics['grasp_retained']=all(retained(r) for r in rows)
    metrics['insertion_success'] &= metrics['grasp_retained']
    metrics.update(max_grasp_slip_mm=max(r['grasp_slip_mm'] for r in rows),max_grasp_slip_deg=max(r['grasp_slip_deg'] for r in rows),
        max_wrist_force_n=max(r['wrist_force_n'] for r in rows),max_wrist_torque_nm=max(r['wrist_torque_nm'] for r in rows))
    return metrics


def report(directory,manifest):
    write_phase2_report(directory,manifest)
    from forge_report import write_profiles
    write_profiles(directory,manifest)
    with (directory/'report.md').open('a') as f:
        f.write('\n## Controlled FORGE interpretation\n\nPanda and stock SDF assets/contact settings; fixed nuisance parameters, '
                'zero dead zone, and full pose targets at the physics rate using the upstream torque law. Operational budgets apply '
                'to raw wrist force/torque norms. Contact torque is about the peg base. Wrist and contact signals are saved separately.\n\n'
                '**Pilot labels are provisional:** the overlap screen and replay check do not establish timestep convergence. '
                'No 100-trajectory collection is running.\n\n![Force profiles](force_profiles.png)\n\n![Force versus depth](force_vs_depth.png)\n')
    from visualization.view_study import build_dashboard
    build_dashboard(directory)


def run(args,app):
    wall_start=time.monotonic()
    directory=args.output_dir.resolve();directory.mkdir(parents=True);p=load_protocol(args.config)
    if args.seed is None:args.seed=p.seed
    else:p=replace(p,seed=args.seed)
    cases=pilot_cases(p) if args.mode=='pilot' else [dict(sample_slot(p,0),trajectory_id=f'centered_{i:02d}',slot=i) for i in range(args.repeats)]
    if args.slots:cases=[sample_slot(p,i) for i in args.slots]
    manifest={'study':'Forge-Controlled-Phase2-v1','status':'running','physics_hz':args.physics_hz,
        'physics':{'physics_hz':args.physics_hz,'radial_clearance_mm':RADIAL_CLEARANCE_MM},
        'mode':args.mode,'seed':args.seed,'attempts':[],'protocol':asdict(p),'pilot_candidate_count':len(cases),
        'requested_checkpoints_mm':args.checkpoints,'validation_status':'provisional','budget_signal':'raw wrist force and torque norms',
        'sources':{src.name:hashlib.sha256(src.read_bytes()).hexdigest() for src in
            (Path(__file__),Path(__file__).with_name('forge_backend.py'),Path(__file__).with_name('forge_calibration.py'),
             Path(__file__).with_name('contact.py'),Path(__file__).with_name('launch_forge_study.py'),
             Path(__file__).parents[1]/'research/forge_protocol.py',Path(__file__).parents[1]/'research/phase2_protocol.py')}}
    (directory/'source').mkdir()
    for name in manifest['sources']:
        src=Path(__file__).parents[1]/'research'/name if name in ('forge_protocol.py','phase2_protocol.py') else Path(__file__).with_name(name)
        (directory/'source'/name).write_bytes(src.read_bytes())
    save(directory/'study.json',manifest);bench=None
    try:
        bench=Bench(args,app)
        bench.protocol=p
        save(directory/'config.json',bench.cfg.to_dict());save(directory/'scene.json',bench.scene_info)
        if args.mode=='calibration':
            from forge_calibration import run_calibration
            run_calibration(bench,directory,args.seed)
            manifest['status']='calibration_complete'
            return
        # Prime the initial grasp/contact solve before any reference is recorded.
        # The first freshly-created scene can otherwise have a different peg
        # angular-velocity transient from subsequent identical seeded resets.
        bench.prepare(args.seed)
        for case in cases:
            folder=directory/case['trajectory_id'];folder.mkdir()
            attempt={**case,'folder':folder.name,'status':'running','checkpoints':[]}
            manifest['attempts'].append(attempt);save(directory/'study.json',manifest)
            print(f'Controlled FORGE: {case["trajectory_id"]} / {case["family"]}',flush=True)
            rows,cps,aborted=reference(bench,case,args.seed,folder/'insertion.csv',args.checkpoints)
            attempt['metrics']=characterize(rows,bench,p,aborted);attempt['checkpoints']=cps
            rr,reason=recover(bench,rows[-1],'straight',p,folder/'final_retreat.csv')
            attempt['final_retreat']={**recovery_summary(rr,p,bench.dt,prefix_valid=attempt['metrics']['numerically_valid']),'reason':reason}
            for cp in cps:
                if cp['reached'] and cp['prefix_numerically_valid']:
                    for policy in ('straight','realign'):
                        tag=f'd{cp["depth_mm"]:g}_{policy}'
                        prefix,_,bad=reference(bench,case,args.seed,folder/f'{tag}_prefix.csv',[],stop_step=cp['step'])
                        matched,errors=match(cp['state'],prefix[-1],p)
                        prefix_valid=not bad and all(screened(r,p) for r in prefix)
                        probe=dict(policy=policy,depth_mm=cp['depth_mm'],replay_matched=matched,replay_errors=errors,
                            label_eligible=False,safe_recovery=None,reason='replay_mismatch' if not matched else 'invalid_replay_prefix')
                        if matched and prefix_valid:
                            recovery,why=recover(bench,prefix[-1],policy,p,folder/f'{tag}_recovery.csv')
                            probe.update(recovery_summary(recovery,p,bench.dt,matched,prefix_valid),reason=why)
                        cp['probes'].append(probe)
                        print(f'  d={cp["depth_mm"]:g} {policy}: {probe["reason"]}; matched={matched}',flush=True)
                        save(directory/'study.json',manifest)
                label,reason=recovery_label(cp['probes'],cp['reached'],cp['prefix_numerically_valid'])
                cp.update(Y_R_tested=label,label_reason=reason)
            attempt['status']='complete';save(folder/'trajectory.json',attempt);save(directory/'study.json',manifest)
            print(f'  {attempt["metrics"]}; final retreat {attempt["final_retreat"]["reason"]}',flush=True)
        manifest['status']='pilot_complete';save(directory/'study.json',manifest);report(directory,manifest)
        print(f'Controlled FORGE results: {directory}',flush=True)
        if not args.headless and not args.exit_after and app.is_running():
            bench.sim.pause()
            while app.is_running():app.update();bench.sim.carb_settings.set_bool('/rtx/ecoMode/enabled',True)
    except BaseException as error:
        manifest['status']='error';manifest['error']=str(error);raise
    finally:
        manifest['elapsed_wall_seconds']=time.monotonic()-wall_start
        save(directory/'study.json',manifest)
        if bench is not None:bench.close()
