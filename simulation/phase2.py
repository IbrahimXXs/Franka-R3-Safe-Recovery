"""Randomized insertion references and independently replayed recovery probes."""
from dataclasses import asdict
from types import SimpleNamespace
from pathlib import Path
import csv
import hashlib
import json
import math
import sys

import torch
from isaaclab.utils.math import quat_apply, quat_mul, quat_slerp
from study import Workbench, smooth
from scene import HOLE_CENTER, START_GAP, PEG_TIP_IN_HAND

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from research.phase2_protocol import Protocol, POLICIES, sample_slot, insertion_metrics, replay_match, recovery_label


def save_json(path,data):
    temp=path.with_suffix(path.suffix+'.tmp')
    temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');temp.replace(path)


def write_csv(path,rows):
    if not rows:return
    with path.open('w',newline='') as stream:
        keys=list(dict.fromkeys(k for r in rows for k in r))
        writer=csv.DictWriter(stream,fieldnames=keys);writer.writeheader();writer.writerows(rows)


class Phase2Workbench(Workbench):
    def prepare_case(self,case):
        self.case=case
        self.steps=0
        self.preparing=True
        try:
            super().prepare(SimpleNamespace(offset_mm=case['offset_x_mm'],insertion_tilt_deg=case['pitch_deg']))
        finally:
            self.preparing=False

    def set_goal(self,tip,tilt_deg):
        # The base preparation routine calls this with a zero Y coordinate.
        tip=list(tip)
        if getattr(self,'preparing',False):tip[1]=HOLE_CENTER[1]+self.case['offset_y_mm']/1000
        self.set_angles(tip,self.case['roll_deg'],tilt_deg)

    def set_angles(self,tip,roll,pitch):
        r,p=math.radians(roll)/2,math.radians(pitch)/2
        qx=torch.tensor([[math.cos(r),math.sin(r),0.,0.]],device=self.sim.device)
        qy=torch.tensor([[math.cos(p),0.,math.sin(p),0.]],device=self.sim.device)
        down=torch.tensor([[0.,1.,0.,0.]],device=self.sim.device)
        self.set_tip_pose(tip,quat_mul(quat_mul(qy,qx),down))

    def set_tip_pose(self,tip,q):
        self.target[:,3:]=q
        self.target[:,:3]=torch.tensor([tip],device=self.sim.device)-quat_apply(q,self.hand_to_tip)

    def observe(self):
        row=super().observe()
        row.update({f'joint_velocity{i+1}_rad_s':v for i,v in enumerate(self.robot.data.joint_vel[0,self.arm_ids].tolist())})
        return row


def capture(bench,index,phase,command_depth,case):
    row=dict(time_s=index*bench.dt,phase=phase,command_depth_mm=command_depth,
             command_roll_deg=case['roll_deg'],command_pitch_deg=case['pitch_deg'],**bench.observe())
    if any(not math.isfinite(v) for v in row.values() if isinstance(v,(int,float))):
        raise RuntimeError('Non-finite Phase 2 sample')
    return row


def guard(row):
    return row['force_norm_n']>500 or row['min_separation_mm'] < -1.


def insertion(bench,case,protocol,path,stop_step=None):
    """Replay the same preparation and exact command prefix up to a recorded step."""
    bench.prepare_case(case)
    rows=[];checkpoints={};index=0;aborted=False;prefix_penetration=0.
    initial_z=HOLE_CENTER[2]+START_GAP
    sim_start=bench.sim.current_time
    with path.open('w',newline='') as stream:
        writer=None
        for phase,duration in [('baseline',.5),('insert',case['insertion_duration_s']),('hold',.5)]:
            n=round(duration/bench.dt)
            for k in range(n):
                z=initial_z if phase=='baseline' else initial_z-(START_GAP+.020)*(smooth((k+1)/n) if phase=='insert' else 1.)
                tip=(HOLE_CENTER[0]+case['offset_x_mm']/1000,HOLE_CENTER[1]+case['offset_y_mm']/1000,z)
                bench.set_angles(tip,case['roll_deg'],case['pitch_deg']);bench.tick();index+=1
                row=capture(bench,index,phase,(HOLE_CENTER[2]-z)*1000,case)
                row['simulation_time_s']=bench.sim.current_time-sim_start
                if abs(row['simulation_time_s']-row['time_s'])>.001:raise RuntimeError('Phase 2 clock mismatch')
                prefix_penetration=max(prefix_penetration,-row['min_separation_mm'])
                if writer is None:writer=csv.DictWriter(stream,fieldnames=list(row));writer.writeheader()
                writer.writerow(row);rows.append(row)
                for depth in protocol.checkpoints_mm:
                    if depth not in checkpoints and row['depth_mm']>=depth:
                        checkpoints[depth]=dict(step=index,state=row.copy(),prefix_max_penetration_mm=prefix_penetration,
                            prefix_max_force_n=max(r['force_norm_n'] for r in rows),
                            prefix_max_torque_nm=max(r['torque_norm_nm'] for r in rows),
                            prefix_max_normal_load_n=max(r['normal_load_n'] for r in rows))
                if guard(row):aborted=True
                if aborted or stop_step==index:return rows,checkpoints,aborted
    return rows,checkpoints,aborted


def probe(bench,case,protocol,checkpoint,policy,folder):
    name=f"d{checkpoint['depth_mm']:g}__{policy}"
    replay,_,aborted=insertion(bench,case,protocol,folder/f'{name}__prefix.csv',checkpoint['step'])
    start=replay[-1]
    matched,errors=replay_match(checkpoint['state'],start,protocol)
    prefix_metrics=insertion_metrics(replay,bench.dt,protocol,bench.args.clearance_mm,aborted)
    result=dict(policy=policy,depth_mm=checkpoint['depth_mm'],replay_matched=matched,replay_errors=errors,
        label_eligible=False,safe_recovery=None,numerically_valid=False,status='replay_mismatch')
    if aborted or not prefix_metrics['numerically_valid']:
        result['status']='invalid_replayed_prefix';return result
    if not matched:return result
    # Start from the attained tool pose. A stop/hold transient is part of recovery.
    tip0=[HOLE_CENTER[0]+start['tip_x_mm']/1000,HOLE_CENTER[1]+start['tip_y_mm']/1000,HOLE_CENTER[2]-start['depth_mm']/1000]
    q0=torch.tensor([[start[k] for k in ('qw','qx','qy','qz')]],device=bench.sim.device)
    down=torch.tensor([[0.,1.,0.,0.]],device=bench.sim.device)
    phases=[('stop',protocol.stop_duration_s)]
    if policy=='realign':phases.append(('realign',protocol.realign_duration_s))
    phases += [('retreat',protocol.retreat_duration_s),('clear_hold',protocol.clear_hold_s)]
    recovery_sim_start=bench.sim.current_time
    rows=[dict(start,time_s=0.,simulation_time_s=0.,phase='recovery_start')];clear_count=0;time_clear=None;termination=None;step=0
    if start['force_norm_n']>protocol.force_budget_n or start['torque_norm_nm']>protocol.torque_budget_nm:
        termination='operational_budget_exceeded_at_start'
    try:
        for phase,duration in phases:
            if termination:break
            n=round(duration/bench.dt)
            for k in range(n):
                blend=smooth((k+1)/n);tip=list(tip0);q=q0
                if phase=='realign':
                    tip[0]+= (HOLE_CENTER[0]-tip0[0])*blend
                    tip[1]+= (HOLE_CENTER[1]-tip0[1])*blend
                    q=quat_slerp(q0[0],down[0].clone(),blend).unsqueeze(0)
                elif phase in ('retreat','clear_hold'):
                    if policy=='realign':tip[0],tip[1],q=HOLE_CENTER[0],HOLE_CENTER[1],down
                    tip[2]=tip0[2]+(HOLE_CENTER[2]+START_GAP-tip0[2])*(blend if phase=='retreat' else 1.)
                bench.set_tip_pose(tip,q);bench.tick();step+=1
                r=capture(bench,step,phase,(HOLE_CENTER[2]-tip[2])*1000,case)
                r['simulation_time_s']=bench.sim.current_time-recovery_sim_start
                if abs(r['simulation_time_s']-r['time_s'])>.001:raise RuntimeError('Recovery clock mismatch')
                # Recovery may use the achieved quaternion, not the randomized target angles.
                for j,key in enumerate(('command_qw','command_qx','command_qy','command_qz')):r[key]=q[0,j].item()
                r.pop('command_roll_deg');r.pop('command_pitch_deg');rows.append(r)
                clear=r['lowest_peg_z_m']>=HOLE_CENTER[2]+.002 and r['normal_load_n']<.1
                clear_count=clear_count+1 if clear else 0
                if clear_count>=round(.2/bench.dt) and time_clear is None:time_clear=step*bench.dt
                if guard(r):termination='numerical_guard'
                elif r['force_norm_n']>protocol.force_budget_n or r['torque_norm_nm']>protocol.torque_budget_nm:
                    termination='operational_budget_exceeded'
                if termination:break
    finally:
        write_csv(folder/f'{name}__recovery.csv',rows)
    penetration=max(0.,-min(r['min_separation_mm'] for r in rows))
    numerical=termination!='numerical_guard' and penetration<=bench.args.clearance_mm*protocol.penetration_fraction
    result.update(numerically_valid=numerical,label_eligible=numerical,
        safe_recovery=(time_clear is not None and termination is None) if numerical else None,
        status='numerically_invalid' if not numerical else termination or ('cleared_within_limits' if time_clear is not None else 'not_cleared'),
        max_force=max(r['force_norm_n'] for r in rows),max_torque=max(r['torque_norm_nm'] for r in rows),
        max_normal_load=max(r['normal_load_n'] for r in rows),max_penetration=penetration,
        time_to_clear_s=time_clear,duration_s=step*bench.dt,
        resisting_work_j=sum(max(0.,-r['contact_power_w'])*bench.dt for r in rows[1:]),
        retreat_peak_resistance_n=max([0.]+[-r['fz'] for r in rows if r['phase']=='retreat']))
    return result


def run_phase2(args,app):
    raw=json.loads(args.phase2.read_text())
    for key in ('checkpoints_mm','offset_range_mm','tilt_range_deg','insertion_duration_s'):
        if key in raw:raw[key]=tuple(raw[key])
    protocol=Protocol(**raw).validate()
    args.force_budget=protocol.force_budget_n;args.torque_budget=protocol.torque_budget_nm
    directory=args.output_dir or args.log.parent
    directory.mkdir(parents=True,exist_ok=True)
    physics={k:getattr(args,k) for k in ('device','physics_hz','friction','clearance_mm','contact_offset_mm',
        'socket_segments','contact_stiffness','contact_damping','translation_stiffness','rotation_stiffness')}
    source_hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in
        [Path(__file__),ROOT/'research/phase2_protocol.py',ROOT/'simulation/study.py',ROOT/'simulation/scene.py',ROOT/'simulation/contact.py']}
    identity=dict(protocol=asdict(protocol),physics=physics,source_sha256=source_hashes)
    fingerprint=hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
    manifest_path=directory/'study.json'
    if manifest_path.exists():
        if not args.phase2_resume:raise FileExistsError('Phase 2 output exists; use a new directory or --phase2-resume')
        manifest=json.loads(manifest_path.read_text())
        if manifest.get('fingerprint')!=fingerprint:raise ValueError('Resume protocol, physics, or simulation source differs from the saved experiment')
    else:
        if args.phase2_resume:raise FileNotFoundError('Cannot resume: study.json is missing')
        manifest=dict(study='FR3-Phase2-v1',status='running',fingerprint=fingerprint,**identity,attempts=[],
            labels='Y_R_tested: 1=observed safe policy; 0=all tested policies failed; null=unknown',
            label_scope='Finite tested policies at replay-matched observable states; not a viability certificate.',
            branch_method='Independent prefix replay; no claim to restore hidden PhysX contact/solver history.',
            numerical_validity='No hard guard and max overlap <= clearance * penetration_fraction',
            hard_guard=dict(net_force_n=500.,penetration_mm=1.),
            )
    manifest['status']='running';save_json(manifest_path,manifest)
    bench=None
    try:
        bench=Phase2Workbench(args,app)
        manifest['payload_model']=bench.robot.payload_model
        manifest['dlss_mode']=bench.sim.carb_settings.get('/rtx/post/dlss/execMode');manifest['eco_mode']=True
        for slot in range(protocol.target_valid):
            previous=[r for r in manifest['attempts'] if r['slot']==slot and r['status']=='complete']
            if any(r['metrics']['numerically_valid'] for r in previous):continue
            for retry in range(len(previous),protocol.attempts_per_slot):
                case=sample_slot(protocol,slot,retry)
                folder=directory/case['trajectory_id'];version=0
                while folder.exists():version+=1;folder=directory/f"{case['trajectory_id']}_resume{version}"
                folder.mkdir()
                attempt=dict(**case,folder=folder.name,status='running',checkpoints=[])
                manifest['attempts'].append(attempt);save_json(manifest_path,manifest);save_json(folder/'trajectory.json',attempt)
                print(f"Phase 2 reference: {case['trajectory_id']} / {case['family']}",flush=True)
                rows,checkpoints,aborted=insertion(bench,case,protocol,folder/'insertion.csv')
                attempt['metrics']=insertion_metrics(rows,bench.dt,protocol,args.clearance_mm,aborted)
                attempt['status']='probing';save_json(manifest_path,manifest)
                for depth in protocol.checkpoints_mm:
                    reached=depth in checkpoints;cp=checkpoints.get(depth,{})
                    prefix_valid=(reached and cp['prefix_max_penetration_mm']<=args.clearance_mm*protocol.penetration_fraction
                                  and cp['prefix_max_force_n']<=500)
                    record=dict(depth_mm=depth,reached=reached,prefix_numerically_valid=prefix_valid,probes=[])
                    if reached:record.update(cp)
                    attempt['checkpoints'].append(record)
                    if prefix_valid:
                        for policy in POLICIES:
                            print(f"  recovery probe: {depth:g} mm / {policy}",flush=True)
                            result=probe(bench,case,protocol,record,policy,folder)
                            record['probes'].append(result)
                            print(f"    {result['status']}",flush=True)
                            save_json(manifest_path,manifest);save_json(folder/'trajectory.json',attempt)
                    label,reason=recovery_label(record['probes'],reached,prefix_valid)
                    record['Y_R_tested']=label;record['label_reason']=reason
                attempt['status']='complete';save_json(folder/'trajectory.json',attempt);save_json(manifest_path,manifest)
                from phase2_report import write_phase2_report
                write_phase2_report(directory,manifest)
                if attempt['metrics']['numerically_valid']:break
        accepted={r['slot'] for r in manifest['attempts'] if r['status']=='complete' and r['metrics']['numerically_valid']}
        manifest['valid_trajectories']=len(accepted)
        manifest['status']='complete' if len(accepted)==protocol.target_valid else 'target_not_met'
        save_json(manifest_path,manifest)
        from phase2_report import write_phase2_report
        write_phase2_report(directory,manifest)
        print(f"Phase 2 finished: {len(accepted)}/{protocol.target_valid} numerically valid trajectories. {directory.resolve()}",flush=True)
    except BaseException as error:
        manifest['status']='incomplete';manifest['error']=str(error);save_json(manifest_path,manifest);raise
    if not args.headless and not args.exit_after and app.is_running():
        bench.sim.pause()
        while app.is_running():app.update();bench.sim.carb_settings.set_bool('/rtx/ecoMode/enabled',True)
