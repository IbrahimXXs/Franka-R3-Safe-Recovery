"""Offline measurement and causal audit of the bounded de-wedging pilot."""
import argparse
from itertools import combinations
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from research.productivity_control_report import read_log
from research.productivity_unloading_report import event_features as axial_features,safe
from research.productivity_control import Design,Detector,recent_signal,stalled_now
from research.productivity_unloading import Unloader,UnloadingDesign
from research.productivity_dewedge import Dewedger,DewedgeDesign,POLICIES,pose_angles,qmul,qconj
from research.forge_protocol import ForgeProtocol,match
from research.future_stall import table


def angular_delta(a,b):return (np.asarray(b)-np.asarray(a)+180)%360-180

def event_features(rows,event,next_event,p,hz,target=.5):
    f=axial_features(rows,event,next_event,p,hz,target)
    start=round(event['trigger_time_s']*hz);end=round(f['recovery_end_time_s']*hz)
    a,b=rows[start],rows[end];nominal=event.get('nominal_peg_quaternion',[0,0,0,1])
    qa=[a[k] for k in ('qw','qx','qy','qz')];qb=[b[k] for k in ('qw','qx','qy','qz')]
    aa=pose_angles(qa,nominal);ba=pose_angles(qb,nominal);change=angular_delta(aa,ba)
    f.update(actual_dx_mm=b['tip_x_mm']-a['tip_x_mm'],actual_dy_mm=b['tip_y_mm']-a['tip_y_mm'],
        actual_droll_deg=float(change[0]),actual_dpitch_deg=float(change[1]),
        trigger_x_mm=a['tip_x_mm'],trigger_y_mm=a['tip_y_mm'],trigger_roll_deg=float(aa[0]),trigger_pitch_deg=float(aa[1]),
        after_x_mm=b['tip_x_mm'],after_y_mm=b['tip_y_mm'],after_roll_deg=float(ba[0]),after_pitch_deg=float(ba[1]),
        lateral_error_before_mm=float(np.hypot(a['tip_x_mm'],a['tip_y_mm'])),
        lateral_error_after_mm=float(np.hypot(b['tip_x_mm'],b['tip_y_mm'])),
        tilt_error_before_deg=float(np.linalg.norm(aa[:2])),tilt_error_after_deg=float(np.linalg.norm(ba[:2])),
        verification_time_s=event.get('verification_time_s'),retry_ready_time_s=event.get('retry_ready_time_s',event.get('verification_time_s')),
        ready_to_retry=event.get('unloading_status')=='verified',online_verified=event.get('verification_time_s') is not None)
    f['load_decreased_after_verified_unloading']=f['contact_load_decreased'] if f['online_verified'] else None
    f['time_to_verified_unloading_s']=f['verification_time_s']-event['trigger_time_s'] if f['online_verified'] else None
    f['commanded_relax_fraction']=event.get('max_relax_fraction',0.)
    for k in ('x_mm','y_mm','roll_deg','pitch_deg'):f['commanded_relax_'+k]=0.
    if 'neutral_peg_position' in event:
        frac=f['commanded_relax_fraction'];delta=np.array(event['neutral_peg_position'])-event['trigger_peg_position']
        command_row=next((r for r in reversed(rows[start+1:end+1]) if r.get('command_peg_quaternion_0') is not None),None)
        final_command_q=[command_row[f'command_peg_quaternion_{k}'] for k in range(4)] if command_row is not None else event['trigger_peg_quaternion']
        da=angular_delta(pose_angles(event['trigger_peg_quaternion'],nominal),pose_angles(final_command_q,nominal))
        f.update(commanded_relax_x_mm=float(delta[0]*1000*frac),commanded_relax_y_mm=float(delta[1]*1000*frac),
                 commanded_relax_roll_deg=float(da[0]),commanded_relax_pitch_deg=float(da[1]))
    # Distinguish motion during relaxation from subsequent axial retraction.
    relaxation=[r for r in rows[start+1:end+1] if r['phase']=='relax']
    if relaxation:
        last=relaxation[-1];angles=pose_angles([last[k] for k in ('qw','qx','qy','qz')],nominal)
        f.update(relax_end_actual_retreat_mm=a['depth_mm']-last['depth_mm'],relax_end_normal_load_n=last['normal_load_n'],
            relax_end_x_mm=last['tip_x_mm'],relax_end_y_mm=last['tip_y_mm'],relax_end_roll_deg=float(angles[0]),relax_end_pitch_deg=float(angles[1]))
    if f['verification_time_s'] is not None:
        idx=round(f['verification_time_s']*hz)
        for k in ('normal_load_n','wrist_force_n','wrist_torque_nm'):
            f['verified_mean_'+k]=float(np.mean([r[k] for r in rows[max(start+1,idx-round(.1*hz)):idx+1]]))
    boundary=round(next_event['trigger_time_s']*hz) if next_event else len(rows)-1
    tail=rows[round(f['retry_start_time_s']*hz):boundary+1] if f['retry_started'] else []
    dwell=0;success=False
    for row in tail:
        dwell=dwell+1 if row['phase'] in ('insert','hold') and row['depth_mm']>=p.success_depth_mm and safe(row,p) else 0
        success|=dwell>=round(p.success_dwell_s*hz)
    f['retry_success']=bool(success) if tail else None
    f['repeated_stall']=f['stall_after_retry']
    return f


def audit(directory,m,raw):
    p=ForgeProtocol(**m['protocol']);design=Design(**m['design']);u=UnloadingDesign(**m['unloading_design']);dd=DewedgeDesign(**m['dewedge_design']);hz=m['physics_hz']
    cal=json.loads((directory/'calibration.json').read_text());checks=actions=ticks=ready=0
    phases=('stop','relax','unload_retract','unload_hold','post_verify_hold')
    for run in m['runs']:
        rows=raw[run['run_id']];case=next(c['case'] for c in m['cases'] if c['case_id']==run['case_id'])
        ee=json.loads((directory/run['run_id']/'events.json').read_text())
        d=Detector('nominal' if run['policy']=='nominal' else 'productivity',cal['eta_threshold'],cal['force_threshold_n'],design)
        recovery=None;event=None;anchor=None
        for i,r in enumerate(rows):
            ticks+=1;t=i/hz
            assert abs(r['time_s']-t)<1e-5
            if i and r['segment']!=rows[i-1]['segment']:d.reset()
            assert r['stalled_now']==stalled_now(rows[:i+1],hz,p)
            if not safe(r,p):assert i==len(rows)-1 and not run['insertion_success']
            if r['phase'] in phases:
                assert recovery is not None
                if run['policy']=='dewedge':
                    phase,frac,command=recovery.target(t)
                    assert abs(frac-r['relax_fraction'])<1e-8
                    pp=np.array(event['trigger_peg_position'])+frac*(np.array(event['neutral_peg_position'])-event['trigger_peg_position'])
                    pp[2]+=command/1000
                    qs=np.array([event['trigger_peg_quaternion'],event['neutral_peg_quaternion']])
                    pq=np.roll(Slerp([0,1],Rotation.from_quat(np.roll(qs,-1,axis=1)))(frac).as_quat(),1)
                    hpq=qmul(pq,qconj(event['captured_grasp_quaternion']))
                    hp=pp-Rotation.from_quat(np.roll(hpq,-1)).apply(event['captured_grasp_position'])
                    np.testing.assert_allclose([r[f'command_peg_position_{k}'] for k in range(3)],pp,atol=2e-7,rtol=0)
                    loggedq=np.array([r[f'command_peg_quaternion_{k}'] for k in range(4)])
                    assert abs(np.dot(loggedq,pq))>1-1e-6
                    np.testing.assert_allclose([r[f'command_hand_position_{k}'] for k in range(3)],hp,atol=3e-7,rtol=0)
                    assert abs(np.dot([r[f'command_hand_quaternion_{k}'] for k in range(4)],hpq))>1-1e-6
                else:
                    phase,command=recovery.target(t)
                    assert run['policy']=='axial'
                    assert abs(r['command_hand_position_2']-anchor['hand_pose_2']-command/1000)<1e-7
                    for k in (0,1):assert r[f'command_hand_position_{k}']==anchor[f'hand_pose_{k}']
                    for k in range(4):assert r[f'command_hand_quaternion_{k}']==anchor[f'hand_pose_{k+3}']
                assert phase==r['phase'] and abs(command-r['unload_command_mm'])<1e-8
                assert command<=u.max_command_mm+1e-9 and abs(recovery.depth0-r['command_depth_mm']-command)<1e-8
                status=recovery.observe(t,r['depth_mm'],safe(r,p))
                assert r['unloading_verified']==(status=='verified')
                assert r['unloading_failed']==(status in ('safety_stop','budget_exhausted'))
                ready+=int(status=='verified')
                if status=='verified':
                    assert event['unloading_status']=='verified'
                    dwell=.25+(.25 if run['policy']=='dewedge' else 0.)
                    assert all(safe(x,p) and recovery.depth0-x['depth_mm']>=.5 for x in rows[i-round(dwell*hz):i+1])
            if r['phase']=='rejoin':assert recovery.status=='verified'
            if run['policy']=='dewedge' and r.get('retry_pose_latched') and r['phase'] in ('insert','hold'):
                pq=event['retry_peg_quaternion'];hpq=qmul(pq,qconj(event['retry_grasp_quaternion']))
                pp=np.array(event['retry_peg_position']);pp[2]-=(r['command_depth_mm']-event['retract_end_actual_depth_mm'])/1000
                hp=pp-Rotation.from_quat(np.roll(hpq,-1)).apply(event['retry_grasp_position'])
                np.testing.assert_allclose([r[f'command_hand_position_{k}'] for k in range(3)],hp,atol=3e-7,rtol=0)
                assert abs(np.dot([r[f'command_hand_quaternion_{k}'] for k in range(4)],hpq))>1-1e-6
            assert r['check_performed']==bool(i and i%round(design.check_s*hz)==0)
            if r['check_performed']:
                checks+=1;signal=recent_signal(rows[:i+1],hz,case['ramp_onset_mm'],design)
                assert r['eta_valid']==(signal is not None)
                if signal:assert abs(signal['eta_raw']-r['eta_raw'])<1e-9
                fired=d.check(t,signal);assert r['soft_trigger']==fired and r['trigger_consecutive']==d.count
            if r['intervention_started']:
                actions+=1;assert r['soft_trigger'] and safe(r,p) and r['intervention_count']<=design.max_interventions
                d.reset();anchor=r;event=ee[int(r['intervention_count'])-1]
                recovery=Dewedger(t,r['depth_mm'],design.stop_s,u,dd) if run['policy']=='dewedge' else Unloader(t,r['depth_mm'],design.stop_s,u)
        assert run['insertion_time_s']<=design.insertion_budget_s+1e-5
    assert not m['preexisting_outputs_changed']
    assert all(hashlib.sha256((directory/'source'/f).read_bytes()).hexdigest()==sha for f,sha in m['source_sha256'].items())
    previous=Path(__file__).resolve().parents[1]/m['previous_experiment']
    assert cal==json.loads((previous/'calibration.json').read_text())
    for name in ('scene.json','config.json'):assert json.loads((directory/name).read_text())==json.loads((previous/name).read_text())
    return dict(status='passed',runs=len(m['runs']),physics_rows=ticks,causal_checks=checks,interventions=actions,
        retry_ready_unloadings=ready,no_unverified_retry=True,command_cap_enforced=True,hard_safety_priority=True,
        unchanged_axial_baseline=True,peg_pivot_relaxation_verified=True,retry_pose_retained=True,
        calibration_unchanged=True,scene_config_unchanged=True,source_archives_verified=len(m['source_sha256']),
        preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=[])


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m=json.loads((directory/'experiment.json').read_text());p=ForgeProtocol(**m['protocol']);hz=m['physics_hz']
    raw={r['run_id']:read_log(directory/r['log']) for r in m['runs']};summaries=[];events=[]
    for r in m['runs']:
        ee=json.loads((directory/r['run_id']/'events.json').read_text());features=[]
        for i,e in enumerate(ee):
            f=event_features(raw[r['run_id']],e,ee[i+1] if i+1<len(ee) else None,p,hz)
            f.update(case_id=r['case_id'],repeat=r['repeat'],policy=r['policy'],run_id=r['run_id'])
            features.append(f);events.append(f)
        hard=r['outcome'] in ('operational_budget_exceeded','grasp_retention_limit','numerical_screen_failed')
        summaries.append(dict(**r,safety_stop=hard,actual_unloading_crossings=sum(e['safe_target_crossed'] for e in features),
            verified_unloadings=sum(e['online_verified'] for e in features),
            recovery_failures=sum(not e['ready_to_retry'] for e in features) if r['policy'] in ('axial','dewedge') else None,
            ready_unloadings=sum(e['ready_to_retry'] for e in features),retry_successes=sum(e['retry_success'] is True for e in features),retries=sum(e['retry_started'] for e in features),evaluable_retries=sum(e['retry_evaluable'] for e in features),
            stalled_retries=sum(e['stall_after_retry'] is True for e in features),
            max_commanded_retract_mm=max((e['commanded_retract_mm'] for e in features),default=0.),
            max_actual_retract_mm=max((e['max_actual_retract_mm'] for e in features),default=0.)))
    table(directory/'summary.csv',summaries);table(directory/'unloading_events.csv',events)
    pairs=[]
    for case in m['cases']:
        for repeat in sorted({r['repeat'] for r in summaries}):
            rr={r['policy']:r for r in summaries if r['case_id']==case['case_id'] and r['repeat']==repeat}
            for a,b in combinations(POLICIES,2):
                if a not in rr or b not in rr:continue
                al,bl=raw[rr[a]['run_id']],raw[rr[b]['run_id']]
                end=min(next((i for i,r in enumerate(log) if r['intervention_started']),len(log)-1) for log in (al,bl))
                matched,errors=match(al[end],bl[end],p)
                pairs.append(dict(case_id=case['case_id'],repeat=repeat,policy_a=a,policy_b=b,prefix_time_s=al[end]['time_s'],
                    prefix_matched=matched,prefix_position_mm=errors['position_mm'],prefix_orientation_deg=errors['orientation_deg'],
                    prefix_depth_rmse_mm=float(np.sqrt(np.mean([(x['depth_mm']-y['depth_mm'])**2 for x,y in zip(al[:end+1],bl[:end+1])]))),
                    success_delta=int(rr[b]['insertion_success'])-int(rr[a]['insertion_success']),
                    final_depth_delta_mm=rr[b]['final_depth_mm']-rr[a]['final_depth_mm']))
    table(directory/'paired_comparisons.csv',pairs)
    totals=[]
    for policy in POLICIES:
        rr=[r for r in summaries if r['policy']==policy];ee=[e for e in events if e['policy']==policy];vv=[e for e in ee if e['online_verified']]
        if not rr:continue
        totals.append(dict(policy=policy,runs=len(rr),successes=sum(r['insertion_success'] for r in rr),stalled_runs=sum(r['stalled'] for r in rr),
            safety_stops=sum(r['safety_stop'] for r in rr),recovery_budget_stops=sum(r['outcome'].startswith('unloading_') for r in rr),
            interventions=len(ee),safe_0_5_mm_crossings=sum(e['safe_target_crossed'] for e in ee),verified_unloadings=len(vv),
            ready_unloadings=sum(e['ready_to_retry'] for e in ee),retry_successes=sum(e['retry_success'] is True for e in ee),verified_load_decreases=sum(e['contact_load_decreased'] for e in vv),
            retries=sum(e['retry_started'] for e in ee),evaluable_retries=sum(e['retry_evaluable'] for e in ee),
            stalled_retries=sum(e['stall_after_retry'] is True for e in ee),
            mean_final_depth_mm=float(np.mean([r['final_depth_mm'] for r in rr])),
            max_wrist_force_n=max(r['max_wrist_force_n'] for r in rr),max_wrist_torque_nm=max(r['max_wrist_torque_nm'] for r in rr),
            max_normal_load_n=max(r['max_normal_load_n'] for r in rr)))
    table(directory/'policy_summary.csv',totals)
    validation=audit(directory,m,raw);(directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    colors=dict(nominal='#555555',axial='#d97706',dewedge='#007f85');cases=[c['case_id'] for c in m['cases']]
    for kind,key,label in (('depth','depth_mm','Actual depth (mm)'),('load','normal_load_n','Privileged contact load (N)')):
        fig,axes=plt.subplots(len(cases),1,figsize=(11,3*len(cases)),squeeze=False)
        for case,ax in zip(cases,axes[:,0]):
            for r in summaries:
                if r['case_id']!=case:continue
                log=raw[r['run_id']];color=colors[r['policy']]
                ax.plot([x['time_s']-2 for x in log],[x[key] for x in log],color=color,ls='-' if r['repeat']==0 else '--',alpha=.8,
                    label=r['policy'] if r['repeat']==0 else None)
                ax.scatter(log[-1]['time_s']-2,log[-1][key],color=color,s=25,marker='x' if r['safety_stop'] else 'o' if r['insertion_success'] else 's',zorder=5)
                for e in [e for e in events if e['run_id']==r['run_id']]:ax.axvline(e['trigger_time_s']-2,color=color,alpha=.2)
            if kind=='depth':ax.axhline(p.success_depth_mm,color='black',ls=':',label='success depth')
            ax.set(title=case,ylabel=label,xlabel='Time since insertion start (s)',xlim=(0,20));ax.grid(alpha=.2);ax.legend(ncol=4,fontsize=8)
        fig.tight_layout();fig.savefig(directory/f'{kind}_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(len(cases),2,figsize=(12,3*len(cases)),squeeze=False)
    for case,aa in zip(cases,axes):
        if not any(e['case_id']==case for e in events):
            for ax in aa:ax.text(.5,.5,'No interventions',transform=ax.transAxes,ha='center',color='gray')
        for e in [e for e in events if e['case_id']==case]:
            rr=[r for r in raw[e['run_id']] if e['trigger_time_s']-1e-7<=r['time_s']<=e['recovery_end_time_s']+1e-7]
            t=[r['time_s']-e['trigger_time_s'] for r in rr];color=colors[e['policy']]
            label=e['policy'] if e['repeat']==0 and e['intervention']==1 else None
            aa[0].plot(t,[e['trigger_depth_mm']-r['depth_mm'] for r in rr],color=color,alpha=.75,ls='-' if e['repeat']==0 else '--',label=label)
            aa[1].plot(t,[r['normal_load_n'] for r in rr],color=color,alpha=.75,ls='-' if e['repeat']==0 else '--',label=label)
        aa[0].axhline(.5,color='black',ls=':',label='actual target')
        for ax,y in zip(aa,('Actual peg retreat (mm)','Privileged contact load (N)')):
            ax.set(title=case,ylabel=y,xlabel='Time since intervention (s)');ax.grid(alpha=.2)
            if ax.get_legend_handles_labels()[0]:ax.legend(fontsize=8)
    fig.tight_layout();fig.savefig(directory/'unloading_motion_load.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(12,4))
    for j,policy in enumerate(('axial','dewedge')):
        ee=[e for e in events if e['policy']==policy]
        axes[0].scatter([e['commanded_retract_mm'] for e in ee],[e['actual_retract_mm'] for e in ee],color=colors[policy],label=policy)
        vv=[e for e in ee if e['online_verified']]
        if vv:axes[1].scatter([e['before_mean_normal_load_n'] for e in vv],[e['after_mean_normal_load_n'] for e in vv],color=colors[policy],label=policy)
        failed=[e for e in ee if not e['online_verified']]
        if failed:axes[1].scatter([e['before_mean_normal_load_n'] for e in failed],[e['after_mean_normal_load_n'] for e in failed],facecolors='none',edgecolors=colors[policy],label=policy+': unverified endpoint')
    axes[0].axhline(.5,color='black',ls=':');axes[0].set(xlabel='Maximum commanded retract (mm)',ylabel='Actual retreat at recovery end (mm)')
    axes[1].plot([0,20],[0,20],ls=':',color='black');axes[1].set(xlabel='Mean load before trigger (N)',ylabel='Mean load after recovery (N)',title='Recovery endpoints; hollow = failed verification')
    for ax in axes:ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(directory/'unloading_comparison.png',dpi=150);plt.close(fig)
    # Actual relaxation relative to neutral, all attempts including safety/budget failures.
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for policy in ('axial','dewedge'):
        ee=[e for e in events if e['policy']==policy]
        for ax,kind,unit in zip(axes,('lateral','tilt'),('mm','deg')):
            ax.scatter([e[f'{kind}_error_before_{unit}'] for e in ee],[e[f'{kind}_error_after_{unit}'] for e in ee],label=policy,color=colors[policy])
            limit=max([e[f'{kind}_error_before_{unit}'] for e in events]+[.1])*1.1
            ax.plot([0,limit],[0,limit],':',color='gray');ax.set(xlabel=f'Before {kind} error ({unit})',ylabel=f'After {kind} error ({unit})');ax.legend();ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(directory/'actual_relaxation.png',dpi=150);plt.close(fig)
    # First-attempt paired load reductions. Keep reset mismatches visible, no filtering.
    load_pairs=[]
    for c in cases:
        for repeat in sorted({r['repeat'] for r in summaries}):
            ee={e['policy']:e for e in events if e['case_id']==c and e['repeat']==repeat and e['intervention']==1}
            if 'axial' not in ee or 'dewedge' not in ee:continue
            a,b=ee['axial'],ee['dewedge'];prefix=next(x for x in pairs if x['case_id']==c and x['repeat']==repeat and x['policy_a']=='axial' and x['policy_b']=='dewedge')
            load_pairs.append(dict(case_id=c,repeat=repeat,prefix_matched=prefix['prefix_matched'],
                axial_load_drop_n=a['normal_load_drop_n'],dewedge_load_drop_n=b['normal_load_drop_n'],
                additional_dewedge_drop_n=b['normal_load_drop_n']-a['normal_load_drop_n'],
                axial_recovery_safe=not a['recovery_safety_stop'],dewedge_recovery_safe=not b['recovery_safety_stop'],
                axial_recovery_duration_s=a['recovery_end_time_s']-a['trigger_time_s'],dewedge_recovery_duration_s=b['recovery_end_time_s']-b['trigger_time_s']))
    table(directory/'paired_load_changes.csv',load_pairs)
    v=next(x for x in totals if x['policy']=='dewedge');axial=next(x for x in totals if x['policy']=='axial');nominal=next(x for x in totals if x['policy']=='nominal')
    lines=['# Productivity-triggered de-wedging pilot','',f"Completed {len(summaries)} runs: four frozen Stage4-style cases × two repeats × three policies, native {hz} Hz FORGE physics.",'',
        '| Policy | Success | Ever stalled | Safety stops | Recovery budget stops | Safe 0.5 mm crossings / attempts | 0.25 s verified | Ready after hold | Retry successes |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in totals:lines.append(f"| {r['policy']} | {r['successes']}/{r['runs']} | {r['stalled_runs']}/{r['runs']} | {r['safety_stops']} | {r['recovery_budget_stops']} | {r['safe_0_5_mm_crossings']}/{r['interventions']} | {r['verified_unloadings']} | {r['ready_unloadings']} | {r['retry_successes']}/{r['retries']} |")
    lines+=['','## Main questions','',
        f"**Does relaxation enable actual unloading?** De-wedging crossed 0.5 mm safely in {v['safe_0_5_mm_crossings']}/{v['interventions']} attempts, held it for 0.25 s in {v['verified_unloadings']}, and remained unloaded through the extra hold in {v['ready_unloadings']}. Axial-only verified {axial['verified_unloadings']}/{axial['interventions']}. All attempts, including safety and deadline failures, remain in the denominator.",
        '',f"**Does load decrease more?** Among first-attempt case/repeat pairs, the de-wedging endpoint load drop exceeded axial-only in {sum(x['additional_dewedge_drop_n']>0 for x in load_pairs)}/{len(load_pairs)} comparisons. Mean additional drop: {np.mean([x['additional_dewedge_drop_n'] for x in load_pairs]):.3f} N. Endpoint durations differ; safety-terminated endpoints and reset mismatches are flagged in paired_load_changes.csv. These descriptive changes do not isolate relaxation from stopping, retraction and elapsed time.",
        '',f"**Can insertion resume successfully?** De-wedging produced {v['retries']} active insertion retries, {v['retry_successes']} successful retries, and {v['stalled_retries']} stalls among {v['evaluable_retries']} retries with enough commanded motion to evaluate the original stall predicate. Overall success: {v['successes']}/{v['runs']}, compared with nominal {nominal['successes']}/{nominal['runs']} and axial-only {axial['successes']}/{axial['runs']}."]
    matched_load=[x for x in load_pairs if x['prefix_matched']]
    if matched_load:
        lines+=['',f"Among the {len(matched_load)} strictly matched axial/de-wedging first-attempt pairs, the endpoint load drop was larger with de-wedging in {sum(x['additional_dewedge_drop_n']>0 for x in matched_load)}/{len(matched_load)}. Mean additional drop within that matched subset: {np.mean([x['additional_dewedge_drop_n'] for x in matched_load]):.3f} N. The full six-pair result above retains the reset mismatches."]
    if not v['ready_unloadings']:lines+=['','No de-wedging attempt became ready to retry. This pilot therefore does not establish successful recovery or post-unloading insertion; fewer recorded stalls after early termination are not stall prevention.']
    elif v['successes']<=nominal['successes']:lines+=['','Achieved unloading and improved insertion success are separate outcomes. This pilot did not improve the success count over nominal insertion.']
    lines+=['','## Case outcomes','', '| Case / policy | Attempts | Verified | Actual retreat at recovery end (mm) | Mean load before → after (N) | Safety-stopped attempts |', '|---|---:|---:|---|---|---:|']
    for c in cases:
        for policy in ('axial','dewedge'):
            ee=[e for e in events if e['case_id']==c and e['policy']==policy]
            if not ee:continue
            lines.append(f"| {c} / {policy} | {len(ee)} | {sum(e['online_verified'] for e in ee)} | {min(e['actual_retract_mm'] for e in ee):.4f}–{max(e['actual_retract_mm'] for e in ee):.4f} | {np.mean([e['before_mean_normal_load_n'] for e in ee]):.3f} → {np.mean([e['after_mean_normal_load_n'] for e in ee]):.3f} | {sum(e['recovery_safety_stop'] for e in ee)} |")
    de=[e for e in events if e['policy']=='dewedge']
    if de:
        lines+=['', '### Achieved de-wedging motion', '',
            '| Case | Lateral error before → after (mm), mean | Tilt magnitude before → after (deg), mean | Load at relaxation end (N), mean | Command / actual retreat at recovery end (mm), mean |',
            '|---|---|---|---:|---|']
        for c in cases:
            ee=[e for e in de if e['case_id']==c]
            if not ee:continue
            mean=lambda k:float(np.mean([e[k] for e in ee if e.get(k) is not None]))
            lines.append(f"| {c} | {mean('lateral_error_before_mm'):.3f} → {mean('lateral_error_after_mm'):.3f} | {mean('tilt_error_before_deg'):.3f} → {mean('tilt_error_after_deg'):.3f} | {mean('relax_end_normal_load_n'):.3f} | {mean('commanded_retract_mm'):.3f} / {mean('actual_retract_mm'):.3f} |")
        lines+=['','Holding commanded depth during relaxation does not guarantee stationary actual depth. The per-event relaxation-end depth records motion released by changing contact preload, including possible additional insertion before axial retraction. Retry authorization always uses the full measured retreat from the original trigger depth.']
    if v['successes']>nominal['successes'] and v['ready_unloadings']:
        lines+=['','The outcomes support de-wedging followed by achieved-motion verification as a feasible bounded recovery on these cases. They justify broader controlled validation with the same safety gates; four geometries and two repeats do not establish reliability across unseen contact states.']
    lines+=['','## Frozen protocol and bounded recovery','',
        'The previous Detector and recent_signal are imported unchanged: raw eta < 0.4212659765112803 for two consecutive 0.1 s checks, 0.5 s history, at least 0.1 mm positive command progress, and the original contact/onset gate. No retuning or ML. Simulator normal load is logged for analysis only, not used for intervention or retry authorization.',
        '', 'Nominal insertion directly calls the original execute function. Axial-only directly calls the previous execute_verified function, including its original verification and retry behavior. Both baseline sources, the detector, calibration, scene, physics and safety protocol are checked against the previous experiment.',
        '', 'De-wedging stops at the measured hand pose for the original 0.25 s. Over 0.5 s, a smoothstep interpolation takes the captured actual peg x/y toward the hole center and relative roll/pitch toward zero, preserving relative yaw. Quaternion slerp rotates about the peg base using the grasp transform captured at the trigger. Commanded peg depth stays fixed during relaxation; hand Z can change due to the rotation lever arm. The authored peg-depth reference, not hand Z displacement, defines commanded axial retreat.',
        '', 'After relaxation, upward command follows the original 5 mm / 4 s smooth ramp. A safe measured depth decrease of at least 0.5 mm from trigger freezes the command. The target must persist for 0.25 s to record verification, then another 0.25 s to authorize retry. A rebound resets the continuous hold and resumes the command ramp without resetting depth, time or command budgets. Relaxation, retraction, verification and the extra hold all share the existing 5 s deadline after the initial stop. The total episode remains 20 s after the 2 s approach, with at most two interventions.',
        '', 'On readiness, the measured peg x/y, orientation and current grasp transform are latched. Rejoin lasts the original 0.5 s and insertion restarts from actual depth, retaining that measured pose rather than restoring the imposed offset/tilt ramp. No assumption is made that the peg actually reached neutral: the before/after pose logs quantify achieved relaxation. The nominal_* CSV fields retain the counterfactual frozen case ramp, while command_hand_* logs the applied target and retry_pose_latched identifies changed retry behavior.',
        '', 'Hard limits stay wrist force 20 N, wrist torque 1 Nm, penetration 0.126475 mm, grasp slip 0.1 mm / 0.5 deg. Every physics tick is screened before verification, retry, success or a soft trigger. Failed recovery ends the episode safely; no expanded budget or lateral search is attempted.',
        '', '## Measurements and limitations','',
        'summary.csv includes success, stalls, maximum wrench/load, safety outcomes, intervention/retry counts, insertion time and final depth. unloading_events.csv contains trigger pose/depth/productivity, commanded relaxation, actual x/y/roll/pitch changes, commanded and achieved retreat, safe crossing and verification times, before/after contact and wrist wrench, retry success, and repeated stall. Per-run trajectory.csv retains full pose, velocity, contact load, wrench, grasp and target histories; events.json captures grasp transforms and the retry pose.',
        '', 'Before/after means use 0.1 s windows ending at trigger/recovery endpoint; sample counts and durations are retained. Verified-time load means are separate from final recovery endpoint means. A historical 0.25 s verification may be followed by rebound or timeout before readiness; these are separate labels. Empty retry/stall fields mean no retry or insufficient exposure, not successful prevention. Retry success and stall labels end at the next intervention or episode termination.',
        '', f"Strict prefix matching passed {sum(x['prefix_matched'] for x in pairs)}/{len(pairs)} pair endpoints. Same cases, preparation seed, repeats, solver priming and randomized policy order are used. All reset mismatches remain in paired_comparisons.csv; no observations are discarded. Two repeated resets of four geometries are a bounded feasibility study, not independent population samples or a guarantee of recovery.",
        '', '## Integrity and reproducibility','',
        f"Audit passed: {validation['physics_rows']} physics rows, {validation['causal_checks']} causal detector checks, {validation['interventions']} interventions. The replay checks the relaxation pivot, command/depth bounds, achieved-motion dwell, safety priority and retained retry pose. {validation['source_archives_verified']} source snapshots verified. {validation['preexisting_outputs_checked']} pre-existing output files checked; changed: 0. Exact scene/configuration and calibration match the previous pilot.",
        '', 'Collection command:', '', '```bash','./productivity_dewedge.sh --headless --output-dir outputs/Contact-Productivity-Dewedge-v1','```',
        '', 'The launcher refuses existing directories. For another authorized collection, choose a new directory. Offline report regeneration: `python -m research.productivity_dewedge_report outputs/Contact-Productivity-Dewedge-v1`.',
        '', 'Plot conventions: solid/dashed curves are repeats 0/1; vertical lines mark triggers. In depth/load comparisons, endpoint circles indicate success, squares indicate budget termination, and crosses indicate hard safety stops. Overlapping repeats may appear as one curve.',
        '', '![Depth comparison](depth_comparison.png)', '', '![Load comparison](load_comparison.png)', '', '![Recovery](unloading_motion_load.png)', '', '![Command versus achievement](unloading_comparison.png)', '', '![Actual relaxation](actual_relaxation.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    return totals

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory.resolve()),indent=2))
