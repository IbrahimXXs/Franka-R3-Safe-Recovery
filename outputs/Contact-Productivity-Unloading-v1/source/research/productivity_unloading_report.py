"""Offline analysis and causal-log audit for verified unloading; no Isaac imports."""
import argparse
from collections import Counter
from itertools import combinations
import hashlib
import json
from pathlib import Path
import numpy as np
from research.productivity_control_report import read_log
from research.productivity_control import Design,Detector,recent_signal,stalled_now
from research.productivity_unloading import Unloader,UnloadingDesign,POLICIES
from research.forge_protocol import ForgeProtocol,screened,retained,within_budget,match
from research.future_stall import table


def safe(row,p):return screened(row,p) and retained(row) and within_budget(row,p)


def event_features(rows,event,next_event,p,hz,target=.5):
    t0=event['trigger_time_s'];start=round(t0*hz);anchor=rows[start]
    boundary=round(next_event['trigger_time_s']*hz) if next_event else len(rows)-1
    join=next((i for i in range(start+1,boundary+1) if rows[i]['phase']=='rejoin'),None)
    end=join-1 if join is not None else boundary
    rr=rows[start:end+1];last=rows[end]
    actual=np.array([anchor['depth_mm']-r['depth_mm'] for r in rr])
    good=np.logical_and.accumulate([safe(r,p) for r in rr])
    crossing=next((r['time_s'] for r,a,g in zip(rr,actual,good) if a>=target and g),None)
    hold=[r for r in rr if r['time_s']>=last['time_s']-.25-1e-7]
    sustained=(len(hold)>1 and hold[-1]['time_s']-hold[0]['time_s']>=.25-1e-7
               and bool(good[-1]) and all(anchor['depth_mm']-r['depth_mm']>=target for r in hold))
    before=[r for r in rows[max(0,start-round(.1*hz)):start+1]]
    after=[r for r in rr[1:] if r['time_s']>=last['time_s']-.1-1e-7] or [last]
    commanded=max((r['command_hand_position_2']-anchor['hand_pose_2'])*1000 for r in rr[1:]) if len(rr)>1 else 0.
    retry=next((i for i in range(join or boundary,boundary+1) if join is not None and rows[i]['phase']=='insert'),None)
    tail=rows[retry:boundary+1] if retry is not None else []
    observed_stall=any(r['stalled_now'] for r in tail)
    n=round(p.stall_window_s*hz)
    evaluable=any(all(x['phase']=='insert' and x['segment']==tail[i]['segment'] for x in tail[i-n:i+1])
        and tail[i]['command_depth_mm']-tail[i-n]['command_depth_mm']>=p.stall_command_progress_mm for i in range(n,len(tail)))
    result=dict(intervention=event['intervention'],trigger_time_s=t0,trigger_depth_mm=anchor['depth_mm'],
        trigger_eta_raw=event['eta_raw'],commanded_retract_mm=max(0.,commanded),actual_retract_mm=float(actual[-1]),
        max_actual_retract_mm=float(actual.max()),safe_target_crossed=crossing is not None,
        time_to_0_5_mm_s=crossing-t0 if crossing is not None else None,
        sustained_0_5_mm_at_recovery_end=bool(sustained),online_verified=event.get('unloading_status')=='verified',
        recovery_status=event.get('unloading_status','fixed_completed' if join is not None else 'fixed_interrupted'),
        recovery_end_time_s=last['time_s'],recovery_end_depth_mm=last['depth_mm'],
        recovery_safety_stop=not bool(good[-1]),before_samples=len(before),after_samples=len(after),
        after_window_duration_s=after[-1]['time_s']-after[0]['time_s'],
        retry_started=retry is not None,retry_start_time_s=rows[retry]['time_s'] if retry is not None else None,
        retry_observed_s=tail[-1]['time_s']-tail[0]['time_s'] if tail else 0.,retry_evaluable=bool(evaluable),
        stall_observed_after_retry=observed_stall,stall_after_retry=observed_stall if evaluable else None,
        first_stall_before_trigger=bool(event['previous_stall']))
    for key in ('wrist_force_n','wrist_torque_nm','normal_load_n','force_norm_n','torque_norm_nm',
                'fx','fy','fz','taux','tauy','tauz'):
        result['trigger_'+key]=anchor[key];result['after_'+key]=last[key]
        result['before_mean_'+key]=float(np.mean([r[key] for r in before]))
        result['after_mean_'+key]=float(np.mean([r[key] for r in after]))
    result['normal_load_drop_n']=result['before_mean_normal_load_n']-result['after_mean_normal_load_n']
    result['contact_load_decreased']=result['normal_load_drop_n']>0
    result['load_decreased_after_verified_unloading']=result['contact_load_decreased'] if result['online_verified'] else None
    return result


def audit(directory,m,raw):
    p=ForgeProtocol(**m['protocol']);design=Design(**m['design']);unloading=UnloadingDesign(**m['unloading_design']);hz=m['physics_hz']
    cal=json.loads((directory/'calibration.json').read_text());checks=actions=ticks=verified=0
    for run in m['runs']:
        rows=raw[run['run_id']];case=next(c['case'] for c in m['cases'] if c['case_id']==run['case_id'])
        d=Detector('nominal' if run['policy']=='nominal' else 'productivity',cal['eta_threshold'],cal['force_threshold_n'],design)
        unloader=None;anchor=None
        for i,r in enumerate(rows):
            ticks+=1;t=i/hz
            assert abs(r['time_s']-t)<1e-5
            if i and r['segment']!=rows[i-1]['segment']:d.reset()
            assert r['stalled_now']==stalled_now(rows[:i+1],hz,p)
            if not safe(r,p):assert i==len(rows)-1 and not run['insertion_success']
            if run['policy']=='verified' and r['phase'] in ('stop','unload_retract','unload_hold'):
                phase,command=unloader.target(t)
                assert phase==r['phase'] and abs(command-r['unload_command_mm'])<1e-8
                assert command<=unloading.max_command_mm+1e-9
                assert abs(r['command_hand_position_2']-anchor['hand_pose_2']-command/1000)<1e-7
                for k in (0,1):assert r[f'command_hand_position_{k}']==anchor[f'hand_pose_{k}']
                for k in range(4):assert r[f'command_hand_quaternion_{k}']==anchor[f'hand_pose_{k+3}']
                status=unloader.observe(t,r['depth_mm'],safe(r,p))
                assert r['unloading_verified']==(status=='verified')
                assert r['unloading_failed']==(status in ('safety_stop','budget_exhausted'))
                verified+=int(status=='verified')
            if run['policy']=='verified' and r['phase']=='rejoin':assert unloader.status=='verified'
            if r['check_performed']:
                checks+=1;assert i%round(design.check_s*hz)==0
                signal=recent_signal(rows[:i+1],hz,case['ramp_onset_mm'],design)
                assert r['eta_valid']==(signal is not None)
                if signal:assert abs(signal['eta_raw']-r['eta_raw'])<1e-9
                fired=d.check(t,signal);assert r['soft_trigger']==fired and r['trigger_consecutive']==d.count
            if r['intervention_started']:
                actions+=1;assert r['soft_trigger'] and safe(r,p) and r['intervention_count']<=design.max_interventions
                d.reset()
                if run['policy']=='verified':unloader=Unloader(t,r['depth_mm'],design.stop_s,unloading);anchor=r
        assert run['insertion_time_s']<=design.insertion_budget_s+1e-5
    assert not m['preexisting_outputs_changed']
    root=Path(__file__).resolve().parents[1]
    assert all(hashlib.sha256((directory/'source'/f).read_bytes()).hexdigest()==sha for f,sha in m['source_sha256'].items())
    previous=root/m['previous_experiment']
    assert cal==json.loads((previous/'calibration.json').read_text())
    for name in ('scene.json','config.json'):assert json.loads((directory/name).read_text())==json.loads((previous/name).read_text())
    return dict(status='passed',runs=len(m['runs']),physics_rows=ticks,causal_checks=checks,interventions=actions,
        verified_unloadings=verified,no_unverified_retry=True,command_cap_enforced=True,hard_safety_priority=True,
        no_added_lateral_or_angular_retraction_command=True,calibration_unchanged=True,scene_config_unchanged=True,
        source_archives_verified=len(m['source_sha256']),preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=[])


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
            recovery_failures=sum(not e['online_verified'] for e in features) if r['policy']=='verified' else None,
            retries=sum(e['retry_started'] for e in features),evaluable_retries=sum(e['retry_evaluable'] for e in features),
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
            verified_load_decreases=sum(e['contact_load_decreased'] for e in vv),
            retries=sum(e['retry_started'] for e in ee),evaluable_retries=sum(e['retry_evaluable'] for e in ee),
            stalled_retries=sum(e['stall_after_retry'] is True for e in ee),
            mean_final_depth_mm=float(np.mean([r['final_depth_mm'] for r in rr])),
            max_wrist_force_n=max(r['max_wrist_force_n'] for r in rr),max_wrist_torque_nm=max(r['max_wrist_torque_nm'] for r in rr),
            max_normal_load_n=max(r['max_normal_load_n'] for r in rr)))
    table(directory/'policy_summary.csv',totals)
    validation=audit(directory,m,raw);(directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    colors=dict(nominal='#555555',fixed='#d97706',verified='#007f85');cases=[c['case_id'] for c in m['cases']]
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
    for j,policy in enumerate(('fixed','verified')):
        ee=[e for e in events if e['policy']==policy]
        axes[0].scatter([e['commanded_retract_mm'] for e in ee],[e['actual_retract_mm'] for e in ee],color=colors[policy],label=policy)
        vv=[e for e in ee if e['online_verified'] or policy=='fixed']
        axes[1].scatter([e['before_mean_normal_load_n'] for e in vv],[e['after_mean_normal_load_n'] for e in vv],color=colors[policy],label=policy)
    axes[0].axhline(.5,color='black',ls=':');axes[0].set(xlabel='Maximum commanded retract (mm)',ylabel='Actual retreat at recovery end (mm)')
    axes[1].plot([0,20],[0,20],ls=':',color='black');axes[1].set(xlabel='Mean load before trigger (N)',ylabel='Mean load after recovery (N)',title='Verified arm: verified events only')
    for ax in axes:ax.grid(alpha=.2);ax.legend()
    fig.tight_layout();fig.savefig(directory/'unloading_comparison.png',dpi=150);plt.close(fig)
    lines=['# Productivity-triggered verified unloading','',f"Status: **{m['status']}**. {len(summaries)} runs; four frozen Stage4 cases and two preparation repeats per policy. Native {hz} Hz FORGE physics.",
        '', '## Results','', '| Policy | Success | Ever stalled | Safety stops | Recovery budget stops | 0.5 mm safe crossings / attempts | Verified unloadings |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in totals:lines.append(f"| {r['policy']} | {r['successes']}/{r['runs']} | {r['stalled_runs']}/{r['runs']} | {r['safety_stops']} | {r['recovery_budget_stops']} | {r['safe_0_5_mm_crossings']}/{r['interventions']} | {r['verified_unloadings']} |")
    v=next((r for r in totals if r['policy']=='verified'),None)
    if v:
        lines+=['',f"**Can 0.5 mm actual unloading be achieved?** The verified arm safely crossed 0.5 mm in {v['safe_0_5_mm_crossings']}/{v['interventions']} attempts and completed the sustained verification hold in {v['verified_unloadings']}/{v['interventions']}. Safety stops and recovery deadlines remain in the denominator; zero attempts on a successful control are not unloading successes.",
            '',f"**Does contact load decrease?** {v['verified_load_decreases']}/{v['verified_unloadings']} verified unloadings had lower mean normal load afterward." if v['verified_unloadings'] else '\n**Does contact load decrease?** No verified unloading completed, so reduction after verified unloading is untested. Failed-attempt endpoint loads are reported separately.',
            '',f"**Does retry improve insertion?** The verified arm completed {v['successes']}/{v['runs']} insertions. Of {v['retries']} retries, {v['evaluable_retries']} had enough continued commanded motion to evaluate the existing stall predicate; {v['stalled_retries']} of those stalled. Compare successes and safety outcomes in the table. A failure that terminates before retry or stall confirmation cannot count as stall prevention."]
        nominal=next(r for r in totals if r['policy']=='nominal');fixed=next(r for r in totals if r['policy']=='fixed')
        if v['successes']<=nominal['successes'] and v['successes']<=fixed['successes']:
            lines+=['','This fixed pilot did not improve insertion success over nominal or the previous fixed retract. Any reduction in recorded stalls must be interpreted alongside early termination, achieved unloading, retry exposure and safety stops.']
    lines+=['','## Frozen protocol','',
        'The previous Detector, recent_signal, calibration JSON and baseline execute function are reused unchanged. Eta < 0.4212659765112803 for two consecutive 0.1 s checks; 0.5 s history, >=0.1 mm positive command progress, original contact/onset gate. No threshold retuning, ML or normal-load predictor.',
        '', 'Nominal and fixed call the previous controller directly. Verified changes only recovery. After the same 0.25 s stop at measured hand pose, upward command grows smoothly to at most 5 mm over 4 s of active retraction. The recovery deadline is 5 s after the stop and includes the verification hold. The target is a measured peg-depth decrease of >=0.5 mm from the original trigger depth. At target crossing the last command is held; 0.25 s continuously above target verifies unloading. Rebound resets the hold and resumes the ramp without a command jump, a new anchor or an extended deadline. Grasp, force/torque and penetration gates take priority, including on the crossing tick.',
        '', 'A verified hold permits the original 0.5 s rejoin and smoothstep retry from measured depth. The maximum-reached-depth offset/tilt ramp is retained, as before. No additional lateral or angular correction is selected; the unloading command changes world Z only. The original lateral/tilt path demand returns smoothly during rejoin. At most two interventions and the same 20 s insertion budget after a 2 s approach. Failure to verify within either recovery or episode budget ends the episode with no retry.',
        '', 'Hard limits are unchanged: wrist force 20 N; wrist torque 1 Nm; grasp slip 0.1 mm / 0.5 deg; penetration 0.126475 mm. Reset/preparation and solver priming are inherited unchanged. Baselines receive the same available episode time, ending early for the existing success dwell or safety limits.',
        '', '## Measurement and interpretation','',
        'unloading_events.csv contains trigger depth, maximum achieved command, actual/max retreat, first safe 0.5 mm crossing time, verification status, wrench/load before and after, load decrease, retry exposure and stall-after-retry. Per-tick trajectory.csv files retain full wrist/contact wrench, pose, velocities, grasp slip, commanded hand pose and detector fields. Before/after means use 0.1 s windows ending at trigger/recovery end; sample counts and after-window duration are reported. The verified after-window is in the hold; the fixed endpoint may still be moving. Short or unsafe failure endpoints are not verified-unloading outcomes. Load decrease is a descriptive strict decrease of these means, not a statistical significance claim.',
        '', 'The fixed arm is also scored offline for actual 0.5 mm crossing/sustained retreat, but its original policy does not enforce that criterion. Time to crossing is measured from trigger (including stop). A safe crossing alone is distinct from completing the verification hold. No commanded displacement is accepted as achieved peg motion.',
        '', 'Stall uses the unchanged 0.5 s, >=0.5 mm commanded, <0.1 mm actual predicate within contiguous insertion segments. stall_after_retry is empty when there is no retry or insufficient qualifying motion to evaluate it; such missing labels are not negatives. Retry windows end at the next intervention or episode termination. Summary outcomes retain all failed and safety-terminated runs.',
        '', f"Strict pair matching passed {sum(x['prefix_matched'] for x in pairs)}/{len(pairs)} common-prefix endpoints across nominal/fixed/verified comparisons. The same preparation seed and randomized within-case policy order are used; four geometries and repeated resets do not establish population-level performance. Mismatches remain visible in paired_comparisons.csv and limit causal attribution.",
        '', '## Integrity and use','',
        f"Audit passed for {validation['physics_rows']} physics rows and {validation['causal_checks']} detector checks, including upward-only command bounds, achieved-motion retry authorization and safety priority. Scene/configuration and calibration match the prior pilot. {validation['preexisting_outputs_checked']} pre-existing output files checked; changed: 0. Source snapshots and validation.json are retained.",
        '', '```bash', './productivity_unloading.sh --headless --output-dir outputs/Contact-Productivity-Unloading-v2','```',
        '', 'Offline regeneration: `python -m research.productivity_unloading_report outputs/Contact-Productivity-Unloading-v1`.',
        '', '![Depth](depth_comparison.png)', '', '![Contact load](load_comparison.png)', '',
        '![Recovery motion and load](unloading_motion_load.png)', '', '![Unloading comparison](unloading_comparison.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    return totals

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory.resolve()),indent=2))
