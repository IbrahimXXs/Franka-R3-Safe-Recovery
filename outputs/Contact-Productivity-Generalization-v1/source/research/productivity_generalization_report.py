"""Frozen benchmark evaluation; failures remain in denominators and no policies are fitted."""
import argparse
from collections import Counter,defaultdict
from itertools import combinations
import json
from pathlib import Path
import numpy as np
from research.productivity_generalization import POLICIES,FAMILIES,validate_plan,verify_lock,sha
from research.productivity_control import Design,Detector,blend,recent_signal,stalled_now
from research.productivity_control_report import read_log
from research.productivity_dewedge_report import event_features,audit as recovery_audit
from research.productivity_unloading_report import safe
from research.forge_protocol import ForgeProtocol,match
from research.future_stall import table


def wilson(success,total):
    if not total:return None,None
    z=1.959963984540054;rate=success/total;den=1+z*z/total
    center=(rate+z*z/(2*total))/den;half=z*np.sqrt(rate*(1-rate)/total+z*z/(4*total*total))/den
    return float(center-half),float(center+half)


def paired_effect(rows,seed=20260916,samples=10000):
    """Resample whole conditions within families; never resample time windows."""
    if not rows:return dict(conditions=0,success_difference=None,ci_low=None,ci_high=None,wins=0,losses=0)
    strata=defaultdict(list)
    for r in rows:strata[r['family']].append(r['success_delta'])
    rng=np.random.default_rng(seed);sums=np.zeros(samples)
    for values in strata.values():
        values=np.array(values);sums+=values[rng.integers(0,len(values),(samples,len(values)))].sum(axis=1)
    ci=np.quantile(sums/len(rows),[.025,.975])
    return dict(conditions=len(rows),success_difference=float(np.mean([r['success_delta'] for r in rows])),
        ci_low=float(ci[0]),ci_high=float(ci[1]),wins=sum(r['success_delta']>0 for r in rows),losses=sum(r['success_delta']<0 for r in rows))


def detector_history(rows,case,policy,cal,hz):
    """Frozen productivity detector evaluated on each policy's own observed history."""
    design=Design(**cal['design']);shadow=Detector('productivity',cal['eta_threshold'],cal['force_threshold_n'],design)
    records=[]
    for i,r in enumerate(rows):
        if i and r['segment']!=rows[i-1]['segment']:shadow.reset()
        if r['check_performed']:
            signal=recent_signal(rows[:i+1],hz,case['ramp_onset_mm'],design)
            fired=shadow.check(r['time_s'],signal)
            records.append(dict(time_s=r['time_s'],depth_mm=r['depth_mm'],phase=r['phase'],eta_valid=signal is not None,
                eta_raw=signal['eta_raw'] if signal else None,productivity_would_trigger=fired,
                productivity_trigger_onset=fired and shadow.count==design.consecutive_checks,
                productivity_consecutive=shadow.count,actual_policy_soft_trigger=r['soft_trigger'],
                actual_intervention_started=r['intervention_started'],wrist_force_n=r['wrist_force_n']))
        if r['intervention_started']:shadow.reset()
    return records


def force_audit(rows,run,case,cal,p,hz):
    design=Design(**cal['design']);detector=Detector('force',cal['eta_threshold'],cal['force_threshold_n'],design)
    anchor=None;checks=actions=0
    for i,r in enumerate(rows):
        assert abs(r['time_s']-i/hz)<1e-5
        if i and r['segment']!=rows[i-1]['segment']:detector.reset()
        assert r['stalled_now']==stalled_now(rows[:i+1],hz,p)
        if not safe(r,p):assert i==len(rows)-1 and not run['insertion_success']
        if r['phase'] in ('stop','retract'):
            assert anchor is not None
            elapsed=r['time_s']-anchor['time_s'];cmd=design.retract_mm*blend((elapsed-design.stop_s)/design.retract_s)
            assert r['phase']==('stop' if elapsed<=design.stop_s+1e-8 else 'retract')
            assert abs(anchor['depth_mm']-r['command_depth_mm']-cmd)<1e-7
            assert abs(r['command_hand_position_2']-anchor['hand_pose_2']-cmd/1000)<1e-7
            for k in (0,1):assert r[f'command_hand_position_{k}']==anchor[f'hand_pose_{k}']
            for k in range(4):assert r[f'command_hand_quaternion_{k}']==anchor[f'hand_pose_{k+3}']
            assert cmd<=design.retract_mm+1e-9 and elapsed<=design.stop_s+design.retract_s+1e-8
        if r['phase']=='rejoin':assert anchor is not None and r['time_s']-anchor['time_s']>=design.stop_s+design.retract_s-1e-8
        assert r['check_performed']==bool(i and i%round(design.check_s*hz)==0)
        if r['check_performed']:
            checks+=1;signal=recent_signal(rows[:i+1],hz,case['ramp_onset_mm'],design)
            assert r['eta_valid']==(signal is not None)
            if signal:assert abs(signal['eta_raw']-r['eta_raw'])<1e-9
            fired=detector.check(r['time_s'],signal)
            assert fired==r['soft_trigger'] and detector.count==r['trigger_consecutive']
        if r['intervention_started']:
            actions+=1;assert safe(r,p) and r['soft_trigger'] and r['intervention_count']<=design.max_interventions
            anchor=r;detector.reset()
    assert run['insertion_time_s']<=design.insertion_budget_s+1e-5
    return dict(physics_rows=len(rows),causal_checks=checks,interventions=actions,retry_ready_unloadings=0)


def failure_class(run,features,checks,last_phase):
    if run['insertion_success']:return 'success'
    hard=run['outcome'] in ('grasp_retention_limit','operational_budget_exceeded','numerical_screen_failed')
    if hard:
        when='during_recovery' if last_phase in ('stop','relax','retract','unload_retract','unload_hold','post_verify_hold') else 'during_retry' if features and features[-1]['rejoin_started'] else 'during_insertion'
        return run['outcome']+'_'+when
    if run['outcome'].startswith('unloading_'):return 'unloading_not_ready_within_frozen_budget'
    if any(e['stall_after_retry'] is True for e in features):return 'stalled_after_retry'
    if features:return 'retry_or_episode_budget_exhausted'
    if run['policy']=='nominal':return 'nominal_stall' if run['stalled'] else 'nominal_no_success_within_budget'
    if not any(c['eta_valid'] for c in checks):return 'no_eligible_detector_history'
    if not any(c['actual_policy_soft_trigger'] for c in checks):return 'no_policy_trigger_before_timeout'
    return 'episode_budget_exhausted'


def summarize_policy(rows,events):
    result=[]
    for policy in POLICIES:
        rr=[r for r in rows if r['policy']==policy];ee=[e for e in events if e['policy']==policy]
        if not rr:continue
        successes=sum(r['insertion_success'] for r in rr);lo,hi=wilson(successes,len(rr))
        result.append(dict(policy=policy,conditions=len(rr),successes=successes,success_rate=successes/len(rr),success_wilson_low=lo,success_wilson_high=hi,
            stalled_runs=sum(r['stalled'] for r in rr),safety_stops=sum(r['safety_stop'] for r in rr),
            productivity_triggered_runs=sum(r['productivity_trigger'] for r in rr),intervened_runs=sum(r['recovery_count']>0 for r in rr),
            recoveries=len(ee),verified_unloadings=sum(e['online_verified'] for e in ee),ready_unloadings=sum(e['ready_to_retry'] for e in ee),
            retries=sum(e['retry_started'] for e in ee),retry_successes=sum(e['retry_success'] is True for e in ee),
            evaluable_retries=sum(e['retry_evaluable'] for e in ee),repeated_stalls=sum(e['stall_after_retry'] is True for e in ee),
            mean_final_depth_mm=float(np.mean([r['final_depth_mm'] for r in rr])),mean_insertion_time_s=float(np.mean([r['insertion_time_s'] for r in rr])),
            mean_success_time_s=float(np.mean([r['time_to_success_s'] for r in rr if r['insertion_success']])) if successes else None,
            max_wrist_force_n=max(r['max_wrist_force_n'] for r in rr),max_wrist_torque_nm=max(r['max_wrist_torque_nm'] for r in rr),max_normal_load_n=max(r['max_normal_load_n'] for r in rr)))
    return result


COLORS=dict(nominal='#555555',force='#b45309',axial='#8464b2',dewedge='#008c91')

def condition_plot(path,condition,runs,raw,events,checks,plt,p,eta):
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
    for run in runs:
        policy=run['policy'];rows=raw[run['run_id']];color=COLORS[policy];times=[r['time_s']-2 for r in rows]
        for ax,key in zip(axes[:2],('depth_mm','normal_load_n')):
            ax.plot(times,[r[key] for r in rows],color=color,label=policy,lw=1)
            ax.scatter(times[-1],rows[-1][key],c=color,s=22,marker='o' if run['insertion_success'] else 'x' if not safe(rows[-1],p) else 's')
        cc=[r for r in checks if r['run_id']==run['run_id']]
        axes[2].plot([r['time_s']-2 for r in cc],[r['eta_raw'] if r['eta_valid'] else np.nan for r in cc],color=color,label=policy)
        for e in [e for e in events if e['run_id']==run['run_id']]:
            for ax in axes:ax.axvline(e['trigger_time_s']-2,color=color,alpha=.15)
    axes[0].axhline(p.success_depth_mm,ls=':',color='black');axes[2].axhline(eta,ls=':',color='black')
    for ax,label in zip(axes,('Actual depth (mm)','Privileged contact load (N)','Recent raw productivity')):
        ax.set_ylabel(label);ax.grid(alpha=.2);ax.legend(ncol=4,fontsize=8)
    axes[2].set(xlabel='Time since insertion start (s)',xlim=(0,20))
    axes[0].set_title(condition['case_id']+f"; onset {condition['ramp_onset_mm']:g} mm")
    fig.tight_layout();fig.savefig(path,dpi=120);plt.close(fig)


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory).resolve();root=Path(__file__).resolve().parents[1]
    m=json.loads((directory/'experiment.json').read_text());plan=json.loads((directory/'frozen_plan.json').read_text());verify_lock(root,plan)
    assert m['status']=='complete' and len(m['runs'])==len(plan['schedule'])==192
    assert [{k:r[k] for k in ('case_id','repeat','policy','policy_order')} for r in m['runs']]==plan['schedule']
    assert sha(directory/'frozen_plan.json')==m['frozen_plan_sha256']
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz'];cal=json.loads((directory/'calibration.json').read_text())
    summaries=[];all_events=[];pairs=[];all_checks=[];condition_results=[]
    audit_counts=dict(physics_rows=0,causal_checks=0,interventions=0,retry_ready_unloadings=0)
    images=directory/'conditions';images.mkdir(exist_ok=True)
    for condition in plan['cases']:
        runs=[r for r in m['runs'] if r['case_id']==condition['case_id']]
        assert {r['policy'] for r in runs}==set(POLICIES) and len(runs)==4
        raw={r['run_id']:read_log(directory/r['log']) for r in runs}
        nonforce=dict(m,runs=[r for r in runs if r['policy']!='force'])
        aa=recovery_audit(directory,nonforce,raw)
        for k in audit_counts:audit_counts[k]+=aa[k]
        group_summary=[]
        for run in runs:
            rows=raw[run['run_id']]
            if run['policy']=='force':
                aa=force_audit(rows,run,condition['case'],cal,p,hz)
                for k in audit_counts:audit_counts[k]+=aa[k]
            # Validate reported maxima and terminal safety independently of controller metrics.
            for metric,column in (('max_wrist_force_n','wrist_force_n'),('max_wrist_torque_nm','wrist_torque_nm'),('max_normal_load_n','normal_load_n')):
                assert abs(run[metric]-max(r[column] for r in rows))<1e-8
            assert abs(run['final_depth_mm']-rows[-1]['depth_mm'])<1e-8
            assert run['stalled']==any(r['stalled_now'] for r in rows)
            if run['insertion_success']:
                tail=rows[-round(p.success_dwell_s*hz):]
                assert all(r['depth_mm']>=p.success_depth_mm and safe(r,p) and r['phase'] in ('insert','hold') for r in tail)
            checks=detector_history(rows,condition['case'],run['policy'],cal,hz)
            for c in checks:c.update(case_id=run['case_id'],run_id=run['run_id'],policy=run['policy'])
            all_checks.extend(checks)
            ee=json.loads((directory/run['run_id']/'events.json').read_text());features=[]
            for i,e in enumerate(ee):
                f=event_features(rows,e,ee[i+1] if i+1<len(ee) else None,p,hz)
                f.update(case_id=run['case_id'],run_id=run['run_id'],policy=run['policy'],family=run['family'],severity=run['severity'],ramp_onset_mm=run['ramp_onset_mm'],verification_enforced=run['policy'] in ('axial','dewedge'))
                features.append(f);all_events.append(f)
            trigger=next((c['time_s'] for c in checks if c['productivity_would_trigger']),None)
            summary=dict(**run,safety_stop=not safe(rows[-1],p),recovery_count=len(features),
                productivity_trigger=trigger is not None,first_productivity_trigger_s=trigger,
                productivity_trigger_onsets=sum(c['productivity_trigger_onset'] for c in checks),
                actual_productivity_recoveries=len(features) if run['policy'] in ('axial','dewedge') else 0,
                eligible_detector_checks=sum(c['eta_valid'] for c in checks),verified_unloadings=sum(e['online_verified'] for e in features),
                ready_unloadings=sum(e['ready_to_retry'] for e in features),retry_successes=sum(e['retry_success'] is True for e in features),
                retries=sum(e['retry_started'] for e in features),repeated_stalls=sum(e['stall_after_retry'] is True for e in features),
                failure_class=failure_class(run,features,checks,rows[-1]['phase']),terminal_phase=rows[-1]['phase'],
                condition_plot='conditions/'+run['case_id']+'.png',
                **{k:v for k,v in condition['case'].items() if k.startswith('final_')})
            group_summary.append(summary);summaries.append(summary)
        by={r['policy']:r for r in group_summary}
        for a,b in combinations(POLICIES,2):
            al,bl=raw[by[a]['run_id']],raw[by[b]['run_id']]
            end=min(next((i for i,r in enumerate(log) if r['intervention_started']),len(log)-1) for log in (al,bl))
            matched,errors=match(al[end],bl[end],p)
            pairs.append(dict(case_id=condition['case_id'],family=condition['family'],severity=condition['severity'],policy_a=a,policy_b=b,prefix_matched=matched,
                prefix_time_s=al[end]['time_s'],prefix_position_mm=errors['position_mm'],prefix_orientation_deg=errors['orientation_deg'],
                prefix_depth_rmse_mm=float(np.sqrt(np.mean([(x['depth_mm']-y['depth_mm'])**2 for x,y in zip(al[:end+1],bl[:end+1])]))),
                success_a=by[a]['insertion_success'],success_b=by[b]['insertion_success'],success_delta=int(by[b]['insertion_success'])-int(by[a]['insertion_success'])))
        cr={k:v for k,v in condition.items() if k!='case'};cr.update({k:v for k,v in condition['case'].items() if k.startswith('final_')})
        for policy,r in by.items():
            for k in ('insertion_success','outcome','failure_class','final_depth_mm','verified_unloadings','retry_successes','safety_stop'):cr[policy+'_'+k]=r[k]
        condition_results.append(cr)
        condition_plot(images/(condition['case_id']+'.png'),condition,runs,raw,all_events,all_checks,plt,p,cal['eta_threshold'])
    table(directory/'summary.csv',summaries);table(directory/'per_condition_results.csv',condition_results)
    table(directory/'recovery_events.csv',all_events);table(directory/'detector_checks.csv',all_checks);table(directory/'paired_comparisons.csv',pairs)
    totals=summarize_policy(summaries,all_events);table(directory/'policy_summary.csv',totals)
    effects=[]
    for baseline in ('nominal','force','axial'):
        pp=[r for r in pairs if r['policy_a']==baseline and r['policy_b']=='dewedge']
        for scope,rr in (('all_conditions',pp),('strict_prefix_matched',[r for r in pp if r['prefix_matched']])):
            effects.append(dict(baseline=baseline,scope=scope,**paired_effect(rr,plan['analysis']['paired_bootstrap_seed'],plan['analysis']['paired_bootstrap_samples'])))
    table(directory/'paired_policy_effects.csv',effects)
    strata=[]
    for dimension in ('family','severity','ramp_onset_mm'):
        for value in sorted({r[dimension] for r in summaries}):
            rr=[r for r in summaries if r[dimension]==value];ids={r['run_id'] for r in rr};ee=[e for e in all_events if e['run_id'] in ids]
            strata.extend(dict(dimension=dimension,value=value,**r) for r in summarize_policy(rr,ee))
    table(directory/'stratified_results.csv',strata)
    failures=[r for r in summaries if not r['insertion_success']];table(directory/'failure_cases.csv',failures)
    validation=dict(status='passed',runs=len(summaries),conditions=len(condition_results),**audit_counts,
        plan_frozen_before_collection=True,plan_sha256=m['frozen_plan_sha256'],exact_previous_path_overlaps=0,
        policies_called_unchanged=True,policy_source_sha256=plan['policy_source_sha256'],calibration_unchanged=True,
        scene_config_unchanged=True,safety_priority=True,force_command_audited=True,actual_unloading_and_retry_pose_audited=True,
        source_archives_verified=len(m['source_sha256']),preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=m['preexisting_outputs_changed'])
    assert not validation['preexisting_outputs_changed']
    (directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    plots(directory,summaries,totals,strata,all_events,condition_results,plt)
    report(directory,m,plan,totals,effects,strata,failures,validation)
    return totals


def plots(directory,rows,totals,strata,events,conditions,plt):
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    x=np.arange(4);colors=[COLORS[p] for p in POLICIES]
    for ax,key,title in zip(axes,('successes','stalled_runs','safety_stops'),('Insertion successes / 48','Ever stalled / 48','Hard safety stops / 48')):
        values=[r[key] for r in totals];ax.bar(x,values,color=colors);ax.set(xticks=x,xticklabels=POLICIES,title=title,ylim=(0,50));ax.grid(axis='y',alpha=.2)
        for i,v in enumerate(values):ax.text(i,v+.6,str(v),ha='center')
    fig.tight_layout();fig.savefig(directory/'policy_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4))
    for ax,dimension in zip(axes,('family','severity','ramp_onset_mm')):
        values=sorted({r['value'] for r in strata if r['dimension']==dimension});xx=np.arange(len(values))
        for pi,policy in enumerate(POLICIES):
            rr=[next(r for r in strata if r['dimension']==dimension and r['value']==v and r['policy']==policy) for v in values]
            ax.bar(xx+(pi-1.5)*.2,[r['success_rate'] for r in rr],width=.2,label=policy,color=COLORS[policy])
        ax.set(xticks=xx,xticklabels=[str(v).replace('_','\n') for v in values],ylim=(0,1.05),title=dimension,ylabel='Success fraction');ax.tick_params(axis='x',labelsize=8);ax.legend(fontsize=8);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'stratified_success.png',dpi=150);plt.close(fig)
    data=np.array([[r[p+'_insertion_success'] for p in POLICIES] for r in conditions],float)
    fig,ax=plt.subplots(figsize=(10,14));ax.imshow(data,aspect='auto',cmap='RdYlGn',vmin=0,vmax=1)
    ax.set(xticks=range(4),xticklabels=POLICIES,yticks=range(len(conditions)),yticklabels=[r['case_id']+f" d0={r['ramp_onset_mm']:g}" for r in conditions],title='Each frozen condition: 0 = no success; 1 = success');ax.tick_params(axis='y',labelsize=7)
    for i in range(len(conditions)):
        for j in range(4):ax.text(j,i,str(int(data[i,j])),ha='center',va='center',fontsize=7)
    fig.tight_layout();fig.savefig(directory/'condition_outcomes.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(14,4))
    for ax,key,label in zip(axes,('max_wrist_force_n','max_wrist_torque_nm','max_normal_load_n'),('Max wrist force (N)','Max wrist torque (Nm)','Max privileged load (N)')):
        ax.boxplot([[r[key] for r in rows if r['policy']==p] for p in POLICIES],tick_labels=POLICIES);ax.set_ylabel(label);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'wrench_load_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for policy in POLICIES[1:]:
        ee=[e for e in events if e['policy']==policy]
        for verified in (False,True):
            subset=[e for e in ee if e['online_verified']==verified]
            if not subset:continue
            style=dict(edgecolors=COLORS[policy],facecolors=COLORS[policy] if verified else 'none',label=policy+(' verified' if verified else ' unverified'))
            axes[0].scatter([e['commanded_retract_mm'] for e in subset],[e['actual_retract_mm'] for e in subset],**style)
            axes[1].scatter([e['before_mean_normal_load_n'] for e in subset],[e['after_mean_normal_load_n'] for e in subset],**style)
    axes[0].axhline(.5,ls=':',color='black');axes[0].set(xlabel='Maximum authored axial command (mm)',ylabel='Actual retreat at recovery end (mm)')
    lim=max([e['before_mean_normal_load_n'] for e in events]+[1.]);axes[1].plot([0,lim],[0,lim],':',color='black');axes[1].set(xlabel='Mean contact load before trigger (N)',ylabel='Mean contact load after recovery (N)')
    for ax in axes:ax.grid(alpha=.2);ax.legend(fontsize=7)
    fig.tight_layout();fig.savefig(directory/'recovery_comparison.png',dpi=150);plt.close(fig)


def report(directory,m,plan,totals,effects,strata,failures,audit):
    by={r['policy']:r for r in totals};d=by['dewedge'];ax=by['axial']
    lines=['# Frozen Contact Productivity generalization benchmark','',
        '48 unseen pose/misalignment conditions × four frozen policies = 192 episodes. No policy retuning, outcome-driven condition replacement or ML. Same audited peg/socket assets and native 120 Hz FORGE physics.',
        '', '| Policy | Insertion success | Ever stalled | Safety stops | Verified / recovery attempts | Successful retries / retries | Mean final depth (mm) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for r in totals:
        verified=f"{r['verified_unloadings']}/{r['recoveries']}" if r['policy'] in ('axial','dewedge') else 'not enforced'
        lines.append(f"| {r['policy']} | {r['successes']}/{r['conditions']} | {r['stalled_runs']} | {r['safety_stops']} | {verified} | {r['retry_successes']}/{r['retries']} | {r['mean_final_depth_mm']:.3f} |")
    lines+=['','## Main questions','',f"**Unseen-condition success:** de-wedging succeeded in {d['successes']}/48 conditions ({d['success_rate']:.1%}). Nominal: {by['nominal']['successes']}/48; force-threshold: {by['force']['successes']}/48; axial-only: {ax['successes']}/48.",
        '',f"**Verified unloading:** de-wedging completed the 0.25 s measured-retreat verification in {d['verified_unloadings']}/{d['recoveries']} recovery attempts and remained unloaded through the extra hold in {d['ready_unloadings']}. It produced {d['retry_successes']} successful retries from {d['retries']} active retries, with {d['repeated_stalls']} repeated stalls among {d['evaluable_retries']} evaluable retries. Axial-only verified {ax['verified_unloadings']}/{ax['recoveries']}. Triggered-attempt rates and overall success answer different questions: policies may intervene on different conditions/states.",
        '', '**Comparison without retuning:** all four existing execute functions and helper sources are frozen against the prior pilot; neither thresholds nor recovery logic changed. Paired effects below include all test conditions, including hard safety and numerical failures.',
        '', '| Baseline vs de-wedging | Conditions | De-wedging wins / losses | Success difference | Stratified condition-bootstrap 95% interval |',
        '|---|---:|---:|---:|---|']
    for r in effects:
        if r['scope']!='all_conditions':continue
        lines.append(f"| {r['baseline']} | {r['conditions']} | {r['wins']} / {r['losses']} | {r['success_difference']:+.1%} | [{r['ci_low']:+.1%}, {r['ci_high']:+.1%}] |")
    if all(d['successes']>by[p]['successes'] for p in ('nominal','force','axial')):
        lines+=['','De-wedging maintained a higher observed success rate than all three baselines in this frozen benchmark. This supports generalization within the tested misalignment design; it is not a guarantee on arbitrary geometries, clearances, materials or contact states.']
    else:lines+=['','De-wedging did not maintain a strictly higher success count than every baseline. Inspect the condition and failure tables before claiming generalization. No retuning or additional test-set search was performed.']
    df=[r for r in failures if r['policy']=='dewedge']
    lines+=['','## Which unseen conditions fail?','',f"De-wedging failures: {len(df)}/48. Failure classes: {dict(Counter(r['failure_class'] for r in df))}.",
        '', '| Condition | Onset (mm) | Outcome | Final depth (mm) | Recoveries | Verified | Plot |', '|---|---:|---|---:|---:|---:|---|']
    for r in df:lines.append(f"| {r['case_id']} | {r['ramp_onset_mm']:g} | {r['failure_class']} | {r['final_depth_mm']:.3f} | {r['recovery_count']} | {r['verified_unloadings']} | [history]({r['condition_plot']}) |")
    if not df:lines+=['','No de-wedging failure was observed. This does not measure a failure boundary; the benchmark remains finite and uses one preparation per condition/policy.']
    failure_lines=['# Failure-case analysis','', 'All unsuccessful episodes are retained, including safety and numerical stops. Categories describe logged outcomes, not proven physical causes. A no-trigger timeout is distinguished from failed unloading and a stall after retry.', '',
        '| Policy | Condition | Failure class | Final depth (mm) | Max wrist force (N) | Max wrist torque (Nm) | Max load (N) | History |', '|---|---|---|---:|---:|---:|---:|---|']
    for r in failures:failure_lines.append(f"| {r['policy']} | {r['case_id']} | {r['failure_class']} | {r['final_depth_mm']:.3f} | {r['max_wrist_force_n']:.3f} | {r['max_wrist_torque_nm']:.4f} | {r['max_normal_load_n']:.3f} | [plot]({r['condition_plot']}) |")
    (directory/'failure_case_analysis.md').write_text('\n'.join(failure_lines)+'\n')
    lines+=['','All-policy failure details: [failure_case_analysis.md](failure_case_analysis.md) and failure_cases.csv. Per-condition endpoint parameters and four-policy outcomes are in per_condition_results.csv; each condition has a complete depth/load/productivity plot.',
        '', '## Frozen held-out design','',
        'Six families: axis offsets, oblique x/y offsets, axis tilts, oblique roll/pitch tilts, cross-axis offset/tilt combinations and oblique four-component combinations. Each has two severity levels and four signed variants: 48 distinct reference paths. Moderate component-vector magnitudes are 0.7 mm offset and 3.5 deg tilt; severe magnitudes are 1.2 mm and 7 deg. Absent components remain zero. Oblique vector components use 0.6/0.8 weights. The tilt magnitude refers to the roll/pitch parameter vector, not an exact quaternion rotation angle.',
        '', 'Ramp onset depths are 7, 12 and 15 mm (16 cases each), all different from the original 5/10 mm pilot onsets. The design balances coverage without claiming a full factorial crossing of every sign, amplitude and onset. Every path starts aligned, uses the existing maximum-previous-actual-depth feedback and reaches its prescribed endpoint at 20 mm. The original 8 s nominal insertion command, 20 s episode budget and preparation seed remain unchanged.',
        '', f"The plan was frozen at {plan['frozen_at_utc']}. {plan['holdout_audit']['descriptors_checked']} previous descriptors ({plan['holdout_audit']['unique_paths']} unique sampled reference paths) were checked; exact overlaps: 0. Paths may share their aligned early history by design. No training/fitting occurs here. This is held-out evaluation after previous pilot development, not a retroactive split of that pilot.",
        '', 'Condition order is randomized once; each policy occupies each within-condition ordinal position exactly 12 times. One run per policy/condition prioritizes 48 distinct conditions over duplicate repeats. Same reset seed and solver priming are used; strict pre-intervention state matching is reported rather than assumed.',
        '', 'Clearance variation was not included: the backend enforces unit mesh scale and the audited 9 mm bore. Changing diameter metadata does not alter the USD geometry; a real clearance change would require mesh/geometry-audit changes. No such changes were made. These are 48 new pose/misalignment conditions on the same solid assets.',
        '', '## Frozen policies and measurement','',
        f"Productivity eta threshold: {plan['eta_threshold']:.16g}, two consecutive 0.1 s checks, unchanged 0.5 s history and positive-command/contact-onset eligibility. Force baseline threshold: {plan['force_threshold_n']:.16g} N with the original same eligibility and two-check behavior; it executes the original fixed 1 mm / 0.5 s retract and rejoin. It is not the 20 N hard safety gate and is not newly calibrated.",
        '', 'Axial-only and de-wedging call the previous functions unchanged. De-wedging stops 0.25 s, relaxes captured actual peg x/y and relative roll/pitch toward neutral over 0.5 s at fixed commanded depth, preserves relative yaw, then retracts. Actual retreat must reach 0.5 mm and remain achieved for 0.25 s, followed by the existing extra 0.25 s hold. Relaxation/retraction/holds share the original 5 s deadline after the stop; axial command cap 5 mm with 4 s active ramp. Retry retains the measured relaxed pose. The original two-intervention cap and all hard force/torque/penetration/grasp limits remain.',
        '', 'Per-run trajectory.csv and events.json retain full wrench, actual pose/motion, target, detector and recovery histories. summary.csv adds productivity-trigger diagnostics on each policy\'s own observed history. For nominal/force policies those are offline shadow-detector results, not interventions executed by those policies. Actual productivity-triggered recoveries are a separate field. Simulator normal load is analysis-only.',
        '', 'Verified-unloading counts refer to the online measured-motion verification in axial/de-wedging arms. The force arm does not enforce verification; its achieved/sustained motion is still recorded separately in recovery_events.csv. Blank retry/stall labels mean no retry or insufficient qualifying motion. Failure endpoints and all safety stops remain in denominators. Time summaries include timeouts and stops; success-only times are separately identified.',
        '', 'Stalls use the unchanged contiguous-insertion predicate. A controller may terminate before confirming a stall; therefore low recorded stall frequency alone is not improved insertion. Before/after load means use the existing 0.1 s windows. Load changes combine changed pose, elapsed time and retraction; no hold-only ablation is implied.',
        '', '## Matching, uncertainty and integrity','',
        'paired_comparisons.csv retains all strict-prefix mismatches. Paired effect estimates resample whole conditions within the six design families, never physics samples. The intervals describe variation across this structured set under a condition-resampling assumption, not independently sampled real-world operating conditions. Wilson intervals in policy_summary.csv are also descriptive. Matching-filtered sensitivity results are separate and do not replace the all-condition primary endpoint.',
        '', '| Baseline | Strictly matched conditions | De-wedging wins / losses in matched subset |', '|---|---:|---:|']
    for r in effects:
        if r['scope']=='strict_prefix_matched':lines.append(f"| {r['baseline']} | {r['conditions']} | {r['wins']} / {r['losses']} |")
    lines+=['',f"Validation passed for {audit['physics_rows']} physics rows and {audit['causal_checks']} causal checks. The audit replays frozen detector decisions, bounded force commands, axial/de-wedging recovery state machines, achieved-motion dwell and retry-pose retention. {audit['source_archives_verified']} source snapshots verified; {audit['preexisting_outputs_checked']} pre-existing output files checked, changed: 0. Exact scene/configuration and calibration match the previous pilot. Frozen plan/policy hashes are checked before and during collection.",
        '', '## Files and reproduction','',
        'Primary data: summary.csv, per_condition_results.csv, policy_summary.csv, stratified_results.csv, paired_policy_effects.csv, paired_comparisons.csv, detector_checks.csv, recovery_events.csv, failure_cases.csv, validation.json and frozen_plan.json. Source snapshots and per-condition plots are retained.',
        '', '```bash', './productivity_generalization.sh --headless --output-dir outputs/Contact-Productivity-Generalization-v1', '```',
        '', 'Collection refuses an existing output directory. Offline report regeneration: `python -m research.productivity_generalization_report outputs/Contact-Productivity-Generalization-v1`.',
        '', '![Policy comparison](policy_comparison.png)', '', '![Stratified success](stratified_success.png)', '', '![Every condition](condition_outcomes.png)', '', '![Recovery](recovery_comparison.png)', '', '![Wrench and load](wrench_load_comparison.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory),indent=2))
