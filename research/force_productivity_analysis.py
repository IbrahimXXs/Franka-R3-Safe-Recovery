"""Causal shadow comparison, development-only selection, and frozen recovery audit."""
from collections import Counter
from datetime import datetime,timezone
import json
from pathlib import Path
from types import FunctionType
import numpy as np
from research.force_productivity import ForceDetector,force_features
from research.force_productivity_design import THRESHOLDS
from research.productivity_control import Design
from research.productivity_detector_v2 import DetectorV2,features,NORMAL_ETA
from research.productivity_detector_v2_report import audit_run as v2_audit
from research.productivity_dewedge_report import audit as original_audit,event_features
from research.productivity_detector_v2_generalization_report import paired_prefix,exact_mcnemar,write_table
from research.productivity_control_report import read_log
from research.productivity_unloading_report import safe
from research.forge_protocol import ForgeProtocol
from research.productivity_generalization import sha


def state_features(rows,i,hz):
    r=rows[i];n=round(.5*hz);w=rows[max(0,i-n):i+1]
    valid=len(w)==n+1 and all(x['segment']==r['segment'] and x['phase']==r['phase'] for x in w)
    da=r['depth_mm']-w[0]['depth_mm'] if valid else None
    dc=r['command_depth_mm']-w[0]['command_depth_mm'] if valid else None
    return dict(time_s=r['time_s'],depth_mm=r['depth_mm'],phase=r['phase'],wrist_force_n=r['wrist_force_n'],
        eta_raw=da/dc if valid and dc>=Design().minimum_command_mm-1e-9 else None,
        actual_progress_rate_mm_s=da/.5 if valid else None,command_progress_rate_mm_s=dc/.5 if valid else None,
        diagnostic_normal_load_n=r['normal_load_n'])


def shadow(rows,case,hz,p,kind,config):
    d=ForceDetector(config['threshold_n']) if kind=='force' else DetectorV2(config);alarms=[]
    for i in range(round(.1*hz),len(rows),round(.1*hz)):
        signal=force_features(rows[:i+1],hz,case['ramp_onset_mm']) if kind=='force' else features(rows[:i+1],hz,case['ramp_onset_mm'],success_depth=p.success_depth_mm)
        if d.check(rows[i]['time_s'],signal):
            alarms.append(dict(**state_features(rows,i,hz),safe=safe(rows[i],p),branch=d.last_decision['trigger_branch']))
    return alarms


def first_safe(alarms):return next((a for a in alarms if a['safe']),None)


def shadow_summary(rows,run,case,hz,p,kind,config):
    alarms=shadow(rows,case,hz,p,kind,config);a=first_safe(alarms)
    safety_time=rows[-1]['time_s'] if not safe(rows[-1],p) else None
    stall=run['first_stall_s']
    return dict(case_id=run['case_id'],family=run['family'],severity=run['severity'],detector=kind,
        nominal_success=run['insertion_success'],nominal_outcome=run['outcome'],nominal_stalled=run['stalled'],
        nominal_first_stall_s=stall,nominal_safety_time_s=safety_time,shadow_triggered=a is not None,
        first_trigger_time_s=a['time_s'] if a else None,first_trigger_depth_mm=a['depth_mm'] if a else None,
        branch=a['branch'] if a else None,wrist_force_at_trigger_n=a['wrist_force_n'] if a else None,
        eta_at_trigger=a['eta_raw'] if a else None,actual_progress_rate_mm_s=a['actual_progress_rate_mm_s'] if a else None,
        command_progress_rate_mm_s=a['command_progress_rate_mm_s'] if a else None,
        safety_lead_s=safety_time-a['time_s'] if safety_time is not None and a else None,
        stall_lead_s=stall-a['time_s'] if stall is not None and a else None,
        safe_alarm_checks=sum(x['safe'] for x in alarms),false_alert=bool(run['insertion_success'] and a)),alarms


def audit_run(directory,m,run,rows,v2config):
    if run['policy']!='force':
        return v2_audit(directory,m,dict(run,policy='v2' if run['policy']=='productivity' else 'nominal'),rows,v2config)
    case=next(c['case'] for c in m['cases'] if c['case_id']==run['case_id']);hz=m['physics_hz']
    class ReplayDetector:
        def __init__(self,policy,eta,force,design):self.d=ForceDetector(run['force_threshold_n'],design)
        @property
        def count(self):return self.d.count
        def reset(self):self.d.reset()
        def check(self,t,normal_signal):
            i=round(t*hz);signal=force_features(rows[:i+1],hz,case['ramp_onset_mm']);fired=self.d.check(t,signal)
            r=rows[i]
            for k,v in self.d.last_decision.items():
                if k=='normal_eta_valid':continue
                assert (r[k] or '')==v if k=='trigger_branch' else r[k]==v,(k,r[k],v)
            assert r['force_threshold_n']==run['force_threshold_n']
            return fired
    fn=FunctionType(original_audit.__code__,dict(original_audit.__globals__,Detector=ReplayDetector))
    proxy=dict(run,policy='dewedge')
    return fn(directory,dict(m,runs=[proxy]),{run['run_id']:rows})


def measure(directory,m,run,rows):
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz'];ee=json.loads((directory/run['run_id']/'events.json').read_text());events=[]
    for i,e in enumerate(ee):
        f=event_features(rows,e,ee[i+1] if i+1<len(ee) else None,p,hz);index=round(e['trigger_time_s']*hz)
        f.update(case_id=run['case_id'],family=run['family'],severity=run['severity'],split=run['split'],
            policy=run['policy'],detector_id=run['detector_id'],run_id=run['run_id'],trigger_branch=rows[index].get('trigger_branch'),
            **{'trigger_'+k:v for k,v in state_features(rows,index,hz).items()})
        events.append(f)
    starts=[r for r in rows if r['intervention_started']]
    result=dict(**run,safety_stop=not safe(rows[-1],p),grasp_retention_failure=run['outcome']=='grasp_retention_limit',
        terminal_timeout=run['outcome']=='time_budget_exhausted' and rows[-1]['phase']=='hold',terminal_phase=rows[-1]['phase'],
        safety_before_any_recovery=not safe(rows[-1],p) and not starts,
        safety_during_recovery=not safe(rows[-1],p) and rows[-1]['phase'] in ('stop','relax','unload_retract','unload_hold','post_verify_hold'),
        verified_unloadings=sum(e['online_verified'] for e in events),ready_unloadings=sum(e['ready_to_retry'] for e in events),
        retries=sum(e['retry_started'] for e in events),retry_successes=sum(e['retry_success'] is True for e in events),
        repeated_stalls=sum(e['stall_after_retry'] is True for e in events),first_trigger_time_s=starts[0]['time_s'] if starts else None,
        first_trigger_depth_mm=starts[0]['depth_mm'] if starts else None)
    assert run['rows']==len(rows) and len(starts)==run['intervention_count']==len(events)
    for key,column in (('max_wrist_force_n','wrist_force_n'),('max_wrist_torque_nm','wrist_torque_nm'),('max_normal_load_n','normal_load_n')):
        assert abs(run[key]-max(r[column] for r in rows))<1e-8
    assert abs(run['final_depth_mm']-rows[-1]['depth_mm'])<1e-8
    if run['insertion_success']:
        assert all(safe(r,p) and r['depth_mm']>=p.success_depth_mm and r['phase'] in ('insert','hold') for r in rows[-round(p.success_dwell_s*hz):])
    return result,events


def development_candidates(directory,m,plan):
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz'];rows=[];sources={};shadow_details=[]
    nominal=[r for r in m['runs'] if r['split']=='development' and r['policy']=='nominal']
    if len(nominal)!=32:raise ValueError('All 32 new nominal development references required')
    for threshold in THRESHOLDS:
        details=[];useful=covered=false=0;lead_score=0.
        for run in nominal:
            log=read_log(directory/run['log']);sources[run['log']]=sha(directory/run['log'])
            c=next(c['case'] for c in plan['cases'] if c['case_id']==run['case_id'])
            d,alarms=shadow_summary(log,run,c,hz,p,'force',dict(threshold_n=threshold));a=first_safe(alarms)
            events=[x for x in (run['first_stall_s'],d['nominal_safety_time_s']) if x is not None]
            deadline=min(events)-plan['selection']['minimum_safety_stall_lead_s'] if events else log[-1]['time_s']-plan['selection']['timeout_recovery_reserve_s']
            good=bool(not run['insertion_success'] and a and a['time_s']<=deadline+1e-8)
            useful+=good;covered+=bool(not run['insertion_success'] and a);false+=d['false_alert']
            lead_score+=min(2.,max(0.,deadline-a['time_s'])) if good else 0.
            details.append(dict(**d,threshold_n=threshold,useful_failure_alert=good,usefulness_deadline_s=deadline))
        successes=sum(r['insertion_success'] for r in nominal);failures=len(nominal)-successes
        if not successes or not failures:raise ValueError('Development needs successful and failed references')
        candidates=[r for r in m['runs'] if r['split']=='development' and r['detector_id']==f'force_{threshold:g}']
        if len(candidates)!=32:raise ValueError('Every threshold needs 32 closed-loop development episodes')
        measured=[]
        for r in candidates:measured.append(measure(directory,m,r,read_log(directory/r['log']))[0])
        median=lambda key:float(np.median([d[key] for d in details if d[key] is not None])) if any(d[key] is not None for d in details) else None
        natural={r['case_id']:r['insertion_success'] for r in nominal}
        attempts=sum(r['intervention_count'] for r in measured);retries=sum(r['retries'] for r in measured)
        rows.append(dict(threshold_n=threshold,nominal_successes=successes,nominal_failures=failures,
            false_alerts=false,false_alert_fraction=false/successes,failed_cases_alerted=covered,failure_coverage=covered/failures,
            useful_failed_cases_alerted=useful,capped_useful_lead_score_s=lead_score,
            admissible=false/successes<=plan['selection']['max_success_alert_fraction'],
            closed_loop_successes=sum(r['insertion_success'] for r in measured),interventions=sum(r['intervention_count'] for r in measured),
            verified_unloadings=sum(r['verified_unloadings'] for r in measured),retry_successes=sum(r['retry_successes'] for r in measured),
            safety_stops=sum(r['safety_stop'] for r in measured),
            median_safety_lead_s=median('safety_lead_s'),median_stall_lead_s=median('stall_lead_s'),
            actual_interventions_on_nominal_success=sum(natural[r['case_id']] and r['intervention_count']>0 for r in measured),
            verified_unloading_rate=sum(r['verified_unloadings'] for r in measured)/attempts if attempts else None,
            retry_success_rate=sum(r['retry_successes'] for r in measured)/retries if retries else None))
        shadow_details.extend(details)
    return rows,sources,shadow_details


def select_candidate(candidates):
    admissible=[r for r in candidates if r['admissible']];pool=admissible or candidates
    if not pool:raise ValueError('Empty candidate set')
    key=lambda r:((0 if admissible else -r['false_alerts']),r['useful_failed_cases_alerted'],r['failed_cases_alerted'],
        -r['false_alerts'],r['capped_useful_lead_score_s'],r['threshold_n'])
    return max(pool,key=key)


def freeze_force(directory,m,plan):
    directory=Path(directory);target=directory/'force_detector_config.json'
    if target.exists():raise FileExistsError('Never overwrite selected detector configuration')
    if len(m['runs'])!=192 or any(r['split']=='held_out' for r in m['runs']):raise ValueError('Freeze only after development and before held-out collection')
    old=json.loads((directory/'preexisting_outputs.json').read_text())
    assert all(Path(f).is_file() and [Path(f).stat().st_size,Path(f).stat().st_mtime_ns]==st for f,st in old.items())
    checked=Counter();audit_manifest=dict(m,preexisting_outputs_changed=[],preexisting_outputs_checked=len(old))
    v2config=json.loads((directory/'detector_v2_config.json').read_text())
    for run in m['runs']:
        a=audit_run(directory,audit_manifest,run,read_log(directory/run['log']),v2config)
        for key in ('physics_rows','causal_checks','interventions','retry_ready_unloadings'):checked[key]+=a[key]
    (directory/'force_development_validation.json').write_text(json.dumps(dict(status='passed',runs=192,**checked),indent=2)+'\n')
    candidates,sources,details=development_candidates(directory,m,plan);selected=select_candidate(candidates)
    config=dict(schema='Frozen-Force-Detector-v1',frozen_at_utc=datetime.now(timezone.utc).isoformat(),
        threshold_n=selected['threshold_n'],comparison='strictly greater',consecutive_checks=2,history_s=.5,check_s=.1,
        input_features=['wrist_force_n'],eligibility=plan['force_eligibility'],selection=plan['selection'],selected_development_metrics=selected,
        false_alert_cap_met=selected['admissible'],development_nominal_sha256=sources,development_episodes=192,
        condition_plan_sha256=m['condition_plan_sha256'],held_out_used_for_selection=False,closed_loop_outcomes_used_for_selection=False,
        privileged_normal_load_used=False)
    target.write_text(json.dumps(config,indent=2,allow_nan=False)+'\n')
    (directory/'force_detector_config.sha256').write_text(sha(target)+'  force_detector_config.json\n')
    write_table(directory/'force_development_candidates.csv',candidates,('threshold_n',))
    write_table(directory/'force_development_shadow.csv',details,('case_id','threshold_n'))
    lines=['# Force detector development','',
        '32 entirely new conditions; 32 nominal references plus 32 closed-loop runs for each of five force thresholds. Every closed-loop candidate uses the exact frozen de-wedging implementation.',
        '', '| Force threshold (N) | Useful failed cases | Any failed cases alerted | Alerts on successes | Closed-loop success /32 | Interventions | Verified | Retry successes | Safety stops | Admissible |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---|']
    for r in candidates:
        lines.append(f"| {r['threshold_n']:g} | {r['useful_failed_cases_alerted']}/{r['nominal_failures']} | {r['failed_cases_alerted']}/{r['nominal_failures']} | {r['false_alerts']}/{r['nominal_successes']} | {r['closed_loop_successes']} | {r['interventions']} | {r['verified_unloadings']} | {r['retry_successes']} | {r['safety_stops']} | {r['admissible']} |")
    lines+=['', '| Threshold (N) | Median safety lead (s) | Median stall lead (s) | Actual interventions on nominal successes |', '|---|---:|---:|---:|']
    fmt=lambda x:'N/A' if x is None else f'{x:.3f}'
    for r in candidates:
        lines.append(f"| {r['threshold_n']:g} | {fmt(r['median_safety_lead_s'])} | {fmt(r['median_stall_lead_s'])} | {r['actual_interventions_on_nominal_success']} |")
    lines+=['',f"Selected: **force > {config['threshold_n']:g} N for two consecutive checks**. Configuration SHA256: `{sha(target)}`. False-alert cap met: {config['false_alert_cap_met']}.",
        '', 'Selection uses nominal development shadow alarms only: at most 10% of successful trajectories alerted; then maximize useful failure coverage, any safe failure coverage, fewer false alerts, capped lead, and a more conservative threshold. If no candidate meets the cap, minimize false alerts first and explicitly report that limitation. Closed-loop successes do not select the threshold.',
        '', 'Useful means a safe alert at least 0.1 s before the first nominal stall/safety event. For failures with neither event, require 5.25 s remaining before the episode deadline, preserving the existing full recovery budget. The latter is an operational time reserve, not an early stall/safety lead claim.',
        '', 'Force decision uses wrist force only. The original contact-onset gate and a full same-phase/segment 0.5 s causal history establish eligibility; insert and endpoint hold are eligible. Positive commanded progress is not required. Two adjacent 0.1 s checks reject single-sample spikes. No normal load or productivity value enters the decision.',
        '', 'Per-case stall/safety lead times and trigger states are in force_development_shadow.csv; per-run outcomes are in development_summary.csv. Development data are not included in the final held-out success estimates. Force-rise and hybrid variants were omitted before collection.',
        '', f"Frozen at {config['frozen_at_utc']}, before any held-out episode. No prior benchmark outcome or held-out result was used for selection."]
    (directory/'force_detector_dev_report.md').write_text('\n'.join(lines)+'\n')
    return config


def paired_statistics(pairs,subset='all'):
    rr=[r for r in pairs if (subset not in ('moderate','severe') or r['severity']==subset)
        and (subset!='strict_matched' or r['prefix_matched'])]
    n=len(rr);wins=sum(r['success_delta']==1 for r in rr);losses=sum(r['success_delta']==-1 for r in rr);ci=(None,None)
    if n:
        rng=np.random.default_rng(20260919);boot=np.zeros(10000)
        for family in sorted({r['family'] for r in rr}):
            a=np.array([r['success_delta'] for r in rr if r['family']==family]);boot+=rng.choice(a,size=(10000,len(a)),replace=True).sum(axis=1)
        ci=tuple(float(x) for x in np.percentile(boot/n,[2.5,97.5]))
    return dict(subset=subset,conditions=n,productivity_wins=wins,force_wins=losses,
        both_succeed=sum(r['force_success'] and r['productivity_success'] for r in rr),
        both_fail=sum(not r['force_success'] and not r['productivity_success'] for r in rr),
        paired_success_difference=(wins-losses)/n if n else None,ci95_low=ci[0],ci95_high=ci[1],
        exact_mcnemar_p=exact_mcnemar(wins,losses) if n else None)
