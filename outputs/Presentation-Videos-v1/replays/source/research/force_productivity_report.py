"""Frozen held-out detector ablation analysis; development and evaluation stay separate."""
import argparse
from collections import Counter
import json
from pathlib import Path
import numpy as np
from research.force_productivity_analysis import (audit_run,measure,shadow_summary,first_safe,state_features,
    development_candidates,select_candidate,paired_statistics,paired_prefix,write_table)
from research.force_productivity_design import verify,POLICIES
from research.productivity_detector_v2_generalization import CONFIG_SHA
from research.productivity_detector_v2 import NORMAL_ETA
from research.productivity_control_report import read_log
from research.productivity_generalization import sha
from research.forge_protocol import ForgeProtocol


def first_order(a,b):
    if a is None and b is None:return 'neither'
    if a is None:return 'productivity_only'
    if b is None:return 'force_only'
    if abs(a-b)<1e-6:return 'same_time'
    return 'force_first' if a<b else 'productivity_first'


def totals(summaries,shadows,pairs):
    result=[]
    for subset in ('all','moderate','severe','strict_matched'):
        ids={r['case_id'] for r in pairs if (subset not in ('moderate','severe') or r['severity']==subset)
            and (subset!='strict_matched' or r['prefix_matched'])}
        for policy in POLICIES:
            rr=[r for r in summaries if r['case_id'] in ids and r['policy']==policy]
            ss=[r for r in shadows if r['case_id'] in ids and r['detector']==policy]
            failures=[r for r in ss if not r['nominal_success']];successes=[r for r in ss if r['nominal_success']]
            safety=[r for r in ss if r['nominal_safety_time_s'] is not None]
            median=lambda key:float(np.median([r[key] for r in ss if r[key] is not None])) if any(r[key] is not None for r in ss) else None
            n=len(rr);recoveries=sum(r['intervention_count'] for r in rr);retries=sum(r['retries'] for r in rr)
            result.append(dict(subset=subset,policy=policy,conditions=n,successes=sum(r['insertion_success'] for r in rr),
                success_rate=sum(r['insertion_success'] for r in rr)/n if n else None,
                safety_stops=sum(r['safety_stop'] for r in rr),grasp_retention_failures=sum(r['grasp_retention_failure'] for r in rr),
                terminal_timeouts=sum(r['terminal_timeout'] for r in rr),stalled_runs=sum(r['stalled'] for r in rr),
                safety_before_any_recovery=sum(r['safety_before_any_recovery'] for r in rr),safety_during_recovery=sum(r['safety_during_recovery'] for r in rr),
                intervened_runs=sum(r['intervention_count']>0 for r in rr),interventions=recoveries,
                verified_unloadings=sum(r['verified_unloadings'] for r in rr),
                verified_unloading_rate=sum(r['verified_unloadings'] for r in rr)/recoveries if recoveries else None,
                ready_unloadings=sum(r['ready_unloadings'] for r in rr),retries=retries,
                retry_successes=sum(r['retry_successes'] for r in rr),retry_success_rate=sum(r['retry_successes'] for r in rr)/retries if retries else None,
                repeated_stalls=sum(r['repeated_stalls'] for r in rr),mean_final_depth_mm=float(np.mean([r['final_depth_mm'] for r in rr])) if rr else None,
                mean_insertion_time_s=float(np.mean([r['insertion_time_s'] for r in rr])) if rr else None,
                nominal_failures=len(failures),failed_cases_alerted=sum(r['shadow_triggered'] for r in failures),
                p_alert_given_failure=sum(r['shadow_triggered'] for r in failures)/len(failures) if failures else None,
                nominal_successes=len(successes),successes_alerted=sum(r['shadow_triggered'] for r in successes),
                p_alert_given_success=sum(r['shadow_triggered'] for r in successes)/len(successes) if successes else None,
                actual_interventions_on_nominal_success=sum(r['actual_interventions']>0 for r in successes),
                matched_nominal_successes=sum(r['actual_prefix_matched_to_nominal'] for r in successes),
                matched_interventions_on_nominal_success=sum(r['actual_prefix_matched_to_nominal'] and r['actual_interventions']>0 for r in successes),
                safety_cases=len(safety),safety_alerts=sum(r['shadow_triggered'] for r in safety),
                safety_alerts_at_least_point1s_early=sum(r['safety_lead_s'] is not None and r['safety_lead_s']>=.1-1e-8 for r in safety),
                median_safety_lead_s=median('safety_lead_s'),median_stall_lead_s=median('stall_lead_s')))
    return result


def compare_case(c,runs,raw,shadows,alarms,p,hz,threshold):
    a,b=runs['force'],runs['productivity'];match=paired_prefix(raw['force'],raw['productivity'],p)
    starts={k:next((i for i,r in enumerate(raw[k]) if r['intervention_started']),None) for k in ('force','productivity')}
    states={k:state_features(raw[k],i,hz) if i is not None else None for k,i in starts.items()}
    times={k:v['time_s'] if v else None for k,v in states.items()}
    out=dict(case_id=c['case_id'],family=c['family'],severity=c['severity'],nominal_success=runs['nominal']['insertion_success'],
        force_success=a['insertion_success'],productivity_success=b['insertion_success'],force_outcome=a['outcome'],productivity_outcome=b['outcome'],
        success_delta=int(b['insertion_success'])-int(a['insertion_success']),
        force_actual_trigger_time_s=times['force'],productivity_actual_trigger_time_s=times['productivity'],
        actual_trigger_order=first_order(times['force'],times['productivity']),
        force_shadow_trigger_time_s=shadows['force']['first_trigger_time_s'],
        productivity_shadow_trigger_time_s=shadows['productivity']['first_trigger_time_s'],
        shadow_trigger_order=first_order(shadows['force']['first_trigger_time_s'],shadows['productivity']['first_trigger_time_s']),
        force_actual_triggered=states['force'] is not None,productivity_actual_triggered=states['productivity'] is not None,
        force_shadow_triggered=shadows['force']['shadow_triggered'],productivity_shadow_triggered=shadows['productivity']['shadow_triggered'],**match)
    for policy in ('force','productivity'):
        for k in ('depth_mm','wrist_force_n','eta_raw','actual_progress_rate_mm_s','command_progress_rate_mm_s','diagnostic_normal_load_n'):
            out[policy+'_actual_trigger_'+k]=states[policy][k] if states[policy] else None
    out['force_magnitude_at_productivity_trigger_n']=out['productivity_actual_trigger_wrist_force_n']
    out['productivity_eta_at_force_trigger']=out['force_actual_trigger_eta_raw']
    examples=[]
    for kind,aa in alarms.items():
        for event in aa:
            if not event['safe']:continue
            eta=event['eta_raw']
            terminal=event['branch']=='terminal'
            low=kind=='productivity' and event['wrist_force_n']<=threshold and ((eta is not None and eta<NORMAL_ETA) or terminal)
            high=kind=='force' and eta is not None and eta>=NORMAL_ETA
            if low or high:
                label='A_terminal_low_force' if low and terminal else 'A_low_force_low_eta' if low else 'B_high_force_healthy_eta'
                if any(e['mechanism']==label for e in examples):continue
                other=shadows['force' if low else 'productivity']['first_trigger_time_s']
                examples.append(dict(case_id=c['case_id'],family=c['family'],severity=c['severity'],mechanism=label,
                    nominal_success=runs['nominal']['insertion_success'],nominal_outcome=runs['nominal']['outcome'],
                    other_detector_ever_alerts=other is not None,other_detector_already_alerted=other is not None and other<=event['time_s']+1e-8,
                    **event))
    return out,examples


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory).resolve();root=Path(__file__).resolve().parents[1]
    m=json.loads((directory/'experiment.json').read_text());plan=json.loads((directory/'condition_plan.json').read_text())
    config=json.loads((directory/'force_detector_config.json').read_text());v2=json.loads((directory/'detector_v2_config.json').read_text())
    verify(root,plan);assert m['status']=='complete' and len(m['runs'])==384
    assert m['force_frozen_after_runs']==192 and sha(directory/'force_detector_config.json')==m['force_detector_config_sha256']
    assert sha(directory/'detector_v2_config.json')==CONFIG_SHA and sha(directory/'condition_plan.json')==m['condition_plan_sha256']
    for r,spec in zip(m['runs'],plan['schedule']):assert all(r[k]==v for k,v in spec.items()),(r['run_id'],spec)
    assert not config['held_out_used_for_selection'] and not config['closed_loop_outcomes_used_for_selection']
    candidates,sources,_=development_candidates(directory,m,plan)
    assert select_candidate(candidates)==config['selected_development_metrics']
    assert sources==config['development_nominal_sha256'] and len(sources)==32
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz'];summaries=[];development=[];events=[];shadows=[];pairs=[];examples=[];audit=Counter()
    histories=directory/'conditions';histories.mkdir(exist_ok=True)
    for c in plan['cases']:
        runs=[r for r in m['runs'] if r['case_id']==c['case_id']];raw={};lookup={}
        for r in runs:
            rows=read_log(directory/r['log']);a=audit_run(directory,m,r,rows,v2)
            for key in ('physics_rows','causal_checks','interventions','retry_ready_unloadings'):audit[key]+=a[key]
            s,ee=measure(directory,m,r,rows)
            (development if c['split']=='development' else summaries).append(s);events.extend(ee)
            if c['split']=='held_out':raw[r['policy']]=rows;lookup[r['policy']]=r
        if c['split']=='development':continue
        assert set(raw)==set(POLICIES)
        ss={};aa={}
        for kind in ('force','productivity'):
            d,al=shadow_summary(raw['nominal'],lookup['nominal'],c['case'],hz,p,kind,config if kind=='force' else v2)
            d.update(actual_interventions=lookup[kind]['intervention_count'],
                actual_prefix_matched_to_nominal=paired_prefix(raw['nominal'],raw[kind],p)['prefix_matched'])
            ss[kind]=d;aa[kind]=al;shadows.append(d)
        pair,ex=compare_case(c,lookup,raw,ss,aa,p,hz,config['threshold_n']);pairs.append(pair);examples.extend(ex)
        fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True)
        for policy,color in zip(POLICIES,('#777777','#b17824','#008c91')):
            rr=raw[policy];t=[r['time_s']-2 for r in rr]
            for ax,key in zip(axes,('depth_mm','wrist_force_n','grasp_slip_mm')):ax.plot(t,[r[key] for r in rr],color=color,label=policy)
            for r in rr:
                if r['intervention_started']:axes[0].scatter(r['time_s']-2,r['depth_mm'],color=color,marker='x')
        axes[0].axhline(p.success_depth_mm,color='black',ls=':');axes[0].set(ylabel='Actual depth (mm)',title=c['case_id'])
        axes[1].axhline(config['threshold_n'],color='#b17824',ls=':');axes[1].set_ylabel('Wrist force (N)')
        axes[2].axhline(.1,color='black',ls=':');axes[2].set(ylabel='Grasp slip (mm)',xlabel='Insertion elapsed time (s)')
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        fig.tight_layout();fig.savefig(histories/(c['case_id']+'.png'),dpi=115);plt.close(fig)
    stats=[paired_statistics(pairs,s) for s in ('all','moderate','severe','strict_matched')]
    policy=totals(summaries,shadows,pairs);discordant=[r for r in pairs if r['success_delta']]
    failures=[r for r in summaries if not r['insertion_success']]
    for name,data in (('summary',summaries),('development_summary',development),('policy_summary',policy),
        ('detector_comparison',pairs),('shadow_detector_analysis',shadows),('paired_comparisons',pairs),('paired_statistics',stats),
        ('discordant_cases',discordant),('recovery_events',[e for e in events if e['split']=='held_out']),
        ('development_recovery_events',[e for e in events if e['split']=='development']),('failure_cases',failures),('mechanism_examples',examples)):
        write_table(directory/(name+'.csv'),data,('case_id','policy'))
    assert all(sha(root/f)==h and sha(directory/'source'/f)==h for f,h in m['source_sha256'].items())
    assert not m['preexisting_outputs_changed']
    old=json.loads((directory/'preexisting_outputs.json').read_text())
    assert all(Path(f).is_file() and [Path(f).stat().st_size,Path(f).stat().st_mtime_ns]==st for f,st in old.items())
    validation=dict(status='passed',development_conditions=32,held_out_conditions=64,episodes=384,**audit,
        exact_same_recovery_code_object=True,identical_recovery_parameters=True,full_recovery_safety_replay=True,
        force_only_decision_features=True,privileged_normal_load_not_used_online=True,force_selection_development_only=True,
        held_out_config_frozen=True,source_archives_verified=len(m['source_sha256']),no_old_paths_or_development_test_overlap=True,
        scene_config_unchanged=m['exact_previous_scene_and_config_match'],
        force_detector_config_sha256=m['force_detector_config_sha256'],detector_v2_config_sha256=CONFIG_SHA,
        strict_preintervention_matches=sum(r['prefix_matched'] for r in pairs),preexisting_outputs_checked=len(old),preexisting_outputs_changed=[])
    (directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    report(directory,plan,config,policy,stats,pairs,shadows,examples,failures,validation)
    fig,axes=plt.subplots(1,3,figsize=(13,4));allrows=[r for r in policy if r['subset']=='all']
    for ax,key,title in zip(axes,('successes','safety_stops','terminal_timeouts'),('Success /64','Safety stops /64','Terminal timeouts /64')):
        vv=[r[key] for r in allrows];ax.bar(POLICIES,vv,color=['#777777','#b17824','#008c91']);ax.set(title=title,ylim=(0,68))
        for i,v in enumerate(vv):ax.text(i,v+.4,str(v),ha='center')
    fig.tight_layout();fig.savefig(directory/'policy_comparison.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    for ax,key,title in zip(axes,('p_alert_given_failure','p_alert_given_success'),('P(alert | nominal failure)','P(alert | nominal success)')):
        rr=[r for r in allrows if r['policy']!='nominal'];vals=[r[key] or 0. for r in rr]
        ax.bar(['force','productivity'],vals,color=['#b17824','#008c91']);ax.set(title=title,ylim=(0,1.05))
    fig.tight_layout();fig.savefig(directory/'shadow_comparison.png',dpi=140);plt.close(fig)
    fig,ax=plt.subplots(figsize=(8,5))
    for kind,color in (('force','#b17824'),('productivity','#008c91')):
        rr=[r for r in shadows if r['detector']==kind and r['eta_at_trigger'] is not None]
        ax.scatter([r['wrist_force_at_trigger_n'] for r in rr],[r['eta_at_trigger'] for r in rr],label=kind,color=color,alpha=.7)
    ax.axvline(config['threshold_n'],ls=':',color='#b17824');ax.axhline(NORMAL_ETA,ls=':',color='#008c91')
    ax.set(xlabel='Wrist force at first safe shadow trigger (N)',ylabel='Recent actual / commanded progress',title='Same nominal histories; terminal eta is undefined and omitted');ax.legend();ax.grid(alpha=.2)
    fig.tight_layout();fig.savefig(directory/'force_productivity_triggers.png',dpi=140);plt.close(fig)
    artifacts=[directory/r['run_id']/name for r in m['runs'] for name in ('trajectory.csv','events.json','metrics.json')]
    artifacts += [directory/name for name in ('condition_plan.json','force_detector_config.json','detector_v2_config.json','calibration.json','scene.json','config.json','experiment.json')]
    provenance=dict(condition_plan_sha256=m['condition_plan_sha256'],frozen_at_utc=plan['frozen_at_utc'],
        collection_started_utc=m['collection_started_utc'],force_frozen_at_utc=config['frozen_at_utc'],force_frozen_after_runs=192,
        force_detector_config_sha256=m['force_detector_config_sha256'],detector_v2_config_sha256=CONFIG_SHA,
        source_sha256=m['source_sha256'],prior_descriptor_overlap_audit=plan['holdout_audit'],
        raw_artifact_sha256={str(f.relative_to(directory)):sha(f) for f in artifacts},environment=m['environment'],
        elapsed_seconds=m['elapsed_seconds'],command='./force_productivity_dewedge.sh --headless',
        preexisting_outputs_checked=len(old),preexisting_outputs_changed=[])
    (directory/'provenance_audit.json').write_text(json.dumps(provenance,indent=2)+'\n');verify(root,plan)
    return policy

def report(directory,plan,config,policy,stats,pairs,shadows,examples,failures,validation):
    overall={r['policy']:r for r in policy if r['subset']=='all'};force=overall['force'];prod=overall['productivity'];primary=stats[0]
    failed=[r for r in pairs if not r['nominal_success']]
    categories=Counter(('both' if r['force_shadow_triggered'] and r['productivity_shadow_triggered'] else
        'force_only' if r['force_shadow_triggered'] else 'productivity_only' if r['productivity_shadow_triggered'] else 'neither') for r in failed)
    a=[r for r in examples if r['mechanism'].startswith('A_')];b=[r for r in examples if r['mechanism'].startswith('B_')]
    a_missed={r['case_id'] for r in a if not r['other_detector_ever_alerts'] and not r['nominal_success']}
    b_success={r['case_id'] for r in b if r['nominal_success']}
    lines=['# Force versus Contact Productivity: identical de-wedging recovery','',
        f"32 new development conditions (192 episodes) selected force **> {config['threshold_n']:g} N for two checks**. A separate frozen set of 64 new conditions compared nominal, force + de-wedging and productivity-v2 + the exact same de-wedging (192 episodes). No held-out outcomes selected any threshold or policy.",
        '', '| Subset | Policy | Success | Safety stops | Grasp failures | Terminal timeouts | Stalled runs |', '|---|---|---:|---:|---:|---:|---:|']
    for r in policy:
        lines.append(f"| {r['subset']} | {r['policy']} | {r['successes']}/{r['conditions']} | {r['safety_stops']} | {r['grasp_retention_failures']} | {r['terminal_timeouts']} | {r['stalled_runs']} |")
    lines+=['','## Paired detector comparison','',
        '| Subset | N | Productivity wins | Force wins | Both succeed | Both fail | Difference [95% CI] | Exact paired p |',
        '|---|---:|---:|---:|---:|---:|---|---:|']
    for r in stats:
        diff=f"{100*r['paired_success_difference']:+.1f} pp [{100*r['ci95_low']:+.1f}, {100*r['ci95_high']:+.1f}]" if r['conditions'] else 'N/A'
        pv=f"{r['exact_mcnemar_p']:.6g}" if r['conditions'] else 'N/A'
        lines.append(f"| {r['subset']} | {r['conditions']} | {r['productivity_wins']} | {r['force_wins']} | {r['both_succeed']} | {r['both_fail']} | {diff} | {pv} |")
    lines+=['', 'Primary inference is the exact two-sided conditional McNemar/binomial test across all 64 paired conditions. The 95% percentile interval uses 10,000 paired condition bootstraps stratified by family (seed 20260919). Secondary moderate/severe and matched analyses are descriptive. Sparse discordances can make bootstrap intervals optimistic; a degenerate interval does not establish equivalence. Conditions are a structured test, not a random population sample.',
        '', '## Shared-history detector quality','',
        '| Subset | Detector | P(alert given failure) | P(alert given success) | Actual interventions on nominal successes | Median safety lead (s) | Median stall lead (s) |',
        '|---|---|---|---|---:|---:|---:|']
    fmt=lambda x:'N/A' if x is None else f'{x:.3f}'
    for r in policy:
        if r['policy']=='nominal':continue
        lines.append(f"| {r['subset']} | {r['policy']} | {r['failed_cases_alerted']}/{r['nominal_failures']} ({fmt(r['p_alert_given_failure'])}) | {r['successes_alerted']}/{r['nominal_successes']} ({fmt(r['p_alert_given_success'])}) | {r['actual_interventions_on_nominal_success']} | {fmt(r['median_safety_lead_s'])} | {fmt(r['median_stall_lead_s'])} |")
    lines+=['', 'Both shadow detectors see exactly the same nominal history. Only safe alarm samples count. Positive lead means the first safe alarm preceded the first nominal stall/safety stop; negative means it followed the first stall. Medians condition on an alarm and an event; missed cases remain in coverage denominators. An endpoint timeout is not converted into a fictitious early safety warning.',
        '', 'A detector alert on a nominal trajectory that subsequently succeeds is the conservative unnecessary-trigger label. Actual intervention on a paired nominal success is separately reported because resets can differ. Strict nominal-versus-controller match counts are in policy_summary.csv and shadow_detector_analysis.csv. Nominal safety events alerted at least 0.1 s early: '+f"force {force['safety_alerts_at_least_point1s_early']}/{force['safety_cases']}, productivity {prod['safety_alerts_at_least_point1s_early']}/{prod['safety_cases']}.",
        '', '## Recovery held identical','',
        '| Policy | Intervened runs | Attempts | Verified | Ready | Retry success / retries | Repeated stalls | Mean final depth (mm) | Safety before / during recovery |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in overall.values():
        lines.append(f"| {r['policy']} | {r['intervened_runs']} | {r['interventions']} | {r['verified_unloadings']} | {r['ready_unloadings']} | {r['retry_successes']}/{r['retries']} | {r['repeated_stalls']} | {r['mean_final_depth_mm']:.3f} | {r['safety_before_any_recovery']} / {r['safety_during_recovery']} |")
    lines+=['', 'Force and productivity bind the same execute_dewedge code object. Only Detector, recent_signal, and diagnostic Stream bindings differ. There is no old fixed-command force recovery in this experiment. Both retain identical x/y and roll/pitch relaxation, axial retraction, ≥0.5 mm actual retreat held for 0.25 s, post-verification hold, retry pose, motion/time budgets, gains and hard safety. Verified and ready-to-retry are distinct because the extra frozen hold can still fail. No normal load enters either online detector.',
        '', '## Discordances and mechanism examples','',
        '| Discordant condition | Force outcome | Productivity outcome | Actual first trigger | Same-nominal first trigger | Force at productivity trigger (N) | Eta at force trigger | Strict match |',
        '|---|---|---|---|---|---:|---:|---|']
    for r in pairs:
        if r['success_delta']:
            lines.append(f"| [{r['case_id']}](conditions/{r['case_id']}.png) | {r['force_outcome']} | {r['productivity_outcome']} | {r['actual_trigger_order']} | {r['shadow_trigger_order']} | {fmt(r['force_magnitude_at_productivity_trigger_n'])} | {fmt(r['productivity_eta_at_force_trigger'])} | {r['prefix_matched']} |")
    lines+=['', 'discordant_cases.csv records every discordance, both actual and shared-history trigger times, whether only one detector alerts, force/eta and actual/commanded progress rates at each actual trigger, outcomes, privileged diagnostic load and strict match errors. States after the first intervention can diverge; their later trigger times are descriptive, not equivalent-state causal evidence.',
        '', f"A: {len({r['case_id'] for r in a})} nominal trajectories had a safe productivity alarm with force ≤ the selected threshold and either meaningful low eta or terminal stagnation. Among these, {len(a_missed)} nominal failures had no safe force alarm anywhere. A force-below-threshold sample alone is not counted as a miss if force alerted earlier. Endpoint eta is undefined (command progress is zero), so terminal cases are labelled separately rather than assigned eta=0.",
        '', f"B: {len({r['case_id'] for r in b})} nominal trajectories had a safe force alarm with eta ≥ {NORMAL_ETA}; {len(b_success)} of these subsequently succeeded naturally. Here healthy means above the frozen normal productivity-trigger threshold, not a guarantee of safe future progress. Elevated force with healthy current progress on a later-failed trajectory can be useful early warning, not an unnecessary intervention.",
        '', f"On nominal failures, shared-history alert overlap: both {categories['both']}, force only {categories['force_only']}, productivity only {categories['productivity_only']}, neither {categories['neither']}. This measures potential complementarity, not hybrid controller performance. Neither threshold nor hybrid logic was tuned on the held-out data.",
        '',f"Similar timing/decisions: {sum(r['shadow_trigger_order']=='same_time' for r in pairs)} conditions with identical first safe shadow alert times; {sum(r['shadow_trigger_order']=='neither' for r in pairs)} with neither detector alerting. Same final success status: {sum(r['success_delta']==0 for r in pairs)}/64.",
        '', '## Answers to the scientific questions','',
        f"1. With recovery identical, observed productivity success was {prod['successes']}/64 versus force {force['successes']}/64, a {100*primary['paired_success_difference']:+.1f} percentage-point difference (exact paired p={primary['exact_mcnemar_p']:.6g}). The strict matched subset contains {validation['strict_preintervention_matches']} conditions; its separate paired table is essential. Unmatched outcome differences alone do not establish detector superiority.",
        f"2. Force missed all safe alarms on {len(a_missed)} failed nominal trajectories exhibiting low-force low-eta or terminal-stagnation productivity alarms. See the A examples and whether force alerted earlier or later.",
        f"3. Force alerted on {force['successes_alerted']}/{force['nominal_successes']} naturally successful nominal trajectories; {len(b_success)} of those have a recorded high-force/healthy-eta example. Productivity alerted on {prod['successes_alerted']}/{prod['nominal_successes']} natural successes.",
        f"4. Unique failed-case coverage was force {categories['force_only']} versus productivity {categories['productivity_only']}. Complementarity is supported only to that extent; no hybrid intervention outcome was measured.",
        '5. This ablation estimates the effect of changing the detector while holding recovery fixed. Improvements of either active policy over nominal measure that complete detector-plus-recovery system. It cannot separately quantify how much the earlier fixed-retract versus de-wedging action contributed; doing so would require a matched recovery-action ablation. No recovery advantage is attributed to the detector here.',
        '', '## Remaining failures','', '| Family | Force failure outcomes | Productivity failure outcomes |','|---|---|---|']
    for family in sorted({r['family'] for r in pairs}):
        a=dict(Counter(r['outcome'] for r in failures if r['family']==family and r['policy']=='force'))
        b=dict(Counter(r['outcome'] for r in failures if r['family']==family and r['policy']=='productivity'))
        lines.append(f'| {family} | {a} | {b} |')
    lines+=['', 'Every safety failure and timeout remains in the primary denominator. Strict matching requires every common-prefix sample through the earlier first intervention or episode end to pass unchanged FORGE pose, joints, velocities, wrench, grasp, finger and contact-state tolerances. Privileged load is used only in this offline matching/diagnostic audit. The matched subset is selective and can be smaller; it never replaces the full-set results. One execution per condition/policy is not a repeatability study.',
        '', '## Development lock, fairness and provenance','',
        f"Force threshold was selected solely from 32 new nominal development histories; all five candidates also received 32 closed-loop runs for reporting. False-alert cap met: {config['false_alert_cap_met']}. Selection details, all candidates, lead times and closed-loop outcomes are in [force_detector_dev_report.md](force_detector_dev_report.md). Configuration frozen at {config['frozen_at_utc']}, SHA256 `{validation['force_detector_config_sha256']}`.",
        '', plan['force_eligibility']+' '+plan['force_decision'],
        '', 'Force eligibility uses the same measured contact-onset gate and full causal history convention as productivity. It allows both insertion and endpoint hold, and does not require positive commanded progress or a low depth, so it is not artificially prevented from detecting terminal contact. Pose is common eligibility metadata, not a predictive force feature. Eta logged with force events is diagnostic only. The frozen productivity-v2 normal/urgent/terminal branches and all their eligibility rules are unchanged.',
        '', 'The full development and held-out condition plan and complete collection/analysis sources were frozen before collection. The sets contain 32 and 64 mutually disjoint paths, with zero exact overlap against all earlier FORGE paths, the 80-condition v2 development set and the previous 48- and 64-condition benchmarks. Previous parameter descriptors were used only to exclude duplicates; no old outcome selected a force threshold. Easy controls are fresh nearly aligned paths because exact centered paths already exist.',
        '', 'Native FORGE physics remains 120 Hz. Same assets, contact settings, gains, reset seed, original solver priming, safety/grasp limits, success/stall definitions, 8 s insertion command and trajectory generator. No clearance changes. Policy order and conditions are predeclared; no outcome-driven exclusions, replacements, extra repeats, early stopping or test tuning. Force-rise and hybrid were omitted before collection.',
        '',f"Validation passed: {validation['physics_rows']} physics rows, {validation['causal_checks']} checks, {validation['interventions']} recovery attempts. Original recovery commands and hard-safety priority were replayed, with independent force/v2 decision reconstruction. {validation['source_archives_verified']} source archives verified; {validation['preexisting_outputs_checked']} prior files unchanged. validation.json and provenance_audit.json retain the hashes, configuration lock and environment evidence.",
        '', 'Artifacts: summary.csv (192 held-out episodes), development_summary.csv (192 development episodes), policy_summary.csv, detector_comparison.csv, shadow_detector_analysis.csv, paired_comparisons.csv, paired_statistics.csv, discordant_cases.csv, recovery_events.csv, failure_cases.csv, mechanism_examples.csv, force_detector_config.json and its .sha256 file, condition_plan.json, per-run CSV/events/metrics and 64 condition plots.',
        '', 'Command: `./force_productivity_dewedge.sh --headless`. Existing output directories and selected configurations are never overwritten. Offline analysis: `python -m research.force_productivity_report outputs/Force-vs-Productivity-Dewedge-v1`.',
        '', '![Policies](policy_comparison.png)','', '![Shared-history detector quality](shadow_comparison.png)','', '![Force and productivity at shadow triggers](force_productivity_triggers.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory),indent=2))
