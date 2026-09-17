"""Predeclared final held-out analysis. No threshold selection or calibration."""
import argparse
from collections import Counter
import csv
import json
import math
from pathlib import Path
import numpy as np
from research.productivity_control_report import read_log
from research.productivity_dewedge_report import event_features
from research.productivity_detector_v2_report import audit_run, paired_prefix as endpoint_prefix
from research.productivity_detector_v2_calibration import nominal_checks, replay, first_safe
from research.productivity_detector_v2_generalization import verify, CONFIG_SHA
from research.productivity_generalization import sha
from research.productivity_unloading_report import safe
from research.forge_protocol import ForgeProtocol, match
from research.future_stall import table


def write_table(path,rows,empty_fields):
    if rows:table(path,rows)
    else:
        with path.open('w',newline='') as f:csv.writer(f).writerow(empty_fields)


def paired_prefix(a,b,p):
    result=endpoint_prefix(a,b,p)
    end=min(next((i for i,r in enumerate(rows) if r['intervention_started']),len(rows)-1) for rows in (a,b))
    matches=[match(x,y,p)[0] for x,y in zip(a[:end+1],b[:end+1])]
    result.update(endpoint_prefix_matched=result['prefix_matched'],prefix_matched=all(matches),
        prefix_samples=len(matches),prefix_mismatched_samples=sum(not v for v in matches))
    return result


def exact_mcnemar(wins,losses):
    if min(wins,losses)<0:raise ValueError('Negative counts')
    n=wins+losses
    return min(1.,2.*sum(math.comb(n,k) for k in range(min(wins,losses)+1))/2**n) if n else 1.


def selected(pairs,subset):
    return [r for r in pairs if (('severe' not in subset or r['severity']=='severe') and
        ('strict_matched' not in subset or r['prefix_matched']))]


def paired_statistics(pairs,subset='all'):
    rr=selected(pairs,subset);wins=sum(r['success_delta']==1 for r in rr);losses=sum(r['success_delta']==-1 for r in rr)
    n=len(rr);ci=(None,None)
    if n:
        rng=np.random.default_rng(20260918);boot=np.zeros(10000)
        for family in sorted({r['family'] for r in rr}):
            a=np.array([r['success_delta'] for r in rr if r['family']==family])
            boot+=rng.choice(a,size=(10000,len(a)),replace=True).sum(axis=1)
        ci=tuple(float(x) for x in np.percentile(boot/n,[2.5,97.5]))
    return dict(subset=subset,conditions=n,v2_wins=wins,v2_losses=losses,
        both_succeed=sum(r['v1_success'] and r['v2_success'] for r in rr),
        both_fail=sum(not r['v1_success'] and not r['v2_success'] for r in rr),
        success_rate_difference=(wins-losses)/n if n else None,ci95_low=ci[0],ci95_high=ci[1],
        exact_mcnemar_p=exact_mcnemar(wins,losses) if n else None)


def policy_totals(summaries,triggers,false,pairs):
    totals=[]
    for subset in ('all','severe','strict_matched','severe_strict_matched'):
        ids={r['case_id'] for r in selected(pairs,subset)}
        for policy in ('nominal','v1','v2'):
            rr=[r for r in summaries if r['policy']==policy and r['case_id'] in ids]
            tt=[r for r in triggers if r['detector']==policy and r['case_id'] in ids]
            ff=[r for r in false if r['detector']==policy and r['case_id'] in ids]
            failed=[r for r in tt if not r['nominal_success']];ss=[r for r in tt if r['nominal_safety_time_s'] is not None]
            leads=[r['safety_lead_s'] for r in ss if r['safety_lead_s'] is not None]
            stall_leads=[r['stall_lead_s'] for r in tt if r['stall_lead_s'] is not None]
            totals.append(dict(subset=subset,policy=policy,conditions=len(rr),successes=sum(r['insertion_success'] for r in rr),
                success_rate=sum(r['insertion_success'] for r in rr)/len(rr) if rr else None,
                stalled_runs=sum(r['stalled'] for r in rr),safety_failures=sum(r['safety_stop'] for r in rr),
                safety_during_recovery=sum(r['safety_during_recovery'] for r in rr),
                safety_before_any_recovery=sum(r['safety_before_any_recovery'] for r in rr),
                terminal_timeouts=sum(r['terminal_timeout'] for r in rr),intervened_runs=sum(r['intervention_count']>0 for r in rr),
                recoveries=sum(r['intervention_count'] for r in rr),verified_unloadings=sum(r['verified_unloadings'] for r in rr),
                ready_unloadings=sum(r['ready_unloadings'] for r in rr),successful_retries=sum(r['successful_retries'] for r in rr),
                repeated_stalls=sum(r['repeated_stalls'] for r in rr),
                normal_interventions=sum(r['normal_interventions'] for r in rr),urgent_interventions=sum(r['urgent_interventions'] for r in rr),terminal_interventions=sum(r['terminal_interventions'] for r in rr),
                nominal_failures=len(failed),nominal_failures_alerted=sum(r['shadow_triggered'] for r in failed),
                nominal_safety_failures=len(ss),nominal_safety_alerted=sum(r['shadow_triggered'] for r in ss),
                nominal_safety_alerted_at_least_0_1s_early=sum(r['safety_lead_s'] is not None and r['safety_lead_s']>=.1-1e-8 for r in ss),
                median_safety_lead_s=float(np.median(leads)) if leads else None,
                median_stall_lead_s=float(np.median(stall_leads)) if stall_leads else None,
                nominal_successes_for_false_alert=len(ff),false_alert_trajectories=sum(r['shadow_alert_on_success'] for r in ff),
                interventions_on_nominal_success=sum(r['actual_intervention_on_nominal_success'] for r in ff),
                matched_nominal_success_denominator=sum(r['prefix_matched'] for r in ff),
                matched_interventions_on_nominal_success=sum(r['actual_intervention_on_nominal_success'] and r['prefix_matched'] for r in ff)))
    return totals


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory).resolve();root=Path(__file__).resolve().parents[1]
    m=json.loads((directory/'experiment.json').read_text());plan=json.loads((directory/'condition_plan.json').read_text())
    config=json.loads((directory/'detector_v2_config.json').read_text());verify(root,plan)
    assert m['status']=='complete' and len(m['runs'])==192
    assert [{k:r[k] for k in ('case_id','policy','policy_order','repeat','stage')} for r in m['runs']]==plan['schedule']
    assert sha(directory/'detector_v2_config.json')==m['detector_v2_config_sha256']==CONFIG_SHA
    assert sha(directory/'condition_plan.json')==m['condition_plan_sha256']
    assert all(sha(root/f)==h and sha(directory/'source'/f)==h for f,h in m['source_sha256'].items())
    assert m['exact_previous_scene_and_config_match'] and not m['preexisting_outputs_changed']
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz']
    summaries=[];triggers=[];false=[];events=[];pairs=[];audited=Counter()
    histories=directory/'conditions';histories.mkdir(exist_ok=True)
    for c in plan['cases']:
        runs={r['policy']:r for r in m['runs'] if r['case_id']==c['case_id']};assert set(runs)=={'nominal','v1','v2'}
        raw={policy:read_log(directory/r['log']) for policy,r in runs.items()}
        checks=nominal_checks(raw['nominal'],c['case'],hz,p)
        reference=runs['nominal'];endpoint=raw['nominal'][-1]['time_s'];safety_time=endpoint if not safe(raw['nominal'][-1],p) else None
        opportunity=next((r['time_s'] for r in checks if r['signal'] and r['signal']['terminal_rate_mm_s'] is not None
            and r['signal']['terminal_rate_mm_s']<=config['terminal_rate_mm_s']),None)
        for policy in ('v1','v2'):
            alarms=replay(checks,config if policy=='v2' else None);first=first_safe(alarms)
            branch=next((a['branch'] for a in alarms if a['safe']),'')
            starts=[r for r in raw[policy] if r['intervention_started']]
            pair=paired_prefix(raw['nominal'],raw[policy],p)
            tr=dict(case_id=c['case_id'],family=c['family'],split=c['split'],group_id=c['group_id'],severity=c['severity'],detector=policy,
                nominal_success=reference['insertion_success'],nominal_outcome=reference['outcome'],nominal_stalled=reference['stalled'],
                nominal_safety_time_s=safety_time,nominal_first_stall_s=reference['first_stall_s'],
                first_safe_shadow_trigger_s=first,shadow_trigger_branch=branch,shadow_triggered=first is not None,
                safety_lead_s=safety_time-first if safety_time is not None and first is not None else None,
                stall_lead_s=reference['first_stall_s']-first if reference['first_stall_s'] is not None and first is not None else None,
                terminal_opportunity_s=opportunity,terminal_coverage=opportunity is not None and first is not None,
                terminal_trigger_delay_s=first-opportunity if opportunity is not None and first is not None else None,
                first_actual_intervention_s=starts[0]['time_s'] if starts else None,
                actual_trigger_branch=(starts[0]['trigger_branch'] if policy=='v2' else 'normal') if starts else '',
                nominal_safety_lead_of_actual_intervention_s=safety_time-starts[0]['time_s'] if safety_time is not None and starts else None,
                actual_interventions=len(starts),**pair)
            triggers.append(tr)
            if reference['insertion_success']:
                false.append(dict(case_id=c['case_id'],family=c['family'],group_id=c['group_id'],split=c['split'],severity=c['severity'],detector=policy,
                    shadow_alert_on_success=first is not None,first_shadow_alert_s=first,shadow_branch=branch,
                    actual_intervention_on_nominal_success=bool(starts),actual_interventions=len(starts),
                    controlled_success=runs[policy]['insertion_success'],controlled_safety_stop=not safe(raw[policy][-1],p),**pair))
        pair=paired_prefix(raw['v1'],raw['v2'],p)
        v1_starts=[r for r in raw['v1'] if r['intervention_started']]
        v2_starts=[r for r in raw['v2'] if r['intervention_started']]
        pairs.append(dict(case_id=c['case_id'],family=c['family'],split=c['split'],
            severity=c['severity'],nominal_success=reference['insertion_success'],nominal_outcome=reference['outcome'],
            v1_minus_nominal_success=int(runs['v1']['insertion_success'])-int(reference['insertion_success']),
            v2_minus_nominal_success=int(runs['v2']['insertion_success'])-int(reference['insertion_success']),
            v1_success=runs['v1']['insertion_success'],v2_success=runs['v2']['insertion_success'],
            v1_outcome=runs['v1']['outcome'],v2_outcome=runs['v2']['outcome'],
            success_delta=int(runs['v2']['insertion_success'])-int(runs['v1']['insertion_success']),
            neither_intervenes=not v1_starts and not v2_starts,
            v2_new_branch_used=any(r['trigger_branch'] in ('urgent','terminal') for r in v2_starts),
            v2_trigger_branches=','.join(r['trigger_branch'] for r in v2_starts),
            v1_first_trigger_s=v1_starts[0]['time_s'] if v1_starts else None,
            v2_first_trigger_s=v2_starts[0]['time_s'] if v2_starts else None,
            first_trigger_time_equal=abs(v1_starts[0]['time_s']-v2_starts[0]['time_s'])<1e-6 if v1_starts and v2_starts else None,**pair))
        for policy,run in runs.items():
            rows=raw[policy];a=audit_run(directory,m,run,rows,config)
            for k in ('physics_rows','causal_checks','interventions','retry_ready_unloadings'):audited[k]+=a[k]
            assert run['rows']==len(rows)
            for metric,column in (('max_wrist_force_n','wrist_force_n'),('max_wrist_torque_nm','wrist_torque_nm'),('max_normal_load_n','normal_load_n')):
                assert abs(run[metric]-max(r[column] for r in rows))<1e-8
            assert abs(run['final_depth_mm']-rows[-1]['depth_mm'])<1e-8
            if run['insertion_success']:
                assert all(safe(r,p) and r['depth_mm']>=p.success_depth_mm and r['phase'] in ('insert','hold') for r in rows[-round(p.success_dwell_s*hz):])
            ee=json.loads((directory/run['run_id']/'events.json').read_text());ff=[]
            for i,e in enumerate(ee):
                f=event_features(rows,e,ee[i+1] if i+1<len(ee) else None,p,hz)
                trigger_row=rows[round(e['trigger_time_s']*hz)]
                f.update(case_id=c['case_id'],split=c['split'],policy=policy,trigger_branch=trigger_row.get('trigger_branch','normal'))
                events.append(f);ff.append(f)
            starts=[r for r in rows if r['intervention_started']]
            summaries.append(dict(**run,safety_stop=not safe(rows[-1],p),terminal_timeout=run['outcome']=='time_budget_exhausted' and rows[-1]['phase']=='hold',
                terminal_phase=rows[-1]['phase'],terminal_grasp_slip_mm=rows[-1]['grasp_slip_mm'],
                safety_during_recovery=not safe(rows[-1],p) and rows[-1]['phase'] in ('stop','relax','unload_retract','unload_hold','post_verify_hold'),
                safety_before_any_recovery=not safe(rows[-1],p) and not starts,
                last_trigger_branch=(starts[-1].get('trigger_branch','normal') if starts else None),
                last_trigger_grasp_slip_mm=starts[-1]['grasp_slip_mm'] if starts else None,
                verified_unloadings=sum(e['online_verified'] for e in ff),ready_unloadings=sum(e['ready_to_retry'] for e in ff),
                successful_retries=sum(e['retry_success'] is True for e in ff),repeated_stalls=sum(e['stall_after_retry'] is True for e in ff),
                normal_interventions=sum(r.get('trigger_branch','normal')=='normal' for r in starts),
                urgent_interventions=sum(r.get('trigger_branch')=='urgent' for r in starts),
                terminal_interventions=sum(r.get('trigger_branch')=='terminal' for r in starts),
                first_intervention_s=starts[0]['time_s'] if starts else None))
        fig,axes=plt.subplots(2,1,figsize=(9,6),sharex=True)
        for policy,color in (('nominal','#666666'),('v1','#9a6b30'),('v2','#008c91')):
            rows=raw[policy];t=[r['time_s']-2 for r in rows]
            axes[0].plot(t,[r['depth_mm'] for r in rows],label=policy,color=color)
            axes[1].plot(t,[r['grasp_slip_mm'] for r in rows],label=policy,color=color)
            for r in rows:
                if r['intervention_started']:axes[0].scatter(r['time_s']-2,r['depth_mm'],color=color,marker='x')
        axes[0].axhline(p.success_depth_mm,color='black',ls=':');axes[0].set(ylabel='Actual depth (mm)',title=c['case_id']+' / '+c['split'])
        axes[1].axhline(.1,color='black',ls=':');axes[1].set(ylabel='Grasp slip (mm)',xlabel='Insertion elapsed time (s)')
        for ax in axes:ax.grid(alpha=.2);ax.legend()
        fig.tight_layout();fig.savefig(histories/(c['case_id']+'.png'),dpi=110);plt.close(fig)
    for name,data in (('summary',summaries),('trigger_analysis',triggers),('false_trigger_analysis',false),('recovery_events',events),('paired_comparisons',pairs)):
        write_table(directory/(name+'.csv'),data,('case_id','policy'))
    totals=policy_totals(summaries,triggers,false,pairs)
    table(directory/'policy_summary.csv',totals)
    failures=[r for r in summaries if not r['insertion_success']]
    write_table(directory/'failure_cases.csv',failures,('case_id','policy','outcome'))
    branches=[]
    for policy in ('v1','v2'):
        for branch in ('normal','urgent','terminal'):
            rr=[e for e in events if e['policy']==policy and e['trigger_branch']==branch]
            branches.append(dict(policy=policy,branch=branch,recoveries=len(rr),
                conditions=len({e['case_id'] for e in rr}),verified_unloadings=sum(e['online_verified'] for e in rr),
                ready_unloadings=sum(e['ready_to_retry'] for e in rr),retry_successes=sum(e['retry_success'] is True for e in rr),
                repeated_stalls=sum(e['stall_after_retry'] is True for e in rr)))
    table(directory/'branch_analysis.csv',branches)
    stats=[paired_statistics(pairs,subset) for subset in ('all','severe','strict_matched','severe_strict_matched')]
    table(directory/'paired_statistics.csv',stats)
    validation=dict(status='passed',conditions=64,runs=192,**audited,
        normal_branch_unchanged=True,same_recovery_code_object=True,full_recovery_and_safety_replay=True,
        all_features_causal=True,normal_load_never_used_by_detector=True,no_evaluation_tuning=True,
        configuration_copied_byte_for_byte=True,detector_v2_config_sha256=CONFIG_SHA,
        exact_previous_path_overlaps=0,condition_plan_sha256=m['condition_plan_sha256'],
        source_archives_verified=len(m['source_sha256']),all_sources_unchanged_through_analysis=True,
        scene_config_unchanged=True,preexisting_outputs_checked=m['preexisting_outputs_checked'],
        preexisting_outputs_changed=m['preexisting_outputs_changed'],
        strict_v1_v2_matches=sum(r['prefix_matched'] for r in pairs),
        strict_match_definition=plan['analysis']['strict_match'],paired_statistics=stats)
    (directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    report(directory,totals,config,pairs,failures,validation,events,stats,branches,plan)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    allrows=[r for r in totals if r['subset']=='all']
    for ax,key,label in zip(axes,('successes','safety_failures','terminal_timeouts'),('Successes / 64','Safety failures / 64','Terminal timeouts / 64')):
        vals=[r[key] for r in allrows];ax.bar(['nominal','v1','v2'],vals,color=['#666666','#9a6b30','#008c91']);ax.set_title(label)
        for i,v in enumerate(vals):ax.text(i,v+.3,str(v),ha='center')
        ax.set_ylim(0,68);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'policy_comparison.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for i,subset in enumerate(('all','severe')):
        for j,policy in enumerate(('v1','v2')):
            r=next(r for r in totals if r['subset']==subset and r['policy']==policy)
            axes[0].bar(i+(j-.5)*.3,r['nominal_failures_alerted']/max(1,r['nominal_failures']),width=.3,color=('#9a6b30','#008c91')[j],label=policy if i==0 else None)
            axes[1].bar(i+(j-.5)*.3,r['false_alert_trajectories']/max(1,r['nominal_successes_for_false_alert']),width=.3,color=('#9a6b30','#008c91')[j],label=policy if i==0 else None)
    for ax,title in zip(axes,('Safe alert coverage of nominal failures','Alerts on successful nominal trajectories')):
        ax.set(xticks=[0,1],xticklabels=['All held out','Severe held out'],ylim=(0,1.05),title=title);ax.legend();ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'coverage_false_alerts.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    axes[0].bar(['V2 win','V2 loss','Both succeed','Both fail'],[stats[0][k] for k in ('v2_wins','v2_losses','both_succeed','both_fail')])
    axes[0].set_title('Paired success outcomes / 64')
    for i,policy in enumerate(('v1','v2')):
        vals=[next(r['recoveries'] for r in branches if r['policy']==policy and r['branch']==b) for b in ('normal','urgent','terminal')]
        axes[1].bar(np.arange(3)+(i-.5)*.3,vals,width=.3,label=policy)
    axes[1].set(xticks=range(3),xticklabels=['normal','urgent','terminal'],title='Recovery branch usage');axes[1].legend()
    fig.tight_layout();fig.savefig(directory/'paired_branches.png',dpi=140);plt.close(fig)
    artifacts=[directory/r['run_id']/name for r in m['runs'] for name in ('trajectory.csv','events.json','metrics.json')]
    artifacts += [directory/name for name in ('condition_plan.json','detector_v2_config.json','calibration.json','scene.json','config.json','experiment.json')]
    provenance=dict(frozen_at_utc=plan['frozen_at_utc'],collection_started_utc=m['collection_started_utc'],
        command='./productivity_detector_v2_generalization.sh --headless',
        policy_code_and_analysis_sha256=m['source_sha256'],condition_plan_sha256=m['condition_plan_sha256'],
        detector_v2_config_sha256=CONFIG_SHA,prior_parameter_only_overlap_audit=plan['holdout_audit'],
        raw_artifact_sha256={str(f.relative_to(directory)):sha(f) for f in artifacts},
        elapsed_seconds=m['elapsed_seconds'],environment=m['environment'],
        preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=[],
        validation='Every logged detector decision, hard-safety priority and recovery command replayed. Full experiment sources verified before each run and after analysis.')
    (directory/'provenance_audit.json').write_text(json.dumps(provenance,indent=2)+'\n')
    verify(root,plan)
    old=json.loads((directory/'preexisting_outputs.json').read_text())
    assert all(Path(f).is_file() and [Path(f).stat().st_size,Path(f).stat().st_mtime_ns]==st for f,st in old.items())
    return totals


def report(directory,totals,config,pairs,failures,audit,events,stats,branches,plan):
    allrows={r['policy']:r for r in totals if r['subset']=='all'};v1=allrows['v1'];v2=allrows['v2']
    lines=['# Frozen Detector v2 held-out evaluation','',
        '64 fresh conditions × three frozen policies = 192 episodes. Every condition and the complete collection/analysis implementation were frozen before the first episode. No calibration, tuning, outcome-driven exclusions, replacement conditions, or added repeats were performed.',
        '', '| Subset | Policy | Success | Safety stops | Terminal timeouts | Before-recovery safety stops |', '|---|---|---:|---:|---:|---:|']
    for r in totals:
        lines.append(f"| {r['subset']} | {r['policy']} | {r['successes']}/{r['conditions']} | {r['safety_failures']} | {r['terminal_timeouts']} | {r['safety_before_any_recovery']} |")
    lines+=['','## Paired v1 versus v2','',
        '| Subset | N | V2 wins | V2 losses | Both succeed | Both fail | Success difference [95% CI] | Exact paired p |',
        '|---|---:|---:|---:|---:|---:|---|---:|']
    for s in stats:
        diff=(f"{100*s['success_rate_difference']:+.1f} pp [{100*s['ci95_low']:+.1f}, {100*s['ci95_high']:+.1f}]" if s['conditions'] else 'N/A')
        pv=f"{s['exact_mcnemar_p']:.6g}" if s['conditions'] else 'N/A'
        lines.append(f"| {s['subset']} | {s['conditions']} | {s['v2_wins']} | {s['v2_losses']} | {s['both_succeed']} | {s['both_fail']} | {diff} | {pv} |")
    lines+=['', 'Primary test: two-sided exact conditional McNemar (binomial among discordant pairs). CI: 10,000 paired condition bootstraps stratified by predeclared family, fixed seed 20260918. With zero discordances, exact p=1; a degenerate bootstrap interval does not establish equivalence. Structured conditions are not a random population sample. Severe and matched subsets are descriptive sensitivity analyses, not additional confirmatory tests.',
        '',f"Overall observed v2 minus v1 success: {v2['successes']-v1['successes']:+d}/64. Safety stops: {v1['safety_failures']} → {v2['safety_failures']}; terminal timeouts: {v1['terminal_timeouts']} → {v2['terminal_timeouts']}; safety stops before any recovery: {v1['safety_before_any_recovery']} → {v2['safety_before_any_recovery']}.",
        '', '## Recovery and branch accounting','',
        '| Policy | Stalled runs | Runs intervened | Recoveries | Verified | Ready to retry | Retry successes | Repeated stalls | Recovery safety stops |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|']
    for r in allrows.values():
        lines.append(f"| {r['policy']} | {r['stalled_runs']} | {r['intervened_runs']} | {r['recoveries']} | {r['verified_unloadings']} | {r['ready_unloadings']} | {r['successful_retries']} | {r['repeated_stalls']} | {r['safety_during_recovery']} |")
    lines+=['','| Policy | Branch | Attempts | Verified | Ready | Retry successes | Repeated stalls |','|---|---|---:|---:|---:|---:|---:|']
    for r in branches:
        lines.append(f"| {r['policy']} | {r['branch']} | {r['recoveries']} | {r['verified_unloadings']} | {r['ready_unloadings']} | {r['retry_successes']} | {r['repeated_stalls']} |")
    lines+=['', 'Verified means actual retreat ≥0.5 mm continuously for 0.25 s; ready additionally requires the unchanged post-verification hold. A later rebound or safety stop can prevent retry even after initial verification. Repeated stalls are stalls in a retry interval, not independent trials. Hard safety takes priority over soft detector alarms; branch counts include actions actually started, not ignored alarms.',
        '', '## Coverage, unnecessary interventions, and lead time','',
        '| Subset | Detector | Failed nominal cases alerted | Successful nominal cases alerted | Actual interventions on nominal successes | Safety cases alerted ≥0.1 s early | Median safety lead (s) | Median stall lead (s) |',
        '|---|---|---:|---:|---:|---:|---:|---:|']
    for r in totals:
        if r['policy']=='nominal':continue
        fmt=lambda x:'N/A' if x is None else f'{x:.3f}'
        lines.append(f"| {r['subset']} | {r['policy']} | {r['nominal_failures_alerted']}/{r['nominal_failures']} | {r['false_alert_trajectories']}/{r['nominal_successes_for_false_alert']} | {r['interventions_on_nominal_success']} | {r['nominal_safety_alerted_at_least_0_1s_early']}/{r['nominal_safety_failures']} | {fmt(r['median_safety_lead_s'])} | {fmt(r['median_stall_lead_s'])} |")
    lines+=['', 'Shadow detectors see identical fresh nominal histories without intervening; only safe alarm samples count. A shadow alert on a trajectory that subsequently succeeds is the conservative unnecessary-intervention label. Closed-loop interventions on paired nominal successes are a separate counterfactual proxy affected by reset differences. Strict nominal-versus-controller matching counts/denominators are retained in policy_summary.csv and false_trigger_analysis.csv.',
        '', 'Positive lead means the alarm preceded the nominal first stall or safety stop. Negative lead means it came after the first stall. Medians include only cases with both an event and an alarm; coverage denominators expose missed cases. A warning lead does not establish recoverability. Terminal endpoint opportunity and its trigger delay are reported separately in trigger_analysis.csv; time remaining to a timeout is not presented as early safety prediction.',
        '', '## Failure cases and attribution','',
        '| Family | V1 failures | V2 failures | V2 failure outcomes |','|---|---:|---:|---|']
    for family in sorted({r['family'] for r in pairs}):
        ff=[r for r in failures if r['family']==family]
        lines.append(f"| {family} | {sum(r['policy']=='v1' for r in ff)} | {sum(r['policy']=='v2' for r in ff)} | {dict(Counter(r['outcome'] for r in ff if r['policy']=='v2'))} |")
    lines+=['', '| Discordant condition | Severity | V1 outcome | V2 outcome | V2 branches | Strict match |', '|---|---|---|---|---|---|']
    for r in pairs:
        if r['success_delta']:
            lines.append(f"| [{r['case_id']}](conditions/{r['case_id']}.png) | {r['severity']} | {r['v1_outcome']} | {r['v2_outcome']} | {r['v2_trigger_branches'] or 'none'} | {r['prefix_matched']} |")
    lines+=['', f"Strict full-prefix v1/v2 matches: {sum(r['prefix_matched'] for r in pairs)}/64. Discordant outcomes without any added v2 branch: {sum(r['success_delta']!=0 and not r['v2_new_branch_used'] for r in pairs)}; discordances with neither policy intervening: {sum(r['success_delta']!=0 and r['neither_intervenes'] for r in pairs)}. These differences remain in primary denominators but cannot be credited to urgent/terminal logic.",
        '', 'Strict matching requires every common-prefix sample through the earlier first intervention (or earlier episode end) to pass the original FORGE pose/orientation, joints, velocities, wrench, load, hand/finger and grasp tolerances. Endpoint-only matching, mismatch counts, and depth RMSE are also retained. Matching is an offline audit; privileged normal load is never an online detector feature. The matched subset can be smaller and selectively easier and does not replace the full-set result. One execution per policy/condition estimates breadth, not repeatability.',
        '', '## Frozen protocol and provenance','',
        f"Plan frozen: {plan['frozen_at_utc']}. Detector-v2 config SHA256: `{CONFIG_SHA}` (byte-for-byte copy of the development-selected file). Normal eta < {config['normal_eta_threshold']} for two consecutive checks; urgent eta < 0.2 and deficit acceleration ≥2 mm/s²; terminal depth-range rate ≤0.01 mm/s over a full 0.5 s endpoint-hold history. All original eligibility guards remain.",
        '', 'Eight families × eight paths: easy controls, axis offset, oblique offset, axis tilt, oblique tilt, combined offset/tilt, terminal-band targets, and late-ramp combined targets. Severity is preassigned (8 easy, 28 moderate, 28 severe); family names express target inputs, not observed outcomes. Ramp onsets span 6.8–18.2 mm. Full ramp depth 20 mm, insertion command 8 s, native FORGE 120 Hz. Identical existing scene, contact, gains, hard safety, success/stall definitions and recovery budgets. No clearance/geometry changes.',
        '', 'All 64 complete commanded paths were checked against the 48-condition benchmark, all 80 development conditions, Phase 2A/2B and earlier pilot descriptors. Metadata and signed zeros are ignored in path fingerprints. Shared aligned approach prefixes are intentional; full paths are distinct. Only prior parameter descriptors are used for exclusion, never prior outcomes for fitting or selection. Source manifest hashes retain the exclusion provenance.',
        '', 'Conditions are randomly ordered with predeclared seed; policy order rotates so each policy occupies each position 21 or 22 times. Reset seed and original solver priming remain unchanged. '+plan['optional_baselines_reason'],
        '', f"Validation passed: {audit['physics_rows']} physics rows, {audit['causal_checks']} checks, {audit['interventions']} interventions replayed. All frozen sources, selected configuration and plan were checked before every episode and after analysis. {audit['preexisting_outputs_checked']} preexisting files checked for size/mtime changes: zero. Full run artifact SHA256 hashes and environment details are in provenance_audit.json.",
        '', 'Artifacts: summary.csv, policy_summary.csv, paired_comparisons.csv, paired_statistics.csv, branch_analysis.csv, failure_cases.csv, trigger_analysis.csv, false_trigger_analysis.csv, recovery_events.csv, condition_plan.json, detector_v2_config.json, validation.json, provenance_audit.json, per-condition plots, per-run trajectory/events/metrics, and complete source snapshots.',
        '', 'Collection command: `./productivity_detector_v2_generalization.sh --headless`. Existing output directories are refused. Analysis command: `python -m research.productivity_detector_v2_generalization_report outputs/Contact-Productivity-DetectorV2-Generalization-v1`.',
        '', '![Policies](policy_comparison.png)', '', '![Paired outcomes and branches](paired_branches.png)', '', '![Coverage and unnecessary alarms](coverage_false_alerts.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory),indent=2))
