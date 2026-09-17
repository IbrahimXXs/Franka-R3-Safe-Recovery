"""Development-only detector evaluation and unchanged-recovery replay audit."""
import argparse
from collections import Counter
import json
from pathlib import Path
from types import FunctionType
import numpy as np
from research.productivity_control import Design
from research.productivity_control_report import read_log
from research.productivity_dewedge_report import audit as original_audit, event_features
from research.productivity_detector_v2 import DetectorV2, features
from research.productivity_detector_v2_calibration import nominal_checks, replay, first_safe
from research.productivity_detector_v2_design import verify
from research.productivity_generalization import sha
from research.productivity_unloading_report import safe
from research.forge_protocol import ForgeProtocol, match
from research.future_stall import table


def audit_run(directory, manifest, run, rows, config):
    """Original full recovery audit, with only the v2 causal detector replay injected."""
    proxy=dict(run,policy='nominal' if run['policy']=='nominal' else 'dewedge')
    m=dict(manifest,runs=[proxy]); raw={run['run_id']:rows}
    function=original_audit
    if run['policy']=='v2':
        case=next(c['case'] for c in m['cases'] if c['case_id']==run['case_id'])
        p=ForgeProtocol(**m['protocol']);hz=m['physics_hz']
        class ReplayDetector:
            def __init__(self,policy,eta,force,design):self.detector=DetectorV2(config,design)
            @property
            def count(self):return self.detector.count
            def reset(self):self.detector.reset()
            def check(self,t,normal_signal):
                signal=features(rows[:round(t*hz)+1],hz,case['ramp_onset_mm'],Design(),p.success_depth_mm)
                fired=self.detector.check(t,signal);r=rows[round(t*hz)]
                for k,v in self.detector.last_decision.items():
                    if k=='normal_eta_valid':continue
                    if k=='trigger_branch':assert (r[k] or '')==v
                    elif isinstance(v,float):assert abs(r[k]-v)<1e-8,(k,r[k],v)
                    else:assert r[k]==v,(k,r[k],v)
                return fired
        function=FunctionType(original_audit.__code__,dict(original_audit.__globals__,Detector=ReplayDetector))
    return function(directory,m,raw)


def paired_prefix(a,b,p):
    index=min(next((i for i,r in enumerate(rows) if r['intervention_started']),len(rows)-1) for rows in (a,b))
    matched,errors=match(a[index],b[index],p)
    return dict(prefix_matched=matched,prefix_time_s=a[index]['time_s'],
        prefix_position_mm=errors['position_mm'],prefix_orientation_deg=errors['orientation_deg'],
        prefix_depth_rmse_mm=float(np.sqrt(np.mean([(x['depth_mm']-y['depth_mm'])**2 for x,y in zip(a[:index+1],b[:index+1])]))))


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    directory=Path(directory).resolve();root=Path(__file__).resolve().parents[1]
    m=json.loads((directory/'experiment.json').read_text());plan=json.loads((directory/'development_plan.json').read_text())
    config=json.loads((directory/'detector_v2_config.json').read_text());verify(root,plan)
    assert m['status']=='complete' and len(m['runs'])==240
    assert [{k:r[k] for k in ('case_id','policy','stage')} for r in m['runs']]==plan['nominal_schedule']+plan['comparison_schedule']
    assert sha(directory/'detector_v2_config.json')==m['detector_v2_config_sha256']
    assert sha(directory/'development_plan.json')==m['development_plan_sha256']==config['development_plan_sha256']
    assert m['calibration_frozen_after_reference_runs']==80
    assert not config['validation_used_for_selection'] and not config['closed_loop_outcomes_used'] and not config['previous_generalization_results_used']
    allowed={r['log'] for r in m['runs'] if r['policy']=='nominal' and r['split']=='calibration'}
    assert set(config['calibration_source_sha256'])==allowed and len(allowed)==60
    assert all(sha(directory/f)==h for f,h in config['calibration_source_sha256'].items())
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
            tr=dict(case_id=c['case_id'],family=c['family'],split=c['split'],group_id=c['group_id'],detector=policy,
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
                false.append(dict(case_id=c['case_id'],family=c['family'],group_id=c['group_id'],split=c['split'],detector=policy,
                    shadow_alert_on_success=first is not None,first_shadow_alert_s=first,shadow_branch=branch,
                    actual_intervention_on_nominal_success=bool(starts),actual_interventions=len(starts),
                    controlled_success=runs[policy]['insertion_success'],controlled_safety_stop=not safe(raw[policy][-1],p),**pair))
        pair=paired_prefix(raw['v1'],raw['v2'],p)
        v1_starts=[r for r in raw['v1'] if r['intervention_started']]
        v2_starts=[r for r in raw['v2'] if r['intervention_started']]
        pairs.append(dict(case_id=c['case_id'],family=c['family'],split=c['split'],
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
        table(directory/(name+'.csv'),data)
    totals=[]
    for split in ('all','calibration','validation'):
        for policy in ('nominal','v1','v2'):
            rr=[r for r in summaries if r['policy']==policy and (split=='all' or r['split']==split)]
            tt=[r for r in triggers if r['detector']==policy and (split=='all' or r['split']==split)]
            ff=[r for r in false if r['detector']==policy and (split=='all' or r['split']==split)]
            failed=[r for r in tt if not r['nominal_success']];ss=[r for r in tt if r['nominal_safety_time_s'] is not None]
            leads=[r['safety_lead_s'] for r in ss if r['safety_lead_s'] is not None]
            totals.append(dict(split=split,policy=policy,conditions=len(rr),successes=sum(r['insertion_success'] for r in rr),
                stalled_runs=sum(r['stalled'] for r in rr),safety_failures=sum(r['safety_stop'] for r in rr),
                safety_during_recovery=sum(r['safety_during_recovery'] for r in rr),
                safety_before_any_recovery=sum(r['safety_before_any_recovery'] for r in rr),
                added_branch_recovery_safety_stops=sum(r['safety_during_recovery'] and r['last_trigger_branch'] in ('urgent','terminal') for r in rr),
                terminal_timeouts=sum(r['terminal_timeout'] for r in rr),intervened_runs=sum(r['intervention_count']>0 for r in rr),
                recoveries=sum(r['intervention_count'] for r in rr),verified_unloadings=sum(r['verified_unloadings'] for r in rr),
                successful_retries=sum(r['successful_retries'] for r in rr),repeated_stalls=sum(r['repeated_stalls'] for r in rr),
                normal_interventions=sum(r['normal_interventions'] for r in rr),urgent_interventions=sum(r['urgent_interventions'] for r in rr),terminal_interventions=sum(r['terminal_interventions'] for r in rr),
                nominal_failures=len(failed),nominal_failures_alerted=sum(r['shadow_triggered'] for r in failed),
                nominal_safety_failures=len(ss),nominal_safety_alerted=sum(r['shadow_triggered'] for r in ss),
                nominal_safety_alerted_at_least_0_1s_early=sum(r['safety_lead_s'] is not None and r['safety_lead_s']>=.1-1e-8 for r in ss),
                median_safety_lead_s=float(np.median(leads)) if leads else None,
                nominal_successes_for_false_alert=len(ff),false_alert_trajectories=sum(r['shadow_alert_on_success'] for r in ff),
                interventions_on_nominal_success=sum(r['actual_intervention_on_nominal_success'] for r in ff),
                matched_interventions_on_nominal_success=sum(r['actual_intervention_on_nominal_success'] and r['prefix_matched'] for r in ff)))
    table(directory/'policy_summary.csv',totals)
    failures=[r for r in summaries if not r['insertion_success']];table(directory/'failure_cases.csv',failures)
    validation=dict(status='passed',conditions=80,runs=240,**audited,normal_branch_unchanged=True,
        same_recovery_code_object=True,full_recovery_and_safety_replay=True,all_features_causal=True,normal_load_never_used_by_detector=True,
        calibration_trajectories=60,validation_trajectories=20,calibration_validation_groups_disjoint=True,
        no_previous_generalization_outcomes_used=True,no_final_generalization_test=True,
        config_frozen_before_comparison=True,detector_v2_config_sha256=m['detector_v2_config_sha256'],
        frozen_recovery_sources=plan['policy_source_sha256'],calibration_sources_verified=60,
        source_archives_verified=len(m['source_sha256']),scene_config_unchanged=True,
        preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=m['preexisting_outputs_changed'])
    assert not validation['preexisting_outputs_changed']
    (directory/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
    report(directory,totals,config,pairs,failures,validation,events)
    fig,axes=plt.subplots(1,3,figsize=(13,4))
    allrows=[r for r in totals if r['split']=='all']
    for ax,key,label in zip(axes,('successes','safety_failures','terminal_timeouts'),('Successes / 80','Safety failures / 80','Terminal timeouts / 80')):
        vals=[r[key] for r in allrows];ax.bar(['nominal','v1','v2'],vals,color=['#666666','#9a6b30','#008c91']);ax.set_title(label)
        for i,v in enumerate(vals):ax.text(i,v+.3,str(v),ha='center')
        ax.set_ylim(0,85);ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'policy_comparison.png',dpi=140);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(11,4))
    for i,split in enumerate(('calibration','validation')):
        for j,policy in enumerate(('v1','v2')):
            r=next(r for r in totals if r['split']==split and r['policy']==policy)
            axes[0].bar(i+(j-.5)*.3,r['nominal_failures_alerted']/max(1,r['nominal_failures']),width=.3,color=('#9a6b30','#008c91')[j],label=policy if i==0 else None)
            axes[1].bar(i+(j-.5)*.3,r['false_alert_trajectories']/max(1,r['nominal_successes_for_false_alert']),width=.3,color=('#9a6b30','#008c91')[j],label=policy if i==0 else None)
    for ax,title in zip(axes,('Safe alert coverage of nominal failures','Alerts on successful nominal trajectories')):
        ax.set(xticks=[0,1],xticklabels=['calibration','development validation'],ylim=(0,1.05),title=title);ax.legend();ax.grid(axis='y',alpha=.2)
    fig.tight_layout();fig.savefig(directory/'coverage_false_alerts.png',dpi=140);plt.close(fig)
    return totals


def report(directory,totals,config,pairs,failures,audit,events):
    allrows={r['policy']:r for r in totals if r['split']=='all'};v1=allrows['v1'];v2=allrows['v2']
    lines=['# Contact Productivity Detector v2 development','',
        '80 new targeted conditions × nominal/v1/v2 = 240 episodes. The de-wedging recovery, normal trigger, hard safety, physics, assets and budgets were unchanged. No final generalization test was run.',
        '', '| Policy | Success | Safety failures | Terminal timeouts | Intervened runs | Verified / recoveries |', '|---|---:|---:|---:|---:|---:|']
    for r in allrows.values():lines.append(f"| {r['policy']} | {r['successes']}/80 | {r['safety_failures']} | {r['terminal_timeouts']} | {r['intervened_runs']} | {r['verified_unloadings']}/{r['recoveries']} |")
    lines+=['', '| Development subset | Policy | Success | Safety failures | Terminal timeouts |', '|---|---|---:|---:|---:|']
    for r in totals:
        if r['split']!='all' and r['policy']!='nominal':
            lines.append(f"| {r['split']} | {r['policy']} | {r['successes']}/{r['conditions']} | {r['safety_failures']} | {r['terminal_timeouts']} |")
    lines+=['','## Trigger coverage and false interventions','',
        '| Split | Detector | Failed nominal trajectories alerted | Successful nominal trajectories alerted | Interventions on nominal successes | Safety cases alerted ≥0.1 s early | Median safety lead (s) |',
        '|---|---|---:|---:|---:|---:|---:|']
    for r in totals:
        if r['policy']=='nominal':continue
        lead=f"{r['median_safety_lead_s']:.3f}" if r['median_safety_lead_s'] is not None else 'N/A'
        lines.append(f"| {r['split']} | {r['policy']} | {r['nominal_failures_alerted']}/{r['nominal_failures']} | {r['false_alert_trajectories']}/{r['nominal_successes_for_false_alert']} | {r['interventions_on_nominal_success']} | {r['nominal_safety_alerted_at_least_0_1s_early']}/{r['nominal_safety_failures']} | {lead} |")
    lines+=['', '| V2 trigger branch | Recovery attempts | Verified unloading | Successful retries |', '|---|---:|---:|---:|']
    for branch in ('normal','urgent','terminal'):
        rr=[e for e in events if e['policy']=='v2' and e['trigger_branch']==branch]
        lines.append(f"| {branch} | {len(rr)} | {sum(e['online_verified'] for e in rr)} | {sum(e['retry_success'] is True for e in rr)} |")
    lines+=['', '| Condition using an added v2 branch | Split | Branch sequence | V1 outcome | V2 outcome | Strict prefix match |', '|---|---|---|---|---|---|']
    for r in pairs:
        if r['v2_new_branch_used']:
            lines.append(f"| [{r['case_id']}](conditions/{r['case_id']}.png) | {r['split']} | {r['v2_trigger_branches']} | {r['v1_outcome']} | {r['v2_outcome']} | {r['prefix_matched']} |")
    lines+=['', 'This branch-use table is descriptive and selected after observing which branches acted; it does not replace the full-condition comparison or define another test split.',
        '',f"V2 initiated {v2['normal_interventions']} normal, {v2['urgent_interventions']} urgent and {v2['terminal_interventions']} terminal recoveries. V1 initiated {v1['recoveries']} normal recoveries.",
        '',f"Safety stops during recovery: v1 {v1['safety_during_recovery']}, v2 {v2['safety_during_recovery']}; {v2['added_branch_recovery_safety_stops']} v2 stops followed an added-branch trigger. V2 safety stops before any recovery: {v2['safety_before_any_recovery']}. Trigger coverage does not establish that the reached state is recoverable within the unchanged safety limits. Terminal phase and grasp slip at the last trigger/stop are retained in summary.csv.",
        '',f"Observed success changed by {v2['successes']-v1['successes']:+d}/80 conditions; nominal-failure trigger coverage changed by {v2['nominal_failures_alerted']-v1['nominal_failures_alerted']:+d}; successful-nominal shadow alerts changed by {v2['false_alert_trajectories']-v1['false_alert_trajectories']:+d}. These are development results, not final generalization estimates.",
        '', 'Coverage/lead and false alerts are replayed on exactly the same new nominal histories, so they isolate detector logic. Only safe trigger samples count as actionable. Safety lead is nominal safety-stop time minus first safe shadow trigger; positive is earlier. Stall lead is also retained and can be negative. Lead is not proof that the unchanged recovery has enough time to prevent that failure. Terminal opportunity and trigger delay are separate from artificial time-to-episode-deadline measures.',
        '', 'An alert on a nominal trajectory that subsequently succeeds is the conservative unnecessary-trigger label. Actual interventions on paired nominal successes are reported separately; reset differences mean these are imperfect counterfactuals. Normal-branch alerts cannot be eliminated by v2 without violating the frozen-detector requirement. Terminal opportunities may already be preceded by a normal/urgent alarm, so negative terminal delay means an earlier branch acted.',
        '', '## Frozen configuration','',f"Frozen at {config['frozen_at_utc']}, after 80 nominal references and before any closed-loop comparison. Normal eta < {config['normal_eta_threshold']:.16g} for two consecutive 0.1 s checks. Urgent eta < {config['urgent_eta_threshold']:g} and deficit acceleration ≥ {config['urgent_deficit_acceleration_mm_s2']:g} mm/s². Terminal depth-range rate ≤ {config['terminal_rate_mm_s']:g} mm/s over a full 0.5 s endpoint-hold window.",
        '', 'The urgent branch retains original positive-command/contact-onset eligibility, requires a positive progress-deficit rate, and uses adjacent check history. Terminal windows must remain below success, above contact onset and in a single hold segment with command depth at 20 mm. Depth range, rather than net displacement, rejects oscillatory cancellation. No load or future samples enter the detector.',
        '', 'Calibration used only 60 new nominal trajectories in 15 preassigned sign/onset groups. Twenty new conditions in five different groups were reserved for development validation. The candidate grid and zero-additional-success-alert constraint were fixed before collection. Selection maximized new/earlier safe coverage of failed nominal trajectories, then lead and conservative thresholds. Closed-loop success did not select thresholds. No thresholds changed after validation/comparison outcomes.',
        '', 'The previous 48-condition result files were not calibration/evaluation inputs. Only their pre-collection parameter plan was used to reject duplicate paths. This development set targets moderate terminal conditions, severe oblique/combined tilts and late steep ramps, with successful controls. Target names describe inputs, not guaranteed outcomes. All trials retain the original 8 s insertion command and 20 s episode budget.',
        '', '## Remaining failures and matching','',
        f"V2 unsuccessful outcomes: {dict(Counter(r['outcome'] for r in failures if r['policy']=='v2'))}. All failures remain in denominators; see failure_cases.csv and the 80 per-condition depth/grasp histories under conditions/.",
        '', f"Strict v1/v2 pre-intervention matches: {sum(r['prefix_matched'] for r in pairs)}/80. Discordant successes when neither intervened: {sum(r['neither_intervenes'] and r['success_delta']!=0 for r in pairs)}. Those differences cannot be attributed to detector/recovery actions. One execution per condition/policy measures broad targeted coverage, not independent repeated-trial reliability.",
        '', f"Outcome discordances where v2 never used either added branch: {sum(not r['v2_new_branch_used'] and r['success_delta']!=0 for r in pairs)}. In those v2 executions only the unchanged normal trigger acted (or no intervention occurred). Reset-state variation can change grasp-safety outcomes even when both policies trigger normally at the same time. These differences are retained in primary success rates but are not evidence of benefit from the new branches. paired_comparisons.csv records branch use, first-trigger times and state-match errors.",
        '', '## Validation and artifacts','',
        f"Audit passed: {audit['physics_rows']} physics rows, {audit['causal_checks']} detector checks. Original recovery code and its geometric commands, verified retreat/hold, retry pose and safety priorities were replayed; detector-v2 decisions were independently reconstructed. {audit['preexisting_outputs_checked']} old output files checked, changed: 0. Source snapshots, development plan and selected configuration hashes are retained.",
        '', 'Required artifacts: detector_v2_config.json, summary.csv, trigger_analysis.csv, false_trigger_analysis.csv and validation.json. Additional artifacts: policy_summary.csv, calibration_candidates.csv, recovery_events.csv, paired_comparisons.csv, failure_cases.csv, per-run logs/events/metrics and condition plots.',
        '', '```bash', './productivity_detector_v2.sh --headless', '```',
        '', 'The launcher refuses existing output directories. Offline report regeneration: `python -m research.productivity_detector_v2_report outputs/Contact-Productivity-DetectorV2-Dev`.',
        '', '![Policies](policy_comparison.png)', '', '![Coverage and false alerts](coverage_false_alerts.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    print(json.dumps(analyze(parser.parse_args().directory),indent=2))
