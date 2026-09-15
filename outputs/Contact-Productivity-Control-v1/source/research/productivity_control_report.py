"""Offline comparison of the bounded productivity controller pilot."""
import argparse
from collections import Counter
import csv
import json
from pathlib import Path
import numpy as np
from research.future_stall import table
from research.forge_protocol import ForgeProtocol, match
from research.productivity_control import POLICIES


def read_log(path):
    with path.open() as f:
        result=[]
        for row in csv.DictReader(f):
            converted={}
            for k,v in row.items():
                if v in ('True','False'):converted[k]=v=='True'
                elif v=='':converted[k]=None
                else:
                    try:converted[k]=float(v)
                    except ValueError:converted[k]=v
            result.append(converted)
        return result


def analyze(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    m=json.loads((directory/'experiment.json').read_text());cal=json.loads((directory/'calibration.json').read_text())
    runs=m['runs'];p=ForgeProtocol(**m['protocol']);raw={r['run_id']:read_log(directory/r['log']) for r in runs}
    table(directory/'summary.csv',runs)
    aggregates=[]
    for policy in POLICIES:
        rs=[r for r in runs if r['policy']==policy]
        if not rs:continue
        aggregates.append(dict(policy=policy,runs=len(rs),successes=sum(r['insertion_success'] for r in rs),
            stalled_runs=sum(r['stalled'] for r in rs),safety_stops=sum(r['outcome'] not in ('insertion_success','time_budget_exhausted') for r in rs),
            interventions=sum(r['intervention_count'] for r in rs),mean_final_depth_mm=float(np.mean([r['final_depth_mm'] for r in rs])),
            mean_elapsed_insertion_s=float(np.mean([r['insertion_time_s'] for r in rs])),
            max_wrist_force_n=max(r['max_wrist_force_n'] for r in rs),max_wrist_torque_nm=max(r['max_wrist_torque_nm'] for r in rs),
            max_normal_load_n=max(r['max_normal_load_n'] for r in rs)))
    table(directory/'policy_summary.csv',aggregates)
    pairs=[];event_rows=[]
    for case in m['cases']:
        for repeat in sorted({r['repeat'] for r in runs}):
            rr={r['policy']:r for r in runs if r['case_id']==case['case_id'] and r['repeat']==repeat}
            if set(rr)!=set(POLICIES):continue
            logs={k:raw[v['run_id']] for k,v in rr.items()}
            end=min(next((i for i,x in enumerate(log) if x['intervention_started']),len(log)-1) for log in logs.values())
            for policy in ('force','productivity'):
                a,b=rr['nominal'],rr[policy];al,bl=logs['nominal'],logs[policy]
                matched,errors=match(al[end],bl[end],p)
                pairs.append(dict(case_id=case['case_id'],repeat=repeat,policy=policy,
                    success_delta=int(b['insertion_success'])-int(a['insertion_success']),stall_delta=int(b['stalled'])-int(a['stalled']),
                    final_depth_delta_mm=b['final_depth_mm']-a['final_depth_mm'],elapsed_delta_s=b['insertion_time_s']-a['insertion_time_s'],
                    prefix_end_s=al[end]['time_s'],prefix_endpoint_matched=matched,
                    prefix_depth_rmse_mm=float(np.sqrt(np.mean([(x['depth_mm']-y['depth_mm'])**2 for x,y in zip(al[:end+1],bl[:end+1])]))),
                    prefix_wrist_force_rmse_n=float(np.sqrt(np.mean([(x['wrist_force_n']-y['wrist_force_n'])**2 for x,y in zip(al[:end+1],bl[:end+1])]))),
                    prefix_position_error_mm=errors['position_mm'],prefix_orientation_error_deg=errors['orientation_deg']))
                for event in json.loads((directory/b['run_id']/'events.json').read_text()):
                    event_rows.append(dict(case_id=case['case_id'],repeat=repeat,policy=policy,**event,
                        nominal_stall_confirmation_s=a['first_stall_s'],
                        lead_to_nominal_stall_s=a['first_stall_s']-event['trigger_time_s'] if a['first_stall_s'] is not None else None,
                        prefix_endpoint_matched=matched))
    table(directory/'paired_comparisons.csv',pairs);table(directory/'interventions.csv',event_rows)
    colors=dict(nominal='#555555',force='#D97706',productivity='#007F85')
    cases=[c['case_id'] for c in m['cases']]
    for kind in ('depth','productivity','force'):
        fig,axes=plt.subplots(len(cases),1,figsize=(11,3*len(cases)),squeeze=False)
        for case,ax in zip(cases,axes[:,0]):
            for r in runs:
                if r['case_id']!=case:continue
                log=raw[r['run_id']];policy=r['policy'];color=colors[policy]
                if kind=='productivity':log=[x for x in log if x['eta_valid']]
                key={'depth':'depth_mm','productivity':'eta_raw','force':'wrist_force_n'}[kind]
                ax.plot([x['time_s']-2 for x in log],[x[key] for x in log],color=color,
                    linestyle='-' if r['repeat']==0 else '--',alpha=.8,label=policy if r['repeat']==0 else None)
                for event in json.loads((directory/r['run_id']/'events.json').read_text()):
                    ax.axvline(event['trigger_time_s']-2,color=color,alpha=.25,lw=1)
            if kind=='depth':ax.axhline(p.success_depth_mm,color='black',ls=':',label='success depth')
            if kind=='productivity':ax.axhline(cal['eta_threshold'],color='black',ls=':',label='eta trigger')
            if kind=='force':ax.axhline(cal['force_threshold_n'],color='black',ls=':',label='force trigger')
            ax.set(title=case,xlabel='Time since insertion start (s)',ylabel={'depth':'Actual depth (mm)','productivity':'Raw trailing eta','force':'Wrist force (N)'}[kind],xlim=(0,20))
            ax.grid(alpha=.2);ax.legend(ncol=4,fontsize=8)
        fig.tight_layout();fig.savefig(directory/f'{kind}_comparison.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(2,2,figsize=(12,8))
    for ax,key,title in zip(axes.flat,('final_depth_mm','insertion_time_s','max_wrist_force_n','max_normal_load_n'),
            ('Final depth (mm)','Elapsed insertion time (s)','Maximum wrist force (N)','Maximum privileged normal load (N)')):
        x=np.arange(len(cases))
        for j,policy in enumerate(POLICIES):
            vals=[[r[key] for r in runs if r['case_id']==c and r['policy']==policy] for c in cases]
            means=[np.mean(v) if v else np.nan for v in vals]
            ax.bar(x+(j-1)*.24,means,width=.23,color=colors[policy],alpha=.7,label=policy)
            for i,vs in enumerate(vals):ax.scatter([x[i]+(j-1)*.24]*len(vs),vs,color='black',s=10)
        ax.set(title=title,xticks=x,xticklabels=[c.replace('_','\n') for c in cases]);ax.grid(axis='y',alpha=.2)
    axes[0,0].legend();fig.tight_layout();fig.savefig(directory/'outcome_comparison.png',dpi=150);plt.close(fig)
    a={r['policy']:r for r in aggregates}
    lines=['# Contact Productivity intervention pilot','',
        f"Status: **{m['status']}**. {len(runs)} runs, {len(cases)} Stage4 path geometries, two deterministic preparation repeats per policy. Native FORGE physics {m['physics_hz']} Hz; unchanged scene, contact settings, controller gains and hard safety limits.",'',
        '## Main result','',
        '| Policy | Success | Ever stalled | Safety stops | Interventions | Mean final depth (mm) |',
        '|---|---:|---:|---:|---:|---:|']
    for r in aggregates:lines.append(f"| {r['policy']} | {r['successes']}/{r['runs']} | {r['stalled_runs']}/{r['runs']} | {r['safety_stops']} | {r['interventions']} | {r['mean_final_depth_mm']:.3f} |")
    if len(a)==3:
        delta=a['productivity']['successes']-a['nominal']['successes']
        lines+=['',f"Productivity changed successes by {delta:+d} versus nominal and {a['productivity']['successes']-a['force']['successes']:+d} versus force intervention. A lower stall count alone is not evidence of better insertion: stopping, exhausting retries, or reaching a safety limit can censor the stall predicate.",
            '', 'This fixed pilot supports further controlled testing only if depth/success improve without worse safety outcomes. It does not establish a robust controller: four geometries and repeated resets are not independent population trials. No policy or threshold was tuned on pilot outcomes.']
    lines+=['','## Frozen design and calibration','',
        f"Trailing actual depth change / commanded depth change over {cal['design']['history_s']} s; raw ratio is never clipped. Checks every {cal['design']['check_s']} s, requiring at least {cal['design']['minimum_command_mm']} mm positive command progress and a complete contiguous insertion history beyond ramp onset + 0.1 mm. No normal-load or ML predictor.",
        '',f"Productivity triggers at eta < **{cal['eta_threshold']:.6f}** for two consecutive checks. Force triggers at wrist force >= **{cal['force_threshold_n']:.6f} N** for two consecutive eligible checks. The common motion/history gate avoids pre-contact transient alarms; the force decision itself uses force only. Missing history, an interrupted segment or a non-consecutive check resets the counter.",
        '',f"Thresholds were frozen from {cal['development_trajectories']} trajectories / {cal['development_groups']} path groups in Phase2A and Phase2B development data. Stage4 and all earlier trajectories sharing its path groups were excluded (including all Stage3). Both thresholds maximize sensitivity within a 10% group-weighted non-stalled trajectory alert budget; centered duplicates share weight. Calibration uses entire insertion histories and is not a claim of pre-stall prediction. See calibration.json for hashes and diagnostic alert rates.",
        '', 'Every policy receives 20 s after a 2 s approach, stopping early only after 19.5 mm for 0.2 s or a safety violation. Nominal retains the archived 8 s smoothstep descent then holds its endpoint for the remaining budget. Thus it gets the same available wall-time as the intervention arms, rather than being artificially stopped at 8 s.',
        '', 'Both intervention arms share the same response: stop at measured hand pose for 0.25 s; command 1 mm straight world-Z retraction over 0.5 s, preserving the measured hand orientation; then smoothly rejoin the original depth-dependent path over 0.5 s. Restart the original smoothstep clock at the measured retracted depth. No lateral/tilt correction is selected. Maximum-reached-depth ramp memory never rewinds. The nominal lateral/tilt demand is restored smoothly during rejoin; this may reload contact. At most two interventions; after that, continue the path subject to the same hard limits and total time budget.',
        '', 'The commanded depth during stop/retract is the captured peg-depth reference; during rejoin it is the interpolated peg-depth reference. Commanded hand pose is logged explicitly. Rejoin/stop/retract histories are excluded from the eta detector. Grasp compliance means commanded and actual retraction can differ; interventions.csv records both net retraction since trigger and motion during the retract phase.',
        '', 'Hard safety gates have priority at every observed physics tick: wrist force 20 N, wrist torque 1 Nm, grasp slip 0.1 mm / 0.5 deg, penetration screen 0.126475 mm. A violation terminates the episode; it is never treated as another recoverable soft trigger. The inherited Bench preparation/reset is unchanged; logged gates cover its returned initial state onward.',
        '', 'Stall uses the original 0.5 s / >=0.5 mm commanded / <0.1 mm actual predicate, restricted to uninterrupted insertion segments so intentional retraction is not labeled a stall. All episodes, including safety terminations and timeouts, remain in denominators. Timeouts have no time-to-success value; insertion_time_s is elapsed time to outcome, including interventions and final hold.',
        '', '## Pairing, repeatability and intervention timing','',
        f"Policy order is randomized within each case/repeat with fixed seed; preparation seed is shared. {sum(r['prefix_endpoint_matched'] for r in pairs)}/{len(pairs)} policy-versus-nominal common-prefix endpoints pass the existing strict full-state match. Prefix depth/wrist RMSE and pose mismatch are in paired_comparisons.csv. Matches do not restore unobserved solver state; mismatches weaken counterfactual interpretation and are retained, not excluded.",
        '', 'Vertical plot lines mark interventions; solid/dashed lines are the two repeats. Interventions.csv reports lead to that repeat’s nominal stall confirmation, with match status; it is a paired descriptive comparison, not a newly measured stall onset. No stall label is backdated.',
        '', '## Artifacts and integrity','',
        'summary.csv contains all requested per-run metrics plus safety, slip, penetration, elapsed/active time and outcomes. policy_summary.csv aggregates the policies; paired_comparisons.csv and interventions.csv expose pairing and actual retract behavior. Each run retains trajectory.csv at physics rate, events.json and metrics.json. Wrench, actual pose, velocity, contact load, commanded hand pose, detector eligibility, checks and actions are logged.',
        '',f"Existing-output audit: {m.get('preexisting_outputs_checked',0)} pre-existing files checked by size/mtime; changed: {len(m.get('preexisting_outputs_changed',[]))}. Calibration source files are additionally SHA-256 checked before simulation. Physics/source hashes and configuration snapshots are retained in this new namespace.",
        '', 'Reproduce with a new directory:', '', '```bash',
        './productivity_control.sh --headless --calibration experiments/productivity_control_calibration.json \\',
        '  --output-dir outputs/Contact-Productivity-Control-v2', '```', '',
        'Offline report regeneration: `python -m research.productivity_control_report outputs/Contact-Productivity-Control-v1`.', '',
        '![Outcomes](outcome_comparison.png)', '', '![Depth](depth_comparison.png)', '',
        '![Productivity](productivity_comparison.png)', '', '![Wrist force](force_comparison.png)']
    (directory/'report.md').write_text('\n'.join(lines)+'\n')
    return aggregates

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path)
    print(json.dumps(analyze(p.parse_args().directory.resolve()),indent=2))
