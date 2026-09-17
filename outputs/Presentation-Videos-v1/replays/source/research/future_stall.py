"""Causal history features and censored future-stall event labels from saved FORGE references."""
import argparse
from bisect import bisect_left,bisect_right
from collections import Counter
import csv
import hashlib
import json
import math
from pathlib import Path

FEATURES=('force_norm_n','torque_norm_nm','wrist_force_n','wrist_torque_nm','normal_load_n',
          'depth_mm','command_depth_mm','vx','vy','vz','fx','fy','fz','taux','tauy','tauz')


def read_rows(path):
    with path.open() as f:
        return [{k:v if k=='phase' else float(v) if v else math.nan for k,v in r.items()} for r in csv.DictReader(f)]


def stall_events(rows,p,hz):
    """Same trailing-window predicate as insertion_metrics; time is confirmation, not backdated onset."""
    window=max(1,round(p['stall_window_s']*hz));events=[];previous=False
    for i,r in enumerate(rows):
        active=False
        if i>=window:
            past=rows[i-window]
            active=(r['phase']=='insert' and past['phase']=='insert'
                and r['command_depth_mm']-past['command_depth_mm']>=p['stall_command_progress_mm']
                and r['depth_mm']-past['depth_mm']<p['stall_progress_mm'])
        if active and not previous:events.append(dict(index=i,time_s=r['time_s'],depth_mm=r['depth_mm']))
        previous=active
    return events


def future_label(t,horizon,first_event,end,eligible=True):
    if not eligible:return None,'invalid_or_incomplete_reference'
    if first_event is not None and first_event<=t+1e-8:return None,'already_stalled'
    if first_event is not None and first_event<=t+horizon+1e-8:return 1,'observed_future_stall'
    if t+horizon>end+1e-8:return None,'right_censored'
    return 0,'full_horizon_without_stall'


def features(rows,times,index,history):
    start=bisect_left(times,times[index]-history-1e-8)
    past=rows[start:index+1]
    if not past or times[index]-past[0]['time_s']<history-1e-7:return None
    dt=past[-1]['time_s']-past[0]['time_s']
    result=dict(history_start_row=start,history_end_row=index,history_samples=len(past))
    for key in FEATURES:
        values=[r[key] for r in past]
        if not all(math.isfinite(v) for v in values):return None
        result.update({key:values[-1],key+'_mean':sum(values)/len(values),key+'_max':max(values),
                       key+'_slope':(values[-1]-values[0])/dt})
    return result


def table(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        writer.writeheader();writer.writerows(rows)


def build(study,output,horizons=(1.,2.),history=.5,stride=.1):
    study=Path(study).resolve();output=Path(output).resolve()
    if any(not math.isfinite(x) or x<=0 for x in (*horizons,history,stride)) or len(set(horizons))!=len(horizons):
        raise ValueError('History, stride and unique horizons must be finite and positive')
    source=study/'study.json';manifest=json.loads(source.read_text())
    if manifest.get('study')!='Forge-Controlled-Phase2-v1':raise ValueError('Expected a FORGE study')
    output.mkdir(parents=True,exist_ok=False)
    hz=manifest['physics_hz'];p=manifest['protocol'];windows=[];checkpoint_rows=[];summaries=[];hashes={}
    for a in manifest['attempts']:
        path=study/a['folder']/'insertion.csv'
        if not path.is_file():continue
        rows=read_rows(path);times=[r['time_s'] for r in rows]
        hashes[str(path.relative_to(study))]=hashlib.sha256(path.read_bytes()).hexdigest()
        clock_ok=all(abs(b-a-1/hz)<1e-5 for a,b in zip(times,times[1:]))
        required=('depth_mm','command_depth_mm','min_separation_mm')
        finite=all(math.isfinite(r[k]) for r in rows for k in required)
        eligible=a['status']=='complete' and a.get('metrics',{}).get('numerically_valid') is True and clock_ok and finite
        events=stall_events(rows,p,hz) if clock_ok and finite else []
        first=events[0]['time_s'] if events else None
        active=[i for i,r in enumerate(rows) if r['phase']=='insert']
        if not active:continue
        end=times[active[-1]]
        base=dict(trajectory_id=a['trajectory_id'],folder=a['folder'],family=a['family'],
                  sample_role=a.get('sample_role','unspecified'),
                  split_group_id=a.get('split_group_id','centered_controls' if a['family']=='centered' else f"slot{a['slot']:03d}"))
        summaries.append(dict(base,numerically_valid=eligible,first_stall_confirmation_s=first,
            first_stall_depth_mm=events[0]['depth_mm'] if events else None,
            recorded_stalled=a.get('metrics',{}).get('stalled'),detected_stalled=bool(events)))
        next_time=times[active[0]]
        for i in active:
            t=times[i]
            if t+1e-8<next_time:continue
            next_time=t+stride
            f=features(rows,times,i,history)
            if f is None:continue
            for h in horizons:
                label,reason=future_label(t,h,first,end,eligible)
                windows.append(dict(base,time_s=t,horizon_s=h,Y_stall=label,label_reason=reason,**f))
        for cp in a['checkpoints']:
            if not cp.get('reached'):continue
            t=cp['state']['time_s']
            for h in horizons:
                label,reason=future_label(t,h,first,end,eligible)
                if cp['state'].get('phase')!='insert':label,reason=None,'not_inserting'
                checkpoint_rows.append(dict(base,checkpoint_depth_mm=cp['depth_mm'],time_s=t,horizon_s=h,
                    Y_R_tested=cp.get('Y_R_tested') if eligible and cp.get('prefix_numerically_valid') else None,
                    Y_stall=label,stall_label_reason=reason,
                    time_to_first_stall_s=first-t if first is not None and first>t else None))
    table(output/'windows.csv',windows);table(output/'checkpoint_labels.csv',checkpoint_rows);table(output/'trajectory_events.csv',summaries)
    counts={str(h):dict(Counter(str(r['Y_stall']) for r in windows if r['horizon_s']==h)) for h in horizons}
    result=dict(study=str(study),study_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),reference_sha256=hashes,
        extractor_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),physics_hz=hz,history_s=history,stride_s=stride,
        horizons_s=horizons,window_counts=counts,trajectories=len(summaries),
        stalled_trajectories=sum(s['detected_stalled'] and s['numerically_valid'] for s in summaries),
        event_definition={k:p[k] for k in ('stall_window_s','stall_progress_mm','stall_command_progress_mm')},
        target='Binary first confirmed stall in (t,t+h]; not a probability or a safe-progress label')
    (output/'labels.json').write_text(json.dumps(result,indent=2)+'\n')
    chosen=next((a for a in manifest['attempts'] if a['slot']==97 and a['status']=='complete'),None)
    if chosen:plot_example(study,chosen,p,hz,output)
    lines=['# Future-stall labels','',f"{result['trajectories']} reference records; {result['stalled_trajectories']} numerically valid stalled trajectories.",'',
        f'History: {history:g} s; stride: {stride:g} s; horizons: {horizons} s.', '',
        'Y_stall is an observed binary event target, not an estimated probability. The event time is the first confirmation of the existing trailing-window stall predicate; it is not backdated by the confirmation window.', '',
        'Only insertion time is at risk. Already-stalled states are excluded from first-event prediction. A negative requires the entire future horizon to be observed during insertion. End-of-insertion windows without a witnessed event are right-censored. Invalid/incomplete references have unknown targets.', '',
        'Features use only [t-history,t]; future labels are separate columns. Row indices point into the unchanged insertion.csv. Do not use event times, outcome fields, or future targets as model inputs. These observational trajectories support prediction under the recorded continuation, not arbitrary action-conditioned safety guarantees.', '',
        'Split all windows, checkpoints and branches using split_group_id. Centered controls share one group. Overlapping windows do not increase the number of independent stalled trajectories; only three positives would be insufficient to claim validated predictive performance. When pooling Phase 2B paths, audit identical pre-ramp histories across groups and exclude shared prefixes from independent evaluation.', '',
        '| Horizon [s] | Positive | Negative | Unknown |','| --- | ---: | ---: | ---: |']
    for h,c in counts.items():lines.append(f"| {h} | {c.get('1',0)} | {c.get('0',0)} | {c.get('None',0)} |")
    if chosen:
        lines+=['','## Slot 097','', '| Depth [mm] | Horizon [s] | Y_R | Y_stall | Time to confirmation [s] |','| --- | --- | --- | --- | --- |']
        for r in checkpoint_rows:
            if r['trajectory_id']==chosen['trajectory_id']:
                lines.append(f"| {r['checkpoint_depth_mm']:g} | {r['horizon_s']:g} | {r['Y_R_tested']} | {r['Y_stall']} | {r['time_to_first_stall_s']} |")
        lines+=['','![Raw histories and confirmed stall](slot097_stall.png)','']
    (output/'report.md').write_text('\n'.join(lines))
    print(json.dumps({k:result[k] for k in ('trajectories','stalled_trajectories','window_counts')},indent=2))


def plot_example(study,a,p,hz,output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    rows=read_rows(study/a['folder']/'insertion.csv');times=[r['time_s'] for r in rows];events=stall_events(rows,p,hz)
    fig,axes=plt.subplots(3,1,figsize=(10,8),sharex=True,constrained_layout=True)
    for ax,key,title in zip(axes,('wrist_force_n','wrist_torque_nm','depth_mm'),('Raw wrist force [N]','Raw wrist torque [Nm]','Depth [mm]')):
        ax.plot(times,[r[key] for r in rows],lw=1);ax.set_ylabel(title);ax.grid(alpha=.2)
        if events:ax.axvline(events[0]['time_s'],color='firebrick',ls='--',label='First confirmed stall')
        for c in a['checkpoints']:
            if c.get('reached') and c.get('Y_R_tested')==1:
                ax.axvline(c['state']['time_s'],color='seagreen',alpha=.4)
    axes[-1].plot(times,[r['command_depth_mm'] for r in rows],ls=':',label='Command depth')
    axes[-1].set_xlabel('Reference time [s]');axes[-1].legend()
    axes[0].set_title(f"{a['trajectory_id']}: green lines = observed safe recovery checkpoints")
    fig.savefig(output/'slot097_stall.png',dpi=160);fig.savefig(output/'slot097_stall.pdf');plt.close(fig)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('study',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--horizons',type=float,nargs='+',default=[1.,2.])
    parser.add_argument('--history',type=float,default=.5);parser.add_argument('--stride',type=float,default=.1)
    args=parser.parse_args();build(args.study,args.output,args.horizons,args.history,args.stride)

if __name__=='__main__':main()
