"""Offline future insertion-productivity prediction; never imports or launches Isaac Sim.

Primary models use logged wrist wrench. True peg/socket contact wrench is a
separate comparison. Simulator normal load is privileged analysis metadata only.
"""
import argparse
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from research.contact_response import quat_increment, table, write_json, digest, auc, verify_frame
from research.future_stall import read_rows, stall_events

STUDIES = ('Forge-Phase2-100', *(f'Forge-Phase2B-Stage{i}' for i in range(1,5)))
PRIMARY = ('force_only','torque_only','current_progress','wrench_history','progress_wrench_motion')
MODELS = (*PRIMARY,'contact_wrench_history','progress_contact_motion')
STATS = ('last','mean','std','max_abs','slope','derivative_rms')


@dataclass(frozen=True)
class Settings:
    horizons: tuple = (.25,.5,1.)
    history_s: float = .5
    stride_s: float = .1
    min_command_progress_mm: float = .1
    prefix_margin_mm: float = .1
    degradation_eta: float = .5
    outer_folds: int = 5
    inner_folds: int = 3
    alphas: tuple = (.001,.01,.1)
    target_nonstall_alert_rate: float = .1
    seed: int = 20260915
    bootstrap: int = 500


def productivity(actual, command, index, end, minimum=.1):
    """Unclipped signed endpoint ratio, in mm/mm. No absolute-value denominator."""
    dc=float(command[end]-command[index]); da=float(actual[end]-actual[index])
    if not np.isfinite([dc,da]).all() or dc<minimum-1e-9:return None,da,dc
    return da/dc,da,dc


def stats(t,x):
    x=np.asarray(x,float);tc=t-t.mean();den=tc@tc
    return dict(last=float(x[-1]),mean=float(x.mean()),std=float(x.std()),
                max_abs=float(np.max(np.abs(x))),slope=float(tc@(x-x.mean())/den),
                derivative_rms=float(np.sqrt(np.mean((np.diff(x)/np.diff(t))**2))))


def history_features(rows,start,end):
    """Explicit causal input allowlist. Normal load, future targets and outcomes cannot enter."""
    rr=rows[start:end+1];t=np.array([r['time_s'] for r in rr]);dt=t[-1]-t[0]
    f={}
    for prefix,keys in (
        ('wrist', [*(f'wrist_force_world_{i}' for i in range(3)),*(f'wrist_torque_about_peg_base_world_{i}' for i in range(3))]),
        ('contact',['fx','fy','fz','taux','tauy','tauz'])):
        for name,key in zip(('fx','fy','fz','taux','tauy','tauz'),keys):
            for stat,value in stats(t,[r[key] for r in rr]).items():f[f'{prefix}_{name}_{stat}']=value
        for name,key in (('force','wrist_force_n' if prefix=='wrist' else 'force_norm_n'),
                         ('torque','wrist_torque_nm' if prefix=='wrist' else 'torque_norm_nm')):
            for stat,value in stats(t,[r[key] for r in rr]).items():f[f'{prefix}_{name}_{stat}']=value
    da=rr[-1]['depth_mm']-rr[0]['depth_mm'];dc=rr[-1]['command_depth_mm']-rr[0]['command_depth_mm']
    # All analysis histories are inside moving insertion; still guard the ratio explicitly.
    if dc<=1e-6:raise ValueError('History has unresolved commanded motion')
    recent=max(0,len(rr)-1-round(.1/(t[1]-t[0])))
    recent_dt=t[-1]-t[recent]
    f.update(actual_depth_mm=rr[-1]['depth_mm'],command_depth_mm=rr[-1]['command_depth_mm'],
        depth_gap_mm=rr[-1]['command_depth_mm']-rr[-1]['depth_mm'],
        actual_rate_mm_s=da/dt,command_rate_mm_s=dc/dt,current_eta=da/dc,
        deficit_rate_mm_s=(dc-da)/dt,
        actual_recent_rate_mm_s=(rr[-1]['depth_mm']-rr[recent]['depth_mm'])/recent_dt,
        command_recent_rate_mm_s=(rr[-1]['command_depth_mm']-rr[recent]['command_depth_mm'])/recent_dt)
    for key in ('vx','vy','vz','omegax','omegay','omegaz'):
        for name,value in stats(t,[r[key] for r in rr]).items():
            if name in ('last','mean','std'):f[f'motion_{key}_{name}']=value
    q=np.array([[r[k] for k in ('qw','qx','qy','qz')] for r in rr])
    absolute=quat_increment(np.array([0.,0.,0.,1.]),q[-1]);change=quat_increment(q[0],q[-1])
    for i,key in enumerate(('roll','pitch','yaw')):
        f[f'pose_{key}_rad']=float(absolute[i]);f[f'pose_{key}_rate_rad_s']=float(change[i]/dt)
    for key in ('tip_x_mm','tip_y_mm'):
        f[f'pose_{key}']=rr[-1][key];f[f'pose_{key}_rate']=(rr[-1][key]-rr[0][key])/dt
    if not np.isfinite(list(f.values())).all():raise ValueError('Nonfinite predictor')
    return f


def feature_sets(example):
    progress=['actual_depth_mm','command_depth_mm','depth_gap_mm','actual_rate_mm_s','command_rate_mm_s',
              'current_eta','deficit_rate_mm_s','actual_recent_rate_mm_s','command_recent_rate_mm_s']
    wrist=[k for k in example if k.startswith('wrist_')]
    contact=[k for k in example if k.startswith('contact_')]
    motion=[k for k in example if k.startswith(('motion_','pose_'))]
    result=dict(force_only=[f'wrist_force_{s}' for s in STATS],torque_only=[f'wrist_torque_{s}' for s in STATS],
        current_progress=progress,wrench_history=wrist,progress_wrench_motion=progress+wrist+motion,
        contact_wrench_history=contact,progress_contact_motion=progress+contact+motion)
    for keys in result.values():
        assert keys and not any('normal_load' in k or k.startswith('priv_') or k.startswith('future_') for k in keys)
    return result


def group_for(study,a):
    if a['family']=='centered':return 'Phase2A:centered_controls'
    if study=='Forge-Phase2-100':return 'Phase2A:'+a.get('split_group_id',f"slot{a['slot']}")
    return 'Phase2B:'+a['path_group_id']


def role_for(study,group,stress_groups):
    if study==STUDIES[-1]:return 'stage4'
    return 'quarantined_shared_path' if group in stress_groups else 'development'


def load_dataset(root,settings):
    manifests={name:json.loads((root/name/'study.json').read_text()) for name in STUDIES}
    stress_groups={group_for(STUDIES[-1],a) for a in manifests[STUDIES[-1]]['attempts']}
    windows=[];summaries=[];sources={};rejections=Counter();raw={};inputs=None
    for name,m in manifests.items():
        if m['study']!='Forge-Controlled-Phase2-v1' or m['mode']!='collect':raise ValueError('Unexpected cohort')
        verify_frame(root/name,m)
        for file in ('study.json','config.json','scene.json','source/forge_backend.py','source/contact.py'):
            path=root/name/file;sources[str(path)]=digest(path)
        hz=m['physics_hz'];history=round(settings.history_s*hz);stride=round(settings.stride_s*hz)
        for a in m['attempts']:
            uid=name+'/'+a['folder'];path=root/name/a['folder']/'insertion.csv'
            rows=read_rows(path);sources[str(path)]=digest(path);raw[uid]=rows
            times=np.array([r['time_s'] for r in rows]);actual=np.array([r['depth_mm'] for r in rows]);command=np.array([r['command_depth_mm'] for r in rows])
            if not np.allclose(np.diff(times),1/hz,atol=1e-6,rtol=0):raise ValueError('Unexpected clock')
            group=group_for(name,a);role=role_for(name,group,stress_groups)
            events=stall_events(rows,m['protocol'],hz);first=events[0]['time_s'] if events else None
            if bool(events)!=a['metrics']['stalled']:raise ValueError('Stall reconstruction disagrees with archive')
            eligible=a['status']=='complete' and a['metrics']['numerically_valid']
            base=dict(trajectory_id=uid,study=name,group_id=group,role=role,family=a['family'],
                      stalled=bool(events),stall_confirmation_s=first,
                      stall_lookback_start_s=first-m['protocol']['stall_window_s'] if first is not None else None)
            summary=dict(**base,insertion_success=a['metrics']['insertion_success'],numerically_valid=eligible,
                         grasp_retained=a['metrics'].get('grasp_retained',True),
                         max_depth_mm=float(actual.max()),max_normal_load_n=a['metrics']['max_normal_load'],windows=0)
            summaries.append(summary)
            for i in range(history,len(rows),stride):
                start=i-history
                if any(r['phase']!='insert' for r in rows[start:i+1]):continue
                if not eligible:rejections['invalid_reference']+=1;continue
                if min(actual[start:i+1])<0:rejections['history_before_entry']+=1;continue
                if a.get('ramp_onset_mm') is not None and actual[start]<=a['ramp_onset_mm']+settings.prefix_margin_mm:
                    rejections['shared_preramp_history']+=1;continue
                f=history_features(rows,start,i)
                if inputs is None:inputs=feature_sets(f)
                signature_keys=('depth_mm','command_depth_mm','tip_x_mm','tip_y_mm','qw','qx','qy','qz',
                                'fx','fy','fz','taux','tauy','tauz','wrist_force_n','wrist_torque_nm','vx','vy','vz','omegax','omegay','omegaz')
                sig=np.round([[r[k] for k in signature_keys] for r in rows[start:i+1]],9)
                signature=hashlib.sha256(sig.tobytes()).hexdigest()
                for h in settings.horizons:
                    end=i+round(h*hz)
                    if end>=len(rows) or rows[end]['phase']!='insert':rejections['right_censored_insertion_end']+=1;continue
                    eta,da,dc=productivity(actual,command,i,end,settings.min_command_progress_mm)
                    if eta is None:rejections['insufficient_future_command_progress']+=1;continue
                    pre=first is None or times[i]<first-1e-8
                    normal=[r['normal_load_n'] for r in rows[start:i+1]]
                    w=dict(**base,window_id=f'{uid}:{i}:{h:g}',time_s=float(times[i]),horizon_s=h,
                        history_start_row=start,history_end_row=i,future_end_row=end,history_sha256=signature,
                        eta_raw=eta,eta_clipped_0_1_diagnostic=float(np.clip(eta,0,1)),
                        future_actual_progress_mm=da,future_command_progress_mm=dc,
                        pre_stall=pre,before_stall_lookback=first is None or times[i]<first-m['protocol']['stall_window_s']-1e-8,
                        modeling_eligible=pre,exclusion_reason='' if pre else 'already_confirmed_stall',
                        priv_normal_load_n=rows[i]['normal_load_n'],priv_normal_load_mean_n=float(np.mean(normal)),
                        priv_normal_load_max_n=float(max(normal)),
                        priv_hidden_load=bool(np.mean(normal)>=10. and rows[i]['wrist_force_n']<=5.),**f)
                    windows.append(w);summary['windows']+=1
    # Remove exact shared physical histories across distinct canonical groups,
    # regardless of whether their future targets happen to match.
    signatures=defaultdict(set)
    for w in windows:signatures[w['history_sha256']].add(w['group_id'])
    shared={k for k,v in signatures.items() if len(v)>1}
    for w in windows:
        if w['history_sha256'] in shared:
            w.update(modeling_eligible=False,exclusion_reason='identical_history_across_groups')
            rejections['identical_history_across_groups']+=1
    return windows,summaries,inputs,sources,dict(rejections),raw


def weights(groups):
    _,inv,counts=np.unique(groups,return_inverse=True,return_counts=True)
    w=1/counts[inv];return w/w.sum()


def ridge_fit(x,y,groups,alpha):
    w=weights(groups);mean=np.sum(x*w[:,None],axis=0)
    std=np.sqrt(np.sum((x-mean)**2*w[:,None],axis=0));std[std<1e-9]=1.
    z=(x-mean)/std;intercept=float(w@y)
    coef=np.linalg.solve((z*w[:,None]).T@z+alpha*np.eye(x.shape[1]),z.T@(w*(y-intercept)))
    return dict(mean=mean,std=std,coef=coef,intercept=intercept,alpha=alpha)


def ridge_predict(model,x):return (x-model['mean'])/model['std']@model['coef']+model['intercept']


def folds(groups,stalled,k,seed):
    """Group-disjoint folds balanced across development stalled/nonstalled groups."""
    unique=sorted(set(groups));k=min(k,len(unique))
    if k<2:raise ValueError('Too few independent groups')
    positive={g:any(s for gg,s in zip(groups,stalled) if gg==g) for g in unique}
    rng=np.random.default_rng(seed);assignment={};offset=0
    for flag in (True,False):
        subset=[g for g in unique if positive[g]==flag];rng.shuffle(subset)
        for j,g in enumerate(subset):assignment[g]=(offset+j)%k
        offset=(offset+len(subset))%k
    return np.array([assignment[g] for g in groups])


def select_alpha(x,y,groups,stalled,settings,seed):
    split=folds(groups,stalled,settings.inner_folds,seed);candidates=[]
    for alpha in settings.alphas:
        pred=np.empty(len(y))
        for fold in np.unique(split):
            test=split==fold;train=~test
            model=ridge_fit(x[train],y[train],groups[train],alpha);pred[test]=ridge_predict(model,x[test])
        candidates.append((float(weights(groups)@((pred-y)**2)),alpha,pred))
    _,alpha,pred=min(candidates,key=lambda v:(v[0],-v[1]))
    return alpha,pred


def sustained_score(rows,scores,stride=.1):
    """Two adjacent history windows (0.1 s apart) must both exceed an alarm threshold."""
    order=np.argsort([r['time_s'] for r in rows]);r=[rows[i] for i in order];s=np.asarray(scores)[order]
    values=[]
    for i in range(1,len(r)):
        if abs(r[i]['time_s']-r[i-1]['time_s']-stride)<1e-6:values.append((r[i]['time_s'],min(s[i],s[i-1])))
    return values


def alarm_threshold(rows,scores,target=.1):
    by=defaultdict(list)
    for i,r in enumerate(rows):
        if not r['stalled']:by[r['trajectory_id']].append(i)
    maxima=[];groups=[]
    for ids in by.values():
        seq=sustained_score([rows[i] for i in ids],np.asarray(scores)[ids])
        if seq:maxima.append(max(v for _,v in seq));groups.append(rows[ids[0]]['group_id'])
    if not maxima:return float('inf')
    maxima=np.asarray(maxima);w=weights(np.array(groups))
    # Equality triggers an alarm: nextafter makes the chosen false-alert budget explicit.
    for threshold in sorted(set(maxima)):
        value=np.nextafter(threshold,np.inf)
        if w@(maxima>=value)<=target+1e-12:return float(value)
    return float('inf')


def metric(rows,pred):
    y=np.array([r['eta_raw'] for r in rows]);g=np.array([r['group_id'] for r in rows]);w=weights(g)
    err=pred-y;variance=float(w@((y-w@y)**2));mse=float(w@(err**2));bad=y<.5
    return dict(windows=len(y),groups=len(set(g)),trajectories=len({r['trajectory_id'] for r in rows}),
        mae=float(w@np.abs(err)),rmse=float(np.sqrt(mse)),r2=1-mse/variance if variance>1e-12 else None,
        degradation_auc=auc(bad,-pred,w),degraded_weight=float(w@bad),
        eta_mean=float(w@y),prediction_outside_0_1_fraction=float(w@((pred<0)|(pred>1))))


def train_evaluate(windows,inputs,settings,output):
    predictions=[];models={};cv_audit=[];metrics=[]
    for h in settings.horizons:
        pool=[r for r in windows if r['horizon_s']==h and r['modeling_eligible']]
        dev=[r for r in pool if r['role']=='development'];targets=[r for r in pool if r['role']!='development']
        y=np.array([r['eta_raw'] for r in dev]);groups=np.array([r['group_id'] for r in dev]);stalled=np.array([r['stalled'] for r in dev])
        split=folds(groups,stalled,settings.outer_folds,settings.seed)
        print(f'H={h:g}: {len(dev)} development windows, {len(set(groups))} groups; {len(targets)} held/quarantined windows',flush=True)
        for name in (*MODELS,'persistence'):
            if name=='persistence':
                oof=np.array([r['current_eta'] for r in dev]);testpred=np.array([r['current_eta'] for r in targets]);cal=np.empty(len(dev))
                for fold in np.unique(split):cal[split==fold]=alarm_threshold([r for i,r in enumerate(dev) if split[i]!=fold],-oof[split!=fold],settings.target_nonstall_alert_rate)
                final_threshold=alarm_threshold(dev,-oof,settings.target_nonstall_alert_rate)
            else:
                keys=inputs[name];x=np.array([[r[k] for k in keys] for r in dev]);xt=np.array([[r[k] for k in keys] for r in targets]);oof=np.empty(len(y));cal=np.empty(len(y))
                for fold in np.unique(split):
                    train=split!=fold;test=~train
                    alpha,inner=select_alpha(x[train],y[train],groups[train],stalled[train],settings,settings.seed+int(fold)+1)
                    fitted=ridge_fit(x[train],y[train],groups[train],alpha);oof[test]=ridge_predict(fitted,x[test])
                    tr=[r for i,r in enumerate(dev) if train[i]]
                    threshold=alarm_threshold(tr,-inner,settings.target_nonstall_alert_rate);cal[test]=threshold
                    train_groups=set(groups[train]);test_groups=set(groups[test]);assert train_groups.isdisjoint(test_groups)
                    cv_audit.append(dict(horizon_s=h,model=name,fold=int(fold),alpha=alpha,
                        train_groups=sorted(train_groups),validation_groups=sorted(test_groups),alarm_score_threshold=threshold))
                alpha,_=select_alpha(x,y,groups,stalled,settings,settings.seed+101)
                fitted=ridge_fit(x,y,groups,alpha);testpred=ridge_predict(fitted,xt)
                final_threshold=alarm_threshold(dev,-oof,settings.target_nonstall_alert_rate)
                models[f'{h}:{name}']=dict(features=keys,**fitted,alarm_score_threshold=final_threshold)
            for source,pp,thresholds,partition in ((dev,oof,cal,'development_oof'),(targets,testpred,np.full(len(targets),final_threshold),'held')):
                for r,pred,threshold in zip(source,pp,thresholds):
                    predictions.append(prediction_row(r,name,float(pred),float(threshold),partition))
            for role in ('development_oof','stage4','quarantined_shared_path'):
                selected=[r for r in predictions if r['horizon_s']==h and r['model']==name and r['evaluation']==role]
                if selected:metrics.append(dict(evaluation=role,horizon_s=h,model=name,**metric(selected,np.array([r['predicted_eta_raw'] for r in selected]))))
        # A truly simple wrist force threshold, calibrated without stress-test data.
        force=np.array([r['wrist_force_last'] for r in dev]);threshold=alarm_threshold(dev,force,settings.target_nonstall_alert_rate)
        for fold in np.unique(split):
            tr=split!=fold;th=alarm_threshold([r for i,r in enumerate(dev) if tr[i]],force[tr],settings.target_nonstall_alert_rate)
            for i in np.flatnonzero(~tr):predictions.append(prediction_row(dev[i],'force_threshold',None,th,'development_oof'))
        for r in targets:predictions.append(prediction_row(r,'force_threshold',None,threshold,'held'))
    write_json(output/'models.json',models);write_json(output/'split_audit.json',cv_audit)
    return predictions,metrics


def prediction_row(r,name,pred,threshold,partition):
    keys=('window_id','trajectory_id','study','group_id','time_s','horizon_s','eta_raw','stalled',
          'stall_confirmation_s','stall_lookback_start_s','before_stall_lookback','actual_depth_mm','current_eta',
          'priv_normal_load_mean_n','priv_normal_load_n','priv_hidden_load','wrist_force_last','wrist_torque_last')
    return dict({k:r[k] for k in keys},model=name,predicted_eta_raw=pred,
        predicted_eta_clipped_0_1_diagnostic=float(np.clip(pred,0,1)) if pred is not None else None,
        alarm_score=r['wrist_force_last'] if pred is None else -pred,
        calibrated_alarm_score_threshold=threshold,
        evaluation=partition if partition=='development_oof' else r['role'])


def warning_results(predictions,settings,summaries=None):
    grouped=defaultdict(list);details=[];aggregated=[]
    for r in predictions:grouped[(r['evaluation'],r['horizon_s'],r['model'],r['trajectory_id'])].append(r)
    for (evaluation,h,name,uid),rr in grouped.items():
        rr=sorted(rr,key=lambda r:r['time_s']);base=rr[0]
        modes=['calibrated']+(['semantic_eta_0.5'] if name!='force_threshold' else [f'force_{f:g}N' for f in (1,2,5,10,20)])
        for mode in modes:
            threshold=base['calibrated_alarm_score_threshold'] if mode=='calibrated' else (-.5 if mode=='semantic_eta_0.5' else float(mode[6:-1]))
            seq=sustained_score(rr,[r['alarm_score'] for r in rr],settings.stride_s)
            first=next((t for t,s in seq if s>=threshold),None)
            event=base['stall_confirmation_s'];lookback=base['stall_lookback_start_s']
            # An alarm completed at/after confirmation is never credited.
            lead=event-first if first is not None and event is not None and first<event else None
            details.append(dict(evaluation=evaluation,horizon_s=h,model=name,trajectory_id=uid,group_id=base['group_id'],
                mode=mode,stalled=base['stalled'],alarm_time_s=first,stall_confirmation_s=event,
                lead_to_confirmation_s=lead,lead_to_lookback_start_s=lookback-first if lead is not None else None,
                detected_before_confirmation=lead is not None,
                detected_before_lookback=first is not None and lookback is not None and first<lookback,
                alarm_score_threshold=threshold,at_risk_windows=len(rr)))
    if summaries is not None:
        existing={(r['evaluation'],r['horizon_s'],r['model'],r['mode'],r['trajectory_id']) for r in details}
        combinations={(r['evaluation'],r['horizon_s'],r['model'],r['mode']) for r in details}
        for evaluation,h,name,mode in combinations:
            role='development' if evaluation=='development_oof' else evaluation
            for s in summaries:
                if s['role']!=role or not s['numerically_valid'] or (evaluation,h,name,mode,s['trajectory_id']) in existing:continue
                details.append(dict(evaluation=evaluation,horizon_s=h,model=name,mode=mode,trajectory_id=s['trajectory_id'],group_id=s['group_id'],
                    stalled=s['stalled'],alarm_time_s=None,stall_confirmation_s=s['stall_confirmation_s'],lead_to_confirmation_s=None,
                    lead_to_lookback_start_s=None,detected_before_confirmation=False,detected_before_lookback=False,
                    alarm_score_threshold=None,at_risk_windows=0))
    groups=defaultdict(list)
    for r in details:groups[(r['evaluation'],r['horizon_s'],r['model'],r['mode'])].append(r)
    for (evaluation,h,name,mode),rr in groups.items():
        st=[r for r in rr if r['stalled']];non=[r for r in rr if not r['stalled']];leads=[r['lead_to_confirmation_s'] for r in st if r['detected_before_confirmation']]
        aggregated.append(dict(evaluation=evaluation,horizon_s=h,model=name,mode=mode,stalled_trajectories=len(st),
            detected_before_confirmation=sum(r['detected_before_confirmation'] for r in st),
            detected_before_lookback=sum(r['detected_before_lookback'] for r in st),
            median_lead_to_confirmation_s=float(np.median(leads)) if leads else None,
            min_lead_to_confirmation_s=float(min(leads)) if leads else None,
            nonstalled_trajectories=len(non),nonstall_alerts=sum(r['alarm_time_s'] is not None for r in non),
            nonstall_alert_rate=float(np.mean([r['alarm_time_s'] is not None for r in non])) if non else None,
            nonstall_alert_rate_groupweighted=float(weights(np.array([r['group_id'] for r in non]))@np.array([r['alarm_time_s'] is not None for r in non])) if non else None,
            trajectories_without_eligible_windows=sum(r['at_risk_windows']==0 for r in rr)))
    return details,aggregated


def paired_comparisons(predictions,settings):
    lookup={(r['window_id'],r['model']):r for r in predictions};result=[];rng=np.random.default_rng(settings.seed)
    for evaluation in ('development_oof','stage4'):
        for h in settings.horizons:
            for comparison in ('force_only','current_progress','persistence','wrench_history'):
                pairs=[]
                for r in predictions:
                    if r['model']!='progress_wrench_motion' or r['evaluation']!=evaluation or r['horizon_s']!=h:continue
                    b=lookup[(r['window_id'],comparison)]
                    pairs.append((r['group_id'],abs(b['predicted_eta_raw']-r['eta_raw'])-abs(r['predicted_eta_raw']-r['eta_raw'])))
                groups=defaultdict(list)
                for g,d in pairs:groups[g].append(d)
                errors=np.array([np.mean(v) for v in groups.values()])
                if not len(errors):continue
                boots=np.mean(rng.choice(errors,(settings.bootstrap,len(errors)),replace=True),axis=1)
                result.append(dict(evaluation=evaluation,horizon_s=h,comparison=comparison,groups=len(errors),
                    mae_improvement=float(errors.mean()),ci_low=float(np.quantile(boots,.025)),ci_high=float(np.quantile(boots,.975))))
    return result


def hidden_analysis(predictions):
    results=[]
    # Primary definition is unchanged; additional definitions are explicitly descriptive sensitivity checks.
    definitions={'mean_10N':lambda r:r['priv_normal_load_mean_n']>=10,
                 'instant_10N_sensitivity':lambda r:r['priv_normal_load_n']>=10,
                 'mean_5N_sensitivity':lambda r:r['priv_normal_load_mean_n']>=5}
    for evaluation in ('development_oof','stage4','quarantined_shared_path'):
        for h in (.25,.5,1.):
            for name in PRIMARY:
                for definition,predicate in definitions.items():
                    rr=[r for r in predictions if r['evaluation']==evaluation and r['horizon_s']==h and r['model']==name and predicate(r) and r['wrist_force_last']<=5]
                    if not rr:continue
                    results.append(dict(evaluation=evaluation,horizon_s=h,model=name,hidden_load=True,definition=definition,
                                        **metric(rr,np.array([r['predicted_eta_raw'] for r in rr]))))
    return results


def hidden_states(windows,summaries,raw,settings):
    details=[];descriptive=[]
    for summary in summaries:
        rows=raw[summary['trajectory_id']];hz=round(1/(rows[1]['time_s']-rows[0]['time_s']))
        actual=np.array([r['depth_mm'] for r in rows]);command=np.array([r['command_depth_mm'] for r in rows])
        for i in range(round(settings.history_s*hz),len(rows),round(settings.stride_s*hz)):
            r=rows[i];start=i-round(settings.history_s*hz)
            load=float(np.mean([a['normal_load_n'] for a in rows[start:i+1]]))
            if max(load,r['normal_load_n'])<10 or r['wrist_force_n']>5:continue
            event=summary['stall_confirmation_s']
            row=dict(trajectory_id=summary['trajectory_id'],role=summary['role'],time_s=r['time_s'],phase=r['phase'],
                depth_mm=r['depth_mm'],normal_load_n=r['normal_load_n'],normal_load_mean_n=load,wrist_force_n=r['wrist_force_n'],
                before_confirmation=event is None or r['time_s']<event,
                time_relative_to_confirmation_s=r['time_s']-event if event is not None else None)
            for h in settings.horizons:
                end=i+round(h*hz);eta=None;reason='future_outside_insertion'
                if r['phase']=='insert' and end<len(rows) and rows[end]['phase']=='insert':
                    eta,_,_=productivity(actual,command,i,end,settings.min_command_progress_mm)
                    reason='valid_productivity_target' if eta is not None else 'insufficient_command_progress'
                row[f'eta_raw_H{h}']=eta;row[f'eta_reason_H{h}']=reason
            details.append(row)
    for role in ('development','stage4','quarantined_shared_path'):
        for h in settings.horizons:
            for pre in (True,False):
                rr=[r for r in windows if r['role']==role and r['horizon_s']==h and r['pre_stall']==pre and r['priv_hidden_load']]
                descriptive.append(dict(role=role,horizon_s=h,pre_stall=pre,definition='mean_10N',windows=len(rr),
                    trajectories=len({r['trajectory_id'] for r in rr}),
                    eta_mean=float(np.mean([r['eta_raw'] for r in rr])) if rr else None,
                    wrist_force_mean_n=float(np.mean([r['wrist_force_last'] for r in rr])) if rr else None))
    return details,descriptive


def plots(output,windows,predictions,summaries,raw):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    h=.5
    fig,axes=plt.subplots(2,5,figsize=(19,7),sharey=True)
    for col,study in enumerate(STUDIES):
        trajectories=sorted({r['trajectory_id'] for r in windows if r['study']==study})
        for uid in trajectories:
            rr=sorted([r for r in windows if r['trajectory_id']==uid and r['horizon_s']==h],key=lambda r:r['time_s'])
            color='C3' if rr and rr[0]['stalled'] else 'C0'
            for row,key in enumerate(('time_s','actual_depth_mm')):
                axes[row,col].plot([r[key] for r in rr],[r['eta_raw'] for r in rr],color=color,alpha=.25,lw=.8)
        axes[0,col].set_title('Phase 2A' if col==0 else f'Phase 2B Stage {col}')
        for row in range(2):
            axes[row,col].axhline(.5,color='k',ls='--',lw=.8)
            axes[row,col].set_xlabel('Time (s)' if row==0 else 'Actual depth (mm)')
    axes[0,0].set_ylabel('Raw future η, H=0.5 s');axes[1,0].set_ylabel('Raw future η, H=0.5 s')
    fig.suptitle('All qualifying targets; red = eventually stalled, blue = other; no ratio clipping')
    fig.tight_layout();fig.savefig(output/'eta_vs_time_depth.png',dpi=150);plt.close(fig)
    stalled=[s for s in summaries if s['role']=='stage4' and s['stalled']]
    fig,axes=plt.subplots(4,2,figsize=(14,14),squeeze=False)
    for ax,s in zip(axes.flat,stalled):
        for name,color in [('progress_wrench_motion','C0'),('force_only','C1'),('current_progress','C2')]:
            rr=sorted([r for r in predictions if r['trajectory_id']==s['trajectory_id'] and r['horizon_s']==h and r['model']==name],key=lambda r:r['time_s'])
            ax.plot([r['time_s'] for r in rr],[r['predicted_eta_raw'] for r in rr],label=name,color=color)
            if name=='progress_wrench_motion':ax.plot([r['time_s'] for r in rr],[r['eta_raw'] for r in rr],color='k',lw=1.5,label='Observed future η target')
        ax.axvline(s['stall_confirmation_s'],color='C3',ls='--',label='Stall confirmation')
        ax.axvline(s['stall_lookback_start_s'],color='gray',ls=':',label='Confirmation lookback starts')
        ax.axhline(.5,color='k',ls=':',lw=.6);ax.set(title=s['trajectory_id'].split('/')[-1],xlabel='Prediction time (s)',ylabel='η (raw)')
    for ax in list(axes.flat)[len(stalled):]:ax.axis('off')
    if stalled:axes[0,0].legend(fontsize=7)
    fig.suptitle('Group-disjoint Stage 4 stress test, H=0.5 s — no predictions after first confirmed stall')
    fig.tight_layout();fig.savefig(output/'predicted_eta_before_stall.png',dpi=150);plt.close(fig)
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,role in zip(axes,('development','stage4')):
        rr=[r for r in windows if r['role']==role and r['horizon_s']==h and r['modeling_eligible']]
        dots=ax.scatter([r['wrist_force_last'] for r in rr],[r['eta_raw'] for r in rr],c=[r['priv_normal_load_mean_n'] for r in rr],s=12,alpha=.65,cmap='viridis')
        ax.axhline(.5,color='k',ls='--');ax.set(xlabel='Current wrist force norm (N)',ylabel='Raw future η (H=0.5 s)',title=role)
        fig.colorbar(dots,ax=ax,label='Privileged mean normal load (N); never a predictor')
    fig.tight_layout();fig.savefig(output/'force_vs_productivity.png',dpi=150);plt.close(fig)
    selected=sorted([s for s in summaries if s['role']=='stage4' and s['stalled']],key=lambda s:-s['max_normal_load_n'])[:2]
    fig,axes=plt.subplots(3,2,figsize=(14,10),squeeze=False)
    for col,s in enumerate(selected):
        rr=[r for r in raw[s['trajectory_id']] if 6<=r['time_s']<=10];ts=[r['time_s'] for r in rr]
        axes[0,col].plot(ts,[r['normal_load_n'] for r in rr],label='Simulator normal load (privileged)')
        axes[0,col].plot(ts,[r['wrist_force_n'] for r in rr],label='Wrist force norm')
        axes[0,col].axhline(10,color='gray',ls=':');axes[0,col].set(title=s['trajectory_id'].split('/')[-1],ylabel='Load / force (N)');axes[0,col].legend(fontsize=8)
        axes[1,col].plot(ts,[r['command_depth_mm'] for r in rr],label='Commanded depth')
        axes[1,col].plot(ts,[r['depth_mm'] for r in rr],label='Actual depth');axes[1,col].set_ylabel('Depth (mm)');axes[1,col].legend(fontsize=8)
        pp=sorted([r for r in predictions if r['trajectory_id']==s['trajectory_id'] and r['horizon_s']==h and r['model']=='progress_wrench_motion'],key=lambda r:r['time_s'])
        axes[2,col].plot([r['time_s'] for r in pp],[r['eta_raw'] for r in pp],label='Future η target')
        axes[2,col].plot([r['time_s'] for r in pp],[r['predicted_eta_raw'] for r in pp],label='Predicted η');axes[2,col].axhline(.5,color='gray',ls=':');axes[2,col].legend(fontsize=8)
        axes[2,col].set(xlabel='Time (s)',ylabel='η, H=0.5 s')
        for ax in axes[:,col]:
            ax.axvline(s['stall_confirmation_s'],color='C3',ls='--');ax.set_xlim(6,10)
    fig.suptitle('Hidden-load examples chosen by load, not prediction quality; red line = stall confirmation')
    fig.tight_layout();fig.savefig(output/'hidden_contact_examples.png',dpi=150);plt.close(fig)


def report(output,settings,summaries,windows,metrics,warnings,comparisons,hidden,validation):
    def fmt(x):return '—' if x is None else f'{x:.3f}'
    counts=Counter(s['role'] for s in summaries)
    lines=['# Contact Productivity — offline proof of concept','',
        f"{len(summaries)} trajectories: {counts['development']} development, {counts['quarantined_shared_path']} quarantined earlier-stage shared paths, and {counts['stage4']} Stage 4 stress-test trajectories.", '',
        '## Target and causal feature contract','',
        '`eta(t,H) = [depth(t+H)-depth(t)] / [command_depth(t+H)-command_depth(t)]`, with H = 0.25, 0.5, 1.0 s. Both progress terms are in mm. Future endpoints must remain in the logged insertion phase and commanded progress must be at least 0.1 mm. The signed raw ratio is the regression target and all primary metrics/predictions are unclipped. A separately named [0,1]-clipped diagnostic is saved but never fitted or scored. Ratios above 1 and below 0 are retained.', '',
        'Features use only the preceding 0.5 s including the current sample, at 0.1 s stride. They include causal force/torque statistics and derivatives, actual/commanded depth and rates, trailing productivity, progress deficit, linear/angular velocity, lateral position and sign-invariant spatial orientation. Future commanded progress is a target denominator only; it is not an input. No experiment ID, severity, time-to-stall, outcome, future target, or normal load enters a model.', '',
        'Primary wrench features use the logged wrist force/torque: raw wrist norms and world-frame wrench transported by the logger to the peg base. The requested fx/fy/fz/taux/tauy/tauz peg-contact columns are retained in separately named contact-wrench comparisons. These true contact signals are simulator-derived and must not be mistaken for a deployable wrist sensor. Actual peg pose is assumed observable. No contact load, contact count, penetration or contact-power proxy is a predictor.', '',
        '## Leakage controls and estimator','',
        'Stage 4 is never used for fitting, scaling, hyperparameter selection or alarm-threshold calibration. All earlier-stage members of its 13 path groups are quarantined, including all Stage 3 paths. They are analyzed and predicted separately. Keeping all Stage 3 paths in training would contradict a group-disjoint Stage 4 test. Centered repeats form one group; repeated severities/retries remain in their path group.', '',
        'Numerically valid trajectories are retained even when the archived grasp-retention flag is false: actual peg motion is the target, and slipping is not silently excluded. That flag remains in trajectory_summary.csv. History must be fully after actual entry. For depth-ramp cases it must begin beyond ramp onset +0.1 mm; this removes common aligned pre-ramp histories. Exact rounded physical-history duplicates across distinct groups are excluded. Group weights give each group equal total influence, including duplicate centered controls. Five outer development folds provide out-of-fold predictions; three inner group folds select ridge alpha from 0.001, 0.01, 0.1. Scaling and fitting occur inside each training fold. Final stress-test fits use only eligible development data. models.json and split_audit.json make the input lists and split decisions reviewable.', '',
        'All seven fitted comparisons use the same standardized linear ridge estimator over engineered histories. The five primary comparisons are force-only norm history, torque-only norm history, current progress, wrist-wrench history, and progress+wrist-wrench+motion. Two extra comparisons substitute true contact wrench. Persistence predicts future productivity using trailing actual/commanded productivity without fitting.', '',
        '## Before-stall interpretation','',
        'The existing stall predicate is reconstructed exactly: over the trailing 0.5 s, command progress ≥0.5 mm and actual progress <0.1 mm while inserting. Its first confirmation time agrees with every archived stalled flag. No model is trained or evaluated on an already-confirmed stalled state. Post-confirmation target rows are retained only for descriptive plots.', '',
        'Confirmation is delayed by a lookback window. Warnings are therefore assessed both before confirmation and strictly before the start of that 0.5 s window. The latter is a conservative timing audit, not a measured physical stall-onset timestamp. Two consecutive predictions 0.1 s apart must trigger; the second timestamp is the alert time. Leads are measured on complete at-risk trajectories, not by treating overlapping windows as independent experiments.', '',
        'Semantic degradation uses predicted eta ≤0.5 (true target eta <0.5). Separately, alarm thresholds are calibrated for at most 10% group-weighted non-stalled-trajectory alerts on training-only inner/outer out-of-fold predictions. A non-stall alert is not automatically a false productivity alert: insertion can degrade without satisfying the formal stall predicate. Fixed wrist-force thresholds of 1, 2, 5, 10 and 20 N are also evaluated. Stage 4 thresholds remain frozen.', '',
        '## Prediction results','',
        'Metrics below weight groups equally. R² is against the evaluation-set mean, not a trained baseline. Compare MAE with current progress and persistence before attributing performance to wrench history. No confidence interval treats windows as independent.', '',
        '| Split | H (s) | Model | MAE | RMSE | R² | Degradation AUC |', '|---|---:|---|---:|---:|---:|---:|']
    for r in metrics:
        if r['evaluation'] in ('development_oof','stage4') and r['model'] in (*PRIMARY,'persistence'):
            lines.append(f"| {r['evaluation']} | {r['horizon_s']} | {r['model']} | {fmt(r['mae'])} | {fmt(r['rmse'])} | {fmt(r['r2'])} | {fmt(r['degradation_auc'])} |")
    lines+=['','## Added value of wrench/motion','',
        'Paired group-bootstrap 95% intervals for baseline MAE minus combined-model MAE. Positive means the combined model improves; intervals spanning zero are inconclusive. These are conditional on the fitted model and these selected paths, not training-population confidence bounds.','',
        '| Split | H | Comparison baseline | MAE improvement | 95% interval |','|---|---:|---|---:|---|']
    for r in comparisons:
        lines.append(f"| {r['evaluation']} | {r['horizon_s']} | {r['comparison']} | {fmt(r['mae_improvement'])} | [{fmt(r['ci_low'])}, {fmt(r['ci_high'])}] |")
    lines+=['','## Warning timing','',
        'Primary timing comparison shown at H=0.5 s. All horizons, fixed thresholds and individual detected/missed trajectories are in warning_summary.csv and warning_events.csv. Lead medians include detected cases only; the detection denominator is shown.','',
        '| Split | Model | Alarm | Before confirmation | Before lookback starts | Median lead to confirmation (s) | Non-stall alerts |',
        '|---|---|---|---:|---:|---:|---:|']
    for r in warnings:
        if r['horizon_s']==.5 and r['evaluation'] in ('development_oof','stage4') and r['model'] in (*PRIMARY,'persistence','force_threshold') and (r['mode'] in ('calibrated','semantic_eta_0.5','force_5N','force_20N')):
            lines.append(f"| {r['evaluation']} | {r['model']} | {r['mode']} | {r['detected_before_confirmation']}/{r['stalled_trajectories']} | {r['detected_before_lookback']}/{r['stalled_trajectories']} | {fmt(r['median_lead_to_confirmation_s'])} | {r['nonstall_alerts']}/{r['nonstalled_trajectories']} |")
    lines+=['','### Stage 4 timing across horizons','',
        'Calibrated thresholds correspond to different predicted eta levels; they are not all tests of eta <0.5. In particular, the 1 s detector can warn on mild predicted degradation. The 0.5 s lookback boundary remains a timing audit, not known physical onset.', '',
        '| H (s) | Alarm | Predicted eta threshold | Before confirmation | Before lookback | Median confirmation lead (s) | Non-stall alerts |',
        '|---|---|---:|---:|---:|---:|---:|']
    fitted=json.loads((output/'models.json').read_text())
    for r in warnings:
        if r['evaluation']=='stage4' and r['model']=='progress_wrench_motion' and r['mode'] in ('calibrated','semantic_eta_0.5'):
            th=.5 if r['mode']=='semantic_eta_0.5' else -fitted[f"{r['horizon_s']}:progress_wrench_motion"]['alarm_score_threshold']
            lines.append(f"| {r['horizon_s']} | {r['mode']} | {th:.3f} | {r['detected_before_confirmation']}/{r['stalled_trajectories']} | {r['detected_before_lookback']}/{r['stalled_trajectories']} | {fmt(r['median_lead_to_confirmation_s'])} | {r['nonstall_alerts']}/{r['nonstalled_trajectories']} |")
    lines+=['','## Hidden contact load','',
        'Predefined hidden-load subset: trailing mean simulator normal load ≥10 N while current wrist force ≤5 N. Load is privileged analysis only, never supplied to fitting, tuning, scaling or alarm calibration. This probes whether opposing contact forces or moments can be missed by a scalar wrist-force threshold; it does not identify a unique physical jamming mechanism.','',
        'The primary mean-load ≥10 N subset has no eligible pre-confirmation prediction windows in this dataset. Its qualifying productivity windows occur after confirmation. hidden_state_summary.csv documents this timing, and hidden_states.csv retains high-load insertion/hold states with undefined productivity explicitly marked. Instantaneous ≥10 N and mean ≥5 N comparisons below are descriptive sensitivity checks added after the coverage audit; no model or threshold is retuned.', '',
        '| Split | H | Model | Load definition | Hidden windows / groups | MAE | Mean eta |', '|---|---:|---|---|---:|---:|---:|']
    for r in hidden:
        if r['hidden_load'] and r['evaluation'] in ('development_oof','stage4') and r['model'] in ('force_only','current_progress','progress_wrench_motion'):
            lines.append(f"| {r['evaluation']} | {r['horizon_s']} | {r['model']} | {r['definition']} | {r['windows']} / {r['groups']} | {fmt(r['mae'])} | {fmt(r['eta_mean'])} |")
    stage=next(r for r in metrics if r['evaluation']=='stage4' and r['model']=='progress_wrench_motion' and r['horizon_s']==.5)
    gain=next(r for r in comparisons if r['evaluation']=='stage4' and r['comparison']=='force_only' and r['horizon_s']==.5)
    prog=next(r for r in comparisons if r['evaluation']=='stage4' and r['comparison']=='current_progress' and r['horizon_s']==.5)
    timing=next(r for r in warnings if r['evaluation']=='stage4' and r['model']=='progress_wrench_motion' and r['horizon_s']==.5 and r['mode']=='semantic_eta_0.5')
    lines+=['','## Decision','',
        f"At the prespecified 0.5 s horizon, the combined model has Stage 4 MAE {fmt(stage['mae'])}, R² {fmt(stage['r2'])}, and degradation AUC {fmt(stage['degradation_auc'])}. Its MAE change relative to force alone is {fmt(gain['mae_improvement'])}; relative to current progress it is {fmt(prog['mae_improvement'])} (positive is improvement).", '',
        f"The semantic observer alerts before confirmation in {timing['detected_before_confirmation']}/{timing['stalled_trajectories']} at-risk stalled stress-test trajectories and before the confirmation lookback begins in {timing['detected_before_lookback']}/{timing['stalled_trajectories']}; it also alerts in {timing['nonstall_alerts']}/{timing['nonstalled_trajectories']} non-stalled trajectories.", '',
        'These results '+('support a limited shadow-mode online productivity observer for validation, with no authority to change robot actions.' if stage['r2'] is not None and stage['r2']>0 and gain['mae_improvement']>0 and timing['detected_before_lookback']>0 else 'do not yet justify a validated online productivity observer; address the reported generalization and early-warning limitations first.'), '',
        'The raw combined-model forecasts can substantially overshoot below zero on Stage 4; clipping would hide part of this extrapolation error and is not used in the reported scores. The tested ridge models do not establish that no nonlinear predictive relationship exists.', '',
        'An incremental benefit from wrench is only supported if the combined model improves over progress/persistence, not merely over force. This small, deterministic dataset uses a fixed scripted insertion duration and selected geometries; forecasts describe that recorded continuation. No action-conditioned safety, arbitrary command schedules, calibrated probability, real-robot transfer, or controller performance is established. No online observer or controller was implemented.', '',
        '## Validation and outputs','',
        f"Input hash and file-metadata audit: {validation['existing_files_checked']} pre-existing output files checked; {len(validation['existing_files_changed'])} changed. All {len(summaries)} archived stall flags matched reconstruction. Predictor lists exclude simulator normal load. Split audit confirms no Stage 4 group in fitting/tuning and no group crossing an outer fold.", '',
        'Raw labels and causal features: trajectory_summary.csv / window_features.csv. Out-of-fold and held-out forecasts: predictions.csv. Primary stress-test metrics: stage4_results.csv. Strict pre-lookback metrics: pre_lookback_metrics.csv. Hidden-load comparisons: hidden_load_results.csv. All model coefficients, training transforms, source hashes, and analysis settings are saved.', '',
        '![Eta over time and depth](eta_vs_time_depth.png)', '', '![Stage 4 pre-stall predictions](predicted_eta_before_stall.png)', '',
        '![Force versus productivity](force_vs_productivity.png)', '', '![Hidden load examples](hidden_contact_examples.png)', '']
    (output/'report.md').write_text('\n'.join(lines))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outputs-root',type=Path,default=Path('outputs'))
    parser.add_argument('--output',type=Path,default=Path('outputs/Contact-Productivity-v1'))
    args=parser.parse_args();root=args.outputs_root.resolve();out=args.output.resolve()
    if out.exists():parser.error('Choose a new output directory; existing outputs are never overwritten')
    old={str(p):(p.stat().st_size,p.stat().st_mtime_ns) for p in root.rglob('*') if p.is_file()}
    settings=Settings();out.mkdir(parents=True,exist_ok=False)
    write_json(out/'settings.json',asdict(settings))
    windows,summaries,inputs,sources,rejections,raw=load_dataset(root,settings)
    table(out/'trajectory_summary.csv',summaries);table(out/'window_features.csv',windows)
    write_json(out/'feature_sets.json',inputs);write_json(out/'source_files.json',sources)
    predictions,metrics=train_evaluate(windows,inputs,settings,out)
    details,warnings=warning_results(predictions,settings,summaries);comparisons=paired_comparisons(predictions,settings);hidden=hidden_analysis(predictions)
    hidden_rows,hidden_summary=hidden_states(windows,summaries,raw,settings)
    table(out/'hidden_states.csv',hidden_rows);table(out/'hidden_state_summary.csv',hidden_summary)
    strict=[]
    for evaluation in ('development_oof','stage4'):
        for h in settings.horizons:
            for name in (*PRIMARY,'persistence'):
                rr=[r for r in predictions if r['evaluation']==evaluation and r['horizon_s']==h and r['model']==name and r['before_stall_lookback']]
                if rr:strict.append(dict(evaluation=evaluation,horizon_s=h,model=name,**metric(rr,np.array([r['predicted_eta_raw'] for r in rr]))))
    for file,rows in [('predictions.csv',predictions),('metrics.csv',metrics),('stage4_results.csv',[r for r in metrics if r['evaluation']=='stage4']),
                      ('warning_events.csv',details),('warning_summary.csv',warnings),('paired_comparisons.csv',comparisons),('hidden_load_results.csv',hidden),('pre_lookback_metrics.csv',strict)]:table(out/file,rows)
    plots(out,windows,predictions,summaries,raw)
    validation=dict(existing_files_checked=len(old),existing_files_changed=[p for p,stat in old.items() if not Path(p).is_file() or (Path(p).stat().st_size,Path(p).stat().st_mtime_ns)!=stat],
        consumed_files_checked=len(sources),consumed_hashes_unchanged=all(digest(Path(p))==sha for p,sha in sources.items()),
        analysis_sha256=digest(Path(__file__)),trajectories=len(summaries),windows=len(windows),rejections=rejections,
        cohort_counts=dict(Counter(s['role'] for s in summaries)),no_isaac_sim_launched=True,
        predictor_privileged_load_excluded=all('normal_load' not in k for keys in inputs.values() for k in keys))
    assert not validation['existing_files_changed'] and validation['consumed_hashes_unchanged']
    write_json(out/'validation.json',validation)
    report(out,settings,summaries,windows,metrics,warnings,comparisons,hidden,validation)
    archive=out/'source';archive.mkdir()
    for file in ('research/contact_productivity.py','research/future_stall.py','research/contact_response.py'):
        source=Path(__file__).resolve().parents[1]/file;target=archive/Path(file).name;target.write_bytes(source.read_bytes())
    print(json.dumps(validation,indent=2),flush=True)


if __name__=='__main__':main()
