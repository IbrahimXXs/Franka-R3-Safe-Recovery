"""Offline, observational contact-response analysis of saved FORGE insertion references.

No Isaac Sim imports, commanded-motion inputs, label rewriting, or active probing.
Run: python -m research.contact_response --outputs-root outputs --output outputs/Contact-Response-Offline-v1
"""
import argparse
import ast
from collections import Counter, defaultdict
import csv
from dataclasses import asdict, dataclass, replace
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np


@dataclass(frozen=True)
class Settings:
    window_s: float = .25
    stride_s: float = .1
    lag_steps: int = 1
    length_scale_m: float = .01
    motion_floor_m: float = 1e-6
    wrench_floor_n: float = 1e-4
    rank_relative_tolerance: float = .01
    ridge_relative: float = .001
    bootstrap_samples: int = 400
    seed: int = 20260915

    def validate(self):
        for k in ('window_s','stride_s','length_scale_m','motion_floor_m','wrench_floor_n','rank_relative_tolerance','ridge_relative'):
            if not np.isfinite(getattr(self,k)) or getattr(self,k)<=0:raise ValueError(f'Invalid {k}')
        if self.rank_relative_tolerance>=1:raise ValueError('Rank tolerance must be below one')
        if type(self.lag_steps) is not int or self.lag_steps<1:raise ValueError('Lag must be a positive integer')
        if self.bootstrap_samples<0:raise ValueError('Bootstrap count cannot be negative')
        return self


MODELS={'full':((0,1,2,3,4,5),(0,1,2,3,4,5)),
        'translation':((0,1,2),(0,1,2)),
        'lateral_tilt':((0,1,3,4),(0,1,2,3,4))}
POSE_NAMES=('dx','dy','dz','dtheta_x','dtheta_y','dtheta_z')
WRENCH_NAMES=('Fx','Fy','Fz','tau_x','tau_y','tau_z')
BASELINES=('force_norm','torque_norm','normal_load','progress_rate','abs_dforce_dt','rms_dforce_dt','abs_dforce_ddepth')
COMPARISONS=('full_sensitivity','translation_sensitivity','lateral_tilt_sensitivity',*BASELINES)


def quat_increment(q0,q1):
    """World/spatial SO(3) log(q1 * conjugate(q0)), scalar-first quaternions."""
    q0=np.asarray(q0,dtype=float);q1=np.asarray(q1,dtype=float)
    n0=np.linalg.norm(q0,axis=-1,keepdims=True);n1=np.linalg.norm(q1,axis=-1,keepdims=True)
    if not np.all(np.isfinite(q0)) or not np.all(np.isfinite(q1)) or np.any(n0<1e-12) or np.any(n1<1e-12):
        raise ValueError('Invalid quaternion')
    q0=q0/n0;q1=q1/n1
    a=q1[...,0];b=q0[...,0];v=q1[...,1:];u=-q0[...,1:]
    scalar=a*b-np.sum(v*u,axis=-1)
    vector=a[...,None]*u+b[...,None]*v+np.cross(v,u)
    sign=np.where(scalar<0,-1.,1.);scalar=scalar*sign;vector=vector*sign[...,None]
    norm=np.linalg.norm(vector,axis=-1)
    angle=2*np.arctan2(norm,np.clip(scalar,0.,1.))
    factor=np.divide(angle,norm,out=np.full_like(norm,2.),where=norm>1e-12)
    return vector*factor[...,None]


def transport_wrench(position,wrench,anchor):
    """World wrench at a common fixed window anchor: tau_anchor=tau_peg+(p-anchor)xF."""
    w=np.array(wrench,dtype=float,copy=True)
    w[...,3:]+=np.cross(np.asarray(position)-anchor,w[...,:3])
    return w


def increments(position,quaternion,wrench,lag=1):
    """Nonoverlapping measured pose/wrench differences at a fixed window torque origin."""
    indices=np.arange(0,len(position),lag)
    if len(indices)<2:raise ValueError('Too few samples')
    p=np.asarray(position)[indices];q=np.asarray(quaternion)[indices]
    w=transport_wrench(p,np.asarray(wrench)[indices],p[0])
    x=np.column_stack((np.diff(p,axis=0),quat_increment(q[:-1],q[1:])))
    return x,np.diff(w,axis=0),indices


def model_scales(name,length):
    ii,oo=MODELS[name]
    sx=np.array([1.,1.,1.,length,length,length])[list(ii)]
    sy=np.array([1.,1.,1.,1/length,1/length,1/length])[list(oo)]
    return sx,sy


def excitation(x,settings):
    _,s,vt=np.linalg.svd(x,full_matrices=False)
    threshold=max(s[0]*settings.rank_relative_tolerance,settings.motion_floor_m*math.sqrt(len(x)))
    rank=int(np.sum(s>threshold));numeric=int(np.sum(s>s[0]*max(x.shape)*np.finfo(float).eps)) if s[0]>0 else 0
    return dict(rank=rank,numerical_rank=numeric,
        condition=float(s[0]/s[-1]) if s[-1]>0 and numeric==x.shape[1] else None,
        excited_condition=float(s[0]/s[rank-1]) if rank else None,
        singular_values=s,excited_basis=vt[:rank],threshold=threshold)


def ridge_fit(x,y,relative):
    """Zero-intercept ridge; regularization scales with the measured input energy."""
    alpha=relative*float(np.sum(x*x))/x.shape[1]
    if alpha<=0:raise ValueError('Cannot fit a zero-motion design')
    return np.linalg.solve(x.T@x+alpha*np.eye(x.shape[1]),x.T@y),alpha


def skill(error,baseline_error,minimum_baseline=1e-20):
    return float(1-error/baseline_error) if baseline_error>minimum_baseline else None


def local_model(xraw,yraw,name,settings):
    ii,oo=MODELS[name];sx,sy=model_scales(name,settings.length_scale_m)
    x=xraw[:,ii]*sx;y=yraw[:,oo]*sy
    motion=float(np.sqrt(np.mean(np.sum(x*x,axis=1))))
    response=float(np.sqrt(np.mean(np.sum(y*y,axis=1))))
    info=excitation(x,settings)
    result=dict(accepted=False,motion_rms_m=motion,wrench_change_rms_n=response,
        resolved_wrench_change=response>settings.wrench_floor_n,
        effective_rank=info['rank'],numerical_rank=info['numerical_rank'],
        input_dimension=len(ii),condition=info['condition'],excited_condition=info['excited_condition'],
        full_excitation=info['rank']==len(ii),sensitivity=None,observed_gain=None,
        validation_skill_zero=None,validation_skill_drift=None,validation_rmse_n=None,validation_coverage=None,
        validation_delta_w_rms_n=None)
    if motion<=settings.motion_floor_m or info['rank']==0:
        result['rejection_reason']='near_zero_or_unresolved_motion';return result,None
    train_count=int(.65*len(x));test_start=train_count+1  # gap prevents a shared difference endpoint
    if train_count<len(ii)+2 or len(x)-test_start<3:
        result['rejection_reason']='insufficient_pairs_for_temporal_validation';return result,None
    fitted,alpha=ridge_fit(x,y,settings.ridge_relative)
    train,_=ridge_fit(x[:train_count],y[:train_count],settings.ridge_relative)
    predicted=x[test_start:]@train;truth=y[test_start:]
    error=float(np.mean((truth-predicted)**2));zero=float(np.mean(truth**2))
    drift=float(np.mean((truth-y[:train_count].mean(axis=0))**2))
    train_info=excitation(x[:train_count],settings);basis=train_info['excited_basis']
    projected=x[test_start:]@basis.T@basis
    total=float(np.sum(x[test_start:]**2))
    result.update(accepted=True,rejection_reason='',sensitivity=float(np.linalg.norm(y)/np.linalg.norm(x)),
        observed_gain=float(np.linalg.norm(fitted.T@info['excited_basis'].T,ord=2)),
        validation_skill_zero=skill(error,zero,settings.wrench_floor_n**2/len(oo)),
        validation_skill_drift=skill(error,drift,settings.wrench_floor_n**2/len(oo)),
        validation_delta_w_rms_n=math.sqrt(zero*len(oo)),
        validation_rmse_n=math.sqrt(error),validation_coverage=float(np.sum(projected**2)/total) if total else None)
    # y_scaled = x_scaled B. SI G maps raw metres/radians to N/Nm.
    g_si=(sx[:,None]*fitted/sy[None,:]).T
    matrix=dict(model=name,input_names=[POSE_NAMES[i] for i in ii],output_names=[WRENCH_NAMES[i] for i in oo],
        G_SI=g_si,G_scaled=fitted.T,regularization_alpha=alpha,
        singular_values_m=info['singular_values'],excitation_threshold=info['threshold'],
        excited_input_basis_scaled=info['excited_basis'],effective_rank=info['rank'],
        full_excitation=result['full_excitation'],
        interpretation='regularized empirical fit; coefficients outside excited subspace are not identifiable',
        validation_train_pairs=train_count,validation_test_start_pair=test_start,validation_test_pairs=len(x)-test_start)
    return result,matrix


def slope(time,values):
    t=np.asarray(time)-np.mean(time);v=np.asarray(values)
    denom=float(t@t)
    return float(t@(v-v.mean())/denom) if denom>0 else None


def window_analysis(data,start,end,settings,matrices=False):
    p=data['position'][start:end+1];q=data['quaternion'][start:end+1];w=data['wrench'][start:end+1]
    time=data['time'][start:end+1];depth=data['depth'][start:end+1]
    x,y,idx=increments(p,q,w,settings.lag_steps)
    force=np.linalg.norm(w[:,:3],axis=1);torque=np.linalg.norm(w[:,3:],axis=1)
    rate=slope(time,depth/1000)
    dforce=slope(time,force)
    dz=np.diff(depth[idx])/1000;df=np.diff(force[idx])
    # A scalar directional force-vs-depth fit, rejected when depth excitation is unresolved.
    dfddepth=float(dz@df/(dz@dz)) if np.sqrt(np.mean(dz*dz))>settings.motion_floor_m else None
    result=dict(start_row=start,end_row=end,time_s=float(time[-1]),start_time_s=float(time[0]),
        depth_mm=float(depth[-1]),start_depth_mm=float(depth[0]),
        contact_fraction=float(np.mean(data['contact'][start:end+1]>0)),
        positive_load_fraction=float(np.mean(data['normal'][start:end+1]>0)),
        force_norm=float(np.median(force)),torque_norm=float(np.median(torque)),
        normal_load=float(np.median(data['normal'][start:end+1])),progress_rate=rate,
        abs_dforce_dt=abs(dforce),abs_dforce_ddepth=abs(dfddepth) if dfddepth is not None else None,
        rms_dforce_dt=float(np.sqrt(np.mean(np.sum((y[:,:3]/np.diff(time[idx])[:,None])**2,axis=1)))),
        dforce_dt=dforce,dforce_ddepth=dfddepth)
    models=[]
    for name in MODELS:
        features,matrix=local_model(x,y,name,settings)
        result.update({f'{name}_{k}':v for k,v in features.items()})
        if matrices and matrix is not None:models.append(matrix)
    # Endpoint ratio complements the primary RMS-increment ratio, which can be noise-sensitive.
    sx,sy=model_scales('full',settings.length_scale_m)
    endpoint_x=np.r_[p[-1]-p[0],quat_increment(q[0],q[-1])]*sx
    transported=transport_wrench(p,w,p[0]);endpoint_y=(transported[-1]-transported[0])*sy
    result['endpoint_sensitivity']=float(np.linalg.norm(endpoint_y)/np.linalg.norm(endpoint_x)) if np.linalg.norm(endpoint_x)>settings.motion_floor_m else None
    return result,models


def json_safe(value):
    if isinstance(value,np.ndarray):return json_safe(value.tolist())
    if isinstance(value,np.generic):return json_safe(value.item())
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:json_safe(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [json_safe(v) for v in value]
    return value


def write_json(path,value):path.write_text(json.dumps(json_safe(value),indent=2,allow_nan=False)+'\n')


def table(path,rows):
    if not rows:return
    with path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        writer.writeheader();writer.writerows(json_safe(r) for r in rows)


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def discover(root):
    selected=[];excluded=[]
    for path in sorted(root.rglob('study.json')):
        # Only ledgers identify cohorts; no glob over recovery or replay CSVs.
        m=json.loads(path.read_text())
        plan=m.get('case_plan')
        phase2b=isinstance(plan,dict) and plan.get('schema')=='Forge-Phase2B-depth-drift-v1'
        # New factor studies reuse the FORGE ledger envelope. Their geometry
        # and command factors are outside this legacy Phase 2A/2B analysis.
        phase2a=m.get('mode')=='collect' and plan is None
        if m.get('study')!='Forge-Controlled-Phase2-v1' or not (phase2a or phase2b):
            excluded.append(str(path.parent));continue
        selected.append((path,m,'Phase2B' if phase2b else 'Phase2A'))
    return selected,excluded


def verify_frame(directory,manifest):
    scene=json.loads((directory/'scene.json').read_text())
    cfg=json.loads((directory/'config.json').read_text())
    transform=np.asarray(ast.literal_eval(scene['socket_mesh_world_transform']),dtype=float)
    if transform.shape!=(4,4) or not np.allclose(transform[:3,:3],np.eye(3),atol=1e-7):
        raise ValueError('Unsupported non-world-aligned fixture; do not mix coordinate frames')
    if scene.get('stage_meters_per_unit')!=1. or cfg['task']['fixed_asset_init_orn_deg']!=0. or cfg['task']['fixed_asset_init_orn_range_deg']!=0.:
        raise ValueError('Fixture orientation/unit assumption not verified')
    for name in ('forge_backend.py','contact.py'):
        source=directory/'source'/name
        if digest(source)!=manifest['sources'][name]:raise ValueError(f'Archived logger source mismatch: {source}')
    source=(directory/'source/forge_backend.py').read_text()
    for evidence in ('rel=quat_apply_inverse(e.fixed_quat,e.held_pos-e.fixed_pos)',
                     "origin=e._held_asset.data.root_pos_w[0]", "c=self.contacts.read(origin)"):
        if evidence not in source:raise ValueError('Unrecognized pose/wrench logger; review frame mapping')


def load_reference(path):
    with path.open() as f:rows=list(csv.DictReader(f))
    if len(rows)<3:raise ValueError('Too few samples')
    def columns(keys):return np.array([[float(r[k]) for k in keys] for r in rows])
    p=columns(('tip_x_mm','tip_y_mm','depth_mm'))/1000;p[:,2]*=-1
    data=dict(position=p,quaternion=columns(('qw','qx','qy','qz')),
              wrench=columns(('fx','fy','fz','taux','tauy','tauz')),
              time=columns(('time_s',))[:,0],depth=columns(('depth_mm',))[:,0],
              normal=columns(('normal_load_n',))[:,0],contact=columns(('contact_count',))[:,0],
              phase=np.array([r['phase'] for r in rows]))
    if not all(np.isfinite(v).all() for k,v in data.items() if k!='phase'):raise ValueError('Nonfinite actual state or wrench')
    if np.any(np.diff(data['time'])<=0):raise ValueError('Non-monotone clock')
    quat_increment(data['quaternion'],data['quaternion'])
    return data


def aggregate(rows,prefix=''):
    out={}
    for key in COMPARISONS:
        values=[r[key] for r in rows if r.get(key) is not None and np.isfinite(r[key])]
        out[prefix+key]=float(np.median(values) if key=='progress_rate' else np.percentile(values,90)) if values else None
    return out


def auc(y,score,weights=None):
    """Weighted ROC AUC with exact ties; larger scores predict the difficult group."""
    y=np.asarray(y,dtype=bool);score=np.asarray(score);weights=np.ones(len(y)) if weights is None else np.asarray(weights)
    order=np.argsort(score);y=y[order];score=score[order];weights=weights[order]
    positive=weights[y].sum();negative=weights[~y].sum()
    if positive<=0 or negative<=0:return None
    numerator=below=0.;i=0
    while i<len(y):
        j=i+1
        while j<len(y) and score[j]==score[i]:j+=1
        pos=weights[i:j][y[i:j]].sum();neg=weights[i:j][~y[i:j]].sum()
        numerator+=pos*(below+.5*neg);below+=neg;i=j
    return float(numerator/(positive*negative))


def compare_features(summaries,settings):
    results=[]
    for cohort in ('all','Phase2A','Phase2B'):
        for task in ('stalled_vs_success','difficult_vs_success'):
            group=[r for r in summaries if r.get('analysis_eligible') and (cohort=='all' or r['cohort']==cohort)
                   and (task!='stalled_vs_success' or r['outcome_group']!='other_unsuccessful')]
            for scope in ('whole_','band10_15_'):
                for feature in COMPARISONS:
                    for baseline in ('force_norm',):
                        valid=[r for r in group if r.get(scope+feature) is not None and r.get(scope+baseline) is not None]
                        if not valid:continue
                        y=np.array([r['outcome_group']!='successful_nonstalled' for r in valid])
                        score=np.array([r[scope+feature] for r in valid])*(-1 if feature=='progress_rate' else 1)
                        ref=np.array([r[scope+baseline] for r in valid])
                        groups=np.array([r['split_group_id'] for r in valid]);unique,inverse,counts=np.unique(groups,return_inverse=True,return_counts=True)
                        weights=1/counts[inverse];point=auc(y,score,weights);reference=auc(y,ref,weights)
                        if point is None:continue
                        rng=np.random.default_rng(settings.seed);boot=[];deltas=[]
                        for _ in range(settings.bootstrap_samples):
                            multiplicities=np.bincount(rng.integers(0,len(unique),len(unique)),minlength=len(unique))
                            w=weights*multiplicities[inverse];a=auc(y,score,w);b=auc(y,ref,w)
                            if a is not None and b is not None:boot.append(a);deltas.append(a-b)
                        ci=np.percentile(boot,[2.5,97.5]).tolist() if boot else [None,None]
                        delta_ci=np.percentile(deltas,[2.5,97.5]).tolist() if deltas else [None,None]
                        results.append(dict(cohort=cohort,task=task,scope=scope.rstrip('_'),feature=feature,
                            trajectories=len(valid),positive_trajectories=int(y.sum()),groups=len(unique),
                            auc=point,auc_low=ci[0],auc_high=ci[1],force_auc_same_subset=reference,
                            delta_vs_force=point-reference,delta_low=delta_ci[0],delta_high=delta_ci[1],
                            interpretation='descriptive grouped association; no held-out classifier or causal inference'))
    return results


def snapshot(root,exclude):
    return {str(p.resolve()):(p.stat().st_size,p.stat().st_mtime_ns) for p in root.rglob('*')
            if p.is_file() and not p.resolve().is_relative_to(exclude)}


def run(root,output,settings):
    root=Path(root).resolve();output=Path(output).resolve();settings.validate()
    if output.exists():raise ValueError('Choose a new output folder; existing outputs are never overwritten')
    before=snapshot(root,output);studies,excluded=discover(root)
    if not studies:raise ValueError('No Phase 2A/2B FORGE study ledgers found')
    output.mkdir(parents=True)
    configs={'primary':settings,'length_5mm':replace(settings,length_scale_m=.005),
             'length_20mm':replace(settings,length_scale_m=.02),'lag_2_steps':replace(settings,lag_steps=2),
             'window_500ms':replace(settings,window_s=.5)}
    window_rows=[];summaries=[];robust_windows=defaultdict(list);robust_summaries=defaultdict(list)
    hashes={};study_counts=[];input_hashes={};configuration_seen={};matrix_path=output/'local_models.jsonl'
    with matrix_path.open('w') as matrix_file:
        for manifest_path,m,cohort in studies:
            directory=manifest_path.parent;verify_frame(directory,m)
            for path in (manifest_path,directory/'scene.json',directory/'config.json',directory/'source/forge_backend.py',directory/'source/contact.py'):
                hashes[str(path)]=digest(path)
            print(f'Analyzing {directory.name}: {len(m["attempts"])} references',flush=True)
            count=0
            for a in m['attempts']:
                folder=a.get('folder','')
                if not folder or Path(folder).name!=folder:raise ValueError('Invalid reference folder')
                path=directory/folder/'insertion.csv';metrics=a.get('metrics',{})
                group=a.get('split_group_id',f"slot{a['slot']:03d}")
                group=('B:' if cohort=='Phase2B' else 'A:')+group
                if a.get('sample_role')=='repeatability_control' or a['family']=='centered':group='all_centered_controls'
                base=dict(study=directory.name,cohort=cohort,trajectory_id=a['trajectory_id'],folder=folder,
                    reference=str(path),record_id=f'{directory.name}/{folder}',split_group_id=group,family=a['family'],
                    insertion_success=metrics.get('insertion_success'),stalled=metrics.get('stalled'),
                    numerically_valid=metrics.get('numerically_valid'),max_depth_mm=metrics.get('max_depth'),
                    outcome_group='stalled' if metrics.get('stalled') else 'successful_nonstalled' if metrics.get('insertion_success') else 'other_unsuccessful')
                if not path.is_file():summaries.append(dict(base,analysis_eligible=False,reason='missing_reference'));continue
                count+=1;hashes[str(path)]=digest(path)
                duplicate=input_hashes.get(hashes[str(path)]);input_hashes.setdefault(hashes[str(path)],base['record_id'])
                command_signature={k:a.get(k) for k in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg','insertion_duration_s',
                    'ramp_onset_mm','ramp_full_depth_mm','final_offset_x_mm','final_offset_y_mm','final_roll_deg','final_pitch_deg')}
                signature=json.dumps(command_signature,sort_keys=True)
                config_repeat=configuration_seen.get(signature);configuration_seen.setdefault(signature,base['record_id'])
                base.update(exact_csv_duplicate_of=duplicate,same_command_configuration_as=config_repeat)
                try:data=load_reference(path)
                except (ValueError,KeyError) as error:
                    summaries.append(dict(base,analysis_eligible=False,reason=str(error)));continue
                dt=1/m['physics_hz']
                clock_ok=bool(np.all(np.abs(np.diff(data['time'])-dt)<1e-5))
                eligible=a['status']=='complete' and metrics.get('numerically_valid') is True and clock_ok
                if not eligible:
                    summaries.append(dict(base,analysis_eligible=False,reason='incomplete_invalid_or_clock_gap'));continue
                for config_name,conf in configs.items():
                    size=round(conf.window_s/dt);stride=max(1,round(conf.stride_s/dt));rows=[]
                    for end in range(size,len(data['time']),stride):
                        start=end-size
                        if not np.all(data['phase'][start:end+1]=='insert') or np.min(data['depth'][start:end+1])<0:continue
                        values,models=window_analysis(data,start,end,conf,config_name=='primary')
                        row=dict(base,**values);rows.append(row)
                        if config_name=='primary':
                            window_rows.append(row)
                            for model in models:
                                model.update(record_id=base['record_id'],start_row=start,end_row=end,
                                             length_scale_m=conf.length_scale_m)
                                matrix_file.write(json.dumps(json_safe(model),allow_nan=False,separators=(',',':'))+'\n')
                    band=[r for r in rows if r['start_depth_mm']>=10. and r['depth_mm']<=15.]
                    summary=dict(base,analysis_eligible=True,reason='',windows=len(rows),band_windows=len(band),
                        **aggregate(rows,'whole_'),**aggregate(band,'band10_15_'))
                    for name in MODELS:
                        accepted=[r for r in rows if r[name+'_accepted']]
                        summary[name+'_accepted_windows']=len(accepted)
                        summary[name+'_fully_excited_windows']=sum(r[name+'_full_excitation'] for r in accepted)
                        summary[name+'_median_rank']=float(np.median([r[name+'_effective_rank'] for r in rows])) if rows else None
                    if config_name=='primary':summaries.append(summary)
                    robust_windows[config_name].extend(rows);robust_summaries[config_name].append(summary)
            study_counts.append(dict(study=directory.name,cohort=cohort,references=count,physics_hz=m['physics_hz']))
    table(output/'window_features.csv',window_rows);table(output/'trajectory_summary.csv',summaries)
    print('Computing grouped trajectory comparisons and uncertainty...',flush=True)
    comparisons=compare_features(summaries,settings);table(output/'feature_comparison.csv',comparisons)
    robustness=[]
    for config_name,rows in robust_windows.items():
        for name in MODELS:
            accepted=[r for r in rows if r[name+'_accepted']]
            contact=[r for r in accepted if r['positive_load_fraction']>=.5]
            eval_rows=[r for r in contact if r[name+'_validation_skill_zero'] is not None]
            vals=[r[name+'_validation_skill_zero'] for r in eval_rows]
            robustness.append(dict(configuration=config_name,model=name,windows=len(rows),accepted=len(accepted),
                contact_windows=len(contact),validation_windows=len(eval_rows),
                resolved_response_windows=sum(r[name+'_resolved_wrench_change'] for r in accepted),
                fully_excited=sum(r[name+'_full_excitation'] for r in accepted),
                loaded_fully_excited=sum(r[name+'_full_excitation'] for r in contact),
                median_rank=float(np.median([r[name+'_effective_rank'] for r in rows])) if rows else None,
                contact_median_validation_skill_zero=float(np.median(vals)) if vals else None,
                contact_fraction_beating_zero=float(np.mean(np.array(vals)>0)) if vals else None,
                contact_median_skill_drift=median_defined([r[name+'_validation_skill_drift'] for r in contact]),
                contact_median_validation_rmse_n=median_defined([r[name+'_validation_rmse_n'] for r in contact]),
                contact_median_test_coverage=median_defined([r[name+'_validation_coverage'] for r in contact])))
    table(output/'robustness.csv',robustness)
    robust_comparison=[]
    for key,rows in robust_summaries.items():
        if key=='primary':continue
        for result in compare_features(rows,replace(settings,bootstrap_samples=0)):
            if result['feature']=='full_sensitivity':robust_comparison.append(dict(configuration=key,**result))
    table(output/'robustness_comparison.csv',robust_comparison)
    after=snapshot(root,output)
    if before!=after:raise RuntimeError('An existing output changed during analysis; review before accepting results')
    if any(digest(Path(path))!=sha for path,sha in hashes.items()):raise RuntimeError('Input content changed during analysis')
    write_json(output/'source_files.json',dict(metadata_unchanged=True,existing_files_checked=len(before),
        input_sha256=hashes,script_sha256=digest(Path(__file__)),original_file_metadata=before))
    info=dict(settings=asdict(settings),robustness_settings={k:asdict(v) for k,v in configs.items()},
        command=' '.join(sys.argv),studies=study_counts,excluded_studies=excluded,
        trajectory_counts=dict(Counter(r['outcome_group'] for r in summaries if r.get('analysis_eligible'))),
        trajectories_found=len(summaries),trajectories_eligible=sum(r.get('analysis_eligible',False) for r in summaries),
        unique_split_groups=len({r['split_group_id'] for r in summaries}),windows=len(window_rows),
        stalled_split_groups=len({r['split_group_id'] for r in summaries if r['outcome_group']=='stalled'}),
        exact_duplicate_references=sum(bool(r.get('exact_csv_duplicate_of')) for r in summaries),
        repeated_command_configurations=sum(bool(r.get('same_command_configuration_as')) for r in summaries),
        input_files_unchanged=True,interpretation='offline observational contact response, not causal contact stiffness')
    write_json(output/'analysis.json',info)
    make_plots(output,window_rows,summaries,comparisons)
    write_report(output,info,robustness,comparisons,robust_comparison)
    print(json.dumps(info['trajectory_counts']),f'; {len(window_rows)} windows; output: {output}',flush=True)


def median_defined(values):
    values=[v for v in values if v is not None and np.isfinite(v)]
    return float(np.median(values)) if values else None


def make_plots(output,windows,summaries,comparisons):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors={'successful_nonstalled':'#218c74','stalled':'#c44536','other_unsuccessful':'#927542'}
    def save(fig,name):
        fig.savefig(output/(name+'.png'),dpi=160);fig.savefig(output/(name+'.pdf'));plt.close(fig)
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    for ax,name in zip(axes,MODELS):
        for group,color in colors.items():
            rows=[r for r in windows if r['outcome_group']==group and r.get(name+'_sensitivity') is not None and r[name+'_sensitivity']>0]
            ax.scatter([r['depth_mm'] for r in rows],[r[name+'_sensitivity'] for r in rows],s=3,alpha=.18,color=color,label=group)
        ax.set(xlabel='Actual depth [mm]',ylabel='RMS response ratio [N/m]',yscale='log',title=name)
        ax.grid(alpha=.2)
    axes[-1].legend(fontsize=7);save(fig,'sensitivity_vs_depth')
    fig,axes=plt.subplots(1,3,figsize=(15,4.5),constrained_layout=True)
    for ax,name in zip(axes,MODELS):
        for cohort,color in (('Phase2A','#4665a8'),('Phase2B','#bf7b24')):
            rows=[r for r in windows if r['cohort']==cohort]
            bins=np.arange(0,21,1);x=[];y=[]
            for lo in bins[:-1]:
                # Each trajectory supplies one median per depth bin.
                by=defaultdict(list)
                for r in rows:
                    if lo<=r['depth_mm']<lo+1:by[r['record_id']].append(r[name+'_effective_rank'])
                if by:x.append(lo+.5);y.append(np.median([np.median(v) for v in by.values()]))
            ax.plot(x,y,'o-',color=color,label=cohort)
        ax.axhline(len(MODELS[name][0]),ls=':',color='gray')
        ax.set(xlabel='Actual depth [mm]',ylabel='Median effective excitation rank',ylim=(-.1,6.3),title=name);ax.grid(alpha=.2)
    axes[-1].legend();save(fig,'observability_vs_depth')
    fig,ax=plt.subplots(figsize=(8,5),constrained_layout=True)
    for group,color in colors.items():
        rows=[r for r in summaries if r.get('whole_full_sensitivity') is not None and r['outcome_group']==group and r['whole_full_sensitivity']>0 and r['whole_force_norm']>0]
        ax.scatter([r['whole_force_norm'] for r in rows],[r['whole_full_sensitivity'] for r in rows],color=color,label=group,alpha=.7)
    ax.set(xlabel='Trajectory p90 window-median force norm [N]',ylabel='Trajectory p90 response ratio [N/m]',xscale='log',yscale='log',title='Whole-insertion descriptive association')
    ax.grid(alpha=.2);ax.legend(fontsize=8);save(fig,'force_vs_sensitivity')
    fig,axes=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    labels={'full_sensitivity':'Full pose response ratio','translation_sensitivity':'Translation response ratio',
            'lateral_tilt_sensitivity':'Lateral/tilt response ratio','force_norm':'Force norm','torque_norm':'Torque norm',
            'normal_load':'Normal load','progress_rate':'Progress rate (reversed)','abs_dforce_dt':'|d(force norm)/dt|',
            'rms_dforce_dt':'RMS vector dF/dt','abs_dforce_ddepth':'|d(force norm)/ddepth|'}
    for ax,scope in zip(axes,('whole','band10_15')):
        rows=[r for r in comparisons if r['cohort']=='all' and r['task']=='stalled_vs_success' and r['scope']==scope]
        for i,r in enumerate(rows):
            ax.plot([r['auc_low'],r['auc_high']],[i,i],color='#486d92')
            ax.scatter(r['auc'],i,color='#486d92')
        ax.set(yticks=range(len(rows)),yticklabels=[labels[r['feature']] for r in rows],xlim=(-.02,1.02),
               xlabel='Group-weighted descriptive AUC',title='Whole insertion (retrospective)' if scope=='whole' else '10–15 mm band')
        ax.axvline(.5,color='gray',ls=':');ax.grid(axis='x',alpha=.2)
    save(fig,'success_vs_stall')


def fmt(v):return '—' if v is None else f'{v:.3g}' if isinstance(v,(int,float)) else str(v)


def write_report(output,info,robustness,comparisons,robust_comparison):
    primary={r['model']:r for r in robustness if r['configuration']=='primary'}
    lag=next(r for r in robustness if r['configuration']=='lag_2_steps' and r['model']=='full')
    early={r['feature']:r for r in comparisons if r['cohort']=='all' and r['task']=='stalled_vs_success' and r['scope']=='band10_15'}
    b_early={r['feature']:r for r in comparisons if r['cohort']=='Phase2B' and r['task']=='stalled_vs_success' and r['scope']=='band10_15'}
    full=primary['full']
    findings=[
        f"**Full 6D identification is unsupported at the primary settings:** {full['fully_excited']} of {full['accepted']} accepted windows have all six directions excited; median effective rank is {fmt(full['median_rank'])}. Translation has {primary['translation']['fully_excited']} fully excited windows; lateral/tilt has {primary['lateral_tilt']['fully_excited']}. These counts do not establish causal identifiability.",
        f"**Some local continuation prediction is possible:** the loaded-window median full-model skill is {fmt(full['contact_median_validation_skill_zero'])} against zero wrench change and {fmt(full['contact_median_skill_drift'])} against drift. But two-step differences give median skill {fmt(lag['contact_median_validation_skill_zero'])}; the result depends on temporal sampling and should not be treated as a stable material property."]
    if 'full_sensitivity' in early:
        r=early['full_sensitivity']
        conclusion='No clear added discrimination over force in the early band'
        if r['delta_low'] is not None and r['delta_low']>0:conclusion='An early-band association advantage over force is observed'
        if r['delta_high'] is not None and r['delta_high']<0:conclusion='Sensitivity underperforms force in the early band'
        findings.append(f"**{conclusion}:** 10–15 mm sensitivity AUC is {fmt(r['auc'])}, versus {fmt(r['force_auc_same_subset'])} for force on the same {r['trajectories']} references. Their paired difference interval is [{fmt(r['delta_low'])}, {fmt(r['delta_high'])}]. Progress-rate AUC is {fmt(early['progress_rate']['auc'])}; torque AUC is {fmt(early['torque_norm']['auc'])}. These are descriptive outcome associations, not test-set predictive accuracy.")
    if 'full_sensitivity' in b_early:
        r=b_early['full_sensitivity']
        findings.append(f"Within Phase 2B alone, early-band sensitivity AUC is {fmt(r['auc'])} versus {fmt(r['force_auc_same_subset'])} for force. Cohort-specific results matter; pooling stages and paths does not remove their design confounding.")
    findings.append('**Proceed only to a controlled active micro-probing identification pilot.** The observed association and excitation deficit justify collecting better measurements; this dataset does not justify deploying the fitted full matrix or claiming superior contact-response features. No active probes were implemented.')
    lines=['# Offline empirical contact response','',
        f"Analyzed **{info['trajectories_eligible']} / {info['trajectories_found']} references** and **{info['windows']} entered-insertion windows**. Outcomes: {info['trajectory_counts']}.",'',
        '**This is an observational proof of concept.** It estimates responses along recorded motion, not a causal stiffness matrix or an identified six-dimensional contact law. No probing or simulation changes were made.', '',
        '## Main findings','',*sum(([text,''] for text in findings),[]),
        '## Data and coordinate conventions','',
        '| Study | References | Physics Hz |','| --- | ---: | ---: |']
    lines += [f"| {s['study']} | {s['references']} | {s['physics_hz']} |" for s in info['studies']]
    lines += ['', 'Only each ledger reference `insertion.csv` is read for motion/wrench data. Approach, hold, negative-depth windows, recovery traces and replay prefixes are excluded. Existing success/stall/numerical labels are read unchanged. Other unsuccessful non-stalled references form a separate group. Comparisons also report the exploratory union of stalled and other unsuccessful references as difficult.', '',
        'Actual peg-base translation is reconstructed as `[tip_x_mm, tip_y_mm, -depth_mm]/1000` up to an irrelevant constant. Archived logger code, unit scale and fixed zero fixture orientation are checked; fixture axes equal world axes in these studies. Orientation increments use the shortest world-frame SO(3) logarithm of `q_next * conjugate(q_previous)`, handling quaternion sign flips. The lateral/tilt model uses infinitesimal world roll/pitch increments, not Euler-angle subtraction. Hand motion and commanded pose are never predictors.', '',
        'Wrench uses socket-on-peg contact force and torque, including friction, in world axes. Raw torque is about the moving peg base. Before differencing, torque is transported to the fixed first peg-base position of each window: `tau_anchor = tau_peg + (p - p_anchor) × F`. This removes the pure moving-origin torque artifact. All models within a window use this same anchor; coefficients are local to it.', '',
        '## Local estimators and observability','',
        f"Primary window: {info['settings']['window_s']} s; stride: {info['settings']['stride_s']} s; lag: {info['settings']['lag_steps']} physics step. Adjacent pose/wrench differences within a window are not smoothed. The primary ratio is `s = ||DeltaW_scaled||_F / ||DeltaXi_scaled||_F`, equivalent to the ratio of RMS increment norms. `endpoint_sensitivity` separately uses the first-to-last increment and is missing when net motion is unresolved.", '',
        f"Mixed units require a declared metric. With L={1000*info['settings']['length_scale_m']:g} mm, pose is `[dp, L*dtheta]` in metres and wrench is `[F, tau/L]` in equivalent newtons; the ratio is N/m. Raw norms that add metres to radians or newtons to newton-metres are not used. Translation-only maps 3 translations to 3 forces; lateral/tilt maps dx,dy,dtheta_x,dtheta_y to Fx,Fy,Fz,tau_x,tau_y. Sensitivities across different model dimensions are not interchangeable material constants.", '',
        f"A window/model is rejected if RMS input motion is at most {info['settings']['motion_floor_m']*1e6:g} micrometres in the scaled metric, no singular direction clears the excitation floor, or there are too few train/test pairs. Effective rank uses singular values greater than max({100*info['settings']['rank_relative_tolerance']:g}% of the largest, motion floor × sqrt(number of pairs)). Numerical rank, unregularized full condition number, retained-subspace condition number, and singular vectors are separately recorded. These are analysis settings, not changes to simulation thresholds. A finite ridge inverse does not establish observability.", '',
        f"A separate {info['settings']['wrench_floor_n']:g} N equivalent RMS response floor flags weak wrench changes and suppresses ill-defined normalized validation scores with near-zero baseline error. Absolute validation RMSE is still retained. The pose and wrench floors are explicit analysis choices, not calibrated sensor-noise bounds; weak-direction identification still requires an independent noise experiment.", '',
        'Ridge fits have no intercept and use alpha = 0.001 × trace(XᵀX)/input_dimension in scaled coordinates. `local_models.jsonl` stores G in SI and scaled units, singular values and the excited input basis. Coefficients outside the excited subspace must not be interpreted. Even fully excited noisy designs are not automatically physically identifiable.', '',
        'Temporal validation fits the first 65% of difference pairs and evaluates later pairs, skipping one pair at the boundary to avoid shared endpoint measurements. Skill is 1 − MSE_model/MSE_baseline, compared with zero wrench change and the training-mean wrench change (local drift). Nonpositive skill means no improvement. Validation projection coverage reports how much later motion lies in the training-excited subspace. This is local continuation checking, not independent trajectory-level model validation.', '',
        '## Observability and local continuation fit','',
        '| Variant | Model | Accepted / windows | Fully excited | Median rank | Loaded validation windows | Test skill vs zero | vs drift | Fraction beating zero |',
        '| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |']
    for r in robustness:
        lines.append(f"| {r['configuration']} | {r['model']} | {r['accepted']} / {r['windows']} | {r['fully_excited']} | {fmt(r['median_rank'])} | {r['validation_windows']} | {fmt(r['contact_median_validation_skill_zero'])} | {fmt(r['contact_median_skill_drift'])} | {fmt(r['contact_fraction_beating_zero'])} |")
    lines+=['','Loaded-contact summaries above require positive recorded normal load in at least half of samples. Contact counts alone are insufficient: the solver can report candidate contacts without nonzero load. The CSV includes both contact-count and positive-load fractions, zero-response fits and rejected-motion windows. Normalized validation summaries additionally require resolved later wrench change. These window-level counts are descriptive, not independent sample sizes.', '',
        '## Comparison against simple baselines','',
        'One feature value per trajectory is used: p90 of window features, except median progress rate. Force and torque baselines are window-median norms; normal load is the window median. dF/dt includes a local time slope of force magnitude and an RMS vector-force difference/time baseline that captures fast fluctuation without a pose denominator. dF/ddepth fits scalar force-magnitude increments against actual depth increments and is missing if depth motion is unresolved. Slope comparisons use absolute values. Progress is scored with the opposite sign; other score directions were fixed before inspecting outcomes.', '',
        'Whole-insertion features describe the completed outcome and can include an already-stalled interval. The predefined 10–15 mm band uses only windows wholly within that band to reduce terminal-state/depth confounding; it is not automatically a pre-stall forecast. Group-weighted AUCs are descriptive associations, not trained-model test scores. Positive class is stalled (or the explicitly indicated difficult union). No classifier, threshold tuning or train/test split is performed.', '',
        f"There are {info['unique_split_groups']} split groups, of which {info['stalled_split_groups']} contain stalled references; {info['exact_duplicate_references']} exact CSV duplicates and {info['repeated_command_configurations']} repeated command configurations. All centered controls share one group; Phase 2B path amplitudes share their path group across stages. Every group has total weight one. Confidence intervals bootstrap groups, not overlapping windows; this handles clustered repeats but not all shared pre-ramp histories or adaptive-selection bias.", '',
        'Each feature is compared with force on exactly the same available trajectory subset. Missing reduced-model features can change that subset, so rows with different sample counts are not direct model rankings. Phase-specific results are included to expose cohort confounding. The force comparison is paired but does not establish incremental predictive value beyond all baselines jointly.', '',
        '| Cohort | Scope | Feature | N / groups | AUC [95% cluster interval] | Delta vs force [95% interval] |',
        '| --- | --- | --- | ---: | --- | --- |']
    for r in comparisons:
        if r['task']!='stalled_vs_success':continue
        lines.append(f"| {r['cohort']} | {r['scope']} | {r['feature']} | {r['trajectories']} / {r['groups']} | {fmt(r['auc'])} [{fmt(r['auc_low'])}, {fmt(r['auc_high'])}] | {fmt(r['delta_vs_force'])} [{fmt(r['delta_low'])}, {fmt(r['delta_high'])}] |")
    lines+=['','Full comparisons, including other unsuccessful references, are in `feature_comparison.csv`. `robustness_comparison.csv` repeats sensitivity associations at L=5/20 mm, two-step differences, and 0.5 s windows; these checks are not used to select a winning hyperparameter.', '',
        '## What this supports next','',
        'These trajectories can test whether a locally measured direction of motion covaries with wrench change. They cannot by themselves establish a full contact-response matrix or causal benefit from a corrective action: pose and force evolve together under feedback, and contact switching, velocity, friction history and differencing noise remain confounders. A large ratio can result from a small denominator or force chatter, not necessarily useful contact stiffness.', '',
        'Active micro-probing is a reasonable **next identification experiment**, rather than a validated controller deployment, if the observed contact response and missing excitation motivate it. The next experiment should deliberately excite independent small pose directions with repeated signed probes at matched states, compare against no-motion wrench variability, and assess held-out response prediction. Independent noise calibration is needed to interpret weak singular directions. No active probing is implemented here.', '',
        '## Reproduce and audit','',
        '```bash','conda activate franka-safe-recovery','python -m research.contact_response --outputs-root outputs --output outputs/Contact-Response-Offline-v1','python -m unittest tests.test_contact_response -v','```','',
        'Use a new output directory for a rerun. `analysis.json` records settings and counts; `source_files.json` records source hashes and verifies that all pre-existing output file sizes/mtimes and all consumed input hashes remain unchanged. Saved simulation labels, geometry, friction, thresholds and prior outputs are untouched.', '',
        '![Sensitivity versus depth](sensitivity_vs_depth.png)','', '![Observability versus depth](observability_vs_depth.png)','',
        '![Success versus stall](success_vs_stall.png)','', '![Force versus sensitivity](force_vs_sensitivity.png)','']
    (output/'report.md').write_text('\n'.join(lines))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--outputs-root',type=Path,default=Path('outputs'))
    parser.add_argument('--output',type=Path,default=Path('outputs/Contact-Response-Offline-v1'))
    parser.add_argument('--window-s',type=float,default=.25)
    parser.add_argument('--stride-s',type=float,default=.1)
    parser.add_argument('--length-scale-mm',type=float,default=10.)
    parser.add_argument('--motion-floor-um',type=float,default=1.)
    parser.add_argument('--bootstrap-samples',type=int,default=400)
    args=parser.parse_args()
    run(args.outputs_root,args.output,Settings(window_s=args.window_s,stride_s=args.stride_s,
        length_scale_m=args.length_scale_mm/1000,motion_floor_m=args.motion_floor_um/1e6,
        bootstrap_samples=args.bootstrap_samples))

if __name__=='__main__':main()
