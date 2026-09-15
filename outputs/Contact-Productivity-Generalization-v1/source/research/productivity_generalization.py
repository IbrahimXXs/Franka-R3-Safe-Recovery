"""Outcome-independent held-out design and policy lock for the frozen benchmark."""
from collections import Counter
from dataclasses import asdict
from datetime import datetime,timezone
import hashlib
import json
import math
from pathlib import Path
import random
import numpy as np
from research.phase2b import COMMAND_KEYS,command_misalignment
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign

SCHEMA='Contact-Productivity-Generalization-v1'
POLICIES=('nominal','force','axial','dewedge')
FAMILIES=('axis_offset','oblique_offset','axis_tilt','oblique_tilt','cross_axis_combined','oblique_combined')
FROZEN_SOURCES=('research/productivity_control.py','simulation/productivity_control.py',
 'research/productivity_unloading.py','simulation/productivity_unloading.py',
 'research/productivity_dewedge.py','simulation/productivity_dewedge.py',
 'simulation/forge_backend.py','simulation/contact.py','simulation/forge_experiment.py',
 'research/forge_protocol.py','research/phase2_protocol.py','research/phase2b.py',
 'experiments/forge_phase2.json','experiments/productivity_control_calibration.json')

def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

def conditions():
    result=[]
    for fi,family in enumerate(FAMILIES):
        for li,(level,offset,tilt) in enumerate((('moderate',.7,3.5),('severe',1.2,7.))):
            for v,(sx,sy) in enumerate(((1,1),(-1,-1),(1,-1),(-1,1))):
                x=y=roll=pitch=0.
                if family=='axis_offset':
                    if v<2:x=(1 if v==0 else -1)*offset
                    else:y=(1 if v==2 else -1)*offset
                elif family=='oblique_offset':x,y=.8*sx*offset,.6*sy*offset
                elif family=='axis_tilt':
                    if v<2:roll=(1 if v==0 else -1)*tilt
                    else:pitch=(1 if v==2 else -1)*tilt
                elif family=='oblique_tilt':roll,pitch=.6*sx*tilt,.8*sy*tilt
                elif family=='cross_axis_combined':
                    if v<2:x,roll=(1 if v==0 else -1)*offset,(1 if v==0 else -1)*tilt
                    else:y,pitch=(1 if v==2 else -1)*offset,(-1 if v==2 else 1)*tilt
                else:
                    x,y=.8*sx*offset,.6*sy*offset
                    roll,pitch=.8*sy*tilt*(1 if v<2 else -1),.6*sx*tilt*(-1 if v<2 else 1)
                onset=(7.,12.,15.)[(fi+li+v)%3]
                cid=f'g{len(result):03d}_{family}_{level}_v{v}'
                case=dict(family=family,offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=0.,
                    insertion_duration_s=8.,ramp_onset_mm=onset,ramp_full_depth_mm=20.,
                    final_offset_x_mm=x,final_offset_y_mm=y,final_roll_deg=roll,final_pitch_deg=pitch,
                    path_group_id=cid,split_group_id=cid,sample_role='frozen_held_out_evaluation')
                result.append(dict(case_id=cid,family=family,severity=level,sign_variant=v,
                    ramp_onset_mm=onset,offset_magnitude_mm=math.hypot(x,y),tilt_magnitude_deg=math.hypot(roll,pitch),case=case))
    return result


def signature(case):
    """Compare actual reference path parameters, not metadata/group names."""
    depth=np.linspace(-10,20,301)
    commands=[command_misalignment(case,float(d)) for d in depth]
    vector=[case.get('insertion_duration_s',8.)]+[c[k] for c in commands for k in COMMAND_KEYS]
    return hashlib.sha256(np.round(vector,10).astype('<f8').tobytes()).hexdigest()


def previous_paths(root):
    hashes={};seen=set();count=0
    paths=sorted(set((root/'outputs').glob('*/study.json'))|set((root/'outputs').glob('*/experiment.json')))
    for path in paths:
        if path.parent.name.startswith('Contact-Productivity-Generalization'):continue
        value=json.loads(path.read_text());candidates=[]
        for key in ('attempts','cases'):
            rows=value.get(key,[])
            if isinstance(rows,list):candidates.extend(rows)
        plan=value.get('case_plan')
        if isinstance(plan,dict):candidates.extend(plan.get('cases',[]))
        accepted=0
        for row in candidates:
            if not isinstance(row,dict):continue
            case=row.get('case',row)
            if not isinstance(case,dict) or not all(k in case for k in COMMAND_KEYS):continue
            if 'ramp_onset_mm' in case and not all('final_'+k in case for k in COMMAND_KEYS):continue
            seen.add(signature(case));accepted+=1;count+=1
        if accepted:hashes[str(path.relative_to(root))]=sha(path)
    return seen,dict(source_sha256=hashes,descriptors_checked=count,unique_paths=len(seen))


def schedule(cases,seed=20260916):
    """Random condition order; each policy occupies each ordinal position 12 times."""
    rng=random.Random(seed);order=list(cases);rng.shuffle(order);base=list(POLICIES);rng.shuffle(base)
    slots=list(range(4))*12;rng.shuffle(slots);result=[]
    for c,rotation in zip(order,slots):
        pp=base[rotation:]+base[:rotation]
        for ordinal,policy in enumerate(pp):
            result.append(dict(case_id=c['case_id'],repeat=0,policy=policy,policy_order=ordinal))
    return result


def validate_plan(value):
    if value['schema']!=SCHEMA or value['cases']!=conditions():raise ValueError('Held-out condition design changed')
    if value['schedule']!=schedule(value['cases']):raise ValueError('Frozen evaluation schedule changed')
    if value['design']!=asdict(Design()) or value['unloading_design']!=asdict(UnloadingDesign()) or value['dewedge_design']!=asdict(DewedgeDesign()):
        raise ValueError('Frozen policy design changed')
    sigs=[signature(c['case']) for c in value['cases']]
    if len(set(sigs))!=48:raise ValueError('Duplicate held-out paths')
    return value


def freeze(root,path):
    if path.exists():raise FileExistsError('Never overwrite a frozen evaluation plan')
    previous=root/'outputs/Contact-Productivity-Dewedge-v1';prior=json.loads((previous/'experiment.json').read_text())
    cal=json.loads((root/'experiments/productivity_control_calibration.json').read_text())
    if cal!=json.loads((previous/'calibration.json').read_text()):raise ValueError('Calibration changed')
    for name in FROZEN_SOURCES:
        if sha(root/name)!=prior['source_sha256'][name]:raise ValueError('Prior policy/physics source changed: '+name)
    old,overlap=previous_paths(root);cases=conditions()
    if any(signature(c['case']) in old for c in cases):raise ValueError('Held-out path duplicates previous data')
    value=dict(schema=SCHEMA,frozen_at_utc=datetime.now(timezone.utc).isoformat(),
        previous_experiment=str(previous.relative_to(root)),previous_manifest_sha256=sha(previous/'experiment.json'),
        design=cal['design'],unloading_design=prior['unloading_design'],dewedge_design=prior['dewedge_design'],
        eta_threshold=cal['eta_threshold'],force_threshold_n=cal['force_threshold_n'],
        policy_source_sha256={f:sha(root/f) for f in FROZEN_SOURCES},cases=cases,schedule=schedule(cases),
        holdout_audit=overlap,exact_previous_path_overlaps=0,condition_selection='Fixed structured design; no outcome-driven selection, exclusions, replacement or retuning.',
        clearance_variation=False,clearance_reason='The current backend requires unit mesh scale and the audited 9 mm bore; diameter metadata is not a geometry parameter. Changing clearance would require changing mesh/geometry audit and penetration interpretation.',
        geometry_scope='48 unseen pose/misalignment paths on the same audited peg/socket assets; not 48 new solid geometries.',
        analysis=dict(primary='Insertion success over all 48 conditions per policy; safety/numerical failures retained.',
            paired_bootstrap_seed=20260916,paired_bootstrap_samples=10000,
            uncertainty='Stratified resampling of conditions within six families; descriptive uncertainty over this structured test design, not population guarantees.'))
    validate_plan(value);path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');return value


def verify_lock(root,value):
    validate_plan(value)
    for f,digest in value['policy_source_sha256'].items():
        if sha(root/f)!=digest:raise ValueError('Frozen policy/physics source changed: '+f)
    for f,digest in value['holdout_audit']['source_sha256'].items():
        if sha(root/f)!=digest:raise ValueError('Previous source manifest changed: '+f)
    if sha(root/value['previous_experiment']/'experiment.json')!=value['previous_manifest_sha256']:
        raise ValueError('Previous pilot manifest changed')
    return True
