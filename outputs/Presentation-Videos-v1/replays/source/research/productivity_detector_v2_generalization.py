"""Outcome-independent final held-out plan and complete experiment lock."""
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import random
import numpy as np
from research.phase2b import COMMAND_KEYS, command_misalignment
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_generalization import FROZEN_SOURCES, sha

SCHEMA = 'Contact-Productivity-DetectorV2-Generalization-v1'
POLICIES = ('nominal', 'v1', 'v2')
FAMILIES = ('easy_controls', 'axis_offset', 'oblique_offset', 'axis_tilt',
            'oblique_tilt', 'combined', 'terminal_band', 'late_combined')
CONFIG = 'outputs/Contact-Productivity-DetectorV2-Dev/detector_v2_config.json'
CONFIG_SHA = '189da99e1ba35a73a68221683cf0d6d215b3443d332c906e7e1afd231194de1d'
PLAN = 'experiments/productivity_detector_v2_generalization.json'
NEW_FILES = ('research/productivity_detector_v2_generalization.py',
    'research/productivity_detector_v2_generalization_report.py',
    'simulation/productivity_detector_v2_generalization.py',
    'simulation/launch_productivity_detector_v2_generalization.py',
    'productivity_detector_v2_generalization.sh',
    'tests/test_productivity_detector_v2_generalization.py',
    'docs/productivity_detector_v2_generalization.md')
DEPENDENCIES = ('research/productivity_detector_v2.py', 'research/productivity_detector_v2_calibration.py',
    'research/productivity_detector_v2_design.py', 'research/productivity_detector_v2_report.py',
    'research/productivity_generalization.py', 'research/productivity_dewedge_report.py',
    'research/productivity_unloading_report.py', 'research/productivity_control_report.py',
    'research/future_stall.py')


def conditions():
    cases=[]
    for fi,family in enumerate(FAMILIES):
        for level in range(2):
            for v,(sx,sy) in enumerate(((1,1),(-1,-1),(1,-1),(-1,1))):
                offset=(.83,1.37)[level];tilt=(3.8,7.3)[level]
                angle=math.radians((22.,47.,63.,31.)[v])
                x=y=roll=pitch=0.
                onset=(6.8,10.4,14.2,17.2)[(fi+v+level)%4]
                severity=('moderate','severe')[level]
                if family=='easy_controls':
                    offset=(.16,.34)[level];tilt=(.8,1.7)[level];severity='easy'
                    x,y=sx*offset*math.cos(angle),sy*offset*math.sin(angle)
                    roll,pitch=sy*tilt*math.sin(angle),-sx*tilt*math.cos(angle)
                elif family=='axis_offset':
                    if v<2:x=sx*offset
                    else:y=sy*offset
                elif family=='oblique_offset':x,y=sx*offset*math.cos(angle),sy*offset*math.sin(angle)
                elif family=='axis_tilt':
                    if v<2:roll=sx*tilt
                    else:pitch=sy*tilt
                elif family=='oblique_tilt':roll,pitch=sx*tilt*math.cos(angle),sy*tilt*math.sin(angle)
                else:
                    if family=='terminal_band':offset=(.28,.52)[level];tilt=(3.28,4.18)[level]
                    if family=='late_combined':
                        offset=(.73,1.13)[level];tilt=(4.7,7.7)[level]
                        onset=(15.8,16.8,17.8,18.2)[v]
                    x,y=sx*offset*math.cos(angle),sy*offset*math.sin(angle)
                    roll,pitch=sy*tilt*math.sin(angle),-sx*tilt*math.cos(angle)
                cid=f'h{len(cases):03d}_{family}_{severity}_v{v}'
                case=dict(family=family,offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=0.,
                    insertion_duration_s=8.,ramp_onset_mm=onset,ramp_full_depth_mm=20.,
                    final_offset_x_mm=x,final_offset_y_mm=y,final_roll_deg=roll,final_pitch_deg=pitch,
                    path_group_id=cid,split_group_id=cid,sample_role='final_frozen_held_out')
                cases.append(dict(case_id=cid,family=family,severity=severity,sign_variant=v,
                    group_id=cid,split='held_out',ramp_onset_mm=onset,
                    offset_magnitude_mm=math.hypot(x,y),tilt_magnitude_deg=math.hypot(roll,pitch),case=case))
    return cases


def path_signature(case):
    """Dense path fingerprint, independent of labels and signed floating-point zero."""
    values=[case.get('insertion_duration_s',8.)]
    for d in np.linspace(-10,20,301):
        command=command_misalignment(case,float(d));values.extend(command[k] for k in COMMAND_KEYS)
    a=np.round(values,10).astype('<f8');a[a==0]=0.
    return hashlib.sha256(a.tobytes()).hexdigest()


def prior_paths(root):
    """Read parameter descriptors only. Never inspect success or other outcomes."""
    root=Path(root);seen=set();sources={};counts={}
    paths=set((root/'outputs').glob('*/study.json'))|set((root/'outputs').glob('*/experiment.json'))
    paths|={root/'experiments/productivity_generalization_frozen.json',root/'experiments/productivity_detector_v2_dev.json'}
    for path in sorted(paths):
        if path.parent.name.startswith(SCHEMA):continue
        obj=json.loads(path.read_text());candidates=[]
        for key in ('cases','attempts','scenarios'):
            if isinstance(obj.get(key),list):candidates.extend(obj[key])
        if isinstance(obj.get('case_plan'),dict):candidates.extend(obj['case_plan'].get('cases',[]))
        accepted=0
        for row in candidates:
            if not isinstance(row,dict):continue
            case=row.get('case',row)
            if not isinstance(case,dict) or not all(k in case for k in COMMAND_KEYS):continue
            if 'ramp_onset_mm' in case and not all('final_'+k in case for k in COMMAND_KEYS):continue
            seen.add(path_signature(case));accepted+=1
        sources[str(path.relative_to(root))]=sha(path);counts[str(path.relative_to(root))]=accepted
    return seen,dict(source_sha256=sources,descriptor_counts=counts,unique_prior_paths=len(seen))


def schedule(cases):
    rng=random.Random(20260918);order=list(cases);rng.shuffle(order)
    # Each policy occupies each position 21 or 22 times. Conditions remain paired.
    rotations=[i%3 for i in range(len(order))];rng.shuffle(rotations);result=[]
    for c,rotation in zip(order,rotations):
        pp=POLICIES[rotation:]+POLICIES[:rotation]
        result.extend(dict(case_id=c['case_id'],policy=policy,policy_order=j,repeat=0,stage='held_out')
                      for j,policy in enumerate(pp))
    return result


def freeze(root,path):
    root=Path(root);path=Path(path)
    if path.exists():raise FileExistsError('Never overwrite a frozen plan')
    prior=root/'outputs/Contact-Productivity-DetectorV2-Dev'
    previous=json.loads((prior/'experiment.json').read_text())
    if sha(root/CONFIG)!=CONFIG_SHA:raise ValueError('Selected development configuration changed')
    for name in (*FROZEN_SOURCES,'research/productivity_detector_v2.py'):
        if sha(root/name)!=previous['source_sha256'][name]:raise ValueError('Frozen policy drift: '+name)
    old,audit=prior_paths(root)
    if audit['descriptor_counts'].get('outputs/Contact-Productivity-Generalization-v1/experiment.json')!=48 or audit['descriptor_counts'].get('outputs/Contact-Productivity-DetectorV2-Dev/experiment.json')!=80:
        raise ValueError('Required previous benchmark/development descriptors are missing')
    cc=conditions();sigs=[path_signature(c['case']) for c in cc]
    if len(set(sigs))!=64 or set(sigs)&old:raise ValueError('Duplicate or overlapping condition paths')
    files=tuple(dict.fromkeys((*FROZEN_SOURCES,*DEPENDENCIES,*NEW_FILES,
        *(str(f.relative_to(root)) for folder in ('research','simulation') for f in sorted((root/folder).glob('*.py'))))))
    plan=dict(schema=SCHEMA,frozen_at_utc=datetime.now(timezone.utc).isoformat(),cases=cc,schedule=schedule(cc),
        policies=list(POLICIES),optional_baselines_included=False,
        optional_baselines_reason='192 required episodes; optional force and axial baselines add 128 episodes (~67% collection time). Prioritize all 64 fresh paired conditions.',
        previous_experiment=str(prior.relative_to(root)),detector_v2_config=CONFIG,detector_v2_config_sha256=CONFIG_SHA,
        design=asdict(Design()),unloading_design=asdict(UnloadingDesign()),dewedge_design=asdict(DewedgeDesign()),
        source_sha256={f:sha(root/f) for f in files},holdout_audit=audit,exact_previous_path_overlaps=0,
        condition_selection='Fixed structured design. No outcome-dependent tuning, exclusions, substitutions, stopping, or repeats.',
        geometry_scope='64 new imposed contact paths on unchanged FORGE peg/socket assets; no clearance variation.',
        analysis=dict(primary='Paired insertion success v2 versus v1, all 64 conditions; all failures retained.',
            severe_definition='Preassigned upper level in each non-control family, including terminal-band and late-ramp strata; 28 conditions, independent of observed outcomes.',
            paired_test='Two-sided exact conditional McNemar/binomial on discordant success pairs, p=1 if none.',
            bootstrap_seed=20260918,bootstrap_samples=10000,
            bootstrap='Paired condition resampling, stratified by the eight predeclared families; 95% percentile CI for success-rate difference. Descriptive structured-design uncertainty.',
            strict_match='All samples of the common prefix through the earlier first intervention (or earlier episode end) pass original FORGE pose, velocity, joints, wrench, load and grasp replay tolerances. No tolerances changed.',
            coverage='Safe shadow alarms replayed on identical nominal histories; failed nominal trajectories form denominator. No future data enter detector.',
            unnecessary='Shadow alarm on subsequently successful nominal history; actual interventions on paired nominal successes reported separately, with strict-match sensitivity.',
            lead='Nominal first-stall and safety-stop time minus first safe shadow alarm; positive means earlier. Report safety lead >=0.1s; terminal opportunity delay separately.',
            secondary='Severe subset, safety, timeouts, coverage, false interventions, branch counts and lead are descriptive; no multiplicity-adjusted confirmatory claims.'))
    path.write_text(json.dumps(plan,indent=2,allow_nan=False)+'\n');verify(root,plan);return plan


def verify(root,plan):
    root=Path(root)
    if plan['schema']!=SCHEMA or plan['cases']!=conditions() or plan['schedule']!=schedule(conditions()):raise ValueError('Frozen condition plan changed')
    for key,cls in (('design',Design),('unloading_design',UnloadingDesign),('dewedge_design',DewedgeDesign)):
        if plan[key]!=asdict(cls()):raise ValueError('Frozen design changed')
    if plan['detector_v2_config_sha256']!=CONFIG_SHA or sha(root/CONFIG)!=CONFIG_SHA:raise ValueError('Frozen v2 configuration changed')
    for f,h in {**plan['source_sha256'],**plan['holdout_audit']['source_sha256']}.items():
        if sha(root/f)!=h:raise ValueError('Frozen experiment source changed: '+f)
    return True


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];plan=freeze(root,root/PLAN)
    print(json.dumps(dict(conditions=len(plan['cases']),episodes=len(plan['schedule']),
        severity=dict(Counter(c['severity'] for c in plan['cases'])),overlaps=0),indent=2))
