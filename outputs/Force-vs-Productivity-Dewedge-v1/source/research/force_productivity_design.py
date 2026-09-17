"""Predeclared development and disjoint held-out detector-only ablation."""
from dataclasses import asdict
from datetime import datetime,timezone
import json
import math
from pathlib import Path
import random
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_generalization import FROZEN_SOURCES,sha
from research.productivity_detector_v2_generalization import path_signature,CONFIG,CONFIG_SHA,FAMILIES
from research.phase2b import COMMAND_KEYS

SCHEMA='Force-vs-Productivity-Dewedge-v1'
PLAN='experiments/force_productivity_dewedge.json'
THRESHOLDS=(1.,2.,3.,4.,5.)
POLICIES=('nominal','force','productivity')
NEW_FILES=('research/force_productivity.py','research/force_productivity_design.py','research/force_productivity_analysis.py',
    'research/force_productivity_report.py','simulation/force_productivity.py','simulation/launch_force_productivity.py',
    'force_productivity_dewedge.sh','tests/test_force_productivity.py','docs/force_productivity_dewedge.md')


def conditions(split):
    dev=split=='development';result=[]
    for fi,family in enumerate(FAMILIES):
        variants=[((fi+v)%2,v) for v in range(4)] if dev else [(level,v) for level in range(2) for v in range(4)]
        for level,v in variants:
            sx,sy=((1,1),(-1,-1),(1,-1),(-1,1))[v]
            angle=math.radians(((19.,43.,61.,29.) if dev else (26.,51.,67.,37.))[v])
            offset=((.78,1.28) if dev else (.91,1.43))[level]
            tilt=((3.6,6.8) if dev else (3.95,7.6))[level]
            onset=((7.4,10.8,14.6,17.4) if dev else (7.7,11.2,14.9,17.7))[(fi+v+level)%4]
            x=y=roll=pitch=0.;severity=('moderate','severe')[level]
            if family=='easy_controls':
                offset=((.10,.25) if dev else (.12,.29))[level];tilt=((.55,1.25) if dev else (.65,1.45))[level];severity='easy'
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
                if family=='terminal_band':
                    offset=((.31,.49) if dev else (.37,.58))[level];tilt=((3.15,4.35) if dev else (3.42,4.45))[level]
                if family=='late_combined':
                    offset=((.67,1.17) if dev else (.79,1.23))[level];tilt=((4.9,7.4) if dev else (5.1,7.9))[level]
                    onset=((15.6,16.6,17.6,18.1) if dev else (15.7,16.7,17.9,18.3))[v]
                x,y=sx*offset*math.cos(angle),sy*offset*math.sin(angle)
                roll,pitch=sy*tilt*math.sin(angle),-sx*tilt*math.cos(angle)
            cid=f"{'fd' if dev else 'ft'}{len(result):03d}_{family}_{severity}_v{v}"
            case=dict(family=family,offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=0.,insertion_duration_s=8.,
                ramp_onset_mm=onset,ramp_full_depth_mm=20.,final_offset_x_mm=x,final_offset_y_mm=y,
                final_roll_deg=roll,final_pitch_deg=pitch,path_group_id=cid,split_group_id=cid,sample_role='force_detector_'+split)
            result.append(dict(case_id=cid,family=family,severity=severity,split=split,group_id=cid,
                sign_variant=v,ramp_onset_mm=onset,offset_magnitude_mm=math.hypot(x,y),tilt_magnitude_deg=math.hypot(roll,pitch),case=case))
    return result


def prior_paths(root):
    root=Path(root);seen=set();sources={};counts={}
    paths=set((root/'outputs').glob('*/study.json'))|set((root/'outputs').glob('*/experiment.json'))
    for path in sorted(paths):
        if path.parent.name.startswith(SCHEMA):continue
        obj=json.loads(path.read_text());rows=[]
        for key in ('cases','attempts'):
            if isinstance(obj.get(key),list):rows.extend(obj[key])
        if isinstance(obj.get('case_plan'),dict):rows.extend(obj['case_plan'].get('cases',[]))
        count=0
        for row in rows:
            if not isinstance(row,dict):continue
            c=row.get('case',row)
            if not isinstance(c,dict) or not all(k in c for k in COMMAND_KEYS):continue
            if 'ramp_onset_mm' in c and not all('final_'+k in c for k in COMMAND_KEYS):continue
            seen.add(path_signature(c));count+=1
        sources[str(path.relative_to(root))]=sha(path);counts[str(path.relative_to(root))]=count
    return seen,dict(source_sha256=sources,descriptor_counts=counts,unique_paths=len(seen))


def schedule():
    rng=random.Random(20260919);dev=conditions('development');test=conditions('held_out');rng.shuffle(dev);ss=[]
    for c in dev:ss.append(dict(case_id=c['case_id'],split='development',policy='nominal',detector_id='nominal',stage='development_reference',threshold_n=None))
    rng.shuffle(dev);rotations=[i%5 for i in range(32)];rng.shuffle(rotations)
    for c,rot in zip(dev,rotations):
        for threshold in THRESHOLDS[rot:]+THRESHOLDS[:rot]:
            ss.append(dict(case_id=c['case_id'],split='development',policy='force',detector_id=f'force_{threshold:g}',stage='development_candidate',threshold_n=threshold))
    rng.shuffle(test);rotations=[i%3 for i in range(64)];rng.shuffle(rotations)
    for c,rot in zip(test,rotations):
        for policy in POLICIES[rot:]+POLICIES[:rot]:
            ss.append(dict(case_id=c['case_id'],split='held_out',policy=policy,detector_id=policy,stage='held_out',threshold_n=None))
    return ss


def freeze(root,path):
    root=Path(root);path=Path(path)
    if path.exists():raise FileExistsError('Frozen plans are never overwritten')
    previous='outputs/Contact-Productivity-DetectorV2-Generalization-v1'
    m=json.loads((root/previous/'experiment.json').read_text())
    for f in (*FROZEN_SOURCES,'research/productivity_detector_v2.py'):
        if sha(root/f)!=m['source_sha256'][f]:raise ValueError('Frozen recovery/physics/v2 changed: '+f)
    old,overlap=prior_paths(root)
    for name,count in (('Contact-Productivity-Generalization-v1',48),('Contact-Productivity-DetectorV2-Dev',80),('Contact-Productivity-DetectorV2-Generalization-v1',64)):
        if overlap['descriptor_counts'].get('outputs/'+name+'/experiment.json')!=count:raise ValueError('Missing prior condition descriptors')
    cc=conditions('development')+conditions('held_out');sigs=[path_signature(c['case']) for c in cc]
    if len(set(sigs))!=96 or set(sigs)&old:raise ValueError('Overlapping paths')
    files=tuple(dict.fromkeys((*FROZEN_SOURCES,*NEW_FILES,*(str(f.relative_to(root)) for d in ('research','simulation') for f in sorted((root/d).glob('*.py'))))))
    value=dict(schema=SCHEMA,frozen_at_utc=datetime.now(timezone.utc).isoformat(),cases=cc,schedule=schedule(),
        development_conditions=32,held_out_conditions=64,episodes=384,force_candidates=list(THRESHOLDS),policies=list(POLICIES),
        previous_experiment=previous,detector_v2_config=CONFIG,detector_v2_config_sha256=CONFIG_SHA,
        design=asdict(Design()),unloading_design=asdict(UnloadingDesign()),dewedge_design=asdict(DewedgeDesign()),
        source_sha256={f:sha(root/f) for f in files},holdout_audit=overlap,exact_previous_path_overlaps=0,development_test_overlaps=0,
        force_eligibility='Full 0.5 s causal history in one insert or hold phase/segment, above the same contact-onset +0.1 mm gate, finite measured force. No positive-command-progress or success-depth restriction. Geometry/phase are common eligibility metadata, not predictive force features.',
        force_decision='Current deployable wrist force norm strictly greater than threshold at two consecutive 0.1 s checks. No eta, velocity, normal load, or future information enters the force decision.',
        selection=dict(max_success_alert_fraction=.1,minimum_safety_stall_lead_s=.1,
            timeout_recovery_reserve_s=Design().stop_s+UnloadingDesign().recovery_timeout_s,
            order=['admissible false-alert fraction <= 0.1','maximize useful failed-case coverage','maximize any safe failed-case coverage','minimize successful-trajectory alerts','maximize summed capped useful lead','prefer larger threshold'],
            fallback='If none meet the false-alert cap, first minimize successful-trajectory alerts, then use the same coverage/lead ordering; flag cap failure explicitly.',
            closed_loop_outcomes_used_for_selection=False),
        analysis=dict(primary='Paired productivity versus force insertion success on all 64 new held-out cases; retain every failure.',
            paired_test='Two-sided exact conditional McNemar/binomial, p=1 for zero discordances.',
            bootstrap_seed=20260919,bootstrap_samples=10000,bootstrap='Paired condition bootstrap stratified by family; percentile 95% interval.',
            subsets=['all','moderate','severe','strict_matched'],
            strict_match='Every common-prefix sample through earlier first intervention or episode end passes the original FORGE state-match tolerances.',
            shadow='Identical nominal histories, only safe alarms counted; coverage among nominal failures and false alerts among nominal successes.',
            low_force_low_productivity='Safe productivity shadow trigger with meaningful eta below the frozen normal threshold, or a separately labelled terminal-stagnation trigger, and force <= selected force threshold. Never assign eta=0 to endpoint hold.',
            high_force_healthy_productivity='Safe force shadow trigger with meaningful eta >= frozen normal threshold; report whether nominal later succeeds.',
            attribution='Identical recovery isolates detector effect; unmatched differences alone do not establish superiority. This experiment cannot quantify the separate effect of changing recovery action.',
            hybrid='Omitted from collection; shadow overlap/complementarity is descriptive only, with no hybrid tuning or controller.'),
        optional_variants_omitted='Force-rise and hybrid omitted to bound collection; all five specified absolute-force thresholds receive closed-loop development evaluation.',
        geometry_scope='New misalignment paths on unchanged assets. Near-aligned controls are used because exact centered paths already exist.')
    path.write_text(json.dumps(value,indent=2,allow_nan=False)+'\n');verify(root,value);return value


def verify(root,plan):
    root=Path(root)
    if plan['schema']!=SCHEMA or plan['cases']!=conditions('development')+conditions('held_out') or plan['schedule']!=schedule():raise ValueError('Frozen plan changed')
    for k,cls in (('design',Design),('unloading_design',UnloadingDesign),('dewedge_design',DewedgeDesign)):
        if plan[k]!=asdict(cls()):raise ValueError('Frozen budget changed')
    if sha(root/CONFIG)!=CONFIG_SHA:raise ValueError('Productivity v2 configuration changed')
    for f,h in {**plan['source_sha256'],**plan['holdout_audit']['source_sha256']}.items():
        if sha(root/f)!=h:raise ValueError('Frozen source changed: '+f)
    return True


if __name__=='__main__':
    root=Path(__file__).resolve().parents[1];p=freeze(root,root/PLAN)
    print('Frozen:',len(p['cases']),'new paths;',len(p['schedule']),'episodes; no overlaps')
