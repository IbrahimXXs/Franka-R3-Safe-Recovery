"""Threshold selection from NEW nominal calibration groups only; no closed-loop tuning."""
from datetime import datetime, timezone
from itertools import product
import json
from pathlib import Path
from research.productivity_control import Design, Detector, recent_signal
from research.productivity_control_report import read_log
from research.productivity_detector_v2 import DetectorV2, NORMAL_ETA, features
from research.productivity_generalization import sha
from research.productivity_unloading_report import safe
from research.forge_protocol import ForgeProtocol
from research.future_stall import table


def nominal_checks(rows, case, hz, p):
    result=[]; design=Design()
    for i in range(round(design.check_s*hz),len(rows),round(design.check_s*hz)):
        r=rows[i]
        result.append(dict(time_s=r['time_s'],depth_mm=r['depth_mm'],phase=r['phase'],safe=safe(r,p),
            signal=features(rows[:i+1],hz,case['ramp_onset_mm'],design,p.success_depth_mm)))
    return result


def replay(checks, config=None):
    detector=DetectorV2(config) if config else Detector('productivity',NORMAL_ETA,0.)
    alarms=[]
    for r in checks:
        signal=r['signal']
        if config:
            fired=detector.check(r['time_s'],signal); branch=detector.last_decision['trigger_branch']
        else:
            fired=detector.check(r['time_s'],signal if signal and signal['normal_valid'] else None);branch='normal' if fired else ''
        if fired:
            alarms.append(dict(time_s=r['time_s'],safe=r['safe'],branch=branch,depth_mm=r['depth_mm']))
    return alarms


def first_safe(alarms):
    return next((r['time_s'] for r in alarms if r['safe']),None)


def calibrate(directory):
    directory=Path(directory); m=json.loads((directory/'experiment.json').read_text());plan=json.loads((directory/'development_plan.json').read_text())
    if (directory/'detector_v2_config.json').exists():raise FileExistsError('Frozen detector config already exists')
    p=ForgeProtocol(**m['protocol']);hz=m['physics_hz'];dataset=[];hashes={}
    cases={c['case_id']:c for c in plan['cases'] if c['split']=='calibration'}
    for run in m['runs']:
        if run['policy']!='nominal' or run['case_id'] not in cases:continue
        c=cases[run['case_id']];path=directory/run['log'];rows=read_log(path)
        checks=nominal_checks(rows,c['case'],hz,p)
        dataset.append(dict(run=run,checks=checks,v1=first_safe(replay(checks)),endpoint=rows[-1]['time_s']))
        hashes[str(path.relative_to(directory))]=sha(path)
    assert len(dataset)==60 and len({cases[d['run']['case_id']]['group_id'] for d in dataset})==15
    positive=sum(d['run']['insertion_success'] for d in dataset)
    if positive<5:raise RuntimeError('Fewer than five successful calibration controls: cannot certify false-alert constraint')
    grid=plan['calibration_grid'];candidates=[]
    for eta,accel,rate in product(grid['urgent_eta_threshold'],grid['urgent_deficit_acceleration_mm_s2'],grid['terminal_rate_mm_s']):
        config=dict(normal_eta_threshold=NORMAL_ETA,normal_consecutive_checks=2,
            urgent_eta_threshold=eta,urgent_deficit_acceleration_mm_s2=accel,terminal_rate_mm_s=rate)
        additional=total_false=covered=improved=0;lead=0.
        for d in dataset:
            t=first_safe(replay(d['checks'],config));old=d['v1'];success=d['run']['insertion_success']
            if success:
                total_false+=t is not None;additional+=t is not None and old is None
            elif t is not None:
                usable=d['endpoint']-t>=plan['selection']['minimum_useful_lead_s']-1e-8
                covered+=usable
                improved+=usable and (old is None or old-t>=.1-1e-8)
                lead+=min(1.,max(0.,(old if old is not None else d['endpoint'])-t))
        candidates.append(dict(**config,additional_success_alerts=additional,total_success_alerts=total_false,
            successful_calibration_trajectories=positive,covered_failures=covered,improved_failure_alerts=improved,capped_lead_gain_s=lead,
            admissible=additional<=plan['selection']['max_additional_successful_trajectory_alerts']))
    valid=[c for c in candidates if c['admissible']]
    if not valid:raise RuntimeError('No v2 candidate meets the predeclared false-alert constraint; no closed-loop collection authorized by this design')
    best=max(valid,key=lambda c:(c['improved_failure_alerts'],c['covered_failures'],c['capped_lead_gain_s'],
        -c['total_success_alerts'],-c['urgent_eta_threshold'],c['urgent_deficit_acceleration_mm_s2'],-c['terminal_rate_mm_s']))
    config={k:best[k] for k in ('normal_eta_threshold','normal_consecutive_checks','urgent_eta_threshold','urgent_deficit_acceleration_mm_s2','terminal_rate_mm_s')}
    config.update(schema='Contact-Productivity-Detector-v2',frozen_at_utc=datetime.now(timezone.utc).isoformat(),
        history_s=.5,check_s=.1,terminal_history_s=.5,terminal_endpoint_mm=20.,success_depth_mm=p.success_depth_mm,
        terminal_definition='All history samples in endpoint hold, same segment, below success and above contact onset; depth range/history <= terminal rate. Range rejects oscillatory cancellation.',
        urgent_definition='Original insertion-history eligibility, eta below urgent threshold, positive progress-deficit rate and >= threshold increase in that rate per second over adjacent checks.',
        selection=plan['selection'],selected_calibration_metrics=best,calibration_trajectories=60,
        calibration_groups=sorted({cases[d['run']['case_id']]['group_id'] for d in dataset}),
        calibration_source_sha256=hashes,development_plan_sha256=sha(directory/'development_plan.json'),
        validation_used_for_selection=False,previous_generalization_results_used=False,closed_loop_outcomes_used=False,
        recovery_source_sha256=plan['policy_source_sha256'])
    table(directory/'calibration_candidates.csv',candidates)
    (directory/'detector_v2_config.json').write_text(json.dumps(config,indent=2,allow_nan=False)+'\n')
    return config
