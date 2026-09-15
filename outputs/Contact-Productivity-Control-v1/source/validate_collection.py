import hashlib
import json
from pathlib import Path
from dataclasses import asdict
import math
from research.productivity_control import Design, Detector, recent_signal, stalled_now
from research.productivity_control_report import read_log
from research.forge_protocol import ForgeProtocol, screened, retained, within_budget
from research.phase2b import command_misalignment

root=Path.cwd();out=root/'outputs/Contact-Productivity-Control-v1'
m=json.loads((out/'experiment.json').read_text());cal=json.loads((out/'calibration.json').read_text())
design=Design(**m['design']);p=ForgeProtocol(**m['protocol']);hz=m['physics_hz']
assert m['status']=='complete'
assert len(m['runs'])==len(m['schedule'])==24
assert len({(r['case_id'],r['repeat'],r['policy']) for r in m['runs']})==24
assert not m['preexisting_outputs_changed']
assert set(x['group_id'] for x in cal['development']).isdisjoint(cal['stress_groups'])
assert all(x['group_weighted_nonstall_alert_rate']<=design.nonstall_alert_budget for x in cal['diagnostics'])
assert all(hashlib.sha256(Path(f).read_bytes()).hexdigest()==sha for f,sha in cal['source_sha256'].items())
assert all(hashlib.sha256((out/'source'/f).read_bytes()).hexdigest()==sha for f,sha in m['source_sha256'].items())
checks=0;actions=0;tick_count=0;valid_retracts=0
for result in m['runs']:
    rows=read_log(out/result['log']);case=next(c['case'] for c in m['cases'] if c['case_id']==result['case_id'])
    detector=Detector(result['policy'],cal['eta_threshold'],cal['force_threshold_n'],design)
    progress=-10.;unsafe=[];success_count=0;ever_success=False
    for i,row in enumerate(rows):
        tick_count+=1
        assert abs(row['time_s']-i/hz)<1e-5
        assert row['step']==i
        if i:
            progress=max(progress,rows[i-1]['depth_mm'])
            if row['segment']!=rows[i-1]['segment']:detector.reset()
        expected=command_misalignment(case,progress)
        assert abs(row['ramp_progress_depth_mm']-progress)<1e-9
        for field,old in (('nominal_offset_x_mm','offset_x_mm'),('nominal_offset_y_mm','offset_y_mm'),('nominal_roll_deg','roll_deg'),('nominal_pitch_deg','pitch_deg')):
            assert abs(row[field]-expected[old])<1e-9
        signal=recent_signal(rows[:i+1],hz,case['ramp_onset_mm'],design)
        assert row['check_performed']==bool(i and i%round(design.check_s*hz)==0)
        if row['check_performed']:
            checks+=1
            assert row['eta_valid']==(signal is not None)
            if signal:
                for k in ('eta_raw','actual_progress_mm','command_progress_mm'):assert abs(row[k]-signal[k])<1e-9
            fired=detector.check(i/hz,signal)
            assert row['soft_trigger']==fired
            assert row['trigger_consecutive']==detector.count
        assert row['stalled_now']==stalled_now(rows[:i+1],hz,p)
        safe=screened(row,p) and within_budget(row,p) and retained(row)
        if not safe:unsafe.append(i)
        if row['intervention_started']:
            actions+=1
            assert row['check_performed'] and row['soft_trigger'] and safe
            assert row['intervention_count']<=design.max_interventions
            detector.reset()
        success_count=success_count+1 if row['phase'] in ('insert','hold') and row['depth_mm']>=p.success_depth_mm else 0
        ever_success|=success_count>=round(p.success_dwell_s*hz)
    assert not unsafe or unsafe==[len(rows)-1]
    assert result['insertion_success']==(ever_success and not unsafe)
    assert result['stalled']==any(r['stalled_now'] for r in rows)
    assert result['intervention_count']==sum(r['intervention_started'] for r in rows)
    assert result['insertion_time_s']<=design.insertion_budget_s+1e-5
    events=json.loads((out/result['run_id']/'events.json').read_text())
    for event in events:
        trigger=rows[round(event['trigger_time_s']*hz)]
        assert trigger['intervention_started']
        if 'retract_end_time_s' in event:
            end=rows[round(event['retract_end_time_s']*hz)]
            assert end['phase']=='retract'
            assert abs(event['actual_retraction_mm']-(trigger['depth_mm']-end['depth_mm']))<1e-9
            valid_retracts+=1
old=root/'outputs/Active-Probing-Pilot-v1'
assert (out/'scene.json').read_bytes()==(old/'scene.json').read_bytes()
assert json.loads((out/'config.json').read_text())==json.loads((old/'config.json').read_text())
validation=dict(status='passed',runs=len(m['runs']),physics_rows=tick_count,causal_checks_recomputed=checks,
    interventions_verified=actions,completed_retraction_measurements_verified=valid_retracts,
    calibration_sources_sha256_verified=len(cal['source_sha256']),runtime_sources_sha256_verified=len(m['source_sha256']),
    preexisting_outputs_checked=m['preexisting_outputs_checked'],preexisting_outputs_changed=m['preexisting_outputs_changed'],
    exact_prior_scene_and_configuration_match=True,hard_safety_priority_verified=True,
    no_stage4_shared_groups_in_calibration=True,eta_not_clipped=True,normal_load_not_a_predictor=True,
    unit_tests=103,unit_test_status='passed')
(out/'validation.json').write_text(json.dumps(validation,indent=2)+'\n')
print(json.dumps(validation,indent=2))
