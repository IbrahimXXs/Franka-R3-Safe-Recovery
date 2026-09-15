#!/usr/bin/env python3
"""Reproduce the completed 480 Hz boundary cases' motion/resistance audit."""
import gzip
import hashlib
import json
import math
from pathlib import Path
import sys

REVIEW=Path(__file__).resolve().parent
sys.path.insert(0,str(REVIEW.parents[1]))
from simulation.analyze_mechanics_states import load_profile, mean100ms_selection
from simulation.analyze_mechanics_contact_regions import region_measurements

STUDY=REVIEW.with_name('Forge-Mechanics-RateBoundary480-20260915')
manifest_bytes=(STUDY/'study.json').read_bytes(); manifest=json.loads(manifest_bytes)
assert manifest['status']=='complete' and all(a['status']=='complete' for a in manifest['attempts']), 'Wait until the study is complete'
scene_bytes=(STUDY/'scene.json').read_bytes(); scene=json.loads(scene_bytes)
dt=1/manifest['physics_hz']; stop_duration=manifest['protocol']['stop_duration_s']
hashes={}; output=[]


def window_stats(rows,indices):
    selected=[rows[i] for i in indices]; before=rows[indices[0]-1]; last=selected[-1]
    count=len(selected)
    return {'first_sample_index':indices[0],'last_sample_index':indices[-1],
        'baseline_sample_index':indices[0]-1,'sample_count':count,'sample_support_s':count*dt,
        'first_retreat_elapsed_s':selected[0]['recovery_time_s']-stop_duration,
        'last_retreat_elapsed_s':last['recovery_time_s']-stop_duration,
        'command_withdrawal_mm':before['command_depth_mm']-last['command_depth_mm'],
        'actual_withdrawal_mm':before['depth_mm']-last['depth_mm'],
        'actual_mean_axial_speed_mm_s':(before['depth_mm']-last['depth_mm'])/(count*dt),
        'mean_normal_load_n':sum(r['normal_load_n'] for r in selected)/count,
        'mean_wrist_force_n':sum(r['wrist_force_n'] for r in selected)/count,
        'mean_signed_contact_fz_n':sum(r['fz'] for r in selected)/count,
        'mean_positive_downward_resistance_n':sum(max(0,-r['fz']) for r in selected)/count,
        'mean_positive_friction_downward_resistance_n':sum(max(0,-r['friction_force_world_z_n']) for r in selected)/count,
        'normal_loaded_fraction_at_0p01n':sum(r['normal_load_n']>=.01 for r in selected)/count,
        'downward_resistance_above1n_fraction':sum(-r['fz']>1 for r in selected)/count,
        'start_end_tilt_deg':[before['tilt_deg'],last['tilt_deg']],
        'start_end_depth_mm':[before['depth_mm'],last['depth_mm']],
        'first_last_actual_withdrawal_mm':selected[0]['depth_mm']-last['depth_mm'],
        'first_last_command_withdrawal_mm':selected[0]['command_depth_mm']-last['command_depth_mm']}


for attempt in manifest['attempts']:
    if attempt['status']!='complete': continue
    path=STUDY/attempt['folder']/'final_retreat.csv'
    rows,hashes[str(path)]=load_profile(path)
    if not any(r['phase']=='retreat' for r in rows):
        output.append({'case_id':attempt['case_id'],'tilt_amplitude_deg':attempt['tilt_amplitude_deg'],'actual_retreat_observed':False,'reason':attempt['final_retreat'].get('reason')})
        continue
    stop_end=next(r for r in reversed(rows) if r['phase']=='stop' and r['recovery_time_s']>0)
    retreat=[i for i,r in enumerate(rows) if r['phase']=='retreat']
    checkpoints=[]
    for elapsed in [.1,.25,.5,2/3,.75,1.,1.5,2.,3.]:
        index=min(retreat,key=lambda i:abs(rows[i]['recovery_time_s']-stop_duration-elapsed))
        r=rows[index]
        checkpoints.append({'sample_index':index,'retreat_elapsed_s':r['recovery_time_s']-stop_duration,
            'command_withdrawal_from_stop_mm':stop_end['command_depth_mm']-r['command_depth_mm'],
            'actual_withdrawal_from_stop_mm':stop_end['depth_mm']-r['depth_mm'],
            'depth_minus_command_mm':r['depth_mm']-r['command_depth_mm'],
            **{k:r[k] for k in ('depth_mm','command_depth_mm','tilt_deg','normal_load_n','wrist_force_n','fz','grasp_slip_mm')}})
    windows={}; requested={}
    for low,high in [(0.,.25),(.25,.5),(.5,.75),(.75,1.),(1.,1.5),(1.5,2.)]:
        indices=[i for i in retreat if low+1e-9<rows[i]['recovery_time_s']-stop_duration<=high+1e-9]
        name=f'retreat_{low:g}_to_{high:g}s'
        if len(indices)!=round((high-low)/dt):
            windows[name]={'observed':False,'reason':'fixed_window_not_fully_observed'}
            continue
        windows[name]=window_stats(rows,indices);requested[name]=indices
    peak=mean100ms_selection(rows,dt)
    indices=list(range(peak['window_start_sample_index'],peak['index']+1))
    windows['wrist_peak100ms']=window_stats(rows,indices)
    requested['wrist_peak100ms']=indices
    assert abs(windows['wrist_peak100ms']['mean_wrist_force_n']-attempt['final_retreat']['retreat_peak_wrist_force_mean100ms_n'])<1e-8
    if attempt['tilt_amplitude_deg']==1.5:
        raw_path=path.with_name('final_retreat_contacts.jsonl.gz');raw_bytes=raw_path.read_bytes()
        hashes[str(raw_path)]=hashlib.sha256(raw_bytes).hexdigest()
        wanted=set(i for indices in requested.values() for i in indices); regions={}
        records=gzip.decompress(raw_bytes).splitlines()
        assert len(records)==len(rows)
        for index,line in enumerate(records):
            if index not in wanted:continue
            raw=json.loads(line)
            assert abs(raw['time_s']-rows[index]['time_s'])<1e-8
            regions[index]=region_measurements(raw['contacts']['normal_contacts'],scene['measured_bore_diameter_mm']/2000)
        for name,indices in requested.items():
            windows[name]['region_observations']={
                'both_wall_rim_sides_loaded_fraction':sum(regions[i]['wall_rim_both_sides_loaded'] for i in indices)/len(indices),
                'mean_upper_rim_load_n':sum(regions[i]['upper_rim_band_normal_load_n'] for i in indices)/len(indices),
                'mean_inner_wall_load_n':sum(regions[i]['inner_straight_wall_normal_load_n'] for i in indices)/len(indices),
                'max_abs_raw_normal_sum_minus_csv_n':max(abs(regions[i]['all_loaded_normal_load_n']-rows[i]['normal_load_n']) for i in indices)}
    run=best=0;end=None
    for i in retreat:
        run=run+1 if -rows[i]['fz']>1 else 0
        if run>best:best=run;end=i
    output.append({'case_id':attempt['case_id'],'tilt_amplitude_deg':attempt['tilt_amplitude_deg'],
        'source_csv':str(path),'profile_sample_count':len(rows),'checkpoints':checkpoints,'windows':windows,
        'max_consecutive_downward_resistance_above1n':{'sample_count':best,'support_s':best*dt,'end_sample_index':end},
        'maximum_grasp_slip_mm':max(r['grasp_slip_mm'] for r in rows),
        'retreat_peak_wrist_force_n':attempt['final_retreat']['retreat_peak_wrist_force_n'],
        'retreat_peak_wrist_mean100ms_n':attempt['final_retreat']['retreat_peak_wrist_force_mean100ms_n']})
def minimum_progress_window(rows,hz):
    count=round(.25*hz); candidates=[]
    for end,row in enumerate(rows):
        first=end-count+1; before=first-1
        if before<0 or any(r['phase']!='retreat' for r in rows[first:end+1]): continue
        command=rows[before]['command_depth_mm']-row['command_depth_mm']
        if command<1.-1e-9 or rows[before]['depth_mm']<=0: continue
        candidates.append((rows[before]['depth_mm']-row['depth_mm'],before,end,command))
    if not candidates:return None
    actual,before,end,command=min(candidates); selected=rows[before+1:end+1]
    return {'baseline_sample_index':before,'end_sample_index':end,'support_s':.25,
        'retreat_elapsed_start_s':rows[before]['recovery_time_s']-stop_duration,
        'retreat_elapsed_end_s':rows[end]['recovery_time_s']-stop_duration,
        'command_withdrawal_mm':command,'actual_withdrawal_mm':actual,
        'mean_normal_load_n':sum(r['normal_load_n'] for r in selected)/count,
        'mean_signed_contact_fz_n':sum(r['fz'] for r in selected)/count}


initial_comparisons=[]; minimum_progress_comparisons=[]
baseline=STUDY.with_name('Forge-Mechanics-Boundary-20260915')
baseline_manifest=json.loads((baseline/'study.json').read_text())
for attempt in manifest['attempts']:
    old=next(a for a in baseline_manifest['attempts'] if a['case_id']==attempt['case_id'])
    profiles=[]
    for folder in (baseline/old['folder'],STUDY/attempt['folder']):
        path=folder/'insertion.csv'; rows,hashes[str(path)]=load_profile(path); profiles.append(rows)
    before,after=[r[0] for r in profiles]
    fields=('depth_mm','tip_x_mm','tip_y_mm','tilt_deg','qw','qx','qy','qz','vx','vy','vz','omegax','omegay','omegaz')
    q0=[before[k] for k in ('qw','qx','qy','qz')];q1=[after[k] for k in ('qw','qx','qy','qz')]
    qdot=abs(sum(a*b for a,b in zip(q0,q1)))/math.sqrt(sum(a*a for a in q0)*sum(a*a for a in q1))
    delta={k:after[k]-before[k] for k in ('depth_mm','tip_x_mm','tip_y_mm','tilt_deg')}
    initial_comparisons.append({'case_id':attempt['case_id'],'reference_initial_240hz':{k:before[k] for k in fields},
        'reference_initial_480hz':{k:after[k] for k in fields},'delta_480_minus_240':delta,
        'relative_peg_base_position_difference_mm':math.sqrt(sum(delta[k]**2 for k in ('depth_mm','tip_x_mm','tip_y_mm'))),
        'orientation_difference_deg':math.degrees(2*math.acos(min(1.,qdot))),
        'recorded_initial_fields_equal':all(before[k]==after[k] for k in fields)})
    minimum_progress={'case_id':attempt['case_id']}
    for folder,hz in ((baseline/old['folder'],baseline_manifest['physics_hz']),
                      (STUDY/attempt['folder'],manifest['physics_hz'])):
        path=folder/'final_retreat.csv';rows,hashes[str(path)]=load_profile(path)
        minimum_progress[str(hz)+'hz']=minimum_progress_window(rows,hz)
    minimum_progress_comparisons.append(minimum_progress)
for path,digest in hashes.items():assert hashlib.sha256(Path(path).read_bytes()).hexdigest()==digest
result={'schema':'rate_boundary_motion_audit_v1','study_dir':str(STUDY),
    'study_sha256':hashlib.sha256(manifest_bytes).hexdigest(),'scene_sha256':hashlib.sha256(scene_bytes).hexdigest(),
    'analyzer_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'input_hashes':hashes,
    'methods':[
        'Retreat elapsed zero is the end of executed .25s STOP. Motion comparisons use the prior sample to cover N*dt, while first/last-only differences are separately retained.',
        'Command withdrawal is change in logged command depth; actual withdrawal is the opposite change in measured peg-base depth. Hand command and peg displacement need not coincide during pose-control tracking lag.',
        'Positive downward resistance=max(0,-contact_world_fz). Signed mean fz is retained separately to expose alternating pulses; average positive-only resistance is not signed net resistance.',
        'Fixed-time window .5-.75s is compared across both completed 480 Hz cases. The wrist-maximum100ms window is selected independently within each case; no same-state cross-case comparison is implied.',
        'Exploratory minimum-progress check scans every complete .25s retreat window beginning inside the hole with commanded withdrawal >=1mm. It is not a new formal stall or safety label and excludes slow commanded startup.',
        'Region definitions match analyze_mechanics_contact_regions.py; side threshold .01N, retained point threshold1e-6N. These are estimated loaded regions, not paired contacts.',
        'Recorded geometry and force signals remain simulator observations with unverified contact/pose within-step timing, not physical calibration.'
    ],'initial_state_comparisons':initial_comparisons,'minimum_progress_250ms_comparisons':minimum_progress_comparisons,'comparison_limitation':'Different reset-derived initial states and single trials: this is a timestep-sensitivity observation, not controlled initial-state numerical convergence.','cases':output}
(REVIEW/'rate_motion_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
for case in output:
    print(case['case_id'],case.get('windows',{}).get('retreat_0.5_to_0.75s'))
