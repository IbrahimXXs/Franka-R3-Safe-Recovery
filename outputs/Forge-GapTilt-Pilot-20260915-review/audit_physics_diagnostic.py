#!/usr/bin/env python3
"""Read closed reference/withdrawal profiles; write the 120/240 Hz audit only."""
import csv
from dataclasses import asdict, replace
import hashlib
import json
import math
from pathlib import Path
import sys

REVIEW=Path(__file__).resolve().parent
PROJECT=REVIEW.parents[1]
sys.path.insert(0,str(PROJECT))
from research.future_stall import read_rows,stall_events
from research.forge_protocol import load_protocol,recovery_summary,screened
from research.forge_gap_metrics import recovery_phase_metrics,insertion_event_metrics,_window_peak


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def norm(a,b,keys):return math.sqrt(sum((a[k]-b[k])**2 for k in keys))
def angle(a,b):
    keys=('qw','qx','qy','qz')
    dot=abs(sum(a[k]*b[k] for k in keys))/math.sqrt(sum(a[k]**2 for k in keys)*sum(b[k]**2 for k in keys))
    return math.degrees(2*math.acos(min(1.,dot)))
def state(r):
    keys=('time_s','depth_mm','tip_x_mm','tip_y_mm','tilt_deg','qw','qx','qy','qz','hand_pose_0','hand_pose_1','hand_pose_2','wrist_force_n','normal_load_n')
    return {k:r[k] for k in keys}
def durations(rows,key,threshold,dt):
    runs=[];start=None
    for i,r in enumerate(rows+[{key:-math.inf}]):
        above=r[key]>threshold
        if above and start is None:start=i
        elif not above and start is not None:runs.append((start,i-1));start=None
    if not runs:return {'cumulative_s':0.,'longest_s':0.}
    lo,hi=max(runs,key=lambda pair:pair[1]-pair[0])
    a,b=rows[lo],rows[hi]
    return {'cumulative_s':sum(r[key]>threshold for r in rows)*dt,'longest_s':(hi-lo+1)*dt,
            'longest_first_sample_s':a['recovery_time_s'],'longest_last_sample_s':b['recovery_time_s'],
            'longest_actual_withdrawal_between_samples_mm':a['depth_mm']-b['depth_mm'],
            'longest_commanded_withdrawal_between_samples_mm':a['command_depth_mm']-b['command_depth_mm']}
def diffs(a,b,path=''):
    if isinstance(a,dict) and isinstance(b,dict):
        out=[]
        for k in sorted(set(a)|set(b)):out+=diffs(a.get(k),b.get(k),path+'.'+k)
        return out
    return [] if a==b else [{'field':path,'hz120':a,'hz240':b}]


def main():
    protocol_path=PROJECT/'experiments/forge_phase2.json'
    p=load_protocol(protocol_path)
    roots={120:PROJECT/'outputs/Forge-GapTilt-Pilot-20260915/gap_0200',
           240:PROJECT/'outputs/Forge-GapTilt-Diagnostic-240Hz-20260915'}
    report={'case_id':'g0200_d30_pitch_pos20_try00','scope':'Closed insertion.csv/final_retreat.csv, immutable config/scene/source snapshots, and completed study metadata for terminal policy summaries; no active replay CSV was read',
            'protocol_source':str(protocol_path),'protocol_sha256':sha(protocol_path),'runs':{}}
    raw={};configs={}
    for hz,root in roots.items():
        config=json.loads((root/'config.json').read_text());configs[hz]=config
        scene=json.loads((root/'scene.json').read_text());dt=config['sim']['dt']
        assert math.isclose(dt,1/hz)
        trial=root/report['case_id'];paths={name:trial/name for name in ('insertion.csv','final_retreat.csv')}
        hashes={name:sha(path) for name,path in paths.items()}
        insertion=read_rows(paths['insertion.csv']);recovery=read_rows(paths['final_retreat.csv'])
        assert hashes=={name:sha(path) for name,path in paths.items()},'Input changed during audit'
        raw[hz]=insertion
        pp=replace(p,effective_radial_clearance_mm=scene['conservative_radial_clearance_mm'])
        prefix_valid=all(screened(r,pp) for r in insertion)
        events=stall_events(insertion,asdict(pp),hz)
        window=round(pp.stall_window_s*hz)
        entered_events=[e for e in events if e['depth_mm']>0 and insertion[e['index']-window]['depth_mm']>0]
        retreat=[r for r in recovery if r['phase']=='retreat']
        peak=max(retreat,key=lambda r:r['wrist_force_n'])
        start=next(r for r in reversed(recovery) if r['phase']=='stop')
        phase=recovery_phase_metrics(recovery,dt)
        rsum=recovery_summary(recovery,pp,dt,prefix_valid=prefix_valid)
        first_release=next((r['recovery_time_s'] for r in retreat if start['depth_mm']-r['depth_mm']>=1),None)
        timeline=[min(insertion,key=lambda r:abs(r['time_s']-t)) for t in (8.,8.5,9.,9.5,10.,11.)]
        trigger=insertion_event_metrics(insertion,{'tilt_amplitude_deg':2.})
        trigger_row=next(r for r in insertion if r.get('tilt_triggered'))
        report['runs'][str(hz)]={
            'root':str(root),'input_sha256':hashes,'config_sha256':sha(root/'config.json'),'scene_sha256':sha(root/'scene.json'),
            'geometry_mesh_sha256':scene['socket_mesh_sha256'],'effective_radial_clearance_mm':scene['conservative_radial_clearance_mm'],
            'screen_threshold_mm':scene['conservative_radial_clearance_mm']*pp.penetration_fraction,
            'dt_s':dt,'initial_state':state(insertion[0]),'trigger_state':state(trigger_row),'tilt_trigger_and_completion':trigger,
            'all_progress_stall_events':events,'post_entry_stall_events':entered_events,
            'insertion_timeline':[state(r)|{'command_depth_mm':r['command_depth_mm']} for r in timeline],
            'insertion':{'success_depth_reached':max(r['depth_mm'] for r in insertion)>=pp.success_depth_mm,
                'terminal_depth_mm':insertion[-1]['depth_mm'],'max_depth_mm':max(r['depth_mm'] for r in insertion),
                'peak_wrist_force_n':max(r['wrist_force_n'] for r in insertion),
                'peak_wrist_force_mean100ms_n':_window_peak(insertion,'wrist_force_n',dt),
                'max_reported_overlap_mm':max(0.,-min(r['min_separation_mm'] for r in insertion)),
                'passes_current_screen':prefix_valid,'max_grasp_slip_mm':max(r['grasp_slip_mm'] for r in insertion),
                'max_grasp_slip_deg':max(r['grasp_slip_deg'] for r in insertion)},
            'continuous_withdrawal':rsum|phase|{
                'force_durations':{str(threshold):durations(retreat,'wrist_force_n',threshold,dt) for threshold in (.5,1.,1.5,2.)},
                'retreat_peak_state':{k:peak[k] for k in ('time_s','recovery_time_s','depth_mm','command_depth_mm','tilt_deg','wrist_force_n','wrist_force_world_2','fx','fy','fz','normal_load_n','min_separation_mm','grasp_slip_mm','grasp_slip_deg')},
                'actual_withdrawal_at_retreat_peak_mm':start['depth_mm']-peak['depth_mm'],
                'commanded_withdrawal_at_retreat_peak_mm':recovery[0]['depth_mm']-peak['command_depth_mm'],
                'first_actual_withdrawal_1mm_recovery_time_s':first_release,
                'max_retreat_reported_overlap_mm':max(0.,-min(r['min_separation_mm'] for r in retreat)),
                'max_recovery_grasp_slip_mm':max(r['grasp_slip_mm'] for r in recovery),
                'max_recovery_grasp_slip_deg':max(r['grasp_slip_deg'] for r in recovery)}}
    a,b=report['runs']['120'],report['runs']['240']
    report['initial_state_difference']={'peg_position_mm':norm(raw[120][0],raw[240][0],('tip_x_mm','tip_y_mm','depth_mm')),
        'peg_orientation_deg':angle(raw[120][0],raw[240][0]),
        'hand_position_mm':1000*norm(raw[120][0],raw[240][0],('hand_pose_0','hand_pose_1','hand_pose_2')),
        'trigger_peg_position_mm':norm(a['trigger_state'],b['trigger_state'],('tip_x_mm','tip_y_mm','depth_mm')),
        'trigger_peg_orientation_deg':angle(a['trigger_state'],b['trigger_state'])}
    report['config_differences']=diffs(configs[120],configs[240])
    names=sorted(set(x.name for x in (roots[120]/'source').glob('*.py'))|set(x.name for x in (roots[240]/'source').glob('*.py')))
    report['source_snapshot_differences']=[name for name in names if not (roots[120]/'source'/name).exists() or not (roots[240]/'source'/name).exists() or sha(roots[120]/'source'/name)!=sha(roots[240]/'source'/name)]
    report['geometry_diagnostics']={name:{'path':name,'sha256':sha(REVIEW/name)} for name in ('diagnostic_240hz_insertion_geometry.json','diagnostic_240hz_retreat_geometry.json')}
    report['conclusions']={
        'qualitative_reproduction':'Both runs stall after entering the hole and show delayed direct withdrawal under axial resistance before clearing safely.',
        'quantitative_convergence':False,
        'reason':'240 Hz has a larger instantaneous force spike but lower 100 ms load, much shorter uninterrupted high-force episodes, less work, earlier release and lower overlap. Initial states also differ, so this does not isolate timestep effects.',
        'first_stall_warning':'The first 240 Hz progress-stall checkpoint is above the hole; only the later post-entry stall is evidence of insertion obstruction.',
        'geometry_limit':'Independent local planar wall violations are not full mesh penetration or physical material deformation; adjacent-frame timing inference is unconfirmed.',
        'no_global_time_shift_applied':True,'current_screen_is_not_a_validated_physical_accuracy_threshold':True,
        'terminal_replays_not_assessed':True}
    report['terminal_policy_comparison']={}
    for hz,root in roots.items():
        metadata_path=root/'study.json'
        metadata=json.loads(metadata_path.read_text())
        assert metadata['status'] in ('complete','target_not_met','paused'), 'Study metadata still active'
        attempt=next(a for a in metadata['attempts'] if a['trajectory_id']==report['case_id'])
        assert attempt['status']=='complete', 'Candidate not complete'
        checkpoint=next(c for c in attempt['checkpoints'] if c['checkpoint_kind']=='terminal')
        keys=('policy','reason','replay_matched','replay_errors','label_eligible','safe_recovery',
              'numerically_valid','grasp_retained','max_wrist_force_n','max_wrist_torque_nm',
              'max_penetration','recovery_work_j','clear_time_s','recovery_censored')
        report['terminal_policy_comparison'][str(hz)]={
            'study_sha256':sha(metadata_path),'terminal_label':checkpoint['Y_R_tested'],
            'policies':[{k:v for k,v in probe.items() if k in keys or k.startswith(('stop_','realign_','retreat_'))}
                        for probe in checkpoint['probes']]}
    report['conclusions']['terminal_replays_not_assessed']=False
    report['conclusions']['terminal_policy_result']='At both rates both terminal policies replay-match exactly and clear safely. Realignment reduces the subsequent retreat load and work, but the shared stopping load remains and the realignment phase itself has load.'
    report['audit_script_sha256']=sha(Path(__file__))
    (REVIEW/'diagnostic_physics_audit.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({'initial_state_difference':report['initial_state_difference'],'source_snapshot_differences':report['source_snapshot_differences'],
                      'insertion_mean100ms':[a['insertion']['peak_wrist_force_mean100ms_n'],b['insertion']['peak_wrist_force_mean100ms_n']],
                      'release_1mm_s':[a['continuous_withdrawal']['first_actual_withdrawal_1mm_recovery_time_s'],b['continuous_withdrawal']['first_actual_withdrawal_1mm_recovery_time_s']]},indent=2))

if __name__=='__main__':main()
