"""Validate frozen-controller reruns and compose 1080p telemetry presentation videos."""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import cv2
import imageio_ffmpeg
from research.productivity_control_report import read_log
from research.productivity_detector_v2 import features
from research.force_productivity_analysis import audit_run
from research.productivity_detector_v2_generalization_report import paired_prefix
from research.forge_protocol import ForgeProtocol, match
from research.force_productivity_design import verify

ROOT=Path(__file__).resolve().parents[1]
SOURCE=ROOT/'outputs/Force-vs-Productivity-Dewedge-v1'
SPECS=(('ft030_axis_tilt_severe_v2','low_force_productivity_recovery','low_force_trigger_frame','normal',
        'Low force at the productivity trigger'),
       ('ft027_axis_tilt_moderate_v3','terminal_stagnation_recovery','terminal_trigger_frame','terminal',
        'Recovering from terminal stagnation'))
FPS=30
W,H=1920,1080
BG='#0B1422';PANEL='#142235';FG='#EFF5FB';MUTED='#A6B6C9';TEAL='#4BE1C0';AMBER='#FFBE65'
FONT='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
BOLD='/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
MONO='/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'


def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def save(p,obj):Path(p).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')
def font(size,bold=False,mono=False):return ImageFont.truetype(MONO if mono else BOLD if bold else FONT,size)


def state(row,success=False):
    if success:return 'SUCCESS'
    if row['intervention_started'] or row['phase'] in ('stop','relax'):return 'DE-WEDGE'
    if row['phase'] in ('unload_retract','unload_hold','post_verify_hold'):return 'RETRACT'
    if row['phase']=='rejoin' or row['retry_pose_latched']:return 'RETRY'
    return 'INSERT'


def telemetry(rows,i,case,hz,success=False):
    assert int(i)==i
    i=int(i)
    row=rows[i]
    signal=features(rows[:i+1],hz,case['ramp_onset_mm'])
    eta=signal['eta_raw'] if signal and signal['normal_valid'] else None
    # Display the detector's exact sampled value on the trigger frame.
    if row['intervention_started']:eta=row['eta_raw']
    endpoint=row['phase']=='hold' and abs(row['command_depth_mm']-20.)<1e-6
    if endpoint:eta=None
    return dict(step=i,time_s=row['time_s'],force_n=row['wrist_force_n'],eta=eta,
                commanded_depth_mm=row['command_depth_mm'],actual_depth_mm=row['depth_mm'],
                state=state(row,success),phase=row['phase'],endpoint_hold=endpoint,
                trigger_branch=row['trigger_branch'] if row['intervention_started'] else None,
                actual_unload_mm=row['unload_actual_mm'])


def draw_frame(raw,t,rows,spec,trigger,event,paused=False,success=False):
    cid,stem,png,branch,title=spec
    canvas=Image.new('RGB',(W,H),BG);d=ImageDraw.Draw(canvas)
    def text(x,y,s,size=24,color=FG,bold=False,mono=False):d.text((x,y),s,font=font(size,bold,mono),fill=color)
    text(28,19,'CONTACT PRODUCTIVITY  /  FROZEN CONTROLLER REPLAY',19,TEAL,True)
    text(26,51,title,39,FG,True)
    text(28,108,cid+'  |  FORGE peg insertion  |  120 Hz physics',20,MUTED)
    d.rounded_rectangle((1592,29,1892,108),radius=16,fill=TEAL if success else PANEL)
    text(1620,46,t['state'],31,BG if success else TEAL,True)
    # Fixed-camera render; no UI, pose manipulation, or image-generated content.
    canvas.paste(raw.resize((1312,738),Image.Resampling.LANCZOS),(24,153))
    d=ImageDraw.Draw(canvas)
    d.rounded_rectangle((1358,153,1896,891),radius=18,fill=PANEL)
    text(1385,175,'LIVE / REPLAYED TELEMETRY',18,MUTED,True)
    text(1385,210,'Wrist force magnitude',25)
    text(1380,246,f"{t['force_n']:.3f}",61,AMBER,mono=True)
    text(1715,275,'N',29,AMBER)
    text(1385,322,'Force threshold = 1.0 N',24,AMBER)
    d.line((1385,365,1866,365),fill='#35465B',width=2)
    text(1385,385,'Productivity eta',25)
    text(1380,420,'N/A' if t['eta'] is None else f"{t['eta']:.3f}",59,TEAL,mono=True)
    text(1385,495,'Eta threshold = 0.421',24,TEAL)
    if t['eta'] is None:
        reason='No commanded progress' if t['endpoint_hold'] else 'Outside valid insertion history'
        text(1385,535,reason,19,MUTED)
    d.line((1385,577,1866,577),fill='#35465B',width=2)
    text(1385,597,'Commanded depth',23,MUTED)
    text(1645,592,f"{t['commanded_depth_mm']:7.3f} mm",25,AMBER,mono=True)
    text(1385,640,'Actual depth',23,MUTED)
    text(1645,635,f"{t['actual_depth_mm']:7.3f} mm",25,TEAL,mono=True)
    text(1385,691,'Success depth = 19.5 mm',24)
    d.rounded_rectangle((1385,743,1866,768),radius=8,fill='#26374B')
    length=max(0.,min(1.,t['actual_depth_mm']/20.))*481
    if length>0:d.rounded_rectangle((1385,743,1385+length,768),radius=8,fill=TEAL)
    mark=1385+481*19.5/20
    d.line((mark,735,mark,779),fill=FG,width=3)
    text(1385,810,f"Replay t = {t['time_s']:.3f} s",23,MUTED,mono=True)
    # A translucent strip is represented by a dark solid backing for readability.
    d.rectangle((24,843,1336,891),fill='#152031')
    sub='Fixed camera  |  Recorded from the unchanged Productivity-v2 + de-wedging controller'
    text(40,855,sub,19,MUTED)
    d.rounded_rectangle((24,913,1896,1054),radius=18,fill=PANEL)
    if success:
        heading='SUCCESS  /  VERIFIED UNLOADING AND RETRY'
        detail=f"Actual retreat held >= 0.5 mm for >= 0.25 s  |  Final depth {t['actual_depth_mm']:.3f} mm"
    elif paused and branch=='normal':
        heading='PRODUCTIVITY TRIGGER  /  FORCE BELOW THRESHOLD'
        detail=f"Force {trigger['wrist_force_n']:.3f} N < 1.0 N   |   eta {trigger['eta_raw']:.3f} < 0.421   |   two consecutive checks"
    elif paused:
        heading='TERMINAL STAGNATION  /  COMMAND COMPLETE'
        detail=f"eta: N/A   |   Command {trigger['command_depth_mm']:.3f} mm   |   Actual {trigger['depth_mm']:.3f} mm < 19.5 mm"
    elif t['endpoint_hold']:
        heading='COMMAND COMPLETE  /  MONITORING TERMINAL STAGNATION'
        detail=f"eta: N/A   |   Actual depth {t['actual_depth_mm']:.3f} mm   |   Success depth = 19.5 mm"
    elif t['state']=='DE-WEDGE':
        heading='DE-WEDGE  /  RELAX LATERAL AND TILT PRELOAD'
        detail='Downward insertion stopped. The frozen recovery relaxes the imposed misalignment.'
    elif t['state']=='RETRACT':
        heading='RETRACT  /  VERIFY ACTUAL UNLOADING'
        detail=f"Actual retreat {t['actual_unload_mm']:.3f} mm   |   Required: >= 0.5 mm, held for 0.25 s"
    elif t['state']=='RETRY':
        heading='RETRY  /  RESUME FROM THE RELAXED POSE'
        detail='Verified unloading achieved. Retrying insertion with unchanged gains and safety limits.'
    else:
        heading='INSERT  /  MONITOR WRENCH AND ACHIEVED PROGRESS'
        detail='The detector compares recent actual depth progress with commanded depth progress.'
    text(48,930,heading,26,TEAL,True)
    text(48,975,detail,23,FG)
    if paused:text(48,1015,'TRIGGER FRAME HELD FOR 3 SECONDS  /  values and simulation time are paused',17,AMBER,True)
    elif success:text(48,1015,'SUCCESS FRAME HELD FOR 2 SECONDS',17,AMBER,True)
    else:text(48,1015,'Real-time playback  /  30 FPS  /  all telemetry from this recorded controller run',17,MUTED)
    return canvas


def validate_replay(directory,spec,m,plan,config):
    cid,stem,png,branch,title=spec
    replay=directory/'replays';rd=replay/cid
    rows=read_log(rd/'trajectory.csv');metrics=json.loads((rd/'metrics.json').read_text());events=json.loads((rd/'events.json').read_text())
    case=next(c for c in plan['cases'] if c['case_id']==cid)
    assert json.loads((rd/'condition.json').read_text())==case
    assert metrics['insertion_success'] and metrics['outcome']=='insertion_success'
    starts=[r for r in rows if r['intervention_started']]
    assert len(starts)==len(events)==metrics['intervention_count']==1
    trigger=starts[0];event=events[0]
    assert trigger['trigger_branch']==branch,(cid,trigger['trigger_branch'],branch)
    assert event['unloading_status']=='verified' and event['verified_before_retry']
    assert event['actual_retraction_mm']>=.5 and event['verification_time_s'] is not None
    assert event['retry_started_s']<rows[-1]['time_s']
    if branch=='normal':
        assert trigger['wrist_force_n']<1. and trigger['eta_raw']<config['normal_eta_threshold']
    else:
        assert trigger['phase']=='hold' and trigger['eta_raw'] is None
        assert abs(trigger['command_depth_mm']-20.)<1e-8 and trigger['depth_mm']<19.5
        assert trigger['terminal_rate_mm_s']<=config['terminal_rate_mm_s']
    proxy=dict(metrics,case_id=cid,policy='productivity',run_id=cid,log=cid+'/trajectory.csv')
    mm=dict(m,preexisting_outputs_changed=[],preexisting_outputs_checked=json.loads((replay/'preservation.json').read_text())['files_checked'])
    audit=audit_run(replay,mm,proxy,rows,config)
    old=next(r for r in m['runs'] if r['case_id']==cid and r['policy']=='productivity')
    source_rows=read_log(SOURCE/old['log']);p=ForgeProtocol(**m['protocol'])
    initial_ok,initial_errors=match(source_rows[0],rows[0],p)
    prefix=paired_prefix(source_rows,rows,p)
    frames=json.loads((rd/'frames.json').read_text())
    assert any(f['step']==trigger['step'] for f in frames)
    assert all(frames[i]['step']<frames[i+1]['step'] for i in range(len(frames)-1))
    return rows,metrics,event,trigger,frames,dict(case_id=cid,expected_branch=branch,actual_branch=trigger['trigger_branch'],
        trigger_time_s=trigger['time_s'],trigger_force_n=trigger['wrist_force_n'],trigger_eta=trigger['eta_raw'],
        trigger_command_depth_mm=trigger['command_depth_mm'],trigger_actual_depth_mm=trigger['depth_mm'],
        max_wrist_force_n=metrics['max_wrist_force_n'],force_below_1N_at_trigger=trigger['wrist_force_n']<1.,
        force_below_1N_entire_run=metrics['max_wrist_force_n']<1.,insertion_success=True,
        verified_actual_retreat_mm=event['actual_retraction_mm'],verification_time_s=event['verification_time_s'],
        successful_retry=True,final_depth_mm=metrics['final_depth_mm'],audit=audit,
        initial_state_matches_source_tolerances=initial_ok,initial_state_errors=initial_errors,
        source_preintervention_comparison=prefix,recorded_rgb_frames=len(frames),
        telemetry_source='new live execution of exact frozen controller and condition; not original numbers pasted onto new motion',
        source_log_sha256=sha(SOURCE/old['log']),replay_log_sha256=sha(rd/'trajectory.csv'))


def encode(directory,spec,data):
    rows,metrics,event,trigger,frames,validation=data
    cid,stem,png,branch,title=spec;rd=directory/'replays'/cid
    case=json.loads((rd/'condition.json').read_text())['case']
    path=directory/(stem+'.mp4');assert not path.exists(),path
    ffmpeg=imageio_ffmpeg.get_ffmpeg_exe()
    cmd=[ffmpeg,'-hide_banner','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','1920x1080','-r','30','-i','-',
         '-an','-c:v','libx264','-preset','medium','-crf','18','-pix_fmt','yuv420p','-movflags','+faststart',str(path)]
    frame_map=[]
    with (directory/'validation'/(stem+'_encoding.log')).open('w') as log:
        proc=subprocess.Popen(cmd,stdin=subprocess.PIPE,stderr=log)
        try:
            for j,f in enumerate(frames):
                i=f['step'];last=j==len(frames)-1;paused=i==trigger['step']
                t=telemetry(rows,i,case,120,success=last)
                raw=Image.open(rd/f['file']).convert('RGB')
                composed=draw_frame(raw,t,rows,spec,trigger,event,paused=paused,success=last)
                if paused:composed.save(directory/(png+'.png'))
                repeats=90 if paused else 60 if last else 1
                rgb=np.asarray(composed).tobytes()
                for k in range(repeats):
                    proc.stdin.write(rgb)
                    frame_map.append(dict(video_frame=len(frame_map),video_time_s=len(frame_map)/FPS,
                        source_step=i,source_time_s=t['time_s'],held_trigger=paused,held_success=last,**{key:value for key,value in t.items() if key not in ('step','time_s')}))
            proc.stdin.close();code=proc.wait()
            assert code==0,(stem,code)
        except BaseException:
            proc.kill();proc.wait();raise
    mapping=directory/'validation'/(stem+'_frame_telemetry.csv')
    with mapping.open('w') as out:
        writer=csv.DictWriter(out,fieldnames=list(frame_map[0]));writer.writeheader();writer.writerows(frame_map)
    cap=cv2.VideoCapture(str(path))
    info=dict(width=int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),height=int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
              fps=cap.get(cv2.CAP_PROP_FPS),frames=int(cap.get(cv2.CAP_PROP_FRAME_COUNT)))
    assert info['width']==W and info['height']==H and info['fps']==FPS and info['frames']==len(frame_map),info
    decoded=0
    while True:
        ok,frame=cap.read()
        if not ok:break
        assert frame.shape==(H,W,3);decoded+=1
    cap.release();assert decoded==len(frame_map)
    subprocess.run([ffmpeg,'-v','error','-xerror','-i',str(path),'-f','null','-'],check=True)
    validation.update(file=path.name,trigger_frame=png+'.png',video=info,decoded_frames=decoded,
        duration_s=len(frame_map)/FPS,mp4_sha256=sha(path),trigger_frame_sha256=sha(directory/(png+'.png')),
        trigger_video_frame=next(i for i,f in enumerate(frame_map) if f['held_trigger']),
        frame_telemetry_csv=str(mapping.relative_to(directory)),frame_telemetry_sha256=sha(mapping),
        video_clock='30 FPS real-time samples, plus labeled 3 s trigger hold and 2 s final success hold')
    return validation


def main(directory):
    directory=Path(directory).resolve();replay=directory/'replays'
    (directory/'validation').mkdir(exist_ok=True)
    m=json.loads((SOURCE/'experiment.json').read_text());plan=json.loads((SOURCE/'condition_plan.json').read_text())
    config=json.loads((SOURCE/'detector_v2_config.json').read_text());verify(ROOT,plan)
    assert json.loads((replay/'config.json').read_text())==json.loads((SOURCE/'config.json').read_text())
    assert json.loads((replay/'scene.json').read_text())==json.loads((SOURCE/'scene.json').read_text())
    # Reuse the original full detector/recovery/safety auditor against the new logs.
    shutil.copy2(SOURCE/'calibration.json',replay/'calibration.json')
    for name,digest in m['source_sha256'].items():
        dest=replay/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(SOURCE/'source'/name,dest)
        assert sha(dest)==digest and sha(ROOT/name)==digest
    data=[validate_replay(directory,spec,m,plan,config) for spec in SPECS]
    save(directory/'validation'/'replay_validation.json',[d[-1] for d in data])
    results=[encode(directory,spec,d) for spec,d in zip(SPECS,data)]
    preservation=json.loads((replay/'preexisting_outputs.json').read_text())
    changed=[f for f,st in preservation.items() if not Path(f).is_file() or [Path(f).stat().st_size,Path(f).stat().st_mtime_ns]!=st]
    assert not changed
    manifest=dict(schema='Presentation-Videos-v1',created_utc=datetime.now(timezone.utc).isoformat(),
        source_experiment=str(SOURCE.relative_to(ROOT)),source_condition_plan_sha256=sha(SOURCE/'condition_plan.json'),
        detector_v2_config_sha256=sha(SOURCE/'detector_v2_config.json'),force_reference_threshold_n=1.,
        physics_hz=120,video_fps=30,resolution=[W,H],codec='H.264 / MP4 / yuv420p',
        physics_gains_detector_recovery_safety_initial_condition_settings_unchanged=True,
        same_seed_and_solver_priming=True,initial_seed=m['seed'],camera=json.loads((replay/'camera.json').read_text()),
        recording_method='Headless RGB render product sampled after each fourth physics tick; rendering checked not to advance physics or change peg/joints.',
        eta_display='Causal frozen detector feature from actual replay history; N/A at endpoint hold or invalid history. Never substitute eta=0 for zero commanded motion.',
        state_labels='INSERT includes approach/endpoint hold; DE-WEDGE includes stop/relaxation; RETRACT includes verification hold; RETRY includes rejoin; SUCCESS only after verified success dwell.',
        caution='ft030 is low-force at its trigger, not below 1 N for the entire trajectory. Earlier transient force spikes remain visible in live telemetry.',
        old_output_files_checked=len(preservation),old_outputs_changed=[],frozen_source_sha256=m['source_sha256'],
        presentation_source_sha256={f:sha(ROOT/f) for f in ('simulation/record_productivity_videos.py','research/presentation_videos.py')},
        videos=results,status='validated')
    save(directory/'video_manifest.json',manifest)
    print(json.dumps(results,indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path,nargs='?',default=ROOT/'outputs/Presentation-Videos-v1')
    main(parser.parse_args().directory)
