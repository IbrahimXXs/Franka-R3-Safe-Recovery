"""Offline, synchronized visualization of two immutable ft037 trajectory logs.

No simulator imports, physics execution, or modifications to source data. The
pose diagrams are explicitly schematic orthographic projections, not footage.
"""
import argparse
import csv
from functools import lru_cache
import hashlib
import json
import math
from pathlib import Path
import subprocess

import cv2
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from scipy.spatial.transform import Rotation

from research.productivity_control_report import read_log
from research.productivity_detector_v2 import features
from research.force_productivity_analysis import audit_run
from research.productivity_detector_v2_generalization_report import paired_prefix
from research.forge_protocol import ForgeProtocol

ROOT = Path(__file__).resolve().parents[1]
CASE = 'ft037_oblique_tilt_severe_v1'
STEM = 'ft037_force_vs_productivity'
FPS, WIDTH, HEIGHT = 30, 1920, 1080
BG, PANEL, FG, MUTED = '#0B1422', '#142235', '#EFF5FB', '#A6B6C9'
AMBER, TEAL, RED = '#FFBE65', '#4BE1C0', '#FF7D85'


def sha(path):
    h = hashlib.sha256()
    with path.open('rb') as f:
        for b in iter(lambda: f.read(1024 * 1024), b''):
            h.update(b)
    return h.hexdigest()


@lru_cache(None)
def font(size, bold=False):
    suffix = '-Bold' if bold else ''
    return ImageFont.truetype(f'/usr/share/fonts/truetype/dejavu/DejaVuSans{suffix}.ttf', size)


def load(source):
    experiment = json.loads((source / 'experiment.json').read_text())
    v2 = json.loads((source / 'detector_v2_config.json').read_text())
    force = json.loads((source / 'force_detector_config.json').read_text())
    case = next(c for c in experiment['cases'] if c['case_id'] == CASE)
    lanes = []
    for policy in ('force', 'productivity'):
        run = next(r for r in experiment['runs'] if r['case_id'] == CASE and r['policy'] == policy)
        rows = read_log(source / run['log'])
        directory = source / run['run_id']
        metrics = json.loads((directory / 'metrics.json').read_text())
        events = json.loads((directory / 'events.json').read_text())
        audit = audit_run(source, experiment, run, rows, v2)
        etas = []
        for i, row in enumerate(rows):
            f = features(rows[max(0, i-round(experiment['physics_hz']*.5)):i+1],
                         experiment['physics_hz'], case['ramp_onset_mm'])
            eta = f['eta_raw'] if f and f['normal_valid'] else None
            if row['eta_valid']:
                assert eta is not None and abs(eta-row['eta_raw']) < 1e-9
            etas.append(eta)
        lanes.append(dict(policy=policy, rows=rows, times=np.array([r['time_s'] for r in rows]),
                          etas=etas, metrics=metrics, events=events, audit=audit))
    assert lanes[0]['metrics']['insertion_success'] and len(lanes[0]['events']) == 1
    assert lanes[0]['events'][0]['verified_before_retry']
    assert lanes[1]['metrics']['outcome'] == 'grasp_retention_limit'
    assert not lanes[1]['events'] and not any(r['intervention_started'] for r in lanes[1]['rows'])
    assert [r['trigger_branch'] for r in lanes[0]['rows'] if r['intervention_started']] == ['force']
    return experiment, v2, force, case, lanes


def sample(lane, t):
    # Causal zero-order hold: no interpolation, extrapolation, or future samples.
    i = max(0, min(len(lane['rows'])-1, int(np.searchsorted(lane['times'], t, side='right')-1)))
    row = lane['rows'][i]
    ended = t >= lane['times'][-1]
    triggered = bool(lane['events'] and t >= lane['events'][0]['trigger_time_s']-1e-8)
    phase = row['phase']
    if ended:
        state = 'SUCCESS' if lane['metrics']['insertion_success'] else 'GRASP RETENTION LIMIT'
    elif row['intervention_started']:
        state = 'FORCE TRIGGER'
    elif phase in ('stop', 'relax'):
        state = 'DE-WEDGE'
    elif phase in ('unload_retract', 'unload_hold', 'post_verify_hold'):
        state = 'RETRACT'
    elif phase == 'rejoin' or row['retry_pose_latched']:
        state = 'RETRY'
    else:
        state = 'INSERT' if phase in ('insert', 'hold') else 'APPROACH'
    return dict(i=i, row=row, ended=ended, triggered=triggered, state=state, eta=lane['etas'][i])


def compose(t, lanes, force_config, v2, end, final_hold=False):
    image = Image.new('RGB', (WIDTH, HEIGHT), BG)
    d = ImageDraw.Draw(image)
    def text(x, y, value, size=22, color=FG, bold=False):
        d.text((x, y), str(value), font=font(size, bold), fill=color)
    text(28, 15, 'FORCE vs PRODUCTIVITY  /  RECORDED EXPERIMENT', 25, FG, True)
    text(28, 54, CASE + '  |  same condition, separate recorded runs', 22, MUTED)
    text(1400, 16, f'Comparison t = {t:6.3f} s', 26, TEAL, True)
    text(1400, 57, 'FINAL FRAME HOLD' if final_hold else '0.5x playback  |  shared clock', 21, MUTED)
    current = []
    for n, lane in enumerate(lanes):
        x = 24 + n*952
        color = AMBER if n == 0 else TEAL
        s = sample(lane, t); current.append(s)
        r, i = s['row'], s['i']
        failure = s['ended'] and not lane['metrics']['insertion_success']
        status_color = RED if failure else color
        d.rounded_rectangle((x, 104, x+920, 1018), radius=20, fill=PANEL)
        title = 'Force-triggered de-wedging' if n == 0 else 'Productivity-v2-triggered de-wedging'
        text(x+24, 122, title, 29 if n == 0 else 27, color, True)
        text(x+24, 165, f"Recorded sample: {r['time_s']:.3f} s", 21, MUTED)
        d.rounded_rectangle((x+20, 204, x+900, 253), radius=10, fill=status_color)
        text(x+35, 211, s['state'], 28, BG, True)
        values = [('Actual depth', f"{r['depth_mm']:.3f} mm"),
                  ('Wrist force ||F||', f"{r['wrist_force_n']:.3f} N"),
                  ('Productivity eta', 'N/A' if s['eta'] is None else f"{s['eta']:.3f}")]
        for j, (label, value) in enumerate(values):
            text(x+24+j*300, 271, label, 21, MUTED)
            text(x+24+j*300, 302, value, 32, color, True)
        text(x+24, 351, f"Command: {r['command_depth_mm']:.3f} mm", 20)
        text(x+330, 351, f"Force threshold: {force_config['threshold_n']:.1f} N", 20, AMBER)
        text(x+636, 351, f"Eta threshold: {v2['normal_eta_threshold']:.3f}", 20, TEAL)

        # Orthographic centerline projections: actual quaternion and actual tip.
        # Socket and shaft are schematic; no claimed camera/mesh reconstruction.
        text(x+24, 399, 'ACTUAL POSE / SCHEMATIC', 17, MUTED, True)
        q = [r['qx'], r['qy'], r['qz'], r['qw']]
        axis = Rotation.from_quat(q).apply([0., 0., 1.])
        for j, key in enumerate(('tip_x_mm', 'tip_y_mm')):
            cx, mouth, scale = x+130+j*190, 622, 2.65
            text(cx-40, 429, 'X-Z view' if j == 0 else 'Y-Z view', 17, MUTED)
            for a, b in ((-23, -4.5), (4.5, 23)):
                d.rectangle((cx+a*scale, mouth, cx+b*scale, mouth+25*scale), fill='#35465B')
            d.line((cx-72, mouth, cx+72, mouth), fill=MUTED, width=1)
            d.line((cx, mouth-163, cx, mouth+66), fill='#35465B', width=1)
            p0 = np.array([r[key], -r['depth_mm']])
            p1 = p0 + 50*np.array([axis[j], axis[2]])
            side = np.array([axis[2], -axis[j]])*3.993
            points = [p0-side, p0+side, p1+side, p1-side]
            coords = [(cx+p[0]*scale, mouth-p[1]*scale) for p in points]
            d.polygon(coords, fill=color, outline=FG)
            d.ellipse((cx+p0[0]*scale-3, mouth-p0[1]*scale-3,
                       cx+p0[0]*scale+3, mouth-p0[1]*scale+3), fill=FG)
        text(x+32, 704, 'Equal scales  |  section through hole', 16, MUTED)
        rx = x+468
        detector = ('FORCE TRIGGER' if s['triggered'] else 'Force detector monitoring') if n == 0 else (
            'No trigger before safety stop' if s['ended'] else 'Productivity not triggered yet')
        text(rx, 402, 'DETECTOR STATE', 17, MUTED, True)
        text(rx, 431, detector, 22, status_color, True)
        text(rx, 469, 'RECORDED PHASE', 17, MUTED, True)
        text(rx, 496, r['phase'].upper().replace('_', ' '), 22)
        if s['ended']:
            text(rx, 538, f"Recording ended at {r['time_s']:.3f} s", 20, status_color)
            text(rx, 568, 'Final sample held; no later data', 19, MUTED)
            if failure:
                text(rx, 608, f"Grasp slip: {r['grasp_slip_mm']:.6f} mm", 20, RED)
                text(rx, 639, 'No recovery was triggered', 20, RED)
            else:
                text(rx, 608, 'Verified unload + successful retry', 20, color)
                text(rx, 639, f"Final depth: {r['depth_mm']:.3f} mm", 20, color)
        elif s['triggered']:
            event = lane['events'][0]
            text(rx, 538, f"FORCE TRIGGER at {event['trigger_time_s']:.3f} s", 21, AMBER, True)
            text(rx, 571, f"F = {event['wrist_force_n']:.3f} N; eta = {event['eta_raw']:.3f}", 21)
            if s['state'] == 'RETRACT':
                text(rx, 608, f"Actual retreat: {r['unload_actual_mm']:.3f} mm", 20)
                text(rx, 639, 'Verify >= 0.5 mm for 0.25 s', 20, MUTED)
            elif s['state'] == 'RETRY':
                text(rx, 608, 'Verified unloading achieved', 20, color)
                text(rx, 639, 'Insertion resumed', 20, color)
            else:
                text(rx, 608, 'Stop insertion; relax offset / tilt', 20)
        else:
            text(rx, 540, 'eta: causal 0.5 s progress ratio', 19, MUTED)
            text(rx, 571, 'N/A outside eligible history', 19, MUTED)
            checks = [v for v in lane['rows'][:i+1] if v['check_performed']]
            if checks:
                check = checks[-1]
                text(rx, 614, f"Last detector check: {check['time_s']:.3f} s", 19)
                text(rx, 644, f"Consecutive count: {int(check['trigger_consecutive'])} / 2", 19, MUTED)

        # Identical axes, past data only, and explicit threshold reference lines.
        for y, key, label, threshold, limits, c in (
            (768, 'wrist_force_n', 'Wrist force [N]', force_config['threshold_n'], (0., 2.6), AMBER),
            (912, 'eta', 'Productivity eta', v2['normal_eta_threshold'], (0., 1.4), TEAL)):
            left, right, bottom = x+155, x+883, y+67
            text(x+23, y-25, label, 17, MUTED)
            def point(tt, value):
                return (left+tt/end*(right-left), bottom-(value-limits[0])/(limits[1]-limits[0])*67)
            for value in (limits[0], threshold, limits[1]):
                yy = point(0, value)[1]
                d.line((left, yy, right, yy), fill=c if value == threshold else '#35465B', width=1)
                text(x+94, yy-9, f'{value:.2f}', 14, MUTED)
            series = lane['etas'] if key == 'eta' else [v[key] for v in lane['rows']]
            segment = []
            for k in range(i+1):
                value = series[k]
                if value is None or not math.isfinite(value):
                    if len(segment)>1: d.line(segment, fill=c, width=3)
                    segment = []
                else:
                    segment.append(point(lane['times'][k], value))
            if len(segment)>1: d.line(segment, fill=c, width=3)
            cursor = point(r['time_s'], 0)[0]
            d.line((cursor, y, cursor, bottom), fill=FG, width=2)
            for tick in (0, 3, 6, 9, 12):
                text(point(tick, 0)[0]-5, bottom+3, str(tick), 14, MUTED)
            text(right-54, y-24, 'time [s]', 14, MUTED)
    text(28, 1037, 'Recorded-data animation, not camera footage. No simulation rerun. Ended runs retain their last recorded sample.', 20, MUTED)
    return image, current


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'outputs/Force-vs-Productivity-Dewedge-v1')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    source = args.source.resolve()
    output = args.output or source/'videos'/f'{STEM}.mp4'
    if output.exists():
        raise FileExistsError(f'Refusing to overwrite {output}')
    # Snapshot every existing experiment file before adding the authorized video.
    before = {str(p.relative_to(source)): sha(p) for p in source.rglob('*') if p.is_file()}
    experiment, v2, force, case, lanes = load(source)
    output.parent.mkdir(parents=True, exist_ok=True)
    end = max(lane['times'][-1] for lane in lanes)
    playback_speed = .5
    clock = np.minimum(np.arange(math.ceil(end/playback_speed*FPS)+1)*playback_speed/FPS, end)
    clock = np.concatenate((clock, np.full(3*FPS, end)))
    ffmpeg = imageio_ffmpeg.get_ffmpeg_exe()
    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-f', 'rawvideo', '-pix_fmt', 'rgb24',
               '-s', '1920x1080', '-r', str(FPS), '-i', '-', '-an', '-c:v', 'libx264',
               '-preset', 'medium', '-crf', '18', '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(output)]
    mapping = []
    previews = {'force_trigger': lanes[0]['events'][0]['trigger_time_s'],
                'grasp_limit': lanes[1]['times'][-1], 'final': end}
    with output.with_suffix('.encoding.log').open('w') as log:
        proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=log)
        try:
            for j, t in enumerate(clock):
                frame, samples = compose(float(t), lanes, force, v2, end, j>=len(clock)-3*FPS)
                for label, target in list(previews.items()):
                    if t>=target-1e-8:
                        frame.save(output.with_name(f'{STEM}_{label}.png'))
                        del previews[label]
                for lane, s in zip(lanes, samples):
                    mapping.append(dict(video_frame=j, video_time_s=j/FPS, comparison_time_s=float(t),
                        policy=lane['policy'], source_step=int(s['row']['step']), source_time_s=s['row']['time_s'],
                        actual_depth_mm=s['row']['depth_mm'], command_depth_mm=s['row']['command_depth_mm'],
                        wrist_force_n=s['row']['wrist_force_n'], eta=s['eta'], state=s['state'],
                        phase=s['row']['phase'], recording_ended=s['ended']))
                proc.stdin.write(frame.tobytes())
                if j % 180 == 0: print(f'Encoded {j}/{len(clock)} frames', flush=True)
            proc.stdin.close()
            assert proc.wait() == 0
        except BaseException:
            proc.kill(); proc.wait(); raise
    with output.with_name(f'{STEM}_frame_telemetry.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(mapping[0])); writer.writeheader(); writer.writerows(mapping)
    cap = cv2.VideoCapture(str(output))
    assert (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))) == (WIDTH, HEIGHT)
    assert cap.get(cv2.CAP_PROP_FPS) == FPS
    count = 0
    while cap.read()[0]: count += 1
    cap.release()
    assert count == len(clock)
    assert all(sha(source/name) == digest for name, digest in before.items())
    for row in mapping:
        assert row['source_time_s'] <= row['comparison_time_s']+1e-10
    manifest = dict(schema='Recorded-Comparison-Video-v1', condition=case, video=str(output.relative_to(source)),
        render='Offline orthographic pose schematic plus recorded telemetry; no camera footage or simulation execution',
        size=[WIDTH, HEIGHT], fps=FPS, frames=count, duration_s=count/FPS,
        playback_speed=playback_speed, final_frame_hold_s=3, synchronization='shared time_s, causal sample hold',
        eta='Unclipped causal 0.5 s ratio reconstructed with frozen detector feature eligibility; checked against every logged valid eta',
        geometry='Illustrative 50 mm shaft, 7.986 mm diameter and 9 mm socket; actual tip position and quaternion; no collision reconstruction',
        ended_run_handling='Last sample frozen, endpoint timestamp and outcome shown; no extrapolation',
        preexisting_files_checked=len(before), preexisting_files_changed=[], video_sha256=sha(output),
        source_sha256=before,
        strict_preintervention_state_match=paired_prefix(lanes[0]['rows'], lanes[1]['rows'], ForgeProtocol(**experiment['protocol'])),
        policies={lane['policy']: dict(outcome=lane['metrics']['outcome'], final_time_s=float(lane['times'][-1]),
            final_depth_mm=lane['rows'][-1]['depth_mm'], events=lane['events'], audit=lane['audit']) for lane in lanes},
        validation=dict(full_decode_passed=True, original_files_unchanged=True,
            force_trigger_and_verified_retry_success=True, productivity_no_trigger_and_grasp_limit=True,
            no_future_telemetry_samples=True))
    output.with_name(f'{STEM}_manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(output=str(output), frames=count, duration_s=count/FPS,
                         preexisting_files_unchanged=len(before)), indent=2))


if __name__ == '__main__':
    main()
