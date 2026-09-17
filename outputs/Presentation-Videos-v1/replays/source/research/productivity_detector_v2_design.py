"""Preassigned targeted development design; prior benchmark outcomes are not inputs."""
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_generalization import FROZEN_SOURCES, signature, sha

FAMILIES = ('successful_controls', 'terminal_band', 'oblique_tilt', 'combined', 'rapid_ramp')
SCHEMA = 'Contact-Productivity-DetectorV2-Dev'


def conditions():
    result = []
    onsets = ((6.5,9.,11.,13.5), (6.5,9.,13.5,16.5), (9.,11.,13.5,16.5), (8.5,11.5,14.,16.), (15.5,16.5,17.5,18.))
    for fi, family in enumerate(FAMILIES):
        for group, (sx,sy) in enumerate(((1,1),(-1,-1),(1,-1),(-1,1))):
            for level in range(4):
                if fi == 0: offset, tilt = .2+.12*level, 1.+.5*level
                elif fi == 1: offset, tilt = (.45+.08*level if group%2 else 0.), (3.1,3.35,3.65,3.9)[level]
                elif fi == 2: offset, tilt = 0., 6.+.6*level
                elif fi == 3: offset, tilt = .85+.15*level, 5.5+.75*level
                else: offset, tilt = (0. if group%2==0 else .6+.15*level), 4.5+level
                angle = math.radians((25,55,35,65)[group])
                x, y = sx*offset*math.cos(angle), sy*offset*math.sin(angle)
                roll, pitch = sy*tilt*math.sin(angle), -sx*tilt*math.cos(angle)
                cid = f'd{len(result):03d}_{family}_s{group}_l{level}'
                gid = f'{family}_sign_onset_{group}'
                case = dict(family=family, offset_x_mm=0., offset_y_mm=0., roll_deg=0., pitch_deg=0.,
                    insertion_duration_s=8., ramp_onset_mm=onsets[fi][group], ramp_full_depth_mm=20.,
                    final_offset_x_mm=x, final_offset_y_mm=y, final_roll_deg=roll, final_pitch_deg=pitch,
                    path_group_id=gid, split_group_id=gid, sample_role='detector_v2_development')
                result.append(dict(case_id=cid, family=family, group_id=gid,
                    split='validation' if group == (fi+1)%4 else 'calibration', case=case))
    return result


def make_plan(root):
    root = Path(root); cc = conditions()
    # Read only the pre-collection parameter plan, never the 48-condition result files.
    old_plan = root/'experiments/productivity_generalization_frozen.json'
    old_paths = {signature(c['case']) for c in json.loads(old_plan.read_text())['cases']}
    assert len({signature(c['case']) for c in cc}) == 80
    assert not any(signature(c['case']) in old_paths for c in cc)
    prior = root/'outputs/Contact-Productivity-Dewedge-v1/experiment.json'
    previous = json.loads(prior.read_text())
    hashes = {f:sha(root/f) for f in FROZEN_SOURCES}
    assert all(h == previous['source_sha256'][f] for f,h in hashes.items())
    rng = random.Random(20260917); order = cc.copy(); rng.shuffle(order)
    nominal = [dict(case_id=c['case_id'], policy='nominal', stage='reference') for c in order]
    rng.shuffle(order); closed = []
    for i,c in enumerate(order):
        for policy in (('v1','v2') if i%2==0 else ('v2','v1')):
            closed.append(dict(case_id=c['case_id'], policy=policy, stage='comparison'))
    return dict(schema=SCHEMA, frozen_at_utc=datetime.now(timezone.utc).isoformat(), cases=cc,
        nominal_schedule=nominal, comparison_schedule=closed, design=asdict(Design()),
        unloading_design=asdict(UnloadingDesign()), dewedge_design=asdict(DewedgeDesign()),
        policy_source_sha256=hashes, previous_experiment='outputs/Contact-Productivity-Dewedge-v1',
        previous_manifest_sha256=sha(prior), excluded_benchmark_plan_sha256=sha(old_plan), exact_benchmark_path_overlaps=0,
        calibration_grid=dict(urgent_eta_threshold=[.025,.05,.1,.15,.2],
            urgent_deficit_acceleration_mm_s2=[.1,.25,.5,1.,2.], terminal_rate_mm_s=[.01,.025,.05,.1]),
        selection=dict(max_additional_successful_trajectory_alerts=0, minimum_useful_lead_s=.1,
            objective='Maximize failed nominal trajectories newly covered safely or alerted >=0.1 s earlier; zero additional alerts on successful calibration trajectories. Then maximize capped lead, then prefer conservative thresholds.',
            calibration_groups=15, validation_groups=5, freeze_before_closed_loop=True),
        no_old_result_inputs=True, no_final_generalization_test=True)


def verify(root, plan):
    root=Path(root)
    if plan['cases'] != conditions(): raise ValueError('Development design changed')
    for f,h in plan['policy_source_sha256'].items():
        if sha(root/f) != h: raise ValueError('Frozen recovery/physics source changed: '+f)
    for name, cls in (('design',Design),('unloading_design',UnloadingDesign),('dewedge_design',DewedgeDesign)):
        if plan[name] != asdict(cls()): raise ValueError('Frozen recovery budget changed')
    return True


if __name__ == '__main__':
    root=Path(__file__).resolve().parents[1]; target=root/'experiments/productivity_detector_v2_dev.json'
    if target.exists(): raise FileExistsError(target)
    target.write_text(json.dumps(make_plan(root),indent=2)+'\n')
