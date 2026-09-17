"""Causal, non-ML productivity intervention helpers and offline calibration."""
from collections import Counter, defaultdict
from dataclasses import dataclass, asdict
from pathlib import Path
import argparse
import json
import math
import hashlib
import numpy as np
from research.contact_productivity import STUDIES, group_for, role_for, alarm_threshold, weights
from research.future_stall import read_rows

POLICIES = ('nominal', 'force', 'productivity')

@dataclass(frozen=True)
class Design:
    history_s: float = .5
    check_s: float = .1
    minimum_command_mm: float = .1
    contact_margin_mm: float = .1
    consecutive_checks: int = 2
    stop_s: float = .25
    retract_mm: float = 1.
    retract_s: float = .5
    rejoin_s: float = .5
    max_interventions: int = 2
    insertion_budget_s: float = 20.
    nonstall_alert_budget: float = .1


def blend(u):
    u = max(0., min(1., u))
    return u*u*(3.-2.*u)


def clock_for_depth(depth, duration=8.):
    """Invert the original smoothstep path; retry resumes from actual retracted depth."""
    target = np.clip((depth+10.)/30., 0., 1.)
    lo, hi = 0., 1.
    for _ in range(50):
        mid = (lo+hi)/2
        if blend(mid) < target: lo = mid
        else: hi = mid
    return duration*(lo+hi)/2


def recent_signal(rows, hz, onset, design=Design()):
    """Causal endpoint ratio; intentional pauses/retractions invalidate the history."""
    n = round(design.history_s*hz)
    if len(rows) <= n: return None
    window = rows[-n-1:]; a, b = window[0], window[-1]
    if any(r['phase'] != 'insert' or r.get('segment', 0) != b.get('segment', 0) for r in window): return None
    if abs(b['time_s']-a['time_s']-design.history_s) > 1e-6: return None
    if min(r['depth_mm'] for r in window) <= onset+design.contact_margin_mm: return None
    dc = b['command_depth_mm']-a['command_depth_mm']
    da = b['depth_mm']-a['depth_mm']
    if not all(math.isfinite(v) for v in (dc,da,b['wrist_force_n'])) or dc < design.minimum_command_mm-1e-9: return None
    return dict(eta_raw=da/dc, actual_progress_mm=da, command_progress_mm=dc,
                wrist_force_n=b['wrist_force_n'])


class Detector:
    def __init__(self, policy, eta_threshold, force_threshold, design=Design()):
        if policy not in POLICIES: raise ValueError('Unknown policy')
        self.policy, self.eta_threshold, self.force_threshold, self.design = policy, eta_threshold, force_threshold, design
        self.count = 0; self.last_time = None

    def reset(self):
        self.count = 0; self.last_time = None

    def check(self, t, signal):
        adjacent = self.last_time is not None and abs(t-self.last_time-self.design.check_s) < 1e-6
        bad = signal is not None and ((self.policy == 'productivity' and signal['eta_raw'] < self.eta_threshold)
                                     or (self.policy == 'force' and signal['wrist_force_n'] >= self.force_threshold))
        self.count = (self.count+1 if adjacent else 1) if bad else 0
        self.last_time = t
        return self.count >= self.design.consecutive_checks


def stalled_now(rows, hz, p):
    n = round(p.stall_window_s*hz)
    if len(rows) <= n: return False
    window = rows[-n-1:]; a,b = window[0],window[-1]
    return (all(r['phase']=='insert' and r.get('segment',0)==b.get('segment',0) for r in window)
            and b['command_depth_mm']-a['command_depth_mm'] >= p.stall_command_progress_mm
            and b['depth_mm']-a['depth_mm'] < p.stall_progress_mm)


def calibration(root, design=Design()):
    """Freeze two thresholds without using Stage4 or any of its shared path groups."""
    manifests = {s:json.loads((root/s/'study.json').read_text()) for s in STUDIES}
    stress = {group_for(STUDIES[-1],a) for a in manifests[STUDIES[-1]]['attempts']}
    windows=[]; trajectories=[]; hashes={}; excluded=Counter()
    for study,m in manifests.items():
        manifest_path=root/study/'study.json';hashes[str(manifest_path)]=hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        for a in m['attempts']:
            group=group_for(study,a);role=role_for(study,group,stress)
            if role!='development': excluded[role]+=1;continue
            if a['status']!='complete' or not a['metrics']['numerically_valid']:excluded['invalid']+=1;continue
            path=root/study/a['folder']/'insertion.csv';rows=read_rows(path)
            hashes[str(path)]=hashlib.sha256(path.read_bytes()).hexdigest()
            base=dict(trajectory_id=study+'/'+a['folder'],group_id=group,study=study,stalled=a['metrics']['stalled'])
            trajectories.append(base)
            for i in range(0,len(rows),round(design.check_s*m['physics_hz'])):
                signal=recent_signal(rows[:i+1],m['physics_hz'],a.get('ramp_onset_mm',0.),design)
                if signal is not None: windows.append(dict(**base,time_s=rows[i]['time_s'],**signal))
    # Low eta maps to high alarm score; equality is deliberately excluded for eta.
    force=alarm_threshold(windows,np.array([w['wrist_force_n'] for w in windows]),design.nonstall_alert_budget)
    eta=-alarm_threshold(windows,-np.array([w['eta_raw'] for w in windows]),design.nonstall_alert_budget)
    if not math.isfinite(force) or not math.isfinite(eta): raise ValueError('Uncalibratable thresholds')
    by=defaultdict(list)
    for w in windows:by[w['trajectory_id']].append(w)
    diagnostics=[]
    for policy in ('force','productivity'):
        records=[]
        for tid,seq in by.items():
            d=Detector(policy,eta,force,design);trigger=next((w['time_s'] for w in seq if d.check(w['time_s'],w)),None)
            records.append(dict(trajectory_id=tid,group_id=seq[0]['group_id'],policy=policy,stalled=seq[0]['stalled'],
                                alerted=trigger is not None,first_alarm_s=trigger))
        nonstall=[r for r in records if not r['stalled']]
        rate=float(weights(np.array([r['group_id'] for r in nonstall]))@np.array([r['alerted'] for r in nonstall]))
        diagnostics.append(dict(policy=policy,group_weighted_nonstall_alert_rate=rate,
            stalled_alerts=sum(r['alerted'] for r in records if r['stalled']),stalled_total=sum(r['stalled'] for r in records)))
    return dict(schema='Contact-Productivity-Control-Calibration-v1',design=asdict(design),
        eta_threshold=eta,force_threshold_n=force,development_trajectories=len(trajectories),
        development_groups=len({t['group_id'] for t in trajectories}),development_windows=len(windows),
        development_by_study=dict(Counter(t['study'] for t in trajectories)),excluded=dict(excluded),
        stress_groups=sorted(stress),development=trajectories,diagnostics=diagnostics,source_sha256=hashes,
        method='No ML. Two consecutive checks; maximum alert sensitivity within 10% group-weighted non-stalled trajectory alert budget; entire nominal insertion histories, including post-stall. Stage4 and shared path groups excluded. Raw unclipped eta.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=Path('outputs'))
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():parser.error('Calibration file already exists')
    result=calibration(args.root.resolve())
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('eta_threshold','force_threshold_n','development_trajectories','development_groups','diagnostics')},indent=2))

if __name__=='__main__':main()
