"""Force-only decisions bound to the exact frozen de-wedging runner."""
import math
from types import FunctionType
from research.productivity_control import Design, recent_signal
from research.productivity_detector_v2 import NORMAL_ETA


def force_features(rows,hz,onset,design=Design()):
    n=round(design.history_s*hz);w=rows[-n-1:]
    normal=recent_signal(w,hz,onset,design)  # Diagnostics only; never a force decision input.
    eligible=bool(len(w)==n+1 and abs(w[-1]['time_s']-w[0]['time_s']-design.history_s)<1e-6
        and w[-1]['phase'] in ('insert','hold')
        and all(r['phase']==w[-1]['phase'] and r.get('segment',0)==w[-1].get('segment',0)
            and math.isfinite(r['depth_mm']) and r['depth_mm']>onset+design.contact_margin_mm
            and math.isfinite(r['wrist_force_n']) for r in w))
    if not eligible and normal is None:return None
    return dict(eta_raw=normal['eta_raw'] if normal else None,
        actual_progress_mm=normal['actual_progress_mm'] if normal else None,
        command_progress_mm=normal['command_progress_mm'] if normal else None,
        normal_valid=normal is not None,force_eligible=eligible,wrist_force_n=w[-1]['wrist_force_n'])


class ForceDetector:
    """Only current measured force norm enters the decision, with two-check persistence."""
    def __init__(self,threshold,design=Design()):
        if not math.isfinite(threshold) or threshold<=0:raise ValueError('Positive finite threshold required')
        self.threshold=threshold;self.design=design;self.last_decision={};self.reset()
    def reset(self):self.count=0;self.last_time=None
    def check(self,t,signal):
        adjacent=self.last_time is not None and abs(t-self.last_time-self.design.check_s)<1e-6
        eligible=bool(signal and signal['force_eligible'])
        bad=bool(eligible and signal['wrist_force_n']>self.threshold)
        self.count=(self.count+1 if adjacent else 1) if bad else 0;self.last_time=t
        fired=self.count>=self.design.consecutive_checks
        self.last_decision=dict(trigger_branch='force' if fired else '',force_eligible=eligible,
            force_condition=bad,normal_eta_valid=bool(signal and signal['normal_valid']))
        return fired


def bind_force(original,threshold):
    state={};base_stream=original.__globals__['Stream']
    def factory(policy,eta,force,design):
        if policy!='productivity' or eta!=NORMAL_ETA:raise ValueError('Frozen runner contract changed')
        state['detector']=ForceDetector(threshold,design);return state['detector']
    class LoggedStream(base_stream):
        def add(self,row):
            decision=state['detector'].last_decision if row['check_performed'] else {}
            row.update(detector_version='force',trigger_branch=decision.get('trigger_branch',''),
                force_eligible=decision.get('force_eligible',False),force_condition=decision.get('force_condition',False),
                force_threshold_n=threshold)
            row['eta_valid']=decision.get('normal_eta_valid',False)
            super().add(row)
    ns=dict(original.__globals__,Detector=factory,recent_signal=force_features,Stream=LoggedStream)
    bound=FunctionType(original.__code__,ns,original.__name__,original.__defaults__,original.__closure__)
    assert bound.__code__ is original.__code__
    assert {k for k in ns if ns[k] is not original.__globals__.get(k)}=={'Detector','recent_signal','Stream'}
    return bound
