"""Measured-depth unloading state machine; the existing productivity detector is untouched."""
from dataclasses import dataclass
import math
from research.productivity_control import blend

POLICIES=('nominal','fixed','verified')

@dataclass(frozen=True)
class UnloadingDesign:
    actual_target_mm: float = .5
    max_command_mm: float = 5.
    command_ramp_s: float = 4.
    recovery_timeout_s: float = 5.
    verification_hold_s: float = .25

    def validate(self):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):raise ValueError('Positive finite unloading bounds required')
        if self.max_command_mm<self.actual_target_mm or self.recovery_timeout_s<self.verification_hold_s:
            raise ValueError('Inconsistent unloading bounds')
        return self


class Unloader:
    """A bounded, monotone upward command; only actual peg motion permits a retry.

    Call target(t) once per physics tick, then observe(t, depth, safe).
    Command ramp time pauses during verification; a rebound resumes the ramp
    continuously without resetting the overall deadline or the trigger anchor.
    """
    def __init__(self,trigger_time_s,trigger_depth_mm,stop_s=.25,design=UnloadingDesign()):
        self.design=design.validate();self.t0=trigger_time_s;self.depth0=trigger_depth_mm;self.stop_s=stop_s
        self.phase='stop';self.status='running';self.command_mm=0.;self.ramp_time=0.
        self.last_target_t=trigger_time_s;self.hold_start=None;self.first_crossing_time=None
        self.verified_time=None;self.max_actual_mm=0.;self.rebounds=0

    def target(self,t):
        if self.status!='running':raise ValueError('Recovery already terminated')
        if t<=self.last_target_t:raise ValueError('Recovery clock must increase')
        dt=t-self.last_target_t;self.last_target_t=t
        if t-self.t0<=self.stop_s+1e-9:return 'stop',0.
        if self.phase=='stop':self.phase='unload_retract'
        if self.phase=='unload_retract':
            # Integrate only the portion after the stop, including coarse-step test fixtures.
            self.ramp_time+=min(dt,max(0.,t-self.t0-self.stop_s))
            self.command_mm=self.design.max_command_mm*blend(self.ramp_time/self.design.command_ramp_s)
        return self.phase,self.command_mm

    def observe(self,t,depth_mm,safe=True):
        actual=self.depth0-depth_mm
        self.max_actual_mm=max(self.max_actual_mm,actual)
        if not safe:self.status='safety_stop';return self.status
        elapsed=t-self.t0-self.stop_s
        if elapsed<=1e-9:return self.status
        if actual>=self.design.actual_target_mm:
            if self.first_crossing_time is None:self.first_crossing_time=t
            if self.phase!='unload_hold':self.phase='unload_hold';self.hold_start=t
            if t-self.hold_start>=self.design.verification_hold_s-1e-9 and elapsed<=self.design.recovery_timeout_s+1e-9:
                self.status='verified';self.verified_time=t;return self.status
        elif self.phase=='unload_hold':
            self.phase='unload_retract';self.hold_start=None;self.rebounds+=1
        if elapsed>=self.design.recovery_timeout_s-1e-9:self.status='budget_exhausted'
        return self.status
