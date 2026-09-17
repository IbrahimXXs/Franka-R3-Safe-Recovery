"""Bounded de-wedging schedule and scalar-first quaternion measurements; no ML."""
from dataclasses import dataclass
import math
import numpy as np
from research.productivity_control import blend
from research.productivity_unloading import UnloadingDesign

POLICIES=('nominal','axial','dewedge')

@dataclass(frozen=True)
class DewedgeDesign:
    relaxation_s: float = .5
    post_verification_hold_s: float = .25

    def validate(self,unloading=UnloadingDesign()):
        if not all(math.isfinite(v) and v>0 for v in vars(self).values()):raise ValueError('Positive finite durations required')
        if self.relaxation_s+unloading.verification_hold_s+self.post_verification_hold_s>=unloading.recovery_timeout_s:
            raise ValueError('Recovery phases do not fit the existing deadline')
        return self


def qmul(a,b):
    a=np.asarray(a,float);b=np.asarray(b,float)
    return np.r_[a[0]*b[0]-a[1:]@b[1:],a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]


def qconj(q):return np.asarray(q,float)*[1,-1,-1,-1]


def rpy(q):
    w,x,y,z=np.asarray(q,float)/np.linalg.norm(q)
    return np.array([math.atan2(2*(w*x+y*z),1-2*(x*x+y*y)),
        math.asin(np.clip(2*(w*y-z*x),-1,1)),math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))])


def neutral_quaternion(actual,nominal):
    """Remove roll/pitch of the world-applied tilt relative to nominal; preserve relative yaw."""
    yaw=rpy(qmul(actual,qconj(nominal)))[2]
    return qmul([math.cos(yaw/2),0,0,math.sin(yaw/2)],nominal)


def pose_angles(actual,nominal):return np.degrees(rpy(qmul(actual,qconj(nominal))))


class Dewedger:
    """Relax at fixed peg depth, retract, verify, then hold; one unchanged deadline.

    A safe 0.25 s verification is recorded separately from readiness to retry
    after the additional hold. Rebound revokes readiness without extending time.
    """
    def __init__(self,trigger_time_s,trigger_depth_mm,stop_s=.25,unloading=UnloadingDesign(),design=DewedgeDesign()):
        self.t0=trigger_time_s;self.depth0=trigger_depth_mm;self.stop_s=stop_s
        self.unloading=unloading.validate();self.design=design.validate(unloading)
        self.status='running';self.phase='stop';self.command_mm=0.;self.fraction=0.
        self.ramp_time=0.;self.last_t=trigger_time_s;self.hold_start=None
        self.first_crossing_time=None;self.verification_time=None;self.ready_time=None
        self.max_actual_mm=0.;self.rebounds=0

    def target(self,t):
        if self.status!='running' or t<=self.last_t:raise ValueError('Terminated recovery or non-increasing clock')
        dt=t-self.last_t;self.last_t=t;elapsed=t-self.t0-self.stop_s
        if elapsed<=1e-9:self.phase='stop';return self.phase,0.,0.
        if elapsed<=self.design.relaxation_s+1e-9:
            self.phase='relax';self.fraction=blend(elapsed/self.design.relaxation_s)
            return self.phase,self.fraction,0.
        self.fraction=1.
        if self.phase in ('stop','relax'):self.phase='unload_retract'
        if self.phase=='unload_retract':
            self.ramp_time+=min(dt,max(0.,elapsed-self.design.relaxation_s))
            self.command_mm=self.unloading.max_command_mm*blend(self.ramp_time/self.unloading.command_ramp_s)
        return self.phase,self.fraction,self.command_mm

    def observe(self,t,depth_mm,safe=True):
        actual=self.depth0-depth_mm;self.max_actual_mm=max(self.max_actual_mm,actual)
        if not safe:self.status='safety_stop';return self.status
        elapsed=t-self.t0-self.stop_s
        if elapsed<=1e-9:return self.status
        if actual>=self.unloading.actual_target_mm and self.first_crossing_time is None:self.first_crossing_time=t
        # Complete the prescribed relaxation before accepting a hold or starting retraction.
        if elapsed>=self.design.relaxation_s-1e-9:
            if actual>=self.unloading.actual_target_mm:
                if self.hold_start is None:self.hold_start=t
                duration=t-self.hold_start
                self.phase='unload_hold'
                if duration>=self.unloading.verification_hold_s-1e-9:
                    if self.verification_time is None:self.verification_time=t
                    self.phase='post_verify_hold'
                if duration>=self.unloading.verification_hold_s+self.design.post_verification_hold_s-1e-9 and elapsed<=self.unloading.recovery_timeout_s+1e-9:
                    self.status='verified';self.ready_time=t;return self.status
            elif self.hold_start is not None:
                self.hold_start=None;self.phase='unload_retract';self.rebounds+=1
        if elapsed>=self.unloading.recovery_timeout_s-1e-9:self.status='budget_exhausted'
        return self.status
