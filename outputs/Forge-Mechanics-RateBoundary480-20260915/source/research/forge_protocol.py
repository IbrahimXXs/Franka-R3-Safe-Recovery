"""Screening and recovery definitions for the freely grasped FORGE peg."""
import math
import json
from dataclasses import dataclass, field, replace
from research.phase2_protocol import FAMILIES, Protocol, replay_match, sample_slot

# Actual USD: 9 mm bore with 144-sided straight wall; config's 8.1 mm is stale.
# Use the wall polygon's inscribed radius minus the peg's 3.993 mm outer radius.
RADIAL_CLEARANCE_MM=.5059
MAX_GRASP_SLIP_MM=.1
MAX_GRASP_SLIP_DEG=.5


@dataclass(frozen=True)
class ForgeProtocol(Protocol):
    family_weights: dict = field(default_factory=lambda: dict(zip(FAMILIES,(8,18,18,18,19,19))))
    effective_radial_clearance_mm: float | None = None

    def validate(self):
        super().validate()
        if self.effective_radial_clearance_mm is not None and (
                not math.isfinite(self.effective_radial_clearance_mm)
                or self.effective_radial_clearance_mm <= 0):
            raise ValueError('Effective radial clearance must be positive and finite')
        weights=self.family_weights
        if (not isinstance(weights,dict) or set(weights)!=set(FAMILIES)
                or any(type(v) is not int or v<0 for v in weights.values())
                or sum(weights.values())<=0):
            raise ValueError('family_weights must give a nonnegative integer for each family and a positive total')
        return self


def protocol():
    return replace(ForgeProtocol(),offset_range_mm=(.1,1.),tilt_range_deg=(.25,4.),
        insertion_duration_s=(8.,8.), replay_position_mm=.005,replay_orientation_deg=.02,
        replay_joint_rad=.0001,replay_joint_velocity_rad_s=.001,
        replay_linear_velocity_m_s=.001,replay_angular_velocity_rad_s=.005,
        replay_force_n=.1,replay_torque_nm=.005)


def load_protocol(path):
    values=json.loads(path.read_text())
    for key in ('checkpoints_mm','offset_range_mm','tilt_range_deg','insertion_duration_s'):
        if key in values:values[key]=tuple(values[key])
    p=ForgeProtocol(**values).validate()
    if not 0<=p.seed<2**32:
        raise ValueError('FORGE seed must be in [0, 2**32)')
    if p.replay_position_mm>=RADIAL_CLEARANCE_MM/2:
        raise ValueError('Replay position tolerance must be below half the radial clearance')
    return p


def pilot_cases(p):
    # Signed X/Y, opposite diagonals, both signs of roll/pitch, and combined tilt.
    return [sample_slot(p,i) for i in (0,1,7,2,8,3,15,4,10,16,22,5,65)]


def retained(r):return r['grasp_slip_mm']<=MAX_GRASP_SLIP_MM and r['grasp_slip_deg']<=MAX_GRASP_SLIP_DEG
def radial_clearance(p):
    return RADIAL_CLEARANCE_MM if p.effective_radial_clearance_mm is None else p.effective_radial_clearance_mm


def screened(r,p):return r['min_separation_mm']>=-radial_clearance(p)*p.penetration_fraction
def within_budget(r,p):return r['wrist_force_n']<=p.force_budget_n and r['wrist_torque_nm']<=p.torque_budget_nm
def clear(r):return r['lowest_peg_z_above_mouth_mm']>=2. and r['normal_load_n']<.1 and retained(r)


def match(reference,candidate,p):
    matched,errors=replay_match(reference,candidate,p)
    for key,limit in (('normal_load_n',.1),('min_separation_mm',.005)):
        error=abs(reference[key]-candidate[key]);errors[key]=error;matched &= error<=limit
    error=max(abs(reference[f'wrist_raw_child_{i}']-candidate[f'wrist_raw_child_{i}']) for i in range(3,6))
    errors['wrist_torque_nm']=error;matched &= error<=.005
    for prefix,n,limit in (('finger_position',2,5e-6),('finger_velocity',2,.001),
                            ('grasp_position',3,5e-6),('hand_pose',3,5e-6),('wrist_raw_child',3,.1)):
        error=max(abs(reference[f'{prefix}_{i}']-candidate[f'{prefix}_{i}']) for i in range(n))
        errors[prefix]=error;matched &= error<=limit
    for prefix,start in (('grasp_quat',0),('hand_pose',3)):
        a=[reference[f'{prefix}_{i}'] for i in range(start,start+4)]
        b=[candidate[f'{prefix}_{i}'] for i in range(start,start+4)]
        dot=abs(sum(x*y for x,y in zip(a,b)))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
        angle=math.degrees(2*math.acos(min(1.,dot)))
        errors[prefix+'_angle_deg']=angle;matched &= angle<=.02
    return bool(matched),errors


def recovery_summary(rows,p,dt,matched=True,prefix_valid=True):
    valid=all(screened(r,p) for r in rows);budget=all(within_budget(r,p) for r in rows)
    grasp=all(retained(r) for r in rows)
    consecutive=0;cleared=False;clear_time=None
    for r in rows:
        consecutive=consecutive+1 if clear(r) else 0
        if consecutive>=math.ceil(.2/dt-1e-9):
            cleared=True
            if clear_time is None:clear_time=r['recovery_time_s']
    eligible=bool(matched and prefix_valid and valid)
    return dict(label_eligible=eligible,safe_recovery=bool(budget and grasp and cleared) if eligible else None,
        numerically_valid=valid,grasp_retained=grasp,cleared=cleared,
        force_budget_exceeded=any(r['wrist_force_n']>p.force_budget_n for r in rows),
        torque_budget_exceeded=any(r['wrist_torque_nm']>p.torque_budget_nm for r in rows),
        max_force=max(r['force_norm_n'] for r in rows),max_torque=max(r['torque_norm_nm'] for r in rows),
        max_wrist_force_n=max(r['wrist_force_n'] for r in rows),max_wrist_torque_nm=max(r['wrist_torque_nm'] for r in rows),
        max_normal_load=max(r['normal_load_n'] for r in rows),max_penetration=max(0.,-min(r['min_separation_mm'] for r in rows)),
        recovery_work_j=sum(max(0.,-r['contact_power_w'])*dt for r in rows[1:]),
        clear_time_s=clear_time,duration_s=rows[-1]['recovery_time_s'])
