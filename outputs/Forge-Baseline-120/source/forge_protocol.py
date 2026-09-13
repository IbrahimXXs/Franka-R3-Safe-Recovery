"""Screening and recovery definitions for the freely grasped FORGE peg."""
import math
from dataclasses import replace
from research.phase2_protocol import Protocol, replay_match, sample_slot

RADIAL_CLEARANCE_MM=.057
MAX_GRASP_SLIP_MM=.1
MAX_GRASP_SLIP_DEG=.5


def protocol():
    return replace(Protocol(),offset_range_mm=(.01,.12),tilt_range_deg=(.1,.75),
        insertion_duration_s=(8.,8.), replay_position_mm=.005,replay_orientation_deg=.02,
        replay_joint_rad=.0001,replay_joint_velocity_rad_s=.001,
        replay_linear_velocity_m_s=.001,replay_angular_velocity_rad_s=.005,
        replay_force_n=.1,replay_torque_nm=.005)


def pilot_cases(p):
    # Signed X/Y, opposite diagonals, both signs of roll/pitch, and combined tilt.
    return [sample_slot(p,i) for i in (0,1,7,2,8,3,15,4,10,16,22,5,65)]


def retained(r):return r['grasp_slip_mm']<=MAX_GRASP_SLIP_MM and r['grasp_slip_deg']<=MAX_GRASP_SLIP_DEG
def screened(r,p):return r['min_separation_mm']>=-RADIAL_CLEARANCE_MM*p.penetration_fraction
def within_budget(r,p):return r['wrist_force_n']<=p.force_budget_n and r['wrist_torque_nm']<=p.torque_budget_nm
def clear(r):return r['lowest_peg_z_above_mouth_mm']>=2. and r['normal_load_n']<.1 and retained(r)


def match(reference,candidate,p):
    matched,errors=replay_match(reference,candidate,p)
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
