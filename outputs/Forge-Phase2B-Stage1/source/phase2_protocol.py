"""Phase 2 design and labels, independent of Isaac Sim."""
from dataclasses import asdict, dataclass
import math
import random

FAMILIES = ('centered', 'x_offset', 'y_offset', 'diagonal_offset', 'tilt_only', 'offset_tilt')
POLICIES = ('straight', 'realign')


@dataclass(frozen=True)
class Protocol:
    target_valid: int = 100
    seed: int = 20260913
    attempts_per_slot: int = 5
    checkpoints_mm: tuple = (5., 10., 15., 20.)
    offset_range_mm: tuple = (.1, .75)
    tilt_range_deg: tuple = (.25, 3.)
    insertion_duration_s: tuple = (5., 7.)
    penetration_fraction: float = .25
    success_depth_mm: float = 19.5
    success_dwell_s: float = .2
    stall_window_s: float = .5
    stall_progress_mm: float = .1
    stall_command_progress_mm: float = .5
    retreat_duration_s: float = 6.
    realign_duration_s: float = 2.
    stop_duration_s: float = .25
    clear_hold_s: float = .5
    force_budget_n: float = 20.
    torque_budget_nm: float = 1.
    replay_position_mm: float = .1
    replay_orientation_deg: float = .2
    replay_joint_rad: float = .01
    replay_joint_velocity_rad_s: float = .05
    replay_linear_velocity_m_s: float = .005
    replay_angular_velocity_rad_s: float = .05
    replay_force_n: float = 1.
    replay_torque_nm: float = .05

    def validate(self):
        if any(type(getattr(self,k)) is not int for k in ('target_valid','seed','attempts_per_slot')):
            raise ValueError('Target, seed, and attempts per slot must be integers')
        if not 0 < self.penetration_fraction <= 1 or not 0 < self.success_depth_mm <= 20:
            raise ValueError('Invalid penetration fraction or success depth')
        if self.target_valid < 1 or self.attempts_per_slot < 1:
            raise ValueError('Target and attempts per slot must be positive')
        if not self.checkpoints_mm or tuple(sorted(set(self.checkpoints_mm))) != self.checkpoints_mm:
            raise ValueError('Checkpoints must be strictly increasing')
        if not all(0 < x <= 20 for x in self.checkpoints_mm):
            raise ValueError('Checkpoints must lie in (0, 20] mm')
        for name in ('offset_range_mm', 'tilt_range_deg', 'insertion_duration_s'):
            a,b=getattr(self,name)
            if not 0 < a <= b or not all(math.isfinite(x) for x in (a,b)):
                raise ValueError(f'Invalid {name}')
        for k,v in asdict(self).items():
            if isinstance(v,(float,int)) and (not math.isfinite(v) or (k!='seed' and v<=0)):
                raise ValueError(f'{k} must be positive and finite')
        return self


def sample_slot(protocol, slot, retry=0):
    """Round-robin family quotas; retries preserve the slot's directional stratum."""
    rng=random.Random(f'{protocol.seed}:{slot}:{retry}')
    family=FAMILIES[slot%len(FAMILIES)];ordinal=slot//len(FAMILIES)
    signs=((1,1),(-1,1),(-1,-1),(1,-1));sx,sy=signs[ordinal%4]
    r=rng.uniform(*protocol.offset_range_mm);a=rng.uniform(*protocol.tilt_range_deg)
    dx=dy=roll=pitch=0.
    if family=='x_offset': dx=r*(1 if ordinal%2==0 else -1)
    if family=='y_offset': dy=r*(1 if ordinal%2==0 else -1)
    if family in ('diagonal_offset','offset_tilt'): dx,dy=sx*r/math.sqrt(2),sy*r/math.sqrt(2)
    if family in ('tilt_only','offset_tilt'):
        # Both axes and both signs. In combined cases cycle tilt independently
        # across blocks of four offset quadrants (16 directional combinations).
        direction=(ordinal if family=='tilt_only' else ordinal//4)%4
        roll,pitch=((a,0.),(0.,a),(-a,0.),(0.,-a))[direction]
    return dict(trajectory_id=f'slot{slot:03d}_try{retry:02d}', slot=slot, retry=retry, family=family,
                offset_x_mm=dx, offset_y_mm=dy, roll_deg=roll, pitch_deg=pitch,
                insertion_duration_s=rng.uniform(*protocol.insertion_duration_s))


def insertion_metrics(rows, dt, protocol, clearance_mm, aborted=False):
    if not rows: raise ValueError('Cannot characterize an empty trajectory')
    maximum=lambda key:max(r[key] for r in rows)
    penetration=max(0.,-min(r['min_separation_mm'] for r in rows))
    dwell=0;success=False;stalled=False
    window=max(1,round(protocol.stall_window_s/dt))
    for i,r in enumerate(rows):
        reached=r['depth_mm']>=protocol.success_depth_mm
        dwell=dwell+1 if reached else 0
        success |= dwell>=round(protocol.success_dwell_s/dt)
        if i>=window and r['phase']=='insert' and rows[i-window]['phase']=='insert':
            past=rows[i-window]
            stalled |= (r['command_depth_mm']-past['command_depth_mm']>=protocol.stall_command_progress_mm
                        and r['depth_mm']-past['depth_mm']<protocol.stall_progress_mm)
    valid=not aborted and penetration<=clearance_mm*protocol.penetration_fraction
    return dict(insertion_success=bool(success), max_force=maximum('force_norm_n'),
                max_torque=maximum('torque_norm_nm'), max_normal_load=maximum('normal_load_n'),
                max_penetration=penetration, max_depth=maximum('depth_mm'), stalled=bool(stalled),
                force_budget_exceeded=maximum('force_norm_n')>protocol.force_budget_n,
                torque_budget_exceeded=maximum('torque_norm_nm')>protocol.torque_budget_nm,
                numerically_valid=valid, outcome_trustworthy=valid)


def replay_match(reference, candidate, protocol):
    """Observable-state tolerance check, not a claim to restore solver/contact memory."""
    norm=lambda keys:math.sqrt(sum((reference[k]-candidate[k])**2 for k in keys))
    dot=abs(sum(reference[k]*candidate[k] for k in ('qw','qx','qy','qz')))
    # Normalize quaternions to avoid false angular error from float roundoff.
    qnorm=math.sqrt(sum(reference[k]**2 for k in ('qw','qx','qy','qz'))*sum(candidate[k]**2 for k in ('qw','qx','qy','qz')))
    orientation=math.degrees(2*math.acos(min(1.,dot/qnorm)))
    errors=dict(position_mm=norm(('tip_x_mm','tip_y_mm','depth_mm')), orientation_deg=orientation,
                joint_rad=max(abs(reference[f'joint{i}_rad']-candidate[f'joint{i}_rad']) for i in range(1,8)),
                joint_velocity_rad_s=max(abs(reference[f'joint_velocity{i}_rad_s']-candidate[f'joint_velocity{i}_rad_s']) for i in range(1,8)),
                linear_velocity_m_s=norm(('vx','vy','vz')), angular_velocity_rad_s=norm(('omegax','omegay','omegaz')),
                force_n=norm(('fx','fy','fz')), torque_nm=norm(('taux','tauy','tauz')))
    matched=all(v<=getattr(protocol,'replay_'+k) for k,v in errors.items())
    return matched,errors


def recovery_label(probes, reached=True, prefix_valid=True):
    """Positive witness; zero only for all tested policies failing valid matched tests."""
    if not reached: return None,'checkpoint_not_reached'
    if not prefix_valid: return None,'numerically_invalid_prefix'
    valid=[p for p in probes if p.get('label_eligible')]
    if any(p.get('safe_recovery') for p in valid): return 1,'safe_policy_witness'
    if {p['policy'] for p in valid}==set(POLICIES): return 0,'tested_policies_failed'
    return None,'unresolved_probe_or_replay'


def main():
    import argparse
    import csv
    import json
    from pathlib import Path
    parser=argparse.ArgumentParser(description='Preview the Phase 2 sampling plan without starting Isaac Sim.')
    parser.add_argument('--config',type=Path,default=Path('experiments/phase2.json'))
    parser.add_argument('--output',type=Path,default=Path('experiments/phase2_plan.csv'))
    args=parser.parse_args()
    values=json.loads(args.config.read_text())
    for key in ('checkpoints_mm','offset_range_mm','tilt_range_deg','insertion_duration_s'):
        if key in values:values[key]=tuple(values[key])
    protocol=Protocol(**values).validate()
    rows=[sample_slot(protocol,i) for i in range(protocol.target_valid)]
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    print(f'{len(rows)} initial candidates; at most {len(rows)*protocol.attempts_per_slot} reference attempts.')
    print('Family quotas:',{f:sum(r['family']==f for r in rows) for f in FAMILIES})
    print(f'Saved sampling plan: {args.output}')


if __name__=='__main__':
    main()
