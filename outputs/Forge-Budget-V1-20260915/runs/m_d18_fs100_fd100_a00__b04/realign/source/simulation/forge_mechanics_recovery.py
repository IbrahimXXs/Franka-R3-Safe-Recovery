"""Mechanics recovery with a common withdrawal speed and velocity ramp."""
import math

from isaaclab.utils.math import quat_apply, quat_mul, quat_conjugate, quat_slerp
from forge_experiment import Stream, blend, guarded
from research.forge_mechanics_motion import WithdrawalProfile
from research.forge_protocol import screened, within_budget, retained, clear


MOTION = dict(profile='smoothstep_velocity_ramps_with_constant_speed_cruise',
              requested_speed_mm_s=5., ramp_s=.5,
              distance_definition='max(0, reference_terminal_depth_mm + 10)',
              duration_definition='distance/speed + ramp for ordinary insertion depths',
              inherited_retreat_duration_s_used=False)


def recover(bench, start, policy, protocol, path):
    if policy not in ('straight', 'realign'):
        raise ValueError('Unknown mechanics recovery policy')
    e = bench.env
    start_clock = e.last_update_timestamp
    hand_p = e.fingertip_midpoint_pos.clone(); hand_q = e.fingertip_midpoint_quat.clone()
    peg_p = e.held_pos.clone(); peg_q = e.held_quat.clone()
    gp = quat_apply(quat_conjugate(hand_q), peg_p-hand_p)
    gq = quat_mul(quat_conjugate(hand_q), peg_q)
    center = peg_p.clone(); center[:, :2] = e.fixed_pos_obs_frame[:, :2]
    aligned_q = bench.peg_q.clone()
    retreat_start = protocol.stop_duration_s + (protocol.realign_duration_s if policy == 'realign' else 0.)
    profile = WithdrawalProfile(max(0., start['depth_mm']+10.),
                                speed_mm_s=MOTION['requested_speed_mm_s'], ramp_s=MOTION['ramp_s'])
    duration = retreat_start+profile.duration_s+protocol.clear_hold_s
    reason = 'time_budget_exhausted'
    stream = Stream(path)
    stream.add({**start, 'phase': 'stop', 'recovery_time_s': 0.,
                'command_withdrawal_mm': 0., 'command_withdrawal_speed_mm_s': 0.})
    try:
        for step in range(1, math.ceil(duration/bench.dt-1e-9)+1):
            previous = stream.rows[-1]
            if guarded(previous): reason = 'numerical_guard'; break
            if not screened(previous, protocol): reason = 'numerical_screen_failed'; break
            if not within_budget(previous, protocol): reason = 'operational_budget_exceeded'; break
            if not retained(previous): reason = 'grasp_retention_limit'; break
            t = step*bench.dt
            withdrawal = speed = 0.
            if t <= protocol.stop_duration_s:
                hp, hq = hand_p, hand_q
                phase = 'stop'; depth = start['depth_mm']; tilt = start['tilt_deg']
            elif t <= retreat_start:
                fraction = blend((t-protocol.stop_duration_s)/protocol.realign_duration_s)
                pp = peg_p+fraction*(center-peg_p)
                pq = quat_slerp(peg_q[0], aligned_q[0].clone(), fraction).unsqueeze(0)
                hq = quat_mul(pq, quat_conjugate(gq)); hp = pp-quat_apply(hq, gp)
                phase = 'realign'; depth = start['depth_mm']; tilt = (1.-fraction)*start['tilt_deg']
            else:
                elapsed = t-retreat_start
                withdrawal = profile.position_mm(elapsed); speed = profile.speed_mm_s(elapsed)
                pp = (center if policy == 'realign' else peg_p).clone()
                pp[:, 2] += withdrawal/1000.
                pq = aligned_q if policy == 'realign' else peg_q
                hq = quat_mul(pq, quat_conjugate(gq)); hp = pp-quat_apply(hq, gp)
                phase = 'retreat' if elapsed <= profile.duration_s else 'clear_hold'
                depth = start['depth_mm']-withdrawal
                tilt = 0. if policy == 'realign' else start['tilt_deg']
            bench.tick(hp, hq)
            row = bench.observe(phase, depth)
            row.update(command_tilt_deg=tilt, reference_step=start['reference_step'], recovery_time_s=t,
                       command_withdrawal_mm=withdrawal, command_withdrawal_speed_mm_s=speed)
            if abs(e.last_update_timestamp-start_clock-t) > 1e-5:
                raise RuntimeError('Mechanics recovery clock discontinuity')
            stream.add(row)
            if guarded(row): reason = 'numerical_guard'; break
            if not screened(row, protocol): reason = 'numerical_screen_failed'; break
            if not within_budget(row, protocol): reason = 'operational_budget_exceeded'; break
            if not retained(row): reason = 'grasp_retention_limit'; break
            dwell = math.ceil(.2/bench.dt-1e-9)
            if len(stream.rows) >= dwell and all(clear(r) for r in stream.rows[-dwell:]):
                reason = 'cleared'; break
    finally:
        stream.close()
    return stream.rows, reason, profile.as_dict()
