"""Pure evidence checks for the depth-controlled mechanics study."""
import json
import math
from research.forge_protocol import match, retained, screened, within_budget


def reference_quality(rows, protocol, reason):
    if not rows:
        raise ValueError('Reference must contain observations')
    valid = all(screened(r, protocol) for r in rows) and reason != 'numerical_guard'
    return dict(
        numerically_valid=bool(valid),
        grasp_retained=all(retained(r) for r in rows),
        force_budget_exceeded=any(r['wrist_force_n'] > protocol.force_budget_n for r in rows),
        torque_budget_exceeded=any(r['wrist_torque_nm'] > protocol.torque_budget_nm for r in rows),
        reference_within_budget=all(within_budget(r, protocol) for r in rows),
        reference_complete=reason == 'completed',
        termination_reason=reason,
        max_penetration_mm=max(0., -min(r['min_separation_mm'] for r in rows)),
        max_grasp_slip_mm=max(r['grasp_slip_mm'] for r in rows),
        max_grasp_slip_deg=max(r['grasp_slip_deg'] for r in rows),
        max_wrist_force_n=max(r['wrist_force_n'] for r in rows),
        max_wrist_torque_nm=max(r['wrist_torque_nm'] for r in rows),
        max_normal_load_n=max(r['normal_load_n'] for r in rows),
    )


def compare_prefixes(reference, replay, protocol):
    """Compare every observable state, not just the branch endpoint.

    Equal recorded values are an additional diagnostic. Physical tolerance
    matching does not certify unobserved contact/solver memory restoration.
    """
    if not reference or not replay:
        raise ValueError('Both prefixes need observations')
    maximum = {}; first = None; equal = len(reference) == len(replay)
    matched = equal
    commands = ('command_depth_mm', 'command_pitch_deg', 'command_offset_x_mm',
                'command_offset_y_mm', 'reference_step')
    for i, (a, b) in enumerate(zip(reference, replay)):
        okay, errors = match(a, b, protocol)
        okay = okay and a['phase'] == b['phase']
        okay = okay and all(a.get(k) == b.get(k) for k in commands)
        okay = okay and math.isclose(a['time_s'], b['time_s'], abs_tol=1e-5, rel_tol=0.)
        for key, value in errors.items():
            maximum[key] = max(maximum.get(key, 0.), value)
        if not okay and first is None:
            first = i
        matched = matched and okay
        equal = equal and all(a.get(k) == b.get(k) for k in set(a) | set(b)
                              if k not in ('time_s', 'physics_time_s'))
    if len(reference) != len(replay) and first is None:
        first = min(len(reference), len(replay))
    endpoint, errors = match(reference[-1], replay[-1], protocol)
    return dict(replay_matched=endpoint, replay_errors=errors,
                replay_prefix_matched=bool(matched), replay_prefix_equal=bool(equal),
                replay_prefix_first_mismatch_index=first,
                replay_prefix_max_errors=maximum,
                reference_sample_count=len(reference), replay_sample_count=len(replay))


def aligned_control_passed(attempt):
    m = attempt.get('metrics', {})
    return (attempt.get('status') == 'complete' and m.get('reference_complete') is True
            and m.get('numerically_valid') is True and m.get('grasp_retained') is True
            and m.get('reference_within_budget') is True
            and m.get('depth_attained') is True
            and attempt.get('final_retreat', {}).get('safe_recovery') is True)


def control_key(case):
    return tuple(case[k] for k in ('target_depth_mm', 'pair_static_friction', 'pair_dynamic_friction'))


def validate_resume(saved, requested):
    # JSON manifests round-trip dataclass tuple fields as lists. Compare the
    # serialized representation, while still rejecting any changed values.
    canonical = lambda value: json.loads(json.dumps(value, allow_nan=False))
    for key in ('study', 'physics_hz', 'seed', 'case_plan', 'protocol', 'sources',
                'radial_clearance_mm', 'recovery_policy_definition', 'simulation_options'):
        if canonical(saved.get(key)) != canonical(requested.get(key)):
            raise ValueError(f'Cannot resume: frozen {key} differs')
