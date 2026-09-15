"""Canonical low-budget conditions and read-only evidence classification.

The budget monitors recorded task samples, starting at reference t=0. Scene
reset, grasp preparation and its three-second stabilization are outside this
scope. Existing pose commands, contact physics and grasp limits are unchanged.
The inherited boundary is inclusive: equal to the budget is allowed, and only
a strictly greater force or torque terminates the recorded task.
"""
import json
import math
from collections.abc import Mapping
from pathlib import Path

from research.forge_mechanics_plan import make_case


SCHEMA = 'Forge-budget-v1'


def make_plan():
    """Eight ordered conditions, each with two independently prepared policies."""
    specifications = (
        (18., (1., 1.), 0., 4., 'aligned_control'),
        (18., (1., 1.), 1., 4., 'angle_control'),
        (18., (1., 1.), 1.5, 4., 'candidate'),
        (18., (1., 1.), 1.5, 5., 'higher_budget'),
        (12., (1., 1.), 1.5, 4., 'shallower_depth'),
        (18., (.5, .5), 1.5, 4., 'lower_friction'),
        (18., (1., 1.), 2., 4., 'grasp_risk'),
        (18., (1., 1.), 1.5, 3., 'lower_budget'),
    )
    conditions = []
    for depth, friction, angle, budget, role in specifications:
        case = make_case(depth, friction, angle)
        conditions.append(dict(condition_id=f'{case["case_id"]}__b{int(budget):02d}',
                               case=case, force_budget_n=budget, torque_budget_nm=1.,
                               physics_hz=240, seed=20260913, role=role))
    return dict(schema=SCHEMA, conditions=conditions, policies=['straight', 'realign'],
                budget_definition=dict(
                    scope='recorded_reference_and_entire_recovery',
                    activation='reference_initial_sample_t0',
                    covered_segments=['reference', 'stop', 'realign', 'retreat', 'clear_hold'],
                    excluded_preparation='scene reset, grasp preparation and three-second stabilization',
                    force_signal='raw_wrist_force_norm_n', torque_signal='raw_wrist_torque_norm_nm',
                    allowed_comparison='<=', termination_comparison='>',
                    equality_allowed=True,
                    monitoring='check initial sample; save each completed physics-step sample before checking; no next task step after crossing',
                    overshoot='sampled monitoring may observe overshoot; forces are not clipped',
                    initial_sample='observed state, not a newly executed task step',
                    policy_comparison='independent process per policy; full-reference-prefix matching required for paired conclusions',
                    preparatory_physics_changed=False))


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f'Duplicate plan key: {key}')
        result[key] = value
    return result


def load_plan(path):
    """Load the exact canonical matrix, including types, order and provenance."""
    value = json.loads(Path(path).read_text(), object_pairs_hook=_unique_object)
    try:
        serialize = lambda x: json.dumps(x, sort_keys=True, allow_nan=False, separators=(',', ':'))
        equal = serialize(value) == serialize(make_plan())
    except (TypeError, ValueError):
        equal = False
    if not equal:
        raise ValueError('Budget plan differs from the exact Forge-budget-v1 canonical plan')
    return value


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def _number(row, key):
    value = row.get(key)
    return float(value) if _finite(value) else None


def _protocol_value(protocol, key):
    value = protocol.get(key) if isinstance(protocol, Mapping) else getattr(protocol, key, None)
    if not _finite(value) or value <= 0:
        raise ValueError(f'{key} must be positive and finite')
    return float(value)


def _peak(rows, key):
    return max((value for row in rows if (value := _number(row, key)) is not None), default=None)


def _clock(row, segment):
    value = _number(row, 'recovery_time_s') if segment == 'recovery' else None
    return value if value is not None else _number(row, 'time_s')


def _mean100ms(rows, indices, key, dt, segment):
    """Only complete contiguous same-phase windows of executed samples count."""
    count = max(1, math.ceil(.1 / dt - 1e-9))
    window = []; previous_index = None; previous_time = None; peaks = []
    for index in indices:
        if index == 0:
            continue
        row = rows[index]; value = _number(row, key); time = _clock(row, segment)
        contiguous = (previous_index is not None and index == previous_index + 1
                      and time is not None and previous_time is not None
                      and math.isclose(time - previous_time, dt, abs_tol=1e-7, rel_tol=1e-5))
        if not contiguous:
            window = []
        previous_index, previous_time = index, time
        if value is None or time is None:
            window = []
            continue
        window.append(value)
        if len(window) > count:
            window.pop(0)
        if len(window) == count:
            peaks.append(sum(window) / count)
    return max(peaks, default=None), len(peaks)


def evidence(rows, protocol, dt, segment='reference'):
    """Summarize sampled budget evidence without inventing completion or safety.

    The first supplied row must be the segment's initial observation. It counts
    toward budget checks and observed peaks but never toward executed-step or
    100 ms window counts. Missing/non-finite budget signals remain explicit;
    they must not make a trajectory eligible for a physical safety label.
    """
    if segment not in ('reference', 'recovery'):
        raise ValueError('segment must be reference or recovery')
    if not _finite(dt) or dt <= 0:
        raise ValueError('dt must be positive and finite')
    rows = list(rows)
    force_budget = _protocol_value(protocol, 'force_budget_n')
    torque_budget = _protocol_value(protocol, 'torque_budget_nm')
    invalid = []; first = None; force_exceeded = torque_exceeded = False
    for index, row in enumerate(rows):
        force, torque = _number(row, 'wrist_force_n'), _number(row, 'wrist_torque_nm')
        if force is None or torque is None or force < 0 or torque < 0:
            invalid.append(index)
        force_hit = force is not None and force > force_budget
        torque_hit = torque is not None and torque > torque_budget
        force_exceeded |= force_hit; torque_exceeded |= torque_hit
        if first is None and (force_hit or torque_hit):
            first = dict(index=index, executed_step=index > 0,
                         time_s=_number(row, 'time_s'), physics_time_s=_number(row, 'physics_time_s'),
                         recovery_time_s=_number(row, 'recovery_time_s'), phase=row.get('phase'),
                         wrist_force_n=force, wrist_torque_nm=torque,
                         depth_mm=_number(row, 'depth_mm'), grasp_slip_mm=_number(row, 'grasp_slip_mm'),
                         grasp_slip_deg=_number(row, 'grasp_slip_deg'),
                         force_budget_exceeded=force_hit, torque_budget_exceeded=torque_hit,
                         force_overshoot_n=max(0., force - force_budget) if force is not None else None,
                         torque_overshoot_nm=max(0., torque - torque_budget) if torque is not None else None,
                         initial_state_already_over_budget=index == 0,
                         budget_signals_finite=index not in invalid)
    force_peak, torque_peak = _peak(rows, 'wrist_force_n'), _peak(rows, 'wrist_torque_nm')
    phases = {}
    for phase in dict.fromkeys(row.get('phase', 'unknown') for row in rows):
        indices = [i for i, row in enumerate(rows) if row.get('phase', 'unknown') == phase]
        selected = [rows[i] for i in indices]
        force_mean, force_windows = _mean100ms(rows, indices, 'wrist_force_n', dt, segment)
        torque_mean, torque_windows = _mean100ms(rows, indices, 'wrist_torque_nm', dt, segment)
        executed = sum(i > 0 for i in indices)
        phases[phase] = dict(sample_count=len(indices), executed_step_count=executed,
                             executed_duration_s=executed * dt, phase_executed=executed > 0,
                             max_wrist_force_n=_peak(selected, 'wrist_force_n'),
                             max_wrist_torque_nm=_peak(selected, 'wrist_torque_nm'),
                             peak_wrist_force_mean100ms_n=force_mean,
                             peak_wrist_torque_mean100ms_nm=torque_mean,
                             complete_force_mean100ms_windows=force_windows,
                             complete_torque_mean100ms_windows=torque_windows)
    return dict(segment=segment, sample_count=len(rows), executed_step_count=max(0, len(rows) - 1),
                executed_duration_s=max(0, len(rows) - 1) * dt, initial_sample_executed=False,
                force_budget_n=force_budget, torque_budget_nm=torque_budget,
                termination_comparison='>', equality_allowed=True,
                force_budget_exceeded=force_exceeded, torque_budget_exceeded=torque_exceeded,
                budget_exceeded=force_exceeded or torque_exceeded,
                first_budget_crossing=first, max_wrist_force_n=force_peak,
                max_wrist_torque_nm=torque_peak,
                force_margin_n=force_budget - force_peak if force_peak is not None else None,
                torque_margin_nm=torque_budget - torque_peak if torque_peak is not None else None,
                phase_metrics=phases, budget_signals_finite=bool(rows) and not invalid,
                budget_signals_invalid_indices=invalid,
                mean100ms_support_s=max(1, math.ceil(.1 / dt - 1e-9)) * dt,
                observed_peaks_include_initial_sample=True)


def reference_eligible(metrics):
    """Only a complete valid depth-gated reference can enter policy recovery."""
    required = ('reference_complete', 'numerically_valid', 'grasp_retained',
                'reference_within_budget', 'depth_gate_passed')
    return (isinstance(metrics, Mapping) and all(metrics.get(key) is True for key in required)
            and metrics.get('budget_signals_finite') is not False
            and not any(metrics.get(key) is True for key in
                        ('force_budget_exceeded', 'torque_budget_exceeded', 'budget_exceeded')))


def _budget_exceeded(metrics, reference=False):
    return (any(metrics.get(key) is True for key in
                ('force_budget_exceeded', 'torque_budget_exceeded', 'budget_exceeded'))
            or reference and metrics.get('reference_within_budget') is False)


def classify(record):
    """Classify one process's continuous reference/recovery, not a policy pair.

    An independently executed realign branch needs a separate full-prefix
    comparison before paired conclusions. A timeout or incomplete recording
    without an observed constraint violation remains unknown. Numerical
    rejection takes precedence over a simultaneous budget/grasp event, while
    all independently observed flags remain available in the returned record.
    """
    reference = (record.get('reference') or {}).get('metrics') or {}
    recovery_record = record.get('recovery')
    recovery = (recovery_record or {}).get('metrics') or {}
    flags = {}
    for prefix, metrics in (('reference', reference), ('recovery', recovery)):
        for key in ('numerically_valid', 'budget_signals_finite', 'grasp_retained',
                    'force_budget_exceeded', 'torque_budget_exceeded'):
            flags[f'{prefix}_{key}'] = metrics.get(key)
        flags[f'{prefix}_budget_exceeded'] = _budget_exceeded(metrics, prefix == 'reference')
    eligible = reference_eligible(reference)

    def result(outcome, recovery_safe=None, task_safe=None):
        return dict(outcome=outcome, recovery_safe=recovery_safe, task_safe=task_safe,
                    reference_eligible=eligible, recovery_present=recovery_record is not None,
                    flags=flags)

    if reference.get('numerically_valid') is not True or reference.get('budget_signals_finite') is False:
        return result('numerical_unknown')
    if _budget_exceeded(reference, True):
        return result('reference_budget_exceeded', task_safe=False)
    if reference.get('grasp_retained') is False:
        return result('reference_grasp_limit', task_safe=False)
    if not eligible or not recovery:
        return result('timeout_unknown')
    if recovery.get('numerically_valid') is not True or recovery.get('budget_signals_finite') is False:
        return result('numerical_unknown')
    if _budget_exceeded(recovery):
        return result('recovery_budget_exceeded', False, False)
    if recovery.get('grasp_retained') is False:
        return result('recovery_grasp_limit', False, False)
    reason = recovery.get('reason', recovery.get('termination_reason'))
    if (recovery.get('safe_recovery') is True and recovery.get('label_eligible') is True
            and recovery.get('cleared') is True and recovery.get('grasp_retained') is True
            and recovery.get('recovery_censored') is False and reason == 'cleared'):
        return result('clear_safe', True, True)
    return result('timeout_unknown')
