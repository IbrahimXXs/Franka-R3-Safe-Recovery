"""Observable gap-study costs; independent of Isaac Sim and of recovery labels.

The stock FORGE hole axis is world +Z (the withdrawal direction).  The raw
wrist signal is not a calibrated external-force estimate: gravity, inertia and
the grasp also contribute.  Its absolute world-Z component is deliberately
kept separate from downward peg/socket contact resistance, ``max(0, -fz)``.
These functions never turn an incomplete withdrawal into a successful one.
"""
import math


PHASES = ('stop', 'realign', 'retreat', 'clear_hold')
FACTOR_FIELDS = (
    'case_id', 'radial_clearance_mm', 'effective_radial_clearance_mm', 'hole_diameter_mm', 'peg_diameter_mm',
    'tilt_onset_fraction', 'tilt_onset_mm', 'tilt_amplitude_deg',
    'tilt_sign', 'tilt_axis', 'tilt_ramp_duration_s',
)


def _number(row, key):
    value = row.get(key)
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def _maximum(rows, key, transform=lambda x: x):
    values = [transform(value) for row in rows if (value := _number(row, key)) is not None]
    return max(values) if values else None


def _window_peak(rows, key, dt, transform=lambda x: x):
    """Complete 100 ms windows only, without bridging missing measurements.

    Samples are the simulator's fixed-dt, right-endpoint observations.  A
    window uses ceil(100 ms/dt) intervals, so its support is at least 100 ms.
    The initial recovery observation at time zero is not an elapsed interval.
    """
    count = max(1, math.ceil(.1 / dt - 1e-9))
    values = []
    peaks = []
    previous_time = None
    for row in rows:
        now = _number(row, 'recovery_time_s')
        if now is None:
            now = _number(row, 'time_s')
        value = _number(row, key)
        if value is None or (now is not None and previous_time is not None
                             and not math.isclose(now - previous_time, dt, rel_tol=1e-5, abs_tol=1e-7)):
            values = []
        previous_time = now
        if value is None or row.get('recovery_time_s') == 0:
            continue
        values.append(transform(value))
        if len(values) >= count:
            peaks.append(sum(values[-count:]) / count)
    return max(peaks) if peaks else None


def recovery_phase_metrics(rows, dt):
    """Return flat costs for all recorded phases, including failed probes.

    ``None`` means no measurement/complete window, never zero force.  Peak
    values describe only recorded motion; the runner separately records the
    termination reason and whether the probe was censored before clearance.
    """
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be positive and finite')
    result = {'force_axis_frame': 'world_z_equals_fixed_hole_axis',
              'wrist_axial_signal': 'absolute_raw_wrist_force_world_z_uncalibrated',
              'retreat_contact_resistance_definition': 'max(0,-world_contact_fz)',
              'moving_average_support_s': max(1, math.ceil(.1 / dt - 1e-9)) * dt}
    for phase in PHASES:
        selected = [row for row in rows if row.get('phase') == phase]
        intervals = [row for i, row in enumerate(rows) if i > 0 and row.get('phase') == phase]
        result.update({
            f'{phase}_observed': bool(selected),
            f'{phase}_sample_count': len(selected),
            f'{phase}_duration_s': len(intervals) * dt,
            f'{phase}_peak_wrist_force_n': _maximum(selected, 'wrist_force_n'),
            f'{phase}_peak_wrist_torque_nm': _maximum(selected, 'wrist_torque_nm'),
            f'{phase}_peak_wrist_world_z_abs_n': _maximum(selected, 'wrist_force_world_2', abs),
            f'{phase}_peak_contact_force_n': _maximum(selected, 'force_norm_n'),
            f'{phase}_peak_contact_downward_resistance_n': _maximum(selected, 'fz', lambda v: max(0., -v)),
            f'{phase}_peak_wrist_force_mean100ms_n': _window_peak(selected, 'wrist_force_n', dt),
            f'{phase}_peak_wrist_world_z_abs_mean100ms_n': _window_peak(selected, 'wrist_force_world_2', dt, abs),
            f'{phase}_peak_contact_downward_resistance_mean100ms_n': _window_peak(selected, 'fz', dt, lambda v: max(0., -v)),
            f'{phase}_contact_resistive_work_j': (sum(max(0., -row['contact_power_w']) * dt for row in intervals)
                                                if intervals and all(_number(row, 'contact_power_w') is not None for row in intervals) else None),
        })
    retreat_indices = [i for i, row in enumerate(rows) if row.get('phase') == 'retreat']
    result.update(retreat_start_depth_mm=None, retreat_end_depth_mm=None,
                  retreat_actual_withdrawal_mm=None, retreat_commanded_withdrawal_mm=None,
                  retreat_max_depth_tracking_error_mm=None)
    if retreat_indices:
        first, last = retreat_indices[0], retreat_indices[-1]
        before, end = rows[max(0, first - 1)], rows[last]
        start_depth, end_depth = _number(before, 'depth_mm'), _number(end, 'depth_mm')
        start_command, end_command = _number(before, 'command_depth_mm'), _number(end, 'command_depth_mm')
        errors = [abs(row['depth_mm'] - row['command_depth_mm']) for row in rows[first:last + 1]
                  if _number(row, 'depth_mm') is not None and _number(row, 'command_depth_mm') is not None]
        result.update(retreat_start_depth_mm=start_depth, retreat_end_depth_mm=end_depth,
                      retreat_actual_withdrawal_mm=start_depth - end_depth if start_depth is not None and end_depth is not None else None,
                      retreat_commanded_withdrawal_mm=start_command - end_command if start_command is not None and end_command is not None else None,
                      retreat_max_depth_tracking_error_mm=max(errors) if errors else None)
    return result


def insertion_event_metrics(rows, case):
    """Summarize actual trigger/ramp observations, distinct from plan values."""
    amplitude = abs(float(case.get('tilt_amplitude_deg', 0.)))
    active = [r for r in rows if r.get('phase') in ('insert', 'hold')]
    triggered = [r for r in active if r.get('tilt_triggered') or
                 (amplitude > 0 and (_number(r, 'command_tilt_deg') or 0.) > 1e-9)]
    completed = [r for r in triggered if (r.get('tilt_ramp_fraction') is not None and r['tilt_ramp_fraction'] >= 1 - 1e-9)
                 or (amplitude > 0 and (_number(r, 'command_tilt_deg') or 0.) >= amplitude - 1e-9)]
    result = {'tilt_applicable': amplitude > 0,
              'tilt_triggered': bool(triggered), 'tilt_ramp_completed': bool(completed),
              'max_actual_tilt_deg': _maximum(active, 'tilt_deg'),
              'max_command_tilt_deg': _maximum(active, 'command_tilt_deg'),
              'terminal_actual_tilt_deg': _number(active[-1], 'tilt_deg') if active else None,
              'terminal_command_tilt_deg': _number(active[-1], 'command_tilt_deg') if active else None}
    for event, selected in (('tilt_trigger', triggered), ('tilt_completion', completed)):
        row = selected[0] if selected else {}
        result[f'{event}_time_s'] = row.get('tilt_trigger_time_s', row.get('time_s')) if event == 'tilt_trigger' else row.get('time_s')
        event_time = result[f'{event}_time_s']
        observed = next((r for r in rows if event_time is not None and _number(r, 'time_s') is not None
                         and abs(r['time_s'] - event_time) < 1e-7), row)
        result[f'{event}_actual_depth_mm'] = (row.get('tilt_trigger_depth_mm', observed.get('depth_mm'))
                                             if event == 'tilt_trigger' else observed.get('depth_mm'))
        result[f'{event}_actual_tilt_deg'] = observed.get('tilt_deg')
        result[f'{event}_command_tilt_deg'] = observed.get('command_tilt_deg')
    return result
