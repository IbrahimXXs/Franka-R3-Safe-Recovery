"""Observable mechanics-study measurements, independent of Isaac Sim.

Numerical, grasp and safe-recovery labels remain the runner's responsibility.
Contact normal/friction forces are fixture-on-peg in world coordinates; +Z is
the withdrawal direction. Their independently clipped resistance peaks must
not be added, because they need not occur at the same time.
"""
import math

from research.forge_gap_metrics import recovery_phase_metrics


REFERENCE_PHASES = ('approach', 'insert', 'settle', 'tilt', 'hold')
REFERENCE_PHASE_ALIASES = {'depth_settle':'settle', 'tilt_ramp':'tilt', 'tilt_hold':'hold'}
RECOVERY_PHASES = ('stop', 'realign', 'retreat', 'clear_hold')
FACTOR_FIELDS = ('case_id', 'radial_clearance_mm', 'effective_radial_clearance_mm',
                 'hole_depth_mm', 'target_depth_mm', 'pair_static_friction',
                 'pair_dynamic_friction', 'peg_static_friction', 'peg_dynamic_friction',
                 'hole_static_friction', 'hole_dynamic_friction', 'friction_combine_mode',
                 'tilt_amplitude_deg', 'tilt_axis', 'tilt_sign', 'split_group_id')


def _number(row, key):
    value = row.get(key)
    return float(value) if isinstance(value, (int, float)) and math.isfinite(value) else None


def _validate_dt(dt):
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be positive and finite')


def _peak(rows, key):
    values = [_number(row, key) for row in rows]
    return max((v for v in values if v is not None), default=None)


def _mean_peak(rows, key, dt):
    """Fixed-rate complete 100 ms windows, reset by missing samples/time gaps."""
    count = max(1, math.ceil(.1 / dt - 1e-9))
    window = []; peaks = []; previous = None
    for row in rows:
        t = _number(row, 'recovery_time_s')
        if t is None:
            t = _number(row, 'time_s')
        value = _number(row, key)
        if value is None or (t is not None and previous is not None and
                            not math.isclose(t - previous, dt, abs_tol=1e-7, rel_tol=1e-5)):
            window = []
        previous = t
        if value is None or row.get('recovery_time_s') == 0:
            continue
        window.append(value)
        if len(window) > count:
            window.pop(0)
        if len(window) == count:
            peaks.append(sum(window) / count)
    return max(peaks, default=None)


def _contact_rows(rows):
    enriched = []
    for raw in rows:
        row = dict(raw)
        for component in ('normal', 'friction'):
            keys = [f'{component}_force_world_{axis}_n' for axis in 'xyz']
            values = [_number(row, key) for key in keys]
            row[f'contact_{component}_force_norm_n'] = (math.sqrt(sum(v*v for v in values))
                                                       if all(v is not None for v in values) else None)
            z = values[2]
            row[f'contact_{component}_axial_resistance_n'] = max(0., -z) if z is not None else None
        for key, validity in (('wall_centroid_axial_span_mm', 'wall_both_sides_loaded'),
                              ('contact_axial_span_mm', 'contact_axial_span_valid'),
                              ('geometry_overlap_estimate_mm', 'geometry_overlap_estimate_valid')):
            if row.get(validity) is not True:
                row[key] = None
        enriched.append(row)
    return enriched


def _phase_metrics(rows, phase, dt):
    selected = [row for row in rows if row.get('phase') == phase]
    keys = ('wrist_force_n', 'wrist_torque_nm', 'force_norm_n', 'normal_load_n',
            'contact_normal_force_norm_n', 'contact_friction_force_norm_n',
            'contact_normal_axial_resistance_n', 'contact_friction_axial_resistance_n',
            'contact_axial_span_mm', 'wall_centroid_axial_span_mm', 'geometry_overlap_estimate_mm')
    result = {f'{phase}_observed': bool(selected), f'{phase}_sample_count': len(selected)}
    for key in keys:
        result[f'{phase}_peak_{key}'] = _peak(selected, key)
        result[f'{phase}_peak_{key.removesuffix("_n")}_mean100ms_n' if key.endswith('_n')
               else f'{phase}_peak_{key}_mean100ms'] = _mean_peak(selected, key, dt)
    loaded = [row['wall_both_sides_loaded'] for row in selected if type(row.get('wall_both_sides_loaded')) is bool]
    result[f'{phase}_both_walls_loaded_fraction'] = sum(loaded)/len(loaded) if loaded else None
    return result


def mechanics_reference_metrics(rows, case, dt):
    """Measure entry separately from later tilt-window motion at fixed command depth.

    Entry success requires a full dwell within the plan's depth tolerance.
    This is an observable geometric condition, independent of numerical/grasp
    acceptance. Signed terminal depth loss can be negative if the peg advances.
    An aligned control has the same post-entry window, with zero target tilt.
    """
    _validate_dt(dt)
    rows = _contact_rows([dict(row, phase=REFERENCE_PHASE_ALIASES.get(row.get('phase'), row.get('phase')))
                          for row in rows])
    target = float(case['target_depth_mm'])
    tolerance = float(case['settle_tolerance_mm'])
    dwell_s = float(case['settle_dwell_s'])
    aligned = [r for r in rows if r.get('phase') in ('insert', 'settle')]
    reached = False; reached_at = None; previous = None; dwell_start = None
    for row in [r for r in rows if r.get('phase') == 'settle']:
        depth = _number(row, 'depth_mm'); t = _number(row, 'time_s')
        if previous is not None and t is not None and not math.isclose(t-previous, dt, abs_tol=1e-7, rel_tol=1e-5):
            dwell_start = None
        previous = t
        if depth is not None and abs(depth-target) <= tolerance:
            if dwell_start is None:dwell_start = t
        else:dwell_start = None
        if t is not None and dwell_start is not None and t-dwell_start >= dwell_s-1e-9 and not reached:
            reached = True; reached_at = t
    # Serialized MechanicsState is authoritative: its dwell starts only after
    # the scheduled aligned insertion, and may include the boundary sample.
    if any('depth_gate_passed' in r for r in rows):
        reached = any(r.get('depth_gate_passed') is True for r in rows)
        reached_at = next((r.get('depth_gate_time_s') for r in rows if r.get('depth_gate_passed')), None)
    tilt_indices = [i for i, r in enumerate(rows) if r.get('phase') in ('tilt', 'hold')]
    before = rows[max(0, tilt_indices[0]-1)] if tilt_indices else {}
    tilted = [rows[i] for i in tilt_indices]
    terminal = tilted[-1] if tilted else (rows[-1] if rows else {})
    baseline = _number(before, 'depth_mm')
    depths = [v for r in tilted if (v := _number(r, 'depth_mm')) is not None]
    terminal_depth = _number(terminal, 'depth_mm')
    target_angle = abs(float(case['tilt_amplitude_deg']))
    result = dict(reference_sample_count=len(rows), aligned_entry_reached=reached,
                  aligned_entry_gate_time_s=reached_at, target_depth_mm=target,
                  aligned_entry_max_depth_mm=_peak(aligned, 'depth_mm'),
                  tilt_window_observed=bool(tilted), tilt_applicable=target_angle > 0,
                  depth_at_tilt_start_mm=baseline, terminal_depth_mm=terminal_depth,
                  tilt_terminal_depth_loss_mm=baseline-terminal_depth if baseline is not None and tilted and terminal_depth is not None else None,
                  tilt_max_depth_loss_mm=max(0., baseline-min(depths)) if baseline is not None and depths else None,
                  tilt_start_actual_tilt_deg=_number(before, 'tilt_deg'),
                  terminal_actual_tilt_deg=_number(terminal, 'tilt_deg'),
                  terminal_command_tilt_deg=_number(terminal, 'command_tilt_deg'),
                  max_actual_tilt_deg=_peak(tilted, 'tilt_deg'),
                  reference_peak_wrist_force_n=_peak(rows, 'wrist_force_n'),
                  reference_peak_wrist_torque_nm=_peak(rows, 'wrist_torque_nm'),
                  moving_average_support_s=max(1, math.ceil(.1/dt-1e-9))*dt,
                  depth_loss_definition='actual depth before tilt window minus actual depth; no claim of material deformation')
    final = rows[-1] if rows else {}
    for key in ('depth_attained', 'depth_gate_passed', 'tilt_sequence_started', 'tilt_triggered',
                'tilt_ramp_completed', 'reference_done', 'reference_termination_reason',
                'settle_duration_observed_s'):
        result[key] = final.get(key)
    result['reference_protocol_completed'] = final.get('reference_termination_reason') == 'reference_complete'
    for phase in REFERENCE_PHASES:
        result.update(_phase_metrics(rows, phase, dt))
    return result


def mechanics_recovery_metrics(rows, dt):
    """Add contact decomposition/span observations to existing phase recovery costs."""
    _validate_dt(dt)
    if not rows:
        # Existing helper handles missing phases, but does not invent safety.
        return recovery_phase_metrics(rows, dt)
    result = recovery_phase_metrics(rows, dt)
    enriched = _contact_rows(rows)
    for phase in RECOVERY_PHASES:
        result.update(_phase_metrics(enriched, phase, dt))
    candidates = [r for r in enriched if r.get('phase') == 'retreat' and _number(r, 'wrist_force_n') is not None]
    peak = max(candidates, key=lambda row: row['wrist_force_n']) if candidates else {}
    for key in ('recovery_time_s', 'normal_force_world_z_n', 'friction_force_world_z_n',
                'normal_load_n', 'wall_pos_normal_load_n', 'wall_neg_normal_load_n',
                'wall_centroid_axial_span_mm', 'contact_axial_span_mm', 'geometry_overlap_estimate_mm',
                'depth_mm', 'command_depth_mm'):
        result[f'retreat_at_wrist_peak_{key}'] = _number(peak, key)
    result['contact_components_definition'] = 'fixture_on_peg_world_frame; withdrawal resistance is max(0,-world_z) for each component'
    result['span_definition'] = 'normal-load-weighted opposite-wall centroid span only when both walls loaded; all-contact span is a separate masked metric'
    return result
