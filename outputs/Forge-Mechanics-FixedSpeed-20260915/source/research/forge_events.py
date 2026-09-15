"""Causal event checkpoints from a completed gap-study reference."""
from dataclasses import asdict
from research.forge_protocol import screened
from research.future_stall import stall_events


EVENT_KINDS = ('pre_tilt', 'ramp_complete', 'first_stall', 'terminal')


def event_checkpoints(rows, protocol, dt, probe_events=EVENT_KINDS):
    if not rows:
        return []
    if set(probe_events) - set(EVENT_KINDS):
        raise ValueError('Unknown recovery event')
    indices = {
        'pre_tilt': next((i for i, r in enumerate(rows) if r.get('tilt_triggered')), None),
        'ramp_complete': next((i for i, r in enumerate(rows)
                               if r.get('tilt_ramp_completed')), None),
        'terminal': len(rows) - 1 if rows[-1]['depth_mm'] > 0 else None,
    }
    stalls = stall_events(rows, asdict(protocol), round(1 / dt))
    indices['first_stall'] = stalls[0]['index'] if stalls else None
    prefix = []
    valid = True
    peaks = dict(prefix_max_force_n=0., prefix_max_torque_nm=0.,
                 prefix_max_normal_load_n=0., prefix_max_penetration_mm=0.)
    for row in rows:
        valid = valid and screened(row, protocol) and row['force_norm_n'] <= 500
        for key, value in (
            ('prefix_max_force_n', row['force_norm_n']),
            ('prefix_max_torque_nm', row['torque_norm_nm']),
            ('prefix_max_normal_load_n', row['normal_load_n']),
            ('prefix_max_penetration_mm', -row['min_separation_mm']),
        ):
            peaks[key] = max(peaks[key], value)
        prefix.append(dict(peaks, prefix_numerically_valid=bool(valid)))
    checkpoints = []
    for kind in EVENT_KINDS:
        index = indices[kind]
        cp = dict(checkpoint_id=kind, checkpoint_kind=kind, reached=index is not None,
                  probe_requested=kind in probe_events, depth_mm=None,
                  prefix_numerically_valid=False, probes=[])
        if index is not None:
            row = rows[index]
            cp.update(prefix[index], state=row.copy(), depth_mm=row['depth_mm'],
                      step=row['reference_step'])
        checkpoints.append(cp)
    return checkpoints


def recovery_termination(reason):
    """Only a completed clearance observes the entire recovery; other peaks are partial."""
    return dict(termination_reason=reason, recovery_censored=reason != 'cleared')
