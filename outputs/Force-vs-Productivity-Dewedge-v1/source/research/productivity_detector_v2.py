"""Causal detector-only extension. Recovery and hard safety are owned by the frozen runner."""
from dataclasses import asdict
import math
from types import FunctionType
from research.productivity_control import Design, Detector, recent_signal

NORMAL_ETA = 0.4212659765112803
FEATURE_KEYS = ('time_s', 'phase', 'segment', 'depth_mm', 'command_depth_mm', 'wrist_force_n')


def features(rows, hz, onset, design=Design(), success_depth=19.5):
    """Only past/present command and measured motion; normal load is never read."""
    n = round(design.history_s * hz)
    window = [{k: r.get(k, 0) for k in FEATURE_KEYS} for r in rows[-n-1:]]
    normal = recent_signal(window, hz, onset, design)
    terminal_rate = None
    if len(window) == n+1:
        a, b = window[0], window[-1]
        valid = (abs(b['time_s']-a['time_s']-design.history_s) < 1e-6
                 and all(r['phase'] == 'hold' and r['segment'] == b['segment']
                         and math.isfinite(r['depth_mm']) and onset+design.contact_margin_mm < r['depth_mm'] < success_depth
                         and math.isfinite(r['command_depth_mm']) and abs(r['command_depth_mm']-20.) < 1e-6
                         for r in window))
        if valid:
            # A small net displacement alone could hide oscillation; require a small range.
            terminal_rate = (max(r['depth_mm'] for r in window)-min(r['depth_mm'] for r in window)) / design.history_s
    if normal is None and terminal_rate is None:
        return None
    return dict(eta_raw=normal['eta_raw'] if normal else None,
                actual_progress_mm=normal['actual_progress_mm'] if normal else window[-1]['depth_mm']-window[0]['depth_mm'],
                command_progress_mm=normal['command_progress_mm'] if normal else 0.,
                wrist_force_n=window[-1]['wrist_force_n'], normal_valid=normal is not None,
                deficit_rate_mm_s=(normal['command_progress_mm']-normal['actual_progress_mm'])/design.history_s if normal else None,
                terminal_rate_mm_s=terminal_rate)


class DetectorV2:
    def __init__(self, config, design=Design()):
        if config['normal_eta_threshold'] != NORMAL_ETA or config['normal_consecutive_checks'] != 2:
            raise ValueError('The original normal trigger is frozen')
        for key in ('urgent_eta_threshold', 'urgent_deficit_acceleration_mm_s2', 'terminal_rate_mm_s'):
            if not math.isfinite(config[key]) or config[key] <= 0:
                raise ValueError('Finite positive detector thresholds required')
        if config['urgent_eta_threshold'] > .2:
            raise ValueError('Urgent eta must remain conservatively below the normal threshold')
        self.config = config; self.design = design
        self.normal = Detector('productivity', NORMAL_ETA, 0., design)
        self.last_decision = {}; self.reset()

    @property
    def count(self):
        return self.normal.count

    def reset(self):
        self.normal.reset(); self.previous_time = None; self.previous_deficit = None

    def check(self, t, signal):
        normal = signal if signal and signal['normal_valid'] else None
        normal_fired = self.normal.check(t, normal)
        deficit = normal['deficit_rate_mm_s'] if normal else None
        acceleration = None
        if deficit is not None and self.previous_deficit is not None and abs(t-self.previous_time-self.design.check_s) < 1e-6:
            acceleration = (deficit-self.previous_deficit)/self.design.check_s
        urgent = bool(normal and normal['eta_raw'] < self.config['urgent_eta_threshold']
                      and deficit > 0 and acceleration is not None
                      and acceleration >= self.config['urgent_deficit_acceleration_mm_s2'])
        terminal = bool(signal and signal['terminal_rate_mm_s'] is not None
                        and signal['terminal_rate_mm_s'] <= self.config['terminal_rate_mm_s'])
        branch = 'normal' if normal_fired else 'urgent' if urgent else 'terminal' if terminal else ''
        self.last_decision = dict(trigger_branch=branch, normal_trigger=normal_fired, urgent_trigger=urgent,
            terminal_trigger=terminal, deficit_rate_mm_s=deficit, deficit_acceleration_mm_s2=acceleration,
            terminal_rate_mm_s=signal['terminal_rate_mm_s'] if signal else None,
            normal_eta_valid=normal is not None)
        self.previous_time = t; self.previous_deficit = deficit
        return bool(branch)


def bind_detector(original, config, success_depth):
    """Reuse the exact recovery function's code with private detector/log dependencies.

    No module globals are mutated. Only Detector, recent_signal and Stream differ;
    Stream changes diagnostic columns, never command/pose/recovery state.
    """
    state = {}
    base_stream = original.__globals__['Stream']
    def factory(policy, eta, force, design):
        if policy != 'productivity' or eta != NORMAL_ETA:
            raise ValueError('Unexpected frozen detector contract')
        state['detector'] = DetectorV2(config, design)
        return state['detector']
    def signal(rows, hz, onset, design):
        return features(rows, hz, onset, design, success_depth)
    class LoggedStream(base_stream):
        def add(self, row):
            decision = state['detector'].last_decision if row['check_performed'] else {}
            row.update(detector_version='v2', trigger_branch=decision.get('trigger_branch', ''),
                normal_trigger=decision.get('normal_trigger', False), urgent_trigger=decision.get('urgent_trigger', False),
                terminal_trigger=decision.get('terminal_trigger', False),
                deficit_rate_mm_s=decision.get('deficit_rate_mm_s'),
                deficit_acceleration_mm_s2=decision.get('deficit_acceleration_mm_s2'),
                terminal_rate_mm_s=decision.get('terminal_rate_mm_s'))
            row['eta_valid'] = decision.get('normal_eta_valid', False)
            super().add(row)
    namespace = dict(original.__globals__, Detector=factory, recent_signal=signal, Stream=LoggedStream)
    bound = FunctionType(original.__code__, namespace, original.__name__, original.__defaults__, original.__closure__)
    assert bound.__code__ is original.__code__
    assert {k for k in namespace if namespace[k] is not original.__globals__.get(k)} == {'Detector', 'recent_signal', 'Stream'}
    return bound
