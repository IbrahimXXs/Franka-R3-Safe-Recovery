"""Independent depth/friction mechanics matrix and causal reference schedule.

The scene has a 0.2 mm radial gap and a 25 mm blind hole. Each condition gets
one reference, with no deterministic retries. A depth gate precedes the tilt
sequence; depth never advances while tilt ramps or the final pose is held.
"""
import argparse
from dataclasses import dataclass, field
import hashlib
import json
import math
from pathlib import Path

SCHEMA = 'Forge-mechanics-v1'
TARGET_DEPTHS_MM = (6., 12., 18.)
FRICTION_PAIRS = ((.5, .5), (.75, .75), (1., 1.), (1., .5))
TILT_AMPLITUDES_DEG = (0., 2.)
BOUNDARY_TILT_AMPLITUDES_DEG = (0., 1., 1.5)
EXTENDED_TILT_AMPLITUDES_DEG = (3., 4., 5., 6.)
BOUNDARY_DESIGN = 'boundary_angles_v1'
PEG_FRICTION = .75
_EPS = 1e-9


def _finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def make_case(target_depth_mm, friction_pair, tilt_amplitude_deg):
    pair = tuple(friction_pair)
    if (not _finite(target_depth_mm) or target_depth_mm not in TARGET_DEPTHS_MM
            or len(pair) != 2 or any(not _finite(value) for value in pair) or pair not in FRICTION_PAIRS
            or not _finite(tilt_amplitude_deg)
            or tilt_amplitude_deg not in (*TILT_AMPLITUDES_DEG, *BOUNDARY_TILT_AMPLITUDES_DEG,
                                          *EXTENDED_TILT_AMPLITUDES_DEG)):
        raise ValueError('Unsupported bounded mechanics condition')
    static, dynamic = pair
    group = f'm_d{round(target_depth_mm):02d}_fs{round(static * 100):03d}_fd{round(dynamic * 100):03d}'
    angle_id = '1p5' if tilt_amplitude_deg == 1.5 else f'{round(tilt_amplitude_deg):02d}'
    case_id = f'{group}_a{angle_id}'
    return dict(
        case_id=case_id, family='centered' if tilt_amplitude_deg == 0 else 'tilt_only',
        radial_clearance_mm=.2, clearance_definition='nominal_radial',
        peg_diameter_mm=7.986, hole_diameter_mm=8.386, hole_depth_mm=25., hole_type='blind',
        target_depth_mm=float(target_depth_mm),
        pair_static_friction=float(static), pair_dynamic_friction=float(dynamic),
        peg_static_friction=PEG_FRICTION, peg_dynamic_friction=PEG_FRICTION,
        hole_static_friction=2. * static - PEG_FRICTION,
        hole_dynamic_friction=2. * dynamic - PEG_FRICTION,
        friction_combine_mode='average',
        tilt_amplitude_deg=float(tilt_amplitude_deg), tilt_axis='pitch', tilt_sign=1,
        offset_x_mm=0., offset_y_mm=0., roll_deg=0., pitch_deg=0.,
        final_pitch_deg=float(tilt_amplitude_deg),
        approach_duration_s=2., insertion_start_depth_mm=-10.,
        insertion_duration_s=(target_depth_mm + 10.) / 3.75,
        insertion_distance_rate_mm_s=3.75,
        settle_tolerance_mm=.1, settle_dwell_s=.2, settle_timeout_s=2.,
        tilt_ramp_duration_s=1., post_tilt_hold_s=1.,
        sample_role='aligned_control' if tilt_amplitude_deg == 0 else 'depth_friction_tilt_characterization',
        split_group_id=group, path_group_id=case_id,
    )


def make_plan():
    cases = [make_case(depth, pair, angle) for angle in TILT_AMPLITUDES_DEG
             for depth in TARGET_DEPTHS_MM for pair in FRICTION_PAIRS]
    cases = [dict(case, slot=slot, source_slot=slot, retry=0,
                  trajectory_id=f'{case["case_id"]}_try00') for slot, case in enumerate(cases)]
    return dict(schema=SCHEMA, attempts_per_case=1, retries_per_case=0,
                clearance_definition='nominal_radial',
                reference_protocol='aligned_insert_depth_gate_time_tilt_fixed_depth_hold',
                observation_feedback='previous_completed_physics_step',
                depth_gate='within_target_plus_minus_tolerance_for_contiguous_dwell',
                friction_combination='arithmetic_mean_of_peg_and_hole_coefficients',
                cases=cases)


def make_boundary_plan():
    """Bounded follow-up after the main matrix: 18 mm, pair (1,1), 0/1/1.5 deg.

    The original 24-case plan remains byte-for-byte unchanged. This independent
    design must use a separate output directory; all other case physics stay
    canonical and the existing state machine needs no new transition logic.
    """
    original = make_plan()
    value = {key: item for key, item in original.items() if key != 'cases'}
    value['design'] = BOUNDARY_DESIGN
    cases = [make_case(18., (1., 1.), angle) for angle in BOUNDARY_TILT_AMPLITUDES_DEG]
    value['cases'] = [dict(case, slot=slot, source_slot=slot, retry=0,
                           trajectory_id=f'{case["case_id"]}_try00') for slot, case in enumerate(cases)]
    return value


def _canonical_plan(case_plan):
    design = case_plan.get('design')
    if design is None:
        return make_plan()
    if design == BOUNDARY_DESIGN:
        return make_boundary_plan()
    raise ValueError('Unsupported mechanics design')


def subset_plan(case_plan, cases=None, effective_friction_pair=None):
    """Select cases with canonical order/identity and frozen full-plan provenance."""
    if (cases is None) == (effective_friction_pair is None):
        raise ValueError('Choose cases or effective_friction_pair')
    if cases is not None:
        ids = [case['case_id'] if isinstance(case, dict) else case for case in cases]
        known = {case['case_id'] for case in case_plan['cases']}
        if not ids or len(set(ids)) != len(ids) or not set(ids) <= known:
            raise ValueError('Subset requires unique known case IDs')
        selected = [case for case in case_plan['cases'] if case['case_id'] in ids]
    else:
        pair = tuple(effective_friction_pair)
        selected = [case for case in case_plan['cases']
                    if (case['pair_static_friction'], case['pair_dynamic_friction']) == pair]
        if not selected:
            raise ValueError('No cases have the selected friction pair')
    original = _canonical_plan(case_plan)
    result = {key: value for key, value in original.items() if key != 'cases'}
    result['source_plan'] = dict(schema=SCHEMA, case_count=len(original['cases']),
        sha256=hashlib.sha256(json.dumps(original, sort_keys=True, separators=(',', ':')).encode()).hexdigest())
    if 'design' in original:
        result['source_plan']['design'] = original['design']
    result['subset_case_ids'] = [case['case_id'] for case in selected]
    result['cases'] = [dict(case, slot=slot) for slot, case in enumerate(selected)]
    return result


def load_plan(path):
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict) or value.get('schema') != SCHEMA:
        raise ValueError('Unsupported mechanics plan schema')
    expected = _canonical_plan(value)
    if 'subset_case_ids' in value:
        expected = subset_plan(expected, cases=value['subset_case_ids'])
    if value != expected:
        raise ValueError('Mechanics plan differs from its bounded canonical matrix or protocol')
    return value


def case_for_slot(case_plan, slot, retry=0):
    if type(slot) is not int or not 0 <= slot < len(case_plan['cases']):
        raise ValueError('Mechanics slot is out of range')
    if type(retry) is not int or retry != 0:
        raise ValueError('Mechanics conditions allow one attempt; deterministic retries are disabled')
    return dict(case_plan['cases'][slot])


def _smoothstep(fraction):
    fraction = max(0., min(1., fraction))
    return fraction * fraction * (3. - 2. * fraction)


@dataclass
class MechanicsState:
    """One state per reference or replay, updated only after completed samples.

    Call ``command(next_time_s)`` before a physics step, then ``observe`` with
    that step's actual depth and validity. The successful settle sample still
    had zero applied tilt; only the next command can begin the time ramp.
    """
    case: dict
    depth_attained: bool = field(default=False, init=False)
    depth_attainment_time_s: float | None = field(default=None, init=False)
    depth_attainment_depth_mm: float | None = field(default=None, init=False)
    depth_attainment_step: int | None = field(default=None, init=False)
    settle_window_start_time_s: float | None = field(default=None, init=False)
    settle_duration_observed_s: float = field(default=0., init=False)
    depth_gate_passed: bool = field(default=False, init=False)
    depth_gate_time_s: float | None = field(default=None, init=False)
    depth_gate_step: int | None = field(default=None, init=False)
    tilt_ramp_completed: bool = field(default=False, init=False)
    tilt_ramp_complete_time_s: float | None = field(default=None, init=False)
    tilt_ramp_fraction: float = field(default=0., init=False)
    prefix_numerically_valid: bool = field(default=True, init=False)
    prefix_grasp_retained: bool = field(default=True, init=False)
    done: bool = field(default=False, init=False)
    termination_reason: str | None = field(default=None, init=False)
    termination_time_s: float | None = field(default=None, init=False)
    last_observation_time_s: float | None = field(default=None, init=False)

    def __post_init__(self):
        expected = make_case(self.case['target_depth_mm'],
                             (self.case['pair_static_friction'], self.case['pair_dynamic_friction']),
                             self.case['tilt_amplitude_deg'])
        if any(self.case.get(key) != value for key, value in expected.items()):
            raise ValueError('Mechanics state needs an unmodified canonical case')
        self.case = dict(self.case)

    @property
    def insertion_end_time_s(self):
        return self.case['approach_duration_s'] + self.case['insertion_duration_s']

    @property
    def settle_deadline_s(self):
        return self.insertion_end_time_s + self.case['settle_timeout_s']

    def command(self, elapsed_s):
        if not _finite(elapsed_s) or elapsed_s < 0:
            raise ValueError('Command time must be finite and nonnegative')
        case = self.case
        if elapsed_s < case['approach_duration_s']:
            phase, depth, pitch = 'approach', case['insertion_start_depth_mm'], 0.
        elif elapsed_s < self.insertion_end_time_s:
            progress = _smoothstep((elapsed_s - case['approach_duration_s']) / case['insertion_duration_s'])
            phase = 'insert'
            depth = case['insertion_start_depth_mm'] + progress * (case['target_depth_mm'] - case['insertion_start_depth_mm'])
            pitch = 0.
        elif not self.depth_gate_passed:
            phase, depth, pitch = 'settle', case['target_depth_mm'], 0.
        else:
            elapsed = max(0., elapsed_s - self.depth_gate_time_s)
            phase = 'tilt' if elapsed < case['tilt_ramp_duration_s'] else 'hold'
            depth = case['target_depth_mm']
            pitch = case['final_pitch_deg'] * _smoothstep(elapsed / case['tilt_ramp_duration_s'])
        return dict(phase=phase, depth_mm=float(depth), pitch_deg=float(pitch), done=self.done,
                    approach_fraction=_smoothstep(elapsed_s / case['approach_duration_s']))

    def _finish(self, reason, elapsed_s):
        self.done = True
        self.termination_reason = reason
        self.termination_time_s = float(elapsed_s)

    def observe(self, elapsed_s, actual_depth_mm, numerically_valid=True,
                grasp_retained=True, physics_step=None):
        if not _finite(elapsed_s) or elapsed_s < 0 or not _finite(actual_depth_mm):
            raise ValueError('Observed clock and depth must be finite; clock must be nonnegative')
        if self.last_observation_time_s is not None and elapsed_s < self.last_observation_time_s:
            raise ValueError('Observation clock must be monotonic; use fresh state for each replay')
        if physics_step is not None and (type(physics_step) is not int or physics_step < 0):
            raise ValueError('Physics step must be a nonnegative integer')
        if self.done:
            return self.as_dict()
        self.last_observation_time_s = float(elapsed_s)
        self.prefix_numerically_valid &= bool(numerically_valid)
        self.prefix_grasp_retained &= bool(grasp_retained)
        if not self.prefix_numerically_valid:
            self._finish('numerical_invalid', elapsed_s)
            return self.as_dict()
        if not self.prefix_grasp_retained:
            self._finish('grasp_lost', elapsed_s)
            return self.as_dict()
        case = self.case
        if (elapsed_s >= case['approach_duration_s'] and not self.depth_attained
                and actual_depth_mm >= case['target_depth_mm'] - case['settle_tolerance_mm']):
            self.depth_attained = True
            self.depth_attainment_time_s = float(elapsed_s)
            self.depth_attainment_depth_mm = float(actual_depth_mm)
            self.depth_attainment_step = physics_step
        if not self.depth_gate_passed and elapsed_s >= self.insertion_end_time_s:
            within = abs(actual_depth_mm - case['target_depth_mm']) <= case['settle_tolerance_mm'] + _EPS
            if within:
                if self.settle_window_start_time_s is None:
                    self.settle_window_start_time_s = float(elapsed_s)
                self.settle_duration_observed_s = elapsed_s - self.settle_window_start_time_s
            else:
                self.settle_window_start_time_s = None
                self.settle_duration_observed_s = 0.
            if (elapsed_s <= self.settle_deadline_s + _EPS and within
                    and self.settle_duration_observed_s + _EPS >= case['settle_dwell_s']):
                self.depth_gate_passed = True
                self.depth_gate_time_s = float(elapsed_s)
                self.depth_gate_step = physics_step
            elif elapsed_s >= self.settle_deadline_s - _EPS:
                self._finish('depth_gate_timeout', elapsed_s)
                return self.as_dict()
        if self.depth_gate_passed:
            elapsed = max(0., elapsed_s - self.depth_gate_time_s)
            self.tilt_ramp_fraction = min(1., elapsed / case['tilt_ramp_duration_s'])
            if not self.tilt_ramp_completed and elapsed + _EPS >= case['tilt_ramp_duration_s']:
                self.tilt_ramp_completed = True
                self.tilt_ramp_complete_time_s = float(elapsed_s)
            if elapsed + _EPS >= case['tilt_ramp_duration_s'] + case['post_tilt_hold_s']:
                self._finish('reference_complete', elapsed_s)
        return self.as_dict()

    def as_dict(self):
        return dict(
            depth_attained=self.depth_attained, depth_attainment_time_s=self.depth_attainment_time_s,
            depth_attainment_depth_mm=self.depth_attainment_depth_mm, depth_attainment_step=self.depth_attainment_step,
            settle_window_start_time_s=self.settle_window_start_time_s,
            settle_duration_observed_s=self.settle_duration_observed_s,
            depth_gate_passed=self.depth_gate_passed, depth_gate_time_s=self.depth_gate_time_s,
            depth_gate_step=self.depth_gate_step, tilt_sequence_started=self.depth_gate_passed,
            tilt_triggered=self.depth_gate_passed and self.case['tilt_amplitude_deg'] > 0,
            tilt_trigger_time_s=self.depth_gate_time_s if self.case['tilt_amplitude_deg'] > 0 else None,
            tilt_ramp_fraction=self.tilt_ramp_fraction, tilt_ramp_completed=self.tilt_ramp_completed,
            tilt_ramp_complete_time_s=self.tilt_ramp_complete_time_s,
            prefix_numerically_valid=self.prefix_numerically_valid, prefix_grasp_retained=self.prefix_grasp_retained,
            reference_done=self.done, reference_termination_reason=self.termination_reason,
            reference_termination_time_s=self.termination_time_s,
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    with args.output.open('x') as stream:
        json.dump(make_plan(), stream, indent=2)
        stream.write('\n')
    load_plan(args.output)


if __name__ == '__main__':
    main()
