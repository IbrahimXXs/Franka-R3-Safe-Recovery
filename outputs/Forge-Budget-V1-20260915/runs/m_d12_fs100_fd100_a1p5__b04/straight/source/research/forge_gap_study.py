"""Bounded radial-gap / depth-triggered, time-ramped pitch study.

All clearances are nominal *radial* clearances.  Mesh metrology supplies the
actual clearance used by screening; a nominal 0.507 mm denotes the stock 9 mm
bore with the 7.986 mm peg.  The new schema intentionally cannot load or mutate
the earlier Phase 2B depth-ramp plans.
"""
import argparse
from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path

SCHEMA = 'Forge-gap-tilt-v1'
DEPTH_FEEDBACK = 'actual_depth_previous_step_first_insertion_crossing'
COMMAND_KEYS = ('offset_x_mm', 'offset_y_mm', 'roll_deg', 'pitch_deg')
RADIAL_CLEARANCES_MM = (.507, .4, .3, .2, .1)
ONSET_FRACTIONS = (.3, .5, .7)
AMPLITUDES_DEG = (.5, 1., 2., 4.)
PEG_DIAMETER_MM = 7.986
TARGET_DEPTH_MM = 20.
RAMP_DURATION_S = 1.


def _finite_number(value):
    return type(value) in (float, int) and math.isfinite(value)


def make_case(radial_clearance_mm, onset_fraction=None, amplitude_deg=0., sign=1):
    """Return a case without collection slot/retry metadata."""
    if (not _finite_number(radial_clearance_mm)
            or radial_clearance_mm not in RADIAL_CLEARANCES_MM
            or not _finite_number(amplitude_deg)
            or amplitude_deg not in (0., *AMPLITUDES_DEG)
            or type(sign) is not int or sign not in (-1, 1)):
        raise ValueError('Unsupported bounded gap-study parameters')
    control = amplitude_deg == 0.
    if control:
        if onset_fraction is not None or sign != 1:
            raise ValueError('Aligned controls have no onset and canonical sign +1')
    elif not _finite_number(onset_fraction) or onset_fraction not in ONSET_FRACTIONS:
        raise ValueError('Tilt onset must be 30%, 50%, or 70% of actual insertion depth')
    gap_key = f'g{round(radial_clearance_mm * 1000):04d}'
    suffix = 'aligned' if control else (
        f'd{round(onset_fraction * 100):02d}_pitch_{"pos" if sign > 0 else "neg"}'
        f'{round(amplitude_deg * 10):02d}')
    case_id = f'{gap_key}_{suffix}'
    return dict(
        case_id=case_id,
        family='centered' if control else 'tilt_only',
        radial_clearance_mm=float(radial_clearance_mm),
        clearance_definition='nominal_radial',
        hole_diameter_mm=round(PEG_DIAMETER_MM + 2 * radial_clearance_mm, 3),
        peg_diameter_mm=PEG_DIAMETER_MM,
        target_depth_mm=TARGET_DEPTH_MM,
        tilt_onset_fraction=None if control else float(onset_fraction),
        tilt_onset_mm=None if control else round(TARGET_DEPTH_MM * onset_fraction, 6),
        tilt_amplitude_deg=float(amplitude_deg),
        tilt_sign=sign,
        tilt_axis='pitch',
        tilt_ramp_duration_s=RAMP_DURATION_S,
        offset_x_mm=0., offset_y_mm=0., roll_deg=0., pitch_deg=0.,
        final_pitch_deg=sign * float(amplitude_deg),
        insertion_duration_s=8.,
        sample_role='aligned_control' if control else 'gap_tilt_characterization',
        # Keep opposite directions and nearby severities together when splitting
        # data: otherwise a train/test split can leak nearly identical paths.
        split_group_id=f'{gap_key}_aligned' if control else f'{gap_key}_d{round(onset_fraction * 100):02d}_pitch',
        path_group_id=case_id,
    )


def make_plan(pilot=False):
    """Create the 23-case pilot or 125-case matrix, with a shared pilot prefix."""
    if type(pilot) is not bool:
        raise ValueError('pilot must be a boolean')
    cases = [make_case(gap) for gap in RADIAL_CLEARANCES_MM]
    cases.extend(make_case(gap, onset, amplitude)
                 for gap in (.507, .2, .1)
                 for onset in ONSET_FRACTIONS for amplitude in (.5, 2.))
    if not pilot:
        existing = {case['case_id'] for case in cases}
        matrix = (make_case(gap, onset, amplitude, sign)
                  for gap in RADIAL_CLEARANCES_MM for onset in ONSET_FRACTIONS
                  for amplitude in AMPLITUDES_DEG for sign in (1, -1))
        cases.extend(case for case in matrix if case['case_id'] not in existing)
    cases = [dict(case, slot=slot, source_slot=slot, retry=0, trajectory_id=f'{case["case_id"]}_try00')
             for slot, case in enumerate(cases)]
    return dict(
        schema=SCHEMA,
        study_size='pilot' if pilot else 'full',
        clearance_definition='nominal_radial',
        depth_feedback=DEPTH_FEEDBACK,
        ramp_clock='elapsed_time_since_first_insertion_depth_crossing',
        ramp_shape='smoothstep',
        probe_terminal=True,
        probe_first_stall=True,
        trigger_observation_note=(
            'The command for the next physics step observes actual depth from '
            'the previous completed step. Trigger timing is quantized to the '
            'physics step; recorded trigger time/depth are the observed sample.'),
        cases=cases,
    )


def load_plan(path):
    """Validate a frozen matrix, rejecting changed geometry or ramp semantics."""
    value = json.loads(Path(path).read_text())
    if not isinstance(value, dict) or value.get('schema') != SCHEMA:
        raise ValueError('Unsupported gap-study schema')
    if value.get('study_size') not in ('pilot', 'full'):
        raise ValueError('Gap study must identify pilot or full matrix')
    expected = make_plan(pilot=value['study_size'] == 'pilot')
    if 'subset_case_ids' in value:
        expected = subset_plan(expected, cases=value['subset_case_ids'])
    if value != expected:
        raise ValueError('Gap-study plan differs from the bounded canonical matrix or protocol')
    return value


def subset_plan(case_plan, cases=None, radial_clearance_mm=None):
    """Select stable cases for a process with fixed geometry, keeping provenance.

    ``cases`` is a sequence of canonical case IDs or case dictionaries. Exactly
    one selector is required. Order always follows the original matrix rather
    than selector order; local slots are reindexed and source slots preserved.
    """
    if (cases is None) == (radial_clearance_mm is None):
        raise ValueError('Choose either cases or radial_clearance_mm')
    if cases is not None:
        ids = [case['case_id'] if isinstance(case, dict) else case for case in cases]
        if not ids or len(set(ids)) != len(ids):
            raise ValueError('Subset must contain unique case IDs')
        known = {case['case_id'] for case in case_plan['cases']}
        if not set(ids) <= known:
            raise ValueError('Subset contains an unknown case ID')
        selected = [case for case in case_plan['cases'] if case['case_id'] in ids]
    else:
        selected = [case for case in case_plan['cases'] if case['radial_clearance_mm'] == radial_clearance_mm]
        if not selected:
            raise ValueError('No cases have the selected radial clearance')
    original = make_plan(pilot=case_plan['study_size'] == 'pilot')
    subset = {key: value for key, value in original.items() if key != 'cases'}
    subset['source_plan'] = dict(
        schema=original['schema'], study_size=original['study_size'],
        case_count=len(original['cases']),
        sha256=hashlib.sha256(json.dumps(original, sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
    )
    subset['subset_case_ids'] = [case['case_id'] for case in selected]
    subset['cases'] = [dict(case, slot=slot) for slot, case in enumerate(selected)]
    return subset


def case_for_slot(case_plan, slot, retry):
    """Retry the exact same physical case; its case/group identifiers stay fixed."""
    if type(slot) is not int or not 0 <= slot < len(case_plan['cases']):
        raise ValueError('Case slot is out of range')
    if type(retry) is not int or retry < 0:
        raise ValueError('Retry must be a nonnegative integer')
    case = case_plan['cases'][slot]
    return dict(case, retry=retry, trajectory_id=f'{case["case_id"]}_try{retry:02d}')


def command_misalignment(case, elapsed_since_trigger_s=None):
    """Tilt continues with time even when depth stops progressing or decreases."""
    if elapsed_since_trigger_s is not None and not _finite_number(elapsed_since_trigger_s):
        raise ValueError('Ramp elapsed time must be finite')
    fraction = (0. if elapsed_since_trigger_s is None else
                max(0., min(1., elapsed_since_trigger_s / case['tilt_ramp_duration_s'])))
    smooth = fraction * fraction * (3. - 2. * fraction)
    commands = {key: float(case[key]) for key in COMMAND_KEYS}
    commands['pitch_deg'] += smooth * case['final_pitch_deg']
    return commands


@dataclass
class TiltTriggerState:
    """Per-reference/replay trigger state; create a fresh instance for each run.

    ``ramp_fraction`` records linear time progress; the command applies a
    smoothstep to that fraction. ``elapsed_s`` must use the same monotonically increasing clock as the
    supplied completed physics observation. A depth sample observed outside the
    insertion phase cannot arm the perturbation. ``None`` is never a failure.
    """
    triggered: bool = False
    trigger_time_s: float | None = None
    trigger_depth_mm: float | None = None
    trigger_step: int | None = None
    ramp_fraction: float = 0.
    ramp_completed: bool = False
    ramp_complete_time_s: float | None = None
    last_observation_time_s: float | None = None

    def update(self, case, elapsed_s, actual_depth_mm, in_insertion=True, physics_step=None):
        if not _finite_number(elapsed_s) or not _finite_number(actual_depth_mm):
            raise ValueError('Trigger time and actual depth must be finite')
        if self.last_observation_time_s is not None and elapsed_s < self.last_observation_time_s:
            raise ValueError('Trigger observation clock must be monotonic; use fresh state for replay')
        if physics_step is not None and (type(physics_step) is not int or physics_step < 0):
            raise ValueError('Physics step must be a nonnegative integer')
        self.last_observation_time_s = float(elapsed_s)
        if (not self.triggered and in_insertion and case['tilt_amplitude_deg'] > 0.
                and actual_depth_mm >= case['tilt_onset_mm']):
            self.triggered = True
            self.trigger_time_s = float(elapsed_s)
            self.trigger_depth_mm = float(actual_depth_mm)
            self.trigger_step = physics_step
        elapsed = None if not self.triggered else elapsed_s - self.trigger_time_s
        self.ramp_fraction = (0. if elapsed is None else
                              max(0., min(1., elapsed / case['tilt_ramp_duration_s'])))
        if (elapsed is not None and not self.ramp_completed
                and elapsed >= case['tilt_ramp_duration_s']):
            self.ramp_completed = True
            self.ramp_complete_time_s = float(elapsed_s)
        return command_misalignment(case, elapsed)

    def command(self, case, elapsed_s):
        """Compute a command on the same clock without consuming a depth sample."""
        return command_misalignment(case, None if not self.triggered else elapsed_s - self.trigger_time_s)

    def as_dict(self):
        return {'tilt_triggered': self.triggered,
                'tilt_trigger_time_s': self.trigger_time_s,
                'tilt_trigger_depth_mm': self.trigger_depth_mm,
                'tilt_trigger_step': self.trigger_step,
                'tilt_ramp_fraction': self.ramp_fraction,
                'tilt_ramp_completed': self.ramp_completed,
                'tilt_ramp_complete_time_s': self.ramp_complete_time_s}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pilot', action='store_true')
    args = parser.parse_args()
    with args.output.open('x') as stream:
        json.dump(make_plan(args.pilot), stream, indent=2)
        stream.write('\n')
    load_plan(args.output)


if __name__ == '__main__':
    main()
