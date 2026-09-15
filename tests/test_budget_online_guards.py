"""Exercise the frozen recovery loop with deterministic post-step observations.

Only selected AST definitions are compiled: importing the simulator modules
would otherwise initialize Isaac dependencies. The recovery control flow,
CSV Stream, guards and metrics wrapper are the actual production definitions.
Identity pose helpers are sufficient because these tests stop within the
first two STOP steps; they do not test robot dynamics or pose interpolation.
"""
import ast
import copy
import csv
from contextlib import nullcontext
from dataclasses import replace
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest

from research.forge_budget import classify, evidence
from research.forge_events import recovery_termination
from research.forge_mechanics_metrics import mechanics_recovery_metrics
from research.forge_mechanics_motion import WithdrawalProfile
from research.forge_protocol import clear, protocol, recovery_summary, retained, screened, within_budget


ROOT = Path(__file__).resolve().parents[1]


def definitions(path, names, namespace):
    """Compile named production definitions without importing a simulator."""
    tree = ast.parse(path.read_text(), filename=str(path))
    selected = []
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.ClassDef)) and node.name in names:
            selected.append(node)
        elif isinstance(node, ast.Assign) and any(
                isinstance(target, ast.Name) and target.id in names for target in node.targets):
            selected.append(node)
    exec(compile(ast.Module(body=selected, type_ignores=[]), str(path), 'exec'), namespace)


class DummyPose:
    """The clone, subtraction and XY slice operations used before STOP."""
    def __init__(self, values):
        self.values = [list(row) for row in values]

    def clone(self):
        return DummyPose(self.values)

    def __sub__(self, other):
        return DummyPose([[a-b for a, b in zip(row, other_row)]
                          for row, other_row in zip(self.values, other.values)])

    def __getitem__(self, key):
        rows, columns = key
        return DummyPose([row[columns] for row in self.values[rows]])

    def __setitem__(self, key, other):
        rows, columns = key
        for row, replacement in zip(self.values[rows], other.values):
            row[columns] = replacement


def forbidden_interpolation(*args, **kwargs):
    raise AssertionError('These bounded guard tests must not enter realign')


def production_functions():
    namespace = dict(csv=csv, math=math, WithdrawalProfile=WithdrawalProfile,
                     screened=screened, within_budget=within_budget, retained=retained, clear=clear,
                     quat_conjugate=lambda q: q.clone(), quat_apply=lambda q, p: p.clone(),
                     quat_mul=lambda a, b: b.clone(), quat_slerp=forbidden_interpolation,
                     contact_recording=lambda *args, **kwargs: nullcontext(),
                     recovery_summary=recovery_summary, mechanics_recovery_metrics=mechanics_recovery_metrics,
                     recovery_termination=recovery_termination)
    definitions(ROOT/'simulation/forge_experiment.py', {'Stream', 'blend', 'guarded'}, namespace)
    definitions(ROOT/'simulation/forge_mechanics_recovery.py', {'MOTION', 'recover'}, namespace)
    definitions(ROOT/'simulation/forge_mechanics.py', {'recovery'}, namespace)
    return namespace['recover'], namespace['recovery']


def state(**changes):
    row = dict(phase='hold', time_s=10., physics_time_s=10., reference_step=100,
               command_depth_mm=18., command_tilt_deg=1.5, depth_mm=18., tilt_deg=1.5,
               wrist_force_n=1., wrist_torque_nm=.1, force_norm_n=.2, torque_norm_nm=.01,
               min_separation_mm=0., normal_load_n=.2, contact_power_w=0.,
               grasp_slip_mm=.01, grasp_slip_deg=.01, lowest_peg_z_above_mouth_mm=-18.)
    row.update(changes)
    return row


class FakeBench:
    """No physics engine: tick advances the clock, observe supplies one sample."""
    def __init__(self, initial, observations):
        self.dt = .1  # Two physical intervals fit inside the original .25 s STOP.
        self.protocol = replace(protocol(), force_budget_n=4., effective_radial_clearance_mm=.2)
        self.initial = copy.deepcopy(initial)
        self.observations = copy.deepcopy(observations)
        self.tick_count = self.observe_count = 0
        self.commands = []
        self.env = SimpleNamespace(
            last_update_timestamp=10.,
            fingertip_midpoint_pos=DummyPose([[0., 0., 0.]]),
            fingertip_midpoint_quat=DummyPose([[1., 0., 0., 0.]]),
            held_pos=DummyPose([[0., 0., 0.]]), held_quat=DummyPose([[1., 0., 0., 0.]]),
            fixed_pos_obs_frame=DummyPose([[0., 0., 0.]]))
        self.peg_q = DummyPose([[1., 0., 0., 0.]])

    def tick(self, position, quaternion):
        if self.tick_count >= len(self.observations):
            raise AssertionError('Unexpected additional task tick after supplied stop condition')
        self.tick_count += 1
        self.commands.append((position.clone(), quaternion.clone()))
        self.env.last_update_timestamp = 10. + self.tick_count * self.dt

    def observe(self, phase, command_depth):
        if self.tick_count != self.observe_count + 1:
            raise AssertionError('Each observation must follow exactly one physical tick')
        row = copy.deepcopy(self.initial)
        row.update(self.observations[self.observe_count])
        row.update(phase=phase, command_depth_mm=command_depth,
                   time_s=self.env.last_update_timestamp, physics_time_s=self.env.last_update_timestamp)
        self.observe_count += 1
        return row


class OnlineBudgetGuardTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.recover, cls.recovery = map(staticmethod, production_functions())

    def run_loop(self, initial, observations):
        bench = FakeBench(initial, observations)
        with TemporaryDirectory() as directory:
            path = Path(directory)/'recovery.csv'
            rows, reason, motion = self.recover(bench, initial, 'straight', bench.protocol, path)
            with path.open(newline='') as stream:
                saved = list(csv.DictReader(stream))
        return bench, rows, saved, reason

    def run_wrapper(self, initial, observations):
        bench = FakeBench(initial, observations)
        with TemporaryDirectory() as directory:
            metrics = self.recovery(bench, [initial], 'straight', Path(directory), 'recovery')
            with (Path(directory)/'recovery.csv').open(newline='') as stream:
                saved = list(csv.DictReader(stream))
        record = dict(reference={'metrics': dict(reference_complete=True, numerically_valid=True,
            grasp_retained=True, reference_within_budget=True, depth_gate_passed=True)},
            recovery={'metrics': metrics})
        return bench, metrics, saved, classify(record)

    def test_initial_over_budget_preserves_copy_without_tick(self):
        bench, rows, saved, reason = self.run_loop(state(wrist_force_n=4.1), [])
        self.assertEqual(reason, 'operational_budget_exceeded')
        self.assertEqual((bench.tick_count, bench.observe_count), (0, 0))
        self.assertEqual(len(rows), 1)
        self.assertEqual(len(saved), 1)
        self.assertEqual(float(saved[0]['wrist_force_n']), 4.1)
        result = evidence(rows, bench.protocol, bench.dt, segment='recovery')
        self.assertEqual(result['executed_step_count'], 0)
        self.assertTrue(result['first_budget_crossing']['initial_state_already_over_budget'])

    def test_equality_allows_next_tick_then_strict_crossing_stops(self):
        bench, rows, saved, reason = self.run_loop(state(wrist_force_n=4.),
            [dict(wrist_force_n=4.), dict(wrist_force_n=4.25)])
        self.assertEqual(reason, 'operational_budget_exceeded')
        self.assertEqual((bench.tick_count, bench.observe_count), (2, 2))
        self.assertEqual([float(row['wrist_force_n']) for row in saved], [4., 4., 4.25])
        self.assertEqual(len(rows), 3)
        crossing = evidence(rows, bench.protocol, bench.dt, 'recovery')['first_budget_crossing']
        self.assertEqual(crossing['index'], 2)
        self.assertEqual(crossing['recovery_time_s'], .2)

    def test_crossing_sample_is_saved_without_consuming_next_observation(self):
        bench, rows, saved, reason = self.run_loop(state(),
            [dict(wrist_force_n=4.7), dict(wrist_force_n=99.)])
        self.assertEqual(reason, 'operational_budget_exceeded')
        self.assertEqual(bench.tick_count, 1)
        self.assertEqual(len(saved), 2)
        self.assertEqual(float(saved[-1]['wrist_force_n']), 4.7)
        self.assertEqual(saved[-1]['phase'], 'stop')
        self.assertAlmostEqual(evidence(rows, bench.protocol, bench.dt, 'recovery')
                               ['first_budget_crossing']['force_overshoot_n'], .7)

    def test_clearance_dwell_completed_on_crossing_sample_cannot_be_safe(self):
        cleared = dict(lowest_peg_z_above_mouth_mm=2.1, normal_load_n=0.)
        bench, metrics, saved, outcome = self.run_wrapper(state(),
            [dict(cleared, wrist_force_n=1.), dict(cleared, wrist_force_n=4.1)])
        self.assertEqual(bench.tick_count, 2)
        self.assertEqual(metrics['reason'], 'operational_budget_exceeded')
        self.assertTrue(metrics['cleared'])  # The final two samples satisfy the geometric dwell.
        self.assertTrue(metrics['force_budget_exceeded'])
        self.assertFalse(metrics['safe_recovery'])
        self.assertTrue(metrics['recovery_censored'])
        self.assertEqual(outcome['outcome'], 'recovery_budget_exceeded')
        self.assertFalse(outcome['task_safe'])
        self.assertEqual(float(saved[-1]['wrist_force_n']), 4.1)

    def test_numerical_rejection_wins_over_simultaneous_force_crossing(self):
        variants = [(dict(min_separation_mm=-.06), 'numerical_screen_failed'),
                    (dict(force_norm_n=501.), 'numerical_guard')]
        for extra, expected_reason in variants:
            with self.subTest(expected_reason=expected_reason):
                bench, metrics, saved, outcome = self.run_wrapper(state(),
                    [dict(extra, wrist_force_n=4.2)])
                self.assertEqual(bench.tick_count, 1)
                self.assertEqual(metrics['reason'], expected_reason)
                self.assertFalse(metrics['numerically_valid'])
                self.assertTrue(metrics['force_budget_exceeded'])
                self.assertFalse(metrics['label_eligible'])
                self.assertIsNone(metrics['safe_recovery'])
                self.assertEqual(outcome['outcome'], 'numerical_unknown')
                self.assertIsNone(outcome['task_safe'])
                self.assertEqual(float(saved[-1]['wrist_force_n']), 4.2)


if __name__ == '__main__':
    unittest.main()
