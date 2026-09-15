"""Coordinator tests use fake child ledgers and never start Isaac Sim."""
import csv
import fcntl
import json
import os
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

from research.forge_gap_study import load_plan
from simulation import run_gap_study as batch


ROOT = Path(__file__).resolve().parents[1]


def args_for(directory, **changes):
    args = dict(case_plan=ROOT / 'experiments/forge_gap_pilot.json',
                output_dir=Path(directory) / 'pilot', physics_hz=120,
                phase='all', prepare_only=False, resume=False, max_new_attempts=None)
    args.update(changes)
    return SimpleNamespace(**args)


def attempt(case, **metrics):
    values = dict(numerically_valid=True, grasp_retained=True, insertion_success=True,
                  force_budget_exceeded=False, torque_budget_exceeded=False)
    values.update(metrics)
    return dict(case, status='complete', metrics=values,
                final_retreat={'safe_recovery': True}, checkpoints=[])


def ledger(path, cases, status='paused'):
    path.parent.mkdir(parents=True, exist_ok=True)
    batch.save(path, dict(attempts=cases, status=status))


class GapBatchTests(unittest.TestCase):
    def running_manifest(self, identity=True):
        command = ['bash', str(ROOT / 'forge_study.sh'), '--headless', '--case-plan',
                   '/tmp/study/plans/gap.json', '--output-dir', '/tmp/study/gap_0100']
        record = dict(group='gap_0100', stage='controls', command=command, pid=123,
                      log='logs/control.log', status='running', started_unix_s=100.)
        if identity:
            record['process_identity'] = dict(boot_id='test-boot', start_time_ticks=1000,
                                              started_unix_s=100.)
        return dict(executions=[record], groups=[dict(folder='gap_0100')])

    def child_snapshot(self, record, **changes):
        value = dict(pid=123, process_group=123, start_time_ticks=1000,
                     started_unix_s=100., boot_id='test-boot',
                     command=['/opt/conda/bin/python', str(ROOT / 'simulation/launch_forge_study.py'),
                              *record['command'][2:]])
        value.update(changes)
        return value

    def test_live_child_identity_blocks_resume_without_signalling_any_process(self):
        manifest = self.running_manifest()
        observed = self.child_snapshot(manifest['executions'][0])
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(batch, 'process_snapshots', return_value=[observed]), patch.object(batch.os, 'kill') as kill:
                with self.assertRaisesRegex(RuntimeError, 'still running.*123'):
                    batch.reconcile_running_executions(Path(directory), manifest)
            kill.assert_not_called()
        self.assertEqual(manifest['executions'][0]['status'], 'running')

    def test_surviving_launcher_in_original_process_group_blocks_after_leader_exit(self):
        manifest = self.running_manifest()
        child = self.child_snapshot(manifest['executions'][0], pid=456, start_time_ticks=1300, started_unix_s=103.)
        with tempfile.TemporaryDirectory() as directory, patch.object(batch, 'process_snapshots', return_value=[child]):
            with self.assertRaisesRegex(RuntimeError, '456'):
                batch.reconcile_running_executions(Path(directory), manifest)

    def test_reused_pid_or_changed_boot_does_not_block_an_unrelated_new_process(self):
        for changes in (dict(start_time_ticks=9999), dict(boot_id='new-boot'),
                        dict(command=['python', '/other/project.py'])):
            manifest = self.running_manifest()
            observed = self.child_snapshot(manifest['executions'][0], **changes)
            with tempfile.TemporaryDirectory() as directory, patch.object(batch, 'process_snapshots', return_value=[observed]):
                batch.reconcile_running_executions(Path(directory), manifest)
            self.assertEqual(manifest['executions'][0]['status'], 'interrupted')

    def test_legacy_record_matches_exec_command_and_start_time(self):
        manifest = self.running_manifest(identity=False)
        observed = self.child_snapshot(manifest['executions'][0])
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(batch, 'process_snapshots', return_value=[observed]):
                with self.assertRaisesRegex(RuntimeError, 'still running'):
                    batch.reconcile_running_executions(Path(directory), manifest)
            observed['started_unix_s'] = 90.
            with patch.object(batch, 'process_snapshots', return_value=[observed]):
                batch.reconcile_running_executions(Path(directory), manifest)
        self.assertEqual(manifest['executions'][0]['status'], 'interrupted')

    def test_live_child_without_saved_pid_and_locked_child_directory_are_protected(self):
        manifest = self.running_manifest(identity=False)
        record = manifest['executions'][0]
        record.pop('pid')
        observed = self.child_snapshot(record)
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(batch, 'process_snapshots', return_value=[observed]):
                with self.assertRaisesRegex(RuntimeError, 'still running'):
                    batch.reconcile_running_executions(Path(directory), manifest)
            folder = Path(directory) / 'gap_0100'
            folder.mkdir()
            with (folder / '.run.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with patch.object(batch, 'process_snapshots', return_value=[]):
                    with self.assertRaisesRegex(RuntimeError, 'run lock'):
                        batch.reconcile_running_executions(Path(directory), manifest)

    def test_stale_record_is_persisted_interrupted_and_prepare_resume_can_continue(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            batch.run(args)
            path = args.output_dir / 'batch.json'
            manifest = json.loads(path.read_text())
            manifest['executions'] = self.running_manifest()['executions']
            batch.save(path, manifest)
            with patch.object(batch, 'process_snapshots', return_value=[]):
                batch.run(args_for(directory, prepare_only=True, resume=True))
            saved = json.loads(path.read_text())
            self.assertEqual(saved['executions'][0]['status'], 'interrupted')
            self.assertIn('interruption_reason', saved['executions'][0])

    def test_linux_snapshot_identifies_this_process_with_kernel_start_time(self):
        observed = batch.process_snapshot(os.getpid())
        self.assertEqual(observed['pid'], os.getpid())
        self.assertGreater(observed['start_time_ticks'], 0)
        self.assertTrue(observed['boot_id'])
        self.assertTrue(observed['command'])

    def test_prepare_preserves_plan_identity_and_puts_control_first_per_geometry(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            with patch.object(batch, 'execute_group') as execute:
                batch.run(args)
            execute.assert_not_called()
            saved = json.loads((args.output_dir / 'batch.json').read_text())
            self.assertEqual(saved['planned_references'], 23)
            self.assertEqual(len(saved['groups']), 5)
            total = 0
            for group in saved['groups']:
                child = load_plan(args.output_dir / group['plan'])
                self.assertEqual(child['cases'][0]['tilt_amplitude_deg'], 0.)
                self.assertEqual(child['cases'][0]['slot'], 0)
                self.assertEqual({c['radial_clearance_mm'] for c in child['cases']},
                                 {group['radial_clearance_mm']})
                total += len(child['cases'])
            self.assertEqual(total, 23)
            self.assertTrue((args.output_dir / 'report.md').is_file())

    def test_all_controls_run_before_tilt_and_failed_controls_gate_their_group(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory)
            calls = []

            def fake_execute(options, output, group, stage, manifest):
                calls.append((group['radial_clearance_mm'], stage))
                cases = load_plan(output / group['plan'])['cases']
                gap = group['radial_clearance_mm']
                if stage == 'controls':
                    result = attempt(cases[0], numerically_valid=gap != .2,
                                     insertion_success=gap != .1)
                    ledger(output / group['folder'] / 'study.json', [result],
                           status='complete' if len(cases) == 1 else 'paused')
                else:
                    ledger(output / group['folder'] / 'study.json',
                           [attempt(case) for case in cases], status='complete')
                batch.refresh(output, manifest)

            with patch.object(batch, 'execute_group', side_effect=fake_execute):
                batch.run(args)
            self.assertEqual([stage for _, stage in calls[:5]], ['controls'] * 5)
            self.assertEqual(calls[5:], [(.507, 'perturbations')])
            saved = json.loads((args.output_dir / 'batch.json').read_text())
            statuses = {g['radial_clearance_mm']: g['baseline_status'] for g in saved['groups']}
            self.assertEqual(statuses[.2], 'numerical_review')
            self.assertEqual(statuses[.1], 'aligned_insertion_review')
            self.assertEqual(saved['status'], 'review_required')
            # Numerically valid insertion failures are retained; they do not
            # falsely clear the aligned-control gate.
            self.assertEqual(saved['valid_references'], 10)
            self.assertEqual(saved['complete_references'], 11)
            with patch.object(batch, 'execute_group') as execute:
                batch.run(args_for(directory, resume=True))
            execute.assert_not_called()

    def test_exhausted_case_budget_is_terminal_and_repeated_resume_never_launches_child(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            batch.run(args)
            manifest = json.loads((args.output_dir / 'batch.json').read_text())
            for group in manifest['groups']:
                cases = load_plan(args.output_dir / group['plan'])['cases']
                records = [attempt(case) for case in cases]
                status = 'complete'
                if group['radial_clearance_mm'] == .2:
                    records = records[:-1] + [dict(attempt(cases[-1], numerically_valid=False), retry=i)
                                              for i in range(5)]
                    status = 'target_not_met'
                ledger(args.output_dir / group['folder'] / 'study.json', records, status=status)
            with patch.object(batch, 'execute_group') as execute:
                batch.run(args_for(directory, resume=True))
                batch.run(args_for(directory, resume=True))
            execute.assert_not_called()
            saved = json.loads((args.output_dir / 'batch.json').read_text())
            self.assertEqual(saved['status'], 'target_not_met')
            self.assertEqual(saved['valid_references'], 22)
            self.assertEqual(saved['complete_references'], 27)
            self.assertEqual(saved['exhausted_groups'], ['gap_0200'])
            report = (args.output_dir / 'report.md').read_text()
            self.assertIn('22 / 23', report)
            self.assertIn('**target_not_met**', report)
            self.assertIn('will not launch again', report)

    def test_exhausted_group_is_skipped_while_resumable_other_group_keeps_batch_paused(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            batch.run(args)
            manifest = json.loads((args.output_dir / 'batch.json').read_text())
            for group in manifest['groups']:
                cases = load_plan(args.output_dir / group['plan'])['cases']
                records = [attempt(case) for case in cases]
                status = 'complete'
                if group['radial_clearance_mm'] == .2:
                    records = records[:-1] + [dict(attempt(cases[-1], numerically_valid=False), retry=i)
                                              for i in range(5)]
                    status = 'target_not_met'
                elif group['radial_clearance_mm'] == .1:
                    records, status = records[:1], 'paused'
                ledger(args.output_dir / group['folder'] / 'study.json', records, status=status)
            with patch.object(batch, 'execute_group') as execute:
                batch.run(args_for(directory, resume=True, max_new_attempts=1))
            self.assertEqual(execute.call_count, 1)
            self.assertEqual(execute.call_args.args[2]['folder'], 'gap_0100')
            self.assertEqual(execute.call_args.args[3], 'perturbations')
            saved = json.loads((args.output_dir / 'batch.json').read_text())
            self.assertEqual(saved['status'], 'paused')
            self.assertEqual(saved['exhausted_groups'], ['gap_0200'])

    def test_failed_control_review_is_distinct_from_exhausted_approved_groups(self):
        groups = [dict(folder='done', baseline_status='passed', study_status='complete'),
                  dict(folder='blocked', baseline_status='aligned_insertion_review', study_status='paused')]
        manifest = dict(groups=groups, valid_references=2, planned_references=3)
        self.assertEqual(batch.batch_status(manifest, 'all'), 'review_required')
        groups[0]['study_status'] = 'target_not_met'
        self.assertEqual(batch.batch_status(manifest, 'all'), 'target_not_met')
        groups[0]['study_status'] = 'paused'
        self.assertEqual(batch.batch_status(manifest, 'all'), 'paused')

    def test_controls_only_retains_phase_boundary_without_hiding_finished_exhaustion(self):
        manifest = dict(groups=[dict(folder='group', baseline_status='passed', study_status='paused')],
                        valid_references=1, planned_references=7)
        self.assertEqual(batch.batch_status(manifest, 'controls'), 'controls_complete')
        manifest['groups'][0]['study_status'] = 'target_not_met'
        self.assertEqual(batch.batch_status(manifest, 'controls'), 'target_not_met')

    def test_baseline_requires_full_aligned_insertion_and_safe_final_withdrawal(self):
        case = load_plan(ROOT / 'experiments/forge_gap_pilot.json')['cases'][0]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'study.json'
            self.assertEqual(batch.baseline_status(path), 'not_run')
            ledger(path, [])
            self.assertEqual(batch.baseline_status(path), 'not_completed')
            for field, value, expected in (
                ('numerically_valid', False, 'numerical_review'),
                ('grasp_retained', False, 'aligned_insertion_review'),
                ('insertion_success', False, 'aligned_insertion_review'),
                ('force_budget_exceeded', True, 'aligned_budget_review'),
                ('torque_budget_exceeded', True, 'aligned_budget_review'),
            ):
                ledger(path, [attempt(case, **{field: value})])
                self.assertEqual(batch.baseline_status(path), expected)
            for safe in (False, None):
                record = attempt(case)
                record['final_retreat']['safe_recovery'] = safe
                ledger(path, [record])
                self.assertEqual(batch.baseline_status(path), 'aligned_retreat_review')
            ledger(path, [attempt(case)])
            self.assertEqual(batch.baseline_status(path), 'passed')

    def test_resume_rejects_changed_physics_and_live_coordinator_lock(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            batch.run(args)
            with self.assertRaisesRegex(ValueError, 'physics'):
                batch.run(args_for(directory, prepare_only=True, resume=True, physics_hz=240))
            with self.assertRaisesRegex(ValueError, 'Existing batch'):
                batch.run(args_for(directory, prepare_only=True))
            with (args.output_dir / '.batch.lock').open('a') as lock:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                with self.assertRaises(BlockingIOError):
                    batch.run(args_for(directory, prepare_only=True, resume=True))

    def test_merged_rows_keep_source_study_and_unknown_values(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            groups = [dict(folder='gap_0100'), dict(folder='gap_0507')]
            for group in groups:
                child = output / group['folder']
                child.mkdir()
                (child / 'recovery_probes.csv').write_text(
                    'trajectory_id,checkpoint_id,safe_recovery,max_wrist_force_n\n'
                    f'{group["folder"]}_case,first_stall,,\n')
            batch.merge_tables(output, groups)
            with (output / 'recovery_probes.csv').open() as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual({r['source_study'] for r in rows}, {g['folder'] for g in groups})
            self.assertTrue(all(r['safe_recovery'] == '' and r['max_wrist_force_n'] == '' for r in rows))

    def test_child_command_and_log_show_exact_resume_scope_without_running_simulator(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True, max_new_attempts=2)
            batch.run(args)
            output = args.output_dir.resolve()
            manifest = json.loads((output / 'batch.json').read_text())
            group = manifest['groups'][0]
            cases = load_plan(output / group['plan'])['cases']
            child = output / group['folder'] / 'study.json'
            ledger(child, [attempt(cases[0])])

            class FakeProcess:
                pid = 987654321

                def wait(self):
                    ledger(child, [attempt(c) for c in cases[:3]])
                    return 0

            with patch.object(batch.subprocess, 'Popen', return_value=FakeProcess()) as popen:
                batch.execute_group(args, output, group, 'perturbations', manifest)
            command = popen.call_args.args[0]
            self.assertIn('--resume', command)
            self.assertEqual(command[command.index('--case-plan') + 1], str(output / group['plan']))
            self.assertEqual(command[command.index('--output-dir') + 1], str(output / group['folder']))
            self.assertEqual(command[command.index('--max-new-attempts') + 1], '2')
            execution = manifest['executions'][-1]
            self.assertEqual(execution['status'], 'complete')
            self.assertEqual(execution['exit_code'], 0)
            self.assertTrue((output / execution['log']).is_file())
            self.assertEqual(manifest['valid_references'], 3)

    def test_child_launch_records_kernel_identity_for_later_resume(self):
        with tempfile.TemporaryDirectory() as directory:
            args = args_for(directory, prepare_only=True)
            batch.run(args)
            output = args.output_dir.resolve()
            manifest = json.loads((output / 'batch.json').read_text())
            group = manifest['groups'][0]
            observed = self.child_snapshot(self.running_manifest()['executions'][0])
            process = SimpleNamespace(pid=123, wait=lambda: 0)
            with patch.object(batch.subprocess, 'Popen', return_value=process), patch.object(batch, 'process_snapshot', return_value=observed):
                batch.execute_group(args, output, group, 'controls', manifest)
            identity = manifest['executions'][-1]['process_identity']
            self.assertEqual(identity['start_time_ticks'], 1000)
            self.assertEqual(identity['boot_id'], 'test-boot')


if __name__ == '__main__':
    unittest.main()
