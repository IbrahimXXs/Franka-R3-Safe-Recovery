"""Checks that recovery costs cannot hide stalled or failed motion."""
import unittest
from simulation.study_report import summarize, invalid_trial, write_report
import csv
import tempfile
from pathlib import Path


def row(time, phase, depth, force=0., power=0., clear=False):
    return dict(time_s=time, phase=phase, depth_mm=depth, tilt_deg=0.,
        lowest_peg_z_m=.870 if clear else .845, normal_load_n=abs(force),
        force_norm_n=abs(force), torque_norm_nm=0., fz=force,
        contact_power_w=power, min_separation_mm=0.)


class RecoveryMetricsTests(unittest.TestCase):
    def test_stall_has_zero_work_but_is_not_recovered(self):
        rows = [row(.1, 'baseline', -25), row(.2, 'insert', 20, 5)]
        rows += [row(t, 'retreat', 20, -30) for t in (.3, .4, .5)]
        result = summarize(rows, 'stall', 'straight', 20, 1, .1)
        self.assertEqual(result['recovery_resistive_work_j'], 0.)
        self.assertEqual(result['retreat_peak_resistance_n'], 30.)
        self.assertEqual(result['status'], 'not_cleared')
        self.assertIsNone(result['time_to_clear_s'])
        self.assertFalse(result['within_budget'])

    def test_force_sign_work_and_sustained_clearance(self):
        rows = [row(.1, 'baseline', -25), row(.2, 'insert', 20, 8),
                row(.3, 'retreat', 10, -4, -.02), row(.4, 'retreat', 5, 2, .01),
                row(.5, 'retreat', -5, clear=True), row(.6, 'clear_hold', -5, clear=True)]
        result = summarize(rows, 'clear', 'straight', 20, 1, .1)
        self.assertEqual(result['insertion_peak_resistance_n'], 8)
        self.assertEqual(result['retreat_peak_resistance_n'], 4)
        self.assertAlmostEqual(result['recovery_resistive_work_j'], .002)
        self.assertAlmostEqual(result['retreat_resistance_impulse_ns'], .4)
        self.assertAlmostEqual(result['time_to_clear_s'], .4)
        self.assertEqual(result['status'], 'cleared_within_budget')

    def test_rim_blockage_is_not_an_inserted_jam(self):
        rows = [row(.1, 'baseline', -25), row(.2, 'insert', -.1, 20),
                row(.3, 'retreat', -5, clear=True), row(.4, 'clear_hold', -5, clear=True)]
        result = summarize(rows, 'rim', 'straight', 20, 1, .1)
        self.assertEqual(result['status'], 'did_not_enter')

    def test_realignment_work_is_part_of_recovery_cost(self):
        rows = [row(.1, 'baseline', -25), row(.2, 'insert', 20),
                row(.3, 'realign', 20, power=-2),
                row(.4, 'retreat', -5, clear=True), row(.5, 'clear_hold', -5, clear=True)]
        result = summarize(rows, 'rotation', 'realign', 20, 1, .1)
        self.assertAlmostEqual(result['recovery_resistive_work_j'], .2)
        self.assertAlmostEqual(result['time_to_clear_s'], .3)

    def test_invalid_only_report_has_blank_costs_and_no_success(self):
        failed = row(3.875, 'insert', 1., 188.103)
        failed['min_separation_mm'] = -1.3121
        result = invalid_trial('offset', 'straight', failed, 'overlap guard')
        with tempfile.TemporaryDirectory() as directory:
            write_report(directory, [], [result])
            with (Path(directory)/'summary.csv').open() as stream:
                records = list(csv.DictReader(stream))
            self.assertEqual(records[0]['status'], 'numerically_invalid')
            self.assertEqual(records[0]['recovery_resistive_work_j'], '')
            self.assertEqual(records[0]['retreat_peak_resistance_n'], '')
            self.assertEqual(records[0]['aborted_separation_mm'], '-1.3121')
            report = (Path(directory)/'report.md').read_text()
            self.assertIn('excluded from the plots', report)
            self.assertNotIn('cleared_within_budget', report)



if __name__ == '__main__':
    unittest.main()
