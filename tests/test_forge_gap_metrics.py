"""Costs must preserve phase separation, missing data and failed recovery."""
import csv
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_gap_metrics import insertion_event_metrics, recovery_phase_metrics
from simulation.phase2_report import write_phase2_report, write_gap_batch_report


def sample(t, phase, depth=10., command=10., wrist=1., contact_z=-1.):
    return dict(time_s=t, recovery_time_s=t, phase=phase, depth_mm=depth,
                command_depth_mm=command, wrist_force_n=wrist, wrist_torque_nm=.01,
                wrist_force_world_2=-wrist, force_norm_n=abs(contact_z), fz=contact_z,
                contact_power_w=-.2)


class GapMetricsTests(unittest.TestCase):
    def test_stop_transient_does_not_become_retreat_peak(self):
        rows=[sample(0.,'stop',wrist=19.),sample(.05,'stop',wrist=18.),
              sample(.1,'retreat',depth=10.,command=5.,wrist=3.,contact_z=-2.),
              sample(.15,'retreat',depth=10.,command=0.,wrist=3.,contact_z=4.)]
        result=recovery_phase_metrics(rows,.05)
        self.assertEqual(result['stop_peak_wrist_force_n'],19.)
        self.assertEqual(result['retreat_peak_wrist_force_n'],3.)
        self.assertEqual(result['retreat_peak_contact_downward_resistance_n'],2.)
        self.assertEqual(result['retreat_peak_wrist_world_z_abs_n'],3.)
        self.assertEqual(result['retreat_actual_withdrawal_mm'],0.)
        self.assertEqual(result['retreat_commanded_withdrawal_mm'],10.)
        self.assertAlmostEqual(result['retreat_contact_resistive_work_j'],.02)
        self.assertNotIn('safe_recovery',result)

    def test_complete_windows_and_absent_phase_are_not_zero_cost(self):
        rows=[sample(0.,'stop'),sample(.05,'stop',wrist=10.),sample(.1,'retreat',wrist=2.),
              sample(.15,'retreat',wrist=4.)]
        result=recovery_phase_metrics(rows,.05)
        self.assertIsNone(result['stop_peak_wrist_force_mean100ms_n'])
        self.assertEqual(result['retreat_peak_wrist_force_mean100ms_n'],3.)
        self.assertIsNone(result['realign_peak_wrist_force_n'])
        self.assertFalse(result['realign_observed'])

    def test_missing_measurement_or_timestamp_gap_breaks_window(self):
        rows=[sample(0.,'stop'),sample(.05,'retreat',wrist=10.),sample(.15,'retreat',wrist=4.)]
        result=recovery_phase_metrics(rows,.05)
        self.assertIsNone(result['retreat_peak_wrist_force_mean100ms_n'])
        del rows[-1]['wrist_force_world_2']
        self.assertIsNone(recovery_phase_metrics(rows,.05)['retreat_peak_wrist_world_z_abs_mean100ms_n'])

    def test_unreached_trigger_and_incomplete_ramp_remain_distinct(self):
        case={'tilt_amplitude_deg':2.,'tilt_onset_mm':10.}
        rows=[dict(phase='insert',time_s=1.,depth_mm=9.,tilt_deg=.1,command_tilt_deg=0.)]
        result=insertion_event_metrics(rows,case)
        self.assertFalse(result['tilt_triggered'])
        self.assertFalse(result['tilt_ramp_completed'])
        rows += [dict(phase='insert',time_s=2.,depth_mm=10.,tilt_deg=.1,command_tilt_deg=0.,
                      tilt_triggered=True,tilt_trigger_time_s=2.,tilt_ramp_fraction=0.),
                 dict(phase='hold',time_s=2.5,depth_mm=10.,tilt_deg=.4,command_tilt_deg=1.,
                      tilt_triggered=True,tilt_trigger_time_s=2.,tilt_ramp_fraction=.5)]
        result=insertion_event_metrics(rows,case)
        self.assertTrue(result['tilt_triggered'])
        self.assertFalse(result['tilt_ramp_completed'])
        self.assertEqual(result['tilt_trigger_time_s'],2.)
        self.assertEqual(result['terminal_actual_tilt_deg'],.4)
        self.assertEqual(result['terminal_command_tilt_deg'],1.)

    def test_zero_angle_is_control_not_trigger_failure(self):
        result=insertion_event_metrics([],{'tilt_amplitude_deg':0.})
        self.assertFalse(result['tilt_applicable'])
        self.assertIsNone(result['terminal_actual_tilt_deg'])

    def test_report_retains_failed_and_missing_policy_pairs(self):
        state=dict(depth_mm=10.,force_norm_n=2.,tilt_deg=1.,wrist_force_n=3.)
        failed=dict(policy='straight',depth_mm=10.,label_eligible=True,replay_matched=True,
                    safe_recovery=False,reason='operational_budget_exceeded',max_force=5.,
                    max_wrist_force_n=21.,recovery_censored=True,termination_reason='operational_budget_exceeded',
                    retreat_peak_contact_downward_resistance_n=5.)
        cp=dict(checkpoint_id='case0:first_stall',depth_mm=10.,checkpoint_kind='first_stall',
                reached=True,prefix_numerically_valid=True,state=state,probes=[failed],
                Y_R_tested=None,label_reason='unresolved_probe_or_replay')
        case=dict(trajectory_id='case0',case_id='case0',slot=0,retry=0,family='tilt_only',
                  offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=1.,insertion_duration_s=8.,
                  radial_clearance_mm=.1,tilt_amplitude_deg=1.,tilt_onset_fraction=.5)
        attempt=dict(case,folder='case0',status='complete',checkpoints=[cp],
                     metrics=dict(numerically_valid=True,insertion_success=False),
                     final_retreat=dict(safe_recovery=True,max_wrist_force_n=.05,
                                        retreat_peak_wrist_force_n=.01727,recovery_censored=False))
        manifest=dict(study='Forge-Controlled-Phase2-v1',status='paused',mode='collect',
                      protocol={'force_budget_n':20.,'target_valid':1},attempts=[attempt],
                      case_plan={'schema':'Forge-gap-tilt-v1','cases':[case]},pilot_candidate_count=1)
        with tempfile.TemporaryDirectory() as temp:
            write_phase2_report(temp,manifest)
            directory=Path(temp)
            with (directory/'comparison.csv').open() as stream:
                rows=list(csv.DictReader(stream))
            self.assertEqual(len(rows),1)
            self.assertEqual(rows[0]['straight_safe_recovery'],'False')
            self.assertEqual(rows[0]['straight_max_wrist_force_n'],'21.0')
            self.assertEqual(rows[0]['straight_recovery_censored'],'True')
            self.assertEqual(rows[0]['straight_termination_reason'],'operational_budget_exceeded')
            self.assertEqual(rows[0]['realign_probe_present'],'False')
            self.assertEqual(rows[0]['realign_max_wrist_force_n'],'')
            self.assertEqual(rows[0]['Y_R_tested'],'')
            with (directory/'trajectories.csv').open() as stream:
                trajectory=next(csv.DictReader(stream))
            self.assertEqual(trajectory['continuation_straight_retreat_peak_wrist_force_n'],'0.01727')
            self.assertEqual(trajectory['continuation_straight_origin'],'original_reference_continuation_without_replay')
            self.assertTrue((directory/'cases.csv').exists())
            self.assertTrue((directory/'gap_recovery_comparison.png').exists())
            self.assertIn('including failed probes',(directory/'report.md').read_text())

    def test_batch_merge_preserves_local_slot_zero_as_two_cases(self):
        cases=[]
        with tempfile.TemporaryDirectory() as temp:
            directory=Path(temp)
            for name,gap in [('wide',.507),('tight',.1)]:
                case=dict(trajectory_id=name,case_id=name,slot=0,retry=0,family='centered',
                          offset_x_mm=0.,offset_y_mm=0.,roll_deg=0.,pitch_deg=0.,insertion_duration_s=8.,
                          radial_clearance_mm=gap,tilt_amplitude_deg=0.,tilt_onset_fraction=None)
                cases.append(case)
                attempt=dict(case,folder=name,status='complete',checkpoints=[],
                             metrics=dict(numerically_valid=True,insertion_success=True))
                child=dict(study='Forge-Controlled-Phase2-v1',status='complete',mode='collect',
                           protocol={'force_budget_n':20.,'target_valid':1},attempts=[attempt])
                (directory/name).mkdir()
                (directory/name/'study.json').write_text(json.dumps(child))
            plan={'schema':'Forge-gap-tilt-v1','cases':cases}
            merged=write_gap_batch_report(directory,plan,['wide','tight'],status='complete')
            self.assertEqual([a['slot'] for a in merged['attempts']],[0,1])
            self.assertEqual([a['folder'] for a in merged['attempts']],['wide/wide','tight/tight'])
            self.assertIn('2 / 2',(directory/'report.md').read_text())
            self.assertTrue((directory/'aggregate_study.json').exists())


if __name__=='__main__':unittest.main()
