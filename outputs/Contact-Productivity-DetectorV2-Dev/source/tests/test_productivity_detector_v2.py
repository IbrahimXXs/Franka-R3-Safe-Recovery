"""Causality, conservative triggers, disjoint groups and unchanged recovery execution."""
from dataclasses import asdict
from pathlib import Path
import json
import tempfile
import unittest
from unittest.mock import patch
from research.productivity_control import Design, Detector
from research.productivity_detector_v2 import DetectorV2, NORMAL_ETA, features, bind_detector
from research.productivity_detector_v2_design import conditions
from research.productivity_generalization import signature
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from tests.test_productivity_dewedge import runner, PegBench, SavedStream

CONFIG=dict(normal_eta_threshold=NORMAL_ETA,normal_consecutive_checks=2,
    urgent_eta_threshold=.1,urgent_deficit_acceleration_mm_s2=1.,terminal_rate_mm_s=.05)


def signal(eta,deficit):
    return dict(eta_raw=eta,normal_valid=True,deficit_rate_mm_s=deficit,terminal_rate_mm_s=None)


def history(phase='hold',depth=19.3):
    return [dict(time_s=i/120,phase=phase,segment=0,depth_mm=depth,
        command_depth_mm=20.,wrist_force_n=1.) for i in range(61)]


class TriggerTests(unittest.TestCase):
    def test_normal_trigger_is_exactly_original_two_checks(self):
        old=Detector('productivity',NORMAL_ETA,0.);new=DetectorV2(CONFIG)
        for i,e in enumerate((.8,NORMAL_ETA,.4,.3,.8,.4,.4)):
            s=signal(e,1.)
            self.assertEqual(old.check(i*.1,s),new.check(i*.1,s))
            self.assertEqual(old.count,new.count)
    def test_urgent_requires_low_eta_and_worsening_deficit(self):
        d=DetectorV2(CONFIG);self.assertFalse(d.check(0.,signal(.8,.1)))
        self.assertTrue(d.check(.1,signal(.05,.4)))
        self.assertEqual(d.count,1);self.assertEqual(d.last_decision['trigger_branch'],'urgent')
        self.assertAlmostEqual(d.last_decision['deficit_acceleration_mm_s2'],3.)
    def test_low_eta_without_worsening_does_not_get_single_check(self):
        d=DetectorV2(CONFIG);d.check(0.,signal(.8,.5))
        self.assertFalse(d.check(.1,signal(.05,.4)))
    def test_worsening_with_moderate_eta_does_not_get_single_check(self):
        d=DetectorV2(CONFIG);d.check(0.,signal(.8,.1))
        self.assertFalse(d.check(.1,signal(.3,1.)))
    def test_gaps_and_reset_invalidate_urgent_history(self):
        for gap in (True,False):
            d=DetectorV2(CONFIG);d.check(0.,signal(.8,.1))
            if not gap:d.reset()
            self.assertFalse(d.check(.2 if gap else .1,signal(.05,1.)))
    def test_terminal_needs_full_endpoint_hold_below_success(self):
        rows=history();s=features(rows,120,12.)
        self.assertIsNotNone(s);self.assertFalse(s['normal_valid'])
        d=DetectorV2(CONFIG);self.assertTrue(d.check(.5,s));self.assertEqual(d.last_decision['trigger_branch'],'terminal')
        for key,value in (('phase','insert'),('segment',1),('command_depth_mm',19.9),('depth_mm',19.5)):
            changed=[dict(r) for r in rows];changed[10][key]=value
            self.assertIsNone(features(changed,120,12.))
    def test_terminal_rejects_oscillation_with_zero_net_progress(self):
        rows=history();rows[30]['depth_mm']+=.1
        s=features(rows,120,12.);self.assertAlmostEqual(s['terminal_rate_mm_s'],.2)
        self.assertFalse(DetectorV2(CONFIG).check(.5,s))
    def test_recovery_and_short_history_are_ineligible(self):
        self.assertIsNone(features(history('unload_hold'),120,12.))
        self.assertIsNone(features(history()[:-1],120,12.))
    def test_privileged_load_is_never_read(self):
        class Guard(dict):
            def get(self,key,default=None):
                if key=='normal_load_n':raise AssertionError('privileged input')
                return super().get(key,default)
        self.assertEqual(features([Guard(r,normal_load_n=1e9) for r in history()],120,12.),features(history(),120,12.))
    def test_terminal_nonfinite_depth_is_ineligible(self):
        rows=history();rows[15]['depth_mm']=float('nan')
        self.assertIsNone(features(rows,120,12.))
    def test_threshold_retuning_of_normal_branch_rejected(self):
        with self.assertRaises(ValueError):DetectorV2(dict(CONFIG,normal_eta_threshold=.42))


class DesignTests(unittest.TestCase):
    def test_eighty_unique_paths_and_disjoint_sign_onset_groups(self):
        cc=conditions();self.assertEqual(len(cc),80)
        self.assertEqual(len({signature(c['case']) for c in cc}),80)
        cal=[c for c in cc if c['split']=='calibration'];val=[c for c in cc if c['split']=='validation']
        self.assertEqual((len(cal),len(val)),(60,20))
        self.assertFalse({c['group_id'] for c in cal}&{c['group_id'] for c in val})
        self.assertEqual(len({c['group_id'] for c in cal}),15)

    def test_calibration_excludes_validation_and_rejects_added_success_alerts(self):
        from research.productivity_detector_v2_calibration import calibrate
        from research.forge_protocol import protocol
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);cc=conditions();runs=[];success={};calids=[c['case_id'] for c in cc if c['split']=='calibration']
            for c in cc:
                cid=c['case_id'];ok=cid in calids[:30];success[cid]=ok
                runs.append(dict(policy='nominal',case_id=cid,log=cid+'.csv',insertion_success=ok))
                if c['split']=='calibration':(root/(cid+'.csv')).write_text('new calibration reference')
            plan=dict(cases=cc,calibration_grid=dict(urgent_eta_threshold=[.025,.05,.1,.15,.2],
                urgent_deficit_acceleration_mm_s2=[.1,1.],terminal_rate_mm_s=[.01,.1]),
                selection=dict(minimum_useful_lead_s=.1,max_additional_successful_trajectory_alerts=0),policy_source_sha256={})
            (root/'development_plan.json').write_text(json.dumps(plan))
            (root/'experiment.json').write_text(json.dumps(dict(protocol=asdict(protocol()),physics_hz=120,runs=runs)))
            paths=[]
            def read(path):
                self.assertIn(path.stem,calids);paths.append(path.stem)
                return [dict(time_s=1.,success=success[path.stem])]
            def checks(rows,*args):
                return [dict(time_s=0.,depth_mm=18.,safe=True,signal=signal(.8,.1)),
                    dict(time_s=.1,depth_mm=18.,safe=True,signal=signal(.15 if rows[0]['success'] else .05,.5))]
            with patch('research.productivity_detector_v2_calibration.read_log',side_effect=read),patch('research.productivity_detector_v2_calibration.nominal_checks',side_effect=checks):
                config=calibrate(root)
            self.assertEqual(set(paths),set(calids));self.assertEqual(config['urgent_eta_threshold'],.1)
            self.assertEqual(config['selected_calibration_metrics']['additional_success_alerts'],0)
            self.assertEqual(config['selected_calibration_metrics']['improved_failure_alerts'],30)
            self.assertFalse(config['validation_used_for_selection'])
            with self.assertRaises(FileExistsError):calibrate(root)


class ExecutionTests(unittest.TestCase):
    frozen=dict(design=asdict(Design()),eta_threshold=NORMAL_ETA,force_threshold_n=1.035449028015137)
    case=conditions()[20]['case']
    def execute(self,bench):
        original=runner();bound=bind_detector(original,CONFIG,19.5)
        self.assertIs(original.__code__,bound.__code__)
        for key in ('Dewedger','neutral_quaternion','quat_slerp','stalled_now','safety_reason'):
            self.assertIs(original.__globals__[key],bound.__globals__[key])
        return bound(bench,self.case,self.frozen,UnloadingDesign(),DewedgeDesign(),Path('unused'))
    def test_successful_control_no_interventions(self):
        r=self.execute(PegBench(ceiling=20));self.assertTrue(r['insertion_success']);self.assertEqual(r['intervention_count'],0)
    def test_terminal_trigger_uses_original_verified_recovery(self):
        class TrackingLagBench(PegBench):
            def tick(self,hp,hq):
                super().tick(hp,hq)
                self.depth=-10.+(self.depth+10.)*.983
                self.env.held_pos[:,2]=-self.depth/1000
                from tests.test_productivity_dewedge import apply
                self.env.fingertip_midpoint_pos=self.env.held_pos-apply(hq,self.grasp_p)
        r=self.execute(TrackingLagBench(ceiling=20.))
        starts=[row for row in SavedStream.latest.rows if row['intervention_started']]
        self.assertTrue(starts);self.assertEqual(starts[0]['trigger_branch'],'terminal')
        self.assertFalse(starts[0]['eta_valid']);self.assertIsNone(starts[0]['eta_raw'])
        for e in r['events']:
            self.assertEqual(e['unloading_status'],'verified')
            self.assertGreaterEqual(e['actual_retraction_mm'],.5)
            self.assertAlmostEqual(e['retry_ready_time_s']-e['verification_time_s'],.25)
    def test_recovery_safety_stop_still_has_priority(self):
        r=self.execute(PegBench(ceiling=19.3,unsafe_relax=True))
        self.assertEqual(r['outcome'],'grasp_retention_limit')
        self.assertNotIn('retry_started_s',r['events'][0])


if __name__=='__main__':unittest.main()
