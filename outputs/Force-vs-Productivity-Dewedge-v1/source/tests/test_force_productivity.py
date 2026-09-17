"""Force-only causality, fair recovery binding, calibration isolation and fresh paths."""
from collections import Counter
from dataclasses import asdict
from pathlib import Path
import tempfile
import unittest
from research.force_productivity import ForceDetector,force_features,bind_force
from research.force_productivity_design import conditions,prior_paths,schedule
from research.force_productivity_analysis import select_candidate,paired_statistics
from research.productivity_detector_v2_generalization import path_signature
from research.productivity_detector_v2 import bind_detector,NORMAL_ETA
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from tests.test_productivity_dewedge import runner,PegBench,SavedStream


def signal(force,eligible=True):return dict(wrist_force_n=force,force_eligible=eligible,normal_valid=False)
def history(phase='hold'):
    return [dict(time_s=i/120,phase=phase,segment=0,depth_mm=19.2,command_depth_mm=20.,wrist_force_n=3.) for i in range(61)]

class ForceTests(unittest.TestCase):
    def test_strict_threshold_two_adjacent_checks(self):
        d=ForceDetector(2.)
        self.assertFalse(d.check(0,signal(2.)));self.assertFalse(d.check(.1,signal(2.1)))
        self.assertTrue(d.check(.2,signal(2.1)));self.assertEqual(d.count,2)
    def test_spikes_gaps_invalid_windows_and_reset_break_persistence(self):
        for mode in ('gap','invalid','reset','below'):
            d=ForceDetector(2.);d.check(0,signal(3.))
            if mode=='reset':d.reset()
            elif mode=='invalid':d.check(.1,signal(3.,False))
            elif mode=='below':d.check(.1,signal(1.))
            self.assertFalse(d.check(.1 if mode=='reset' else .2,signal(3.)))
    def test_hold_does_not_require_positive_commanded_progress(self):
        f=force_features(history(),120,10)
        self.assertTrue(f['force_eligible']);self.assertFalse(f['normal_valid']);self.assertIsNone(f['eta_raw'])
        d=ForceDetector(2.);d.check(.5,f);self.assertTrue(d.check(.6,f))
    def test_full_single_phase_single_segment_and_contact_gate(self):
        for mode in ('short','mixed','segment','precontact'):
            rows=history()
            if mode=='short':rows=rows[1:]
            elif mode=='mixed':rows[0]['phase']='insert'
            elif mode=='segment':rows[0]['segment']=1
            else:rows[0]['depth_mm']=10.1
            self.assertIsNone(force_features(rows,120,10),mode)
    def test_force_decision_ignores_productivity_and_privileged_load(self):
        class Restricted(dict):
            def __getitem__(self,key):
                if key not in ('wrist_force_n','force_eligible','normal_valid'):raise AssertionError('Forbidden predictor '+key)
                return super().__getitem__(key)
        d=ForceDetector(2.);s=Restricted(signal(3.),eta_raw=-100.,normal_load_n=1e9)
        d.check(0,s);self.assertTrue(d.check(.1,s))
        class NoLoad(dict):
            def __getitem__(self,key):
                if key=='normal_load_n':raise AssertionError('Privileged load read')
                return super().__getitem__(key)
        self.assertTrue(force_features([NoLoad(r) for r in history()],120,10)['force_eligible'])
    def test_same_exact_recovery_code_and_only_detector_bindings_differ(self):
        original=runner();force=bind_force(original,2.);prod=bind_detector(original,dict(normal_eta_threshold=NORMAL_ETA,
            normal_consecutive_checks=2,urgent_eta_threshold=.2,urgent_deficit_acceleration_mm_s2=2.,terminal_rate_mm_s=.01),19.5)
        self.assertIs(force.__code__,prod.__code__);self.assertIs(force.__code__,original.__code__)
        for bound in (force,prod):
            changed={k for k in bound.__globals__ if bound.__globals__[k] is not original.__globals__.get(k)}
            self.assertEqual(changed,{'Detector','recent_signal','Stream'})
    def test_force_calls_verified_dewedge_not_old_fixed_retract(self):
        frozen=dict(design=asdict(Design()),eta_threshold=NORMAL_ETA,force_threshold_n=1.035449028015137)
        case=conditions('development')[0]['case']
        result=bind_force(runner(),1.)(PegBench(),case,frozen,UnloadingDesign(),DewedgeDesign(),Path('unused'))
        self.assertGreater(result['intervention_count'],0)
        for event in result['events']:
            if event['unloading_status']=='verified':
                self.assertGreaterEqual(event['actual_retraction_mm'],.5)
                self.assertAlmostEqual(event['retry_ready_time_s']-event['verification_time_s'],.25)
                self.assertIn('neutral_peg_position',event)
        self.assertTrue(any(r['phase']=='relax' for r in SavedStream.latest.rows))
        self.assertTrue(any(r['unloading_verified'] for r in SavedStream.latest.rows))

class DesignSelectionTests(unittest.TestCase):
    def test_all_paths_fresh_disjoint_and_previous_final64_included(self):
        root=Path(__file__).resolve().parents[1];old,audit=prior_paths(root)
        dev={path_signature(c['case']) for c in conditions('development')};test={path_signature(c['case']) for c in conditions('held_out')}
        self.assertEqual((len(dev),len(test)),(32,64));self.assertFalse(dev&test);self.assertFalse((dev|test)&old)
        self.assertEqual(audit['descriptor_counts']['outputs/Contact-Productivity-DetectorV2-Generalization-v1/experiment.json'],64)
        self.assertEqual(Counter(c['severity'] for c in conditions('held_out')),dict(easy=8,moderate=28,severe=28))
    def test_every_candidate_and_policy_has_full_paired_plan(self):
        s=schedule();self.assertEqual(len(s),384)
        self.assertTrue(all(x['split']=='development' for x in s[:192]));self.assertTrue(all(x['split']=='held_out' for x in s[192:]))
        self.assertEqual(Counter(x['detector_id'] for x in s[:192]),dict(nominal=32,force_1=32,force_2=32,force_3=32,force_4=32,force_5=32))
        self.assertEqual(Counter(x['policy'] for x in s[192:]),dict(nominal=64,force=64,productivity=64))
        self.assertEqual(s,schedule())
    def candidates(self):
        return [dict(threshold_n=t,admissible=t>1,false_alerts=5 if t==1 else 0,
            useful_failed_cases_alerted=10-t,failed_cases_alerted=15-t,capped_useful_lead_score_s=1.,closed_loop_successes=t) for t in range(1,6)]
    def test_false_alert_constraint_then_useful_coverage(self):
        self.assertEqual(select_candidate(self.candidates())['threshold_n'],2)
    def test_selection_ignores_closed_loop_success(self):
        rows=self.candidates();rows[-1]['closed_loop_successes']=99999
        self.assertEqual(select_candidate(rows)['threshold_n'],2)
    def test_no_admissible_fallback_minimizes_false_alerts(self):
        rows=self.candidates()
        for r in rows:r['admissible']=False;r['false_alerts']=6-r['threshold_n']
        self.assertEqual(select_candidate(rows)['threshold_n'],5)
    def test_ties_choose_conservative_threshold(self):
        rows=self.candidates()[1:]
        for r in rows:r['useful_failed_cases_alerted']=0;r['failed_cases_alerted']=0
        self.assertEqual(select_candidate(rows)['threshold_n'],5)
    def test_paired_success_test_and_subset_include_failures(self):
        rows=[dict(case_id=str(i),family='combined',severity='severe',prefix_matched=i<4,
            force_success=False,productivity_success=True,success_delta=1) for i in range(6)]
        rows.append(dict(case_id='fail',family='combined',severity='moderate',prefix_matched=True,
            force_success=False,productivity_success=False,success_delta=0))
        a=paired_statistics(rows);self.assertEqual(a['conditions'],7);self.assertEqual(a['both_fail'],1)
        self.assertEqual(a['exact_mcnemar_p'],.03125)
        self.assertEqual(paired_statistics(rows,'strict_matched')['conditions'],5)
        self.assertEqual(paired_statistics(rows,'moderate')['both_fail'],1)
        self.assertEqual(paired_statistics([])['paired_success_difference'],None)

class ReportTests(unittest.TestCase):
    def test_empty_matched_subset_and_no_alerts_report(self):
        from research.force_productivity_report import totals,report,first_order
        self.assertEqual(first_order(None,None),'neither');self.assertEqual(first_order(1.,None),'force_only')
        self.assertEqual(first_order(None,1.),'productivity_only');self.assertEqual(first_order(1.,1.),'same_time')
        self.assertEqual(first_order(1.,2.),'force_first')
        pair=dict(case_id='fixture',family='combined',severity='severe',nominal_success=True,force_success=True,
            productivity_success=True,force_outcome='insertion_success',productivity_outcome='insertion_success',success_delta=0,
            prefix_matched=False,force_shadow_triggered=False,productivity_shadow_triggered=False,shadow_trigger_order='neither')
        keys=('safety_stop grasp_retention_failure terminal_timeout stalled safety_before_any_recovery safety_during_recovery '
            'intervention_count verified_unloadings ready_unloadings retries retry_successes repeated_stalls').split()
        summaries=[dict(case_id='fixture',policy=k,insertion_success=True,final_depth_mm=20.,insertion_time_s=8.,**dict.fromkeys(keys,0))
            for k in ('nominal','force','productivity')]
        pp=totals(summaries,[],[pair]);ss=[paired_statistics([pair],k) for k in ('all','moderate','severe','strict_matched')]
        config=dict(threshold_n=2.,false_alert_cap_met=True,frozen_at_utc='fixture')
        plan=dict(force_eligibility='fixture',force_decision='fixture')
        validation=dict(strict_preintervention_matches=0,force_detector_config_sha256='fixture',physics_rows=1,
            causal_checks=0,interventions=0,source_archives_verified=1,preexisting_outputs_checked=1)
        with tempfile.TemporaryDirectory() as tmp:
            report(Path(tmp),plan,config,pp,ss,[pair],[],[],[],validation)
            self.assertIn('N/A',(Path(tmp)/'report.md').read_text())


if __name__=='__main__':unittest.main()
