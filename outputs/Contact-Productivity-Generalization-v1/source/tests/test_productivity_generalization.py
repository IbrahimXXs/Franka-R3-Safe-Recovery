"""Held-out coverage, frozen locks, policy replay and condition-level comparisons."""
from collections import Counter
from dataclasses import asdict
import copy
import json
from pathlib import Path
import tempfile
import unittest
from research.productivity_generalization import conditions,schedule,signature,verify_lock,sha,SCHEMA,POLICIES
from research.productivity_generalization_report import paired_effect,force_audit,detector_history,failure_class
from research.productivity_control import Design
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.phase2b import command_misalignment
from tests.test_productivity_control_execution import FakeBench,runner,MemoryStream

class DesignTests(unittest.TestCase):
    def test_unique_balanced_coverage_and_both_signs(self):
        cc=conditions();self.assertEqual(len(cc),48);self.assertEqual(len({signature(c['case']) for c in cc}),48)
        self.assertEqual(set(Counter(c['family'] for c in cc).values()),{8})
        self.assertEqual(Counter(c['ramp_onset_mm'] for c in cc),{7.:16,12.:16,15.:16})
        self.assertEqual(Counter(c['severity'] for c in cc),{'moderate':24,'severe':24})
        for key in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg'):
            self.assertTrue(any(c['case']['final_'+key]>0 for c in cc));self.assertTrue(any(c['case']['final_'+key]<0 for c in cc))
    def test_existing_depth_feedback_and_no_initial_misalignment(self):
        for c in conditions():
            case=c['case'];self.assertTrue(all(x==0 for x in command_misalignment(case,c['ramp_onset_mm']).values()))
            end=command_misalignment(case,20);half=command_misalignment(case,(20+c['ramp_onset_mm'])/2)
            for k in end:self.assertAlmostEqual(half[k],end[k]/2)
    def test_metadata_does_not_make_a_path_unseen(self):
        a=conditions()[0]['case'];b=dict(a,family='different_name',split_group_id='different_group')
        self.assertEqual(signature(a),signature(b));b['ramp_onset_mm']+=.5;self.assertNotEqual(signature(a),signature(b))
    def test_every_policy_once_per_condition_and_balanced_order(self):
        ss=schedule(conditions());self.assertEqual(len(ss),192)
        self.assertEqual(len({(s['case_id'],s['policy']) for s in ss}),192)
        self.assertEqual(set(Counter((s['policy'],s['policy_order']) for s in ss).values()),{12})
        self.assertEqual(ss,schedule(conditions()))
    def test_source_edit_rejected_without_rewriting_lock(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'prior').mkdir();(root/'prior/experiment.json').write_text('{}')
            source=root/'controller.py';source.write_text('frozen')
            c=conditions();value=dict(schema=SCHEMA,cases=c,schedule=schedule(c),design=asdict(Design()),
                unloading_design=asdict(UnloadingDesign()),dewedge_design=asdict(DewedgeDesign()),
                policy_source_sha256={'controller.py':sha(source)},holdout_audit={'source_sha256':{}},
                previous_experiment='prior',previous_manifest_sha256=sha(root/'prior/experiment.json'))
            self.assertTrue(verify_lock(root,value));source.write_text('changed')
            with self.assertRaises(ValueError):verify_lock(root,value)
    def test_design_edit_is_not_a_new_allowed_test_outcome(self):
        c=conditions();value=dict(schema=SCHEMA,cases=c,schedule=schedule(c),design=asdict(Design()),
            unloading_design=asdict(UnloadingDesign()),dewedge_design=asdict(DewedgeDesign()))
        value['cases'][0]['case']['final_offset_x_mm']+=.01
        from research.productivity_generalization import validate_plan
        with self.assertRaises(ValueError):validate_plan(value)

class StatisticsTests(unittest.TestCase):
    def test_paired_effect_uses_whole_condition_differences(self):
        rows=[dict(family=f,success_delta=1) for f in ('a','a','b','b')]
        r=paired_effect(rows,samples=100)
        self.assertEqual(r['conditions'],4);self.assertEqual(r['success_difference'],1);self.assertEqual(r['ci_low'],1);self.assertEqual(r['ci_high'],1)
        self.assertEqual(r['wins'],4);self.assertEqual(r['losses'],0)
    def test_no_matched_conditions_is_missing_not_zero_effect(self):
        self.assertIsNone(paired_effect([])['success_difference'])
    def test_budget_failure_and_no_trigger_are_not_stall_prevention(self):
        run=dict(insertion_success=False,outcome='unloading_budget_exhausted',policy='dewedge',stalled=False)
        self.assertEqual(failure_class(run,[],[],'unload_retract'),'unloading_not_ready_within_frozen_budget')
        run['outcome']='time_budget_exhausted'
        self.assertEqual(failure_class(run,[],[],'hold'),'no_eligible_detector_history')

class LogBench(FakeBench):
    def observe(self,phase,command):
        row=super().observe(phase,command)
        pose=list(self.env.fingertip_midpoint_pos[0])+list(self.env.fingertip_midpoint_quat[0])
        row.update({f'hand_pose_{i}':float(x) for i,x in enumerate(pose)})
        return row

class SavedStream(MemoryStream):
    latest=None
    def __init__(self,path):super().__init__(path);SavedStream.latest=self

class ReplayTests(unittest.TestCase):
    def execute(self,policy):
        frozen=dict(design=asdict(Design()),eta_threshold=.4212659765112803,force_threshold_n=1.035449028015137)
        case=conditions()[0]['case'];fn=runner();fn.__globals__['Stream']=SavedStream
        run=fn(LogBench(),case,policy,frozen,Path('unused'))
        return run,SavedStream.latest.rows,case,frozen
    def test_original_force_policy_replays_without_productivity_substitution(self):
        run,rows,case,cal=self.execute('force');r=force_audit(rows,run,case,cal,LogBench.protocol,120)
        self.assertEqual(r['interventions'],2)
        for row in rows:
            if row['intervention_started']:self.assertGreaterEqual(row['wrist_force_n'],cal['force_threshold_n'])
    def test_shadow_productivity_alarm_is_not_a_nominal_intervention(self):
        run,rows,case,cal=self.execute('nominal');checks=detector_history(rows,case,'nominal',cal,120)
        self.assertTrue(any(r['productivity_would_trigger'] for r in checks));self.assertEqual(run['intervention_count'],0)
        self.assertFalse(any(r['actual_intervention_started'] for r in checks))

if __name__=='__main__':unittest.main()
