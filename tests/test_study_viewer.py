"""Viewer data boundaries: invalid costs, missing samples, and safe embedding."""
import json
from pathlib import Path
import tempfile
import unittest
from visualization.view_study import build_dashboard, load_study


class StudyViewerTests(unittest.TestCase):
    def test_phase2b_terminal_profile_does_not_alias_fixed_depth(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);folder=p/'b000_try00';folder.mkdir()
            for name,force in (('d20_straight',1),('terminal_straight',2)):
                (folder/f'{name}_recovery.csv').write_text(f'time_s,recovery_time_s,phase,force_norm_n\n11,0,stop,{force}\n')
            probe=dict(policy='straight',replay_matched=True,label_eligible=True,numerically_valid=True)
            cps=[dict(depth_mm=20.,checkpoint_kind=kind,reached=True,prefix_numerically_valid=True,probes=[probe])
                 for kind in ('fixed_depth','terminal')]
            (p/'study.json').write_text(json.dumps(dict(study='Forge-Controlled-Phase2-v1',case_plan={'schema':'test'},
                attempts=[dict(folder=folder.name,family='offset_tilt',status='complete',split_group_id='path',
                    metrics={'numerically_valid':True},checkpoints=cps)])))
            trial=load_study(p)['trials'][0]
            self.assertEqual(trial['profiles']['d20_straight']['series']['force_norm_n'],[1.])
            self.assertEqual(trial['profiles']['terminal_straight']['series']['force_norm_n'],[2.])
            self.assertEqual(trial['summary']['split_group_id'],'path')

    def test_forge_recovery_profiles_preserve_unknown_replay_and_recorded_time(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);folder=p/'slot004_try00';folder.mkdir()
            data='time_s,recovery_time_s,phase,force_norm_n\n10,0,stop,2\n10.1,0.1,retreat,3\n'
            for name in ('final_retreat','d10_straight_recovery','d10_realign_recovery'):
                (folder/f'{name}.csv').write_text(data)
            probes=[dict(policy=policy,replay_matched=matched,label_eligible=True,numerically_valid=True,reason='cleared')
                    for policy,matched in (('straight',False),('realign',True))]
            (p/'study.json').write_text(json.dumps({'study':'Forge-Controlled-Phase2-v1','attempts':[
                dict(folder=folder.name,family='tilt_only',status='complete',metrics={'numerically_valid':True},
                     final_retreat={'numerically_valid':False,'label_eligible':False},
                     checkpoints=[dict(depth_mm=10,reached=True,prefix_numerically_valid=True,probes=probes)])]}))
            profiles=load_study(p)['trials'][0]['profiles']
            self.assertFalse(profiles['final_retreat']['eligible'])
            self.assertFalse(profiles['d10_straight']['eligible'])
            self.assertTrue(profiles['d10_realign']['eligible'])
            self.assertEqual(profiles['d10_realign']['series']['recovery_time_s'],[0.,.1])

    def test_forge_results_keep_provisional_warning_and_wrist_budget_context(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'study.json').write_text(json.dumps({'study':'Forge-Controlled-Phase2-v1',
                'protocol':{'force_budget_n':20.,'torque_budget_nm':1.},'attempts':[]}))
            data=load_study(p)
            self.assertIn('phase2',data)
            self.assertTrue(any('provisional' in w and 'wrist' in w for w in data['warnings']))

    def test_manifest_invalid_status_overrides_stale_summary(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'study.json').write_text(json.dumps({'status':'complete_with_invalid_trials',
                'invalid_trials':[{'scenario':'offset','recovery':'straight','status':'numerically_invalid',
                                   'recovery_resistive_work_j':None}]}))
            (p/'summary.csv').write_text('scenario,recovery,status,recovery_resistive_work_j\noffset,straight,cleared_within_budget,4\n')
            t=load_study(p)['trials'][0]
            self.assertFalse(t['eligible'])
            self.assertIsNone(t['summary']['recovery_resistive_work_j'])
            self.assertFalse(t['series'])

    def test_partial_csv_is_incomplete_and_nonfinite_stays_missing(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'study.json').write_text('{"status":"incomplete"}')
            (p/'tilted__straight.csv').write_text('time_s,phase,fz\n0.1,insert,NaN\n0.2,insert,3\n')
            t=load_study(p)['trials'][0]
            self.assertEqual(t['status'],'incomplete')
            self.assertFalse(t['eligible'])
            self.assertEqual(t['series']['fz'],[None,3.0])

    def test_script_text_cannot_escape_data_and_original_files_untouched(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);content=json.dumps({'status':'complete','note':'</script><script>alert(1)</script>@@APP@@'})
            (p/'study.json').write_text(content)
            out=build_dashboard(p)
            page=out.read_text()
            self.assertIn('\\u003c/script>',page)
            self.assertNotIn('</script><script>alert(1)',page)
            self.assertEqual((p/'study.json').read_text(),content)
            self.assertIn('@@APP@@',page)

    def test_invalid_sensor_study_never_eligible(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);(p/'study.json').write_text(json.dumps({'status':'invalid_sensor_configuration',
                'completed_trials':[{'scenario':'aligned','recovery':'straight','status':'cleared_within_budget'}]}))
            self.assertFalse(load_study(p)['trials'][0]['eligible'])

    def test_phase2_invalid_parent_cannot_color_checkpoint_as_safe(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            (p/'study.json').write_text(json.dumps({'study':'FR3-Phase2-v1','status':'target_not_met',
                'attempts':[{'family':'tilt_only','folder':'slot004_try00','status':'complete',
                    'metrics':{'numerically_valid':False,'max_force':30.},
                    'checkpoints':[{'depth_mm':10,'reached':True,'Y_R_tested':1,
                        'state':{'depth_mm':10.01,'force_norm_n':2.},'label_reason':'safe_policy_witness','probes':[]}]}]}))
            data=load_study(p)
            self.assertFalse(data['trials'][0]['eligible'])
            self.assertIsNone(data['phase2'][0]['Y_R_tested'])
            self.assertIn('Parent invalid',data['phase2'][0]['label_reason'])



if __name__=='__main__':
    unittest.main()
