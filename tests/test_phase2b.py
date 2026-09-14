import json
import hashlib
import tempfile
from dataclasses import asdict,replace
from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import unittest
from research.phase2b import make_case,command_misalignment,initial_plan,load_plan,recommend,path_outcome,analyze,comparison_sources
from research.forge_collection import next_case,validate_resume
from research.forge_protocol import protocol
from research.future_stall import stall_events,future_label,features,FEATURES


def outcome(severity,label,valid=True):
    return dict(make_case(5,'x_pitch',1,'combined',severity),status='complete',
        metrics={'numerically_valid':valid},checkpoints=[dict(checkpoint_kind='terminal',
        reached=label is not None,prefix_numerically_valid=valid,Y_R_tested=label)])


class Phase2BTests(unittest.TestCase):
    def test_ramp_aligned_entry_bounded_endpoint_and_signed_axes(self):
        for case in initial_plan()['cases']:
            for d in (-10,0,case['ramp_onset_mm']):
                self.assertEqual(set(command_misalignment(case,d).values()),{0.})
            end=command_misalignment(case,20)
            self.assertEqual(end,command_misalignment(case,100))
            middle=command_misalignment(case,(20+case['ramp_onset_mm'])/2)
            for k,v in end.items():self.assertAlmostEqual(middle[k],v/2)
        self.assertEqual(len(initial_plan()['cases']),24)
        self.assertEqual(len({c['path_group_id'] for c in initial_plan()['cases']}),24)

    def test_replay_retry_reuses_exact_path_and_group(self):
        plan=initial_plan(pilot=True);p=replace(protocol(),target_valid=2)
        a=next_case(p,[],plan)
        b=next_case(p,[dict(slot=0,retry=0,status='complete',metrics={'numerically_valid':False})],plan)
        for k in a:
            if k not in ('retry','trajectory_id'):self.assertEqual(a[k],b[k])
        self.assertEqual(b['retry'],1)
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'plan.json';path.write_text(json.dumps(plan));self.assertEqual(load_plan(path),plan)
            plan['cases'][0]['final_pitch_deg']=99;path.write_text(json.dumps(plan))
            with self.assertRaises(ValueError):load_plan(path)

    def test_unknowns_do_not_become_failures_or_trigger_escalation(self):
        for a in (outcome(3,None),outcome(3,0,False)):
            self.assertEqual(path_outcome(a),'unknown');self.assertEqual(recommend([a])[0],[])
        self.assertEqual(recommend([outcome(3,1)])[0][0]['severity_deg'],4.)
        self.assertEqual(recommend([outcome(6,1)])[0],[])
        self.assertEqual(recommend([outcome(3,1),outcome(4,None)])[0],[])

    def test_brackets_conflicting_repeats_and_nonmonotonicity(self):
        self.assertEqual(recommend([outcome(3,1),outcome(4,0)])[0][0]['severity_deg'],3.5)
        self.assertEqual(recommend([outcome(3,0)])[0][0]['severity_deg'],0.)
        self.assertEqual(recommend([outcome(3,1),outcome(3,0)])[0],[])
        self.assertEqual(recommend([outcome(3,0),outcome(4,1)])[0],[])
        self.assertEqual(recommend([outcome(3,1),outcome(3.25,0)])[0],[])

    def test_resume_checks_plan(self):
        saved=dict(study='Forge-Controlled-Phase2-v1',mode='collect',case_plan=initial_plan(True))
        current=dict(saved,case_plan=initial_plan())
        with self.assertRaisesRegex(ValueError,'case_plan'):validate_resume(saved,current)

    def test_cross_stage_transition_keeps_source_study_identity(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);studies=[]
            for stage in ('stage1','stage2'):
                d=root/stage;d.mkdir();studies.append(d)
                cps=[dict(reached=True,prefix_numerically_valid=True,Y_R_tested=label,
                          state=dict(depth_mm=depth,time_s=t,wrist_force_n=1.,wrist_torque_nm=.01))
                     for depth,t,label in ((5,1,1),(15,2,0))]
                a=dict(outcome(3,0),trajectory_id='b000_try00',folder='b000_try00',checkpoints=cps)
                m=dict(case_plan=initial_plan(True),protocol=asdict(protocol()),physics_hz=120,
                       sources={},stock_buffers=False,requested_checkpoints_mm=[5,10,15,20],attempts=[a])
                (d/'study.json').write_text(json.dumps(m))
            with redirect_stdout(StringIO()):analyze(studies,root/'analysis')
            result=json.loads((root/'analysis/boundary.json').read_text())
            self.assertEqual({r['source_study'] for r in result['transitions']},{str(d) for d in studies})
            self.assertEqual(len(result['transitions']),2)

    def test_stage_comparison_accepts_analysis_edits_but_rejects_runtime_changes(self):
        source=(Path(__file__).resolve().parents[1]/'research/phase2b.py').read_text()
        with tempfile.TemporaryDirectory() as tmp:
            d=Path(tmp);(d/'source').mkdir();path=d/'source/phase2b.py'
            def fingerprint(text):
                path.write_text(text)
                return comparison_sources({'sources':{'phase2b.py':hashlib.sha256(path.read_bytes()).hexdigest()}},d)
            original=fingerprint(source)
            report_edit=source.replace('def analyze(studies,output):','def analyze(studies,output):\n    "New report description."')
            self.assertEqual(original,fingerprint(report_edit))
            command_edit=source.replace('def command_misalignment(case,reached_depth):',
                'def command_misalignment(case,reached_depth):\n    reached_depth += 1')
            self.assertNotEqual(original,fingerprint(command_edit))
            with self.assertRaisesRegex(ValueError,'hash mismatch'):
                comparison_sources({'sources':{'phase2b.py':'wrong'}},d)


class FutureStallTests(unittest.TestCase):
    def test_confirmation_matches_trailing_window_not_initial_low_speed(self):
        p=dict(stall_window_s=.5,stall_progress_mm=.1,stall_command_progress_mm=.5)
        rows=[dict(time_s=i/10,phase='insert',command_depth_mm=i/5,depth_mm=min(i/5,1.)) for i in range(16)]
        events=stall_events(rows,p,10)
        self.assertEqual(events[0]['time_s'],1.)
        self.assertEqual(len(events),1)
        for r in rows:r['command_depth_mm']=0.
        self.assertEqual(stall_events(rows,p,10),[])

    def test_horizon_boundaries_censoring_and_invalidity(self):
        self.assertEqual(future_label(1,1,2,3),(1,'observed_future_stall'))
        self.assertEqual(future_label(1,.5,2,3),(0,'full_horizon_without_stall'))
        self.assertEqual(future_label(2,1,2,3),(None,'already_stalled'))
        self.assertEqual(future_label(2,2,None,3),(None,'right_censored'))
        self.assertEqual(future_label(1,1,2,3,False)[0],None)

    def test_features_never_read_future(self):
        rows=[dict.fromkeys(FEATURES,float(i)) for i in range(11)]
        for i,r in enumerate(rows):r['time_s']=i/10
        times=[r['time_s'] for r in rows]
        before=features(rows,times,5,.5)
        for r in rows[6:]:
            for k in FEATURES:r[k]=99999.
        self.assertEqual(before,features(rows,times,5,.5))
        self.assertEqual(before['history_end_row'],5)
        self.assertEqual(before['force_norm_n_max'],5.)
        self.assertIsNone(features(rows,times,2,.5))

if __name__=='__main__':unittest.main()
