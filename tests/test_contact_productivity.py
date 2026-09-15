"""Focused target, causal-feature, grouping and pre-stall warning tests."""
from dataclasses import replace
import unittest
import numpy as np
from research.contact_productivity import (Settings,productivity,history_features,feature_sets,
    group_for,role_for,folds,weights,ridge_fit,ridge_predict,alarm_threshold,sustained_score,warning_results)


def fake_rows(n=122):
    result=[]
    for i in range(n):
        t=i/120
        r=dict(time_s=t,phase='insert',depth_mm=10+t,command_depth_mm=10+2*t,
               tip_x_mm=.1,tip_y_mm=.2,qw=0.,qx=0.,qy=0.,qz=1.,
               normal_load_n=10000.,wrist_force_n=1+t,wrist_torque_nm=.1+t/10,
               force_norm_n=2+t,torque_norm_nm=.2+t/10)
        for k in ['vx','vy','vz','omegax','omegay','omegaz','fx','fy','fz','taux','tauy','tauz',
                  *[f'wrist_force_world_{j}' for j in range(3)],*[f'wrist_torque_about_peg_base_world_{j}' for j in range(3)]]:r[k]=t
        result.append(r)
    return result


class ContactProductivityTests(unittest.TestCase):
    def test_ratio_is_signed_unclipped_and_denominator_is_positive(self):
        for actual,expected in (([0,2],2),([0,-.5],-.5),([0,.25],.25)):
            eta,da,dc=productivity(actual,[0,1],0,1)
            self.assertEqual(eta,expected);self.assertEqual(dc,1)
        for command in ([0,0],[0,-1],[0,.099]):self.assertIsNone(productivity([0,1],command,0,1)[0])
        self.assertEqual(productivity([0,.2],[0,.1],0,1)[0],2)

    def test_features_ignore_future_and_privileged_load(self):
        rows=fake_rows();before=history_features(rows,0,60)
        for i,r in enumerate(rows):
            r['normal_load_n']=-100000.
            if i>60:
                for k in r:
                    if k!='phase':r[k]=999999.
        self.assertEqual(before,history_features(rows,0,60))
        sets=feature_sets(before)
        self.assertTrue(all('normal_load' not in k for keys in sets.values() for k in keys))
        self.assertTrue(all(not k.startswith('contact_') for k in sets['progress_wrench_motion']))
        self.assertAlmostEqual(before['current_eta'],.5)
        self.assertAlmostEqual(before['actual_rate_mm_s'],1)
        self.assertAlmostEqual(before['command_rate_mm_s'],2)
        self.assertAlmostEqual(before['wrist_fx_slope'],1)

    def test_orientation_is_invariant_to_quaternion_sign(self):
        rows=fake_rows();a=history_features(rows,0,60)
        for r in rows:
            for k in ('qw','qx','qy','qz'):r[k]*=-1
        b=history_features(rows,0,60)
        for k in a:self.assertAlmostEqual(a[k],b[k])

    def test_shared_stage4_paths_are_quarantined(self):
        a=dict(family='tilt_only',path_group_id='path1')
        g=group_for('Forge-Phase2B-Stage3',a)
        self.assertEqual(g,group_for('Forge-Phase2B-Stage4',a))
        self.assertEqual(role_for('Forge-Phase2B-Stage3',g,{g}),'quarantined_shared_path')
        self.assertEqual(role_for('Forge-Phase2B-Stage4',g,{g}),'stage4')
        self.assertEqual(group_for('Forge-Phase2-100',dict(family='centered')), 'Phase2A:centered_controls')

    def test_folds_never_split_a_group_and_duplicate_weights_balance(self):
        g=np.array(['a']*10+['b']*2+['c']*3+['d']*4)
        f=folds(g,g=='a',3,42)
        for group in set(g):self.assertEqual(len(set(f[g==group])),1)
        w=weights(g)
        for group in set(g):self.assertAlmostEqual(w[g==group].sum(),.25)

    def test_ridge_transforms_are_training_only_and_raw_predictions_unclipped(self):
        x=np.arange(20.)[:,None];y=2*x[:,0]-3;g=np.array([str(i//2) for i in range(20)])
        model=ridge_fit(x,y,g,1e-8);mu=model['mean'].copy()
        result=ridge_predict(model,np.array([[100.],[-100.]]))
        self.assertGreater(result[0],1);self.assertLess(result[1],0)
        np.testing.assert_equal(model['mean'],mu)
        np.testing.assert_allclose(ridge_predict(model,x),y,atol=1e-5)

    def test_alarm_requires_adjacent_windows_and_uses_second_timestamp(self):
        rr=[dict(time_s=t) for t in (1.,1.1,1.4,1.5)]
        seq=sustained_score(rr,[5,4,1,6])
        self.assertEqual(len(seq),2);self.assertEqual(seq[0],(1.1,4))
        self.assertEqual(seq[1],(1.5,1))

    def test_force_calibration_cannot_count_a_single_spike(self):
        rr=[dict(trajectory_id='a',group_id='a',stalled=False,time_s=i*.1) for i in range(3)]
        threshold=alarm_threshold(rr,[100,0,100],.1)
        self.assertGreater(threshold,0);self.assertLess(threshold,1)

    def test_warning_lead_is_confirmation_not_backdated_onset(self):
        rr=[]
        for t in (1.,1.1,1.2):
            rr.append(dict(evaluation='stage4',horizon_s=.5,model='current_progress',trajectory_id='a',
                group_id='a',time_s=t,stalled=True,stall_confirmation_s=1.5,stall_lookback_start_s=1.,
                alarm_score=-.2,calibrated_alarm_score_threshold=-.5))
        detail,summary=warning_results(rr,Settings())
        d=next(r for r in detail if r['mode']=='semantic_eta_0.5')
        self.assertAlmostEqual(d['lead_to_confirmation_s'],.4)
        self.assertAlmostEqual(d['lead_to_lookback_start_s'],-.1)
        self.assertFalse(d['detected_before_lookback'])

    def test_no_eligible_windows_is_a_miss_not_removed_denominator(self):
        rr=[dict(evaluation='stage4',horizon_s=.5,model='current_progress',trajectory_id='a',group_id='a',time_s=1.,
                 stalled=False,stall_confirmation_s=None,stall_lookback_start_s=None,alarm_score=-1.,calibrated_alarm_score_threshold=-.5)]
        summaries=[dict(role='stage4',numerically_valid=True,trajectory_id='b',group_id='b',stalled=True,stall_confirmation_s=2.)]
        _,summary=warning_results(rr,Settings(),summaries)
        d=next(r for r in summary if r['mode']=='semantic_eta_0.5')
        self.assertEqual(d['stalled_trajectories'],1);self.assertEqual(d['detected_before_confirmation'],0)
        self.assertEqual(d['trajectories_without_eligible_windows'],1)


if __name__=='__main__':unittest.main()
