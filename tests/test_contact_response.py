"""Focused checks for actual-motion contact-response math and inference boundaries."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest
import numpy as np
from research.contact_response import (Settings,quat_increment,transport_wrench,increments,
    ridge_fit,local_model,excitation,model_scales,auc,window_analysis,discover,load_reference)


def qaxis(axis,angle):
    q=np.zeros(4);q[0]=np.cos(angle/2);q[axis+1]=np.sin(angle/2);return q


def multiply(a,b):
    return np.r_[a[0]*b[0]-a[1:]@b[1:],a[0]*b[1:]+b[0]*a[1:]+np.cross(a[1:],b[1:])]


class ContactResponseTests(unittest.TestCase):
    def test_quaternion_sign_normalization_and_world_increment(self):
        previous=qaxis(2,np.pi);change=qaxis(0,.002);following=multiply(change,previous)
        np.testing.assert_allclose(quat_increment(previous,following),[.002,0,0],atol=1e-12)
        np.testing.assert_allclose(quat_increment(-2*previous,3*following),[.002,0,0],atol=1e-12)
        np.testing.assert_allclose(quat_increment(previous,-previous),[0,0,0],atol=1e-12)
        with self.assertRaises(ValueError):quat_increment(np.zeros(4),previous)

    def test_quaternion_wrap_uses_shortest_rotation(self):
        result=quat_increment(qaxis(2,np.deg2rad(179)),qaxis(2,np.deg2rad(-179)))
        np.testing.assert_allclose(result,[0,0,np.deg2rad(2)],atol=1e-12)

    def test_wrench_transport_removes_moving_origin_artifact(self):
        p=np.array([[0.,0.,0.],[.001,0,0],[.002,0,0]])
        force=np.tile([0.,4.,0.],(3,1));contact=np.array([.01,.02,.03])
        torque=np.cross(contact-p,force);w=np.column_stack((force,torque))
        corrected=transport_wrench(p,w,p[0])
        np.testing.assert_allclose(corrected,np.tile(corrected[0],(3,1)),atol=1e-14)
        self.assertGreater(np.linalg.norm(np.diff(w,axis=0)),0.)
        np.testing.assert_allclose(w[:,3:],torque)  # no in-place mutation

    def test_increments_use_pose_and_disjoint_lag_pairs(self):
        p=np.zeros((7,3));p[:,2]=np.arange(7)*.001
        q=np.tile([1.,0,0,0],(7,1));w=np.zeros((7,6));w[:,2]=np.arange(7)*2
        x,y,idx=increments(p,q,w,2)
        np.testing.assert_array_equal(idx,[0,2,4,6])
        np.testing.assert_allclose(x[:,2],.002);np.testing.assert_allclose(y[:,2],4.)

    def test_known_full_rank_map_is_recovered_in_si_units(self):
        rng=np.random.default_rng(42);settings=replace(Settings(),ridge_relative=1e-8)
        sx,sy=model_scales('full',settings.length_scale_m)
        xs=rng.normal(size=(100,6))*2e-5;g=rng.normal(size=(6,6))*1000
        x=xs/sx;y=(xs@g.T)/sy
        result,matrix=local_model(x,y,'full',settings)
        self.assertTrue(result['accepted']);self.assertEqual(result['effective_rank'],6)
        self.assertGreater(result['validation_skill_zero'],.999999)
        expected=g*sx[None,:]/sy[:,None]
        np.testing.assert_allclose(matrix['G_SI'],expected,rtol=1e-5,atol=1e-6)
        np.testing.assert_allclose(matrix['G_scaled'],g,rtol=1e-5,atol=1e-6)
        self.assertEqual(matrix['validation_test_start_pair'],matrix['validation_train_pairs']+1)

    def test_rank_deficient_map_is_not_declared_identified(self):
        x=np.zeros((40,6));x[:,2]=2e-5
        y=np.zeros((40,6));y[:,2]=x[:,2]*2000
        result,matrix=local_model(x,y,'full',Settings())
        self.assertTrue(result['accepted']);self.assertFalse(result['full_excitation'])
        self.assertEqual(result['effective_rank'],1);self.assertIsNone(result['condition'])
        self.assertEqual(matrix['excited_input_basis_scaled'].shape,(1,6))
        self.assertAlmostEqual(result['sensitivity'],2000)

    def test_numerical_rank_does_not_overrule_excitation_floor(self):
        rng=np.random.default_rng(0);x=rng.normal(size=(60,6))*1e-10;x[:,0]*=1e6
        info=excitation(x,Settings())
        self.assertEqual(info['numerical_rank'],6);self.assertEqual(info['rank'],1)
        self.assertGreater(info['condition'],1e5)

    def test_zero_and_near_zero_motion_rejected_even_with_force_changes(self):
        for amplitude in (0.,1e-10):
            x=np.full((40,6),amplitude);y=np.ones((40,6))
            result,matrix=local_model(x,y,'full',Settings())
            self.assertFalse(result['accepted']);self.assertIsNone(result['sensitivity']);self.assertIsNone(matrix)

    def test_unresolved_wrench_does_not_produce_a_normalized_validation_claim(self):
        rng=np.random.default_rng(4);x=rng.normal(size=(60,6))*1e-4;y=rng.normal(size=(60,6))*1e-10
        result,_=local_model(x,y,'full',Settings())
        self.assertTrue(result['accepted']);self.assertFalse(result['resolved_wrench_change'])
        self.assertIsNone(result['validation_skill_zero']);self.assertIsNone(result['validation_skill_drift'])
        self.assertIsNotNone(result['validation_rmse_n'])

    def test_csv_loader_uses_actual_pose_and_not_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'insertion.csv'
            path.write_text('time_s,phase,tip_x_mm,tip_y_mm,depth_mm,qw,qx,qy,qz,fx,fy,fz,taux,tauy,tauz,normal_load_n,contact_count,command_depth_mm\n'
                +''.join(f'{i/120},insert,{i},2,{i+5},1,0,0,0,1,2,3,4,5,6,1,1,999999\n' for i in range(3)))
            data=load_reference(path)
            np.testing.assert_allclose(data['position'][1],[.001,.002,-.006])
            np.testing.assert_allclose(data['wrench'][0],[1,2,3,4,5,6])
            self.assertNotIn('command_depth_mm',data)

    def test_reduced_translation_ignores_rotation_and_torque(self):
        rng=np.random.default_rng(12);x=rng.normal(size=(50,6))*1e-4;y=rng.normal(size=(50,6))
        a,_=local_model(x,y,'translation',Settings())
        x[:,3:]*=1e5;y[:,3:]*=1e5
        b,_=local_model(x,y,'translation',Settings())
        self.assertEqual(a,b)

    def test_ridge_validation_detects_a_changed_response(self):
        rng=np.random.default_rng(3);x=rng.normal(size=(60,6))*1e-4
        y=3000*x;y[40:]*=-1
        result,_=local_model(x,y,'translation',Settings())
        self.assertLess(result['validation_skill_zero'],0)

    def test_metric_length_scaling_is_explicit(self):
        x=np.zeros((40,6));x[:,3]=.001;y=np.zeros((40,6));y[:,3]=.01
        a,_=local_model(x,y,'full',Settings(length_scale_m=.01))
        b,_=local_model(x,y,'full',Settings(length_scale_m=.02))
        self.assertAlmostEqual(a['sensitivity']/b['sensitivity'],4.)

    def test_stationary_depth_does_not_produce_infinite_force_depth_slope(self):
        n=61;p=np.zeros((n,3));p[:,0]=np.arange(n)*1e-5
        data=dict(position=p,quaternion=np.tile([1.,0,0,0],(n,1)),wrench=np.zeros((n,6)),
            time=np.arange(n)/120,depth=np.full(n,10.),normal=np.ones(n),contact=np.ones(n))
        data['wrench'][:,0]=np.arange(n)*.1
        row,_=window_analysis(data,0,n-1,Settings())
        self.assertIsNone(row['abs_dforce_ddepth']);self.assertAlmostEqual(row['progress_rate'],0.)
        self.assertAlmostEqual(row['abs_dforce_dt'],12.)

    def test_auc_ties_cluster_weights_and_single_class(self):
        self.assertEqual(auc([0,1],[0,1]),1.)
        self.assertEqual(auc([0,1],[1,0]),0.)
        self.assertEqual(auc([0,1],[2,2]),.5)
        self.assertIsNone(auc([1,1],[0,1]))
        # Duplicating a control with half weight leaves pair probabilities intact.
        self.assertEqual(auc([0,1,1],[0,1,-1]),auc([0,0,1,1],[0,0,1,-1],[.5,.5,1,1]))

    def test_discovery_excludes_non_phase2_studies(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            for name,manifest in [('a',dict(study='Forge-Controlled-Phase2-v1',mode='collect')),
                ('b',dict(study='Forge-Controlled-Phase2-v1',case_plan={'schema':'Forge-Phase2B-depth-drift-v1'})),
                ('pilot',dict(study='Forge-Controlled-Phase2-v1',mode='baseline')),
                ('legacy',dict(study='FR3-Recovery-Study-v1'))]:
                (root/name).mkdir();(root/name/'study.json').write_text(json.dumps(manifest))
            chosen,excluded=discover(root)
            self.assertEqual([x[2] for x in chosen],['Phase2A','Phase2B']);self.assertEqual(len(excluded),2)

    def test_settings_reject_invalid_scales(self):
        for settings in (Settings(length_scale_m=0),Settings(motion_floor_m=-1),Settings(rank_relative_tolerance=2),Settings(lag_steps=0)):
            with self.assertRaises(ValueError):settings.validate()


if __name__=='__main__':unittest.main()
