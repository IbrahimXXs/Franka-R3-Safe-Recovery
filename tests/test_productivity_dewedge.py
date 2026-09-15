"""Recovery math, timing, safety and actual runner scheduling without Isaac."""
import ast
from dataclasses import asdict
from pathlib import Path
import unittest
import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp
from research.productivity_dewedge import Dewedger,DewedgeDesign,qmul,qconj,pose_angles,neutral_quaternion
from research.productivity_unloading import UnloadingDesign
from research.productivity_control import Design
from research.phase2b import make_case
from tests.test_productivity_control_execution import FakeBench,runner as old_runner,MemoryStream


def quat(rpy):return np.roll(Rotation.from_euler('xyz',rpy,degrees=True).as_quat(),1)
def mul(a,b):return torch.tensor(np.array([qmul(x,y) for x,y in zip(a.numpy(),b.numpy())]),dtype=a.dtype)
def conj(a):return a*torch.tensor([1,-1,-1,-1],dtype=a.dtype)
def apply(q,v):return torch.tensor(np.array([Rotation.from_quat(np.roll(x,-1)).apply(y) for x,y in zip(q.numpy(),v.numpy())]),dtype=v.dtype)
def slerp(a,b,f):return torch.tensor(np.roll(Slerp([0,1],Rotation.from_quat(np.roll(np.array([a.numpy(),b.numpy()]),-1,axis=1)))(f).as_quat(),1),dtype=a.dtype)

class ScheduleTests(unittest.TestCase):
    def run_schedule(self,depth,safe=lambda t:True):
        u=Dewedger(0,15);logs=[]
        for i in range(1,632):
            t=i/120;phase,f,c=u.target(t);state=u.observe(t,depth(t),safe(t));logs.append((t,phase,f,c,state))
            if state!='running':break
        return u,logs
    def test_fixed_depth_relax_then_bounded_retract(self):
        u,logs=self.run_schedule(lambda t:15)
        self.assertEqual(u.status,'budget_exhausted');self.assertAlmostEqual(logs[-1][0],5.25)
        self.assertTrue(all(c==0 for t,_,f,c,_ in logs if t<=.75))
        self.assertTrue(all(f==1 for t,_,f,c,_ in logs if t>.75))
        self.assertAlmostEqual(logs[-1][3],5);self.assertIsNone(u.first_crossing_time)
    def test_crossing_requires_verification_and_additional_hold(self):
        u,logs=self.run_schedule(lambda t:14.4 if t>=1 else 15)
        self.assertEqual(u.status,'verified');self.assertAlmostEqual(u.verification_time,1.25)
        self.assertAlmostEqual(u.ready_time,1.5)
        self.assertEqual(len(set(c for t,_,_,c,_ in logs if t>=1)),1)
    def test_relaxation_alone_can_achieve_unloading_but_must_finish(self):
        u,logs=self.run_schedule(lambda t:14.4 if t>=.4 else 15)
        self.assertAlmostEqual(u.verification_time,1.);self.assertAlmostEqual(u.ready_time,1.25)
        self.assertEqual(u.command_mm,0.)
    def test_rebound_revokes_readiness(self):
        u,logs=self.run_schedule(lambda t:14.4 if 1<=t<1.3 or t>=1.6 else 15)
        self.assertEqual(u.rebounds,1);self.assertAlmostEqual(u.verification_time,1.25)
        self.assertAlmostEqual(u.ready_time,2.1)
    def test_verified_target_does_not_extend_budget_for_extra_hold(self):
        u,logs=self.run_schedule(lambda t:14.4 if t>=4.9 else 15)
        self.assertEqual(u.status,'budget_exhausted');self.assertIsNotNone(u.verification_time);self.assertIsNone(u.ready_time)
    def test_safety_wins_on_retry_ready_tick(self):
        u,logs=self.run_schedule(lambda t:14.4 if t>=1 else 15,lambda t:t<1.5)
        self.assertEqual(u.status,'safety_stop');self.assertIsNone(u.ready_time)
    def test_neutral_removes_relative_tilt_preserves_yaw_and_sign(self):
        nominal=np.array([0,0,0,1]);actual=qmul(quat([5,-4,7]),nominal)
        np.testing.assert_allclose(pose_angles(neutral_quaternion(actual,nominal),nominal),[0,0,7],atol=1e-10)
        np.testing.assert_allclose(neutral_quaternion(-actual,nominal),neutral_quaternion(actual,nominal),atol=1e-10)
    def test_rotate_about_peg_keeps_peg_depth(self):
        pp=torch.tensor([[.001,0,-.015]],dtype=torch.float64);gp=torch.tensor([[.01,0,-.025]],dtype=torch.float64)
        hqs=[torch.tensor([quat(a)],dtype=torch.float64) for a in ([0,6,0],[0,0,0])]
        hand=[pp-apply(q,gp) for q in hqs]
        for hp,hq in zip(hand,hqs):np.testing.assert_allclose((hp+apply(hq,gp)).numpy(),pp.numpy(),atol=1e-12)
        self.assertGreater(abs(hand[0][0,2]-hand[1][0,2]),.0001)

class PegBench(FakeBench):
    def __init__(self,locked=False,unsafe_relax=False,ceiling=14.):
        super().__init__(ceiling);self.locked=locked;self.unsafe_relax=unsafe_relax
        self.peg_q=torch.tensor([[0.,0,0,1]],dtype=torch.float64)
        self.grasp_p=torch.tensor([[0.,0,-.02]],dtype=torch.float64);self.grasp_q=torch.tensor([[1.,0,0,0]],dtype=torch.float64)
        self.env.fixed_pos_obs_frame=torch.zeros((1,3),dtype=torch.float64)
    def prepare(self,seed):
        self.engaged=False;self.t=0.;self.depth=-10.
        hp,hq=self.target(-10.)
        self.env.fingertip_midpoint_pos=hp;self.env.fingertip_midpoint_quat=hq
        self.env.held_pos=hp+apply(hq,self.grasp_p);self.env.held_quat=hq.clone()
        return self.observe('initial',-10.)
    def target(self,depth,x_mm=0.,y_mm=0.,roll_deg=0.,pitch_deg=0.):
        pq=mul(torch.tensor([quat([roll_deg,pitch_deg,0])],dtype=torch.float64),self.peg_q)
        pp=torch.tensor([[x_mm/1000,y_mm/1000,-depth/1000]],dtype=torch.float64)
        return pp-apply(pq,self.grasp_p),pq
    def tick(self,hp,hq):
        self.t+=self.dt;pp=hp+apply(hq,self.grasp_p)
        depth=min(-pp[0,2].item()*1000,self.ceiling)
        if self.locked and self.engaged:depth=self.ceiling
        self.depth=depth;self.engaged|=depth>=self.ceiling;pp[:,2]=-depth/1000
        self.env.held_pos=pp;self.env.held_quat=hq.clone()
        self.env.fingertip_midpoint_pos=pp-apply(hq,self.grasp_p);self.env.fingertip_midpoint_quat=hq.clone()
    def observe(self,phase,command):
        row=super().observe(phase,command)
        if self.unsafe_relax and phase=='relax':row['grasp_slip_mm']=.101
        return row

class SavedStream(MemoryStream):
    latest=None
    def __init__(self,path):super().__init__(path);SavedStream.latest=self

def runner():
    ns=old_runner().__globals__.copy();ns.update(Dewedger=Dewedger,DewedgeDesign=DewedgeDesign,UnloadingDesign=UnloadingDesign,
        neutral_quaternion=neutral_quaternion,torch=torch,quat_mul=mul,quat_apply=apply,quat_conjugate=conj,quat_slerp=slerp,Stream=SavedStream)
    path=Path(__file__).resolve().parents[1]/'simulation/productivity_dewedge.py'
    tree=ast.parse(path.read_text());subset=ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute_dewedge'],type_ignores=[])
    exec(compile(subset,str(path),'exec'),ns);return ns['execute_dewedge']

class ExecutionTests(unittest.TestCase):
    frozen=dict(design=asdict(Design()),eta_threshold=.4212659765112803,force_threshold_n=1.035449028015137)
    case=make_case(10,'y_roll',-1,'tilt',6.)
    def execute(self,bench):return runner()(bench,self.case,self.frozen,UnloadingDesign(),DewedgeDesign(),Path('unused'))
    def test_same_first_trigger_as_frozen_controller(self):
        old=old_runner()(FakeBench(),self.case,'productivity',self.frozen,Path('unused'));new=self.execute(PegBench())
        for k in ('trigger_time_s','trigger_depth_mm','eta_raw','trigger_command_depth_mm'):
            self.assertAlmostEqual(old['events'][0][k],new['events'][0][k],places=10)
    def test_retry_retains_relaxed_pose_and_verified_actual_retreat(self):
        r=self.execute(PegBench());self.assertEqual(r['intervention_count'],2)
        for e in r['events']:
            self.assertEqual(e['unloading_status'],'verified');self.assertGreaterEqual(e['actual_retraction_mm'],.5)
            self.assertAlmostEqual(e['retry_ready_time_s']-e['verification_time_s'],.25)
            np.testing.assert_allclose(pose_angles(e['retry_peg_quaternion'],[0,0,0,1]),0,atol=1e-8)
        retry=[x for x in SavedStream.latest.rows if x['retry_pose_latched'] and x['phase']=='insert']
        self.assertTrue(retry)
        for row in retry:
            q=[row[f'command_hand_quaternion_{k}'] for k in range(4)]
            np.testing.assert_allclose(pose_angles(q,[0,0,0,1]),0,atol=1e-8)
    def test_locked_target_ends_without_retry(self):
        r=self.execute(PegBench(locked=True));self.assertEqual(r['outcome'],'unloading_budget_exhausted')
        self.assertNotIn('retry_started_s',r['events'][0]);self.assertEqual(r['intervention_count'],1)
    def test_safety_during_relaxation_terminates_immediately(self):
        r=self.execute(PegBench(unsafe_relax=True));self.assertEqual(r['outcome'],'grasp_retention_limit')
        self.assertNotIn('retry_started_s',r['events'][0]);self.assertEqual(r['events'][0]['unloading_status'],'safety_stop')
    def test_successful_control_has_no_intervention(self):
        r=self.execute(PegBench(ceiling=20));self.assertTrue(r['insertion_success']);self.assertEqual(r['intervention_count'],0)

if __name__=='__main__':unittest.main()
