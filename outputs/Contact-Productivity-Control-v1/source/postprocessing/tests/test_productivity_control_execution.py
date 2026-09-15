"""Exercise the actual runner loop against an idealized, CPU-only peg fixture.

This checks scheduling/accounting and safety priority, not physical performance.
Extracting just the runner functions avoids importing the Isaac application.
"""
import ast
from pathlib import Path
import tempfile
import unittest
import torch
from types import SimpleNamespace
from research.productivity_control import Design, Detector, blend, clock_for_depth, recent_signal, stalled_now
from research.forge_protocol import protocol, screened, within_budget, retained
from research.phase2b import make_case, command_misalignment
from dataclasses import asdict


class MemoryStream:
    def __init__(self,path):self.rows=[]
    def add(self,row):self.rows.append(row)
    def close(self):pass


class FakeBench:
    dt=1/120
    protocol=protocol()
    def __init__(self,ceiling=14.,unsafe=False):
        self.ceiling=ceiling;self.unsafe=unsafe
        self.env=SimpleNamespace(fingertip_midpoint_pos=torch.tensor([[0.,0.,.01]],dtype=torch.float64),
                                 fingertip_midpoint_quat=torch.tensor([[1.,0.,0.,0.]],dtype=torch.float64))
    def prepare(self,seed):
        self.t=0.;self.depth=-10.;self.env.fingertip_midpoint_pos[:,2]=.01
        return self.observe('initial',-10.)
    def target(self,depth,**kwargs):
        return torch.tensor([[0.,0.,-depth/1000]],dtype=torch.float64),self.env.fingertip_midpoint_quat.clone()
    def tick(self,hp,hq):
        self.t+=self.dt;self.depth=min(-hp[0,2].item()*1000,self.ceiling)
        self.env.fingertip_midpoint_pos=hp.clone();self.env.fingertip_midpoint_pos[:,2]=-self.depth/1000
        self.env.fingertip_midpoint_quat=hq.clone()
    def observe(self,phase,command):
        return dict(time_s=self.t,phase=phase,depth_mm=self.depth,command_depth_mm=command,
            wrist_force_n=21. if self.unsafe and self.depth>=self.ceiling else 2.,wrist_torque_nm=.01,
            force_norm_n=2.,normal_load_n=3.,min_separation_mm=0.,grasp_slip_mm=0.,grasp_slip_deg=0.)


def runner():
    path=Path(__file__).resolve().parents[1]/'simulation/productivity_control.py'
    tree=ast.parse(path.read_text())
    subset=ast.Module(body=[node for node in tree.body if isinstance(node,ast.FunctionDef) and node.name in ('execute','safety_reason')],type_ignores=[])
    namespace=dict(Design=Design,Detector=Detector,blend=blend,clock_for_depth=clock_for_depth,
        recent_signal=recent_signal,stalled_now=stalled_now,command_misalignment=command_misalignment,
        screened=screened,within_budget=within_budget,retained=retained,Stream=MemoryStream,
        quat_slerp=lambda a,b,f:(1-f)*a+f*b)
    exec(compile(subset,str(path),'exec'),namespace)
    return namespace['execute']


class ExecutionTests(unittest.TestCase):
    def run_policy(self,policy,ceiling=14.,unsafe=False):
        frozen=dict(design=asdict(Design()),eta_threshold=.42,force_threshold_n=1.)
        return runner()(FakeBench(ceiling,unsafe),make_case(10,'y_roll',-1,'tilt',6.),policy,frozen,Path('unused'))

    def test_nominal_has_no_interventions_and_same_time_budget(self):
        r=self.run_policy('nominal')
        self.assertEqual(r['intervention_count'],0);self.assertEqual(r['outcome'],'time_budget_exhausted')
        self.assertTrue(r['stalled']);self.assertAlmostEqual(r['insertion_time_s'],20.)

    def test_bounded_retract_retry_really_retracts_and_resets_history(self):
        for policy in ('force','productivity'):
            r=self.run_policy(policy)
            self.assertEqual(r['intervention_count'],2);self.assertEqual(r['outcome'],'time_budget_exhausted')
            self.assertAlmostEqual(r['insertion_time_s'],20.)
            for event in r['events']:
                self.assertAlmostEqual(event['actual_retraction_mm'],1.,places=6)
                self.assertAlmostEqual(event['actual_retract_phase_mm'],1.,places=6)
                self.assertAlmostEqual(event['retry_started_s']-event['trigger_time_s'],1.25,places=6)
            self.assertGreaterEqual(r['events'][1]['trigger_time_s']-r['events'][0]['retry_started_s'],.6-1e-6)

    def test_hard_safety_wins_over_soft_force_trigger(self):
        r=self.run_policy('nominal',unsafe=True)
        self.assertEqual(r['outcome'],'operational_budget_exceeded');self.assertFalse(r['insertion_success'])
        self.assertTrue(r['force_budget_exceeded'])
        # A hard violation at the first moving tick must stop even the force policy.
        r=self.run_policy('force',ceiling=-10.,unsafe=True)
        self.assertEqual(r['outcome'],'operational_budget_exceeded');self.assertEqual(r['intervention_count'],0)

    def test_success_requires_dwell_and_terminates_early(self):
        r=self.run_policy('nominal',ceiling=20.)
        self.assertTrue(r['insertion_success']);self.assertFalse(r['stalled'])
        self.assertGreaterEqual(r['final_depth_mm'],19.5);self.assertLess(r['insertion_time_s'],8.)
        self.assertEqual(r['time_to_success_s'],r['insertion_time_s'])

if __name__=='__main__':unittest.main()
