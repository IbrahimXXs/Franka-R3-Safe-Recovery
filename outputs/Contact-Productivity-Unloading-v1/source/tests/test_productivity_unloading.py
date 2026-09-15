"""Measured unloading, bounded retry authorization and unchanged detector checks."""
import ast
from dataclasses import asdict
from pathlib import Path
import unittest
from research.productivity_unloading import Unloader,UnloadingDesign
from research.productivity_control import Design
from research.phase2b import make_case
from tests.test_productivity_control_execution import FakeBench,runner as previous_runner

class UnloaderTests(unittest.TestCase):
    def test_command_never_substitutes_for_actual_motion(self):
        u=Unloader(0,15)
        for step in range(1,632):
            t=step/120;_,cmd=u.target(t)
            state=u.observe(t,15.)
            self.assertLessEqual(cmd,5.)
            if state!='running':break
        self.assertEqual(state,'budget_exhausted');self.assertIsNone(u.first_crossing_time)
        self.assertAlmostEqual(t,5.25);self.assertAlmostEqual(cmd,5.)

    def test_measured_target_needs_full_hold(self):
        u=Unloader(0,15)
        for i in range(1,121):
            t=i/120;phase,cmd=u.target(t)
            state=u.observe(t,14.5 if t>=.5 else 15.)
            if state!='running':break
        self.assertEqual(state,'verified');self.assertAlmostEqual(t,.75)
        self.assertAlmostEqual(u.first_crossing_time,.5)
        self.assertLess(cmd,.5)  # Actual motion, not the size of the command, authorizes retry.

    def test_rebound_resets_hold_without_resetting_budget(self):
        u=Unloader(0,15);commands=[]
        for i in range(1,150):
            t=i/120;_,cmd=u.target(t);commands.append(cmd)
            depth=14.4 if .5<=t<.65 or t>=.8 else 15.
            state=u.observe(t,depth)
            if state!='running':break
        self.assertEqual(state,'verified');self.assertEqual(u.rebounds,1)
        self.assertAlmostEqual(t,1.05)
        self.assertTrue(all(b>=a for a,b in zip(commands,commands[1:])))
        self.assertLess(max(b-a for a,b in zip(commands,commands[1:])),.016)

    def test_safety_priority_even_at_achieved_target(self):
        u=Unloader(0,15)
        for i in range(1,91):
            t=i/120;u.target(t);state=u.observe(t,14.4 if t>=.5 else 15.,safe=t<.75)
        self.assertEqual(state,'safety_stop');self.assertIsNone(u.verified_time)

    def test_no_overshoot_of_command_and_no_clock_rewind(self):
        u=Unloader(0,15);u.target(.1)
        with self.assertRaises(ValueError):u.target(.1)
        with self.assertRaises(ValueError):UnloadingDesign(max_command_mm=-1).validate()

    def test_time_budget_includes_verification(self):
        u=Unloader(0,15)
        for i in range(1,632):
            t=i/120;u.target(t);state=u.observe(t,14.4 if t>=5.15 else 15.)
            if state!='running':break
        self.assertEqual(state,'budget_exhausted');self.assertIsNotNone(u.first_crossing_time)
        self.assertIsNone(u.verified_time)


def new_runner():
    ns=previous_runner().__globals__.copy();ns.update(Unloader=Unloader,UnloadingDesign=UnloadingDesign)
    path=Path(__file__).resolve().parents[1]/'simulation/productivity_unloading.py';tree=ast.parse(path.read_text())
    subset=ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='execute_verified'],type_ignores=[])
    exec(compile(subset,str(path),'exec'),ns)
    return ns['execute_verified']

class LockedBench(FakeBench):
    def tick(self,hp,hq):
        was_locked=getattr(self,'depth',-10)>=self.ceiling
        super().tick(hp,hq)
        if was_locked:
            self.depth=self.ceiling;self.env.fingertip_midpoint_pos[:,2]=-self.depth/1000

class UnsafeRetreatBench(FakeBench):
    def prepare(self,seed):
        self.engaged=False;return super().prepare(seed)
    def tick(self,hp,hq):
        super().tick(hp,hq)
        if self.depth>=self.ceiling:self.engaged=True
    def observe(self,phase,command):
        r=super().observe(phase,command)
        if self.engaged and self.depth<self.ceiling-.5:r['grasp_slip_mm']=.101
        return r

class ExecutionTests(unittest.TestCase):
    def execute(self,bench):
        frozen=dict(design=asdict(Design()),eta_threshold=.4212659765112803,force_threshold_n=1.035449028015137)
        return new_runner()(bench,make_case(10,'y_roll',-1,'tilt',6.),frozen,UnloadingDesign(),Path('unused'))

    def test_free_retreat_verified_before_every_retry(self):
        r=self.execute(FakeBench())
        self.assertEqual(r['intervention_count'],2)
        for e in r['events']:
            self.assertEqual(e['unloading_status'],'verified');self.assertTrue(e['verified_before_retry'])
            self.assertGreaterEqual(e['actual_retraction_mm'],.5)
            self.assertGreaterEqual(e['retry_started_s']-e['verification_time_s'],.5-1e-8)
            self.assertLessEqual(e['max_commanded_retract_mm'],5.)

    def test_locked_peg_times_out_with_no_retry(self):
        r=self.execute(LockedBench())
        self.assertEqual(r['outcome'],'unloading_budget_exhausted');self.assertEqual(r['intervention_count'],1)
        e=r['events'][0];self.assertNotIn('retry_started_s',e)
        self.assertAlmostEqual(e['max_actual_retract_mm'],0.)
        self.assertAlmostEqual(e['after_time_s']-e['trigger_time_s'],5.25)

    def test_safety_failure_blocks_retry_despite_actual_motion(self):
        r=self.execute(UnsafeRetreatBench())
        self.assertEqual(r['outcome'],'grasp_retention_limit');self.assertEqual(r['intervention_count'],1)
        self.assertNotIn('retry_started_s',r['events'][0]);self.assertFalse(r['insertion_success'])
        self.assertEqual(r['events'][0]['unloading_status'],'safety_stop')

    def test_successful_path_does_not_intervene(self):
        r=self.execute(FakeBench(ceiling=20.))
        self.assertTrue(r['insertion_success']);self.assertEqual(r['intervention_count'],0)

    def test_trigger_is_identical_to_previous_fixed_controller(self):
        frozen=dict(design=asdict(Design()),eta_threshold=.4212659765112803,force_threshold_n=1.035449028015137)
        case=make_case(10,'y_roll',-1,'tilt',6.)
        old=previous_runner()(FakeBench(),case,'productivity',frozen,Path('unused'))
        new=self.execute(FakeBench())
        for key in ('trigger_time_s','trigger_depth_mm','eta_raw','trigger_command_depth_mm'):
            self.assertEqual(old['events'][0][key],new['events'][0][key])

if __name__=='__main__':unittest.main()
