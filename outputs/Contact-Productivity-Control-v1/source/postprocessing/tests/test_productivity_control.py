"""Focused causal signal, trigger and retry/stall accounting tests (no simulator)."""
import unittest
from dataclasses import replace
from research.productivity_control import Design, Detector, recent_signal, clock_for_depth, blend, stalled_now
from research.forge_protocol import protocol


def history(eta=.4, command_rate=2., phase='insert', segment=0):
    return [dict(time_s=i/120,depth_mm=6.+eta*command_rate*i/120,
                 command_depth_mm=6.+command_rate*i/120,wrist_force_n=2.,phase=phase,segment=segment) for i in range(61)]

class ProductivityControlTests(unittest.TestCase):
    def test_raw_ratio_unclipped(self):
        for eta in (-.3,.4,1.5):self.assertAlmostEqual(recent_signal(history(eta),120,5)['eta_raw'],eta)

    def test_zero_negative_and_small_command_rejected(self):
        for rate in (0.,-1.,.1):self.assertIsNone(recent_signal(history(command_rate=rate),120,5))
        self.assertIsNotNone(recent_signal(history(command_rate=.2),120,5))

    def test_full_contiguous_contact_history_required(self):
        self.assertIsNone(recent_signal(history()[:-1],120,5))
        self.assertIsNone(recent_signal(history(),120,10))
        for key,value in (('phase','retract'),('segment',1),('time_s',.02)):
            rows=history();rows[0][key]=value;self.assertIsNone(recent_signal(rows,120,5))

    def test_two_checks_and_strict_eta_threshold(self):
        d=Detector('productivity',.42,1.)
        s=dict(eta_raw=.4,wrist_force_n=0.)
        self.assertFalse(d.check(1.,s));self.assertTrue(d.check(1.1,s))
        self.assertFalse(d.check(1.2,dict(s,eta_raw=.42)))
        self.assertFalse(d.check(1.3,s));self.assertTrue(d.check(1.4,s))

    def test_missing_gap_reset(self):
        d=Detector('productivity',.42,1.);s=dict(eta_raw=.4,wrist_force_n=0.)
        d.check(1,s);self.assertFalse(d.check(1.2,s))
        self.assertFalse(d.check(1.3,None));self.assertFalse(d.check(1.4,s))
        d.reset();self.assertFalse(d.check(1.5,s))

    def test_force_uses_only_force_and_nominal_never_triggers(self):
        s=dict(eta_raw=1.,wrist_force_n=1.)
        d=Detector('force',.42,1.);d.check(1,s);self.assertTrue(d.check(1.1,s))
        n=Detector('nominal',.42,1.)
        for i in range(10):self.assertFalse(n.check(i*.1,dict(eta_raw=-1,wrist_force_n=50)))

    def test_no_privileged_contact_predictor(self):
        a=history();b=[dict(r,normal_load_n=1e9,fx=1e9) for r in a]
        self.assertEqual(recent_signal(a,120,5),recent_signal(b,120,5))

    def test_retry_clock_inverts_depth_and_is_bounded(self):
        for depth in (-10.,0.,5.,15.,19.,20.):
            clock=clock_for_depth(depth)
            self.assertAlmostEqual(-10+30*blend(clock/8),depth,places=10)
        self.assertAlmostEqual(clock_for_depth(-30),0);self.assertAlmostEqual(clock_for_depth(50),8)

    def test_stall_uses_existing_predicate_and_no_pause_bridging(self):
        rows=history(eta=0.)
        self.assertTrue(stalled_now(rows,120,protocol()))
        rows[20]['phase']='stop';self.assertFalse(stalled_now(rows,120,protocol()))
        rows[20]['phase']='insert';rows[20]['segment']=1
        self.assertFalse(stalled_now(rows,120,protocol()))
        self.assertFalse(stalled_now(history(eta=1.),120,protocol()))

if __name__=='__main__':unittest.main()
