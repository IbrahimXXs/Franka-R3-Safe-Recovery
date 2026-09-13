import math
import unittest
from dataclasses import replace
from research.phase2_protocol import Protocol, FAMILIES, sample_slot, insertion_metrics, recovery_label, replay_match


def row(i,depth=0.,command=0.,force=0.,penetration=0.):
    r=dict(time_s=i*.1,phase='insert',command_depth_mm=command,depth_mm=depth,tip_x_mm=0.,tip_y_mm=0.,
           force_norm_n=force,torque_norm_nm=0.,normal_load_n=force,min_separation_mm=-penetration,
           qw=1.,qx=0.,qy=0.,qz=0.)
    for key in ('fx','fy','fz','taux','tauy','tauz','vx','vy','vz','omegax','omegay','omegaz'):r[key]=0.
    for j in range(1,8):r[f'joint{j}_rad']=0.;r[f'joint_velocity{j}_rad_s']=0.
    return r


class Phase2ProtocolTests(unittest.TestCase):
    def test_balanced_seeded_families_and_signed_directions(self):
        p=Protocol();samples=[sample_slot(p,i) for i in range(100)]
        self.assertEqual(samples,[sample_slot(p,i) for i in range(100)])
        counts=[sum(s['family']==f for s in samples) for f in FAMILIES]
        self.assertEqual(sum(counts),100);self.assertLessEqual(max(counts)-min(counts),1)
        for family,key in [('x_offset','offset_x_mm'),('y_offset','offset_y_mm'),('tilt_only','roll_deg'),('tilt_only','pitch_deg')]:
            values=[s[key] for s in samples if s['family']==family]
            self.assertLess(min(values),0);self.assertGreater(max(values),0)
        for family in ('diagonal_offset','offset_tilt'):
            signs={(math.copysign(1,s['offset_x_mm']),math.copysign(1,s['offset_y_mm'])) for s in samples if s['family']==family}
            self.assertEqual(len(signs),4)

    def test_retries_keep_family_and_direction(self):
        p=Protocol()
        for slot in range(30):
            a,b=sample_slot(p,slot,0),sample_slot(p,slot,1)
            self.assertEqual(a['family'],b['family'])
            for k in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg'):
                self.assertEqual(math.copysign(1,a[k]),math.copysign(1,b[k]))
                self.assertEqual(a[k]==0,b[k]==0)

    def test_depth_success_does_not_hide_invalid_contacts_or_budget(self):
        rows=[row(i,20.,20.,force=25.,penetration=.2) for i in range(8)]
        m=insertion_metrics(rows,.1,Protocol(),.5)
        self.assertTrue(m['insertion_success']);self.assertFalse(m['numerically_valid'])
        self.assertTrue(m['force_budget_exceeded']);self.assertFalse(m['outcome_trustworthy'])

    def test_stall_requires_advancing_command(self):
        rows=[row(i,5.,i*.2) for i in range(10)]
        self.assertTrue(insertion_metrics(rows,.1,Protocol(),.5)['stalled'])
        for r in rows:r['command_depth_mm']=5.
        self.assertFalse(insertion_metrics(rows,.1,Protocol(),.5)['stalled'])

    def test_labels_require_valid_tests_and_do_not_infer_global_failure(self):
        fail=lambda policy:dict(policy=policy,label_eligible=True,safe_recovery=False)
        self.assertIsNone(recovery_label([fail('straight')])[0])
        self.assertEqual(recovery_label([fail('straight'),fail('realign')]),(0,'tested_policies_failed'))
        self.assertEqual(recovery_label([dict(policy='straight',label_eligible=True,safe_recovery=True)])[0],1)
        self.assertIsNone(recovery_label([dict(policy='straight',label_eligible=False,safe_recovery=True)])[0])
        self.assertIsNone(recovery_label([],reached=False)[0])
        self.assertIsNone(recovery_label([fail('straight'),fail('realign')],prefix_valid=False)[0])

    def test_replay_checks_velocity_and_quaternion_sign(self):
        a=row(0);b=a.copy();b['qw']=-1.
        self.assertTrue(replay_match(a,b,Protocol())[0])
        b['joint_velocity3_rad_s']=.2
        self.assertFalse(replay_match(a,b,Protocol())[0])
        b=a.copy();b['tip_y_mm']=.15
        self.assertFalse(replay_match(a,b,Protocol())[0])

    def test_checkpoint_validation(self):
        with self.assertRaises(ValueError):replace(Protocol(),checkpoints_mm=(10.,5.)).validate()
        with self.assertRaises(ValueError):replace(Protocol(),checkpoints_mm=(25.,)).validate()

if __name__=='__main__':unittest.main()
