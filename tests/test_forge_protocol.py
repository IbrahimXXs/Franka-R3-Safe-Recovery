import unittest
from research.forge_protocol import protocol,pilot_cases,retained,clear,match,recovery_summary
from tests.test_phase2_protocol import row


def sample(i=0,dt=.01):
    r=row(i)
    r.update(grasp_slip_mm=0.,grasp_slip_deg=0.,wrist_force_n=0.,wrist_torque_nm=0.,
             lowest_peg_z_above_mouth_mm=3.,normal_load_n=0.,contact_power_w=0.,recovery_time_s=i*dt)
    for prefix,n in (('finger_position',2),('finger_velocity',2),('grasp_position',3),('hand_pose',7),('wrist_raw_child',6),('grasp_quat',4)):
        for j in range(n):r[f'{prefix}_{j}']=0.
    r['grasp_quat_0']=1.;r['hand_pose_3']=1.
    return r


class ForgeProtocolTests(unittest.TestCase):
    def test_pilot_covers_families_and_signed_rotations(self):
        cases=pilot_cases(protocol())
        self.assertEqual(len({c['family'] for c in cases}),6)
        for k in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg'):
            self.assertLess(min(c[k] for c in cases),0.)
            self.assertGreater(max(c[k] for c in cases),0.)

    def test_gripper_state_can_reject_an_otherwise_matching_replay(self):
        a=sample();b=a.copy()
        self.assertTrue(match(a,b,protocol())[0])
        b['finger_position_0']+=.00001
        self.assertFalse(match(a,b,protocol())[0])

    def test_quaternion_sign_does_not_reject_replay(self):
        a=sample();b=a.copy();b['hand_pose_3']=-1.;b['grasp_quat_0']=-1.
        self.assertTrue(match(a,b,protocol())[0])

    def test_replay_tolerance_smaller_than_clearance(self):
        a=sample();b=a.copy();b['tip_x_mm']+=.01
        self.assertFalse(match(a,b,protocol())[0])

    def test_initial_over_budget_state_cannot_be_erased_by_retreat(self):
        rows=[sample(i) for i in range(30)];rows[0]['wrist_force_n']=21.
        result=recovery_summary(rows,protocol(),.01)
        self.assertTrue(result['label_eligible']);self.assertFalse(result['safe_recovery'])
        self.assertTrue(result['force_budget_exceeded'])

    def test_budget_uses_wrist_and_not_contact_torque_origin(self):
        rows=[sample(i) for i in range(30)];rows[0]['wrist_torque_nm']=1.1
        self.assertFalse(recovery_summary(rows,protocol(),.01)['safe_recovery'])

    def test_clearance_requires_entire_peg_and_contact_release(self):
        r=sample();r['lowest_peg_z_above_mouth_mm']=1.9
        self.assertFalse(clear(r))
        r['lowest_peg_z_above_mouth_mm']=3.;r['normal_load_n']=.2
        self.assertFalse(clear(r))

    def test_grasp_loss_is_failure_not_numerical_invalidity(self):
        rows=[sample(i) for i in range(30)];rows[0]['grasp_slip_mm']=.2
        result=recovery_summary(rows,protocol(),.01)
        self.assertTrue(result['numerically_valid']);self.assertTrue(result['label_eligible'])
        self.assertFalse(result['safe_recovery']);self.assertFalse(retained(rows[0]))

    def test_clearance_dwell_and_unknown_invalid_probes(self):
        self.assertFalse(recovery_summary([sample(i) for i in range(10)],protocol(),.01)['cleared'])
        rows=[sample(i) for i in range(30)]
        self.assertTrue(recovery_summary(rows,protocol(),.01)['safe_recovery'])
        self.assertIsNone(recovery_summary(rows,protocol(),.01,matched=False)['safe_recovery'])
        rows[0]['min_separation_mm']=-.2
        self.assertIsNone(recovery_summary(rows,protocol(),.01)['safe_recovery'])


if __name__=='__main__':unittest.main()
