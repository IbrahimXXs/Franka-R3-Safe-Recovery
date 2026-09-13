from dataclasses import asdict,replace
import copy
import unittest
from research.forge_protocol import protocol
from research.forge_collection import accepted_slots,next_case,collection_status,validate_resume


def attempt(slot=0,retry=0,valid=True,status='complete'):
    return dict(slot=slot,retry=retry,status=status,metrics=dict(numerically_valid=valid,insertion_success=False),
                checkpoints=[dict(Y_R_tested=None)])


class CollectionTests(unittest.TestCase):
    def test_physical_failures_and_unknown_labels_do_not_bias_quota(self):
        p=replace(protocol(),target_valid=2)
        self.assertEqual(accepted_slots([attempt()]),{0})
        self.assertEqual(next_case(p,[attempt()])['slot'],1)

    def test_incomplete_valid_reference_does_not_fill_slot(self):
        self.assertEqual(accepted_slots([attempt(status='interrupted')]),set())
        self.assertEqual(next_case(protocol(),[attempt(status='interrupted')])['retry'],0)

    def test_retry_preserves_direction_and_resamples_magnitude(self):
        p=protocol();a=next_case(p,[attempt()]);b=next_case(p,[attempt(),attempt(slot=1,valid=False)])
        self.assertEqual(a['family'],b['family']);self.assertEqual(b['retry'],1)
        self.assertGreater(a['offset_x_mm']*b['offset_x_mm'],0)
        self.assertNotEqual(a['offset_x_mm'],b['offset_x_mm'])

    def test_attempt_limit_and_completion_are_distinct(self):
        p=replace(protocol(),target_valid=1,attempts_per_slot=2)
        self.assertEqual(collection_status(p,[]),'paused')
        self.assertEqual(collection_status(p,[attempt(valid=False),attempt(retry=1,valid=False)]),'target_not_met')
        self.assertEqual(collection_status(p,[attempt(valid=False),attempt(retry=1)]),'complete')

    def test_resume_rejects_changed_protocol_clock_and_sources(self):
        m=dict(study='Forge-Controlled-Phase2-v1',mode='collect',protocol=asdict(protocol()),physics_hz=120,
               requested_checkpoints_mm=[5.,10.,15.,20.],sources={'runner':'hash'},stock_buffers=False)
        validate_resume(m,copy.deepcopy(m))
        for key,value in (('physics_hz',480),('sources',{}),('requested_checkpoints_mm',[5.]),('stock_buffers',True)):
            changed=copy.deepcopy(m);changed[key]=value
            with self.assertRaisesRegex(ValueError,key):validate_resume(m,changed)
        changed=copy.deepcopy(m);changed['protocol']['target_valid']=200
        with self.assertRaisesRegex(ValueError,'protocol'):validate_resume(m,changed)

    def test_resume_cannot_convert_a_pilot_into_collection(self):
        with self.assertRaisesRegex(ValueError,'collect-mode'):validate_resume({'mode':'pilot'},{})


if __name__=='__main__':unittest.main()
