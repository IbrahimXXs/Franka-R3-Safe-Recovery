from dataclasses import asdict,replace
import copy
import unittest
import csv
import math
import tempfile
from collections import Counter
from pathlib import Path
from research.forge_protocol import protocol,load_protocol
from research.forge_collection import accepted_slots,next_case,collection_status,validate_resume,family_quotas,sample_collection_slot


def attempt(slot=0,retry=0,valid=True,status='complete'):
    return dict(slot=slot,retry=retry,status=status,metrics=dict(numerically_valid=valid,insertion_success=False),
                checkpoints=[dict(Y_R_tested=None)])


class CollectionTests(unittest.TestCase):
    def test_saved_plan_matches_runtime_quotas_and_ranges(self):
        root=Path(__file__).resolve().parents[1]
        p=load_protocol(root/'experiments/forge_phase2.json')
        self.assertEqual(asdict(p),asdict(protocol()))
        rows=[sample_collection_slot(p,i) for i in range(100)]
        self.assertEqual(Counter(r['family'] for r in rows),dict(centered=8,x_offset=18,
            y_offset=18,diagonal_offset=18,tilt_only=19,offset_tilt=19))
        with (root/'experiments/forge_phase2_plan.csv').open() as f:saved=list(csv.DictReader(f))
        self.assertEqual(saved,[{k:str(v) for k,v in r.items()} for r in rows])
        offsets=[math.hypot(r['offset_x_mm'],r['offset_y_mm']) for r in rows if 'offset' in r['family']]
        tilts=[math.hypot(r['roll_deg'],r['pitch_deg']) for r in rows if 'tilt' in r['family']]
        self.assertTrue(all(.1<=r<=1. for r in offsets));self.assertGreater(max(offsets),.75)
        self.assertTrue(all(.25<=a<=4. for a in tilts));self.assertGreater(max(tilts),3.)
        for family in ('x_offset','y_offset','diagonal_offset','tilt_only','offset_tilt'):
            subset=[r for r in rows if r['family']==family]
            keys={'x_offset':['offset_x_mm'],'y_offset':['offset_y_mm'],
                  'diagonal_offset':['offset_x_mm','offset_y_mm'],
                  'tilt_only':['roll_deg','pitch_deg'],
                  'offset_tilt':['offset_x_mm','offset_y_mm','roll_deg','pitch_deg']}[family]
            for key in keys:
                self.assertLess(min(r[key] for r in subset),0.)
                self.assertGreater(max(r[key] for r in subset),0.)

    def test_scaled_quotas_fill_target_and_reject_invalid_weights(self):
        for target in (1,2,5,6,99,100,101,200):
            p=replace(protocol(),target_valid=target)
            self.assertEqual(sum(family_quotas(p).values()),target)
            self.assertEqual(Counter(sample_collection_slot(p,i)['family'] for i in range(target)),
                             {k:v for k,v in family_quotas(p).items() if v})
        for weights in ({},dict.fromkeys(protocol().family_weights,0),
                        dict(protocol().family_weights,centered=-1)):
            with self.assertRaisesRegex(ValueError,'family_weights'):replace(protocol(),family_weights=weights).validate()

    def test_controls_and_retries_keep_split_groups(self):
        p=protocol();rows=[sample_collection_slot(p,i) for i in range(100)]
        controls=[r for r in rows if r['sample_role']=='repeatability_control']
        self.assertEqual(len(controls),8)
        self.assertEqual({r['split_group_id'] for r in controls},{'centered_controls'})
        self.assertEqual(len({r['split_group_id'] for r in rows}),93)
        for r in rows:
            retry=sample_collection_slot(p,r['slot'],1)
            self.assertEqual(retry['split_group_id'],r['split_group_id'])
            self.assertEqual(retry['family'],r['family'])
            for key in ('offset_x_mm','offset_y_mm','roll_deg','pitch_deg'):
                self.assertEqual(retry[key]>0,r[key]>0)
                self.assertEqual(retry[key]<0,r[key]<0)

    def test_group_metadata_reaches_all_report_tables(self):
        from simulation.phase2_report import write_phase2_report
        p=protocol();case=sample_collection_slot(p,0)
        a=dict(case,folder=case['trajectory_id'],status='complete',metrics={'numerically_valid':True},
               checkpoints=[dict(depth_mm=5.,reached=True,prefix_numerically_valid=True,
                    state=dict(depth_mm=5.,force_norm_n=1.),Y_R_tested=None,
                    probes=[dict(policy='straight',label_eligible=False)])])
        manifest=dict(study='Forge-Controlled-Phase2-v1',mode='collect',status='paused',
                      protocol=asdict(p),planned_family_quotas=family_quotas(p),attempts=[a])
        with tempfile.TemporaryDirectory() as tmp:
            write_phase2_report(tmp,manifest)
            for name in ('trajectories.csv','checkpoints.csv','recovery_probes.csv'):
                with (Path(tmp)/name).open() as f:rows=list(csv.DictReader(f))
                self.assertEqual(rows[0]['split_group_id'],'centered_controls')
                self.assertEqual(rows[0]['sample_role'],'repeatability_control')

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
