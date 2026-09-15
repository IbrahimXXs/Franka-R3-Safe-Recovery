"""Output-audit checks reject corrupt evidence and retain partial-study scope."""
import copy
import csv
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_mechanics_plan import make_plan
from research.forge_protocol import protocol
from dataclasses import asdict
from tests.test_forge_protocol import sample
from simulation.validate_mechanics_output import Audit, _motion_sample, _stream, validate_study, control_prefix_checks


def fixture(directory):
    plan = make_plan()
    case = plan['cases'][0]
    material = dict(requested_pair_static_friction=.5, requested_pair_dynamic_friction=.5,
                    effective_pair_static_friction=.5, effective_pair_dynamic_friction=.5,
                    runtime_materials={name:[[[coefficient, coefficient, 0.]]] for name, coefficient in
                                       [('hole', .25), ('peg', .75), ('robot', .75)]},
                    bindings={name:dict(material_path='/'+name, resolved_friction_combine_mode='average',
                                        bound_before_physics_initialization=True, usd_static_friction=coefficient,
                                        usd_dynamic_friction=coefficient) for name, coefficient in [('hole', .25), ('peg', .75)]})
    straight = dict(numerically_valid=True, label_eligible=True, safe_recovery=True, cleared=True,
                    grasp_retained=True, force_budget_exceeded=False, torque_budget_exceeded=False,
                    reason='cleared', recovery_censored=False)
    probe = dict(policy='realign', replay_matched=False, replay_prefix_matched=False,
                 replay_prefix_equal=False, replay_prefix_numerically_valid=True,
                 label_eligible=False, safe_recovery=None, reason='replay_mismatch')
    attempt = dict(case, status='complete', folder='closed', effective_radial_clearance_mm=.199,
                   material=material, final_retreat=straight, probes=[probe],
                   metrics=dict(reference_complete=True, numerically_valid=True, grasp_retained=True,
                                reference_within_budget=True, depth_attained=True))
    source = directory/'source'/'example.py'
    source.parent.mkdir(parents=True)
    source.write_text('ARCHIVED = True\n')
    return dict(study='Forge-mechanics-v1', status='running', case_plan=plan, attempts=[attempt],
                skipped_cases=[], physics_hz=10., protocol={}, geometry=dict(effective_radial_clearance_mm=.199),
                sources={'example.py': hashlib.sha256(source.read_bytes()).hexdigest()})


def write_fixture(directory, manifest):
    (directory/'study.json').write_text(json.dumps(manifest))
    for attempt in manifest['attempts']:
        if attempt['status'] == 'complete':
            folder=directory/attempt['folder'];folder.mkdir(exist_ok=True)
            (folder/'trajectory.json').write_text(json.dumps(attempt))


class MechanicsOutputAuditTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory()
        self.directory=Path(self.temporary.name)
        self.manifest=fixture(self.directory)

    def tearDown(self):
        self.temporary.cleanup()

    def validate(self):
        write_fixture(self.directory,self.manifest)
        return validate_study(self.directory)

    def test_pending_and_incomplete_files_are_not_opened(self):
        self.manifest['attempts'].append(dict(self.manifest['case_plan']['cases'][1],
                                              status='running',folder='not-created-yet'))
        result=self.validate()
        self.assertTrue(result['passed'],result['errors'])
        self.assertEqual(result['planned_cases'],24)
        self.assertEqual(result['completed_cases_checked'],1)
        self.assertEqual(result['incomplete_attempts_not_opened'],1)
        self.assertEqual(result['pending_cases'],23)
        self.assertFalse(any('not-created' in r['path'] for r in result['checked_files']))

    def test_duplicate_plan_and_complete_attempts_are_rejected(self):
        self.manifest['case_plan']['cases'].append(copy.deepcopy(self.manifest['case_plan']['cases'][0]))
        self.manifest['attempts'].append(copy.deepcopy(self.manifest['attempts'][0]))
        codes={r['code'] for r in self.validate()['errors']}
        self.assertIn('plan_unique_cases',codes)
        self.assertIn('duplicate_completed_condition',codes)

    def test_archive_and_robot_material_tampering_are_detected(self):
        (self.directory/'source'/'example.py').write_text('ARCHIVED = False\n')
        self.manifest['attempts'][0]['material']['runtime_materials']['robot'][0][0][0]=1.
        codes={r['code'] for r in self.validate()['errors']}
        self.assertIn('archive_source_sha256',codes)
        self.assertIn('material_runtime_friction',codes)

    def test_unmatched_probe_cannot_become_negative_label(self):
        self.manifest['attempts'][0]['probes'][0]['safe_recovery']=False
        codes={r['code'] for r in self.validate()['errors']}
        self.assertIn('unknown_recovery_became_negative',codes)

    def test_exclusion_requires_matching_failed_aligned_control(self):
        control=self.manifest['attempts'][0]
        tilted=next(c for c in self.manifest['case_plan']['cases'] if c['split_group_id']==control['split_group_id'] and c['tilt_amplitude_deg']==2.)
        self.manifest['skipped_cases']=[dict(case_id=tilted['case_id'],control_case_id=control['case_id'],reason='aligned_control_failed')]
        self.assertIn('exclusion_control_not_failed',{r['code'] for r in self.validate()['errors']})
        control['final_retreat'].update(safe_recovery=False,grasp_retained=False,cleared=False,
                                        reason='grasp_retention_limit',recovery_censored=True)
        result=self.validate()
        self.assertTrue(result['passed'],result['errors'])
        self.assertEqual(result['skipped_cases_checked'],1)

    def test_deep_cross_checks_independently_saved_clock_and_force(self):
        rows=[];contacts=[]
        for i in range(3):
            t=i*.1;phase='initial' if i==0 else 'insert'
            rows.append(dict(time_s=t,physics_time_s=t,phase=phase,effective_pair_static_friction=.5,
                             effective_pair_dynamic_friction=.5,fx=1.,fy=0.,fz=0.,
                             normal_force_world_x_n=1.,normal_force_world_y_n=0.,normal_force_world_z_n=0.,
                             friction_force_world_x_n=0.,friction_force_world_y_n=0.,friction_force_world_z_n=0.))
            snapshot=dict(time_s=t,physics_time_s=t,phase=phase,normal_contact_count_all=1,friction_contact_count_all=0,
                          normal_contacts=[dict(normal_load_n=1.,normal_world=[1.,0.,0.])],friction_contacts=[])
            contacts.append(dict(time_s=t,physics_time_s=t,recording_time_s=t,phase=phase,contacts=snapshot))
        with (self.directory/'insertion.csv').open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        def save_contacts():
            with gzip.open(self.directory/'insertion_contacts.jsonl.gz','wt') as f:
                for value in contacts:f.write(json.dumps(value)+'\n')
        save_contacts();audit=Audit()
        _stream(audit,self.directory,'insertion',.1,self.manifest['attempts'][0])
        self.assertEqual(audit.errors,[])
        contacts[-1]['recording_time_s']+=.01
        contacts[-1]['contacts']['normal_contacts'][0]['normal_load_n']=2.
        save_contacts();audit=Audit()
        _stream(audit,self.directory,'insertion',.1,self.manifest['attempts'][0])
        codes={r['code'] for r in audit.errors}
        self.assertIn('contact_recording_clock',codes)
        self.assertIn('raw_contact_force_sum',codes)

    def test_motion_audit_rejects_old_depth_dependent_velocity(self):
        # At half of a 0.5 s velocity ramp the speed is 2.5 mm/s and
        # the integrated displacement is 0.234375 mm, for every normal depth.
        protocol=dict(stop_duration_s=.25,realign_duration_s=2.)
        first=dict(depth_mm=12.)
        row=dict(recovery_time_s=.5,phase='retreat',command_withdrawal_mm=.234375,
                 command_withdrawal_speed_mm_s=2.5,command_depth_mm=11.765625)
        audit=Audit();_motion_sample(audit,row,first,{'policy':'straight'},protocol,'sample',1)
        self.assertEqual(audit.errors,[])
        row['command_withdrawal_speed_mm_s']=3.
        audit=Audit();_motion_sample(audit,row,first,{'policy':'straight'},protocol,'sample',1)
        self.assertIn('motion_velocity',{r['code'] for r in audit.errors})

    def test_control_prefix_mismatch_is_an_analysis_limit(self):
        control=self.manifest['attempts'][0]
        tilted_case=next(c for c in self.manifest['case_plan']['cases'] if c['split_group_id']==control['split_group_id'] and c['tilt_amplitude_deg']==2.)
        tilted=dict(tilted_case,status='complete',metrics={'depth_gate_step':1})
        control['metrics']['depth_gate_step']=1
        rows=[]
        for i in range(2):
            row=sample();row.update(time_s=i*.1,physics_time_s=i*.1,phase='settle',reference_step=i,
                                    command_depth_mm=6.,command_pitch_deg=0.,command_offset_x_mm=0.,command_offset_y_mm=0.)
            rows.append({key:str(value) for key,value in row.items()})
        originals={control['case_id']:rows,tilted['case_id']:copy.deepcopy(rows)}
        result=control_prefix_checks([control,tilted],[control,tilted],originals,asdict(protocol()),deep=True)
        self.assertTrue(result[0]['analysis_prefix_matched'])
        originals[tilted['case_id']][0]['depth_mm']=str(float(originals[tilted['case_id']][0]['depth_mm'])+.1)
        result=control_prefix_checks([control,tilted],[control,tilted],originals,asdict(protocol()),deep=True)
        self.assertFalse(result[0]['analysis_prefix_matched'])
        self.assertEqual(result[0]['status'],'checked')
        self.assertIn('descriptive',result[0]['interpretation'])


if __name__=='__main__':
    unittest.main()
