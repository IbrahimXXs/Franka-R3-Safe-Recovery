"""Budget audit rejects altered evidence and never opens running branches."""
import copy
import csv
from dataclasses import asdict, replace
import gzip
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from research.forge_budget import make_plan, make_angle6_plan, evidence, classify
from research.forge_mechanics_plan import MechanicsState
from research.forge_mechanics_metrics import mechanics_reference_metrics
from research.forge_mechanics_protocol import reference_quality
from research.forge_protocol import protocol
from simulation.run_budget_study import SOURCE_FILES
from simulation.validate_mechanics_output import Audit
from simulation.validate_budget_output import budget_checks, validate_study, _same, _pairs
from tests.test_forge_protocol import sample


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, allow_nan=False))


def rows_for(forces, condition=None):
    condition = condition or make_plan()['conditions'][0]
    case = condition['case']; state = MechanicsState(case); dt = 1/condition['physics_hz']
    rows = []
    for i, force in enumerate(forces):
        row = sample(i, dt)
        row.pop('recovery_time_s')
        command = state.command(i*dt)
        row.update(phase=command['phase'] if i else 'initial', time_s=i*dt, physics_time_s=i*dt,
                   depth_mm=-10., tilt_deg=0., command_depth_mm=command['depth_mm'],
                   command_pitch_deg=command['pitch_deg'], command_tilt_deg=abs(command['pitch_deg']),
                   command_offset_x_mm=0., command_offset_y_mm=0., reference_step=i,
                   wrist_force_n=force, wrist_raw_child_0=force,
                   effective_pair_static_friction=case['pair_static_friction'],
                   effective_pair_dynamic_friction=case['pair_dynamic_friction'])
        state.observe(i*dt, -10., True, True, physics_step=i)
        row.update(state.as_dict())
        for kind in ('normal','friction'):
            for axis in 'xyz':row[f'{kind}_force_world_{axis}_n']=0.
        rows.append(row)
    return rows


def fixture(directory, plan=None):
    plan = make_plan() if plan is None else plan
    condition = plan['conditions'][0]; case = condition['case']
    base = protocol(); p = replace(base, seed=condition['seed'], force_budget_n=4., torque_budget_nm=1.)
    effective = replace(p, effective_radial_clearance_mm=.199)
    rows = rows_for([0.,4.,4.1], condition); dt=1/240
    state_keys = MechanicsState(case).as_dict()
    metrics = {**mechanics_reference_metrics(rows,case,dt),
               **{k:rows[-1][k] for k in state_keys},
               **reference_quality(rows,effective,'operational_budget_exceeded'), 'budget_signals_finite':True}
    material = dict(requested_pair_static_friction=1., requested_pair_dynamic_friction=1.,
                    effective_pair_static_friction=1., effective_pair_dynamic_friction=1.,
                    runtime_materials={name:[[[mu,mu,0.]]] for name,mu in [('hole',1.25),('peg',.75),('robot',.75)]},
                    bindings={name:dict(material_path='/'+name,resolved_friction_combine_mode='average',
                                       bound_before_physics_initialization=True,usd_static_friction=mu,
                                       usd_dynamic_friction=mu) for name,mu in [('hole',1.25),('peg',.75)]})
    folder = directory/'runs'/'closed'
    sources = {}
    for relative in SOURCE_FILES:
        path=directory/'source'/relative;path.parent.mkdir(parents=True,exist_ok=True);path.write_text('# archived fixture\n')
        target=folder/'source'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(path.read_bytes())
        sources[relative]=sha(path)
    save(directory/'case_plan.json',plan);save(directory/'base_protocol.json',asdict(base))
    save(folder/'config.json',dict(sim=dict(dt=dt)))
    save(folder/'scene.json',dict(conservative_radial_clearance_mm=.199,socket_mesh_sha256='mesh'))
    with (folder/'insertion.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    with gzip.open(folder/'insertion_contacts.jsonl.gz','wt') as stream:
        for row in rows:
            snapshot=dict(time_s=row['time_s'],physics_time_s=row['physics_time_s'],phase=row['phase'],
                          normal_contacts=[],friction_contacts=[],normal_contact_count_all=0,friction_contact_count_all=0)
            value=dict(time_s=row['time_s'],physics_time_s=row['physics_time_s'],recording_time_s=row['time_s'],
                       phase=row['phase'],contacts=snapshot)
            stream.write(json.dumps(value)+'\n')
    record=dict(schema='Forge-budget-branch-v1',status='complete',condition=condition,policy='straight',
                physics_hz=240,seed=condition['seed'],protocol=asdict(p),effective_protocol=asdict(effective),
                geometry=dict(socket_mesh_sha256='mesh',effective_radial_clearance_mm=.199),sources=sources,
                budget_definition=plan['budget_definition'],material=material,material_after_run=material,
                preparation=dict(budget_active_during_prepare=False),
                termination_response=dict(command_stream_ended=True,post_trigger_physics_steps=0,hardware_braking_validated=False),
                reference=dict(metrics=metrics,evidence=evidence(rows,effective,dt),trajectory='insertion.csv',
                               contacts='insertion_contacts.jsonl.gz'),recovery=None)
    record['outcome']=classify(record)
    record['artifact_sha256']={name:sha(folder/name) for name in ('config.json','scene.json','insertion.csv','insertion_contacts.jsonl.gz')}
    save(folder/'run.json',record)
    attempt=dict(condition_id=condition['condition_id'],policy='straight',status='complete',folder='runs/closed',
                 run_sha256=sha(folder/'run.json'),outcome=record['outcome'])
    study=dict(schema='Forge-budget-v1',status='paused',case_plan=plan,sources=sources,
               base_protocol_sha256=sha(directory/'base_protocol.json'),attempts=[attempt])
    save(directory/'study.json',study)
    return study,record,rows,effective


class BudgetOutputAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.directory=Path(self.temp.name)
        self.study,self.record,self.rows,self.protocol=fixture(self.directory)

    def tearDown(self):self.temp.cleanup()

    def rewrite(self):
        save(self.directory/'runs/closed/run.json',self.record)
        self.study['attempts'][0]['run_sha256']=sha(self.directory/'runs/closed/run.json')
        save(self.directory/'study.json',self.study)

    def test_closed_early_failure_passes_deep_and_running_files_not_opened(self):
        self.study['attempts'].append(dict(condition_id=self.study['case_plan']['conditions'][1]['condition_id'],
                                           policy='straight',status='running',folder='never-created'))
        self.rewrite();result=validate_study(self.directory,deep=True)
        self.assertTrue(result['passed'],result['errors'])
        self.assertEqual(result['completed_branches_checked'],1)
        self.assertEqual(result['incomplete_branches_not_opened'],1)
        self.assertEqual(result['actual_branch_results'][0]['first_reference_budget_crossing']['index'],2)

    def test_equality_is_allowed_but_later_step_after_crossing_rejected(self):
        audit=Audit();e=evidence(self.rows,self.protocol,1/240)
        budget_checks(audit,self.rows,e,self.record['reference']['metrics'],self.protocol,1/240,'reference','case')
        self.assertEqual(audit.errors,[])
        rows=rows_for([0.,4.1,3.]);audit=Audit();e=evidence(rows,self.protocol,1/240)
        budget_checks(audit,rows,e,self.record['reference']['metrics'],self.protocol,1/240,'reference','case')
        self.assertIn('steps_after_budget_crossing',{x['code'] for x in audit.errors})

    def test_initial_crossing_is_observation_without_executed_step(self):
        rows=rows_for([4.1]);e=evidence(rows,self.protocol,1/240)
        m=dict(self.record['reference']['metrics']);m.update(max_wrist_force_n=4.1)
        audit=Audit();budget_checks(audit,rows,e,m,self.protocol,1/240,'reference','initial')
        self.assertEqual(audit.errors,[])
        self.assertFalse(e['first_budget_crossing']['executed_step'])
        self.assertEqual(e['executed_step_count'],0)
        self.assertIsNone(e['phase_metrics']['initial']['peak_wrist_force_mean100ms_n'])

    def test_tampered_first_crossing_metadata_rejected(self):
        self.record['reference']['evidence']['first_budget_crossing']['index']=1
        self.rewrite();result=validate_study(self.directory,deep=True)
        self.assertIn('recomputed_budget_evidence',{x['code'] for x in result['errors']})

    def test_changed_artifact_rejected_even_without_deep(self):
        path=self.directory/'runs/closed/insertion.csv';path.write_text(path.read_text()+'\n')
        result=validate_study(self.directory)
        self.assertIn('artifact_sha256',{x['code'] for x in result['errors']})

    def test_not_tested_alternative_has_no_invented_result(self):
        self.study['attempts'].append(dict(condition_id=self.study['case_plan']['conditions'][0]['condition_id'],
            policy='realign',status='not_tested',reason='reference_not_eligible_in_straight_branch',source_run='runs/closed'))
        self.rewrite();result=validate_study(self.directory,deep=True)
        self.assertTrue(result['passed'],result['errors'])
        self.assertEqual(result['not_tested_branches_checked'],1)
        self.study['attempts'][-1]['outcome']=dict(recovery_safe=False)
        self.rewrite();result=validate_study(self.directory)
        self.assertIn('not_tested_has_no_policy_result',{x['code'] for x in result['errors']})

    def test_null_and_boolean_are_not_zero_and_one(self):
        self.assertFalse(_same(None,0.));self.assertFalse(_same(False,0.));self.assertFalse(_same(1.,True))

    def test_independent_prefix_mismatch_keeps_actual_branch_outcomes(self):
        condition=self.study['case_plan']['conditions'][0];identity=condition['condition_id']
        record=copy.deepcopy(self.record)
        record['reference']['metrics'].update(reference_complete=True,depth_gate_passed=True,
            reference_within_budget=True,force_budget_exceeded=False)
        record['outcome']={'outcome':'clear_safe'}
        a=rows_for([0.,0.]);b=copy.deepcopy(a);b[1]['depth_mm']+=.1
        result=_pairs([condition],{(identity,'straight'):record,(identity,'realign'):record},
                      {(identity,'straight'):a,(identity,'realign'):b},True)[0]
        self.assertEqual(result['status'],'checked')
        self.assertFalse(result['paired_policy_comparison_eligible'])
        self.assertEqual(result['straight_actual_outcome'],{'outcome':'clear_safe'})

    def test_second_canonical_angle_plan_passes_deep_audit(self):
        study, record, rows, p = fixture(self.directory, make_angle6_plan())
        result = validate_study(self.directory, deep=True)
        self.assertTrue(result['passed'], result['errors'])
        self.assertEqual(result['planned_conditions'], 6)
        self.assertEqual(result['planned_policy_branches'], 12)
        self.assertEqual(result['completed_branches_checked'], 1)

    def test_second_plan_parameter_or_numeric_type_tampering_rejected(self):
        for key, value in (('physics_hz', 480), ('physics_hz', 240.)):
            with self.subTest(key=key, value=value):
                plan = make_angle6_plan()
                plan['conditions'][-1][key] = value
                fixture(self.directory, plan)
                result = validate_study(self.directory)
                self.assertIn('canonical_plan', {error['code'] for error in result['errors']})

    def test_manifest_plan_type_disagreement_rejected(self):
        study, record, rows, p = fixture(self.directory, make_angle6_plan())
        study['case_plan']['conditions'][-1]['physics_hz'] = 240.
        save(self.directory/'study.json', study)
        result = validate_study(self.directory)
        self.assertIn('plan_file_agreement', {error['code'] for error in result['errors']})


if __name__=='__main__':unittest.main()
