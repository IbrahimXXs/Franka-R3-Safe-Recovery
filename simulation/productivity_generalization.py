"""Collect a pre-frozen held-out design by calling all four existing policies unchanged."""
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import time
from forge_backend import Bench,resolve_physics_rate
from forge_experiment import save
from productivity_control import execute
from productivity_unloading import execute_verified
from productivity_dewedge import execute_dewedge
from research.forge_protocol import load_protocol
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_generalization import verify_lock,sha


def run(args,app):
    root=Path(__file__).resolve().parents[1];directory=args.output_dir.resolve()
    plan=json.loads(args.plan.read_text());verify_lock(root,plan)
    previous=root/plan['previous_experiment']
    frozen=json.loads((root/'experiments/productivity_control_calibration.json').read_text())
    if frozen!=json.loads((previous/'calibration.json').read_text()):raise ValueError('Frozen calibration changed')
    if json.loads(args.config.read_text())!=json.loads((previous/'source/experiments/forge_phase2.json').read_text()):raise ValueError('Protocol changed')
    unloading=UnloadingDesign(**plan['unloading_design']);dewedge=DewedgeDesign(**plan['dewedge_design'])
    p=load_protocol(args.config);args.seed=p.seed;rate=resolve_physics_rate(args)
    directory.mkdir(parents=True,exist_ok=False)
    old={str(f.resolve()):(f.stat().st_size,f.stat().st_mtime_ns) for f in (root/'outputs').rglob('*')
         if f.is_file() and directory not in f.resolve().parents}
    manifest=dict(schema=plan['schema'],status='running',frozen_plan_sha256=sha(args.plan),
        frozen_at_utc=plan['frozen_at_utc'],physics_hz=args.physics_hz,physics_rate_selection=rate,
        seed=p.seed,protocol=asdict(p),design=plan['design'],unloading_design=plan['unloading_design'],dewedge_design=plan['dewedge_design'],
        cases=plan['cases'],schedule=plan['schedule'],runs=[],previous_experiment=plan['previous_experiment'],
        policy_source_sha256=plan['policy_source_sha256'],condition_count=len(plan['cases']),planned_runs=len(plan['schedule']),
        no_retuning=True,clearance_variation=False,geometry_scope=plan['geometry_scope'],holdout_audit=plan['holdout_audit'],
        seed_note='Original preparation seed and solver priming. One execution of each policy per distinct condition; no duplicate repeatability controls. Reset mismatches are measured, not excluded.')
    files=[*plan['policy_source_sha256'],
        'research/productivity_generalization.py','research/productivity_generalization_report.py',
        'research/productivity_dewedge_report.py','research/productivity_unloading_report.py',
        'research/productivity_control_report.py','research/future_stall.py',
        'simulation/productivity_generalization.py','simulation/launch_productivity_generalization.py',
        'productivity_generalization.sh','tests/test_productivity_generalization.py','docs/productivity_generalization.md',str(args.plan.resolve().relative_to(root))]
    manifest['source_sha256']={f:sha(root/f) for f in files}
    for f in files:
        target=directory/'source'/f;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/f,target)
    save(directory/'calibration.json',frozen);save(directory/'frozen_plan.json',plan);save(directory/'experiment.json',manifest)
    bench=None;start=time.monotonic()
    try:
        bench=Bench(args,app);bench.protocol=p
        save(directory/'scene.json',bench.scene_info);save(directory/'config.json',bench.cfg.to_dict())
        for filename in ('scene.json','config.json'):
            if json.loads((directory/filename).read_text())!=json.loads((previous/filename).read_text()):raise ValueError('Scene/configuration changed: '+filename)
        manifest['exact_previous_scene_and_config_match']=True
        bench.prepare(p.seed);manifest['initial_solver_priming']=True
        for i,spec in enumerate(plan['schedule']):
            # Fail closed if any policy/configuration is edited during the long collection.
            for f,digest in plan['policy_source_sha256'].items():
                if sha(root/f)!=digest:raise ValueError('Frozen source changed during collection: '+f)
            if sha(args.plan)!=manifest['frozen_plan_sha256']:raise ValueError('Held-out plan changed during collection')
            c=next(c for c in plan['cases'] if c['case_id']==spec['case_id'])
            rid=f"{c['case_id']}/repeat0/{spec['policy']}";rd=directory/rid;rd.mkdir(parents=True,exist_ok=False)
            print(f"[generalization] {i+1}/{len(plan['schedule'])} {rid}",flush=True)
            if spec['policy']=='dewedge':metrics=execute_dewedge(bench,c['case'],frozen,unloading,dewedge,rd/'trajectory.csv')
            elif spec['policy']=='axial':metrics=execute_verified(bench,c['case'],frozen,unloading,rd/'trajectory.csv')
            else:metrics=execute(bench,c['case'],spec['policy'],frozen,rd/'trajectory.csv')
            events=metrics.pop('events');save(rd/'events.json',events)
            result=dict(**spec,run_id=rid,log=rid+'/trajectory.csv',family=c['family'],severity=c['severity'],ramp_onset_mm=c['ramp_onset_mm'],
                **{k:v for k,v in metrics.items() if k!='policy'})
            save(rd/'metrics.json',result);manifest['runs'].append(result);save(directory/'experiment.json',manifest)
            print(f"[generalization] {metrics['outcome']}; depth={metrics['final_depth_mm']:.3f} stall={metrics['stalled']} recoveries={metrics['intervention_count']} ready={sum(e.get('unloading_status')=='verified' for e in events)} wrist={metrics['max_wrist_force_n']:.3f} N load={metrics['max_normal_load_n']:.3f} N",flush=True)
        manifest['status']='complete'
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));raise
    finally:
        manifest['elapsed_seconds']=time.monotonic()-start
        manifest['preexisting_outputs_checked']=len(old)
        manifest['preexisting_outputs_changed']=[f for f,stat in old.items() if not Path(f).is_file() or (Path(f).stat().st_size,Path(f).stat().st_mtime_ns)!=stat]
        save(directory/'experiment.json',manifest)
        if bench is not None:bench.close()
    from research.productivity_generalization_report import analyze
    analyze(directory)
