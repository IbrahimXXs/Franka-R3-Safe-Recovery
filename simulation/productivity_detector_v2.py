"""New nominal development data, frozen calibration, then paired detector comparison."""
from dataclasses import asdict
import json
from pathlib import Path
import shutil
import time
from forge_backend import Bench, resolve_physics_rate
from forge_experiment import save
from productivity_control import execute
from productivity_dewedge import execute_dewedge
from research.forge_protocol import load_protocol
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_detector_v2 import bind_detector
from research.productivity_detector_v2_design import verify
from research.productivity_detector_v2_calibration import calibrate
from research.productivity_generalization import sha


def run(args, app):
    root=Path(__file__).resolve().parents[1];directory=args.output_dir.resolve()
    plan=json.loads(args.plan.read_text());verify(root,plan)
    previous=root/plan['previous_experiment'];frozen=json.loads((root/'experiments/productivity_control_calibration.json').read_text())
    if frozen != json.loads((previous/'calibration.json').read_text()):raise ValueError('V1 detector changed')
    p=load_protocol(args.config);args.seed=p.seed;rate=resolve_physics_rate(args)
    unloading=UnloadingDesign(**plan['unloading_design']);dewedge=DewedgeDesign(**plan['dewedge_design'])
    directory.mkdir(parents=True,exist_ok=False)
    old={str(f.resolve()):(f.stat().st_size,f.stat().st_mtime_ns) for f in (root/'outputs').rglob('*')
         if f.is_file() and directory not in f.resolve().parents}
    manifest=dict(schema=plan['schema'],status='running',stage='reference',physics_hz=args.physics_hz,
        physics_rate_selection=rate,protocol=asdict(p),design=frozen['design'],unloading_design=asdict(unloading),
        dewedge_design=asdict(dewedge),cases=plan['cases'],runs=[],development_plan_sha256=sha(args.plan),
        previous_experiment=plan['previous_experiment'],no_final_generalization_test=True,
        seed=p.seed,seed_note='Original reset seed and solver priming. Paired state differences are measured, not assumed absent.')
    files=[*plan['policy_source_sha256'], 'research/productivity_generalization.py',
        'research/productivity_detector_v2.py','research/productivity_detector_v2_design.py',
        'research/productivity_detector_v2_calibration.py','research/productivity_detector_v2_report.py',
        'research/productivity_dewedge_report.py','research/productivity_unloading_report.py','research/productivity_control_report.py',
        'simulation/productivity_detector_v2.py','simulation/launch_productivity_detector_v2.py',
        'productivity_detector_v2.sh','docs/productivity_detector_v2.md','tests/test_productivity_detector_v2.py',
        str(args.plan.resolve().relative_to(root))]
    manifest['source_sha256']={f:sha(root/f) for f in files}
    for f in files:
        dest=directory/'source'/f;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/f,dest)
    save(directory/'calibration.json',frozen);save(directory/'development_plan.json',plan);save(directory/'experiment.json',manifest)
    bench=None;start=time.monotonic();config=None;v2=None
    try:
        bench=Bench(args,app);bench.protocol=p
        save(directory/'scene.json',bench.scene_info);save(directory/'config.json',bench.cfg.to_dict())
        for name in ('scene.json','config.json'):
            if json.loads((directory/name).read_text()) != json.loads((previous/name).read_text()):raise ValueError('Physics/scene drift: '+name)
        manifest['exact_previous_scene_and_config_match']=True
        bench.prepare(p.seed);manifest['initial_solver_priming']=True
        schedule=plan['nominal_schedule']+plan['comparison_schedule']
        for i,spec in enumerate(schedule):
            verify(root,plan)
            if sha(args.plan)!=manifest['development_plan_sha256']:raise ValueError('Development plan changed')
            # Detector, calibration and collection code cannot change after collection starts.
            for name in ('research/productivity_detector_v2.py','research/productivity_detector_v2_calibration.py',
                         'research/productivity_detector_v2_design.py','simulation/productivity_detector_v2.py'):
                if sha(root/name)!=manifest['source_sha256'][name]:raise ValueError('Development implementation changed: '+name)
            if spec['stage']=='comparison' and config is None:
                config=calibrate(directory)
                manifest['detector_v2_config_sha256']=sha(directory/'detector_v2_config.json')
                manifest['calibration_frozen_after_reference_runs']=len(manifest['runs'])
                manifest['stage']='comparison';save(directory/'experiment.json',manifest)
                v2=bind_detector(execute_dewedge,config,p.success_depth_mm)
                print('[detector-v2] CONFIG FROZEN '+json.dumps(config['selected_calibration_metrics']),flush=True)
            if config is not None and sha(directory/'detector_v2_config.json')!=manifest['detector_v2_config_sha256']:
                raise ValueError('Selected detector thresholds changed')
            c=next(c for c in plan['cases'] if c['case_id']==spec['case_id'])
            rid=f"{c['case_id']}/{spec['policy']}";rd=directory/rid;rd.mkdir(parents=True,exist_ok=False)
            print(f"[detector-v2] {i+1}/{len(schedule)} {rid} ({c['split']})",flush=True)
            if spec['policy']=='nominal':metrics=execute(bench,c['case'],'nominal',frozen,rd/'trajectory.csv')
            else:metrics=(execute_dewedge if spec['policy']=='v1' else v2)(bench,c['case'],frozen,unloading,dewedge,rd/'trajectory.csv')
            events=metrics.pop('events');save(rd/'events.json',events)
            result=dict(**spec,run_id=rid,log=rid+'/trajectory.csv',family=c['family'],group_id=c['group_id'],split=c['split'],
                **{k:v for k,v in metrics.items() if k!='policy'})
            save(rd/'metrics.json',result);manifest['runs'].append(result);save(directory/'experiment.json',manifest)
            print(f"[detector-v2] {metrics['outcome']}; depth={metrics['final_depth_mm']:.3f} recoveries={metrics['intervention_count']} verified={sum(e.get('unloading_status')=='verified' for e in events)}",flush=True)
        manifest['status']='complete'
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));raise
    finally:
        manifest['elapsed_seconds']=time.monotonic()-start;manifest['preexisting_outputs_checked']=len(old)
        manifest['preexisting_outputs_changed']=[f for f,st in old.items() if not Path(f).is_file() or (Path(f).stat().st_size,Path(f).stat().st_mtime_ns)!=st]
        save(directory/'experiment.json',manifest)
        if bench is not None:bench.close()
    from research.productivity_detector_v2_report import analyze
    analyze(directory)
