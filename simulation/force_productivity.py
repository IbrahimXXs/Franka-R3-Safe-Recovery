"""Collect force development, freeze selection, then run the fresh detector-only ablation."""
from dataclasses import asdict
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import time
from forge_backend import Bench, resolve_physics_rate
from forge_experiment import save
from productivity_control import execute
from productivity_dewedge import execute_dewedge
from research.forge_protocol import load_protocol
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.productivity_detector_v2 import bind_detector
from research.force_productivity_design import verify
from research.productivity_detector_v2_generalization import CONFIG, CONFIG_SHA
from research.force_productivity import bind_force
from research.force_productivity_analysis import freeze_force
from research.productivity_generalization import sha


def run(args,app):
    root=Path(__file__).resolve().parents[1];directory=args.output_dir.resolve()
    plan=json.loads(args.plan.read_text());verify(root,plan);plan_hash=sha(args.plan)
    previous=root/plan['previous_experiment']
    frozen=json.loads((root/'experiments/productivity_control_calibration.json').read_text())
    if frozen!=json.loads((previous/'calibration.json').read_text()):raise ValueError('V1 configuration drift')
    config=json.loads((root/CONFIG).read_text());p=load_protocol(args.config);args.seed=p.seed
    if args.config.resolve()!=root/'experiments/forge_phase2.json':raise ValueError('Use the frozen FORGE protocol')
    rate=resolve_physics_rate(args);unloading=UnloadingDesign(**plan['unloading_design']);dewedge=DewedgeDesign(**plan['dewedge_design'])
    v2=bind_detector(execute_dewedge,config,p.success_depth_mm)
    directory.mkdir(parents=True,exist_ok=False)
    old={str(f.resolve()):(f.stat().st_size,f.stat().st_mtime_ns) for f in (root/'outputs').rglob('*')
         if f.is_file() and directory not in f.resolve().parents}
    env=dict(python=sys.version,platform=platform.platform(),executable=sys.executable,
        packages={k:importlib.metadata.version(k) for k in ('numpy','scipy','torch','matplotlib')},
        nvidia_smi=subprocess.run(['nvidia-smi','--query-gpu=name,driver_version','--format=csv,noheader'],capture_output=True,text=True).stdout.strip())
    manifest=dict(schema=plan['schema'],status='running',collection_started_utc=datetime.now(timezone.utc).isoformat(),
        physics_hz=args.physics_hz,physics_rate_selection=rate,protocol=asdict(p),design=frozen['design'],
        unloading_design=asdict(unloading),dewedge_design=asdict(dewedge),cases=plan['cases'],runs=[],schedule=plan['schedule'],
        condition_plan_sha256=plan_hash,detector_v2_config_sha256=CONFIG_SHA,previous_experiment=plan['previous_experiment'],
        source_sha256=dict(plan['source_sha256']),seed=p.seed,environment=env,
        seed_note='Original reset seed and one solver priming reset; state matching audited, not assumed.')
    manifest['source_sha256'][str(args.plan.resolve().relative_to(root))]=plan_hash
    for f in manifest['source_sha256']:
        dest=directory/'source'/f;dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(root/f,dest)
    shutil.copy2(args.plan,directory/'condition_plan.json');shutil.copy2(root/CONFIG,directory/'detector_v2_config.json')
    save(directory/'calibration.json',frozen);save(directory/'experiment.json',manifest)
    save(directory/'preexisting_outputs.json',old)
    bench=None;start=time.monotonic();selected=None;force_lock=None
    force_runners={t:bind_force(execute_dewedge,t) for t in (1.,2.,3.,4.,5.)}
    assert all(f.__code__ is execute_dewedge.__code__ for f in (*force_runners.values(),v2))
    try:
        bench=Bench(args,app);bench.protocol=p
        save(directory/'scene.json',bench.scene_info);save(directory/'config.json',bench.cfg.to_dict())
        for name in ('scene.json','config.json'):
            if json.loads((directory/name).read_text())!=json.loads((previous/name).read_text()):raise ValueError('Physics/scene drift: '+name)
        manifest['exact_previous_scene_and_config_match']=True
        bench.prepare(p.seed);manifest['initial_solver_priming']=True
        for i,spec in enumerate(plan['schedule']):
            verify(root,plan)
            if sha(args.plan)!=plan_hash or sha(directory/'condition_plan.json')!=plan_hash:raise ValueError('Frozen plan changed')
            if sha(directory/'detector_v2_config.json')!=CONFIG_SHA:raise ValueError('Frozen configuration copy changed')
            if spec['stage']=='held_out' and selected is None:
                selected=freeze_force(directory,manifest,plan);force_lock=sha(directory/'force_detector_config.json')
                manifest.update(force_detector_config_sha256=force_lock,force_frozen_after_runs=len(manifest['runs']))
                save(directory/'experiment.json',manifest)
                print('[force-ablation] FORCE CONFIG FROZEN: '+str(selected['threshold_n'])+' N; SHA '+force_lock,flush=True)
            if force_lock and sha(directory/'force_detector_config.json')!=force_lock:raise ValueError('Selected force configuration changed')
            c=next(c for c in plan['cases'] if c['case_id']==spec['case_id'])
            rid=f"{c['split']}/{c['case_id']}/{spec['detector_id']}";rd=directory/rid;rd.mkdir(parents=True,exist_ok=False)
            print(f"[force-ablation] {i+1}/384 {rid} ({c['severity']})",flush=True)
            if spec['policy']=='nominal':metrics=execute(bench,c['case'],'nominal',frozen,rd/'trajectory.csv')
            else:
                threshold=spec['threshold_n'] if spec['split']=='development' else selected['threshold_n']
                runner=force_runners[threshold] if spec['policy']=='force' else v2
                metrics=runner(bench,c['case'],frozen,unloading,dewedge,rd/'trajectory.csv')
            events=metrics.pop('events');save(rd/'events.json',events)
            result=dict(**spec,run_id=rid,log=rid+'/trajectory.csv',family=c['family'],severity=c['severity'],
                group_id=c['group_id'],force_threshold_n=(spec['threshold_n'] if spec['split']=='development' else selected['threshold_n']) if spec['policy']=='force' else None,
                **{k:v for k,v in metrics.items() if k!='policy'})
            save(rd/'metrics.json',result);manifest['runs'].append(result);save(directory/'experiment.json',manifest)
            print(f"[force-ablation] {metrics['outcome']}; depth={metrics['final_depth_mm']:.3f} recoveries={metrics['intervention_count']}",flush=True)
        verify(root,plan);manifest['status']='complete'
    except BaseException as exc:
        manifest.update(status='failed',error=repr(exc));raise
    finally:
        manifest['elapsed_seconds']=time.monotonic()-start;manifest['preexisting_outputs_checked']=len(old)
        manifest['preexisting_outputs_changed']=[f for f,st in old.items() if not Path(f).is_file() or (Path(f).stat().st_size,Path(f).stat().st_mtime_ns)!=st]
        save(directory/'experiment.json',manifest)
        if bench is not None:bench.close()
    from research.force_productivity_report import analyze
    analyze(directory)
