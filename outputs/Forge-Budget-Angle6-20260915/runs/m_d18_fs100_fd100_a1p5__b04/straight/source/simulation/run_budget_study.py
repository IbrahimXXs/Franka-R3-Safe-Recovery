"""Run each budget condition/policy in an isolated simulator process.

No deterministic retries or successful-case filtering. Failed references remain
in the denominator; their alternative policy is explicitly not tested. Resume
requires an unchanged plan, base protocol and executable source snapshot.
"""
import argparse
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.forge_budget import SCHEMA, load_plan, reference_eligible

SOURCE_FILES = (
    'simulation/forge_mechanics.py', 'simulation/launch_forge_mechanics.py',
    'simulation/forge_mechanics_backend.py', 'research/forge_mechanics_contacts.py',
    'simulation/forge_mechanics_recovery.py', 'research/forge_mechanics_motion.py',
    'research/forge_mechanics_plan.py', 'research/forge_mechanics_metrics.py',
    'research/forge_mechanics_protocol.py', 'simulation/forge_backend.py',
    'simulation/forge_experiment.py', 'simulation/contact.py', 'research/forge_protocol.py',
    'research/phase2_protocol.py', 'research/forge_gap_metrics.py', 'research/forge_geometry.py',
    'research/forge_events.py', 'research/future_stall.py', 'research/forge_gap_study.py',
    'research/forge_collection.py', 'research/phase2b.py', 'research/forge_budget.py',
    'simulation/forge_budget.py', 'simulation/launch_forge_budget.py',
    'simulation/run_budget_study.py', 'forge_budget.sh',
)


def save(path, value):
    temporary = path.with_suffix(path.suffix+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+'\n')
    temporary.replace(path)


def now():
    return datetime.now(timezone.utc).isoformat()


def run(args):
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=args.resume)
    with (directory/'.run.lock').open('a') as lock:
        try: fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError: raise RuntimeError('Another process owns this study') from None
        return execute(args, directory)


def execute(args, directory):
    plan = load_plan(args.case_plan)
    sources = {p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCE_FILES}
    frozen = dict(schema=SCHEMA, case_plan=plan, sources=sources,
                  base_protocol_sha256=hashlib.sha256(args.config.read_bytes()).hexdigest(),
                  simulation_options=dict(device=args.device, headless=True, stock_buffers=False))
    path = directory/'study.json'
    if args.resume:
        study = json.loads(path.read_text())
        for key, value in frozen.items():
            if study.get(key) != value: raise ValueError(f'Cannot resume: frozen {key} differs')
        if any(a['status'] not in ('complete','not_tested') for a in study['attempts']):
            raise ValueError('Incomplete invocation retained. Diagnose it and choose a new study; automatic retries are disabled.')
        study['resume_count'] = study.get('resume_count',0)+1
    else:
        study = dict(frozen, status='running', started_utc=now(), attempts=[],
                     elapsed_wall_seconds=0., seed_interpretation='One fixed seed, exploratory conditions; no independent stochastic replications.',
                     process_design='One newly initialized process per condition and policy; compare full observed prefixes afterwards.',
                     control_gate='Every physical/budget condition is attempted independently; control outcomes do not delete tilted cases.',
                     selection_provenance=plan.get('selection_provenance',
                         'Prospective budgets and conditions selected using prior 2026-09-15 mechanics studies. They are not a blinded holdout.'))
        save(directory/'case_plan.json', plan)
        (directory/'base_protocol.json').write_bytes(args.config.read_bytes())
        for relative in sources:
            target=directory/'source'/relative
            target.parent.mkdir(parents=True,exist_ok=True)
            target.write_bytes((ROOT/relative).read_bytes())
    study['status']='running'
    save(path,study)
    start=time.monotonic(); previous=study['elapsed_wall_seconds']; launched=0
    try:
        for condition in plan['conditions']:
            condition_id=condition['condition_id']
            for policy in plan['policies']:
                key=(condition_id,policy)
                if any((a['condition_id'],a['policy'])==key for a in study['attempts']): continue
                straight=next((a for a in study['attempts'] if a['condition_id']==condition_id and a['policy']=='straight'),None)
                if policy=='realign' and straight is not None:
                    original=json.loads((directory/straight['folder']/'run.json').read_text())
                    if not reference_eligible(original['reference']['metrics']):
                        study['attempts'].append(dict(condition_id=condition_id,policy=policy,status='not_tested',
                            reason='reference_not_eligible_in_straight_branch',source_run=straight['folder'],
                            interpretation='No alternative-policy trajectory was executed or assigned a recovery label.'))
                        save(path,study)
                        continue
                if args.max_invocations is not None and launched>=args.max_invocations:
                    study['status']='paused'; return study
                folder=Path('runs')/condition_id/policy
                log=Path('logs')/(condition_id+'__'+policy+'.log')
                (directory/log).parent.mkdir(parents=True,exist_ok=True)
                command=['bash',str(ROOT/'forge_budget.sh'),'--headless','--device',args.device,
                         '--case-plan',str((directory/'case_plan.json').resolve()),
                         '--config',str((directory/'base_protocol.json').resolve()),
                         '--condition-id',condition_id,'--policy',policy,
                         '--output-dir',str(directory/folder)]
                attempt=dict(condition_id=condition_id,policy=policy,status='running',folder=str(folder),
                             log=str(log),command=command,started_utc=now())
                study['attempts'].append(attempt);save(path,study)
                print(f'Starting {condition_id} / {policy} at {attempt["started_utc"]}',flush=True)
                invocation_start=time.monotonic()
                with (directory/log).open('w') as stream:
                    process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,
                                             start_new_session=True)
                    attempt['pid']=process.pid;save(path,study)
                    try: return_code=process.wait()
                    except BaseException:
                        os.killpg(process.pid,signal.SIGINT)
                        try: process.wait(timeout=15)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid,signal.SIGKILL);process.wait()
                        raise
                attempt.update(return_code=return_code,elapsed_wall_seconds=time.monotonic()-invocation_start,
                               finished_utc=now())
                result_path=directory/folder/'run.json'
                result=json.loads(result_path.read_text()) if result_path.exists() else None
                if return_code!=0 or not result or result.get('status')!='complete':
                    attempt['status']='error';raise RuntimeError(f'Branch did not complete: {condition_id}/{policy}; see {log}')
                if result['sources']!=sources: raise RuntimeError('Child source snapshot differs from frozen parent')
                attempt.update(status='complete',outcome=result['outcome'],
                               run_sha256=hashlib.sha256(result_path.read_bytes()).hexdigest())
                save(path,study);launched+=1
                print(f'Completed {condition_id} / {policy}: {result["outcome"]["outcome"]}',flush=True)
        study['status']='complete'
        study['finished_utc']=now()
    except BaseException as error:
        study['status']='interrupted' if isinstance(error,KeyboardInterrupt) else 'error'
        study['error']=repr(error)
        for attempt in study['attempts']:
            if attempt['status']=='running': attempt['status']='interrupted'
        raise
    finally:
        study['elapsed_wall_seconds']=previous+time.monotonic()-start
        save(path,study)
    return study


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case-plan',type=Path,default=ROOT/'experiments/forge_budget_v1.json')
    parser.add_argument('--config',type=Path,default=ROOT/'experiments/forge_phase2.json')
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--resume',action='store_true')
    parser.add_argument('--device',default='cuda:0')
    parser.add_argument('--max-invocations',type=int)
    args=parser.parse_args()
    if args.max_invocations is not None and args.max_invocations<1:parser.error('max-invocations must be positive')
    result=run(args)
    print(f'Budget study status: {result["status"]}',flush=True)


if __name__=='__main__':main()
