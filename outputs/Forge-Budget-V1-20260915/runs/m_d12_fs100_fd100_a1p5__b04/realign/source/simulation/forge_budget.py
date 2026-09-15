"""One prospective low-budget branch in a fresh, unchanged mechanics scene.

This reuses the mechanics reference and recovery controllers. The only changed
operational protocol field is force_budget_n; no forces are clipped or edited.
Each policy starts in a separate process. Pairing is checked offline from the
entire observed reference, never inferred from seed or process isolation.
"""
from dataclasses import asdict, replace
import csv
import hashlib
from pathlib import Path
import time

from forge_experiment import save
from forge_mechanics import (SOURCE_PATHS as MECHANICS_SOURCES, reference, recovery)
from forge_mechanics_backend import MechanicsBench
from forge_mechanics_recovery import MOTION
from research.forge_protocol import load_protocol, radial_clearance
from research.forge_mechanics_metrics import mechanics_reference_metrics
from research.forge_mechanics_protocol import reference_quality
from research.forge_budget import load_plan, reference_eligible, evidence, classify

ROOT = Path(__file__).resolve().parents[1]
EXTRA_SOURCES = ('research/forge_budget.py', 'simulation/forge_budget.py',
                 'simulation/launch_forge_budget.py', 'simulation/run_budget_study.py',
                 'forge_budget.sh')
SOURCE_PATHS = (*MECHANICS_SOURCES, *(ROOT/p for p in EXTRA_SOURCES))


def read_rows(path):
    def value(text):
        if text == '': return None
        if text in ('True', 'False'): return text == 'True'
        try: return float(text)
        except ValueError: return text
    with Path(path).open() as stream:
        return [{k:value(v) for k,v in row.items()} for row in csv.DictReader(stream)]


def actuator_readback(bench):
    """Read actuator limits without changing them or treating them as forces."""
    robot = bench.env._robot
    result = dict(joint_names=robot.joint_names,
                  meaning='Drive parameters, not measured finger normal force; no grip force sensor was added.',
                  hand_position_target=0., gripper_command_changed=False,
                  rigid_plastic_damage_model=False)
    view = robot.root_physx_view
    for key, method in (('effort_limits', 'get_dof_max_forces'),
                        ('stiffnesses', 'get_dof_stiffnesses'),
                        ('dampings', 'get_dof_dampings')):
        try:
            result[key] = getattr(view, method)().tolist()
        except (AttributeError, RuntimeError, TypeError) as error:
            result[key] = None
            result[key+'_unavailable'] = str(error)
    return result


def run(args, app):
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True, exist_ok=False)
    plan = load_plan(args.case_plan)
    condition = next(c for c in plan['conditions'] if c['condition_id'] == args.condition_id)
    case = condition['case']
    p = replace(load_protocol(args.config), seed=args.seed,
                force_budget_n=condition['force_budget_n'],
                torque_budget_nm=condition['torque_budget_nm']).validate()
    start = time.monotonic()
    record = dict(schema='Forge-budget-branch-v1', status='running',
                  condition=condition, policy=args.policy, physics_hz=args.physics_hz,
                  seed=args.seed, protocol=asdict(p),
                  budget_definition=plan['budget_definition'],
                  recovery_motion=MOTION,
                  preparation=dict(warmup_pair=[.75,.75], warmup_seed=args.seed,
                                   recorded_reset_seed=args.seed,
                                   budget_active_during_prepare=False),
                  sources={}, reference=None, recovery=None, outcome=None)
    for source in SOURCE_PATHS:
        relative = source.relative_to(ROOT)
        data = source.read_bytes()
        record['sources'][str(relative)] = hashlib.sha256(data).hexdigest()
        target = directory/'source'/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
    save(directory/'run.json', record)
    bench = None
    try:
        bench = MechanicsBench(args, app)
        p = replace(p, effective_radial_clearance_mm=
                    bench.scene_info['conservative_radial_clearance_mm']).validate()
        bench.protocol = p
        record['effective_protocol'] = asdict(p)
        record['geometry'] = dict(socket_mesh_sha256=bench.scene_info['socket_mesh_sha256'],
                                  effective_radial_clearance_mm=radial_clearance(p))
        save(directory/'config.json', bench.cfg.to_dict())
        save(directory/'scene.json', bench.scene_info)
        record['actuator_readback'] = actuator_readback(bench)
        # Identical preparation to the successful isolated mechanics studies.
        bench.set_case_material(.75, .75)
        bench.prepare(args.seed)
        record['material'] = bench.set_case_material(case['pair_static_friction'],
                                                     case['pair_dynamic_friction'])
        save(directory/'run.json', record)
        print(f'Budget branch: {condition["condition_id"]} / {args.policy}', flush=True)
        rows, reason, state = reference(bench, case, args.seed, directory)
        metrics = {**mechanics_reference_metrics(rows, case, bench.dt), **state,
                   **reference_quality(rows, p, reason)}
        reference_evidence = evidence(rows, p, bench.dt, segment='reference')
        metrics['budget_signals_finite'] = reference_evidence['budget_signals_finite']
        record['reference'] = dict(metrics=metrics,
                                    evidence=reference_evidence,
                                    trajectory='insertion.csv', contacts='insertion_contacts.jsonl.gz')
        save(directory/'run.json', record)
        if reference_eligible(metrics):
            recovery_metrics = recovery(bench, rows, args.policy, directory, 'recovery',
                                        prefix_valid=metrics['numerically_valid'])
            samples = read_rows(directory/'recovery.csv')
            recovery_evidence = evidence(samples, p, bench.dt, segment='recovery')
            recovery_metrics['budget_signals_finite'] = recovery_evidence['budget_signals_finite']
            record['recovery'] = dict(metrics=recovery_metrics,
                                       evidence=recovery_evidence,
                                       trajectory='recovery.csv', contacts='recovery_contacts.jsonl.gz')
        record['outcome'] = classify(record)
        record['status'] = 'complete'
        record['termination_response'] = dict(
            command_stream_ended=True, post_trigger_physics_steps=0,
            hardware_braking_validated=False,
            explanation='The triggering sample is retained. No new task command is issued after detection; simulator shutdown is not a physical braking experiment.')
        record['material_after_run'] = bench.material_state
        print(f'  reference={reason}; outcome={record["outcome"]}', flush=True)
    except BaseException as error:
        record['status'] = 'interrupted' if isinstance(error, KeyboardInterrupt) else 'error'
        record['error'] = repr(error)
        raise
    finally:
        record['elapsed_wall_seconds'] = time.monotonic()-start
        for name in ('insertion.csv','insertion_contacts.jsonl.gz','recovery.csv','recovery_contacts.jsonl.gz',
                     'config.json','scene.json'):
            if (directory/name).exists():
                record.setdefault('artifact_sha256', {})[name] = hashlib.sha256((directory/name).read_bytes()).hexdigest()
        save(directory/'run.json', record)
        if bench is not None: bench.close()
