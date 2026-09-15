"""Controlled FORGE characterization and recovery validation; no robot connection."""
import argparse
from pathlib import Path
from datetime import datetime, timezone
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'experiments/forge_phase2.json')
parser.add_argument('--case-plan', type=Path, help='Explicit Phase 2B depth-dependent plan; requires collect mode')
parser.add_argument('--physics-hz', type=int, choices=(120,240,480,960), default=None,
                    help='Explicit experimental override; omitted uses the installed FORGE scene timestep')
parser.add_argument('--stock-buffers', action='store_true', help='Use upstream GPU contact allocation sizes for an A/B check')
parser.add_argument('--mode', choices=('baseline','pilot','calibration','collect'), default='baseline')
parser.add_argument('--resume', action='store_true', help='Resume a collect run in the same output directory')
parser.add_argument('--target-valid', type=int, help='Collect-mode quota override; default comes from the protocol (100)')
parser.add_argument('--max-new-attempts', type=int, help='Pause collection after this many new attempts; useful for batches')
parser.add_argument('--repeats', type=int, default=3)
parser.add_argument('--slots', type=int, nargs='+', help='Selected randomized-plan slots for a small comparison')
parser.add_argument('--checkpoints', type=float, nargs='*', default=None, metavar='MM', help='Recovery depths; collect mode defaults to the protocol depths (5,10,15,20)')
parser.add_argument('--seed', type=int, help='Override the protocol seed')
parser.add_argument('--exit-after', action='store_true')
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if args.case_plan is not None and (args.mode!='collect' or not args.case_plan.is_file() or args.target_valid is not None):
    parser.error('--case-plan requires collect mode, an existing plan, and no --target-valid override')
if not args.config.is_file():parser.error('Protocol config does not exist')
if args.repeats<1: parser.error('--repeats must be positive')
if args.seed is not None and not 0<=args.seed<2**32:
    parser.error('--seed must be in [0, 2**32) for reproducible NumPy/PhysX preparation')
if args.slots and (args.mode!='pilot' or min(args.slots)<0 or len(set(args.slots))!=len(args.slots)):
    parser.error('--slots requires pilot mode and unique nonnegative slot numbers')
if args.checkpoints is not None and (sorted(set(args.checkpoints))!=args.checkpoints or any(not 0<d<=20 for d in args.checkpoints)):
    parser.error('--checkpoints must be increasing depths in (0,20] mm')
if (args.resume or args.target_valid is not None or args.max_new_attempts is not None) and args.mode!='collect':
    parser.error('--resume, --target-valid and --max-new-attempts require --mode collect')
if any(v is not None and v<1 for v in (args.target_valid,args.max_new_attempts)):
    parser.error('Collection target and batch size must be positive')
if args.resume and (args.output_dir is None or not (args.output_dir/'study.json').is_file()):
    parser.error('--resume requires --output-dir pointing to an existing collection')
if args.output_dir is None:
    args.output_dir=Path('outputs')/('Forge-Controlled-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S-%fZ'))
if args.output_dir.exists() and not args.resume: parser.error('Choose a new output directory, or --resume an existing collection')
args.kit_args=(args.kit_args+' --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true').strip()
app=AppLauncher(args).app
import torch
import isaaclab.sim as sim_utils
from forge_experiment import run

try:
    with torch.inference_mode(): run(args,app)
except BaseException:
    import traceback
    import omni.kit.app
    traceback.print_exc()
    omni.kit.app.get_app().post_quit(1)
    raise
finally:
    sim=sim_utils.SimulationContext.instance()
    if sim is not None:
        sim.clear_all_callbacks()
        sim.clear_instance()
        sim.stop()
    app.close(wait_for_replicator=False)
