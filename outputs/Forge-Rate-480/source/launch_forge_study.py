"""Controlled FORGE characterization and recovery validation; no robot connection."""
import argparse
from pathlib import Path
from datetime import datetime, timezone
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--config', type=Path, default=Path(__file__).resolve().parents[1]/'experiments/forge_phase2.json')
parser.add_argument('--physics-hz', type=int, choices=(120,240,480,960), default=120)
parser.add_argument('--stock-buffers', action='store_true', help='Use upstream GPU contact allocation sizes for an A/B check')
parser.add_argument('--mode', choices=('baseline','pilot','calibration'), default='baseline')
parser.add_argument('--repeats', type=int, default=3)
parser.add_argument('--slots', type=int, nargs='+', help='Selected randomized-plan slots for a small comparison')
parser.add_argument('--checkpoints', type=float, nargs='*', default=[], metavar='MM', help='Actual depths at which to replay and probe both recoveries')
parser.add_argument('--seed', type=int, help='Override the protocol seed')
parser.add_argument('--exit-after', action='store_true')
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if not args.config.is_file():parser.error('Protocol config does not exist')
if args.repeats<1: parser.error('--repeats must be positive')
if args.slots and (args.mode!='pilot' or min(args.slots)<0 or len(set(args.slots))!=len(args.slots)):
    parser.error('--slots requires pilot mode and unique nonnegative slot numbers')
if sorted(set(args.checkpoints))!=args.checkpoints or any(not 0<d<=20 for d in args.checkpoints):
    parser.error('--checkpoints must be increasing depths in (0,20] mm')
if args.output_dir is None:
    args.output_dir=Path('outputs')/('Forge-Controlled-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S-%fZ'))
if args.output_dir.exists(): parser.error('Choose a new output directory; previous data are never overwritten')
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
