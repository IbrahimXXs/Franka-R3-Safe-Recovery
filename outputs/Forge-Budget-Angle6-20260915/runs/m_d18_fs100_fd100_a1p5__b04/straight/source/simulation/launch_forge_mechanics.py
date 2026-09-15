"""Isolated launcher for depth, tilt, and friction mechanics experiments."""
import argparse
from datetime import datetime, timezone
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.forge_mechanics_plan import load_plan
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--case-plan', type=Path, default=ROOT/'experiments/forge_mechanics_pilot.json')
parser.add_argument('--output-dir', type=Path)
parser.add_argument('--config', type=Path, default=ROOT/'experiments/forge_phase2.json')
parser.add_argument('--physics-hz', type=int, choices=(120, 240, 480), default=240)
parser.add_argument('--seed', type=int, default=20260913)
parser.add_argument('--resume', action='store_true')
parser.add_argument('--phase', choices=('controls', 'all'), default='all')
parser.add_argument('--case-ids', nargs='+', help='Select cases without changing the frozen directory plan')
parser.add_argument('--max-new-attempts', type=int)
parser.add_argument('--stock-buffers', action='store_true')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
plan = load_plan(args.case_plan)
known = {c['case_id'] for c in plan['cases']}
if args.case_ids and (len(set(args.case_ids)) != len(args.case_ids) or not set(args.case_ids) <= known):
    parser.error('--case-ids must be distinct planned cases')
if not 0 <= args.seed < 2**32:
    parser.error('Seed must be in [0, 2**32)')
if args.max_new_attempts is not None and args.max_new_attempts < 1:
    parser.error('--max-new-attempts must be positive')
if args.output_dir is None:
    args.output_dir = ROOT/'outputs'/('Forge-Mechanics-'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S'))
if args.resume:
    if not (args.output_dir/'study.json').is_file():
        parser.error('--resume requires an existing study.json')
elif args.output_dir.exists():
    parser.error('Choose a new directory or use --resume')
args.radial_clearance_mm = .2
args.kit_args = (args.kit_args+' --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true').strip()
app = AppLauncher(args).app
import torch
import isaaclab.sim as sim_utils
from forge_mechanics import run

try:
    with torch.inference_mode():
        run(args, app)
except BaseException:
    import traceback
    import omni.kit.app
    traceback.print_exc()
    omni.kit.app.get_app().post_quit(1)
    raise
finally:
    sim = sim_utils.SimulationContext.instance()
    if sim is not None:
        sim.clear_all_callbacks(); sim.clear_instance(); sim.stop()
    app.close(wait_for_replicator=False)
