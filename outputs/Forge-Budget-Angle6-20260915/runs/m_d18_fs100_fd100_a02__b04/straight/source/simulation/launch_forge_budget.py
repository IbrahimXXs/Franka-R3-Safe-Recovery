"""Fresh-process launcher for one force-budget condition and policy."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from research.forge_budget import load_plan
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--case-plan', type=Path, required=True)
parser.add_argument('--condition-id', required=True)
parser.add_argument('--policy', choices=('straight','realign'), required=True)
parser.add_argument('--output-dir', type=Path, required=True)
parser.add_argument('--config', type=Path, default=ROOT/'experiments/forge_phase2.json')
parser.add_argument('--stock-buffers', action='store_true')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
plan = load_plan(args.case_plan)
conditions = {c['condition_id']:c for c in plan['conditions']}
if args.condition_id not in conditions: parser.error('Unknown planned condition')
if args.output_dir.exists(): parser.error('Each branch requires a new output directory')
condition = conditions[args.condition_id]
args.physics_hz = condition['physics_hz']
args.seed = condition['seed']
args.radial_clearance_mm = condition['case']['radial_clearance_mm']
args.kit_args = (args.kit_args+' --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true').strip()
app = AppLauncher(args).app
import torch
import isaaclab.sim as sim_utils
from forge_budget import run

try:
    with torch.inference_mode(): run(args, app)
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

