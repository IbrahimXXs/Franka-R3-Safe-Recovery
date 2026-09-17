"""Launch development and the frozen force-versus-productivity detector ablation."""
import argparse
from pathlib import Path
from isaaclab.app import AppLauncher

root=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir',type=Path,default=root/'outputs/Force-vs-Productivity-Dewedge-v1')
parser.add_argument('--config',type=Path,default=root/'experiments/forge_phase2.json')
parser.add_argument('--plan',type=Path,default=root/'experiments/force_productivity_dewedge.json')
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if args.output_dir.exists():parser.error('Use a new output directory; existing data is never overwritten')
if not args.config.is_file() or not args.plan.is_file():parser.error('Missing protocol or frozen held-out plan')
args.physics_hz=None;args.stock_buffers=False
args.kit_args=(args.kit_args+' --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true').strip()
app=AppLauncher(args).app
import sys
sys.path.insert(0,str(root))
import torch
import isaaclab.sim as sim_utils
from force_productivity import run
try:
    with torch.inference_mode():run(args,app)
finally:
    sim=sim_utils.SimulationContext.instance()
    if sim is not None:
        sim.clear_all_callbacks();sim.clear_instance();sim.stop()
    app.close(wait_for_replicator=False)
