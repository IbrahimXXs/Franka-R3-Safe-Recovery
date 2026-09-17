"""Presentation-only RGB recording around the unchanged frozen FORGE controller."""
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output-dir', type=Path, default=ROOT/'outputs/Presentation-Videos-v1')
parser.add_argument('--preview', action='store_true')
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
args.physics_hz = None
args.stock_buffers = False
args.kit_args = (args.kit_args + ' --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true').strip()
app = AppLauncher(args).app

import hashlib
import json
import shutil
from types import FunctionType
import numpy as np
from PIL import Image
import torch
from pxr import UsdGeom, Gf
import omni.replicator.core as rep
import isaaclab.sim as sim_utils
from forge_backend import Bench, resolve_physics_rate
from forge_experiment import save
from productivity_dewedge import execute_dewedge
from research.forge_protocol import ForgeProtocol
from research.productivity_detector_v2 import bind_detector
from research.productivity_unloading import UnloadingDesign
from research.productivity_dewedge import DewedgeDesign
from research.force_productivity_design import verify

SOURCE = ROOT/'outputs/Force-vs-Productivity-Dewedge-v1'
CASES = ('ft030_axis_tilt_severe_v2', 'ft027_axis_tilt_moderate_v3')
CAMERA = dict(eye=[.68, -.16, .13], target=[.6, 0., .09], focal_length_mm=22.,
              horizontal_aperture_mm=20.955, clipping_range_m=[.001, 10.], resolution=[1920,1080])


def camera(bench):
    path='/World/PresentationCamera'
    cam=UsdGeom.Camera.Define(bench.sim.stage,path)
    cam.CreateFocalLengthAttr(CAMERA['focal_length_mm'])
    cam.CreateHorizontalApertureAttr(CAMERA['horizontal_aperture_mm'])
    cam.CreateVerticalApertureAttr(CAMERA['horizontal_aperture_mm']*1080/1920)
    cam.CreateClippingRangeAttr(Gf.Vec2f(*CAMERA['clipping_range_m']))
    cam.CreateFStopAttr(0.)
    matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*CAMERA['eye']),Gf.Vec3d(*CAMERA['target']),Gf.Vec3d(0,0,1)).GetInverse()
    UsdGeom.Xformable(cam).AddTransformOp().Set(matrix)
    product=rep.create.render_product(path,tuple(CAMERA['resolution']))
    rgb=rep.AnnotatorRegistry.get_annotator('rgb',device='cpu')
    rgb.attach([product])
    for _ in range(8): bench.sim.render()
    return product,rgb


class Recorder:
    def __init__(self,bench,rgb,directory):
        self.bench=bench;self.rgb=rgb;self.directory=directory
        self.frames=[];self.last_step=None
        (directory/'frames').mkdir(parents=True,exist_ok=False)

    def capture(self,row,force=False):
        if row['step']==self.last_step:return
        if not force and row['step']%4:return
        b=self.bench;t=b.sim.current_time
        q=b.env.joint_pos.clone();peg=b.env.held_pos.clone()
        # Two render updates flush the current physics pose through the RGB pipeline.
        # SimulationContext.render disables simulation advancement during app updates.
        b.sim.render();b.sim.render()
        assert b.sim.current_time==t and torch.equal(q,b.env.joint_pos) and torch.equal(peg,b.env.held_pos)
        data=np.asarray(self.rgb.get_data())
        assert data.shape[:2]==(1080,1920),data.shape
        image=Image.fromarray(np.ascontiguousarray(data[:,:,:3]),'RGB')
        name=f'frames/{len(self.frames):05d}.png';image.save(self.directory/name,compress_level=1)
        self.frames.append(dict(file=name,step=row['step'],time_s=row['time_s'],phase=row['phase']))
        self.last_step=row['step']
        if row['intervention_started']:
            image.save(self.directory/'trigger_raw.png')
            print('VIDEO TRIGGER',row['trigger_branch'],row['time_s'],row['wrist_force_n'],row['eta_raw'],flush=True)


def main():
    directory=args.output_dir.resolve()
    directory.mkdir(parents=True,exist_ok=True)
    target=directory/(f'preview-{len(list(directory.glob("preview*"))):03d}' if args.preview else 'replays')
    target.mkdir(exist_ok=False)
    m=json.loads((SOURCE/'experiment.json').read_text())
    plan=json.loads((SOURCE/'condition_plan.json').read_text())
    verify(ROOT,plan)
    for name,digest in m['source_sha256'].items():
        assert hashlib.sha256((ROOT/name).read_bytes()).hexdigest()==digest,name
    p=ForgeProtocol(**m['protocol']);args.seed=p.seed
    resolve_physics_rate(args);assert args.physics_hz==m['physics_hz']==120
    config=json.loads((SOURCE/'detector_v2_config.json').read_text())
    assert hashlib.sha256((SOURCE/'detector_v2_config.json').read_bytes()).hexdigest()==m['detector_v2_config_sha256']
    frozen=json.loads((SOURCE/'calibration.json').read_text())
    old={str(f.resolve()):[f.stat().st_size,f.stat().st_mtime_ns] for f in (ROOT/'outputs').rglob('*') if f.is_file() and directory not in f.resolve().parents}
    save(target/'preexisting_outputs.json',old)
    bench=None
    try:
        bench=Bench(args,app);bench.protocol=p
        save(target/'config.json',bench.cfg.to_dict());save(target/'scene.json',bench.scene_info)
        assert json.loads((target/'config.json').read_text())==json.loads((SOURCE/'config.json').read_text()),'Configuration differs'
        assert json.loads((target/'scene.json').read_text())==json.loads((SOURCE/'scene.json').read_text()),'Scene differs'
        product,rgb=camera(bench)
        save(target/'camera.json',CAMERA)
        bench.prepare(p.seed)  # Same single solver-priming reset as the frozen experiment.
        if args.preview:
            for _ in range(3):bench.sim.render()
            Image.fromarray(np.asarray(rgb.get_data())[:,:,:3]).save(target/'camera.png')
            return
        summary=[]
        for cid in CASES:
            verify(ROOT,plan)
            case=next(c for c in plan['cases'] if c['case_id']==cid)
            rd=target/cid;rd.mkdir()
            save(rd/'condition.json',case)
            recorder=Recorder(bench,rgb,rd)
            bound=bind_detector(execute_dewedge,config,p.success_depth_mm)
            original_stream=bound.__globals__['Stream']
            class RecordingStream(original_stream):
                def add(self,row):
                    super().add(row)
                    recorder.capture(row)
                def close(self):
                    if self.rows:recorder.capture(self.rows[-1],force=True)
                    super().close()
            run=FunctionType(bound.__code__,dict(bound.__globals__,Stream=RecordingStream),bound.__name__,bound.__defaults__,bound.__closure__)
            assert run.__code__ is execute_dewedge.__code__
            print('RECORDING',cid,flush=True)
            result=run(bench,case['case'],frozen,UnloadingDesign(**m['unloading_design']),DewedgeDesign(**m['dewedge_design']),rd/'trajectory.csv')
            events=result.pop('events');save(rd/'events.json',events);save(rd/'metrics.json',result)
            save(rd/'frames.json',recorder.frames)
            print('RECORDED',cid,result['outcome'],result['intervention_count'],flush=True)
            summary.append(dict(case_id=cid,**result))
        save(target/'summary.json',summary)
        verify(ROOT,plan)
    finally:
        changed=[f for f,st in old.items() if not Path(f).is_file() or [Path(f).stat().st_size,Path(f).stat().st_mtime_ns]!=st]
        save(target/'preservation.json',dict(files_checked=len(old),changed=changed))
        assert not changed
        if bench is not None:bench.close()


try:
    with torch.inference_mode():main()
except BaseException:
    import traceback
    traceback.print_exc()
    raise
finally:
    sim=sim_utils.SimulationContext.instance()
    if sim is not None:
        sim.clear_all_callbacks();sim.clear_instance();sim.stop()
    app.close(wait_for_replicator=False)
