"""GPU force-readout validation. Run in the project's Conda environment."""

import argparse
from pathlib import Path
import sys
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
app = AppLauncher(args).app

import torch
import isaaclab.sim as sim_utils
from isaaclab.assets import RigidObject, RigidObjectCfg
from pxr import PhysxSchema
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "simulation"))
from contact import ContactWrench


def main():
    dt = 1 / 240
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=dt, device=args.device,
        physx=sim_utils.PhysxCfg(min_position_iteration_count=64, min_velocity_iteration_count=4,
            friction_offset_threshold=.00005, friction_correlation_distance=.0005)))
    sim.carb_settings.set_bool("/physics/disableContactProcessing", False)
    material = sim_utils.RigidBodyMaterialCfg(static_friction=.3, dynamic_friction=.3, restitution=0.,
        compliant_contact_stiffness=1e6, compliant_contact_damping=100.)
    floor = sim_utils.CuboidCfg(size=(2., 2., .1), collision_props=sim_utils.CollisionPropertiesCfg(), physics_material=material)
    floor.func("/World/Floor", floor, translation=(0., 0., -.05))
    cube = RigidObject(RigidObjectCfg(prim_path="/World/Probe",
        spawn=sim_utils.CuboidCfg(size=(.1, .1, .1), rigid_props=sim_utils.RigidBodyPropertiesCfg(),
            mass_props=sim_utils.MassPropertiesCfg(mass=1.), physics_material=material,
            collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.00005, rest_offset=0.)),
        init_state=RigidObjectCfg.InitialStateCfg(pos=(0., 0., .3))))
    PhysxSchema.PhysxContactReportAPI.Apply(sim.stage.GetPrimAtPath('/World/Probe')).CreateThresholdAttr(0.)
    sim.reset()
    sim._disable_app_control_on_stop_handle = True
    sensor = ContactWrench(sim, '/World/Probe', ['/World/Floor/geometry/mesh'])
    force = torch.tensor([[[2., 0., 0.]]], device=sim.device)
    torque = torch.tensor([[[0., 0., .02]]], device=sim.device)
    measurements = []
    friction_samples = []
    balance_errors = []
    for i in range(960):
        if i >= 480:
            cube.set_external_force_and_torque(force, torque, is_global=True)
        velocity_before = cube.data.root_com_lin_vel_w[0].clone()
        cube.write_data_to_sim()
        sim.step(render=False)
        cube.update(dt)
        if i % 8 == 0:
            sim.render()
        value = sensor.read(cube.data.root_pos_w[0])
        if i == 0:
            assert value['force'].norm() < 1e-5, value
        if i >= 840:
            measurements.append(torch.cat((value['force'], value['torque'])))
            friction_samples.append(value['friction_force'].clone())
            acceleration = (cube.data.root_com_lin_vel_w[0]-velocity_before)/dt
            expected_contact = acceleration-force[0, 0]-torch.tensor([0., 0., -9.81], device=sim.device)
            balance_errors.append((value['force']-expected_contact).abs().max())
    actual = torch.stack(measurements).mean(0)
    expected = torch.tensor([-2., 0., 9.81, 0., 0., -.02], device=sim.device)
    print('CONTACT_PROBE_MEASURED', actual.tolist(), flush=True)
    torch.testing.assert_close(actual[:3], expected[:3], atol=.01, rtol=.01)
    torch.testing.assert_close(actual[3:], expected[3:], atol=.002, rtol=.01)
    torch.testing.assert_close(torch.stack(friction_samples).mean(0), expected[:3]*torch.tensor([1., 1., 0.], device=sim.device), atol=.01, rtol=.01)
    residual = torch.stack(balance_errors).max().item()
    print('CONTACT_FORCE_BALANCE_MAX_ERROR_N', residual, flush=True)
    assert residual < .1, residual
    print('CONTACT_PROBE_PASSED: free space, weight, tangential load, and torque.', flush=True)


try:
    main()
except Exception:
    import traceback
    traceback.print_exc()
    import omni.kit.app
    omni.kit.app.get_app().post_quit(1)
    raise
finally:
    sim = sim_utils.SimulationContext.instance()
    if sim is not None:
        sim.clear_all_callbacks()
        sim.clear_instance()
        sim.stop()
    app.close(wait_for_replicator=False)
