"""Controlled simulation experiments; contact profiles and policy-specific recovery costs."""

from dataclasses import asdict, dataclass
import csv
import json
import math
from pathlib import Path

import numpy as np
import torch
import isaaclab.sim as sim_utils
from isaaclab.utils.math import quat_apply, quat_mul, compute_pose_error

from contact import ContactWrench
from scene import build_scene, HOLE_CENTER, HOLE_DEPTH, PEG_RADIUS, PEG_LENGTH, PEG_MASS, PEG_TIP_IN_HAND, PEG_CENTER_IN_HAND, START_GAP, ROBOT_USD


@dataclass(frozen=True)
class Scenario:
    name: str
    offset_mm: float = 0.
    insertion_tilt_deg: float = 0.
    loaded_tilt_deg: float = 0.
    depth_mm: float = 20.


SCENARIOS = {
    x.name: x for x in (
        Scenario("aligned"),
        Scenario("offset", offset_mm=.4),
        Scenario("tilted", insertion_tilt_deg=3., loaded_tilt_deg=3.),
        Scenario("loaded_shallow", loaded_tilt_deg=5., depth_mm=10.),
        Scenario("loaded_deep", loaded_tilt_deg=5., depth_mm=20.),
    )
}


class NumericalTrialError(RuntimeError):
    """A finite observed trial crossed the numerical guard; other trials may run."""

    def __init__(self, row):
        self.row = row
        super().__init__(f"Numerical guard at {row['time_s']:.4f}s: "
            f"{row['force_norm_n']:.3f} N, separation {row['min_separation_mm']:.4f} mm")


def smooth(t):
    t = max(0., min(1., t))
    return t*t*(3.-2.*t)


class Workbench:
    def __init__(self, args, app):
        self.args, self.app = args, app
        self.dt = 1 / args.physics_hz
        self.sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
            dt=self.dt, render_interval=args.physics_hz // 30, device=args.device,
            physx=sim_utils.PhysxCfg(min_position_iteration_count=64, min_velocity_iteration_count=4,
                friction_offset_threshold=args.contact_offset_mm/1000,
                friction_correlation_distance=.0005, enable_stabilization=False),
            render=sim_utils.RenderCfg(antialiasing_mode="DLSS", dlss_mode=0),
        ))
        # Isaac Lab disables CPU contact processing by default. Its standard
        # ContactSensor opts back in; our direct tensor readout must do so too.
        self.sim.carb_settings.set_bool("/physics/disableContactProcessing", False)
        self.robot = build_scene(self.sim, friction=args.friction, clearance=args.clearance_mm/1000,
            contact_offset=args.contact_offset_mm/1000, segments=args.socket_segments,
            contact_stiffness=args.contact_stiffness, contact_damping=args.contact_damping, torque_control=True)
        self.sim.reset()
        self.sim._disable_app_control_on_stop_handle = True
        self.arm_ids, _ = self.robot.find_joints("fr3_joint[1-7]")
        self.hand_id = self.robot.find_bodies("fr3_hand")[0][0]
        actual_mass = self.robot.root_physx_view.get_masses()[0, self.hand_id].item()
        if abs(actual_mass - self.robot.payload_model['hand_mass_kg']) > 1e-5:
            raise RuntimeError('Compound payload mass was not applied')
        self.target = torch.zeros((1, 7), device=self.sim.device)
        self.joint_target = self.robot.data.default_joint_pos.clone()
        self.tip_offset = torch.tensor([PEG_TIP_IN_HAND], device=self.sim.device)
        self.hand_to_tip = torch.tensor([PEG_TIP_IN_HAND], device=self.sim.device)
        self.axis_local = torch.tensor([[0., 0., 1.]], device=self.sim.device)
        self.steps = 0

    def set_goal(self, tip, tilt_deg):
        half = math.radians(tilt_deg) / 2
        tilt = torch.tensor([[math.cos(half), 0., math.sin(half), 0.]], device=self.sim.device)
        down = torch.tensor([[0., 1., 0., 0.]], device=self.sim.device)
        q = quat_mul(tilt, down)
        self.target[:, 3:] = q
        self.target[:, :3] = torch.tensor([tip], device=self.sim.device) - quat_apply(q, self.hand_to_tip)

    def tick(self):
        if not self.app.is_running() or self.sim.is_stopped():
            raise InterruptedError("Study interrupted; only completed trials can be compared")
        while self.app.is_running() and not self.sim.is_playing():
            if self.sim.is_stopped():
                raise InterruptedError("Study stopped")
            self.sim.render()
        if not self.app.is_running():
            raise InterruptedError("Study window closed")
        robot = self.robot
        hand = robot.data.body_link_pose_w[:, self.hand_id]
        p_error, q_error = compute_pose_error(hand[:, :3], hand[:, 3:], self.target[:, :3], self.target[:, 3:])
        velocity = robot.data.body_link_vel_w[:, self.hand_id]
        wrench = torch.cat((self.args.translation_stiffness*p_error - 80.*velocity[:, :3],
                            self.args.rotation_stiffness*q_error - 2.*velocity[:, 3:]), -1)
        jacobian = robot.root_physx_view.get_jacobians()[:, self.hand_id-1, :, self.arm_ids]
        current_q = robot.data.joint_pos[:, self.arm_ids]
        qdot = robot.data.joint_vel[:, self.arm_ids]
        jt = jacobian.transpose(1, 2)
        if self.steps % 8 == 0 or not hasattr(self, 'null_projector'):
            self.null_projector = torch.eye(7, device=self.sim.device).unsqueeze(0) - jt @ torch.linalg.pinv(jt)
        null_projector = self.null_projector
        posture = 5.*(robot.data.default_joint_pos[:, self.arm_ids]-current_q) - 2.*qdot
        nominal = (jt @ wrench.unsqueeze(-1) + null_projector @ posture.unsqueeze(-1)).squeeze(-1)
        # Implicit PD prediction prevents explicit damping from exciting the
        # low-inertia wrist at finite physics dt. M includes the payload.
        stiffness = torch.diag(torch.tensor([self.args.translation_stiffness]*3 +
            [self.args.rotation_stiffness]*3, device=self.sim.device))
        damping = torch.diag(torch.tensor([80.]*3+[2.]*3, device=self.sim.device))
        kj = jt @ stiffness @ jacobian + 5.*null_projector
        dj = jt @ damping @ jacobian + 2.*null_projector
        mass = robot.root_physx_view.get_generalized_mass_matrices()[:, :7, :7]
        system = mass + self.dt*dj + self.dt**2*kj
        predicted = nominal - self.dt*(kj @ qdot.unsqueeze(-1)).squeeze(-1)
        acceleration = torch.linalg.solve(system, predicted.unsqueeze(-1))
        effort = (mass @ acceleration).squeeze(-1)
        caps = torch.tensor([[87.,87.,87.,87.,12.,12.,12.]], device=self.sim.device)
        self.effort_saturated = bool((effort.abs() >= caps).any())
        robot.set_joint_effort_target(effort.clamp(-caps, caps), joint_ids=self.arm_ids)
        robot.set_joint_position_target(self.joint_target[:, 7:], joint_ids=[7, 8])
        robot.write_data_to_sim()
        self.sim.step(render=False)
        self.steps += 1
        if self.steps % (self.args.physics_hz // 30) == 0:
            self.sim.render()
        robot.update(self.dt)
        self.sim.carb_settings.set_bool("/rtx/ecoMode/enabled", True)

    def prepare(self, scenario):
        self.sim.reset()
        self.sim._disable_app_control_on_stop_handle = True
        self.robot.write_joint_state_to_sim(self.robot.data.default_joint_pos, self.robot.data.default_joint_vel)
        self.robot.reset()
        if hasattr(self, "null_projector"):
            del self.null_projector
        self.robot.update(self.dt)
        self.set_goal((HOLE_CENTER[0]+scenario.offset_mm/1000, 0., HOLE_CENTER[2]+START_GAP), scenario.insertion_tilt_deg)
        for i in range(self.args.physics_hz * 12):
            self.tick()
            error = torch.linalg.vector_norm(self.robot.data.body_link_pose_w[:, self.hand_id, :3]-self.target[:, :3]).item()
            if i >= self.args.physics_hz * 3 and error < .0001:
                break
        if error > .002:
            raise RuntimeError(f"Prepared pose did not converge: {error*1000:.3f} mm")
        self.sensor = ContactWrench(self.sim, '/World/Robot/fr3_hand',
            ['/World/Fixture/Socket', '/World/Fixture/Pedestal/geometry/mesh'])

    def observe(self):
        pose = self.robot.data.body_link_pose_w[:, self.hand_id]
        tip = pose[:, :3] + quat_apply(pose[:, 3:], self.tip_offset)
        axis = quat_apply(pose[:, 3:], self.axis_local)
        offset = torch.tensor([PEG_CENTER_IN_HAND], device=self.sim.device)
        com = (pose[:, :3]+quat_apply(pose[:, 3:], offset))[0]
        hand_com = self.robot.data.body_com_pos_w[0, self.hand_id]
        velocity = self.robot.data.body_com_vel_w[0, self.hand_id].clone()
        velocity[:3] += torch.cross(velocity[3:], com-hand_com, dim=-1)
        contact = self.sensor.read(com)
        values = torch.cat((pose.flatten(), velocity, contact['force'], contact['torque']))
        if not torch.isfinite(values).all():
            raise RuntimeError("Non-finite experiment state")
        force, torque = contact['force'].tolist(), contact['torque'].tolist()
        result = {
            'depth_mm': (HOLE_CENTER[2]-tip[0, 2].item())*1000,
            'tip_x_mm': (tip[0, 0].item()-HOLE_CENTER[0])*1000,
            'tip_y_mm': tip[0, 1].item()*1000,
            'tilt_deg': math.degrees(math.acos(max(-1., min(1., -axis[0, 2].item())))),
            'effort_saturated': self.effort_saturated,
            'force_norm_n': contact['force'].norm().item(), 'torque_norm_nm': contact['torque'].norm().item(),
            'normal_load_n': contact['normal_load_n'], 'contact_count': contact['contact_count'],
            'min_separation_mm': contact['min_separation_m']*1000,
            'contact_power_w': (contact['force'] @ velocity[:3] + contact['torque'] @ velocity[3:]).item(),
            'lowest_peg_z_m': com[2].item()-PEG_LENGTH/2*abs(axis[0, 2].item())
                - PEG_RADIUS*math.sqrt(max(0., 1.-axis[0, 2].item()**2)),
        }
        result.update(dict(zip(('qw', 'qx', 'qy', 'qz'), pose[0, 3:].tolist())))
        result.update({f'joint{i+1}_rad': value for i, value in enumerate(self.robot.data.joint_pos[0, self.arm_ids].tolist())})
        for key, vector in [('f', force), ('tau', torque), ('normal_f', contact['normal_force'].tolist()),
                            ('friction_f', contact['friction_force'].tolist()), ('v', velocity[:3].tolist()),
                            ('omega', velocity[3:].tolist())]:
            result.update({f'{key}{axis}': value for axis, value in zip('xyz', vector)})
        return result


def run_trial(bench, scenario, recovery, directory):
    bench.prepare(scenario)
    dt = bench.dt
    depth = scenario.depth_mm/1000
    initial = np.array([HOLE_CENTER[0]+scenario.offset_mm/1000, 0., HOLE_CENTER[2]+START_GAP])
    bottom = initial.copy(); bottom[2] = HOLE_CENTER[2]-depth
    stages = [('baseline', .5), ('insert', 6.), ('hold', .5), ('load', 2.), ('loaded_hold', .5)]
    if recovery == 'realign':
        stages.append(('realign', 2.))
    stages.extend([('retreat', 6.), ('clear_hold', .5)])
    rows = []
    start_time = bench.sim.current_time
    t = 0.
    csv_path = directory/f'{scenario.name}__{recovery}.csv'
    with csv_path.open('w', newline='') as stream:
        writer = None
        for phase, duration in stages:
            for k in range(round(duration/dt)):
                blend = smooth((k+1)*dt/duration)
                tilt = scenario.loaded_tilt_deg
                tip = bottom.copy()
                if phase == 'baseline':
                    tip, tilt = initial, scenario.insertion_tilt_deg
                elif phase == 'insert':
                    tip = initial+(bottom-initial)*blend
                    tilt = scenario.insertion_tilt_deg
                elif phase == 'hold':
                    tilt = scenario.insertion_tilt_deg
                elif phase == 'load':
                    tilt = scenario.insertion_tilt_deg+(scenario.loaded_tilt_deg-scenario.insertion_tilt_deg)*blend
                elif phase == 'realign':
                    tilt *= 1.-blend
                    tip[0] = bottom[0]+(HOLE_CENTER[0]-bottom[0])*blend
                elif phase in ('retreat', 'clear_hold'):
                    if recovery == 'realign':
                        tilt, tip[0] = 0., HOLE_CENTER[0]
                    tip[2] = bottom[2]+(initial[2]-bottom[2])*(blend if phase == 'retreat' else 1.)
                bench.set_goal(tuple(tip), tilt)
                bench.tick()
                t += dt
                row = {'time_s': t, 'simulation_time_s': bench.sim.current_time-start_time, 'phase': phase,
                    'command_depth_mm': (HOLE_CENTER[2]-tip[2])*1000, 'command_tilt_deg': tilt,
                    **bench.observe()}
                if abs(row['time_s']-row['simulation_time_s']) > .001:
                    raise RuntimeError('Physics and logged time diverged')
                if writer is None:
                    writer = csv.DictWriter(stream, fieldnames=list(row)); writer.writeheader()
                writer.writerow(row)
                rows.append(row)
                # Preserve the offending sample before stopping this trial.
                if row['force_norm_n'] > 500 or row['min_separation_mm'] < -1.:
                    raise NumericalTrialError(row)
    from study_report import summarize
    result = summarize(rows, scenario.name, recovery, bench.args.force_budget, bench.args.torque_budget, dt, bench.args.clearance_mm, HOLE_CENTER[2])
    result.update(asdict(scenario))
    return result


def run_study(args, app):
    directory = args.output_dir or args.log.parent
    directory.mkdir(parents=True, exist_ok=True)
    if (directory/'study.json').exists():
        raise FileExistsError('Study output already exists; use a new --output-dir')
    bench = Workbench(args, app)
    selected = list(SCENARIOS.values()) if args.scenario == ['all'] else [SCENARIOS[name] for name in args.scenario]
    policies = ['straight', 'realign'] if args.recovery == 'both' else [args.recovery]
    metadata = {'study': 'FR3-Recovery-Study-v1', 'status': 'running', 'parameters': {
        key: getattr(args, key) for key in ('physics_hz', 'friction', 'clearance_mm', 'contact_offset_mm',
            'socket_segments', 'force_budget', 'torque_budget', 'contact_stiffness', 'contact_damping',
            'translation_stiffness', 'rotation_stiffness')},
        'device': str(bench.sim.device), 'contact_processing_enabled': True, 'robot_usd': ROBOT_USD, 'payload_model': bench.robot.payload_model, 'peg_mass_kg': PEG_MASS, 'peg_diameter_mm': PEG_RADIUS*2000, 'peg_length_mm': PEG_LENGTH*1000,
        'hole_depth_mm': HOLE_DEPTH*1000, 'scenarios': [asdict(x) for x in selected], 'recovery_policies': policies,
        'force_signal': 'fixture_on_peg_normal_plus_friction', 'force_frame': 'world',
        'torque_origin': 'peg_center_of_mass', 'grasp': 'compound_rigid_body_no_slip_analytical_payload_inertia',
        'contact_model': 'compliant_Coulomb_equal_static_dynamic_friction', 'gravity': 'disabled_on_arm_and_payload_ideal_compensation',
        'controller': 'Cartesian_impedance_with_implicit_PD_prediction', 'translation_damping_ns_m': 80., 'rotation_damping_nms_rad': 2.,
        'friction_correlation_distance_m': .0005, 'solver': 'TGS_64_position_4_velocity',
        'force_budget_is': 'evaluation_threshold_not_controller_limit',
        'dlss_mode': bench.sim.carb_settings.get('/rtx/post/dlss/execMode'), 'eco_mode': True,
        'note': 'Synthetic parameter study, not calibrated materials or hardware validation.'}
    manifest = directory/'study.json'
    manifest.write_text(json.dumps(metadata, indent=2)+'\n')
    results, invalid = [], []
    try:
        for scenario in selected:
            for recovery in policies:
                print(f'Study trial: {scenario.name} / {recovery}', flush=True)
                try:
                    result = run_trial(bench, scenario, recovery, directory)
                except NumericalTrialError as error:
                    from study_report import invalid_trial
                    failure = invalid_trial(scenario.name, recovery, error.row, str(error))
                    failure.update(asdict(scenario))
                    invalid.append(failure)
                    print(f"  numerically_invalid; {error}. Partial CSV saved; continuing.", flush=True)
                else:
                    results.append(result)
                    print(f"  {result['status']}; retreat peak {result['retreat_peak_resistance_n']:.3f} N; "
                          f"recovery work {result['recovery_resistive_work_j']:.6f} J", flush=True)
                metadata['completed_trials'] = results
                metadata['invalid_trials'] = invalid
                manifest.write_text(json.dumps(metadata, indent=2)+'\n')
        from study_report import write_report
        write_report(directory, results, invalid)
        metadata['status'] = 'complete_with_invalid_trials' if invalid else 'complete'
        manifest.write_text(json.dumps(metadata, indent=2)+'\n')
        print(f"Study finished: {len(results)} completed, {len(invalid)} numerically invalid. "
              f"Results: {directory.resolve()}", flush=True)
    except BaseException as error:
        metadata['status'] = 'incomplete'
        metadata['error'] = str(error)
        manifest.write_text(json.dumps(metadata, indent=2)+'\n')
        raise
    if not args.headless and not args.exit_after and app.is_running():
        bench.sim.pause()
        print('Diagnostics finished. Simulation paused; close the window to exit.', flush=True)
        while app.is_running():
            app.update()
            bench.sim.carb_settings.set_bool('/rtx/ecoMode/enabled', True)
