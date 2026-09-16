"""Custom Franka Research 3 peg-in-hole workbench. No robot hardware connection."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import shutil
import subprocess

from isaaclab.app import AppLauncher

PROJECT_ROOT = Path(__file__).resolve().parents[1]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, choices=[1], default=1, help="This initial workbench supports one FR3.")
parser.add_argument("--demo", action="store_true", help="Execute a slow, scripted 20 mm insertion.")
parser.add_argument("--phase2-resume", action="store_true", help="Resume the same Phase 2 protocol and output directory.")
parser.add_argument("--phase2", type=Path, metavar="CONFIG", help="Run randomized characterization from a Phase 2 JSON protocol.")
parser.add_argument("--study", action="store_true", help="Run insertion/retreat experiments and produce force profiles.")
parser.add_argument("--output-dir", type=Path, help="Study output directory; defaults to outputs/<timestamp>.")
parser.add_argument("--scenario", choices=["all", "aligned", "offset", "tilted", "loaded_shallow", "loaded_deep"], nargs="+", default=["all"])
parser.add_argument("--recovery", choices=["straight", "realign", "both"], default="both")
parser.add_argument("--physics-hz", type=int, choices=[120, 240, 480, 960], default=480)
parser.add_argument("--friction", type=float, default=0.3, help="Synthetic static and dynamic coefficient for study contacts.")
parser.add_argument("--clearance-mm", type=float, default=0.5, help="Radial peg/bore clearance for the study.")
parser.add_argument("--contact-offset-mm", type=float, default=0.05)
parser.add_argument("--socket-segments", type=int, choices=[96, 192, 384], default=192)
parser.add_argument("--contact-stiffness", type=float, default=100000., help="Synthetic compliant contact stiffness, N/m; study only.")
parser.add_argument("--contact-damping", type=float, default=100., help="Synthetic contact damping, Ns/m; study only.")
parser.add_argument("--translation-stiffness", type=float, default=1500., help="Cartesian controller stiffness, N/m.")
parser.add_argument("--rotation-stiffness", type=float, default=20., help="Cartesian controller rotational stiffness, Nm/rad.")
parser.add_argument("--force-budget", type=float, default=20., help="Recovery force evaluation threshold, N; not a controller limit.")
parser.add_argument("--torque-budget", type=float, default=1., help="Recovery torque evaluation threshold, N m.")
parser.add_argument("--exit-after", action="store_true", help="Close the GUI after the requested steps.")
parser.add_argument("--steps", type=int, default=300, help="Number of 30 Hz control steps to log (default: 300).")
parser.add_argument(
    "--keep-open", action="store_true",
    help="Keep the GUI open after diagnostics (already the GUI default).",
)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument(
    "--log", type=Path,
    help="Diagnostic JSONL path (default: outputs/<UTC timestamp>/diagnostics.jsonl).",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.phase2_resume and not args.phase2:
    parser.error("--phase2-resume requires --phase2")
if "all" in args.scenario and len(args.scenario) > 1:
    parser.error("Use --scenario all or a list of named scenarios")
if len(args.scenario) != len(set(args.scenario)):
    parser.error("Scenario names must be unique")
if args.output_dir and not (args.study or args.phase2):
    parser.error("--output-dir is used with --study or --phase2")
if sum((bool(args.study), bool(args.demo), bool(args.phase2))) > 1:
    parser.error("Choose --study, --demo, or --phase2")
import math
if not all(math.isfinite(v) for v in (args.friction, args.clearance_mm, args.contact_offset_mm, args.force_budget, args.torque_budget, args.contact_stiffness, args.contact_damping, args.translation_stiffness, args.rotation_stiffness)):
    parser.error("Study parameters must be finite")
if min(args.contact_stiffness, args.translation_stiffness, args.rotation_stiffness) <= 0 or args.contact_damping < 0:
    parser.error("Contact/controller stiffness must be positive and contact damping nonnegative")
if not 0 <= args.friction <= 2 or not 0.02 <= args.clearance_mm <= 2:
    parser.error("Use friction in [0, 2] and radial clearance in [0.02, 2] mm")
if not 0 < args.contact_offset_mm < args.clearance_mm / 2 or min(args.force_budget, args.torque_budget) <= 0:
    parser.error("Contact offset must be positive and below half the radial clearance; budgets must be positive")
if args.num_envs < 1 or args.steps < 1:
    parser.error("num_envs and steps must be positive")
if args.keep_open and args.exit_after:
    parser.error("--keep-open and --exit-after cannot be combined")
if args.keep_open and args.headless:
    parser.error("--keep-open requires the GUI; remove --headless")
if args.log is None:
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ")
    args.log = PROJECT_ROOT / "outputs" / run_id / "diagnostics.jsonl"

# An NVIDIA update can leave the running kernel module behind its libraries.
# Detect this before Kit starts: failed CUDA initialization can crash its cleanup.
nvidia_smi = shutil.which("nvidia-smi")
if nvidia_smi is not None:
    try:
        driver_check = subprocess.run(
            [nvidia_smi, "--query-gpu=driver_version", "--format=csv,noheader"],
            capture_output=True, text=True, timeout=10, check=False,
        )
    except (OSError, subprocess.TimeoutExpired):
        pass  # Let Isaac Sim report other GPU initialization problems.
    else:
        driver_output = driver_check.stdout + driver_check.stderr
        if driver_check.returncode and "driver/library version mismatch" in driver_output.lower():
            parser.exit(
                1,
                "NVIDIA driver/library version mismatch. Isaac Sim cannot initialize the GPU.\n"
                "Save your work and reboot Ubuntu, then check that `nvidia-smi` succeeds.\n"
                "If it still fails after reboot, the system NVIDIA driver needs investigation.\n"
                "The Conda environment cannot repair this system-driver mismatch.\n",
            )

# Apply project defaults before Kit creates its viewport.
args.kit_args = (
    f"{args.kit_args} --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0"
    " --/rtx/ecoMode/enabled=true"
).strip()
launcher = AppLauncher(args)
simulation_app = launcher.app

# Isaac/Omniverse imports must follow AppLauncher initialization.
import torch
import isaaclab.sim as sim_utils
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.utils.math import subtract_frame_transforms, quat_apply
from scene import (
    build_scene, ROBOT_USD, PEG_RADIUS, PEG_LENGTH, PEG_TIP_IN_HAND,
    HOLE_CENTER, HOLE_RADIUS, HOLE_DEPTH, START_GAP, TARGET_DEPTH, DT, DECIMATION,
)


def main():
    if args.phase2:
        from phase2 import run_phase2
        return run_phase2(args, simulation_app)
    if args.study:
        from study import run_study
        return run_study(args, simulation_app)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(
        dt=DT, render_interval=DECIMATION, device=args.device,
        render=sim_utils.RenderCfg(antialiasing_mode="DLSS", dlss_mode=0),
    ))
    robot = build_scene(sim)
    sim.reset()
    # Lab 2.3's standalone STOP callback waits indefinitely for Play, including
    # during window closure. This launcher owns pause/stop/exit handling instead.
    sim._disable_app_control_on_stop_handle = True
    robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    robot.reset()
    robot.update(DT)
    arm_ids, arm_names = robot.find_joints("fr3_joint[1-7]")
    hand_id = robot.find_bodies("fr3_hand")[0][0]
    assert len(arm_ids) == 7 and robot.is_fixed_base, "Unexpected FR3 articulation"
    controller = DifferentialIKController(
        DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls"),
        num_envs=1, device=sim.device,
    )
    target = torch.tensor([[*HOLE_CENTER, 0., 1., 0., 0.]], device=sim.device)
    target[:, 2] += START_GAP + PEG_TIP_IN_HAND[2]
    tip_offset = torch.tensor([PEG_TIP_IN_HAND], device=sim.device)
    down = torch.tensor([[0., 0., -1.]], device=sim.device)
    local_axis = torch.tensor([[0., 0., 1.]], device=sim.device)
    joint_target = robot.data.default_joint_pos.clone()
    physics_steps = 0

    def command_hand(world_target):
        root = robot.data.root_state_w
        hand = robot.data.body_state_w[:, hand_id, :7]
        goal_p, goal_q = subtract_frame_transforms(root[:, :3], root[:, 3:7], world_target[:, :3], world_target[:, 3:7])
        hand_p, hand_q = subtract_frame_transforms(root[:, :3], root[:, 3:7], hand[:, :3], hand[:, 3:7])
        controller.set_command(torch.cat((goal_p, goal_q), dim=-1))
        jac = robot.root_physx_view.get_jacobians()[:, hand_id - 1, :, arm_ids]
        q = robot.data.joint_pos[:, arm_ids]
        desired = controller.compute(hand_p, hand_q, jac, q)
        # Limit each incremental update and keep commands within asset limits.
        desired = q + (desired - q).clamp(-0.02, 0.02)
        limits = robot.data.soft_joint_pos_limits[:, arm_ids]
        joint_target[:, arm_ids] = desired.clamp(limits[..., 0], limits[..., 1])
        robot.set_joint_position_target(joint_target)

    def tick():
        nonlocal physics_steps
        physics_steps += 1
        robot.write_data_to_sim()
        sim.step(render=False)
        if physics_steps % DECIMATION == 0:
            sim.render()
        robot.update(DT)
        # Isaac Sim's throttling extension disables Eco on every Play event.
        sim.carb_settings.set_bool("/rtx/ecoMode/enabled", True)

    # Settle before recording; allow up to 10 simulated seconds for convergence.
    for settling_step in range(1200):
        if not simulation_app.is_running():
            return
        command_hand(target)
        tick()
        if settling_step >= 359 and torch.linalg.vector_norm(
            robot.data.body_state_w[:, hand_id, :3] - target[:, :3]
        ).item() < 0.0001:
            break
    hand = robot.data.body_state_w[:, hand_id, :7]
    if torch.linalg.vector_norm(hand[:, :3] - target[:, :3]).item() > 0.001:
        raise RuntimeError("FR3 could not reach the prepared insertion pose")
    print("Rendering: DLSS Performance; Eco mode enabled.", flush=True)
    print("FR3 ready: peg held 25 mm above a 9 mm bore. " +
          ("Scripted insertion demo." if args.demo else "Holding the starting pose; use --demo for insertion."), flush=True)
    args.log.parent.mkdir(parents=True, exist_ok=True)
    with args.log.open("w", encoding="utf-8") as stream:
        stream.write(json.dumps({
            "record": "metadata", "task": "FR3-Custom-PegInsert-v1", "robot_usd": ROBOT_USD,
            "arm_joints": arm_names, "hand_body": "fr3_hand", "seed": args.seed,
            "physics_dt_s": DT, "decimation": DECIMATION,
            "settling_time_s": (settling_step + 1) * DT,
            "peg_diameter_m": 2 * PEG_RADIUS, "peg_length_m": PEG_LENGTH,
            "hole_diameter_m": 2 * HOLE_RADIUS, "hole_depth_m": HOLE_DEPTH,
            "hole_center_world_m": HOLE_CENTER, "target_insertion_depth_m": TARGET_DEPTH,
            "grasp": "compound_rigid_grasp_with_30g_payload", "gravity_compensation": "gravity_disabled_on_robot",
            "mode": "scripted_insertion" if args.demo else "hold", "wrench_sensor_implemented": False,
            "dlss_mode": sim.carb_settings.get("/rtx/post/dlss/execMode"),
            "eco_mode": sim.carb_settings.get("/rtx/ecoMode/enabled"),
            "note": "Geometric seating only; state is privileged. No force controller or recovery policy.",
        }) + "\n")
        experiment_start_time = sim.current_time
        for step in range(args.steps):
            if not simulation_app.is_running():
                break
            if sim.is_stopped():
                break
            if not sim.is_playing():
                while simulation_app.is_running() and not sim.is_playing() and not sim.is_stopped():
                    sim.render()
                if not simulation_app.is_running() or sim.is_stopped():
                    break
            t = (step + 1) * DT * DECIMATION
            if args.demo:
                # One second hold, then six seconds of smooth vertical motion.
                fraction = max(0., min(1., (t - 1.) / 6.))
                blend = fraction * fraction * (3. - 2. * fraction)
                target[:, 2] = HOLE_CENTER[2] + PEG_TIP_IN_HAND[2] + START_GAP - (START_GAP + TARGET_DEPTH) * blend
            for _ in range(DECIMATION):
                command_hand(target)
                tick()
            hand = robot.data.body_state_w[:, hand_id, :7]
            tip = hand[:, :3] + quat_apply(hand[:, 3:7], tip_offset)
            axis = quat_apply(hand[:, 3:7], local_axis)
            tilt = torch.acos((axis * down).sum(-1).clamp(-1., 1.))
            hole_xy = torch.tensor([HOLE_CENTER[:2]], device=sim.device)
            xy_error = torch.linalg.vector_norm(tip[:, :2] - hole_xy, dim=-1)
            depth = HOLE_CENTER[2] - tip[:, 2]
            # Check the shaft at the mouth as well as its tip.
            mouth_xy = tip[:, :2] + axis[:, :2] * (depth / axis[:, 2].clamp(max=-1e-6)).unsqueeze(-1)
            mouth_error = torch.linalg.vector_norm(mouth_xy - hole_xy, dim=-1)
            seated = ((depth >= TARGET_DEPTH - 0.0005) & (depth <= HOLE_DEPTH)
                      & (torch.maximum(xy_error, mouth_error) < HOLE_RADIUS - PEG_RADIUS)
                      & (tilt < 0.01))
            if not torch.isfinite(robot.data.joint_pos).all() or not torch.isfinite(tip).all():
                raise RuntimeError("Non-finite simulation state")
            stream.write(json.dumps({
                "record": "diagnostic", "step": step, "nominal_time_s": t,
                "simulation_time_s": sim.current_time - experiment_start_time,
                "peg_tip_pos_w": tip.tolist(), "hand_quat_w": hand[:, 3:7].tolist(),
                "joint_pos": robot.data.joint_pos.tolist(), "lateral_error_m": xy_error.tolist(),
                "insertion_depth_m": depth.tolist(), "tilt_rad": tilt.tolist(),
                "mouth_lateral_error_m": mouth_error.tolist(), "geometrically_seated": seated.tolist(),
            }) + "\n")
        print(f"Scene inspection complete. Diagnostics: {args.log.resolve()}", flush=True)
    if not args.headless and not args.exit_after and simulation_app.is_running():
        sim.pause()
        print("Diagnostics finished. Simulation paused; close the window to exit.", flush=True)
        while simulation_app.is_running():
            simulation_app.update()
            sim.carb_settings.set_bool("/rtx/ecoMode/enabled", True)


if __name__ == "__main__":
    try:
        with torch.inference_mode():
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
        simulation_app.close(wait_for_replicator=False)
