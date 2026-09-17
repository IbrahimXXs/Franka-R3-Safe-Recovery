"""Native FORGE/Panda pilot; scripted actions, no learned policy or recovery labels."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--output-dir", type=Path)
parser.add_argument("--seed", type=int, default=20260913)
parser.add_argument("--exit-after", action="store_true")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.output_dir is None:
    args.output_dir = Path("outputs") / ("Forge-Pilot-" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S-%fZ"))
if args.output_dir.exists():
    parser.error("Choose a new output directory; pilot data are never overwritten.")
args.kit_args = (args.kit_args + " --/rtx/post/aa/op=3 --/rtx/post/dlss/execMode=0 --/rtx/ecoMode/enabled=true").strip()
app = AppLauncher(args).app

import torch
from pxr import UsdPhysics
import isaaclab.sim as sim_utils
from isaaclab.utils.math import quat_apply_inverse
from isaaclab_tasks.direct.forge.forge_env import ForgeEnv
from isaaclab_tasks.direct.forge.forge_env_cfg import ForgeTaskPegInsertCfg
from contact import ContactWrench


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, default=str, allow_nan=False) + "\n")


def smooth(t):
    t = min(1.0, max(0.0, t))
    return t * t * (3 - 2 * t)


class LoggedForge(ForgeEnv):
    """Observe each completed physics step without changing native control/stepping."""

    capture = False

    def _compute_intermediate_values(self, dt):
        super()._compute_intermediate_values(dt)
        if self.capture and self.last_update_timestamp > self.logged_timestamp + 1e-9:
            self.logged_timestamp = self.last_update_timestamp
            self.record_sample()

    def record_sample(self):
        contact = self.contact.read(self._held_asset.data.root_pos_w[0])
        rel = quat_apply_inverse(self.fixed_quat, self.held_pos - self.fixed_pos)[0]
        row = {
            "time_s": self.last_update_timestamp - self.start_timestamp,
            "phase": self.phase,
            "depth_mm": (self.cfg_task.fixed_asset_cfg.height - rel[2].item()) * 1000,
            "peg_x_mm": rel[0].item() * 1000,
            "peg_y_mm": rel[1].item() * 1000,
            "force_norm_n": contact["force"].norm().item(),
            "torque_norm_nm": contact["torque"].norm().item(),
            "normal_load_n": contact["normal_load_n"],
            "penetration_mm": -contact["min_separation_m"] * 1000,
            "contact_count": contact["contact_count"],
            "stock_success": bool(self._get_curr_successes(self.cfg_task.success_threshold)[0]),
            "wrist_force_norm_n": self.force_sensor_world[0, :3].norm().item(),
            "wrist_torque_norm_nm": self.force_sensor_world[0, 3:].norm().item(),
        }
        for name, values in (
            ("contact_force_world_n", contact["force"]),
            ("contact_torque_about_peg_base_world_nm", contact["torque"]),
            ("wrist_raw_child_joint", self.force_sensor_world[0]),
            ("forge_filtered_observation", self.force_sensor_smooth[0]),
            ("forge_noisy_force_n", self.noisy_force[0]),
            ("peg_pose_world", self._held_asset.data.root_state_w[0, :7]),
            ("peg_velocity_world", self._held_asset.data.root_state_w[0, 7:]),
            ("joint_position_rad", self.joint_pos[0, :7]),
            ("joint_velocity_rad_s", self.joint_vel[0, :7]),
            ("smoothed_action", self.actions[0]),
            ("requested_action", self.requested_action[0]),
            ("controller_dead_zone", self.dead_zone_thresholds[0]),
        ):
            row.update({f"{name}_{i}": v for i, v in enumerate(values.tolist())})
        if self.writer is None:
            self.writer = csv.DictWriter(self.stream, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        self.rows.append(row)


def describe_scene(env):
    """Record composed collider settings and effective materials, not config assumptions."""
    colliders = []
    bodies = {}
    for prim in env.sim.stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith("/World/envs/env_0/"):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            bodies[path] = prim.GetAppliedSchemas()
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            colliders.append({
                "path": path, "schemas": prim.GetAppliedSchemas(),
                "settings": {a.GetName(): str(a.Get()) for a in prim.GetAttributes()
                             if any(k in a.GetName().lower() for k in ("sdf", "approximation", "contactoffset", "restoffset"))},
            })
    return {"bodies": bodies, "colliders": colliders, "effective_assets": {
        name: {"body_names": asset.body_names,
               "materials": asset.root_physx_view.get_material_properties().tolist(),
               "masses_kg": asset.root_physx_view.get_masses().tolist()}
        for name, asset in env.scene.articulations.items()
    }}


def make_report(directory, manifest):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True, constrained_layout=True)
    depth_fig, depth_ax = plt.subplots(figsize=(9, 5), constrained_layout=True)
    for trial in manifest["trials"]:
        with (directory / trial["csv"]).open() as stream:
            rows = list(csv.DictReader(stream))
        t = [float(r["time_s"]) for r in rows]
        for ax, key in zip(axes, ("force_norm_n", "wrist_force_norm_n", "depth_mm")):
            ax.plot(t, [float(r[key]) for r in rows], label=trial["name"], linewidth=0.8)
        for phase, style in (("insert", "-"), ("retreat", "--")):
            subset = [r for r in rows if r["phase"] == phase]
            depth_ax.plot([float(r["depth_mm"]) for r in subset],
                          [float(r["force_norm_n"]) for r in subset], style,
                          label=f'{trial["name"]} / {phase}', linewidth=0.8)
    for ax, label in zip(axes, ("Peg–socket net force [N]", "Raw wrist force norm [N]", "Actual base insertion depth [mm]")):
        ax.set_ylabel(label)
        ax.grid(alpha=.25)
    axes[0].legend()
    axes[-1].set_xlabel("Trial time [s]")
    fig.savefig(directory / "force_profiles.png", dpi=160)
    plt.close(fig)
    depth_ax.set(xlabel="Actual base insertion depth [mm]", ylabel="Peg–socket net force [N]")
    depth_ax.grid(alpha=.25)
    depth_ax.legend(fontsize=8)
    depth_fig.savefig(directory / "force_vs_depth.png", dpi=160)
    plt.close(depth_fig)
    summaries = [{k: v for k, v in tr.items() if k not in ("reset_parameters",)} for tr in manifest["trials"]]
    if summaries:
        with (directory / "summary.csv").open("w") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(summaries[0]))
            writer.writeheader()
            writer.writerows(summaries)
    lines = ["# Native FORGE pilot", "", "Panda; native 120 Hz physics / 15 Hz actions; full-rate logging.", "",
             "Scripted position targets use measured initial grasp geometry. Native controller, contact settings, "
             "pose noise and dynamics randomization remain enabled. No trained FORGE policy is used.", "",
             "| Trial | Max depth (mm) | Contact peak (N) | Wrist peak (N) | Max overlap (mm) | Exit |",
             "|---|---:|---:|---:|---:|---|"]
    for tr in manifest["trials"]:
        lines.append(f'| {tr["name"]} | {tr["max_depth_mm"]:.3f} | {tr["max_contact_force_n"]:.3f} | '
                     f'{tr["max_wrist_force_n"]:.3f} | {tr["max_penetration_mm"]:.5f} | {tr["status"]} |')
    lines += ["", "The stock success flag uses Factory's geometric reward criterion. These are diagnostics, "
              "not accepted Phase 2 trajectories. Numerical validity and Y_R are deliberately unknown until "
              "the geometry, timestep dependence, grasp retention and matched recovery branching are validated.", "",
              "Contact wrench includes normal and friction forces from socket onto peg, in world axes; moment "
              "is about the peg base. Raw wrist wrench is the PhysX incoming joint reaction in the child joint "
              "frame, including hand/grasp dynamics. FORGE's processed observation is stored verbatim; its "
              "frame naming must not be used to interpret it as a calibrated world-frame torque.", "",
              "![Profiles](force_profiles.png)", "", "![Force versus depth](force_vs_depth.png)"]
    (directory / "report.md").write_text("\n".join(lines) + "\n")


def main():
    directory = args.output_dir.resolve()
    directory.mkdir(parents=True)
    cfg = ForgeTaskPegInsertCfg()
    cfg.seed = args.seed
    cfg.scene.num_envs = 1
    cfg.sim.device = args.device
    # Lengthen only the bookkeeping timeout so insertion and retreat cannot auto-reset.
    cfg.episode_length_s = 60.0
    cfg.sim.render_interval = cfg.decimation
    cfg.sim.render = sim_utils.RenderCfg(antialiasing_mode="DLSS", dlss_mode=0)
    cfg.viewer.eye = (1.3, 0.8, 0.8)
    cfg.viewer.lookat = (0.6, 0.0, 0.12)
    manifest = {
        "schema": "Forge-Pilot-v1", "status": "running", "seed": args.seed,
        "task": "Isaac-Forge-PegInsert-Direct-v0", "robot": "Factory franka_mimic.usd (Panda)",
        "physics_hz": 120, "action_hz": 15, "radial_clearance_mm": 0.5059,
        "measured_bore_diameter_mm": 9.0, "upstream_config_bore_diameter_mm": 8.1,
        "geometry_note": "USD mesh audit corrects stale bore metadata; conservative polygon-wall clearance.",
        "overrides": ["one environment", "60 s timeout", "viewer, DLSS Performance, Eco"],
        "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "trials": [],
    }
    write_json(directory / "study.json", manifest)
    write_json(directory / "config.json", cfg.to_dict())
    env = None
    try:
        env = LoggedForge(cfg)
        env.sim._disable_app_control_on_stop_handle = True
        for index, (name, offset_mm) in enumerate((("centered", 0.0), ("x_positive", 0.1), ("x_negative", -0.1))):
            env.capture = False
            env.reset(seed=args.seed + index)
            scene_info = describe_scene(env)
            write_json(directory / f"{name}_scene.json", scene_info)
            held_paths = [p for p in scene_info["bodies"] if "/HeldAsset/" in p]
            fixed_paths = [p for p in scene_info["bodies"] if "/FixedAsset/" in p]
            if len(held_paths) != 1 or len(fixed_paths) != 1:
                raise RuntimeError(f"Expected one peg and one socket rigid body: {held_paths}, {fixed_paths}")
            env.contact = ContactWrench(env.sim, held_paths[0], fixed_paths)
            initial_finger = env.fingertip_midpoint_pos.clone()
            grasp_offset = initial_finger - env.held_pos
            # The stock socket randomization rotates about world Z only.
            top = env.fixed_pos_obs_frame.clone()
            goal_above = top + grasp_offset
            goal_above[:, 0] += offset_mm / 1000
            # A 10 mm tip gap leaves room for the grasp offset inside the native
            # +/-50 mm fingertip action range (reset itself may start outside it).
            goal_above[:, 2] += 0.01
            goal_insert = goal_above.clone()
            goal_insert[:, 2] -= 0.03  # 10 mm above -> 20 mm below socket mouth.
            action = env.actions.clone()
            action[:, 6] = -1.0  # Success-prediction channel unused by the script.
            env.requested_action = action.clone()
            bounds = torch.tensor(cfg.ctrl.pos_action_bounds, device=env.device)
            parameters = {
                "seed": args.seed + index, "gains": env.task_prop_gains.tolist(),
                "ema_factor": env.ema_factor.tolist(), "pos_threshold": env.pos_threshold.tolist(),
                "rot_threshold": env.rot_threshold.tolist(),
                "contact_reward_threshold_n": env.contact_penalty_thresholds.tolist(),
                "initial_grasp_finger_minus_peg_m": grasp_offset.tolist(),
            }
            csv_name = f"{name}.csv"
            env.rows = []
            env.writer = None
            env.start_timestamp = env.last_update_timestamp
            env.logged_timestamp = env.last_update_timestamp
            status = "complete"
            print(f"FORGE pilot: {name}; requested x offset {offset_mm:+.3f} mm", flush=True)
            with (directory / csv_name).open("w") as env.stream:
                env.capture = True
                for step in range(20 * 15):
                    if not app.is_running():
                        status = "interrupted"
                        break
                    t = (step + 1) * env.step_dt
                    if t <= 2:
                        env.phase = "approach"
                        target = initial_finger + smooth(t / 2) * (goal_above - initial_finger)
                    elif t <= 10:
                        env.phase = "insert"
                        target = goal_above + smooth((t - 2) / 8) * (goal_insert - goal_above)
                    elif t <= 11:
                        env.phase = "hold"
                        target = goal_insert
                    elif t <= 19:
                        env.phase = "retreat"
                        target = goal_insert + smooth((t - 11) / 8) * (goal_above - goal_insert)
                    else:
                        env.phase = "clear_hold"
                        target = goal_above
                    action[:, :3] = (target - (env.fixed_pos_obs_frame + env.init_fixed_pos_obs_noise)) / bounds
                    action[:, :3].clamp_(-1.0, 1.0)
                    env.requested_action = action.clone()
                    _, _, terminated, truncated, _ = env.step(action)
                    env.sim.carb_settings.set_bool("/rtx/ecoMode/enabled", True)
                    if bool(terminated.any() or truncated.any()):
                        raise RuntimeError("Unexpected automatic reset during pilot")
                    if any(r["force_norm_n"] > 500 or r["penetration_mm"] > 1.0 for r in env.rows[-cfg.decimation:]):
                        status = "numerical_guard"
                        break
                env.capture = False
            rows = env.rows
            if not rows:
                raise RuntimeError("Pilot ended before any samples were collected")
            if any(abs(r["time_s"] - (i + 1) * env.physics_dt) > 1e-5 for i, r in enumerate(rows)):
                raise RuntimeError("Physics sample times are discontinuous")
            trial = {
                "name": name, "requested_x_offset_mm": offset_mm, "csv": csv_name,
                "status": status, "samples": len(rows),
                "stock_success_observed": any(r["stock_success"] for r in rows if r["phase"] in ("insert", "hold")),
                "max_depth_mm": max(r["depth_mm"] for r in rows),
                "max_contact_force_n": max(r["force_norm_n"] for r in rows),
                "max_contact_torque_nm": max(r["torque_norm_nm"] for r in rows),
                "max_normal_load_n": max(r["normal_load_n"] for r in rows),
                "max_wrist_force_n": max(r["wrist_force_norm_n"] for r in rows),
                "max_penetration_mm": max(r["penetration_mm"] for r in rows),
                "final_depth_mm": rows[-1]["depth_mm"],
                "numerically_valid": None, "Y_R_tested": None,
                "reset_parameters": parameters,
            }
            manifest["trials"].append(trial)
            write_json(directory / "study.json", manifest)
            print(f'  {status}; max depth {trial["max_depth_mm"]:.3f} mm; '
                  f'contact peak {trial["max_contact_force_n"]:.3f} N; '
                  f'overlap {trial["max_penetration_mm"]:.5f} mm', flush=True)
            if status == "interrupted":
                break
        manifest["status"] = "complete" if len(manifest["trials"]) == 3 else "interrupted"
        make_report(directory, manifest)
        print(f"Pilot data: {directory}", flush=True)
        if not args.headless and not args.exit_after and app.is_running():
            env.sim.pause()
            while app.is_running():
                app.update()
                env.sim.carb_settings.set_bool("/rtx/ecoMode/enabled", True)
    except BaseException as error:
        manifest["status"] = "error"
        manifest["error"] = f"{type(error).__name__}: {error}"
        raise
    finally:
        write_json(directory / "study.json", manifest)
        if env is not None:
            env.capture = False
            env.close()


if __name__ == "__main__":
    try:
        with torch.inference_mode():
            main()
    except BaseException:
        import traceback
        import omni.kit.app
        traceback.print_exc()
        omni.kit.app.get_app().post_quit(1)
        raise
    finally:
        sim = sim_utils.SimulationContext.instance()
        if sim is not None:
            sim.clear_all_callbacks()
            sim.clear_instance()
            sim.stop()
        app.close(wait_for_replicator=False)
