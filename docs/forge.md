# FORGE as an experimental backend

The installed Isaac Lab 2.3 task `Isaac-Forge-PegInsert-Direct-v0` extends
Factory's scene and controller. It is a suitable foundation to investigate,
but adopting it does not by itself validate jamming forces or recoverability.
The current pilot uses its Panda robot; the custom FR3 studies remain separate.

## Run the native pilot

```bash
# Watch three scripted insertion/retreat attempts; pauses afterward
./forge_pilot.sh

# Collect the same small pilot without a window, in a new directory
./forge_pilot.sh --headless --output-dir outputs/My-Forge-Pilot
```

Uses the existing `franka-safe-recovery` Miniconda environment, DLSS Performance
and Eco mode. No new dependencies or drivers. Outputs include `study.json`,
`config.json`, `summary.csv`, `report.md`, `force_profiles.png`,
`force_vs_depth.png`, three raw trajectory CSVs and per-reset scene metadata.
This schema is separate from the FR3 study viewer; open the report and PNGs
directly. Existing `run.sh` and `view_study.sh` keep their existing behavior.

The pilot contains centered and ±0.1 mm world-X target offsets, a two-second
approach, eight-second insertion, one-second hold, eight-second retreat and
one-second clearance hold. The intended peg-base depth is 20 mm, with a 10 mm
gap on retreat. Native action bounds clip commands, including any reset pose
outside the bounds. Targets use the measured initial finger-to-peg displacement
and true socket position; a learned policy is **not** loaded or trained.
The peg can slip/rotate in the fingers, so target offset and depth are not
the achieved offset and depth. Those actual values are logged separately.

Native pose/grasp uncertainty, observation noise, controller gain randomization,
action smoothing, mass events and periodic dead-zone randomization remain active.
Different trials use different seeds; they are not a controlled comparison of
offset alone. Only environment count (one), episode timeout (60 seconds, to
allow retreat), rendering and camera settings are overridden.

## What comes from upstream

| Item | Installed configuration |
|---|---|
| Robot | `Factory/franka_mimic.usd`, Panda joints and gripper, wrist force-sensor link |
| Peg | 7.986 mm diameter, 50 mm length; nominal mass 19 g |
| Socket | Actual USD: 9.0 mm bore, 25 mm height; Python config's 8.1 mm is stale |
| Radial clearance | 0.507 mm nominal; 0.5059 mm conservative polygon-wall clearance |
| Collision geometry | SDF on both peg and socket; composed USD resolution 256 |
| Physics | 120 Hz, TGS, 192 position / 1 velocity solver iterations |
| Actions | 15 Hz, decimation 8; control torque recomputed each physics step |
| Contact offset / rest offset | 5 mm / 0 mm |
| Gravity | Disabled on robot and held peg in the stock asset configuration |
| Nominal Cartesian proportional gains | 565 N/m translation; 28 Nm/rad rotation, randomized |
| Force observation | Smoothed incoming wrist joint reaction plus synthetic observation noise |

The 5 mm **contact offset is a contact-generation margin**, not the clearance
or permission for 5 mm overlap. It is recorded as supplied by upstream.

Read effective friction/mass from each `*_scene.json`. In this installed version,
Factory's constructor calls `set_friction` after the startup material events;
the pilot reads back 0.75 static and dynamic friction for peg, socket and robot.
Quoting Forge's configured startup friction range alone would misdescribe the
actual simulation. The config and effective material readback are both retained.

## Force interpretation and validation

The CSV separates these quantities:

- **Socket-on-peg contact wrench:** normal plus friction contact impulses divided
  by the physics timestep, in world axes. Torque is about the current peg base.
  Also logs summed normal load, contact count and maximum reported overlap.
- **Raw wrist wrench:** PhysX incoming joint reaction at the force-sensor link.
  The installed PhysX API documents this in the **child joint frame**. It can
  include hand/grasp dynamics and is not identical to the peg–socket wrench.
- **Forge observations:** upstream filtered six-component signal and noisy
  three-component force, saved verbatim. The upstream variable name
  `force_sensor_world` does not establish a world-frame convention: inspect and
  calibrate its transform before interpreting individual axes or torque.

The collector observes the upstream environment's intermediate-value update,
once per completed physics step. It asserts continuous 1/120-second sample
times. Joint positions/velocities, peg pose/velocity, requested and smoothed
actions, and changing controller dead zones are logged too. A 500 N contact
force or 1 mm reported overlap stops the run at the next action boundary;
this emergency guard is not a numerical validity certificate or an operational
20 N / 1 Nm recovery budget. Data through the guard are preserved.

`numerically_valid` and `Y_R_tested` are intentionally null for this pilot.
The stock success flag is Factory's geometric reward criterion, with a
2.5 mm lateral threshold and proximity to the bottom of the 25 mm socket.
It is not our 20 mm insertion criterion or proof of physically valid seating.

## Moving Phase 2 onto this backend

1. Keep the stock SDF assets, contact solver and Panda while validating force
   frames, geometry, grasp retention, and resolution/timestep sensitivity.
2. Establish a controlled nominal experiment with recorded, fixed nuisance
   parameters. Add randomized friction/gains/grasp later as explicit factors;
   otherwise they confound the effect of signed offsets and tilts.
   In particular, the native randomized wrench dead zone can leave millimetres
   of tracking error under a scripted target, far above the radial clearance.
   A nominal zero-dead-zone variant should be named and recorded explicitly.
3. Add roll/pitch target support in a project subclass. Stock FORGE explicitly
   sets action roll and pitch to zero, so it cannot express the requested tilt
   families unmodified. Keep its torque controller and document the change.
4. Use the audited USD geometry, rather than the stale config diameter, to set
   offsets and tilts. The corrected 0.1–0.75 mm / 0.25–3° ranges straddle the
   actual approximately 0.507 mm radial clearance.
5. Add checkpoint recovery probes at actual depths 5, 10, 15 and 20 mm. Replay
   must reproduce RNG/event timers, controller/filter state, arm state **and
   the freely grasped peg state**. Check matching before assigning a label.
6. Compare 120 / 240 / 480 Hz physics while preserving the 15 Hz action period
   (decimations 8 / 16 / 32). Filtering also has a timestep-dependent time
   constant; explicitly distinguish a native baseline from retuned variants.

Safe recovery remains a statement about tested policies within stated force,
torque, time and clearance limits. A failed policy is not proof that all possible
recoveries are impossible. Neither Factory nor Forge supplies this label for us.
The full 100-valid-trajectory collection has not been started on this backend.

The [first GPU pilot report](../outputs/Forge-Native-Pilot/report.md) contains
7,200 physics samples across three runs. See its
[assessment](../outputs/Forge-Native-Pilot/pilot_assessment.md): all three made
entrance contact, with roughly 9–10 N peaks, but none achieved insertion.

## Sources

- [FORGE paper](https://arxiv.org/abs/2408.04587): force-aware policy learning,
  force-threshold conditioning and dynamics randomization.
- [Installed-version Forge implementation](https://github.com/isaac-sim/IsaacLab/blob/v2.3.0/source/isaaclab_tasks/isaaclab_tasks/direct/forge/forge_env.py).
- [Installed-version Factory scene configuration](https://github.com/isaac-sim/IsaacLab/blob/v2.3.0/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_env_cfg.py).
- [Installed-version peg/socket configuration](https://github.com/isaac-sim/IsaacLab/blob/v2.3.0/source/isaaclab_tasks/isaaclab_tasks/direct/factory/factory_tasks_cfg.py).
