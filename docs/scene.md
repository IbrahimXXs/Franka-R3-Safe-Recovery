# Custom FR3 insertion workbench

The project owns the bench, fixture, peg geometry, robot configuration, and
launcher. It no longer launches the stock Factory/Panda task.

## Robot asset

Source: NVIDIA's [Isaac Sim 5.1 robot asset catalog](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/assets/usd_assets_robots.html#frankarobotics),
`Isaac/Robots/FrankaRobotics/FrankaFR3/fr3.usd` under the Isaac 5.1 asset root.
The unmodified USD and its `configuration/fr3_robot_schema.usd` sublayer are
cached under `.deps/fr3/`. If the local USD is absent, the launcher references
the official cloud asset instead. No new Python packages are needed.

Downloaded asset root:
`https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaFR3/`

SHA-256:

```text
fr3.usd
edd3be9975fa94a9add48a691d7daccb3725c8546d85272d528e36c16a2d2945
configuration/fr3_robot_schema.usd
605dcf77d193bde0d19bb6cc5c7f355a3245cef1368a885a8002f12cf346111f
```

The runtime checks the fixed-base articulation and seven `fr3_joint1` through
`fr3_joint7` arm joints. The fingers are `fr3_finger_joint1/2`; the controller
uses `fr3_hand`. The USD's joint limits are retained. Position-drive gains and
effort caps are simulation settings and have not been calibrated against hardware.
The asset's two otherwise massless fixed frames receive negligible explicit
mass/inertia overrides in the scene to avoid PhysX inertia warnings.

## Geometry and frames

All dimensions in `simulation/scene.py` use meters. World Z is up. The tabletop
is at Z = 0.75 m; the hole entry center is `(0.50, 0.0, 0.865)` m. The robot is
mounted at `(0, 0, 0.762)` m. The peg extends along hand +Z, which points downward
in the prepared pose. Its center is 105 mm from the hand origin and its tip is
135 mm from that origin.

The 8 mm peg fits a 9 mm bore, giving nominal 0.5 mm radial clearance. This is an
intentionally forgiving starting task. The bore is a 192-sided static triangle
mesh; its mouth widens to 11 mm over the upper 1 mm. The mesh has no face across
the opening. The pedestal underneath supplies the bottom of the 25 mm blind
hole. Peg/socket collision contact offset is 0.05 mm and rest offset is zero; tighter future
clearances require revisiting geometry resolution and contact settings.

The peg is a collision shape on the hand rigid body. Its 30 g synthetic mass
and analytical cylinder inertia are combined with the original hand mass
properties. It has no independent slip or release. The fingers visually close
to an 8 mm opening. A frictional grasp and calibrated material properties are
later steps. See [the force study](study.md) for instrumented experiments.

## Motion and diagnostics

Startup settles into the prepared pose for at least 3 simulated seconds,
allowing up to 10 seconds for convergence. The default
run then holds for 300 logging intervals, or 10 seconds, and pauses with the GUI
open. `--demo` uses a smooth vertical trajectory: 1 second hold, 6 seconds descent,
then a hold at 20 mm insertion. Differential IK updates position-drive targets
at 120 Hz, with rendered frames and state records at 30 Hz. Each joint command
increment is capped and clamped to the USD's limits.

Geometric seating requires depth between 19.5 and 25 mm, tilt below 0.01 rad,
and both the shaft's tip and its intersection with the hole mouth inside the
nominal radial clearance. This does not certify stable contact or safe force.
The JSONL state is privileged simulator state, not sensor measurements.

The GUI stays open by default; headless runs exit. In this pinned Isaac Lab
version, the launcher disables the standalone STOP callback's wait-for-Play loop
so closing a window or finishing a headless run does not hang. Eco mode is
reapplied after playback events. The upstream Isaac Lab checkout is unchanged.

## Validation

GPU checks cover the prepared hold, the 300-interval insertion, a deliberately
15 mm shifted fixture that blocks descent, and GUI rendering/closure. Validation
logs are kept under `outputs/fr3_validation/`. These checks establish a usable
scene baseline, not hardware fidelity or a robust insertion policy.
