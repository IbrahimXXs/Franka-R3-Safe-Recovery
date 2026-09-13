# FR3 peg-in-hole research

Our own insertion workbench using **Franka Research 3**, with a peg already held
above a chamfered hole. Built on the existing isolated Isaac Sim 5.1 / Isaac Lab
2.3 environment.

## Run

```bash
./run.sh
```

The FR3 settles into its prepared pose, holds the peg 25 mm above the hole for
10 simulated seconds, then pauses. The window stays open until you close it.
**DLSS Performance and Eco mode are enabled on every launch.**

```bash
# Watch a slow, scripted insertion; the window stays open afterward
./run.sh --demo

# Headless insertion check (exits on completion)
./run.sh --headless --demo --steps 300

# Short headless hold check
./run.sh --headless --steps 10
```

`--steps` counts 30 Hz control/logging intervals. The arm controller and physics
run at 120 Hz. `--exit-after` closes the GUI after the requested steps;
`--keep-open` remains accepted for compatibility. This first scene supports one
robot. The seed option is retained, but the scene currently has no randomization.

## Force and recovery study

```bash
./run.sh --study --headless
```

This runs five insertion/misalignment scenarios with straight withdrawal and
realignment before withdrawal. It writes full normal-plus-friction force/torque
profiles, plots, and recovery-cost summaries to `outputs/<timestamp>/`.

```bash
./run.sh --study --scenario loaded_deep --recovery both
./run.sh --study --headless --scenario loaded_deep --recovery straight --friction 0.6
```

The study uses synthetic contact parameters, a 30 g rigidly grasped payload,
and Cartesian impedance control. It records actual depth and tilt so attempted
insertion is not confused with achieved insertion. **See [study definitions and
limitations](docs/study.md)** before using the profiles as evidence of jamming.
Force/torque budgets classify results; they are not controller limits.
If a trial crosses the numerical guard, its partial data are saved and it is
marked `numerically_invalid`; remaining trials continue. Invalid trials have
blank recovery costs and are excluded from comparison plots.
The [verified full GPU sweep](outputs/full_suite_guard_fix/report.md) finished
with eight completed trials and two numerically invalid offset trials; see its
[validation notes](outputs/full_suite_guard_fix/validation.md).

A [saved pilot](outputs/recovery_pilot/report.md) includes both recoveries. Its
[validation notes](outputs/recovery_pilot/validation.md) flag excessive overlap
and timestep-sensitive costs; quantitative rankings still need refinement.

## Scene

- FR3 arm and Franka hand from NVIDIA's FR3 USD asset.
- Custom workbench, teal fixture base, metal socket, and brass-colored peg.
- Peg: **8 mm diameter, 60 mm long**.
- Hole: **9 mm diameter, 25 mm deep**, with a 1 mm entry chamfer.
- Demo: hold for 1 second, descend smoothly for 6 seconds, then hold at
  **20 mm insertion depth**. Short runs can end before insertion finishes.

The peg is rigidly attached to the hand for this first building block. It has
collision geometry and can be blocked by the fixture, but cannot slip or be
released. The ordinary hold/demo arm uses position control with gravity disabled to approximate
compensation. This is a scene and motion baseline, not a calibrated contact or
force-control model. The study mode adds contact-wrench measurements and two
scripted recovery policies. Autonomous search and the existing research planner
are not connected to the robot. The 30 g peg mass/inertia are now merged into
the hand for the rigid-grasp model.

Ordinary hold/demo runs write `outputs/<UTC timestamp>/diagnostics.jsonl`: joint positions,
peg tip pose, lateral error, tilt, insertion depth, and geometric seating status.
Negative depth means the tip is above the opening. Seating status is a geometric
check, not a force-based success measurement. `--log PATH` selects a different
file and overwrites it if it exists.

## Layout

| Path | Purpose |
| --- | --- |
| `run.sh` | Select the project environment and launch |
| `simulation/scene.py` | FR3 configuration, dimensions, and scene geometry |
| `simulation/launch.py` | Launcher, hold/demo control, and diagnostics |
| `simulation/study.py` | Controlled insertion/recovery experiments |
| `simulation/contact.py` | Normal-plus-friction contact wrench |
| `simulation/study_report.py` | Recovery metrics and plots |
| `research/planner.py` | Standalone recovery-filter reference |
| `tests/` | Planner checks and an end-to-end GPU scene check |
| `environment/` | Pinned Conda and Python dependencies |
| `docs/` | Setup, scene details, and original research material |
| `outputs/` | Generated runs; original Panda baseline preserved |
| `.deps/` | Private Isaac Lab checkout and cached FR3 asset |

Run the scene regression check with `python tests/check_scene.py`; run the
planner checks with `python -m unittest discover -s tests -v`.

See [scene details](docs/scene.md), [setup](docs/setup.md), and the preserved
[research plan](docs/research_plan.md).
