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

For the upstream Factory/FORGE alternative with its **Panda, SDF peg/socket and
native contact/controller configuration**, see [the FORGE pilot](docs/forge.md).
Run `./forge_pilot.sh` to inspect it or add `--headless` to collect three small
diagnostic runs. This is separate from the custom FR3 experiment below.

The [controlled FORGE experiment](docs/forge_controlled.md) adds repeatable pose
commands, signed tilt/offset cases, grasp monitoring and matched checkpoint
recovery tests. Start its centered baseline with `./forge_study.sh --headless`.
The working setup uses the **installed scene's default physics rate, currently
120 Hz**; leave `--physics-hz` unset. See the
[native-rate repeatability check](outputs/Forge-Native-Stability/report.md).
The [earlier validation assessment](outputs/Forge-Validation/report.md) and
[pilot findings](outputs/Forge-Validation/pilot_notes.md) preserve the 240/480 Hz
comparison results. Those rates were explicit experiments. Results remain
provisional, and the full 100-reference collection is not started automatically.

To start the exploratory FORGE dataset at its native rate:

```bash
./forge_study.sh --headless --mode collect --output-dir outputs/Forge-Phase2-100
```

This targets 100 numerically screened references with checkpoint recoveries at
5/10/15/20 mm. Add `--resume` to continue the same directory after interruption.
The quota is 8 centered controls, 18 each X/Y/diagonal offset, and 19 each
tilt-only/offset + tilt, with offsets 0.1–1.0 mm and tilts 0.25–4°.
Use the exported `split_group_id` for ML splits to keep controls and related
recovery branches together.
Physical failures are retained; unknown recovery labels remain unknown. See
[collection rules and retry limits](docs/forge_controlled.md).

[Phase 2B](docs/phase2b.md) introduces tilt/drift only after entry and searches
bounded paths for changes in tested recoverability. It also provides a causal
future-stall label extractor for the completed Phase 2A references.

The [gap and insertion-tilt study](docs/forge_gap_study.md) reduces the actual
bore clearance to 0.1 mm radial clearance and introduces smooth pitch commands
at 30/50/70% of actual insertion depth. Its 23-case pilot runs aligned controls
before tilted cases and records first-stall and terminal recovery probes:

```bash
python3 simulation/run_gap_study.py --output-dir outputs/Forge-GapTilt-Pilot
```

The [fixed-depth mechanics study](docs/forge_mechanics_study.md) tests 24
depth/friction/tilt combinations at an explicit 240 Hz. It reaches 6/12/18 mm
before applying tilt, audits the peg–hole friction pair while changing only
the hole material, and records separate normal/friction contact streams.
Continuous straight withdrawal is compared with a fully matched replay followed
by XY recentering and upright alignment. See the
[analysis criteria](docs/forge_mechanics_analysis_zh.md) for force, contact,
numerical-validity and recovery interpretations.

The completed 2026-09-15 main matrix has 24 recorded conditions: 20 completed
straight withdrawals and four references stopped by the grasp-slip limit before
withdrawal. See the [Chinese results report](outputs/Forge-Mechanics-FixedSpeed-20260915-review/results_zh.md)
and [compact condition table](outputs/Forge-Mechanics-FixedSpeed-20260915-review/mechanics_overview.csv).
Three independent boundary-angle records and two 480 Hz records are also
complete. At 240 Hz, the 18 mm / 1.5° case briefly obstructed withdrawal and
recentring/alignment reduced its full recovery peak from 4.58 N to 3.75 N.
The full-path benefit and the same near-stall window did not persist at 480 Hz;
initial states and controller update rates also differed. These are exploratory
mechanics results, not validated over-budget recovery labels. The report keeps
each study's denominator separate and links the full frequency audit.

```bash
./forge_mechanics.sh --headless --output-dir outputs/Forge-Mechanics-New
python3 simulation/summarize_mechanics_study.py --study-dir outputs/Forge-Mechanics-New \
  --output-dir outputs/Forge-Mechanics-New-review --plots
```

The [prospective force-budget study](docs/forge_budget_study.md) keeps the
mechanics controllers and gripper model unchanged while applying 3/4/5 N raw
wrist-load budgets from the recorded reference through the whole recovery.
Eight conditions cover depth, angle, friction and budget controls. Each policy
uses a fresh simulator process, and the exporter checks full observed prefixes
before comparing outcomes. Threshold crossings stop subsequent task commands;
truncated peaks and untested branches remain explicit. See the
[Chinese budget results](outputs/Forge-Budget-V1-20260915-review/results_zh.md).
The completed study contains 13 executed branches and 3 planned branches left
untested after reference-stage violations. At 18 mm / 1.5° / friction 1, the
4 N straight branch stops at 4.54 N while recentering/alignment clears with a
3.75 N full-recovery peak; their complete recorded reference prefixes match.
The 5 N straight branch clears. These sampled wrist-load results do not
calibrate fragile-part strength or finger gripping capacity.

The [6-degree angle extension](docs/forge_budget_angle6_study.md) adds a separate
1.5/2/3/4/5/6-degree command sweep at 18 mm and 4 N. Logs distinguish the target
angle from the actually attained pose and retain reference-stage terminations.
The inherited 1-second tilt ramp also increases commanded angular speed as the
target angle rises. See the [angle extension results](outputs/Forge-Budget-Angle6-20260915-review/results_zh.md).

```bash
python3 simulation/run_budget_study.py --output-dir outputs/Forge-Budget-New
```

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

## Phase 2: randomized insertion characterization

The [Phase 2 protocol](docs/phase2.md) targets 100 numerically valid 480 Hz GPU
reference trajectories across six signed offset/tilt families. Independent
recovery probes at 5, 10, 15 and 20 mm produce `Y_R_tested` labels with explicit
unknowns for invalid or unmatched states. The full collection is not started
by setup; first inspect the pilot and its numerical rejection rate.

```bash
./run.sh --phase2 experiments/phase2_pilot.json --headless --physics-hz 480 \
  --device cuda:0 --output-dir outputs/Phase2-Pilot-480G
./view_study.sh outputs/Phase2-Pilot-480G
```

Use a new output directory for a fresh pilot, or the documented resume command
for an interrupted run with unchanged code/configuration. The full protocol is
[experiments/phase2.json](experiments/phase2.json); its first 100 planned candidates
are saved in [phase2_plan.csv](experiments/phase2_plan.csv).


## Interactive results viewer

```bash
./view_study.sh outputs/full_suite_guard_fix
```

Replace the path with a specific study folder. This builds and opens an offline
HTML dashboard with selectable trials, zoomable force/torque profiles, phase
filters, force versus depth, recovery-cost comparisons, and the original report
and images. Invalid trial costs stay excluded. No simulation or extra Python
packages are required. See the [viewer guide](visualization/README.md).


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
| `view_study.sh` | Build and open an interactive viewer for one study |
| `visualization/` | Offline dashboard generator, browser UI, and bundled plotting library |
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
