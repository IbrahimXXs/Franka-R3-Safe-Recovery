# Controlled FORGE experiment

This is the characterization layer on the installed FORGE/Panda scene. It is
separate from both the native randomized FORGE demo and the custom FR3 study.
The preparation and pilot validation are complete. Collection is an explicit
mode; launching the default baseline does not start a dataset collection.

The working physics rate follows the **installed FORGE scene default: 120 Hz**
in Isaac Lab 2.3 (`FactoryEnvCfg.sim.dt = 1/120`, inherited by FORGE). Native
policy actions are 15 Hz with decimation 8; this scripted controller and its
measurements run at the physics rate. The main pilot used 120 Hz. The 240/480 Hz
runs were explicit comparison experiments. Omit `--physics-hz` for normal use;
the launcher prints the selected rate and records its source in `study.json`.

## Commands

To collect the initial **100 numerically screened references at native 120 Hz**:

```bash
./forge_study.sh --headless --mode collect --output-dir outputs/Forge-Phase2-100
```

This is an exploratory dataset specific to the configured model and timestep.
The default protocol uses 0.1–1.0 mm offsets and 0.25–4° tilts, across all six
signed families, and probes straight/realigned recovery at actual first depth
crossings of 5, 10, 15 and 20 mm. The separate 5° stress configuration is not
silently included. All previous force, torque, pose and grasp measurements are
retained.

The 100 valid-slot quotas are **8 centered, 18 X-offset, 18 Y-offset,
18 diagonal offset, 19 tilt-only, and 19 offset + tilt**. Families are interleaved
until their quotas fill; signs and tilt axes cycle within each family. Offset
range refers to the total XY offset magnitude, including diagonal cases.
`family_weights` in the FORGE protocol specifies these shares. Other
`--target-valid` sizes scale them using largest remainders (ties follow family
order); very small tests may omit families. Pilot `--slots` retains its original
six-family diagnostic indexing and is separate from collection slot indexing.

The eight centered references remain identical repeatability controls with
fixed 8 s insertion duration. They share `split_group_id=centered_controls` and
`sample_role=repeatability_control`. All retries, checkpoints and recovery
branches of each contact-rich slot share that slot's split group. These fields
are recorded in the ledger and all three dataset CSV tables. A downstream ML
split must use these groups, never individual rows or trajectory IDs; report
controls separately or deduplicate them for evaluation. No ML split is created
automatically. Group IDs are local to the study; pooling studies requires
checking identical configurations across studies as well.

The saved candidate plan reflects the updated ranges and quotas. Existing pilot
outputs retain their original settings; their results do not validate every
configuration in the expanded range. Start a fresh collection directory when
changing the sampling protocol; resume rejects changed protocol/source files.

An accepted reference has completed processing and passed the numerical screen.
Physical stalls, insertion failures, grasp losses and budget violations remain
in the dataset when numerically screened; replacing those would bias it toward
easy cases. Unreached or unmatched recovery checkpoints remain unknown and do
not cause a reference to be replaced. Therefore 100 valid references does not
mean 100 successful insertions or 400 known recovery labels.

Numerical rejects are retried up to five times per slot, preserving the family
and directional stratum while resampling magnitudes. If any slot exhausts its
attempt allowance, the final status is `target_not_met`; the runner does not
claim 100 valid references or relax the numerical screen. `study.json` records
`valid_trajectories`, all attempts and the final status. Tables update after each
completed attempt. Full profiles and the offline viewer are generated when the
run finishes or pauses at a requested batch boundary.

Resume a stopped run using the same command and `--resume`:

```bash
./forge_study.sh --headless --mode collect --resume --output-dir outputs/Forge-Phase2-100
./view_study.sh outputs/Forge-Phase2-100
```

Completed accepted slots are skipped. An interrupted attempt is restarted in a
fresh folder with the same sampled parameters; its partial files remain saved.
Resume checks the protocol, checkpoint depths, physics rate, runner source
hashes and buffer choice. Concurrent writers to the same directory are blocked.
Use the same overrides if the original invocation included any. A normal launch
never overwrites an existing output directory.

For a smaller batch, add `--max-new-attempts 5`, then resume to continue toward
the same quota. `--target-valid` changes the quota for a separate small test.
Collection can take many hours because each reached checkpoint requires up to
two independent preparation/replay/recovery runs. No full collection is started
by the preparation or smoke test.

Other diagnostic commands:

```bash
# Repeat centered insertion and final withdrawal three times
./forge_study.sh --headless --mode baseline --repeats 3

# Independently test both recovery policies at four actual insertion depths
./forge_study.sh --headless --repeats 1 --checkpoints 5 10 15 20

# Small pilot spanning all six families and signed roll/pitch/X/Y directions
./forge_study.sh --headless --mode pilot --checkpoints 10

# Repeat selected contact cases at the upstream default timestep
./forge_study.sh --headless --mode pilot --slots 2 4

# Known applied forces/torques, outside the socket, to check wrist measurements
./forge_study.sh --headless --mode calibration

# Separate 5 degree stress case; outside the initial randomized-plan tilt range
./forge_study.sh --headless --mode pilot --slots 4 --checkpoints 15 \
  --config experiments/forge_challenge.json

# Open a completed result (replace the directory with your run)
./view_study.sh outputs/Forge-Baseline-120
```

Omit `--headless` to see the scene. The window pauses after completion and stays
open; `--exit-after` closes it. DLSS Performance and Eco are enabled.
`--output-dir` must name a new directory. The environment remains
`franka-safe-recovery`, with no driver or package changes.

`--config experiments/forge_phase2.json` supplies the sampling ranges, budgets
and screen/matching thresholds. `--seed` overrides its seed. `--checkpoints`
selects which depths to probe in this pilot invocation; the default is no
checkpoint probes, with a final withdrawal still performed for every reference.
The [100-candidate plan](../experiments/forge_phase2_plan.csv) is a sampling plan,
not collected or accepted data.

## Controlled baseline

The robot, finger actuators, freely grasped peg, socket, SDF collisions,
friction, contact offsets, TGS solver and Cartesian torque law come from the
installed Factory/FORGE implementation. The nominal peg and socket diameters
are 7.986 and **9.0 mm**, measured from the USD meshes. The upstream Python
config's 8.1 mm bore value is stale. Radial vertex clearance is 0.507 mm; the
144-sided wall's inscribed radius gives a conservative 0.5059 mm clearance.
The socket has a 1 mm entry chamfer; the peg has 0.5 mm end chamfers. See the
[geometry audit](../experiments/forge_geometry_audit.json). Effective masses, materials
and collider settings are read back into `scene.json`.

The following deliberate changes isolate the experiment:

- One environment, with fixed initial socket/grasp poses and fixed nominal
  controller gains. Mass/material/dead-zone events and observation noise are
  disabled. The native controller dead zone is zeroed.
- GPU contact allocations are reduced to 262,144 contacts and 65,536 patches,
  with ample capacity for one scene. `--stock-buffers` restores the upstream
  allocations for comparison. The initial centered A/B check gave identical
  numeric samples while reducing GPU memory use by approximately 1.6 GB.
- Full hand pose targets feed the upstream `generate_ctrl_signals` torque law.
  This supports roll and pitch and bypasses the RL action limits and action EMA.
  No trained policy is involved. Torques and smooth pose targets update every
  physics step. The Gym reward and automatic episode-reset loop are not used.
- The first grasp/contact solve is warmed up before recording. Each reset is
  followed by three seconds of free-space settling. Its measured hand-to-peg
  transform is then fixed for the reference command, while actual slip remains
  measurable. Nominal desired peg orientation is exactly upright, compensating
  the small reset/grasp tilt through the hand target.
- A two-second approach establishes the selected offset/tilt above the socket;
  insertion moves the intended peg base from 10 mm above to 20 mm below the mouth
  over eight seconds, then holds for one second. Actual depth and orientation
  are recorded; commands are not treated as achievements.

The nominal controller remains compliant (565 N/m translation, 28 Nm/rad
rotation), so a specified target tilt/offset can relax under contact. This is
part of the experiment and must be assessed using the logged actual pose.
The robot and peg retain the upstream gravity-disabled settings.

`--physics-hz` changes integration and measurement resolution. Continuous
reference paths and controller gains remain fixed; targets are evaluated at
each physics step. The force-observation EMA is adjusted to preserve its native
120 Hz time constant, although it is not used by this scripted controller.

## Measurements and operational limits

Each physics sample includes depth, world peg orientation/velocity, actual
lateral offset, arm and finger states, hand pose, grasp displacement/rotation,
contact normal load and reported separation, contact power, and two wrenches:

- Socket-on-peg **normal plus friction** contact wrench in world axes, with
  torque about the peg base. Its peaks populate `max_force` and `max_torque`.
- Raw incoming wrist joint reaction. It is retained in the documented child
  joint frame and transformed using the composed USD joint transform into
  world axes, with its moment also shifted to the peg base for comparison.

Operational limits are **20 N raw wrist force norm and 1 Nm raw wrist torque
norm at the sensor**, including the initial state and stop transient. They
are checked each physics step. Both contact and wrist peaks are exported so
different torque reference points cannot silently change the interpretation.
Known-load calibration tests verify axes, sign and the moment transform.
The calibration assesses mean vector bias separately from instantaneous
residual norms, which include distal-link and grasp dynamics. Its limits and
all eight load cases are saved in its report. Passing this check does not
establish timestep convergence of insertion forces.

The requested insertion metrics are logged, plus grasp retention and wrist
peaks. `stalled` means less than 0.1 mm achieved advance over 0.5 seconds while
the command advances at least 0.5 mm. Insertion success requires at least
19.5 mm achieved depth for 0.2 seconds and retention of the grasp.
`stalled` is a motion flag, not a jamming label: the rate pilot also triggered
it in a centered run. Interpret it together with contact, achieved alignment
and the recovery probes.

The field `numerically_valid` is an **initial numerical screen**, not a
convergence certificate: no hard-guard abort and no reported contact overlap
larger than 25% of conservative radial clearance (0.126475 mm). A 500 N net contact force or
1 mm overlap triggers the emergency guard. Invalid rows and partial traces
remain saved. Results carry a provisional validation status until comparisons
support quantitative use.

Grasp retention is a separate physical criterion: displacement from the
settled reference grasp must remain within 0.1 mm and rotation within 0.5°.
Crossing it can constitute an unsuccessful recovery even with valid numerics;
it is not automatically treated as a solver failure.

## Recovery experiments

A checkpoint is the **first actual depth crossing** at a completed physics
step. The insertion reference continues independently. Each recovery probe
repeats the entire seeded preparation and command prefix up to that exact step.

Matching checks pose, velocities, all arm joints, both fingers, the peg's pose
within the hand, wrist force, and contact wrench. Position tolerance is 5 µm,
below the radial clearance. Angular, velocity and wrench tolerances are recorded
in the protocol. Missing or unmatched states cannot contribute a label. This
checks observable equivalence; hidden solver/contact memory is not certified.

Both policies begin with a 0.25-second hold at the **attained hand pose**:

- `straight`: retain attained lateral position/orientation and withdraw
  vertically toward a 10 mm gap over six seconds.
- `realign`: first center/upright the peg over two seconds at the attained
  depth, then withdraw over six seconds. The realignment cost is included.

The probe stops after the entire nominal peg is at least 2 mm above the mouth,
normal load is below 0.1 N, and this condition persists for 0.2 seconds with
the grasp retained. Budget/retention violations stop the probe; failed numerical
screens make its label unknown. Positive resisting contact work is a contact
energy proxy, not motor energy or a complete cost for an unfinished recovery.

`Y_R_tested=1` requires one valid matched safe-policy witness; zero requires
valid failures of both tested policies. Unreached, invalid or unmatched
checkpoints remain null. Zero does not prove that every possible recovery
motion would fail. The central plot does not interpolate between checkpoints.

## Output and readiness

`study.json` is the experiment ledger; `trajectories.csv`, `checkpoints.csv`
and `recovery_probes.csv` contain the outcome tables. Each trajectory folder
contains the insertion, final withdrawal, replay prefixes and tested recovery
CSVs. Reports, force plots and an offline interactive viewer are generated.
Exact runner/backend/protocol source snapshots are kept with the run.
In the interactive viewer, **Explore profiles → Recorded profile** selects
reference insertion, final withdrawal, or a checkpoint policy. Select **Time
from recovery start** to align withdrawals at zero, or actual depth to inspect
their contact-force paths. Unmatched probes have no invented recovery trace;
invalid recorded traces require diagnostic viewing.

The intended readiness checks are repeated centered insertion/withdrawal,
known-load sensor calibration, contact cases across the six scenario families,
and timestep comparisons of depth, force, penetration, grasp retention and
recovery outcomes. Numerical failures, mismatches and operational failures must
remain distinguishable before scaling to 100 valid references.

The prepared pilot has already exposed appreciable rate sensitivity: the
same positive-Y case reached a contact-force peak of approximately 4.9 N at
240 Hz and 15.5 N at 480 Hz. Centered free-space tracking also differs before
socket contact; see [the tracking diagnostic](../experiments/forge_tracking_diagnostic.json).
Thus successful insertion and a passed overlap screen are insufficient to
approve the 100-reference collection. The higher rate is not automatically
the more trustworthy result.

Once the pilot, calibration and rate runs finish, reproduce the assessment
inside the project Conda environment:

```bash
python research/validate_forge.py \
  --baseline outputs/Forge-Baseline-120 \
  --calibration outputs/Forge-Wrist-Calibration \
  --studies outputs/Forge-Pilot-120 outputs/Forge-Rate-240 outputs/Forge-Rate-480 \
  --output-dir outputs/Forge-Validation
```

The report retains per-rate metrics, matching-case force plots, individual
acceptance checks and their tolerances. It is the earlier rate-sensitivity
assessment. Current work keeps the upstream 120 Hz default and checks
[repeatability at that fixed rate](../outputs/Forge-Native-Stability/report.md).
Passing a fixed-rate check does not establish cross-timestep convergence;
force/cost conclusions remain specific to this model and timestep until
further numerical validation supports them.

See the [completed pilot findings](../outputs/Forge-Validation/pilot_notes.md)
for the 13 signed cases, checkpoint outcomes, eight-load calibration, and the
separate 5° stress case that stalled near 13.5 mm and then withdrew within the
operational budgets. Its 15 mm checkpoint remained unreached. The initial
randomized plan and this deliberate stress case remain separately identified.
