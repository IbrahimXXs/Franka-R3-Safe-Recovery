# Phase 2: randomized insertion characterization

The question is whether a tested recovery can clear the peg within operational
limits from an observed insertion state. Insertion force is a feature, not the
recoverability label itself. This is an exploratory simulation dataset, with
explicit numerical acceptance criteria and a finite recovery-policy set.

## Prepared experiments

- **Full protocol:** [experiments/phase2.json](../experiments/phase2.json), targeting
  100 numerically valid trajectories at **480 Hz, GPU physics (480G)**.
- **Pilot:** [experiments/phase2_pilot.json](../experiments/phase2_pilot.json), one
  attempt per family, with a 10 mm recovery checkpoint. It deliberately does not
  resample until it gets six valid results. This tests data generation and exposes
  numerical failures before committing to a long collection.
- **Reviewable first candidates:** [phase2_plan.csv](../experiments/phase2_plan.csv).
  This is a plan, not simulated data or a promise that those samples will pass.

```bash
# Preview / regenerate the planned candidates, without Isaac Sim
python3 research/phase2_protocol.py

# Pilot on 480 Hz GPU physics
./run.sh --phase2 experiments/phase2_pilot.json --headless \
  --physics-hz 480 --device cuda:0 --output-dir outputs/Phase2-Pilot-480G

# Full collection, when ready to run it
./run.sh --phase2 experiments/phase2.json --headless \
  --physics-hz 480 --device cuda:0 --output-dir outputs/Phase2-100-480G

# Resume an interrupted collection with the SAME protocol, physics, and source
./run.sh --phase2 experiments/phase2.json --phase2-resume --headless \
  --physics-hz 480 --device cuda:0 --output-dir outputs/Phase2-100-480G
```

The full collection is not launched automatically. It can take substantially
longer than the original ten-trial study: each trajectory can require eight
independent recovery probes, each with its own insertion-prefix replay. At the
attempt cap the design permits up to 500 reference insertions and 4,000 probes.
Reserve runtime and storage based on the pilot before starting the full job.

## Sampling and controls

| Family | Valid-slot target | Randomized perturbation |
| --- | ---: | --- |
| Centered | 17 | Zero offset and tilt; randomized insertion duration |
| X offset only | 17 | Signed X offset, Y = 0 |
| Y offset only | 17 | Signed Y offset, X = 0 |
| Diagonal offset | 17 | Equal-magnitude X and Y components, all four quadrants |
| Tilt only | 16 | Signed roll or pitch, all four cardinal tilt directions |
| Offset + tilt | 16 | All offset quadrants crossed with the four tilt directions |

Offset **radial magnitude** is uniform from 0.1 to 0.75 mm. Diagonal components
are radius / √2, so a diagonal sample is not inadvertently √2 harder. Tilt
magnitude is uniform from 0.25° to 3°. Both positive and negative roll and pitch
are sampled. No yaw perturbation is introduced for this rotationally symmetric
peg. Orientation is `Ry(pitch) Rx(roll) Qdown`, with rotations in world axes.
Offsets are relative to the bore center. The commanded rotation is about the peg tip.

Insertion duration is uniform from 5 to 7 s, independent of family, with the same
smoothstep motion from a 25 mm initial gap to a 20 mm commanded depth. Every
reference has a 0.5 s baseline and a 0.5 s final hold. Randomized duration gives
centered controls a defined source of variation rather than presenting identical
runs as independent geometric conditions. These remain simulation samples, not
independent hardware subjects.

Use the existing 8 × 60 mm, 30 g peg, 0.5 mm radial clearance, 25 mm bore, friction
0.3, compliant contact parameters, rigid grasp, compensated-gravity assumption,
and Cartesian impedance controller. Physics parameters stay fixed within the
batch and are recorded in `study.json`. DLSS Performance and Eco mode remain enabled.
Phase 2 force/torque budgets come from its JSON protocol, not the older study flags.

Slots are round-robin by family; signed directions are fixed by slot. Each slot
gets at most five independent magnitude/duration draws within that same family
and direction. The first numerically valid trajectory fills the slot, whether
insertion succeeds or fails. An exhausted slot remains unfilled; the batch ends
with `target_not_met`, not a misleading claim of 100 valid trajectories. Rejected
attempts stay in the ledger and raw files. Never substitute easier families or
silently narrow the distributions. Report rejection rates: the accepted dataset
is conditional on the simulator's numerical acceptance and is not an unbiased
sample of the original parameter distribution.

## Reference insertion labels

`trajectories.csv` has the requested fields, evaluated over baseline, insertion,
and final hold; recovery trajectories are separate.

| Field | Definition |
| --- | --- |
| `insertion_success` | Achieved depth ≥19.5 mm for ≥0.2 s |
| `max_force` | Maximum net contact-force magnitude, N |
| `max_torque` | Maximum contact-torque magnitude at peg COM, N m |
| `max_normal_load` | Maximum summed normal contact load, N |
| `max_penetration` | Maximum negative contact separation magnitude, mm |
| `max_depth` | Maximum achieved insertion depth, mm |
| `stalled` | In an insertion-phase 0.5 s window: commanded advance ≥0.5 mm but achieved advance <0.1 mm |
| `force_budget_exceeded` | Any sample exceeds the default 20 N net-force limit |
| `torque_budget_exceeded` | Any sample exceeds the default 1 N m torque limit |
| `numerically_valid` | No hard numerical guard and overlap ≤25% of radial clearance throughout |
| `outcome_trustworthy` | Same numerical screen; prevents interpreting an invalid trajectory's observed outcome as reliable |

A depth success is not a claim of successful force-limited insertion. A stall is
a tracking observation, not proof of frictional jamming. Recorded booleans and
maxima for rejected trajectories are diagnostic only.

The 500 N / 1 mm hard guard remains unchanged and stops an invalid reference or
probe. The stricter 0.125 mm overlap screen determines dataset acceptance at the
baseline clearance. It is a numerical screen, not material identification or
proof of timestep convergence. A 100-valid target may be unattainable at the
current contact settings; that is a result to address, not a reason to relax
validity silently.

## Checkpoints and recovery branches

Full-protocol checkpoints are the **first actual crossings** of 5, 10, 15 and
20 mm depth. Save the crossing step, achieved pose, seven joint positions and
velocities, peg velocity, force/torque, normal load and prefix maxima. Never
replace an unreached actual-depth checkpoint with a commanded-depth checkpoint.
Unreached checkpoints receive an unknown label.

The reference insertion continues without being interrupted by its probes.
For each checkpoint/policy, reset to preparation and replay the exact insertion
command prefix up to that recorded physics step. Compare against the original
checkpoint before executing the recovery:

- Tip position within 0.1 mm; orientation within 0.2°.
- Each joint position within 0.01 rad; velocity within 0.05 rad/s.
- Peg linear velocity within 0.005 m/s; angular velocity within 0.05 rad/s.
- Net contact-force vector within 1 N; contact-torque vector within 0.05 N m.

All tolerances are stored in the protocol. These are observable-state matching
checks, **not an exact snapshot/restore of PhysX friction patches, warm-start
impulses, or other hidden solver state**. Near a contact transition, replay may
fail to match and must yield unknown. A passing check supports an approximate
matched-state experiment, not an exact counterfactual or a certified boundary.
Both policies start independently; one policy never follows another's recovery.

Two fixed candidate policies are tested:

1. **Straight:** freeze the achieved tip pose for 0.25 s, then withdraw vertically
   over 6 s while keeping the attained orientation and lateral offset; hold clear
   for 0.5 s.
2. **Realign:** the same initial stop; remove the attained tilt and lateral offset
   over 2 s at the starting depth, then the same retreat and clear hold.

The stop transient, initial state, realignment, retreat and final hold all count
against operational limits. Exceeding 20 N net force or 1 N m torque stops that
probe and records an operational failure. Recovery success needs the entire peg
≥2 mm above the mouth and normal contact load <0.1 N for 0.2 s, with no operational
violation throughout the executed policy. Actuator effort caps remain active.
Failed probes have partial costs; those are not costs of a completed withdrawal.

## Recoverability labels

The conceptual target is `Y_R(t) = safe recovery possible from this state`.
The measurable label in this first phase is **`Y_R_tested`**:

- **1:** at least one valid, sufficiently matched probe cleared within limits.
- **0:** both tested policies failed valid, sufficiently matched tests.
- **null:** depth unreached, invalid prefix/probe, or unmatched/missing replay;
  a successful eligible policy can still provide a positive witness if the other
  test is unresolved.

Zero is relative to these two policies and their durations. It does not establish
that every recovery policy would fail. Store per-policy outcomes alongside the
aggregate label. Do not train on unknown labels as if they were negative examples,
and do not fill labels continuously between checkpoints by interpolation.

## Dataset and central plot

Each experiment contains:

- `study.json`: complete protocol, physics, source hashes, attempt ledger, full
  checkpoint states and replay diagnostics. Updates are atomic.
- `trajectories.csv`: one reference characterization per attempted trajectory.
- `checkpoints.csv`: pre-recovery state/prefix features, `Y_R_tested`, eligibility,
  parent validity and reasons for unknown labels.
- `recovery_probes.csv`: policy outcomes, matching errors, peak loads, work and time.
- `recovery_characterization.png` / `.pdf`: checkpoint depth versus insertion-state
  force colored by recovery label, plus successful policy force costs by depth.
- `report.md`: counts, validity caveats and links.
- One folder per attempt: `insertion.csv`, `trajectory.json`, and independent
  `dDEPTH__POLICY__prefix.csv` / `dDEPTH__POLICY__recovery.csv` files.

The first plot is a characterization scatter, not an estimated decision boundary.
Invalid or unfinished parents are drawn as unknown. Preserve a separate analysis
of valid prefixes from rejected references; filtering by later trajectory validity
can bias which early states remain in the dataset. Include family, signs and seed
in stratified analyses, and keep every checkpoint, retry and recovery from the
same slot in the same training/test split. Never use final-trajectory maxima or
recovery outcomes as predictors of an earlier checkpoint's recoverability.

Interrupted attempts are replayed from preparation on resume and written into a
new suffixed folder. Previously completed attempts are retained; source, protocol
or physics changes require a new experiment directory.

The experiment builds on contact-aware manipulation principles in
[Tedrake's manipulator-control notes](https://manipulation.mit.edu/force.html).
The policy set, sampling ranges, thresholds and replay tolerances here are our
explicit experimental choices, not externally validated physical limits.

## Completed pilot and collection gate

The [GPU pilot assessment](../outputs/Phase2-Pilot-480G/pilot_assessment.md) records
2 of 6 numerically accepted reference trajectories; both accepted cases were
contact-free. Four were rejected, and no valid tested-policy-failure label was
observed. The full 100-trajectory collection has **not** been started.
This is a working collection/labeling pipeline with a numerical-model limitation,
not evidence that the current model is ready to characterize the recovery boundary.

Open its Phase 2 dashboard with:

```bash
./view_study.sh outputs/Phase2-Pilot-480G
```
