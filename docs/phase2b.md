# Phase 2B: evolving contact and tested recovery boundaries

Phase 2B starts aligned and introduces misalignment after entry. It uses the
existing FORGE/Panda scene, contact settings, controller gains, fixed preparation
seed, 8 s insertion duration, recovery policies and operational limits. The
native scene timestep remains 120 Hz. Phase 2A's completed 100 references and
archived sources are preserved.

The lateral/tilt command grows linearly between a reached depth of 5 or 10 mm
and 20 mm. The feedback variable is the **maximum actual depth reached through
the previous physics step**, rather than commanded depth or elapsed time:

```
u = clip((maximum_reached_depth - onset_depth) / (20 - onset_depth), 0, 1)
x_target = u * final_x
rotation_target = u * final_rotation
```

The maximum prevents contact jitter from repeatedly unwinding/reapplying the
ramp. If insertion stops advancing, the ramp stops growing. Vertical insertion
continues using the original smooth depth command; its tracking error remains
observable. This is a commanded pose path under a compliant controller, not a
claim that the peg attains the specified tilt. Both commanded and actual motion
are logged, and the full feedback law is replayed for each checkpoint test.

The initial search contains 24 paths: onset 5/10 mm, X/pitch or Y/roll, both
signs, and tilt-only, drift-only or combined motion. Each starts at severity 3.
Severity determines a 3° tilt endpoint and/or 0.5 mm drift endpoint at 20 mm.
Subsequent stages can advance through 4°, 5°, 6°, with proportional drift up to
1 mm. X and pitch signs (or Y and roll signs) are paired; this initial design
does not cover every independent offset/rotation sign combination.

The two-case pilot samples positive X/pitch with onset 5 mm and severity 3,
and negative Y/roll with onset 10 mm and severity 6. It checks the lower and
upper nominal path settings and replay plumbing; it is not a complete boundary search.

The pilot has been run in `outputs/Forge-Phase2B-Pilot`. Both references passed
the numerical and grasp screens. All eight reached checkpoint/terminal states
had safe recovery witnesses; both 20 mm checkpoints were unreached. Thirteen
of sixteen policy replays matched, with three unknown probes. The stronger
path stalled at 17.46 mm (maximum actual tilt about 4.14°) but recovered. No
Y_R=0 transition was found. The [pilot report](../outputs/Forge-Phase2B-Pilot/report.md)
and [audit](../outputs/Forge-Phase2B-Pilot/validation.json) contain the results;
54 unit tests passed. Use a fresh output name if repeating the pilot below.

```bash
./forge_study.sh --headless --mode collect \
  --case-plan experiments/forge_phase2b_pilot.json \
  --output-dir outputs/Forge-Phase2B-Pilot
```

Fixed-depth recovery probes remain at actual first crossings of 5, 10, 15 and
20 mm. A separate **terminal** probe tests the final insertion/hold state,
including when a later fixed checkpoint was not reached. Its filename is
`terminal_straight_recovery.csv` or `terminal_realign_recovery.csv`, distinct from
fixed-depth probes. Terminal state follows the existing one-second hold and may
have relaxed relative to the first stall; it does not label the onset of a stall.
All probes still require independent prefix replay and state matching. Unknown
or unreached checkpoints never become Y_R=0. No limit is changed to manufacture
negative labels. Recovery retries repeat the exact path; numerical rejects do
not resample a different amplitude.

To begin the initial 24-path search (not launched automatically):

```bash
./forge_study.sh --headless --mode collect \
  --case-plan experiments/forge_phase2b_initial.json \
  --output-dir outputs/Forge-Phase2B-Stage1
```

Use `--max-new-attempts 2` for small batches, and `--resume` with the same plan
and output directory to continue. Plans and source hashes are frozen in the
ledger. To analyze completed observations and propose the next stage:

```bash
conda activate franka-safe-recovery
python -m research.phase2b --from-study outputs/Forge-Phase2B-Stage1 \
  --output outputs/Forge-Phase2B-Search1
# Only if next_plan.json was produced:
./forge_study.sh --headless --mode collect \
  --case-plan outputs/Forge-Phase2B-Search1/next_plan.json \
  --output-dir outputs/Forge-Phase2B-Stage2
python -m research.phase2b \
  --from-study outputs/Forge-Phase2B-Stage1 outputs/Forge-Phase2B-Stage2 \
  --output outputs/Forge-Phase2B-Search2
./view_study.sh outputs/Forge-Phase2B-Stage2
```

Each path advances by one severity unit after an observed safe terminal recovery.
Valid recovery failures trigger a zero-amplitude control if needed, then local
midpoint refinement between sampled safe/failed levels to 0.25 severity units.
Unknown tests, conflicting repeat outcomes and observed nonmonotonicity require
review instead of escalation. Safe recovery at the ceiling ends that path with
no negative found. The algorithm does not automatically widen ranges or run new
simulations. Candidate brackets compare different contact histories and do not
prove a single monotone or continuous physical boundary.

`boundary.json` records observed within-trajectory safe-to-failed transitions
with their times and depths. Failures already exceeding a wrist budget at the
checkpoint are flagged separately: these differ from a state initially within
budget whose withdrawal subsequently exceeds it. Y_R=0 remains failure of the
two tested recovery policies within their limits, not proof that no recovery
policy exists. An unknown terminal replay cannot support a safe endpoint claim.

For learning, keep all amplitudes of a path and their branches in one
`split_group_id`. Zero-amplitude controls share a control group. Different paths
also share aligned prefixes: check duplicate histories and exclude common
pre-ramp windows from independent evaluation. No train/test split or model is
created here.

## Auxiliary future-stall dataset from Phase 2A

```bash
conda activate franka-safe-recovery
python -m research.future_stall outputs/Forge-Phase2-100 \
  --output outputs/Forge-Phase2-100/analysis/future_stall \
  --history 0.5 --stride 0.1 --horizons 1 2
```

The derived directory contains `windows.csv` (causal history features and row
indices), `checkpoint_labels.csv` (Y_R beside horizon-specific Y_stall),
`trajectory_events.csv`, provenance in `labels.json`, and a report with a raw
slot097 history figure in PNG/PDF. Existing reference recordings are unchanged.
Choose a new output directory to extract another set of horizons.

Y_stall is the binary observation of the **first confirmed stall** in `(t,t+h]`.
Confirmation uses the existing 0.5 s trailing-window rule: commanded insertion
advances at least 0.5 mm while actual depth advances less than 0.1 mm. The event
is not backdated. Already-stalled states are outside the first-event risk set;
negative labels require a fully observed insertion horizon. Otherwise labels
are censored/unknown. Features use only `[t-0.5,t]`; future outcomes, event times
and eligibility fields must not enter model inputs.

The initial extraction finds three stalled references: slots 076, 096 and 097.
At slot097's 15 mm checkpoint Y_R=1; stall is confirmed 1.6 s later, giving
Y_stall=0 at h=1 s and Y_stall=1 at h=2 s. Overlapping windows remain just three
independent stalled trajectories. This supports the distinction between
recovering now and continuing successfully, but is insufficient to establish a
validated predictive model. A non-stall label alone does not certify safe
progress: force/torque budgets, grasp retention and numerical validity remain
separate requirements. Action-conditioned probabilities also require suitable
action variation and evaluation; these fixed-policy records do not identify
counterfactual safety for arbitrary actions.
