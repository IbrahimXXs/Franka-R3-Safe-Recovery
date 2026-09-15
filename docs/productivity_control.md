# Contact Productivity control pilot

This is a separate, bounded closed-loop FORGE experiment comparing nominal insertion, wrist-force intervention and trailing-productivity intervention. It uses the existing Panda/peg/socket pipeline, native 120 Hz physics, and unchanged hard safety limits. No ML, normal-load predictor, gain adjustment or geometry/material change is involved.

Run the frozen pilot with:

```bash
./productivity_control.sh --headless \
  --calibration experiments/productivity_control_calibration.json \
  --output-dir outputs/Contact-Productivity-Control-v2
```

Choose a new output directory; existing directories are rejected. The default output is `outputs/Contact-Productivity-Control-v1`. There are four frozen Stage4 cases, two repeats and three policies: **24 episodes**. The repeated preparation seeds assess numerical repeatability, not independent samples. Policy order is randomized within each case/repeat.

The calibration file records the input hashes and is checked before collection. To deliberately create a new calibration from the same development protocol:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python \
  -m research.productivity_control \
  --root outputs --output experiments/productivity_control_calibration_v2.json
```

Calibration excludes Stage4 and its shared earlier path groups. Centered duplicates share statistical weight. The frozen thresholds are eta < 0.421266 and wrist force >= 1.035449 N, each for two consecutive eligible checks. Both meet the same 10% group-weighted non-stalled alert budget on development data; these are soft intervention thresholds, distinct from hard safety limits.

Productivity is the raw, unclipped actual-depth change divided by commanded-depth change over the preceding 0.5 s. Checks occur every 0.1 s and need at least 0.1 mm positive command progress, a full contiguous insertion history and actual depth beyond the case's ramp onset + 0.1 mm throughout that history. Stop, retract and rejoin invalidate the history and reset consecutive-check counting. The force arm shares the eligibility gate and two-check debounce, but its decision uses wrist force alone.

At intervention, the controller stops at the **measured hand pose** for 0.25 s, commands a straight upward 1 mm hand translation over 0.5 s, then smoothly rejoins the original path over 0.5 s. Retraction keeps the measured hand orientation. The retry restarts the original smoothstep depth clock at the **actual retracted depth**. The maximum-depth-driven tilt/offset ramp does not rewind; rejoin restores its original target smoothly. No new lateral or angular correction is chosen. This may reload the same contact state and is intentionally tested rather than assumed to resolve it.

Both intervention arms allow at most two interventions. All policies have 20 s after the 2 s approach. Nominal follows the original 8 s descent then holds the endpoint for the remaining time. Success requires the existing 19.5 mm depth for 0.2 s. Safety violations stop immediately at an observed physics tick: 20 N wrist force, 1 Nm wrist torque, 0.1 mm/0.5 degree grasp slip, or the existing 0.126475 mm penetration screen. The inherited reset/preparation is unchanged.

The stall predicate keeps its original numerical values but requires a contiguous insertion segment; intentional retract motion must not create a stall label. A policy cannot be judged from stall count alone: compare success, final depth, elapsed time, failed/exhausted retries and safety stops. `time_to_success_s` is empty for failures; `insertion_time_s` includes all elapsed insertion, intervention and hold time.

Each run logs a physics-rate `trajectory.csv`, `events.json` and `metrics.json`. In addition to full measured wrench, pose, velocity, slip and load, logs include commanded hand pose, raw eta, validity, consecutive count and actions. The nominal offset/tilt fields describe the underlying path; commanded hand pose is the authoritative target during stop/retract/rejoin. Event logs distinguish commanded retraction from achieved peg retreat.

The output includes:

- `summary.csv`, `policy_summary.csv`, `paired_comparisons.csv`, `interventions.csv`;
- `report.md` and depth, productivity, force and outcome comparison plots;
- frozen calibration, schedule, configuration, scene/source snapshots and old-output integrity audit.

Strict common-prefix endpoint matching and prefix motion/wrench RMSE expose reset discrepancies. No mismatched or unsuccessful episode is silently removed. Plots show individual repeats, not confidence intervals treating repeats as independent geometries.

Regenerate the report without Isaac Sim:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python \
  -m research.productivity_control_report outputs/Contact-Productivity-Control-v1
```

Run math and controller-scheduling tests:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python \
  -m unittest tests.test_productivity_control tests.test_productivity_control_execution -v
```

The execution tests exercise the actual runner loop with a CPU fixture stand-in. They verify debounce, history resets, bounded retraction/retry, success dwell and safety priority; they do not claim to validate physical contact behavior.
