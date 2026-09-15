# Bounded productivity-triggered de-wedging

Run the frozen 24-episode pilot (four Stage4 cases × two repeats × nominal, previous axial-only unloading, and de-wedging):

```bash
./productivity_dewedge.sh --headless --output-dir outputs/Contact-Productivity-Dewedge-v1
```

Existing directories are refused. Native FORGE 120 Hz physics, DLSS Performance and Eco remain unchanged. The detector, calibration, hard safety limits, case descriptors and baseline implementations are frozen against Contact-Productivity-Unloading-v1. No ML or retuning.

De-wedging captures actual peg pose/grasp, stops for 0.25 s, then smoothly relaxes x/y and relative roll/pitch toward neutral over 0.5 s while holding commanded peg depth. Relative yaw is preserved. Retraction then follows the previous 5 mm / 4 s command ramp. Actual depth must retreat at least 0.5 mm and remain there for 0.25 s, followed by another 0.25 s hold. All recovery phases after the initial stop share the original 5 s deadline. Rebound resets the dwell, without extending the budget. Hard safety has priority on every tick.

A successful hold permits the previous 0.5 s rejoin, but retry retains the measured peg lateral pose, orientation and grasp rather than restoring the imposed misalignment. Actual achievement is logged; neutral pose is not assumed. The original 20 s insertion budget and two-intervention cap remain. Failed unloading terminates the episode.

Outputs include per-tick trajectories/events, summary.csv, unloading_events.csv, paired_comparisons.csv, paired_load_changes.csv, policy_summary.csv, report.md, five comparison plots, source snapshots and validation.json. Safe crossing, sustained verification, readiness, retry exposure and insertion success are separate outcomes. Failed/unsafe endpoints and reset mismatches remain visible. Normal load is analysis-only.

Offline regeneration (no Isaac):

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python -m research.productivity_dewedge_report outputs/Contact-Productivity-Dewedge-v1
```

Focused tests:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python -m unittest tests.test_productivity_dewedge -v
```
