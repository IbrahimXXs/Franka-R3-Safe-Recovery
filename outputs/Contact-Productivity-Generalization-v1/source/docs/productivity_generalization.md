# Frozen generalization benchmark

This benchmark runs 48 unseen contact-condition paths on the current audited FORGE peg/socket assets, one run per condition and policy: 192 episodes. The four policies are the unchanged nominal, original force-threshold fixed retract, axial-only verified unloading, and productivity-triggered de-wedging implementations.

```bash
./productivity_generalization.sh --headless --output-dir outputs/Contact-Productivity-Generalization-v1
```

The launcher refuses existing output directories. Native 120 Hz physics, DLSS Performance and Eco remain. Collection takes roughly three hours on the current machine. A source or plan edit during collection aborts; the collector never recalibrates or replaces failed conditions.

`experiments/productivity_generalization_frozen.json` records the complete pre-collection condition set, schedule, timestamp, prior-data overlap audit, policy/calibration/physics hashes and statistical analysis settings. Six families × two severities × four signed variants cover axis/oblique offsets, axis/oblique tilts, cross-axis combinations and four-component combinations. Moderate endpoint magnitudes: 0.7 mm / 3.5 deg; severe: 1.2 mm / 7 deg. The 7, 12 and 15 mm ramp onsets each have 16 cases. The design is balanced but not a full factorial. All paths start aligned and use the existing depth-dependent command helper and 8 s nominal command.

No clearance variation: the current backend requires the audited unit-scale 9 mm bore. Changing metadata would not change geometry. No assets, geometry pipeline, friction, controller, detector, threshold, recovery budget or hard safety limits are changed.

The original force threshold (about 1.035 N) and original fixed-command recovery remain distinct from the 20 N hard force stop. Both productivity policies retain the exact eta detector (about 0.4213), eligibility and two-check trigger. De-wedging retains its actual 0.5 mm retreat verification, 0.25 s verification plus 0.25 s extra hold, 5 mm command cap / 5 s deadline, and measured-pose retry.

Outputs include summary.csv, per_condition_results.csv, policy_summary.csv, stratified_results.csv, detector_checks.csv, recovery_events.csv, paired_comparisons.csv, paired_policy_effects.csv, failure_cases.csv, failure_case_analysis.md, report.md, comparison plots, all 48 condition-history plots, validation.json, source snapshots and raw per-run logs/events/metrics.

Productivity diagnostics for nominal/force are offline shadow-detector results on those policies' own histories; they are not executed interventions. Verified unloading is enforced only in axial/de-wedging. All failures and reset mismatches remain in primary denominators. Success differences are paired by condition; bootstrap uncertainty resamples conditions within families. This is structured contact-condition generalization on fixed assets, not population-level or geometry/material generalization.

Offline reporting:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python -m research.productivity_generalization_report outputs/Contact-Productivity-Generalization-v1
```

Tests:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python -m unittest tests.test_productivity_generalization -v
```
