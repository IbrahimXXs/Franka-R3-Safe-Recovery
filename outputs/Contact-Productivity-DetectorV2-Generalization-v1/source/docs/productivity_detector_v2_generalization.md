# Final fresh held-out Detector v2 evaluation

Run `./productivity_detector_v2_generalization.sh --headless` from the repository root.
The output namespace is `outputs/Contact-Productivity-DetectorV2-Generalization-v1`.
Existing directories are refused. This is evaluation only: no calibration or tuning.

The predeclared plan is `experiments/productivity_detector_v2_generalization.json`.
Generate it once, after all implementation and tests are final, using
`python -m research.productivity_detector_v2_generalization` in the existing conda environment.
It locks the entire collection and analysis implementation, original policy/physics
sources, prior descriptor manifests, configuration and the 192-episode schedule.
Every source is checked before each episode and after analysis. Do not edit any
frozen source or the plan after collection begins. No early stopping or replacements.

64 new paths, eight per family: easy controls, axis/oblique offsets, axis/oblique
tilts, combined offset and tilt, terminal-band targets and late-ramp combined
conditions. Eight easy, 28 moderate and 28 severe cases are assigned before
collection. Four signed variants and two magnitudes per family, ramp onsets
6.8–18.2 mm, full ramp at 20 mm. Easy and terminal labels describe intended input
strata, never guaranteed outcomes. Assets and clearance remain unchanged.

All complete commanded paths are compared against previous Phase 2A/2B,
pilot, 48-condition benchmark and 80-condition development descriptors. Metadata
and floating-point signed zero cannot disguise duplicates. Shared aligned approach
prefixes are intentional. No old outcome is used to select conditions or thresholds.

Policies: nominal, Detector v1 + de-wedging, Detector v2 + identical de-wedging.
The optional force/axial baselines are omitted to keep collection to 192 episodes
instead of 320. Random condition order and balanced policy order are frozen with
seed 20260918. Native FORGE physics is 120 Hz; original reset seed, solver priming,
DLSS performance and Eco remain. No driver or environment changes.

The development-selected v2 configuration is copied byte-for-byte. Normal:
eta < 0.4212659765112803 for two checks. Urgent: eta < 0.2 and deficit acceleration
>=2 mm/s². Terminal: depth-range rate <=0.01 mm/s over 0.5 s endpoint hold.
Original causal eligibility guards remain. The function binding preserves the
exact original de-wedging code object. Hard safety, thresholds, gains, recovery
budgets and success/stall definitions are unchanged. Normal load is offline only.

The primary outcome is paired success over all 64 conditions, keeping safety
failures and timeouts in the denominator. Report wins/losses/both succeed/both fail,
two-sided exact conditional McNemar and a 10,000-sample paired, family-stratified
bootstrap 95% interval. Severe and strict-match analyses are descriptive. Strict
matching requires every common pre-intervention sample to satisfy the existing
FORGE replay tolerances, not just the endpoint. One run per condition/policy is
not a repeated-trial reliability estimate.

Shadow detector replay on identical nominal histories isolates coverage and
unnecessary alarms. Actual interventions on paired nominal successes are reported
separately with matching diagnostics. Lead time is first nominal stall/safety stop
minus first safe alarm. Positive is earlier; missed cases retain their denominator.
Terminal opportunity delay is separate. Branch usage counts actions actually begun.

Artifacts include the requested report, summary, policy summary, paired comparisons,
branch analysis, failure cases, condition plan, validation, provenance, and plots;
also paired statistics, recovery events, trigger/false-trigger tables and full CSV logs.
The original recovery/safety audit and an independent v2 decision replay check every
physics sample. Prior output size/mtime and raw artifact SHA256 audits are retained.

Offline report command:
`python -m research.productivity_detector_v2_generalization_report outputs/Contact-Productivity-DetectorV2-Generalization-v1`

Tests: `python -m unittest discover -s tests` in the existing conda environment.
