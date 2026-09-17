# Force versus productivity with identical de-wedging

Run `./force_productivity_dewedge.sh --headless` in the repository root.
Output: `outputs/Force-vs-Productivity-Dewedge-v1`. Existing output directories
are refused. The experiment is serial and may take roughly 4–6 hours.

First freeze the complete implementation and condition plan once:
`python -m research.force_productivity_design`.
The plan contains 32 development and 64 held-out conditions. Both sets are new,
mutually disjoint, and excluded against all earlier FORGE/Phase2A/Phase2B/pilot
paths, the 80 v2 development cases, and the 48- and 64-condition benchmarks.
Only prior parameter descriptors are used for duplicate exclusion. Eight families
cover nearly aligned controls, axis/oblique offsets, axis/oblique tilts, combined,
terminal-band and late-ramp conditions. Exact centered duplicates are avoided.

Phase A: 32 nominal references and 32 closed-loop episodes at each force threshold
1, 2, 3, 4 and 5 N (192 episodes). Force uses current wrist-force norm strictly
above threshold at two consecutive 0.1 s checks. Eligibility requires a full
0.5 s causal history in a single insert or endpoint-hold phase/segment, above the
same contact-onset +0.1 mm gate. Pose/phase are common eligibility metadata, not
predictive force features. No positive-command-progress restriction is applied,
so force can detect terminal contact. Eta is logged for diagnostics only.

Select one threshold using nominal development shadow alarms: at most 10% of
naturally successful cases alerted, then maximize useful failed-case coverage,
any safe failure coverage, fewer false alerts, capped lead, higher threshold.
Useful means >=0.1 s before the first stall/safety event, or a full 5.25 s recovery
reserve before the timeout when neither event exists. If none meets the false
alert cap, minimize false alerts first and report the limitation. Closed-loop
success does not select the threshold. Save the selected config and SHA256 before
any held-out run. All candidates still receive full closed-loop evaluation.

Phase B: nominal, selected FORCE + DE-WEDGE, and frozen PRODUCTIVITY V2 + DE-WEDGE
on all 64 held-out conditions (192 episodes). No tuning, exclusions, replacements,
or extra repeats based on test outcomes. Force-rise and hybrid are omitted.

Both active policies execute the exact same frozen `execute_dewedge` code object.
Only private Detector, recent_signal and diagnostic Stream bindings differ.
All recovery parameters, actual retreat verification, retry behavior, hard safety,
grip retention, gains, physics/contact, trajectory generator and success/stall
criteria remain unchanged. Native FORGE is 120 Hz with original reset/priming,
DLSS performance and Eco. Normal load is never an online decision input.

The entire collection and analysis implementation is hashed before collection and
checked before every episode. The already-frozen v2 configuration is copied exactly.
The force config is generated once after development and checked through the test.
All old output size/mtime metadata are audited; none is modified.

Final analysis replays all detector and recovery/safety commands, compares both
shadow detectors on identical nominal histories, and reports paired success,
moderate/severe strata and strict full-prefix state matching. Primary paired test:
exact two-sided McNemar; 10,000 paired family-stratified bootstrap samples for a
95% interval. No superiority claim from unmatched differences alone.

Discordances include actual/shared-history trigger ordering, force at productivity
trigger, eta at force trigger, actual/commanded progress rates and recovery outcome.
Low-force low-eta cases and terminal stagnation are distinct (eta is undefined at
zero command progress). High-force/healthy-eta is called unnecessary only when the
nominal trajectory subsequently succeeds. Shadow overlap describes potential
complementarity; it does not validate a hybrid controller. This comparison cannot
separately estimate the effect of replacing fixed retraction with de-wedging.

Outputs include all requested CSVs, reports, configs/condition plan, audits, plots,
per-run logs and frozen source snapshots. `summary.csv` contains held-out episodes;
`development_summary.csv` contains the separate development episodes.

Offline report:
`python -m research.force_productivity_report outputs/Force-vs-Productivity-Dewedge-v1`

Tests: `python -m unittest discover -s tests` in the existing conda environment.
