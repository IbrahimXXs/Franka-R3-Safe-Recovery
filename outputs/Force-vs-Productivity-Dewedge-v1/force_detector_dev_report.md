# Force detector development

32 entirely new conditions; 32 nominal references plus 32 closed-loop runs for each of five force thresholds. Every closed-loop candidate uses the exact frozen de-wedging implementation.

| Force threshold (N) | Useful failed cases | Any failed cases alerted | Alerts on successes | Closed-loop success /32 | Interventions | Verified | Retry successes | Safety stops | Admissible |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 1 | 3/13 | 6/13 | 0/19 | 25 | 6 | 6 | 6 | 5 | True |
| 2 | 0/13 | 3/13 | 0/19 | 23 | 4 | 4 | 4 | 5 | True |
| 3 | 0/13 | 1/13 | 0/19 | 21 | 2 | 2 | 2 | 6 | True |
| 4 | 0/13 | 0/13 | 0/19 | 19 | 0 | 0 | 0 | 6 | True |
| 5 | 0/13 | 0/13 | 0/19 | 19 | 0 | 0 | 0 | 5 | True |

| Threshold (N) | Median safety lead (s) | Median stall lead (s) | Actual interventions on nominal successes |
|---|---:|---:|---:|
| 1 | N/A | 0.033 | 0 |
| 2 | N/A | -0.442 | 0 |
| 3 | N/A | -0.842 | 0 |
| 4 | N/A | N/A | 0 |
| 5 | N/A | N/A | 0 |

Selected: **force > 1 N for two consecutive checks**. Configuration SHA256: `e6e7f8cc2faa72c0dbba302a80cb7b29fea0a30c822d911592467dcd153654c2`. False-alert cap met: True.

Selection uses nominal development shadow alarms only: at most 10% of successful trajectories alerted; then maximize useful failure coverage, any safe failure coverage, fewer false alerts, capped lead, and a more conservative threshold. If no candidate meets the cap, minimize false alerts first and explicitly report that limitation. Closed-loop successes do not select the threshold.

Useful means a safe alert at least 0.1 s before the first nominal stall/safety event. For failures with neither event, require 5.25 s remaining before the episode deadline, preserving the existing full recovery budget. The latter is an operational time reserve, not an early stall/safety lead claim.

Force decision uses wrist force only. The original contact-onset gate and a full same-phase/segment 0.5 s causal history establish eligibility; insert and endpoint hold are eligible. Positive commanded progress is not required. Two adjacent 0.1 s checks reject single-sample spikes. No normal load or productivity value enters the decision.

Per-case stall/safety lead times and trigger states are in force_development_shadow.csv; per-run outcomes are in development_summary.csv. Development data are not included in the final held-out success estimates. Force-rise and hybrid variants were omitted before collection.

Frozen at 2026-09-16T17:50:58.537700+00:00, before any held-out episode. No prior benchmark outcome or held-out result was used for selection.
