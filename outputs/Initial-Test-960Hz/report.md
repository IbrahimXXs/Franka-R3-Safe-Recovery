# FR3 recovery study

Synthetic compliant-contact study. Rankings apply to these parameters and recovery policies.

9 completed trials; 1 numerically invalid trials excluded from recovery costs and plots.

Positive Fz is upward fixture-on-peg force. Insertion resistance is max(Fz, 0); retreat resistance is max(-Fz, 0). All raw samples include normal and friction forces.

| Scenario | Recovery | Depth before recovery (mm) | Peak retreat (N) | 50 ms peak (N) | Recovery work proxy (J) | Time to clear (s) | Max overlap (mm) | Contact screen | Outcome |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| aligned | straight | 20.04 | 0.000 | 0.000 | 0.000000 | 3.219 | 0.0000 | passed | cleared_within_budget |
| aligned | realign | 20.04 | 0.000 | 0.000 | 0.000000 | 5.219 | 0.0000 | passed | cleared_within_budget |
| offset | realign | 20.09 | 0.000 | 0.000 | 0.000000 | 5.214 | 0.9180 | review required | cleared_within_budget |
| tilted | straight | 19.06 | 6.944 | 1.270 | 0.001995 | 3.239 | 0.3389 | review required | cleared_over_budget |
| tilted | realign | 19.06 | 0.000 | 0.000 | 0.001070 | 5.218 | 0.4401 | review required | cleared_over_budget |
| loaded_shallow | straight | 9.70 | 3.404 | 0.126 | 0.000222 | 2.669 | 0.2949 | review required | cleared_over_budget |
| loaded_shallow | realign | 9.76 | 0.000 | 0.000 | 0.000256 | 4.630 | 0.3203 | review required | cleared_over_budget |
| loaded_deep | straight | 19.73 | 45.320 | 39.520 | 0.076606 | 3.863 | 0.5975 | review required | cleared_over_budget |
| loaded_deep | realign | 19.89 | 0.000 | 0.000 | 0.000164 | 5.216 | 0.4575 | review required | cleared_over_budget |

The work proxy integrates max(0, −(F·v + τ·ω)) over the full recovery, using contact wrench and peg COM velocity in the same world frame. It is not motor energy; a static jam can have zero work while requiring high force. Time and force limits must be considered separately. Failure to clear means this tested policy failed within its duration, not that all recovery motions are impossible.

Force and torque budgets classify outcomes after execution; the controller does not enforce those budgets. Different achieved depths are reported and must not be treated as matched-depth comparisons. See study.json for all run parameters.

Contact screen: overlaps above 25% of radial clearance require review. A passed screen does not establish timestep convergence; compare refinements before trusting a cost ranking.

![Force profiles](force_profiles.png)

![Force versus actual depth](force_vs_depth.png)

## Numerically invalid trials

These trials stopped at the numerical guard. Their partial CSVs include the offending sample. They are excluded from the plots and recovery-cost comparison; missing costs are not zero. An invalid trial is not evidence of physical jamming or irrecoverability.

| Scenario | Recovery | Time (s) | Phase | Force (N) | Separation (mm) | Partial samples |
| --- | --- | ---: | --- | ---: | ---: | --- |
| offset | straight | 3.8833 | insert | 1058.181 | -2.6455 | [CSV](offset__straight.csv) |
