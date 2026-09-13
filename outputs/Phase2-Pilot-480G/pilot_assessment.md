# Phase 2 pilot assessment

**Do not start the 100-valid-trajectory collection on the strength of this pilot.**
The data-generation and labeling pipeline ran, but the present contact model
rejected four of six reference insertions under the declared numerical criterion.
Only centered and X-offset references passed. This pilot is too small to estimate
family-level acceptance rates reliably, but it directly exposes a collection risk.

| Family | Numerically valid reference | Maximum overlap (mm) | 10 mm checkpoint |
| --- | --- | ---: | --- |
| centered | Yes | 0 | Both policies cleared within limits |
| X offset | Yes | 0 | Both policies cleared within limits |
| Y offset | No | 1.2567 | Not reached; unknown |
| diagonal offset | No | 0.4081 | Invalid prefix; unknown |
| tilt only | No | 0.2531 | Valid earlier prefix: both policies cleared; parent later failed the numerical screen |
| offset + tilt | No | 1.0177 | Not reached; unknown |

The two accepted reference trajectories had no contact force. This is not yet a
useful sample of the safe/unsafe recovery boundary. There are **no observed
valid tested-policy-failure labels** in this pilot. Do not manufacture negative
labels from numerical rejects or unreachable checkpoints.

All six requested families were attempted at 480 Hz on GPU, once each, with a
10 mm checkpoint. This first block uses the initial directional strata; the full
100-slot sampling plan contains positive/negative directions and every combined
directional pairing. Signed balance was checked in the sampling tests, not
empirically covered by this six-attempt GPU pilot.

A separate centered CPU plumbing check exercised all four configured depths
(5, 10, 15, 20 mm), both policies, observable replay matching, recovery clocks,
and a no-op resume without duplicate attempts. This checks the implementation;
it does not validate GPU contact behavior or physical jamming at all four depths.
The GPU pilot's recorded source hash predates the addition of recovery-clock
logging and the standalone sampling-plan CLI; the CPU check exercised those
additions. Sampling, controller commands, policy definitions and label rules
were unchanged by those additions. Exact executed source hashes are retained
in each study manifest.

Verification: 22 unit tests passed; Chrome checks passed for Phase 2 checkpoint
labels, insertion metrics and profiles, plus legacy viewer compatibility.

Next experimental gate: isolate/refine the contact response, rerun contact-rich
pilot cases with timestep and geometry/contact-parameter refinements, and assess
whether numerical rejects are concentrated in particular families/directions.
Keep this pilot intact and use a new experiment directory for changed physics.
The current numerical screen is not a convergence certificate.

[Interactive pilot](interactive_viewer.html) · [Dataset report](report.md) ·
[Full Phase 2 design](../../docs/phase2.md)
