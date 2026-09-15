# Gap / insertion-tilt pilot

Nominal clearance is one-sided radial clearance. Trigger depths are 6/10/14 mm.
Commanded tilt ramps smoothly for 1 s after the first actual insertion-depth crossing.
Numerical acceptance does not certify physical convergence. Missing probes remain unknown.

| Radial gap [mm] | Complete references | Numerically screened | Aligned control | Study status |
| ---: | ---: | ---: | --- | --- |
| 0.507 | 7 | 7 | passed | complete |
| 0.4 | 1 | 1 | passed | complete |
| 0.3 | 1 | 1 | passed | complete |
| 0.2 | 11 | 6 | passed | target_not_met |
| 0.1 | 19 | 4 | passed | target_not_met |

Batch status: **target_not_met**.
19 / 23 planned references passed numerical screening.

[Reference outcomes](trajectories.csv) · [Recovery tests](recovery_probes.csv) · [Paired comparisons](comparison.csv)

Each child directory contains its frozen plan provenance, scene audit, raw traces, and report.

Attempt budgets exhausted in: gap_0200, gap_0100. These groups are terminal and will not launch again on resume; numerical rejects remain in the data.
