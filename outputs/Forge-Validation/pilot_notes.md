# Pilot findings

**The experiment is prepared and pilot-tested; the 100-reference collection remains on hold.** The force profiles and an operational torque label change with physics rate. These are provisional simulation observations, not quantitative recoverability rankings.

The backend uses the upstream FORGE/Panda assets, freely grasped peg, SDF contacts and Cartesian torque law. Controlled targets support signed roll and pitch; nuisance randomization is disabled. The measured bore is 9.0 mm, despite stale upstream 8.1 mm metadata. Conservative radial clearance is 0.5059 mm.

- The 13-case pilot covers all six families with signed offsets and rotations. All 13 inserted, passed the overlap screen, retained the grasp, and completed final withdrawal.
- The centered baseline tested straight and realigned recovery at 5, 10, 15 and 20 mm: all eight probes replay-matched and cleared within the budgets.
- At 10 mm in the signed pilot, 26 probes were considered: 22 matched safe recoveries and 4 unmatched replays. Each reference had at least one safe witness. Unmatched outcomes remain unknown.
- The additional 20 mm tilted checkpoint had a matched safe straight retreat and an unmatched realignment replay. Contact had released at that checkpoint; it is not a loaded-jam recovery demonstration.
- All eight known-load calibration cases passed the separate static-bias and instantaneous-residual checks. These verify wrench axes and moment handling, not contact convergence.
- All 33 software tests passed. Browser testing verified insertion, final-withdrawal and checkpoint-policy profiles, recovery time starting at zero, unmatched-probe exclusion, and legacy study compatibility.

## Rate sensitivity

| Physics rate | Centered contact peak (N) | Y-offset contact peak (N) | Tilt contact peak (N) | Tilt wrist torque peak (Nm) | Tilt torque budget exceeded |
|---|---:|---:|---:|---:|---|
| 120 | 0.000 | 4.208 | 0.989 | 0.054 | False |
| 240 | 0.000 | 4.869 | 5.696 | 0.256 | False |
| 480 | 3.807 | 15.481 | 23.604 | 1.179 | True |

Centered lateral tracking already differs before socket contact. These runs vary integration, per-step controller sampling and seeded preparation together; they do not identify the contact solver as the sole cause. The next validation task is to isolate and stabilize that tracking, then repeat the contact/recovery rate comparison. Increasing the trajectory count would not resolve this discrepancy.

## Deliberate 5 degree stress case

The separate stress reference stalled at 13.472 mm and did not meet insertion success. It retained the grasp and passed the initial numerical screen (maximum reported overlap 0.02927 mm). Straight withdrawal then cleared within the budgets in 4.217 s, with peak wrist force 3.811 N and positive resisting contact work 6.077 mJ. That work is a provisional contact-energy proxy, not motor energy. The requested 15 mm checkpoint was unreached and has no inferred recovery label. This stress case is outside the initial 0.25–3 degree randomized-plan range; it does not silently change that plan.

## Inspect the results

```bash
./view_study.sh outputs/Forge-Pilot-120
./view_study.sh outputs/Forge-Challenge-120
```

In Explore profiles, choose the recorded insertion, final withdrawal or checkpoint policy. Raw wrist and peg/socket contact signals are separate. The 20 N / 1 Nm operational budgets apply to raw wrist norms.

[Rate assessment](report.md) · [Centered baseline](../Forge-Baseline-120/report.md) · [Signed pilot](../Forge-Pilot-120/report.md) · [Known-load calibration](../Forge-Wrist-Calibration/report.md) · [Deeper checkpoint](../Forge-Deep-Recovery-120/report.md) · [Stress case](../Forge-Challenge-120/report.md)
