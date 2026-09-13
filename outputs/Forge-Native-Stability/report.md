# Native FORGE rate check

Working physics rate: **120 Hz**, inherited from the installed scene. The earlier 240/480 Hz runs were explicit comparisons.

Three-case same-rate repeatability check passed: **True**.

| Case | Tilt | Passed | Failed checks |
|---|---:|---|---|
| y_offset | 0.000° | True |  |
| tilt_only | 1.857° | True |  |
| tilt_only | 5.000° | True |  |

All compared numerical outcome metrics matched exactly in these three repetitions (zero difference, including final recovery work).

The comparisons check force/torque peaks, reported overlap, depth, grasp retention, insertion/stall/budget labels, final recovery outcome and contact work. Thresholds are retained in validation.json.

## Loaded checkpoint

At requested depth 13 mm: reached=True, Y_R_tested=1.
- straight: cleared; replay matched=True; safe within configured limits=True.
- realign: replay_mismatch; replay matched=False; safe within configured limits=None.

Repeatability and operational screening for three seeded cases at 120 Hz. Not a guarantee for every randomized trajectory or a cross-timestep convergence certificate. Unknown recovery probes remain unknown.

Normal launch (no rate override):

```bash
./forge_study.sh --headless
```

[Contact repetitions](../Forge-Native-Repeat/report.md) · [Loaded recovery](../Forge-Native-Loaded-Recovery/report.md) · [Earlier rate comparison](../Forge-Validation/report.md)
