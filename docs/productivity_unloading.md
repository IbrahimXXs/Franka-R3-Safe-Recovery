# Productivity-triggered verified unloading

Run the new 24-episode pilot (four previous Stage4 cases × two repeats × three policies):

```bash
./productivity_unloading.sh --headless
```

Default output: `outputs/Contact-Productivity-Unloading-v1`. Existing folders are rejected; for another run use `--output-dir outputs/Contact-Productivity-Unloading-v2`.

The arms are nominal, previous fixed-command productivity recovery, and verified-unloading productivity recovery. Nominal and fixed call the previous implementation directly. The existing calibration and detector are checked against the previous experiment and reused unchanged. Native FORGE physics remains 120 Hz; existing scene, gains, friction, geometry and hard safety limits are checked against the prior pilot.

Verified recovery uses measured peg depth:

1. At the original two-check productivity trigger, capture actual peg depth and hand pose. Stop downward insertion for the original 0.25 s.
2. Grow an upward-only hand command smoothly toward a maximum 5 mm over 4 s of active retraction. No lateral or angular correction is added.
3. When the measured peg depth has decreased by at least 0.5 mm from the trigger depth, freeze the last upward command and hold for 0.25 s.
4. If the peg rebounds below the measured target, resume the command ramp continuously, preserving the original depth anchor and deadline.
5. After a sustained verified hold, use the previous 0.5 s rejoin and retry from the measured depth. The original depth-dependent lateral/tilt demand is restored during rejoin; maximum-depth ramp memory is unchanged.
6. If verification cannot finish within 5 s after stopping, terminate the episode without retrying. The deadline includes the verification hold. The common 20 s insertion budget also remains binding.

At most two interventions. Hard gates take priority every observed physics tick: wrist force 20 N, torque 1 Nm, grasp slip 0.1 mm / 0.5 degrees, and penetration screen 0.126475 mm. A safety violation terminates the episode even if 0.5 mm retreat occurs on that tick. Inherited simulator reset/preparation is unchanged. There is no ML, detector retuning or contact-load feedback to the controller.

Outputs include `summary.csv`, `policy_summary.csv`, `unloading_events.csv`, `paired_comparisons.csv`, `report.md`, comparison plots, source/config snapshots and an automatic causal-log audit in `validation.json`. Raw physics-rate logs retain measured and commanded motion, all original wrench/velocity/slip/contact fields, detector decisions, and unloading states. The event CSV distinguishes an actual target crossing, sustained verified unloading, unsuccessful recovery and a retry with sufficient insertion exposure to evaluate stalls.

Contact load is privileged analysis only. Its decrease is reported separately from achieved 0.5 mm motion; motion alone does not prove that contact loading reduced. Before/after means use 0.1 s windows at the trigger and recovery endpoint, with sample counts and endpoint safety status. Empty stall-after-retry labels are censored or untested, not successful recoveries.

The old matching protocol is retained: common seed, randomized policy order, strict full-state prefix matching and all mismatches retained. Repeats measure numerical reproducibility rather than independent geometries. A smaller stall count caused by early recovery failure or a safety termination is not evidence of better insertion.

Offline report regeneration:

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python \
  -m research.productivity_unloading_report outputs/Contact-Productivity-Unloading-v1
```

Tests (no Isaac Sim launch):

```bash
/home/ibrahim/miniconda3/envs/franka-safe-recovery/bin/python \
  -m unittest tests.test_productivity_unloading -v
```
