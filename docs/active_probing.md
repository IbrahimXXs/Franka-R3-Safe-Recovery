# Active micro-probing pilot

Run from this repository in the existing `franka-safe-recovery` Miniconda environment:

```bash
./active_probing.sh --headless
```

The default output is `outputs/Active-Probing-Pilot-v1`. The runner refuses to overwrite an existing directory. To repeat the pilot, choose a fresh directory:

```bash
./active_probing.sh --headless --output-dir outputs/Active-Probing-Pilot-v2
```

This runs **36 scheduled attempts**: two selected Phase 2B terminal states × two repeat blocks × nine conditions (hold, ±X, ±Y, ±roll, ±pitch). There are also two local insertion references. Each attempt independently resets and replays insertion, preserving contact history as closely as the existing replay checks can verify. There are no automatic replacement attempts. A failed preparation is logged and excluded, so 36 attempts do not guarantee 36 eligible responses. `--repeats 3` requests 54 attempts; larger collections are deliberately outside this launcher.

`experiments/active_probing_pilot.json` selects archived successful/contact-rich and stalled/high-contact cases. Their insertion outcomes are re-evaluated locally. Existing scene assets, friction, grasp, controller, safety thresholds and experiment data are reused without changes. Physics uses the installed FORGE default (120 Hz in this environment); DLSS Performance and Eco mode remain enabled.

After insertion, the runner freezes the final insertion **target**, including its preload, for 0.75 s. It checks both the insertion endpoint and paused state against a local reference. An accepted attempt observes 0.25 s before the probe, ramps over 0.25 s, dwells for 0.25 s, ramps back over 0.25 s, then observes the return for 0.5 s. Holding a target does not guarantee a stationary peg: actual pose and velocity remain part of the measurements.

Probe amplitudes are ±0.1 mm in world X/Y and ±0.2° in spatial roll/pitch about the nominal peg base. The full wrench is recorded at every physics step, alongside world pose, depth, COM and peg-base velocities, contact load, penetration, grasp slip and safety flags. Raw wrist and contact wrenches have separate columns. Limit violations terminate that attempt; no corrective action follows. Before the next attempt, the entire insertion is replayed from reset.

Outputs:

- `pilot.json`: frozen schedule, protocol, source hashes, local outcomes, replay diagnostics, individual completion/exclusion reasons and preservation audit.
- `<case>/insertion.csv`, `nominal_hold.csv`: local reference and pause.
- `<case>/<probe>/prefix.csv`, `preparation.csv`, `probe.csv`: replay, pause, full before/during/after trace (truncated on rejection or safety stop).
- `summary.csv`: actual motion, wrench differences, baseline ratios, safety and return errors.
- `central_differences.csv`: actual signed-span response secants and excitation/coupling eligibility.
- `repeatability.csv`, `distinguishability.csv`, `analysis.json`: explicit descriptive criteria.
- `report.md` and PNG plots: findings and interpretation limits.
- `source/`: copies of the executed pilot and reused simulation/protocol code.

Rebuild the analysis without launching Isaac Sim:

```bash
conda run -n franka-safe-recovery python -m research.active_probing outputs/Active-Probing-Pilot-v1
```

Analysis transports contact torque to one fixed anchor per case and compares mean plateau wrench with the preceding dwell. Central differences use actual signed pose changes, rejecting missing opposite signs and near-zero motion. Opposite-sign trials must also match each other at the end of their before dwell using the existing state tolerances. Excessive off-axis motion prevents interpreting a coupled secant as a directional partial derivative. The combined force/torque metric explicitly uses a 10 mm length scale; raw SI components remain available.

The no-motion baseline includes both drift and within-dwell variability. At least two valid holds are needed. Exploratory criteria require responses ≥3× baseline, repeat cosine ≥0.8 and relative repeat deviation ≤0.5. Pairwise positive-axis responses must also separate beyond repeat spread plus 3× baseline. These are analysis criteria, not changes to simulation or safety thresholds, and two repeats cannot establish statistical confidence or controller safety.

Returning a command is not proof of restoring contact memory. Full return-state matching and actual pose/wrench residuals are reported, while independent preparation protects subsequent attempts from accumulated hysteresis. This pilot implements identification only: no active recovery policy, corrective insertion controller or ML.
