# Offline Contact Productivity analysis

Run from the repository using the existing environment (no Isaac Sim launch):

```bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
conda run -n franka-safe-recovery python -m research.contact_productivity \
  --outputs-root outputs --output outputs/Contact-Productivity-v1
```

An existing output directory is refused. Choose a fresh name to repeat the analysis. The script reads only `insertion.csv` and provenance from the five explicit Phase 2A / Phase 2B Stage 1–4 collections. Recovery, replay, active-probing and legacy FR3 logs are excluded.

The target is the **signed, unclipped** actual depth change divided by commanded depth change over 0.25, 0.5 or 1.0 seconds. The full future interval must remain within insertion, with at least 0.1 mm positive commanded progress. The separately saved `[0,1]` diagnostic never enters training or primary metrics. Features use only the trailing 0.5 seconds, sampled every 0.1 seconds.

The primary comparison uses wrist-wrench histories and actual/commanded motion. Logged true peg-contact wrench is an explicitly separate simulator-signal comparison. `normal_load_n` is reserved for privileged hidden-load analysis and never enters a model. Feature names and exact sensor mappings are in `feature_sets.json` and the report.

Stage 4 shares 13 paths with earlier stages. Its entire path groups are excluded from development fitting and tuning: 118 trajectories remain for development, 39 earlier-stage trajectories are quarantined, and 13 form the stress test. All Stage 3 paths fall in that quarantine. They are still analyzed and predicted separately. Pre-ramp common histories are removed; centered duplicates share one group. Numerically valid slipping trajectories remain included, with their archived grasp-retention flag recorded.

Group-disjoint nested validation chooses ridge regularization using development data only. Final transforms, coefficients and alarm thresholds are frozen before Stage 4 evaluation. There is no window-wise random split. Five primary feature sets, persistence, two contact-wrench comparisons, and simple force thresholds are evaluated.

Stall timing exactly follows the existing trailing-window predicate. Prediction and early-warning scoring stop before its first confirmation. Alerts require two adjacent predictions and are also checked against the beginning of the 0.5-second confirmation lookback; that boundary is not asserted to be physical stall onset. Trajectories with no eligible prediction windows cannot silently disappear from warning denominators.

Outputs include:

- `report.md`, `trajectory_summary.csv`, `window_features.csv`, `predictions.csv`.
- `metrics.csv`, `stage4_results.csv`, `pre_lookback_metrics.csv`.
- `warning_events.csv`, `warning_summary.csv`, `paired_comparisons.csv`.
- `hidden_load_results.csv`, `hidden_states.csv`, `hidden_state_summary.csv`.
- Eta, pre-stall forecasts, force/productivity and hidden-load plots.
- `models.json`, `split_audit.json`, `settings.json`, source hashes and preservation audit.

The primary hidden-load definition is mean normal load ≥10 N with wrist force ≤5 N. Instantaneous 10 N and mean 5 N subsets are labeled descriptive sensitivity checks. High-load states outside moving insertion retain undefined productivity; neither zero denominators nor after-stall measurements become early-warning evidence.

This implements offline forecasting only. It neither runs an online observer nor changes insertion commands or existing experiment outputs.
