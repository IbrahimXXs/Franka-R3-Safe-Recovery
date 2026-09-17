# Postcollection command-reconstruction audit erratum

All 384 episodes were collected under the frozen plan. The original experiment,
source archive, detector configurations, raw trajectories, event logs and metrics
remain unchanged. No episode was rerun, removed or replaced.

The original offline audit passed 383 episodes and rejected
`held_out/ft023_oblique_offset_severe_v3/productivity`. Its first discrepancy was
at 11.1166666665 s during relaxation: the hand-position reconstruction differed
from the logged command by 0.357826 micrometres, exceeding the unchanged
0.3-micrometre reconstruction tolerance. The complete original failure and its
traceback are retained in `frozen_audit_diagnostic.json`.

The original offline audit uses SciPy's normalized, double-precision SLERP.
The existing controller uses Isaac Lab's float32 `quat_slerp`. For nearly
identical orientations, that runtime function returns the starting quaternion
when the absolute quaternion dot product is within four float32 epsilons of 1.
At the flagged sample the runtime dot product was exactly 1.0. SciPy instead
interpolated the tiny orientation difference. Replaying the existing runtime
math from the captured event transforms and independently reconstructed recovery
schedule matched the logged hand command within 0.349246 nanometres.

`runtime_math_diagnostic.json` records this comparison. The four quaternion
function bodies in `runtime_math_snapshot.py` are exact copies of the installed
Isaac Lab functions, including the original TorchScript decorators. Their ASTs
are checked against the installed source, whose SHA256 is recorded. This
calculation initializes PyTorch only; it does not run Isaac Sim.

`finish_with_runtime_audit.py` is a separate, explicitly postcollection audit
correction. It replaces only the expected recovery hand-transform calculation
in a private in-memory copy of the original auditor. It applies that correction
to every episode, not just the failing case. All detector checks, causal history
rules, safety checks, recovery schedules, achieved-motion verification, command
budgets, retry checks, assertions and numerical tolerances remain unchanged.
AST checks verify that every assertion and `assert_allclose` call is retained.
A 10-micrometre perturbed command is explicitly checked to fail the original
tolerance.

The original frozen analysis computes the tables, paired statistics and plots.
Its metric calculations, selection rule, shadow detectors, condition inclusion,
bootstrap settings and strict pre-intervention matching criteria are unchanged.
The resulting validation status distinguishes the runtime replay from the
original audit exception. The report and provenance audit disclose the
correction and retain hashes of these additional analysis artifacts.

Commands:

```bash
./force_productivity_dewedge.sh --headless
python -m research.force_productivity_report outputs/Force-vs-Productivity-Dewedge-v1
python outputs/Force-vs-Productivity-Dewedge-v1/validation/finish_with_runtime_audit.py
```

Collection exited with status 0 after saving all episodes; its in-process
report stopped at the audit exception. The standalone original analysis exposed
the traceback and exited with status 1. The final separate replay's exit status
is recorded with the execution logs. This erratum does not claim that the
original frozen auditor passed every episode.
