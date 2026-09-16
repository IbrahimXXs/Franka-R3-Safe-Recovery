"""Pure-Python reference for belief aggregation and a finite candidate filter.

Inputs are supplied predictions, NOT a learned or certified safety model.
The synthetic demo is a mathematical example, not simulated robot results.
"""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class Candidate:
    name: str
    progress_score: float
    path_success_by_particle: tuple[float, ...]
    recovery_success: dict[str, tuple[float, ...]]


def probability(values, weights):
    if not values or len(values) != len(weights):
        raise ValueError("Nonempty values and weights must have equal lengths")
    if any(not math.isfinite(w) or w < 0 for w in weights) or sum(weights) <= 0:
        raise ValueError("Weights must be finite, nonnegative, and have positive sum")
    if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
        raise ValueError("Predicted probabilities must be finite and in [0, 1]")
    return sum(v * w for v, w in zip(values, weights)) / sum(weights)


def best_common_recovery(recovery_success, weights):
    """Maximize AFTER belief averaging: one feedback policy across all particles."""
    if not recovery_success:
        return None, 0.0
    scores = [(key, probability(values, weights)) for key, values in recovery_success.items()]
    return max(scores, key=lambda item: item[1])


def select_candidate(candidates, weights, *, path_min=0.95, recovery_min=0.95):
    """Return selected prediction record or None (supervisor must handle recovery).

    Thresholds are independent research settings, not a total episode-risk bound.
    No candidate means no proposed progress action; it never means a safe hold.
    """
    if not 0 <= path_min <= 1 or not 0 <= recovery_min <= 1:
        raise ValueError("Thresholds must be in [0, 1]")
    accepted = []
    for candidate in candidates:
        if not math.isfinite(candidate.progress_score):
            raise ValueError("Score must be finite")
        path_p = probability(candidate.path_success_by_particle, weights)
        backup, recovery_p = best_common_recovery(candidate.recovery_success, weights)
        if backup is not None and path_p >= path_min and recovery_p >= recovery_min:
            accepted.append({"candidate": candidate.name, "backup": backup,
                             "score": candidate.progress_score,
                             "predicted_path_success": path_p,
                             "predicted_recovery_success": recovery_p})
    return max(accepted, key=lambda item: item["score"]) if accepted else None


if __name__ == "__main__":
    options = [
        Candidate("deep_advance", 2.0, (0.99, 0.99), {"retreat": (0.98, 0.30)}),
        Candidate("shallow_align", 1.0, (0.99, 0.99), {"unload_retreat": (0.99, 0.98)}),
    ]
    print("SYNTHETIC EXAMPLE ONLY")
    print(select_candidate(options, (0.5, 0.5)))
