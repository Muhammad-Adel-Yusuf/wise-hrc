"""Procedure 1 - workload classification: low / medium / high physical intensity from camera-derived interval features.

Every feature is mapped to a load level g in [0, 1] by a piece-wise linear ramp between a "low" and a "high" value; the
intensity score is the weighted mean of the available g. The score is cut into L / M / H.

ALL RAMP VALUES, WEIGHTS AND CUTS ARE INITIAL ENGINEERING VALUES (posture limits follow the usual ergonomic ranges, e.g.
ISO 11226 / EN 1005-4 for arm elevation and trunk inclination). They must be checked against the pilot data
(evaluate / tune_cuts below). A missing feature (NaN) is left out and the weights are renormalised; if less than
`min_weight_coverage` of the weights is available the result is "UNKNOWN" - a value is never invented."""
from __future__ import annotations

import itertools
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

LABELS = ("L", "M", "H")
UNKNOWN = "UNKNOWN"

DEFAULT_WORKLOAD: Dict[str, Any] = {
    # feature name: (value giving g = 0, value giving g = 1, weight)
    "features": {
        "arm_elev_p90_deg":         (20.0, 60.0, 0.25),     # upper-arm elevation (90th percentile)
        "trunk_incl_p90_deg":       (10.0, 40.0, 0.15),     # trunk inclination
        "wrist_speed_mean_mps":     (0.05, 0.40, 0.10),     # wrist motion
        "elbow_speed_mean_dps":     (5.0, 60.0, 0.10),      # elbow motion
        "repetition_rate_per_min":  (10.0, 30.0, 0.15),     # repetition rate
        "static_posture_s":         (5.0, 30.0, 0.10),      # longest held non-neutral posture
        "reach_norm_p90":           (0.50, 0.90, 0.10),     # reaching distance / arm length
        "continuous_work_min":      (5.0, 20.0, 0.05),      # continuous work duration (time since the last rest)
    },
    "cuts": (1.0 / 3.0, 2.0 / 3.0),                         # score < c1: L, score < c2: M, else H
    "min_weight_coverage": 0.5,
}


def ramp(x: float, lo: float, hi: float) -> float:
    if hi <= lo:
        raise ValueError("ramp needs hi > lo")
    return float(min(1.0, max(0.0, (x - lo) / (hi - lo))))


def score(features: Dict[str, float], cfg: Optional[Dict[str, Any]] = None) -> Tuple[Optional[float], float, Dict[str, float]]:
    """(score or None, weight coverage, per-feature load levels)."""
    c = cfg or DEFAULT_WORKLOAD
    num = den = 0.0
    parts: Dict[str, float] = {}
    total_w = sum(w for _, _, w in c["features"].values())
    for name, (lo, hi, w) in c["features"].items():
        x = features.get(name)
        if x is None or not math.isfinite(x):
            continue
        g = ramp(float(x), lo, hi)
        parts[name] = g
        num += w * g
        den += w
    coverage = den / total_w if total_w > 0 else 0.0
    if den <= 0 or coverage < c["min_weight_coverage"]:
        return None, coverage, parts
    return num / den, coverage, parts


def classify(features: Dict[str, float], cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """{'label': 'L'|'M'|'H'|'UNKNOWN', 'score', 'coverage', 'parts'}"""
    c = cfg or DEFAULT_WORKLOAD
    sc, cov, parts = score(features, c)
    if sc is None:
        return {"label": UNKNOWN, "score": None, "coverage": cov, "parts": parts}
    c1, c2 = c["cuts"]
    return {"label": "L" if sc < c1 else ("M" if sc < c2 else "H"), "score": sc, "coverage": cov, "parts": parts}


def confusion(true: Sequence[str], pred: Sequence[str]) -> Dict[str, Any]:
    """3x3 confusion matrix (rows = designed intensity, columns = classified), accuracy and balanced accuracy.
    UNKNOWN predictions count as errors (and are reported)."""
    idx = {l: i for i, l in enumerate(LABELS)}
    m = np.zeros((3, 3), int)
    unknown = 0
    for t, p in zip(true, pred):
        if p not in idx:
            unknown += 1
            continue
        m[idx[t], idx[p]] += 1
    n = len(true)
    row = m.sum(axis=1) + np.array([sum(1 for t, p in zip(true, pred) if t == l and p not in idx) for l in LABELS])
    rec = [m[i, i] / row[i] if row[i] else float("nan") for i in range(3)]
    return {"matrix": m.tolist(), "labels": list(LABELS), "n": n, "unknown": unknown,
            "accuracy": float(np.trace(m) / n) if n else float("nan"),
            "balanced_accuracy": float(np.nanmean(rec)) if n else float("nan"), "recall": rec}


def evaluate(rows: Sequence[Dict[str, Any]], cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """rows: [{'intensity': 'L'|'M'|'H', <features>}, ...] (work intervals with the DESIGNED intensity)."""
    pred = [classify(r, cfg)["label"] for r in rows]
    return confusion([r["intensity"] for r in rows], pred)


def tune_cuts(rows: Sequence[Dict[str, Any]], cfg: Optional[Dict[str, Any]] = None, step: float = 0.02) -> Tuple[float, float]:
    """Grid search of the two cut points maximising the balanced accuracy on pilot intervals (designed intensity = reference)."""
    c = dict(cfg or DEFAULT_WORKLOAD)
    scores = [score(r, c)[0] for r in rows]
    keep = [(s, r["intensity"]) for s, r in zip(scores, rows) if s is not None]
    if len(keep) < 3:
        raise ValueError("not enough classified intervals to tune the cut points")
    grid = np.arange(step, 1.0, step)
    best, best_cuts = -1.0, c["cuts"]
    for c1, c2 in itertools.combinations(grid, 2):
        pred = ["L" if s < c1 else ("M" if s < c2 else "H") for s, _ in keep]
        ba = confusion([t for _, t in keep], pred)["balanced_accuracy"]
        if ba > best + 1e-12:
            best, best_cuts = ba, (float(c1), float(c2))
    return best_cuts


def leave_one_participant_out(rows: Sequence[Dict[str, Any]], cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Cut points tuned on the other participants, applied to the held-out one (rows need 'participant_id')."""
    c = dict(cfg or DEFAULT_WORKLOAD)
    true: List[str] = []
    pred: List[str] = []
    for pid in sorted({r["participant_id"] for r in rows}):
        train = [r for r in rows if r["participant_id"] != pid]
        test = [r for r in rows if r["participant_id"] == pid]
        cc = dict(c)
        cc["cuts"] = tune_cuts(train, c)
        for r in test:
            true.append(r["intensity"])
            pred.append(classify(r, cc)["label"])
    return confusion(true, pred)
