"""Small helpers shared by the three fatigue algorithms and the combiner.

INITIAL ENGINEERING PARAMETERS: every threshold / rate / weight used with these helpers lives in
config.json and requires experimental validation.
"""
from __future__ import annotations

import math
from typing import Sequence

LEVEL_LABELS = {0: "Low", 1: "Medium", 2: "High"}
MAX_DT_S = 0.5            # integration steps are capped so a camera gap cannot cause a jump


def clip01(x: float) -> float:
    return 0.0 if x <= 0.0 else (1.0 if x >= 1.0 else float(x))


def score_to_level(score: float, medium_threshold: float, high_threshold: float) -> int:
    """0 = Low, 1 = Medium, 2 = High. Thresholds come from the configuration (never hard-coded)."""
    if score < medium_threshold:
        return 0
    if score < high_threshold:
        return 1
    return 2


def integrate_state(state: float, accumulation_rate: float, score: float,
                    recovery_rate: float, recovery_signal: float, dt: float) -> float:
    """state = clip(state + accumulation_rate * score * dt - recovery_rate * recovery_signal * dt, 0, 1)."""
    return clip01(state + accumulation_rate * score * dt - recovery_rate * recovery_signal * dt)


def piecewise_linear(x: float, knots: Sequence[Sequence[float]]) -> float:
    """Linear interpolation through knots [[x0, r0], ...], clamped to the ends."""
    if x <= knots[0][0]:
        return float(knots[0][1])
    for (x0, r0), (x1, r1) in zip(knots, knots[1:]):
        if x <= x1:
            return float(r0 + (r1 - r0) * (x - x0) / (x1 - x0))
    return float(knots[-1][1])


def microsleep_score(count: float, growth: float) -> float:
    """1 - exp(-growth * count), bounded to [0, 1]."""
    if count <= 0:
        return 0.0
    return min(1.0, max(0.0, 1.0 - math.exp(-growth * count)))


def step_dt(last_t, t: float) -> float:
    """Time step since the previous update, capped to [0, MAX_DT_S]."""
    if last_t is None:
        return 0.0
    return min(MAX_DT_S, max(0.0, t - last_t))
