"""Data-quality tracking and level constants.

The old single MFR scorer was replaced by three independent algorithms (task_fatigue.py, visual_fatigue.py,
attention_fatigue.py) and the combiner fatigue_fusion.py. This module keeps only what they share.
"""
from __future__ import annotations

from collections import deque
from typing import Deque, Optional, Tuple

from .fatigue_common import LEVEL_LABELS, microsleep_score, piecewise_linear  # noqa: F401  (re-exported)

LEVELS = ("Low", "Medium", "High")
# Numeric output: Low=0, Medium=1, High=2 (-1 only if fatigue_fusion.report_unknown_as_minus_one is enabled).
LEVEL_CODES = {"Low": 0, "Medium": 1, "High": 2, "Unknown": -1}


def level_code(level: str) -> int:
    """0 = Low, 1 = Medium, 2 = High, -1 = Unknown."""
    return LEVEL_CODES[level]


class QualityTracker:
    """Q(t) = valid observed time / total time in a rolling window.

    Camera gaps longer than max_gap_s count as INVALID time.
    """

    def __init__(self, window_s: float, max_gap_s: float) -> None:
        self.window_s = window_s
        self.max_gap_s = max_gap_s
        self._buf: Deque[Tuple[float, float, bool]] = deque()   # (t, dt, valid)
        self._last_t: Optional[float] = None
        self._first_t: Optional[float] = None

    def add(self, t: float, valid: bool) -> None:
        if self._first_t is None:
            self._first_t = t
        dt = 0.0 if self._last_t is None else max(0.0, t - self._last_t)
        self._last_t = t
        if dt > self.max_gap_s:
            self._buf.append((t, dt, False))     # frames were missing for this interval
        else:
            self._buf.append((t, dt, valid))
        while self._buf and self._buf[0][0] < t - self.window_s:
            self._buf.popleft()

    def observed_s(self) -> float:
        return sum(dt for _, dt, _ in self._buf)

    def value(self) -> float:
        total = self.observed_s()
        if total <= 0:
            return 0.0
        return sum(dt for _, dt, v in self._buf if v) / total
