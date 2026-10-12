"""LFFR-type normalised physical-fatigue model: accumulation during work, exponential recovery, residual fatigue preserved.

Time unit of the model: MINUTES (lambda and mu are in 1 / min). Data files use seconds; physical_fatigue.schema converts.

  work      f(t)   = f_ini + (1 - f_ini) (1 - exp(-lambda_i t))        i in {L, M, H}
  recovery  f(t_r) = f_0 exp(-mu t_r)

Both are exact solutions of  df/dt = lambda (1 - f)  and  df/dt = -mu f, so the state can be advanced in any number of
steps with the same result. The state is never reset at the start of a task: the fatigue at the end of one interval is the
f_ini of the next. It is reset to 0 only at a defined full-recovery condition (f <= f_full at the end of a recovery
interval, or an interval explicitly flagged full_recovery)."""
from __future__ import annotations

import bisect
import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

INTENSITIES = ("L", "M", "H")
WORK, RECOVERY = "work", "recovery"
MIN_GAP = 1e-4                      # smallest allowed difference lambda_M - lambda_L, lambda_H - lambda_M


@dataclass(frozen=True)
class Params:
    lam_L: float
    lam_M: float
    lam_H: float
    mu: float

    def validate(self) -> "Params":
        if not (0.0 < self.lam_L < self.lam_M < self.lam_H):
            raise ValueError(f"need 0 < lambda_L < lambda_M < lambda_H, got {self.lam_L}, {self.lam_M}, {self.lam_H}")
        if not self.mu > 0.0:
            raise ValueError(f"need mu > 0, got {self.mu}")
        return self

    def lam(self, intensity: str) -> float:
        try:
            return {"L": self.lam_L, "M": self.lam_M, "H": self.lam_H}[intensity]
        except KeyError:
            raise ValueError(f"intensity must be one of {INTENSITIES}, got {intensity!r}") from None

    def as_dict(self) -> Dict[str, float]:
        return {"lambda_L": self.lam_L, "lambda_M": self.lam_M, "lambda_H": self.lam_H, "mu": self.mu}

    @classmethod
    def from_dict(cls, d: Dict[str, float]) -> "Params":
        return cls(float(d["lambda_L"]), float(d["lambda_M"]), float(d["lambda_H"]), float(d["mu"])).validate()

    @classmethod
    def load(cls, path: str) -> "Params":
        """Read parameters.json written by calibrate.py."""
        import json
        with open(path, encoding="utf-8") as fh:
            return cls.from_dict(json.load(fh))


DEFAULT_PARAMS = Params(0.014, 0.042, 0.084, 0.15)   # INITIAL values only (lambdas 0.7 x 0.02 / 0.06 / 0.12: slower build-up) (starting point of the calibration), not results


def clip01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def work_state(f_ini: float, lam: float, t_min: float) -> float:
    """f(t) = f_ini + (1 - f_ini)(1 - exp(-lambda t)); always in [f_ini, 1]."""
    return clip01(f_ini + (1.0 - f_ini) * (1.0 - math.exp(-lam * t_min)))


def recovery_state(f_0: float, mu: float, t_r_min: float) -> float:
    """f(t_r) = f_0 exp(-mu t_r); always in [0, f_0]."""
    return clip01(f_0 * math.exp(-mu * t_r_min))


@dataclass
class Segment:
    """One interval of a participant's timeline (absolute minutes from the start of the session)."""
    kind: str                              # "work" | "recovery"
    intensity: Optional[str]               # "L" | "M" | "H" for work, None for recovery
    start_min: float
    end_min: float
    full_recovery: bool = False            # the operator is fully rested at the end of this interval (long break): f := 0

    def __post_init__(self) -> None:
        if self.kind not in (WORK, RECOVERY):
            raise ValueError(f"segment kind must be 'work' or 'recovery', got {self.kind!r}")
        if self.kind == WORK and self.intensity not in INTENSITIES:
            raise ValueError(f"a work segment needs intensity in {INTENSITIES}, got {self.intensity!r}")
        if self.kind == RECOVERY and self.intensity not in (None, ""):
            raise ValueError("a recovery segment has no intensity")
        if not self.end_min > self.start_min:
            raise ValueError("segment must have a positive duration")

    @property
    def duration_min(self) -> float:
        return self.end_min - self.start_min


def _advance(params: Params, seg: Segment, f: float, t_in_min: float, f_full: float) -> float:
    if seg.kind == WORK:
        return work_state(f, params.lam(seg.intensity), t_in_min)
    out = recovery_state(f, params.mu, t_in_min)
    return 0.0 if out <= f_full else out


def segment_start_states(params: Params, segments: Sequence[Segment], f0: float = 0.0, f_full: float = 0.0) -> List[float]:
    """Fatigue at the start of every segment (residual fatigue is carried over; reset only on full recovery)."""
    states, f = [], clip01(f0)
    for seg in segments:
        states.append(f)
        f = _advance(params, seg, f, seg.duration_min, f_full)
        if seg.full_recovery:
            f = 0.0
    return states


def predict_at(params: Params, segments: Sequence[Segment], times_min: Sequence[float], f0: float = 0.0,
               f_full: float = 0.0) -> np.ndarray:
    """Model fatigue f_hat at the given absolute times (minutes). Times must lie inside the timeline."""
    if not segments:
        raise ValueError("empty timeline")
    starts = [s.start_min for s in segments]
    f_start = segment_start_states(params, segments, f0, f_full)
    out = np.empty(len(times_min), float)
    tol = 1e-9
    for k, t in enumerate(times_min):
        if t < segments[0].start_min - tol or t > segments[-1].end_min + tol:
            raise ValueError(f"time {t} min is outside the timeline [{segments[0].start_min}, {segments[-1].end_min}]")
        i = max(0, min(len(segments) - 1, bisect.bisect_right(starts, t + tol) - 1))
        seg = segments[i]
        out[k] = _advance(params, seg, f_start[i], min(max(t - seg.start_min, 0.0), seg.duration_min), f_full)
    return out


def simulate_curve(params: Params, segments: Sequence[Segment], dt_min: float = 0.1, f0: float = 0.0,
                   f_full: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """(t, f) on a regular grid, for plots and checks."""
    t = np.arange(segments[0].start_min, segments[-1].end_min + 1e-9, dt_min)
    return t, predict_at(params, segments, t, f0, f_full)
