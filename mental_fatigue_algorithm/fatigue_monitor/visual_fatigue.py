"""ALGORITHM 2 - visual fatigue and alertness estimator (standalone).

Inputs are the counts produced by the existing detectors (eye closures, yawns, nods, eye rubs, PERCLOS);
it does not depend on the task-difficulty or the attention estimator.

    microsleep_score = 1 - exp(-microsleep_growth * microsleep_count)           (bounded 0..1)
    closure term     = max(microsleep_score, normalised prolonged-closure count)  (prolonged closures preserved)
    visual_score     = w_microsleep*closure_term + w_yawn*yawn_score + w_nod*nod_score
                       + w_eye_rub*eye_rub_score + w_perclos*perclos_score       (weights renormalised to sum 1)
    smoothing        : exponential smoothing with time constant smoothing_tau_s
    visual_state     = clip(visual_state + acc * smoothed_score * dt - rec * recovery_signal * dt, 0, 1)

The reported visual_score is the cumulative state, so one isolated event cannot by itself produce High
(unless single_microsleep_forces_high is explicitly enabled). Persistence / hysteresis of the FINAL level is
done by the combiner; the integration above provides the recovery (state decays while there is no evidence).

INITIAL ENGINEERING PARAMETERS (config.json -> visual_fatigue), not validated experimentally.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .fatigue_common import (clip01, integrate_state, microsleep_score, piecewise_linear, score_to_level, step_dt)


class VisualFatigueEstimator:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg                                    # the "visual_fatigue" section
        self.reset()

    def reset(self) -> None:
        self.state = 0.0
        self._smoothed: Optional[float] = None
        self._last_t: Optional[float] = None
        self._diag: Dict[str, Any] = {}
        self._out = {"visual_score": 0.0, "visual_fatigue_level": 0, "microsleep_count": 0,
                     "yawn_count": 0, "nod_count": 0, "eye_rub_count": 0}

    def update(self, t: float, *, microsleep_count: float = 0, prolonged_closure_weighted: float = 0.0,
               yawn_count: float = 0.0, nod_count: float = 0.0, eye_rub_count: float = 0.0,
               perclos: Optional[float] = None, valid: bool = True) -> Dict[str, Any]:
        """valid=False (e.g. eye baseline not ready): output is held and nothing is integrated."""
        c = self.cfg
        kn = c["normalization"]
        ms = microsleep_score(microsleep_count, c["microsleep_growth"])
        prolonged = piecewise_linear(prolonged_closure_weighted, kn["closure"])
        scores = {
            "microsleep": ms,
            "closure": max(ms, prolonged),                # carried by the "microsleep" weight
            "yawn": piecewise_linear(yawn_count, kn["yawn"]),
            "nod": piecewise_linear(nod_count, kn["nod"]),
            "eye_rub": piecewise_linear(eye_rub_count, kn["rub"]),
            "perclos": piecewise_linear(perclos, kn["perclos"]) if perclos is not None else 0.0,
        }
        w = c["weights"]
        wsum = sum(w.values()) or 1.0
        instant = clip01((w["microsleep"] * scores["closure"] + w["yawn"] * scores["yawn"] + w["nod"] * scores["nod"]
                          + w["eye_rub"] * scores["eye_rub"] + w["perclos"] * scores["perclos"]) / wsum)

        dt = step_dt(self._last_t, t)
        self._last_t = t
        if valid:
            a = math.exp(-dt / c["smoothing_tau_s"]) if (dt > 0 and c["smoothing_tau_s"] > 0) else 0.0
            self._smoothed = instant if self._smoothed is None else a * self._smoothed + (1.0 - a) * instant
            rec = 1.0 if self._smoothed < c["recovery_score_threshold"] else 0.0
            self.state = integrate_state(self.state, c["accumulation_rate"], self._smoothed,
                                         c["recovery_rate"], rec, dt)
        score = self.state
        level = score_to_level(score, c["medium_threshold"], c["high_threshold"])
        forced = bool(c["single_microsleep_forces_high"] and microsleep_count >= 1 and ms >= c["microsleep_high_score"])
        if forced:                                        # explicit, documented exception (off by default)
            level, score = 2, max(score, c["high_threshold"])
        self._diag = {"instant_score": instant, "smoothed_score": self._smoothed, "cumulative_state": self.state,
                      "scores": scores, "forced_high": forced, "valid": valid}
        self._out = {"visual_score": float(score), "visual_fatigue_level": int(level),
                     "microsleep_count": int(round(microsleep_count)), "yawn_count": int(round(yawn_count)),
                     "nod_count": int(round(nod_count)), "eye_rub_count": int(round(eye_rub_count))}
        return dict(self._out)

    def output(self) -> Dict[str, Any]:
        return dict(self._out)

    def diagnostics(self) -> Dict[str, Any]:
        return dict(self._diag)
