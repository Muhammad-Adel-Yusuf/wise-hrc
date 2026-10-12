"""ALGORITHM 3 - working-area attention and sustained-disengagement estimator (standalone).

Reports mental-fatigue evidence only. It does NOT control the robot and does not talk to any decision layer.

Working area = laptop + assembly + storage + the calibrated angular region between them.
Per-frame state: LAPTOP | ASSEMBLY | STORAGE | WORKING_AREA | OUTSIDE_WORKING_AREA | UNKNOWN.
Missing landmarks / invalid pose / occlusion = UNKNOWN (never disengagement by itself).

One continuous stay outside the working area is classified exactly once:
  * shorter than sustained_threshold_s and the worker returned -> BRIEF disengagement (counted on return);
  * at least sustained_threshold_s                              -> SUSTAINED disengagement (never also brief).

INITIAL ENGINEERING PARAMETERS (config.json -> attention_fatigue), not validated experimentally.
"""
from __future__ import annotations

import math
from typing import Any, Dict, Optional

from .attention import LABEL_AREA, LABEL_OUTSIDE, LABEL_UNKNOWN, AttentionClassifier
from .events import DistractionTracker
from .fatigue_common import clip01, integrate_state, score_to_level, step_dt

LAPTOP, ASSEMBLY, STORAGE = "LAPTOP", "ASSEMBLY", "STORAGE"
WORKING_AREA, OUTSIDE_WORKING_AREA, UNKNOWN = "WORKING_AREA", "OUTSIDE_WORKING_AREA", "UNKNOWN"

STATE_FOCUSED = "FOCUSED"
STATE_BRIEF = "OUTSIDE_BRIEF"                 # currently outside for less than sustained_threshold_s
STATE_SUSTAINED = "SUSTAINED_DISENGAGEMENT"   # currently outside for at least sustained_threshold_s
STATE_UNKNOWN = "UNKNOWN"
TASK_ENGAGED = "TASK_ENGAGED"                 # face hidden, body/hands in the calibrated assembly posture; gaze UNKNOWN
STATE_ENGAGED = "TASK_ENGAGED"
STATE_UNVERIFIED = "ATTENTION_UNVERIFIED"     # collaboration active, face not seen, no engagement evidence for a while
STATE_STANDBY = "STANDBY"                     # collaboration inactive: nothing is measured
LABEL_TASK_ENGAGED = "Task engaged (face hidden)"

_TO_PUBLIC = {LABEL_TASK_ENGAGED: TASK_ENGAGED, "Laptop": LAPTOP, "Assembly": ASSEMBLY, "Storage": STORAGE,
              LABEL_AREA: WORKING_AREA, LABEL_OUTSIDE: OUTSIDE_WORKING_AREA, LABEL_UNKNOWN: UNKNOWN}
_TO_TRACKER = {LAPTOP: "Laptop", ASSEMBLY: "Assembly", STORAGE: "Storage", WORKING_AREA: LABEL_AREA,
               OUTSIDE_WORKING_AREA: LABEL_OUTSIDE, UNKNOWN: LABEL_UNKNOWN,
               TASK_ENGAGED: LABEL_UNKNOWN}           # engaged = gaze unknown: never ends an outside event as a "return"


def normalize_label(label: Optional[str]) -> str:
    """Accept classifier labels ("Laptop", "Outside working area", ...) or the public names; return the public name."""
    if label is None:
        return UNKNOWN
    if label in _TO_TRACKER:
        return label
    return _TO_PUBLIC.get(label, UNKNOWN)


def sustained_score(outside_s: float, cfg: Dict[str, Any]) -> float:
    """Score of the CURRENT continuous stay outside the working area (0..1).

    d < T2 (sustained_threshold_s)  -> 0
    d >= T3 (high_threshold_s)      -> 1 (capped)
    T2 <= d < T3, p = (d - T2) / (T3 - T2):
        linear (default, sustained_growth = 0):  0.5 + 0.5 * p          (= specification for T3 - T2 = 1 s)
        exponential (sustained_growth = g > 0):  0.5 + 0.5 * (1 - exp(-g p)) / (1 - exp(-g))
    => 0.5 at 2 s (Medium-level evidence), 1.0 at 3 s (High-level evidence).
    """
    t2, t3, g = cfg["sustained_threshold_s"], cfg["high_threshold_s"], cfg["sustained_growth"]
    if outside_s < t2:
        return 0.0
    if outside_s >= t3:
        return 1.0
    p = (outside_s - t2) / (t3 - t2)
    if g > 0:
        return 0.5 + 0.5 * (1.0 - math.exp(-g * p)) / (1.0 - math.exp(-g))
    return 0.5 + 0.5 * p


class AttentionFatigueEstimator:
    """Feed one label (or yaw/pitch) per frame; read the output dict."""

    def __init__(self, cfg: Dict[str, Any], classifier: Optional[AttentionClassifier] = None) -> None:
        self.cfg = cfg                                  # the "attention_fatigue" section
        self.classifier = classifier
        self.reset()

    def reset(self) -> None:
        self.tracker = DistractionTracker(self.cfg)
        self.state = 0.0                                # cumulative (integrated) attention score
        self._last_t: Optional[float] = None
        self._label = UNKNOWN
        self._instant = 0.0
        self._standby = False
        self._unverified_since: Optional[float] = None
        self._unverified = False
        self._out = self._output(0.0)

    # -- input -----------------------------------------------------------------
    def update_pose(self, t: float, yaw: Optional[float], pitch: Optional[float], valid: bool = True) -> Dict[str, Any]:
        """Classify yaw/pitch with the calibrated classifier, then update. valid=False -> UNKNOWN."""
        if self.classifier is None:
            raise RuntimeError("update_pose needs an AttentionClassifier; use update_label instead")
        label = self.classifier.classify(yaw if valid else None, pitch if valid else None).label
        return self.update_label(t, label)

    def suspend(self) -> None:
        """Collaboration inactive (worker left): nothing is measured, the cumulative state is kept."""
        self._standby = True
        self._instant = 0.0
        self._unverified_since, self._unverified = None, False
        self._out = self._output(self._last_t or 0.0)

    def resume(self, t: float) -> None:
        """Collaboration active again: the look-away timer starts from zero; counts and cumulative state are kept."""
        self._standby = False
        self.tracker.clear_current()
        self._last_t = None
        self._label = UNKNOWN
        self._unverified_since, self._unverified = None, False

    def update_label(self, t: float, label: Optional[str], since: Optional[float] = None,
                     unverified_ok: bool = False) -> Dict[str, Any]:
        """label: classifier label or public name. since: optional back-dated start of an outside episode.

        unverified_ok: the caller says 'collaboration is active, the worker should be here, but the face was not
        seen and there is no task-engagement evidence'. After unverified_after_s of that the score is floored at
        unverified_score (Medium-level evidence, always below High)."""
        public = normalize_label(label)
        self._label = public
        self.tracker.update(t, _TO_TRACKER[public], since=since)
        dt = step_dt(self._last_t, t)
        self._last_t = t

        c = self.cfg
        d = self.tracker.d_outside(t)
        sus = sustained_score(d, c)
        brief_n = self.tracker.brief_count(t)
        brief = clip01(brief_n / c["brief_full_count"])
        # sustained and (moderate) brief evidence combine like independent evidence: 1 - (1-a)(1-b).
        # Brief alone can never exceed brief_weight, so it stays "moderate".
        self._instant = 1.0 - (1.0 - sus) * (1.0 - c["brief_weight"] * brief)
        c = self.cfg
        if unverified_ok and public == UNKNOWN and c.get("unverified_enabled", True):
            if self._unverified_since is None:
                self._unverified_since = t
            self._unverified = t - self._unverified_since >= c["unverified_after_s"]
        else:
            self._unverified_since, self._unverified = None, False
        if public not in (UNKNOWN, TASK_ENGAGED):        # unknown / gaze-unknown frames: neither accumulate nor recover
            rec = 1.0 if self._instant < c["recovery_score_threshold"] else 0.0
            self.state = integrate_state(self.state, c["accumulation_rate"], self._instant,
                                         c["recovery_rate"], rec, dt)
        self._sus, self._brief_score, self._d = sus, brief, d
        self._out = self._output(t)
        return self._out

    # -- output ------------------------------------------------------------------
    def _state_name(self, d: float) -> str:
        if d >= self.cfg["sustained_threshold_s"]:
            return STATE_SUSTAINED
        if self._standby:
            return STATE_STANDBY
        if self._label == TASK_ENGAGED:
            return STATE_ENGAGED
        if self._label == UNKNOWN:
            return STATE_UNVERIFIED if self._unverified else STATE_UNKNOWN
        return STATE_BRIEF if d > 0 else STATE_FOCUSED

    def _output(self, t: float) -> Dict[str, Any]:
        c = self.cfg
        d = self.tracker.d_outside(t) if self._last_t is not None else 0.0
        # Reported score = max(instantaneous, cumulative): the 2 s / 3 s rules must show at once, while the
        # cumulative state keeps repeated / long disengagement visible after the events end.
        score = max(self._instant, self.state)
        if self._unverified and not self._standby:
            score = max(score, c["unverified_score"])        # Medium-level evidence; never above the High threshold
        return {
            "attention_score": float(score),
            "attention_fatigue_level": score_to_level(score, c["medium_threshold"], c["high_threshold"]),
            "brief_disengagement_count": int(self.tracker.brief_total),
            "sustained_disengagement_count": int(self.tracker.sustained_total),
            "maximum_outside_duration_s": float(self.tracker.max_outside_s(t)) if self._last_t is not None else 0.0,
            "attention_state": self._state_name(d),
        }

    def output(self) -> Dict[str, Any]:
        return dict(self._out)

    def diagnostics(self) -> Dict[str, Any]:
        t = self._last_t if self._last_t is not None else 0.0
        return {"label": self._label, "instant_score": self._instant, "cumulative_state": self.state,
                "current_outside_s": self.tracker.d_outside(t), "sustained_score": getattr(self, "_sus", 0.0),
                "brief_score": getattr(self, "_brief_score", 0.0),
                "brief_events_in_window": self.tracker.brief_count(t)}
