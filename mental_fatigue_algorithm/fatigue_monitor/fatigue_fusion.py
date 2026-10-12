"""FOURTH FILE - final mental-fatigue combiner.

Imports nothing from the three algorithms' internals: it only consumes their OUTPUT dicts
(task_fatigue.TaskFatigueEstimator, visual_fatigue.VisualFatigueEstimator,
attention_fatigue.AttentionFatigueEstimator) and reports ONLY the final mental-fatigue result:

    Low = 0, Medium = 1, High = 2                     (integers; no robot / MDP integration)

Rules (INITIAL ENGINEERING PARAMETERS, config.json -> fatigue_fusion, not validated experimentally):

    combined_score = w_task*task_score + w_visual*visual_score + w_attention*attention_score   (weights renormalised)
    candidate      = 2 if any component level is High
                     else 1 if combined_score >= medium_threshold (or any component is Medium, use_max_rule)
                     else 0
    The levels are NOT averaged: a severe component cannot be masked by two low ones.

Persistence / hysteresis (the final level never follows one noisy frame):
    Medium activation : candidate >= 1 must persist medium_persistence_s
    High activation   : candidate == 2 must persist high_persistence_s
    Recovery          : the level drops only after the candidate stayed lower for recovery_persistence_s
                        (dropping to Low also needs combined_score < recovery_threshold)
Documented exception (attention_sustained_bypasses_persistence, default True): while the attention estimator
reports SUSTAINED_DISENGAGEMENT, its level applies at once - the 2 s / 3 s durations are already the
persistence of that evidence. Set it to false to apply medium/high persistence to it as well.
"""
from __future__ import annotations

from typing import Any, Dict, List, Optional, Sequence

from .fatigue_common import LEVEL_LABELS, clip01

SUSTAINED_STATE = "SUSTAINED_DISENGAGEMENT"      # attention_state value that triggers the bypass


class FatigueFusion:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg                                    # the "fatigue_fusion" section
        self.reset()

    def reset(self) -> None:
        self.level = 0
        self._since: Dict[int, Optional[float]] = {1: None, 2: None}
        self._below_since: Optional[float] = None
        self._below_max = 0
        self._diag: Dict[str, Any] = {}
        self._out = self._result(0.0, 0, 0, 0, 0.0, [])

    def _result(self, t: float, tl: int, vl: int, al: int, combined: float, evidence: List[str],
                level: Optional[int] = None) -> Dict[str, Any]:
        lvl = self.level if level is None else level
        return {"final_mental_fatigue_level": int(lvl),
                "final_mental_fatigue_label": LEVEL_LABELS.get(lvl, "Unknown"),
                "combined_score": float(combined), "task_level": int(tl), "visual_level": int(vl),
                "attention_level": int(al), "active_evidence": list(evidence), "timestamp": float(t)}

    def update(self, t: float, task: Dict[str, Any], visual: Dict[str, Any], attention: Dict[str, Any],
               data_valid: bool = True, warnings: Sequence[str] = (),
               enabled: Optional[Dict[str, bool]] = None) -> Dict[str, Any]:
        """task / visual / attention: the output dicts of the three algorithms.
        data_valid=False (baseline learning, calibration incomplete, low data quality): timers and recovery are
        paused and the level is held (sustained-disengagement evidence still applies).
        enabled: {"task": bool, "visual": bool, "attention": bool}; a disabled algorithm is ignored completely
        (weight removed and renormalised, level forced to 0, no persistence bypass)."""
        c = self.cfg
        en = {"task": True, "visual": True, "attention": True}
        if enabled:
            en.update({k: bool(v) for k, v in enabled.items() if k in en})
        w = {k: (c["weights"][k] if en[k] else 0.0) for k in ("task", "visual", "attention")}
        tl, vl, al = (int(task["task_fatigue_level"]) if en["task"] else 0,
                      int(visual["visual_fatigue_level"]) if en["visual"] else 0,
                      int(attention["attention_fatigue_level"]) if en["attention"] else 0)
        wsum = (w["task"] + w["visual"] + w["attention"]) or 1.0
        combined = clip01((w["task"] * float(task["task_score"]) + w["visual"] * float(visual["visual_score"])
                           + w["attention"] * float(attention["attention_score"])) / wsum)
        if not any(en.values()):
            combined = 0.0

        any_high = 2 in (tl, vl, al)
        if any_high:
            rule = 2
        elif combined >= c["medium_threshold"]:
            rule = 1
        else:
            rule = 0
        cand = max(rule, tl, vl, al) if c["use_max_rule"] else rule

        immediate = 0
        if en["attention"] and c["attention_sustained_bypasses_persistence"] \
                and attention.get("attention_state") == SUSTAINED_STATE:
            immediate = al

        persist = {1: c["medium_persistence_s"], 2: c["high_persistence_s"]}
        if data_valid:
            for lvl in (1, 2):
                if cand >= lvl:
                    if self._since[lvl] is None:
                        self._since[lvl] = t
                else:
                    self._since[lvl] = None
            for lvl in (2, 1):
                since = self._since[lvl]
                if lvl > self.level and since is not None and t - since >= persist[lvl]:
                    self.level, self._below_since = lvl, None
                    break
            if cand < self.level:
                if cand > 0 or combined < c["recovery_threshold"]:
                    if self._below_since is None:
                        self._below_since, self._below_max = t, cand
                    self._below_max = max(self._below_max, cand)
                    if t - self._below_since >= c["recovery_persistence_s"]:
                        self.level, self._below_since = self._below_max, None
                else:
                    self._below_since = None              # combined score inside the hysteresis band: hold
            else:
                self._below_since = None
        if immediate > self.level:
            self.level, self._below_since = immediate, None

        evidence: List[str] = []
        for name, lv in (("task", tl), ("visual", vl), ("attention", al)):
            if lv > 0:
                evidence.append(f"{name}:{LEVEL_LABELS[lv]}")
        if combined >= c["medium_threshold"]:
            evidence.append("combined_score>=medium_threshold")
        if en["attention"] and attention.get("attention_state") == SUSTAINED_STATE:
            evidence.append("sustained_disengagement")
        for name in ("task", "visual", "attention"):
            if not en[name]:
                evidence.append(f"{name}: OFF (not used)")
        if cand < self.level:
            evidence.append("holding level (recovery pending)")
        if not data_valid:
            evidence.append("data not valid (level held)")

        shown = self.level
        if c["report_unknown_as_minus_one"] and not data_valid and immediate == 0:
            shown = -1
        self._diag = {"candidate": cand, "rule_level": rule, "immediate_level": immediate, "data_valid": data_valid,
                      "warnings": list(warnings), "persisted_level": self.level,
                      "recovery_elapsed_s": (t - self._below_since) if self._below_since is not None else 0.0}
        self._out = self._result(t, tl, vl, al, combined, evidence, level=shown)
        return dict(self._out)

    def output(self) -> Dict[str, Any]:
        return dict(self._out)

    def diagnostics(self) -> Dict[str, Any]:
        return dict(self._diag)
