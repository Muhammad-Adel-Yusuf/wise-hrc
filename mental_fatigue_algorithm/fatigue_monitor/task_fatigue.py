"""ALGORITHM 1 - task-difficulty and instruction-behaviour estimator (standalone).

Analyses how the worker moves through the laptop instructions and the assembly task. It needs an
instruction number from the laptop application (main.py: slides of a PowerPoint with U + arrow keys,
--instruction-file, or keys N / P / A / C) and, optionally, the per-frame attention label (LAPTOP / ASSEMBLY / ...).
It does not depend on the visual or the attention estimator.

Tracked: current / previous instruction, forward and backward transitions, backward jumps of more than
big_backward_steps, check-backs, repeated returns to the same instruction, prolonged viewing, hesitation,
suspected errors, repeated or corrective actions.

    backward_steps = previous_instruction - current_instruction            (> 0 = backward)

BACKWARD TRIPS: REVIEW or SUSPECTED ERROR (each trip is counted once, never as both)
    A backward trip is a run of consecutive backward moves (arrow key by arrow key: 5 -> 4 -> 3 -> 2 -> 1 is ONE trip
    from 5 to 1). It is judged only where the worker SETTLES: after review_min_dwell_s (2 s) on the destination.
    Instructions passed through in less than that are transit, and a trip abandoned before settling counts as nothing.
    * REVIEW           : the worker settled on the earlier instruction and did NOT redo its assembly work.
    * SUSPECTED ERROR  : after settling on the earlier instruction the worker does the assembly of THAT instruction again
                         (attempt: >= attempt_min_assembly_s looking at the assembly area, or an "attempted" event) and that
                         instruction had already been attempted / completed before (= rework). The review is upgraded to
                         an error (not counted twice). Going back to a step that was never attempted (a skipped step)
                         and doing it is catching up, so it stays a review. Distance must be >= error_min_backward_steps.
    check-back : the instruction being left was viewed or attempted before the worker went back.
    hesitation : instruction unchanged, attention alternates LAPTOP <-> ASSEMBLY (>= min alternations),
                 no expected action completed, and time on the instruction > the worker's calibrated normal duration
                 (median of completed instructions * duration_factor).

    task_score  = w_backtracking*backtracking + w_error*error + w_viewing*prolonged_viewing + w_hesitation*hesitation
                  (each part in [0, 1]; weights renormalised to sum 1 -> task_score in [0, 1]);
                  backtracking comes from REVIEWS, error from suspected errors and corrective actions.
    mental_state = clip(mental_state + accumulation_rate*task_score*dt - recovery_rate*recovery_signal*dt, 0, 1)

The reported task_score is the cumulative mental_state (used for the level and by the combiner).
backtracking_count = settled backward trips (reviews + suspected errors); review_count = reviews only.

INITIAL ENGINEERING PARAMETERS (config.json -> task_fatigue), not validated experimentally.
"""
from __future__ import annotations

import statistics
from collections import deque
from typing import Any, Deque, Dict, Optional, Tuple

from .fatigue_common import clip01, integrate_state, score_to_level, step_dt

ACTION_ATTEMPTED = "attempted"      # the worker tried the assembly step
ACTION_COMPLETED = "completed"      # the expected action of the current instruction was completed
ACTION_CORRECTIVE = "corrective"    # the worker repeated / corrected an action


class _Ev:
    """One piece of evidence (time, units); an object so that it can be removed again (review -> error upgrade)."""
    __slots__ = ("t", "units")

    def __init__(self, t: float, units: float) -> None:
        self.t, self.units = t, units


class _Episode:
    """One stay on one instruction (a return to the same instruction starts a new episode)."""

    def __init__(self, t: float) -> None:
        self.t0 = t
        self.laptop_s = 0.0
        self.assembly_s = 0.0
        self.alternations = 0
        self.last_side: Optional[str] = None
        self.attempts = 0
        self.completed = False
        self.hesitation_latched = False
        self.view_latched = False


def _side(label: Optional[str]) -> Optional[str]:
    """'laptop' / 'assembly' for classifier labels ('Laptop') or public names ('LAPTOP'); else None."""
    if not label:
        return None
    u = str(label).upper()
    return "laptop" if u == "LAPTOP" else ("assembly" if u == "ASSEMBLY" else None)


class TaskFatigueEstimator:
    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg                                    # the "task_fatigue" section
        self._durations: list = []                        # calibration: durations of completed instructions
        self._views: list = []                            # calibration: laptop time of completed instructions
        self.reset()

    def reset(self, keep_calibration: bool = True) -> None:
        if not keep_calibration:
            self._durations, self._views = [], []
        self.current: Optional[int] = None
        self.previous: Optional[int] = None
        self.forward_transitions = 0
        self.backward_moves = 0                           # raw backward instruction changes (transit included)
        self.backtracking_trips = 0                       # settled backward trips = reviews + suspected errors
        self.review_count = 0
        self._trip: Optional[Dict[str, Any]] = None
        self.big_backward_jumps = 0
        self.check_back_count = 0
        self.repeated_return_count = 0
        self.prolonged_view_count = 0
        self.hesitation_count = 0
        self.suspected_error_count = 0
        self.corrective_action_count = 0
        self._seen: Dict[int, Dict[str, Any]] = {}
        self._bt: Deque[_Ev] = deque()                    # review evidence
        self._err: Deque[_Ev] = deque()                   # suspected-error / corrective evidence
        self._ep: Optional[_Episode] = None
        self.state = 0.0
        self._last_t: Optional[float] = None
        self._instant = 0.0
        self._parts = {"backtracking": 0.0, "error": 0.0, "prolonged_viewing": 0.0, "hesitation": 0.0}
        self._paused_at: Optional[float] = None
        self._out = self._output()

    # -- standby (collaboration inactive) -----------------------------------------------------
    def pause(self, t: float) -> None:
        if getattr(self, "_paused_at", None) is None:
            self._paused_at = t

    def resume(self, t: float) -> None:
        """Shift every running timer by the standby time: standby is neither hesitation nor viewing time."""
        t0 = getattr(self, "_paused_at", None)
        self._paused_at = None
        self._last_t = None
        if t0 is None or t <= t0:
            return
        d = t - t0
        if self._ep is not None:
            self._ep.t0 += d
        for ev in list(self._bt) + list(self._err):
            ev.t += d

    # -- calibrated "normal" durations ------------------------------------------------
    def normal_duration_s(self) -> float:
        c = self.cfg
        if len(self._durations) >= c["calibration_min_instructions"]:
            return float(statistics.median(self._durations))
        return float(c["default_normal_duration_s"])

    def normal_view_s(self) -> float:
        c = self.cfg
        if len(self._views) >= c["calibration_min_instructions"]:
            return max(float(statistics.median(self._views)) * c["duration_factor"], c["viewed_min_s"])
        return float(c["normal_view_s"])

    def calibrated(self) -> bool:
        return len(self._durations) >= self.cfg["calibration_min_instructions"]

    # -- events ---------------------------------------------------------------------
    def _memory(self, n: int) -> Dict[str, Any]:
        return self._seen.setdefault(n, {"viewed": False, "attempted": False, "completed": False, "advanced": False, "returns": 0})

    def _transition(self, t: float, n: int) -> None:
        c = self.cfg
        prev, ep = self.current, self._ep
        if prev is not None and ep is not None:
            steps = prev - n                              # backward_steps = previous - current
            dwell = t - ep.t0
            mem = self._memory(prev)
            if steps > 0:                                 # ---- backward move
                self.backward_moves += 1
                trip = self._trip
                if trip is not None and not trip["settled"] and dwell < c["review_min_dwell_s"]:
                    trip["dest"] = n                      # still moving: transit, the same trip goes on
                else:                                     # a new backward trip starts from the instruction being left
                    viewed = bool(mem["viewed"] or mem["attempted"] or ep.laptop_s >= c["viewed_min_s"]
                                  or ep.attempts > 0)
                    trip = self._trip = {"origin": prev, "dest": n, "settled": False, "upgraded": False,
                                         "check_back": viewed, "ev": None, "done_before": False}
                tgt = self._memory(n)
                trip["done_before"] = bool(tgt["attempted"] or tgt["completed"])   # before this visit
            else:                                         # ---- forward move ends / cancels any trip
                self._trip = None
                if steps < 0:
                    self.forward_transitions += 1
                    mem["advanced"] = True                # left forwards; NOT "completed": only an event / attempt says work was done
                    if dwell >= c["viewed_min_s"]:        # skipped-through steps do not define "normal"
                        self._durations.append(dwell)
                        self._views.append(ep.laptop_s)
        self.previous = prev
        self.current = n
        self._ep = _Episode(t)
        self._memory(n)

    def _settle(self, t: float) -> None:
        """The worker has stayed review_min_dwell_s on the destination of a backward trip: count it as a review."""
        trip, ep, c = self._trip, self._ep, self.cfg
        if trip is None or trip["settled"] or ep is None or self.current != trip["dest"]:
            return
        if t - ep.t0 < c["review_min_dwell_s"]:
            return
        trip["settled"] = True
        steps = trip["origin"] - trip["dest"]
        self.backtracking_trips += 1
        self.review_count += 1
        if steps > c["big_backward_steps"]:
            self.big_backward_jumps += 1
        units = 1.0 if trip["check_back"] else c["unviewed_backward_unit"]
        if trip["check_back"]:
            self.check_back_count += 1
        tgt = self._memory(trip["dest"])
        tgt["returns"] += 1
        if tgt["returns"] >= 2:                           # repeated return to the same instruction
            self.repeated_return_count += 1
            units += c["repeated_return_unit"]
        trip["ev"] = _Ev(t, units)
        self._bt.append(trip["ev"])

    def _rework(self, t: float) -> None:
        """Settled on an earlier instruction and now doing its assembly again -> the review becomes a suspected error."""
        trip, ep, c = self._trip, self._ep, self.cfg
        if trip is None or not trip["settled"] or trip["upgraded"] or ep is None:
            return
        if not (ep.attempts > 0 or ep.assembly_s >= c["attempt_min_assembly_s"]):
            return
        if not trip["done_before"] or trip["origin"] - trip["dest"] < c["error_min_backward_steps"]:
            return                                        # catching up a skipped step / too small a move: stays a review
        trip["upgraded"] = True
        self.review_count -= 1
        self.suspected_error_count += 1
        if trip["ev"] is not None:
            try:
                self._bt.remove(trip["ev"])               # one event = one category
            except ValueError:
                pass
        self._err.append(_Ev(t, c["error_unit"]))

    def _action(self, t: float, kind: str) -> None:
        if self._ep is None:
            return
        ep, mem = self._ep, self._memory(self.current)
        if kind == ACTION_ATTEMPTED:
            repeated = ep.completed or ep.attempts >= 1
            ep.attempts += 1
            mem["attempted"] = True
            if repeated:                                  # a second attempt / an attempt after completion = corrective
                self._corrective(t)
        elif kind == ACTION_COMPLETED:
            ep.completed = True
            mem["attempted"] = mem["completed"] = True
        elif kind == ACTION_CORRECTIVE:
            self._corrective(t)

    def _corrective(self, t: float) -> None:
        self.corrective_action_count += 1
        self._err.append(_Ev(t, self.cfg["corrective_unit"]))

    # -- per-frame update -------------------------------------------------------------
    def update(self, t: float, current_instruction: Optional[int], attention_label: Optional[str] = None,
               action_event: Optional[str] = None) -> Dict[str, Any]:
        """current_instruction: number shown on the laptop (None = unknown, nothing is analysed).
        attention_label: LAPTOP / ASSEMBLY / other (classifier labels are accepted).
        action_event: None | 'attempted' | 'completed' | 'corrective'."""
        c = self.cfg
        dt = step_dt(self._last_t, t)
        self._last_t = t
        self._settle(t)
        if current_instruction is not None and current_instruction != self.current:
            self._transition(t, int(current_instruction))
        ep = self._ep
        if ep is None:                                    # no instruction information yet
            self._out = self._output()
            return dict(self._out)

        mem = self._memory(self.current)
        side = _side(attention_label)
        if side == "laptop":
            ep.laptop_s += dt
            if ep.laptop_s >= c["viewed_min_s"]:
                mem["viewed"] = True
        elif side == "assembly":
            ep.assembly_s += dt
            if ep.assembly_s >= c["attempt_min_assembly_s"]:
                mem["attempted"] = True
        if side is not None:
            if ep.last_side is not None and side != ep.last_side:
                ep.alternations += 1
            ep.last_side = side
        if action_event:
            self._action(t, action_event)
        self._settle(t)
        self._rework(t)

        # -- component scores, each in [0, 1] --
        lo = t - c["window_s"]
        while self._bt and self._bt[0].t < lo:
            self._bt.popleft()
        while self._err and self._err[0].t < lo:
            self._err.popleft()
        backtracking = clip01(sum(e.units for e in self._bt) / c["backtracking_full_units"])
        error = clip01(sum(e.units for e in self._err) / c["error_full_units"])

        view_thr = self.normal_view_s()
        viewing = clip01((ep.laptop_s - view_thr) / c["prolonged_view_full_s"])
        if ep.laptop_s > view_thr and not ep.view_latched:
            ep.view_latched = True
            self.prolonged_view_count += 1

        elapsed = t - ep.t0
        dur_thr = c["duration_factor"] * self.normal_duration_s()
        hesitating = (ep.alternations >= c["hesitation_min_alternations"] and not ep.completed and elapsed > dur_thr)
        hesitation = clip01((elapsed - dur_thr) / c["hesitation_full_s"]) if hesitating else 0.0
        if hesitating and not ep.hesitation_latched:
            ep.hesitation_latched = True
            self.hesitation_count += 1

        w = c["weights"]
        wsum = sum(w.values()) or 1.0
        self._parts = {"backtracking": backtracking, "error": error, "prolonged_viewing": viewing,
                       "hesitation": hesitation}
        self._instant = clip01((w["backtracking"] * backtracking + w["error"] * error
                                + w["prolonged_viewing"] * viewing + w["hesitation"] * hesitation) / wsum)
        rec = 1.0 if self._instant < c["recovery_score_threshold"] else 0.0
        self.state = integrate_state(self.state, c["accumulation_rate"], self._instant, c["recovery_rate"], rec, dt)
        self._out = self._output()
        return dict(self._out)

    # -- output -------------------------------------------------------------------------
    def _output(self) -> Dict[str, Any]:
        c = self.cfg
        return {
            "task_score": float(self.state),
            "task_fatigue_level": score_to_level(self.state, c["medium_threshold"], c["high_threshold"]),
            "backtracking_count": int(self.backtracking_trips),
            "review_count": int(self.review_count),
            "suspected_error_count": int(self.suspected_error_count),
            "hesitation_count": int(self.hesitation_count),
            "current_instruction": int(self.current) if self.current is not None else -1,   # -1 = no instruction yet
        }

    def output(self) -> Dict[str, Any]:
        return dict(self._out)

    def diagnostics(self) -> Dict[str, Any]:
        return {"active": self._ep is not None, "instant_score": self._instant, "cumulative_state": self.state,
                "components": dict(self._parts), "previous_instruction": self.previous,
                "forward_transitions": self.forward_transitions, "backward_moves": self.backward_moves, "big_backward_jumps": self.big_backward_jumps,
                "check_back_count": self.check_back_count, "repeated_return_count": self.repeated_return_count,
                "prolonged_view_count": self.prolonged_view_count,
                "corrective_action_count": self.corrective_action_count,
                "normal_duration_s": self.normal_duration_s(), "calibrated": self.calibrated()}
