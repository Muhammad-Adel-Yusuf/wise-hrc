"""Procedure 2 - continuous fatigue accumulation and recovery during task execution (online / replay).

The estimator advances the fatigue state with the exact closed-form steps of physical_fatigue.model:
  work      f <- f + (1 - f)(1 - exp(-lambda_i dt))   with lambda_i of the CURRENT workload class
  recovery  f <- f exp(-mu dt)
The state is never reset between tasks (residual fatigue is kept). It is reset to 0 only at the defined full-recovery
condition: in recovery, f <= f_full. The operating mode (work / recovery) and the workload class come either from the
experiment schedule (explicit: task_active / intensity) or from the camera features of the last window."""
from __future__ import annotations

from typing import Any, Dict, Optional

import numpy as np

from . import workload as W
from .model import DEFAULT_PARAMS, INTENSITIES, RECOVERY, WORK, Params, clip01, recovery_state, work_state

DEFAULT_ESTIMATOR: Dict[str, Any] = {
    "f_full": 0.02,                    # full-recovery condition: f <= f_full while recovering -> f := 0
    "rest_wrist_speed_mps": 0.03,      # resting = hands still ...
    "rest_arm_elev_deg": 20.0,         # ... and arms low
    "rest_confirm_s": 20.0,            # ... for this long before the state is treated as recovery
    "class_confirm_windows": 2,         # a new workload class must be seen in this many consecutive windows
    "workload": W.DEFAULT_WORKLOAD,
}


class PhysicalFatigueEstimator:
    def __init__(self, params: Params = DEFAULT_PARAMS, cfg: Optional[Dict[str, Any]] = None, f0: float = 0.0,
                 t0_min: float = 0.0) -> None:
        self.params = params.validate()
        self.cfg = {**DEFAULT_ESTIMATOR, **(cfg or {})}
        self.f = clip01(f0)
        self.t_min = t0_min
        self.mode = RECOVERY                      # until work is seen
        self.intensity: Optional[str] = None      # current workload class (None = not known yet)
        self._cand: Optional[str] = None
        self._cand_n = 0
        self._rest_s = 0.0
        self._work_since_min = t0_min             # start of the current continuous work (for continuous_work_min)
        self.full_recovery_reached = False
        self.unknown_windows = 0

    # --- core: exact steps -------------------------------------------------------------------------------------
    def step(self, dt_min: float, mode: str, intensity: Optional[str] = None) -> float:
        """Advance the state by dt_min minutes in the given mode (the exact closed-form solution, any step size)."""
        if dt_min < 0:
            raise ValueError("dt must be >= 0")
        if mode == WORK:
            if intensity not in INTENSITIES:
                raise ValueError("work needs a workload class L / M / H")
            self.f = work_state(self.f, self.params.lam(intensity), dt_min)
            self.full_recovery_reached = False
        elif mode == RECOVERY:
            self.f = recovery_state(self.f, self.params.mu, dt_min)
            if self.f <= self.cfg["f_full"]:
                self.f, self.full_recovery_reached = 0.0, True          # defined full-recovery condition
        else:
            raise ValueError("mode must be 'work' or 'recovery'")
        self.mode, self.t_min = mode, self.t_min + dt_min
        return self.f

    # --- camera / schedule driven ------------------------------------------------------------------------------
    def update(self, t_min: float, window: Optional[Dict[str, float]] = None, task_active: Optional[bool] = None,
               intensity: Optional[str] = None) -> Dict[str, Any]:
        """Advance to absolute time t_min. The elapsed time since the last call is assigned to the mode / class found now.

        window: camera interval features of the last window (physical_fatigue.features.FrameBuffer.aggregate()).
        task_active / intensity: explicit values from the experiment schedule (they override the camera)."""
        dt = max(0.0, t_min - self.t_min)
        c = self.cfg
        score_info: Dict[str, Any] = {}
        mode, cls = self._decide(dt, window, task_active, intensity, score_info)
        if mode == WORK and self.mode != WORK:
            self._work_since_min = self.t_min                         # a new continuous work period starts
        self.step(dt, mode, cls) if (mode == RECOVERY or cls in INTENSITIES) else self.step(dt, RECOVERY)
        return {"t_min": self.t_min, "f": self.f, "borg_equiv": 10.0 * self.f, "mode": self.mode, "intensity": self.intensity if self.mode == WORK else None,
                "workload_score": score_info.get("score"), "full_recovery": self.full_recovery_reached,
                "unknown_windows": self.unknown_windows}

    def _decide(self, dt_min: float, window, task_active, intensity, info):
        c = self.cfg
        if task_active is not None:                                   # explicit schedule
            if not task_active:
                self._rest_s = 0.0
                return RECOVERY, None
            cls = intensity if intensity in INTENSITIES else self.intensity
            if cls in INTENSITIES:
                self.intensity = cls
                return WORK, cls
            return RECOVERY, None                                     # work declared but no class known: nothing is assumed
        if window is None or not np.isfinite(window.get("valid_fraction", np.nan)) or window.get("valid_fraction", 0.0) <= 0:
            self.unknown_windows += 1                                 # no camera data: keep mode and class, do not guess
            return (self.mode, self.intensity) if self.mode == WORK else (RECOVERY, None)
        w = dict(window)
        w["continuous_work_min"] = max(0.0, self.t_min - self._work_since_min) if self.mode == WORK else 0.0
        ws, ae = w.get("wrist_speed_mean_mps"), w.get("arm_elev_p90_deg")
        resting = (ws is not None and ae is not None and np.isfinite(ws) and np.isfinite(ae)
                   and ws < c["rest_wrist_speed_mps"] and ae < c["rest_arm_elev_deg"])
        self._rest_s = self._rest_s + dt_min * 60.0 if resting else 0.0
        if resting and self._rest_s >= c["rest_confirm_s"]:
            self._cand, self._cand_n = None, 0
            return RECOVERY, None
        res = W.classify(w, c["workload"])
        info.update(res)
        if res["label"] == W.UNKNOWN:
            self.unknown_windows += 1
            return (self.mode, self.intensity) if self.mode == WORK else (RECOVERY, None)
        if self.intensity is None or res["label"] == self.intensity:
            self.intensity, self._cand, self._cand_n = res["label"], None, 0
        elif res["label"] == self._cand:
            self._cand_n += 1
            if self._cand_n >= c["class_confirm_windows"]:
                self.intensity, self._cand, self._cand_n = res["label"], None, 0
        else:
            self._cand, self._cand_n = res["label"], 1
        return WORK, self.intensity
