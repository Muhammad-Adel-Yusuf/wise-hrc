"""Algorithm 4 runtime - PHYSICAL fatigue, estimated ALONGSIDE (never inside) the mental / cognitive load.

This module is only the glue between the camera loop and the standalone model in the sibling folder `physical_fatigue_model`
(LFFR work / recovery model, camera workload classification, calibration tools). It imports nothing of the other three
algorithms, the combiner or the monitor, and none of them imports it: physical fatigue is a separate state variable,
there is no weight function between it and the cognitive load, and no NASA-TLX / cognitive variable enters it.

Input per camera frame: MediaPipe pose WORLD landmarks (metres) of the worker. Output: f in 0..1 (normalised physical
fatigue), its Borg-CR10 equivalent (10 f), the current workload class (L / M / H), work / recovery mode and the
camera features. It is an operational state for scheduling and human-robot decisions. It is NOT a physiological
measurement unless validated against EMG, force or muscle activity. Until physical_params.json exists (calibrate.py of the
pilot study) the INITIAL default rates are used and the state is labelled 'uncalibrated'.

Baselines before the estimation (key Y): the starting fatigue (Borg CR10 rating B0 -> f0 = B0 / 10) and a ~12 s neutral-posture capture
(sit upright, arms relaxed, hands still) that sets the trunk reference axis and the rest threshold. Without them f0 = 0 and the camera is
assumed level (the panel says 'no baseline')."""
from __future__ import annotations

import datetime as dt
import logging
import sys
from collections import deque
from pathlib import Path
from typing import Any, Deque, Dict, Optional, Tuple

import numpy as np

log = logging.getLogger("fatigue.physical")

BAND_NAMES = ("Low", "Medium", "High")
LOG_COLUMNS = ["timestamp_iso", "t_s", "physical_f", "borg_equiv", "band", "mode", "workload_class", "workload_score",
               "valid_fraction", "arm_elev_p90_deg", "trunk_incl_p90_deg", "wrist_speed_mean_mps", "elbow_speed_mean_dps",
               "repetition_rate_per_min", "static_posture_s", "reach_norm_p90", "params", "unknown_windows", "start_borg", "neutral_posture"]


class PhysicalUnavailable(RuntimeError):
    """The physical_fatigue_model folder / its dependencies are missing. Never fatal for the combined program."""


def load_package(package_dir: str, base_dir: Path) -> Dict[str, Any]:
    pd = Path(package_dir)
    if not pd.is_absolute():
        pd = (Path(base_dir) / pd).resolve()
    if not (pd / "physical_fatigue").is_dir():
        raise PhysicalUnavailable(f"physical_fatigue_model not found at {pd} (set physical.package_dir in config.json)")
    if str(pd) not in sys.path:
        sys.path.insert(0, str(pd))
    try:
        from physical_fatigue import baseline, features, model
        from physical_fatigue import estimator as est
    except ImportError as exc:                                         # e.g. scipy missing
        raise PhysicalUnavailable(f"physical_fatigue_model cannot be imported: {exc} (pip install -r {pd / 'requirements.txt'})") from exc
    return {"features": features, "model": model, "estimator": est, "baseline": baseline, "dir": pd}


class PhysicalAlgorithm:
    def __init__(self, cfg: Dict[str, Any], base_dir: Path) -> None:
        self.c = cfg["physical"]
        self.pkg = load_package(self.c["package_dir"], base_dir)
        M = self.pkg["model"]
        self.B = self.pkg["baseline"]
        pf = Path(self.c["params_file"])
        if not pf.is_absolute():
            pf = Path(base_dir) / pf
        if pf.exists():
            self.params, self.params_source = M.Params.load(str(pf)), f"calibrated ({pf.name})"
        else:
            self.params, self.params_source = M.DEFAULT_PARAMS, "initial defaults - UNCALIBRATED"
        self.est_cfg = dict(self.c.get("estimator") or {})
        self.base_cfg = dict(self.c.get("baseline") or {})
        bf = Path(self.c["baseline_file"])
        self.baseline_path = bf if bf.is_absolute() else Path(base_dir) / bf
        self.baseline: Optional[Dict[str, Any]] = self.B.load(str(self.baseline_path))   # neutral posture of the last capture (same camera / seat)
        self.f0 = 0.0
        self.start_borg: Optional[float] = None
        self.capture: Optional[Dict[str, Any]] = None
        self.message = ""
        self.enabled = True
        if self.c.get("start_borg") is not None:
            self.set_start_rating(self.c["start_borg"])
        else:
            self._build(None)

    # -- (re)build the estimator / buffer with the current baselines ---------------------------------------
    def _build(self, t: Optional[float]) -> None:
        E, F = self.pkg["estimator"], self.pkg["features"]
        self.estimator = E.PhysicalFatigueEstimator(self.params, {**self.est_cfg, **self.B.estimator_cfg(self.baseline, self.base_cfg)}, f0=self.f0)
        self.buffer = F.FrameBuffer(self.c["window_s"], {"min_visibility": self.c["min_visibility"], **self.B.feature_cfg(self.baseline)})
        self.t0: Optional[float] = t
        self._last_update: Optional[float] = None
        self._last_valid_t: Optional[float] = None
        self.hist: Deque[Tuple[float, float]] = deque()
        self.state: Dict[str, Any] = self._blank()

    # -- control ------------------------------------------------------------------------------------
    def reset(self, t: Optional[float]) -> None:
        """Fatigue back to 0 (new worker / new shift; the starting rating is cleared too - press Y again). Otherwise residual fatigue is kept."""
        self.f0, self.start_borg = 0.0, None
        self._build(None)

    def set_start_rating(self, borg_cr10: float, t: Optional[float] = None) -> None:
        """Starting fatigue f0 = B0 / 10 (Borg CR10 before the first task). Restarts the estimate from f0."""
        self.f0 = self.B.initial_fatigue(borg_cr10)
        self.start_borg = float(borg_cr10)
        self._build(None)
        self.message = f"Starting fatigue set: Borg {self.start_borg:g} -> f0 = {self.f0:.2f}"

    def start_capture(self, t: float) -> None:
        """Neutral-posture capture: the worker sits upright, arms relaxed, hands still, for neutral_capture_s."""
        self.capture = {"t0": t, "t": [], "w": [], "v": [], "n": 0}
        self.message = "Sit upright, arms relaxed, hands still"

    @property
    def capturing(self) -> bool:
        return self.capture is not None

    def _finish_capture(self) -> None:
        cap, self.capture = self.capture, None
        res = self.B.neutral_from_frames(cap["t"], cap["w"], cap["v"], self.base_cfg)
        res["n_frames_total"] = cap["n"]
        if res["ok"]:
            res["captured_iso"] = dt.datetime.now().isoformat(timespec="seconds")
            self.baseline = res
            try:
                self.B.save(res, str(self.baseline_path))
            except OSError as exc:
                log.warning("Could not save %s: %s", self.baseline_path, exc)
            self._build(None)                                         # features / rest threshold now use the neutral posture
            self.message = f"Neutral posture saved (camera tilt {res['tilt_from_default_deg']:.1f} deg, rest arm {res['arm_elev_rest_p90_deg']:.0f} deg)"
        else:
            self.message = "Neutral posture NOT saved: " + res["reason"]
        log.info("Physical baseline: %s", self.message)

    def _blank(self) -> Dict[str, Any]:
        return {"enabled": self.enabled, "f": self.f0, "borg_equiv": 10.0 * self.f0, "band": 0, "band_name": "Low", "mode": "recovery",
                "intensity": None, "workload_score": None, "valid_fraction": 0.0, "features": {}, "params": self.params_source,
                "unknown_windows": 0, "t_min": 0.0, "seen": False, "full_recovery": False, "history": [], **self._baseline_info()}

    def _baseline_info(self, progress: Optional[float] = None) -> Dict[str, Any]:
        parts = ["neutral posture " + ("OK" if self.baseline else "MISSING"),
                 "start Borg " + ("not set (f0 = 0)" if self.start_borg is None else f"{self.start_borg:g}")]
        return {"baseline_text": " | ".join(parts) + ("" if self.baseline and self.start_borg is not None else "  - press Y"),
                "baseline_complete": bool(self.baseline and self.start_borg is not None),
                "capture_progress": progress, "message": self.message}

    # -- per frame ----------------------------------------------------------------------------------
    def feed(self, t: float, world: Optional[np.ndarray]) -> None:
        """world: None = the pose model did not run on this frame (nothing is added); (0, 4) = ran, nobody found;
        (N, 4) = x, y, z (m), visibility."""
        if not self.enabled or world is None:
            return
        if self.capture is not None:                                  # baseline capture: frames are collected, not estimated
            self.capture["n"] += 1
            if len(world):
                self.capture["t"].append(t)
                self.capture["w"].append(world[:, :3])
                self.capture["v"].append(world[:, 3])
            else:
                self.capture["t"].append(t)
                self.capture["w"].append(None)
                self.capture["v"].append(None)
            return
        if self.t0 is None:
            self.t0 = t
        if len(world) == 0:
            self.buffer.add(t, None)
        else:
            self.buffer.add(t, world[:, :3], world[:, 3])
            self._last_valid_t = t

    def update(self, t: float) -> Dict[str, Any]:
        """Advance the fatigue state (every update_s) and return the latest state dict."""
        if self.capture is not None:
            el = t - self.capture["t0"]
            if el >= self.c["neutral_capture_s"]:
                self._finish_capture()
            else:
                self.state = {**self.state, **self._baseline_info(el / self.c["neutral_capture_s"])}
                return self.state
        if not self.enabled or self.t0 is None:
            self.state = {**self.state, "enabled": self.enabled, **self._baseline_info()}
            return self.state
        if self._last_update is not None and t - self._last_update < self.c["update_s"]:
            return self.state
        self._last_update = t
        t_min = (t - self.t0) / 60.0
        win = self.buffer.aggregate()
        vf = win.get("valid_fraction", 0.0)
        absent = (self.c["absent_is_recovery"] and (self._last_valid_t is None or t - self._last_valid_t >= self.c["window_s"]))
        out = self.estimator.update(t_min, None if absent else win, False if absent else None)
        f = float(out["f"])
        b1, b2 = self.c["bands"]
        band = 0 if f < b1 else (1 if f < b2 else 2)
        self.hist.append((t_min, f))
        while self.hist and self.hist[0][0] < t_min - self.c["history_min"]:
            self.hist.popleft()
        self.state = {"enabled": True, "f": f, "borg_equiv": 10.0 * f, "band": band, "band_name": BAND_NAMES[band], "mode": out["mode"],
                      "intensity": out["intensity"], "workload_score": out["workload_score"],
                      "valid_fraction": float(vf) if np.isfinite(vf) else 0.0, "features": {k: (None if not np.isfinite(v) else float(v)) for k, v in win.items()},
                      "params": self.params_source, "unknown_windows": out["unknown_windows"], "t_min": t_min,
                      "seen": not absent, "full_recovery": bool(out["full_recovery"]), "history": list(self.hist), **self._baseline_info()}
        return self.state

    def log_row(self) -> Dict[str, Any]:
        s, ft = self.state, self.state["features"]
        return {"t_s": None, "physical_f": s["f"], "borg_equiv": s["borg_equiv"], "band": s["band_name"], "mode": s["mode"],
                "workload_class": s["intensity"], "workload_score": s["workload_score"], "valid_fraction": s["valid_fraction"],
                "params": s["params"], "unknown_windows": s["unknown_windows"], "start_borg": self.start_borg,
                "neutral_posture": int(bool(self.baseline)),
                **{k: ft.get(k) for k in ("arm_elev_p90_deg", "trunk_incl_p90_deg", "wrist_speed_mean_mps", "elbow_speed_mean_dps",
                                          "repetition_rate_per_min", "static_posture_s", "reach_norm_p90")}}
