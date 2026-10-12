"""Baselines taken before the estimation starts (per worker / session). Still the standalone physical-fatigue model: nothing here
touches the cognitive load.

1. STARTING FATIGUE   f0 = B0 / 10, B0 = Borg CR10 rating given before the first task (0 = rested). Without it the model assumes f0 = 0.
2. NEUTRAL POSTURE    10-15 s sitting upright, arms relaxed, hands still. It gives
     up                 unit vector of the worker's upright trunk axis in the camera frame -> the trunk inclination is measured from the
                        NEUTRAL posture (0 deg = neutral), which also removes a constant camera tilt (the default 'up' assumes a level camera)
     arm_elev_rest_p90  upper-arm elevation at rest (used to adapt the rest threshold of the estimator)
   Arm elevation, elbow angle and reach are already relative (to the trunk / to the arm length of the worker), so they need no further baseline.
The same baseline file is used by the live program and by the offline feature extraction of the pilot study, so features stay comparable."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from . import features as F

DEFAULT_BASELINE_CFG: Dict[str, Any] = {
    "min_frames": 30,                 # valid frames needed
    "min_valid_fraction": 0.7,        # of all frames in the capture
    "max_trunk_sd_deg": 8.0,          # the trunk must stay still (angular spread around its mean axis, after the trimming below)
    "trim_fraction": 0.10,            # the most deviating share of the frames (tracking glitches, esp. hidden hips) is ignored
    "max_wrist_speed_mps": 0.15,      # hands still
    "rest_arm_margin_deg": 10.0,      # rest threshold = max(default, neutral arm elevation + margin)
}


def initial_fatigue(borg_cr10: Optional[float]) -> float:
    """Starting fatigue f0 = B0 / 10 (0 when no rating was given)."""
    if borg_cr10 is None:
        return 0.0
    b = float(borg_cr10)
    if not (0.0 <= b <= 10.0):
        raise ValueError("Borg CR10 rating must be between 0 and 10")
    return b / 10.0


def trunk_axis(world: np.ndarray) -> Optional[np.ndarray]:
    w = np.asarray(world, float)
    if w.ndim != 2 or w.shape[0] <= F.R_HIP or not np.all(np.isfinite(w[[F.L_SH, F.R_SH, F.L_HIP, F.R_HIP]])):
        return None
    a = (w[F.L_SH] + w[F.R_SH]) / 2.0 - (w[F.L_HIP] + w[F.R_HIP]) / 2.0
    n = float(np.linalg.norm(a))
    return a / n if n > 1e-6 else None


def neutral_from_frames(t_s: Sequence[float], worlds: Sequence[Optional[np.ndarray]], visibility: Optional[Sequence[Optional[np.ndarray]]] = None,
                        cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Neutral-posture baseline from the frames of the capture. worlds[i] = (33, 3) world landmarks or None (nobody seen)."""
    c = {**DEFAULT_BASELINE_CFG, **(cfg or {})}
    n_all = len(worlds)
    axes: List[np.ndarray] = []
    keep: List[int] = []
    smooth = F.make_smoother(None if "smooth_tau_s" not in c else {"smooth_tau_s": c["smooth_tau_s"]})
    sworlds: Dict[int, np.ndarray] = {}
    for i, w in enumerate(worlds):
        v = None if visibility is None else visibility[i]
        if w is None or (v is not None and np.any(np.asarray(v, float)[list(F.JOINTS)] < F.DEFAULT_FEATURE_CFG["min_visibility"])):
            smooth(t_s[i], None)
            continue
        sw = smooth(t_s[i], w)                      # same smoothing as the features use later
        a = trunk_axis(sw)
        if a is not None:
            axes.append(a)
            keep.append(i)
            sworlds[i] = sw
    out: Dict[str, Any] = {"ok": False, "reason": "", "frames": len(axes), "valid_fraction": (len(axes) / n_all) if n_all else 0.0}
    if len(axes) < c["min_frames"] or out["valid_fraction"] < c["min_valid_fraction"]:
        out["reason"] = f"too few valid frames ({len(axes)}, {100 * out['valid_fraction']:.0f} %): stand/sit in view of the camera, shoulders to hips visible"
        return out
    arr = np.array(axes)
    mean0 = arr.mean(axis=0)
    mean0 = mean0 / np.linalg.norm(mean0)
    ang0 = np.degrees(np.arccos(np.clip(arr @ mean0, -1.0, 1.0)))
    out["trunk_sd_raw_deg"] = float(np.sqrt(np.mean(ang0 ** 2)))
    trim = min(max(float(c["trim_fraction"]), 0.0), 0.5)
    sel = ang0 <= np.quantile(ang0, 1.0 - trim) if trim > 0 else np.ones(len(ang0), bool)
    mean = arr[sel].mean(axis=0)
    mean = mean / np.linalg.norm(mean)
    ang = np.degrees(np.arccos(np.clip(arr[sel] @ mean, -1.0, 1.0)))
    sd = float(np.sqrt(np.mean(ang ** 2)))
    out["frames_trimmed"] = int((~sel).sum())
    out["trunk_sd_deg"] = sd
    default_up = np.asarray(F.DEFAULT_FEATURE_CFG["up"], float)
    out["tilt_from_default_deg"] = float(np.degrees(np.arccos(np.clip(float(mean @ default_up), -1.0, 1.0))))
    series = F.build_series([t_s[i] for i in keep], [F.frame_features(sworlds[i], None, {"up": mean.tolist()}) for i in keep])
    ae = series["arm_elev"][np.isfinite(series["arm_elev"])]
    ws = series["wrist_speed"][np.isfinite(series["wrist_speed"])]
    out["arm_elev_rest_p90_deg"] = float(np.percentile(ae, 90)) if len(ae) else None
    out["wrist_speed_mean_mps"] = float(np.mean(ws)) if len(ws) else 0.0
    if sd > c["max_trunk_sd_deg"]:
        out["reason"] = f"trunk moved too much during the capture (spread {sd:.1f} deg > {c['max_trunk_sd_deg']:.0f}): sit still and upright"
        return out
    if out["wrist_speed_mean_mps"] > c["max_wrist_speed_mps"]:
        out["reason"] = f"hands moved during the capture ({out['wrist_speed_mean_mps']:.2f} m/s): keep the arms relaxed and still"
        return out
    out.update(ok=True, reason="", up=[float(x) for x in mean])
    return out


def feature_cfg(baseline: Optional[Dict[str, Any]]) -> Dict[str, Any]:
    """Overrides of the frame-feature settings given by the baseline (trunk inclination from the neutral trunk axis)."""
    return {"up": list(baseline["up"])} if baseline and baseline.get("ok") and baseline.get("up") else {}


def estimator_cfg(baseline: Optional[Dict[str, Any]], cfg: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """Overrides of the estimator settings: rest = arms at (neutral elevation + margin), never below the default 20 deg."""
    c = {**DEFAULT_BASELINE_CFG, **(cfg or {})}
    if baseline and baseline.get("ok") and baseline.get("arm_elev_rest_p90_deg") is not None:
        return {"rest_arm_elev_deg": max(20.0, float(baseline["arm_elev_rest_p90_deg"]) + c["rest_arm_margin_deg"])}
    return {}


def save(baseline: Dict[str, Any], path: str) -> None:
    Path(path).write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")


def load(path: str) -> Optional[Dict[str, Any]]:
    p = Path(path)
    if not p.exists():
        return None
    try:
        b = json.loads(p.read_text(encoding="utf-8"))
    except ValueError:
        return None
    return b if isinstance(b, dict) and b.get("ok") else None
