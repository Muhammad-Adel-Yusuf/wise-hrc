"""Camera-derived posture and motion features (from body landmarks) and their aggregation over a work interval.

Input: 3-D body landmarks in metres, e.g. MediaPipe Pose world landmarks (33 x 3, origin at the hips). Only the shoulders,
elbows, wrists and hips are used. Angles are computed from vectors between joints, so they do not depend on the camera
position; the only assumption is the "up" axis (default: -y, as in MediaPipe world coordinates), needed for the trunk
inclination from the vertical.

Frame features
  arm_elev      upper-arm elevation relative to the trunk, deg (0 = arm hanging along the trunk, 90 = horizontal, 180 = overhead)
  trunk_incl    trunk inclination from the vertical, deg
  elbow_flex    elbow flexion, deg (0 = straight arm)
  reach         wrist distance from the shoulder centre / arm length (upper arm + forearm), 0..~1
  wrist_rel     wrist position relative to the shoulder centre (m), used for the wrist speed
Interval features (aggregate_interval)
  arm_elev_p90_deg, arm_elev_mean_deg, trunk_incl_p90_deg, wrist_speed_mean_mps, elbow_speed_mean_dps,
  repetition_rate_per_min, static_posture_s, reach_norm_p90, valid_fraction
"""
from __future__ import annotations

import csv
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

# MediaPipe Pose landmark indices
L_SH, R_SH, L_EL, R_EL, L_WR, R_WR, L_HIP, R_HIP = 11, 12, 13, 14, 15, 16, 23, 24
JOINTS = (L_SH, R_SH, L_EL, R_EL, L_WR, R_WR, L_HIP, R_HIP)
JOINT_NAMES = ("l_shoulder", "r_shoulder", "l_elbow", "r_elbow", "l_wrist", "r_wrist", "l_hip", "r_hip")

FEATURE_KEYS = ("arm_elev_p90_deg", "arm_elev_mean_deg", "trunk_incl_p90_deg", "wrist_speed_mean_mps", "elbow_speed_mean_dps",
                "repetition_rate_per_min", "static_posture_s", "reach_norm_p90", "valid_fraction")

DEFAULT_FEATURE_CFG: Dict[str, Any] = {
    "up": (0.0, -1.0, 0.0),             # vertical "up" in the landmark frame (MediaPipe world: y points down)
    "min_visibility": 0.5,              # joints below this visibility make the frame invalid
    "smooth_s": 0.3,                    # moving-average length for the repetition signal
    "rep_min_prominence": 0.04,         # smallest reach change (fraction of arm length) that counts as one repetition
    "rep_min_distance_s": 0.5,          # shortest time between two repetitions
    "static_wrist_speed_mps": 0.05,     # slower than this ...
    "static_elbow_speed_dps": 10.0,     # ... and this = posture held
    "static_min_arm_elev_deg": 20.0,    # ... and in a non-neutral posture (arm elevated or trunk inclined)
    "static_min_trunk_deg": 20.0,
    "max_gap_s": 0.5,                   # frame gaps longer than this break a static run / are not integrated
    "min_valid_fraction": 0.6,          # fewer valid frames than this: the interval features are NaN (not guessed)
    "speed_lag_s": 0.3,                 # speeds = displacement over this lag / lag (a frame-to-frame difference turns landmark jitter into fake speed)
    "max_speed_mps": 3.0,               # faster wrist motion is a tracking glitch, not movement (ignored)
    "smooth_tau_s": 0.15,               # landmark smoothing (time constant of an exponential average); 0 = off
}


def _angle(a: np.ndarray, b: np.ndarray) -> float:
    na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
    if na < 1e-9 or nb < 1e-9:
        return float("nan")
    return float(np.degrees(np.arccos(np.clip(np.dot(a, b) / (na * nb), -1.0, 1.0))))


class LandmarkSmoother:
    """Time-aware exponential smoothing of the landmark positions (reduces the frame-to-frame jitter of the pose model).

    alpha = 1 - exp(-dt / tau). The state restarts after a missing/invalid frame or a gap longer than `max_gap_s`, so
    nothing is smoothed across a gap. tau <= 0 switches the smoothing off. Only the joints used here are smoothed."""

    def __init__(self, tau_s: float = 0.15, max_gap_s: float = 0.5) -> None:
        self.tau_s, self.max_gap_s = float(tau_s), float(max_gap_s)
        self.reset()

    def reset(self) -> None:
        self._state: Optional[np.ndarray] = None
        self._t: Optional[float] = None

    def __call__(self, t_s: float, world: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if world is None:
            self.reset()
            return None
        w = np.asarray(world, float)
        if w.ndim != 2 or w.shape[0] <= R_HIP or w.shape[1] != 3 or not np.all(np.isfinite(w[list(JOINTS)])):
            self.reset()
            return w
        if self.tau_s <= 0.0:
            return w
        t = float(t_s)
        if self._state is None or self._t is None or not (0.0 < t - self._t <= self.max_gap_s):
            self._state = w.copy()
        else:
            a = 1.0 - float(np.exp(-(t - self._t) / self.tau_s))
            self._state[list(JOINTS)] += a * (w[list(JOINTS)] - self._state[list(JOINTS)])
        self._t = t
        out = w.copy()
        out[list(JOINTS)] = self._state[list(JOINTS)]
        return out


def make_smoother(cfg: Optional[Dict[str, Any]] = None) -> LandmarkSmoother:
    c = {**DEFAULT_FEATURE_CFG, **(cfg or {})}
    return LandmarkSmoother(c["smooth_tau_s"], c["max_gap_s"])


def frame_features(world: np.ndarray, visibility: Optional[np.ndarray] = None,
                   cfg: Optional[Dict[str, Any]] = None) -> Optional[Dict[str, Any]]:
    """Features of one frame, or None if a needed joint is missing / not visible. world: (N>=25, 3) in metres."""
    c = {**DEFAULT_FEATURE_CFG, **(cfg or {})}
    w = np.asarray(world, float)
    if w.ndim != 2 or w.shape[0] <= R_HIP or w.shape[1] != 3:
        return None
    if visibility is not None and np.any(np.asarray(visibility, float)[list(JOINTS)] < c["min_visibility"]):
        return None
    if not np.all(np.isfinite(w[list(JOINTS)])):
        return None
    sh_mid, hip_mid = (w[L_SH] + w[R_SH]) / 2.0, (w[L_HIP] + w[R_HIP]) / 2.0
    trunk = sh_mid - hip_mid
    out: Dict[str, Any] = {"trunk_incl": _angle(trunk, np.asarray(c["up"], float))}
    for side, (sh, el, wr) in (("L", (L_SH, L_EL, L_WR)), ("R", (R_SH, R_EL, R_WR))):
        upper, fore = w[el] - w[sh], w[wr] - w[el]
        out[f"arm_elev_{side}"] = _angle(upper, -trunk)                      # relative to the trunk
        out[f"elbow_flex_{side}"] = 180.0 - _angle(-upper, fore)
        arm_len = float(np.linalg.norm(upper) + np.linalg.norm(fore))
        out[f"reach_{side}"] = float(np.linalg.norm(w[wr] - sh_mid) / arm_len) if arm_len > 1e-9 else float("nan")
        out[f"wrist_rel_{side}"] = (w[wr] - sh_mid).astype(float)
    out["arm_elev"] = float(np.nanmax([out["arm_elev_L"], out["arm_elev_R"]]))
    out["reach"] = float(np.nanmax([out["reach_L"], out["reach_R"]]))
    return out


def build_series(t_s: Sequence[float], frames: Sequence[Optional[Dict[str, Any]]], cfg: Optional[Dict[str, Any]] = None) -> Dict[str, np.ndarray]:
    """Arrays over time (NaN where a frame is invalid) plus the derived wrist speed (m/s) and elbow speed (deg/s)."""
    t = np.asarray(t_s, float)
    n = len(t)
    s = {k: np.full(n, np.nan) for k in ("arm_elev", "trunk_incl", "reach", "arm_elev_L", "arm_elev_R", "reach_L", "reach_R",
                                          "elbow_flex_L", "elbow_flex_R")}
    wr = {"L": np.full((n, 3), np.nan), "R": np.full((n, 3), np.nan)}
    for i, f in enumerate(frames):
        if f is None:
            continue
        for k in s:
            s[k][i] = f[k]
        for side in "LR":
            wr[side][i] = f[f"wrist_rel_{side}"]
    s["t"] = t
    s["valid"] = np.isfinite(s["arm_elev"]) & np.isfinite(s["trunk_incl"])
    c = {**DEFAULT_FEATURE_CFG, **(cfg or {})}
    dt = np.diff(t, prepend=np.nan)
    # reference sample for every frame: the latest one at least `speed_lag_s` earlier (jitter is divided by a longer time)
    ref = np.searchsorted(t, t - float(c["speed_lag_s"]), side="right") - 1 if n else np.array([], int)
    ok_ref = (ref >= 0) & (ref < np.arange(n)) if n else np.array([], bool)
    ref = np.where(ok_ref, ref, 0)
    lag = np.where(ok_ref, t - t[ref], np.nan)
    lag = np.where(lag > 2.0 * max(float(c["speed_lag_s"]), 1e-6) + float(c["max_gap_s"]), np.nan, lag)   # across a gap: no speed
    with np.errstate(invalid="ignore", divide="ignore"):
        ws, es = [], []
        for side in "LR":
            d = np.linalg.norm(wr[side] - wr[side][ref], axis=1)
            v = d / lag
            ws.append(np.where(v > float(c["max_speed_mps"]), np.nan, v))
            es.append(np.abs(s[f"elbow_flex_{side}"] - s[f"elbow_flex_{side}"][ref]) / lag)
    s["wrist_speed"] = np.fmax(ws[0], ws[1])                                                  # fastest wrist
    s["elbow_speed"] = np.fmax(es[0], es[1])
    s["dt"] = dt
    return s


def _smooth(x: np.ndarray, n: int) -> np.ndarray:
    if n <= 1:
        return x
    xs = np.where(np.isfinite(x), x, 0.0)
    k = np.ones(n)
    num, den = np.convolve(xs, k, "same"), np.convolve(np.isfinite(x).astype(float), k, "same")
    return np.where(den > 0, num / np.maximum(den, 1e-9), np.nan)


def count_repetitions(t: np.ndarray, signal: np.ndarray, cfg: Dict[str, Any]) -> int:
    """Number of repetitions = peaks of the (smoothed) reach signal: at least `rep_min_prominence` high, `rep_min_distance_s` apart."""
    from scipy.signal import find_peaks
    ok = np.isfinite(signal)
    if ok.sum() < 5:
        return 0
    tt, ss = t[ok], signal[ok]
    fs = (len(tt) - 1) / max(tt[-1] - tt[0], 1e-9)
    ss = _smooth(ss, max(1, int(round(cfg["smooth_s"] * fs))))
    peaks, _ = find_peaks(ss, prominence=cfg["rep_min_prominence"], distance=max(1, int(round(cfg["rep_min_distance_s"] * fs))))
    return int(len(peaks))


def static_posture_s(s: Dict[str, np.ndarray], cfg: Dict[str, Any]) -> float:
    """Longest run (s) in a non-neutral posture (arm elevated or trunk inclined) that is held (slow wrist and elbow)."""
    t = s["t"]
    with np.errstate(invalid="ignore"):
        held = (s["valid"] & (s["wrist_speed"] < cfg["static_wrist_speed_mps"]) & (s["elbow_speed"] < cfg["static_elbow_speed_dps"])
                & ((s["arm_elev"] >= cfg["static_min_arm_elev_deg"]) | (s["trunk_incl"] >= cfg["static_min_trunk_deg"])))
    best = run = 0.0
    for i in range(1, len(t)):
        dt = t[i] - t[i - 1]
        if held[i] and held[i - 1] and dt <= cfg["max_gap_s"]:
            run += dt
            best = max(best, run)
        else:
            run = 0.0
    return best


def aggregate_interval(s: Dict[str, np.ndarray], t0_s: float, t1_s: float, cfg: Optional[Dict[str, Any]] = None) -> Dict[str, float]:
    """Interval features from the frames with t0 <= t < t1. NaN when too few valid frames (nothing is guessed)."""
    c = {**DEFAULT_FEATURE_CFG, **(cfg or {})}
    m = (s["t"] >= t0_s) & (s["t"] < t1_s)
    nan = float("nan")
    out = {k: nan for k in FEATURE_KEYS}
    n = int(m.sum())
    out["valid_fraction"] = float(s["valid"][m].mean()) if n else 0.0
    if n < 5 or out["valid_fraction"] < c["min_valid_fraction"]:
        return out
    sub = {k: v[m] for k, v in s.items()}
    v = sub["valid"]
    sub["dt"][0] = np.nan                                                    # the first frame of the window has no predecessor
    nanp = lambda x, q: float(np.nanpercentile(x[v], q)) if np.isfinite(x[v]).any() else nan   # noqa: E731
    out["arm_elev_p90_deg"] = nanp(sub["arm_elev"], 90)
    out["arm_elev_mean_deg"] = float(np.nanmean(sub["arm_elev"][v]))
    out["trunk_incl_p90_deg"] = nanp(sub["trunk_incl"], 90)
    out["reach_norm_p90"] = nanp(sub["reach"], 90)
    okg = np.isfinite(sub["dt"]) & (sub["dt"] <= c["max_gap_s"])
    okw, oke = np.isfinite(sub["wrist_speed"]) & okg, np.isfinite(sub["elbow_speed"]) & okg
    out["wrist_speed_mean_mps"] = float(np.mean(sub["wrist_speed"][okw])) if okw.any() else nan
    out["elbow_speed_mean_dps"] = float(np.mean(sub["elbow_speed"][oke])) if oke.any() else nan
    dur_min = (sub["t"][-1] - sub["t"][0]) / 60.0
    if dur_min > 0:
        rl, rr = sub["reach_L"], sub["reach_R"]
        dom = rl if np.nanvar(rl) >= np.nanvar(rr) else rr                  # the hand that moves more
        out["repetition_rate_per_min"] = count_repetitions(sub["t"], dom, c) / dur_min
    out["static_posture_s"] = static_posture_s(sub, c)
    return out


# --- raw landmark CSV (written by extract_features.py) ------------------------------------------------------------
def frames_csv_columns() -> List[str]:
    cols = ["t_s", "valid"]
    for n in JOINT_NAMES:
        cols += [f"{n}_x", f"{n}_y", f"{n}_z"]
    return cols


def frame_to_row(t_s: float, world: Optional[np.ndarray], visibility: Optional[np.ndarray] = None,
                 cfg: Optional[Dict[str, Any]] = None) -> List[Any]:
    ok = world is not None and frame_features(world, visibility, cfg) is not None
    row: List[Any] = [f"{t_s:.4f}", int(ok)]
    for j in JOINTS:
        row += [f"{world[j, k]:.5f}" for k in range(3)] if ok else ["", "", ""]
    return row


def load_frames_csv(path: str, cfg: Optional[Dict[str, Any]] = None) -> Dict[str, np.ndarray]:
    """Read the raw landmark CSV and compute the per-frame feature series."""
    ts, frames = [], []
    smooth = make_smoother(cfg)
    with open(path, newline="", encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            ts.append(float(r["t_s"]))
            if r["valid"] != "1":
                smooth(ts[-1], None)
                frames.append(None)
                continue
            w = np.full((R_HIP + 1, 3), np.nan)
            for j, n in zip(JOINTS, JOINT_NAMES):
                w[j] = [float(r[f"{n}_x"]), float(r[f"{n}_y"]), float(r[f"{n}_z"])]
            frames.append(frame_features(smooth(ts[-1], w), None, cfg))
    return build_series(ts, frames, cfg)


class FrameBuffer:
    """Live use: keeps the last `window_s` seconds of frame features and aggregates them on request."""

    def __init__(self, window_s: float = 30.0, cfg: Optional[Dict[str, Any]] = None) -> None:
        self.window_s, self.cfg = window_s, cfg
        self._smooth = make_smoother(cfg)
        self._t: List[float] = []
        self._f: List[Optional[Dict[str, Any]]] = []

    def add(self, t_s: float, world: Optional[np.ndarray], visibility: Optional[np.ndarray] = None) -> None:
        self._t.append(float(t_s))
        ok = world is not None and frame_features(world, visibility, self.cfg) is not None
        self._f.append(frame_features(self._smooth(t_s, world), None, self.cfg) if ok else None)
        if not ok:
            self._smooth(t_s, None)
        while self._t and self._t[0] < t_s - self.window_s:
            self._t.pop(0)
            self._f.pop(0)

    def aggregate(self) -> Dict[str, float]:
        if len(self._t) < 5:
            return {k: float("nan") for k in FEATURE_KEYS}
        s = build_series(self._t, self._f, self.cfg)
        return aggregate_interval(s, self._t[0], self._t[-1] + 1e-6, self.cfg)