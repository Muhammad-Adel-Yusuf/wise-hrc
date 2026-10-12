"""Link between this camera program and the MATLAB POMDP framework (POMDP model/liveHRC.m).

The camera program measures the REAL worker; the robot is VIRTUAL (simulated in MATLAB). Every camera frame this module
sends one small UDP message (JSON) with
  * the worker state for the POMDP: physical fatigue level (algorithm 4), final mental fatigue level (algorithms 1-3),
    attention (0 focused / 1 uncertain / 2 distracted), face detected, presence, estimate confidence, task counters;
  * both upper limbs (shoulder, elbow, wrist, palm) as 3-D points in the WORKSTATION frame W (metres): origin at the
    robot base, x towards the worker, y to the worker's right, z up. The frame W comes from workspace_calibration.py;
    without it no limb points are sent and MATLAB treats the safety geometry as NOT verified (the robot stops).
    With the robot-camera calibration (robot_camera_calibration.py) W sits on the REAL robot base and is the robot
    frame turned about its vertical axis (R_w_robot in the workspace file); otherwise on a clicked stand-in object;
  * the worker's head (face model, else pose model: nose, eyes, ears) in W;
  * the REAL robot (RobotPoller, read only): base, shoulder, elbow, tool in W and the tool speed, when the workspace
    file has R_w_robot and the robot pose is fresh. MATLAB then measures the distances to the real robot.
MATLAB computes the distances, zones and risk itself (geometryObservationHRC), exactly as in the simulation.

Nothing here changes the fatigue algorithms: it only reads their outputs.
"""
from __future__ import annotations

import datetime as dt
import json
import logging
import socket
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from . import geometry as geo

log = logging.getLogger("fatigue.pomdp")

# MediaPipe Pose (33 points): shoulder, elbow, wrist of the worker's left / right arm
ARM_POSE_IDX = {"left": (11, 13, 15), "right": (12, 14, 16)}
PALM_IDX = (0, 5, 9, 13, 17)          # Hand Landmarker: wrist + finger bases -> palm centre
FOCUSED_LABELS = ("Assembly", "Laptop", "Storage", "Working area")
OUTSIDE_LABEL = "Outside working area"


# ---------------------------------------------------------------------------------------------------------------
# Workstation frame W
# ---------------------------------------------------------------------------------------------------------------
class WorkspaceFrame:
    """Rigid transform camera -> W:  p_W = R_wc @ (p_cam - origin_cam)   (metres)."""

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = Path(path) if path else None
        self.R: Optional[np.ndarray] = None
        self.origin: Optional[np.ndarray] = None
        self.R_w_robot: Optional[np.ndarray] = None               # W on the real robot: p_W = R_w_robot p_robot_mm / 1000
        self.data: Dict[str, Any] = {}
        if self.path is not None and self.path.exists():
            self.load(self.path)

    @property
    def calibrated(self) -> bool:
        return self.R is not None

    def load(self, path: Path) -> None:
        try:
            d = json.loads(Path(path).read_text(encoding="utf-8"))
            self.R = np.array(d["R_wc"], dtype=float).reshape(3, 3)
            self.origin = np.array(d["origin_cam_m"], dtype=float).reshape(3)
            self.R_w_robot = np.array(d["R_w_robot"], dtype=float).reshape(3, 3) if "R_w_robot" in d else None
            self.data = d
            log.info("Workstation frame loaded from %s", path)
        except (OSError, ValueError, KeyError) as exc:
            log.warning("Cannot read the workstation calibration %s (%s): limb points are NOT sent", path, exc)
            self.R = self.origin = self.R_w_robot = None

    @property
    def on_real_robot(self) -> bool:
        return self.R is not None and self.R_w_robot is not None

    def robot_to_w(self, p_robot_mm) -> Optional[np.ndarray]:
        """Robot frame [mm] -> W [m] (only when W sits on the real robot)."""
        if self.R_w_robot is None or p_robot_mm is None:
            return None
        return self.R_w_robot @ np.asarray(p_robot_mm, dtype=float) / 1000.0

    def to_w(self, p_cam: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if p_cam is None or self.R is None:
            return None
        return self.R @ (np.asarray(p_cam, dtype=float) - self.origin)

    @staticmethod
    def from_table(origin_cam: np.ndarray, normal_cam: np.ndarray, toward_worker_cam: np.ndarray) -> "WorkspaceFrame":
        """Frame from the clicked robot base on the table (workspace_calibration.py): origin = base point on the table,
        z = table normal (up, towards the camera), x = direction to the worker / assembly area projected on the table,
        y = z x x (the worker's right)."""
        z = np.asarray(normal_cam, dtype=float)
        z = z / np.linalg.norm(z)
        h = np.asarray(toward_worker_cam, dtype=float) - np.asarray(origin_cam, dtype=float)
        x = h - np.dot(h, z) * z
        if np.linalg.norm(x) < 1e-6:
            raise ValueError("the assembly area lies on the table normal through the robot base")
        x /= np.linalg.norm(x)
        f = WorkspaceFrame()
        f.R = np.vstack([x, np.cross(z, x), z])
        f.origin = np.asarray(origin_cam, dtype=float).reshape(3)
        return f

    def save(self, path: str, extra: Dict[str, Any]) -> None:
        d = {"R_wc": self.R.tolist(), "origin_cam_m": self.origin.tolist(),
             **({"R_w_robot": self.R_w_robot.tolist()} if self.R_w_robot is not None else {}),
             "frame": "W: origin robot base on the table, x towards the worker, y to the worker's right, z up; metres",
             "created": dt.datetime.now().isoformat(timespec="seconds"), **extra}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
        self.path, self.data = Path(path), d


# ---------------------------------------------------------------------------------------------------------------
# 3-D points from the aligned depth image
# ---------------------------------------------------------------------------------------------------------------
FOREGROUND_PCT = 30.0                 # depth of a landmark = this percentile of the valid depths around it (nearest surface)


def point_3d(depth_mm: Optional[np.ndarray], uv_inf: Sequence[float], inf_size, intr, radius: int,
             dmin: float, dmax: float) -> Optional[np.ndarray]:
    """Pixel (inference image) -> camera point in metres, using the depth image aligned to colour. None if no depth.
    The depth is the FOREGROUND_PCT percentile of the valid depths in the window, not the median: at the edge of an arm
    or hand the window also covers the table / wall behind it, and the limb is always the nearer surface."""
    if depth_mm is None:
        return None
    h, w = depth_mm.shape[:2]
    u = float(uv_inf[0]) * w / inf_size[0]
    v = float(uv_inf[1]) * h / inf_size[1]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None
    patch = depth_mm[max(0, vi - radius):vi + radius + 1, max(0, ui - radius):ui + radius + 1]
    valid = patch[(patch >= dmin) & (patch <= dmax)]
    if valid.size < max(3, 0.2 * patch.size):
        return None
    z = float(np.percentile(valid, FOREGROUND_PCT))
    sx, sy = w / intr.width, h / intr.height                      # intrinsics of the colour image -> depth image size
    return geo.deproject(u, v, z, intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy) / 1000.0


class OneEuro:
    """One-Euro filter for a 3-D point (Casiez et al. 2012): strong smoothing when the point is still (removes depth and
    landmark jitter), little smoothing when it moves fast (so the distance to the robot is not delayed).
    Restarts after a gap of more than reset_s."""

    def __init__(self, min_cutoff: float = 2.0, beta: float = 4.0, d_cutoff: float = 1.5, reset_s: float = 0.3) -> None:
        self.min_cutoff, self.beta, self.d_cutoff, self.reset_s = min_cutoff, beta, d_cutoff, reset_s
        self.t = self.x = self.dx = None

    @staticmethod
    def _alpha(cutoff: float, dt: float) -> float:
        tau = 1.0 / (2.0 * np.pi * cutoff)
        return 1.0 / (1.0 + tau / dt)

    def __call__(self, t: float, x: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if x is None:
            return None
        x = np.asarray(x, dtype=float)
        if self.t is None or t - self.t > self.reset_s or t <= self.t:
            self.t, self.x, self.dx = t, x, np.zeros(3)
            return x
        dt = t - self.t
        dx = (x - self.x) / dt
        a_d = self._alpha(self.d_cutoff, dt)
        self.dx = a_d * dx + (1.0 - a_d) * self.dx
        cutoff = self.min_cutoff + self.beta * float(np.linalg.norm(self.dx))
        a = self._alpha(cutoff, dt)
        self.x = a * x + (1.0 - a) * self.x
        self.t = t
        return self.x


POSE_HAND = {"left": (15, 17, 19), "right": (16, 18, 20)}    # pose wrist, pinky knuckle, index knuckle -> palm centre


class ArmTracker:
    """Both upper limbs in W. The pose model runs on every n-th frame: its points are held for hold_s seconds; the palm
    comes from the hand model (every frame) and is matched to the arm whose wrist is nearest in the image."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.c = cfg
        self.held: Dict[str, Dict[str, Any]] = {s: {"t": -1e9} for s in ARM_POSE_IDX}
        self.filters = {(s, k): OneEuro() for s in ARM_POSE_IDX for k in ("shoulder", "elbow", "wrist", "palm")}

    def update(self, t: float, frame, body_px: Optional[np.ndarray], hands_px: List[np.ndarray], inf_size, intr,
               wf: WorkspaceFrame, cam_cfg: Dict[str, Any], hands_fresh: bool = True, hand_frame=None) -> Dict[str, Any]:
        """frame: the camera frame the POSE landmarks were computed on (its depth is used for them); hand_frame: the
        frame of the HAND landmarks (default: frame). hands_fresh: the hand result is recent enough; otherwise the palm
        comes from the pose model (wrist + pinky + index knuckles)."""
        self._hand_frame = hand_frame if hand_frame is not None else frame
        out = self._update(t, frame, body_px, hands_px if hands_fresh else [], inf_size, intr, wf, cam_cfg)
        for side, a in out.items():                         # smoothing (One-Euro) of every point in W
            for k in ("shoulder", "elbow", "wrist", "palm"):
                a[k] = self.filters[(side, k)](t, a[k])
        return out

    def _update(self, t: float, frame, body_px: Optional[np.ndarray], hands_px: List[np.ndarray], inf_size, intr,
                wf: WorkspaceFrame, cam_cfg: Dict[str, Any]) -> Dict[str, Any]:
        c = self.c
        depth = frame.depth_mm if getattr(frame, "depth_aligned", False) else None
        hf = getattr(self, "_hand_frame", frame)
        depth_h = hf.depth_mm if getattr(hf, "depth_aligned", False) else None
        args = (inf_size, intr, c["depth_radius_px"], cam_cfg["depth_min_mm"], cam_cfg["depth_max_mm"])
        args_h = (inf_size, intr, c["depth_radius_px"], cam_cfg["depth_min_mm"], cam_cfg["depth_max_mm"])
        # pose -> shoulder / elbow / wrist (held between pose frames)
        if body_px is not None and len(body_px) >= 17:
            for side, idx in ARM_POSE_IDX.items():
                vis = float(min(body_px[i, 2] for i in idx))
                pts = [point_3d(depth, body_px[i, :2], *args) if body_px[i, 2] >= c["min_visibility"] else None for i in idx]
                hp = [point_3d(depth, body_px[i, :2], *args) for i in POSE_HAND[side] if body_px[i, 2] >= c["min_visibility"]]
                hp = [p for p in hp if p is not None]
                self.held[side] = {"t": t, "vis": vis, "wrist_px": body_px[idx[2], :2].copy(),
                                   "pts": [wf.to_w(p) for p in pts],
                                   "pose_palm": wf.to_w(np.median(np.array(hp), axis=0)) if len(hp) >= 2 else None}
        out: Dict[str, Any] = {}
        for side in ARM_POSE_IDX:
            hd = self.held[side]
            fresh = t - hd["t"] <= c["arm_hold_s"]
            pts = hd.get("pts", [None, None, None]) if fresh else [None, None, None]
            out[side] = {"shoulder": pts[0], "elbow": pts[1], "wrist": pts[2], "palm": None,
                         "vis": float(hd.get("vis", 0.0)) if fresh else 0.0,
                         "wrist_px": hd.get("wrist_px") if fresh else None,
                         "pose_palm": hd.get("pose_palm") if fresh else None}
        # hands -> palm centre, matched to the nearest pose wrist
        free = []
        for hand in hands_px:
            centre_px = hand[list(PALM_IDX)].mean(axis=0)
            pts = [point_3d(depth_h, hand[i], *args_h) for i in PALM_IDX]       # depth of the hand's own frame
            pts = [p for p in pts if p is not None]
            palm = wf.to_w(np.median(np.array(pts), axis=0)) if len(pts) >= 2 else None
            best, best_d = None, c["hand_match_px"] * inf_size[0] / 960.0
            for side in ARM_POSE_IDX:
                wp = out[side]["wrist_px"]
                if wp is not None and out[side]["palm"] is None:
                    d = float(np.linalg.norm(hand[0] - wp))
                    if d < best_d:
                        best, best_d = side, d
            if best is not None:
                out[best]["palm"] = palm
            else:
                free.append(palm)
        for side in ARM_POSE_IDX:                                  # a hand without a pose arm: palm only
            if out[side]["palm"] is None and out[side]["wrist_px"] is None and free:
                out[side]["palm"] = free.pop(0)
        for side in ARM_POSE_IDX:
            a = out[side]
            a.pop("wrist_px", None)
            pose_palm = a.pop("pose_palm", None)
            a["palm_estimated"] = False
            if a["palm"] is None and pose_palm is not None:     # hand model missed / older than this frame: pose palm
                a["palm"] = pose_palm
            if a["palm"] is None and a["wrist"] is not None and a["elbow"] is not None:
                d = a["wrist"] - a["elbow"]                       # hand model missed this hand: palm ~8 cm beyond the wrist
                if np.linalg.norm(d) > 1e-6:
                    a["palm"] = a["wrist"] + c["palm_from_wrist_m"] * d / np.linalg.norm(d)
                    a["palm_estimated"] = True
        return out


class HeadTracker:
    """Worker's head in W: face model (face surface at the nose bridge), else pose model (nose, eyes, ears). The pose
    model runs on every n-th frame only, so its head point is held for hold_s seconds."""

    FACE_IDX = (1, 168, 10, 50, 280, 152)
    POSE_IDX = (0, 2, 5, 7, 8)

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.c = cfg
        self.held: Dict[str, Any] = {"t": -1e9, "p": None, "src": ""}
        self.filter = OneEuro(min_cutoff=1.5, beta=3.0)

    def update(self, t: float, frame, face_px, body_px, inf_size, intr, wf, cam_cfg, pose_frame=None):
        """(head point in W, smoothed, or None; source 'face' | 'pose' | ''). frame: the frame of the FACE landmarks,
        pose_frame: the frame of the POSE landmarks (default: frame)."""
        self._pose_frame = pose_frame if pose_frame is not None else frame
        p, src = self._update(t, frame, face_px, body_px, inf_size, intr, wf, cam_cfg)
        return self.filter(t, p), src

    def _update(self, t: float, frame, face_px: Optional[np.ndarray], body_px: Optional[np.ndarray], inf_size, intr,
                wf: WorkspaceFrame, cam_cfg: Dict[str, Any]):
        """(head point in W or None, source 'face' | 'pose' | '')."""
        depth = frame.depth_mm if getattr(frame, "depth_aligned", False) else None
        if depth is None:
            return None, ""
        args = (inf_size, intr, self.c["depth_radius_px"], cam_cfg["depth_min_mm"], cam_cfg["depth_max_mm"])
        if face_px is not None and len(face_px) >= 468:
            pts = [point_3d(depth, face_px[i], *args) for i in self.FACE_IDX]
            pts = np.array([p for p in pts if p is not None])
            if len(pts) >= 2:
                z = np.median(pts[:, 2])
                pts = pts[np.abs(pts[:, 2] - z) <= 0.08]            # drop hair / background hits
                if len(pts) >= 2:
                    return wf.to_w(np.median(pts, axis=0)), "face"
        if body_px is not None and len(body_px) >= 9:
            pf = getattr(self, "_pose_frame", frame)
            depth_p = pf.depth_mm if getattr(pf, "depth_aligned", False) else depth
            pts = [point_3d(depth_p, body_px[i, :2], *args) for i in self.POSE_IDX if body_px[i, 2] >= self.c["min_visibility"]]
            pts = [p for p in pts if p is not None]
            if pts:
                self.held = {"t": t, "p": wf.to_w(np.median(np.array(pts), axis=0)), "src": "pose"}
        if t - self.held["t"] <= self.c["arm_hold_s"] and self.held["p"] is not None:
            return self.held["p"], self.held["src"]
        return None, ""


def robot_block(poller, wf: WorkspaceFrame, max_age_s: float) -> Optional[Dict[str, Any]]:
    """Real robot points for the message (W, metres); None when W is not on the real robot. Without a fresh pose
    only {'status': reason, 'age': s}: MATLAB then uses no real robot (stops it) and prints the reason."""
    if poller is None or not wf.on_real_robot:
        return None
    last = poller.latest()
    if last is None:
        return {"status": "no robot pose yet (reader: %s)" % poller.status}
    age = time.monotonic() - last["t"]
    if age > max_age_s:
        return {"status": "robot pose %.1f s old (reader: %s)" % (age, poller.status), "age": round(float(age), 3)}
    p = last["pts"]
    return {k: _vec(wf.robot_to_w(p[k])) for k in ("base", "shoulder", "elbow", "tool")} | {
        "speed": round(float(last["speed_mps"]), 4), "jspeed": round(float(last.get("jspeed_dps", 0.0)), 2),
        "age": round(float(age), 3)}


def arm_status(arms: Dict[str, Any]) -> str:
    """Overlay text: which arm points are sent (S shoulder, E elbow, W wrist, P palm, p = palm estimated, - = missing).
    MATLAB verifies the safety geometry only when an arm it sees is complete (S E W P)."""
    out = []
    for side in ("left", "right"):
        a = arms.get(side, {})
        txt = "".join(k if a.get(f) is not None else "-" for k, f in (("S", "shoulder"), ("E", "elbow"), ("W", "wrist")))
        txt += ("p" if a.get("palm_estimated") else "P") if a.get("palm") is not None else "-"
        out.append(f"{side} {txt}")
    return "arms sent: " + "  ".join(out)


# ---------------------------------------------------------------------------------------------------------------
# Message
# ---------------------------------------------------------------------------------------------------------------
def _vec(p: Optional[np.ndarray]) -> Optional[List[float]]:
    return None if p is None or not np.all(np.isfinite(p)) else [round(float(x), 4) for x in p]


MIN_USABLE_QUALITY = 0.5     # mental level still sent (with confidence = quality) when only the quality is low
FATIGUE_RESETS = [0]     # counter of manual fatigue resets (main.py: F, Z Z); MATLAB resets its monitor on a change


def attention_code(st) -> Optional[int]:
    """Algorithm 3 label of this frame -> POMDP attention: 0 focused, 1 uncertain (outside < 2 s), 2 distracted (>= 2 s)."""
    if st.attention in FOCUSED_LABELS:
        return 0
    if st.attention == OUTSIDE_LABEL:
        return 2 if st.immediate_distraction else 1
    return None                                                   # Unknown: no head pose -> not observed


def build_message(seq: int, st, obs, phys_state: Optional[Dict[str, Any]], arms: Dict[str, Any],
                  wf: WorkspaceFrame, collab: bool, head: Optional[np.ndarray] = None,
                  robot: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
    """One UDP message. None / null = not available (MATLAB reads it as NaN)."""
    valid = bool(st.data_valid) and not st.standby
    # mental estimate invalid ONLY because of the data-quality index (e.g. Q = 0.7 < 0.8): the level is still sent,
    # with the measured quality as its confidence; the decision layer applies its own threshold (0.5) to it.
    # Without any estimate (baseline, warming up, standby) the confidence stays 0.3.
    warns = list(getattr(st, "data_warnings", []) or [])
    quality_only = (not valid) and not st.standby and bool(warns) and all(str(w).startswith("data quality") for w in warns)
    usable = valid or (quality_only and float(st.quality) >= MIN_USABLE_QUALITY)
    phys_ok = phys_state is not None and phys_state.get("enabled") and phys_state.get("seen")
    person = not str(st.presence).startswith("not detected")
    task = st.task or {}
    return {
        "v": 1, "seq": seq, "t": round(time.monotonic(), 3),
        "calibrated": wf.calibrated, "person": person, "presence": st.presence,
        "Fm": int(st.level_code) if usable else None, "mental_label": st.level, "mental_valid": valid,
        "mental_usable": usable,
        "mental_score": round(float(st.combined_score), 3),
        "Fp": int(phys_state["band"]) if phys_ok else None,
        "physical_f": round(float(phys_state["f"]), 3) if phys_ok else None,
        "attention": attention_code(st), "attention_label": st.attention,
        "away_s": round(float(getattr(st, "d_outside_s", 0.0) or 0.0), 2),   # time looking away from the work now
        "fatigue_reset": int(FATIGUE_RESETS[0]),                  # +1 at every manual reset (keys F, Z Z)
        "face": bool(getattr(obs, "face_ok", False)),
        "conf": round(float(st.quality), 3) if usable else 0.3,
        "instruction": task.get("current_instruction"),
        "counts": {"backtracking": task.get("backtracking_count", 0), "hesitation": task.get("hesitation_count", 0),
                   "error": task.get("suspected_error_count", 0)},
        "collab_flag": bool(collab),
        "arms": {side: {k: _vec(a[k]) for k in ("shoulder", "elbow", "wrist", "palm")}
                 | {"vis": round(a["vis"], 3), "palm_estimated": bool(a.get("palm_estimated", False))}
                 for side, a in arms.items()},
        "head": _vec(head),
        "real_robot_frame": wf.on_real_robot,
        "robot": robot,                                           # null: MATLAB uses its virtual robot
    }


class BridgeSender:
    """UDP sender (fire and forget: MATLAB always reads the newest message; a frozen camera shows up as 'stale')."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.addr = (cfg["host"], int(cfg["port"]))
        self.min_dt = 1.0 / float(cfg["send_hz"])
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.seq, self._last, self.sent, self.errors = 0, -1e9, 0, 0

    def maybe_send(self, t: float, make_msg) -> bool:
        if t - self._last < self.min_dt:
            return False
        self._last = t
        self.seq += 1
        try:
            self.sock.sendto(json.dumps(make_msg(self.seq), allow_nan=False).encode("utf-8"), self.addr)
            self.sent += 1
            return True
        except (OSError, ValueError) as exc:
            self.errors += 1
            if self.errors <= 3:
                log.warning("POMDP link: send failed (%s)", exc)
            return False

    def close(self) -> None:
        try:
            self.sock.close()
        except OSError:
            pass
