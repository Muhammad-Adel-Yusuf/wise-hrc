"""Camera -> REAL robot base frame (UFactory Lite 6), from robot_camera_calibration.py.

    p_robot_mm = R @ p_cam_mm + t_mm

Robot base frame B: the Lite 6 controller's own base frame (x, y, z in mm, origin at the base, z up); the same frame
as arm.get_position() and the gateway safety box. The calibration is an eye-to-hand fit: a marker fixed on the robot
tool is seen by the depth camera at several robot poses; the fit finds R, t and the marker offset on the tool.

Also: the 3-D head and palm points of the worker in the camera frame (depth aligned to colour), the connection to
the robot through the Jetson gateway relay (robot_code/lite6.py, saved login), the Lite 6 link points from the joint
angles, the workstation frame W built on the robot base (workspace_frame_from_robot) and a read-only robot pose reader
(RobotPoller) for the MATLAB link.
"""
from __future__ import annotations

import datetime as dt
import json
import logging
import sys
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as geo

log = logging.getLogger("fatigue.robot_frame")

PROJECT = Path(__file__).resolve().parents[2]                    # pilot_physical_full_code
CALIB_FILE = PROJECT / "POMDP model" / "robot_camera_calibration.json"
ROBOT_CODE_DIR = PROJECT / "robot_code"
PALM_IDX = (0, 5, 9, 13, 17)          # Hand Landmarker: wrist + finger bases -> palm centre
HEAD_IDX = (1, 168, 10, 50, 280, 152)  # Face Landmarker: nose tip, nose bridge, forehead, cheeks, chin


# ---------------------------------------------------------------------------------------------------------------
# Transform
# ---------------------------------------------------------------------------------------------------------------
def rpy_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """xArm tool orientation (degrees) -> rotation matrix base<-tool:  Rz(yaw) Ry(pitch) Rx(roll)."""
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", [roll, pitch, yaw], degrees=True).as_matrix()


class RobotFrame:
    """Rigid transform camera -> robot base, both in mm."""

    def __init__(self, path: Optional[Path] = CALIB_FILE) -> None:
        self.path = Path(path) if path else None
        self.R: Optional[np.ndarray] = None
        self.t: Optional[np.ndarray] = None
        self.data: Dict[str, Any] = {}
        if self.path is not None and self.path.exists():
            self.load(self.path)

    @property
    def calibrated(self) -> bool:
        return self.R is not None

    def load(self, path: Path) -> None:
        try:
            d = json.loads(Path(path).read_text(encoding="utf-8"))
            self.R = np.array(d["R_robot_cam"], dtype=float).reshape(3, 3)
            self.t = np.array(d["t_robot_cam_mm"], dtype=float).reshape(3)
            self.data = d
            log.info("Robot-camera calibration loaded from %s (rms %.1f mm)", path, d.get("rms_mm", float("nan")))
        except (OSError, ValueError, KeyError) as exc:
            log.warning("Cannot read the robot-camera calibration %s (%s)", path, exc)
            self.R = self.t = None

    def to_robot(self, p_cam_mm: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if p_cam_mm is None or self.R is None:
            return None
        return self.R @ np.asarray(p_cam_mm, dtype=float) + self.t

    def to_cam(self, p_robot_mm: np.ndarray) -> np.ndarray:
        return self.R.T @ (np.asarray(p_robot_mm, dtype=float) - self.t)

    def save(self, path: Path, extra: Dict[str, Any]) -> None:
        d = {"R_robot_cam": self.R.tolist(), "t_robot_cam_mm": self.t.tolist(),
             "use": "p_robot_mm = R_robot_cam @ p_cam_mm + t_robot_cam_mm",
             "frame": "Lite 6 base frame (same as arm.get_position()), mm; camera: x right, y down, z forward, mm",
             "created": dt.datetime.now().isoformat(timespec="seconds"), **extra}
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        Path(path).write_text(json.dumps(d, indent=2) + "\n", encoding="utf-8")
        self.path, self.data = Path(path), d


def kabsch(a: np.ndarray, b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """R, t minimising |R a_i + t - b_i| (rows are points)."""
    ca, cb = a.mean(axis=0), b.mean(axis=0)
    U, _, Vt = np.linalg.svd((a - ca).T @ (b - cb))
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    return R, cb - R @ ca


def solve_eye_to_hand(cam_mm: np.ndarray, tcp_pos_mm: np.ndarray, tcp_rot: np.ndarray,
                      fit_offset: bool = True) -> Dict[str, Any]:
    """Fit R, t (camera -> base) and the marker offset o (tool frame) to
        R c_i + t = R_tool_i o + p_tool_i        for every sample i.
    cam_mm (N,3) marker in the camera, tcp_pos_mm (N,3) tool position, tcp_rot (N,3,3) tool orientation."""
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    n = len(cam_mm)
    R0, t0 = kabsch(cam_mm, tcp_pos_mm)                          # first guess: marker at the tool point
    x0 = np.r_[Rotation.from_matrix(R0).as_rotvec(), t0, np.zeros(3 if fit_offset else 0)]

    def resid(x):
        R = Rotation.from_rotvec(x[:3]).as_matrix()
        o = x[6:9] if fit_offset else np.zeros(3)
        target = tcp_pos_mm + np.einsum("nij,j->ni", tcp_rot, o)
        return ((cam_mm @ R.T + x[3:6]) - target).ravel()

    sol = least_squares(resid, x0, loss="soft_l1", f_scale=5.0)
    x = sol.x
    R = Rotation.from_rotvec(x[:3]).as_matrix()
    err = np.linalg.norm(resid(x).reshape(n, 3), axis=1)
    return {"R": R, "t": x[3:6], "offset_mm": x[6:9] if fit_offset else np.zeros(3),
            "errors_mm": err, "rms_mm": float(np.sqrt(np.mean(err ** 2))), "max_mm": float(err.max())}


def rotation_spread_deg(tcp_rot: np.ndarray) -> float:
    """Largest angle between any tool orientation and the first one (offset observability)."""
    from scipy.spatial.transform import Rotation
    return max(float(np.degrees(Rotation.from_matrix(tcp_rot[0].T @ Ri).magnitude())) for Ri in tcp_rot)


# ---------------------------------------------------------------------------------------------------------------
# 3-D points of the worker in the camera frame (mm)
# ---------------------------------------------------------------------------------------------------------------
def pixel_point_mm(depth_mm: np.ndarray, u: float, v: float, intr, radius: int, dmin: float, dmax: float,
                   percentile: float = 50.0) -> Optional[np.ndarray]:
    """Colour pixel -> camera point (mm). Depth aligned to colour; a percentile below 50 favours the nearest surface
    (small objects in front of the background)."""
    h, w = depth_mm.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None
    patch = depth_mm[max(0, vi - radius):vi + radius + 1, max(0, ui - radius):ui + radius + 1]
    valid = patch[(patch >= dmin) & (patch <= dmax)]
    if valid.size < max(3, 0.2 * patch.size):
        return None
    z = float(np.percentile(valid, percentile))
    sx, sy = w / intr.width, h / intr.height
    return geo.deproject(u, v, z, intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy)


def head_point_mm(depth_mm: np.ndarray, face_px: np.ndarray, intr, dmin: float, dmax: float) -> Optional[np.ndarray]:
    """Face surface point (nose bridge pixel at the robust face depth), camera frame, mm."""
    z, _ = geo.robust_face_depth_mm(depth_mm, face_px[list(HEAD_IDX)], 6, dmin, dmax, 0.2, 80.0, 2)
    if z is None:
        return None
    h, w = depth_mm.shape[:2]
    u, v = face_px[168]
    sx, sy = w / intr.width, h / intr.height
    return geo.deproject(float(u), float(v), z, intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy)


def palm_point_mm(depth_mm: np.ndarray, hand_px: np.ndarray, intr, dmin: float, dmax: float) -> Optional[np.ndarray]:
    """Palm centre: median of the wrist and finger-base points that have depth, camera frame, mm."""
    pts = [pixel_point_mm(depth_mm, hand_px[i][0], hand_px[i][1], intr, 4, dmin, dmax) for i in PALM_IDX]
    pts = [p for p in pts if p is not None]
    return np.median(np.array(pts), axis=0) if len(pts) >= 2 else None


# Red of the robot tool ring. Measured on 2026-10-07: ring hue -2..+1, saturation 170-212, LAB a* 46; skin, wood and
# table hue +7..+10, saturation 121-168, a* 13-20. Hue alone is not enough: both limits below must hold.
RED_HUE_MAX = 4              # |hue - 0| in OpenCV units (0..180)
RED_SAT_MIN = 150
RED_VAL_MIN = 60
RED_A_MIN = 30               # LAB a* (green -128 .. red +127)


def red_mask(bgr: np.ndarray) -> np.ndarray:
    """Pixels of the robot's red ring (uint8 0/255), small specks removed."""
    import cv2
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    a = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)[..., 1].astype(np.int16) - 128
    m = (((h <= RED_HUE_MAX) | (h >= 180 - RED_HUE_MAX)) & (s >= RED_SAT_MIN) & (v >= RED_VAL_MIN) & (a >= RED_A_MIN))
    m = m.astype(np.uint8) * 255
    return cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


# MediaPipe Pose (33 points): head = nose, eyes, ears; hands = wrist, index and pinky bases
POSE_HEAD_IDX = (0, 2, 5, 7, 8)
POSE_HAND_IDX = {"left": (15, 17, 19), "right": (16, 18, 20)}


def _pose_points_mm(depth_mm, body_px, idx, intr, dmin, dmax, min_vis) -> Optional[np.ndarray]:
    pts = [pixel_point_mm(depth_mm, body_px[i, 0], body_px[i, 1], intr, 5, dmin, dmax, percentile=30)
           for i in idx if body_px[i, 2] >= min_vis]
    pts = [p for p in pts if p is not None]
    return np.median(np.array(pts), axis=0) if pts else None


def pose_head_point_mm(depth_mm, body_px, intr, dmin, dmax, min_vis=0.5) -> Optional[np.ndarray]:
    """Head from the body-pose model (works at a distance and with the face turned away; the face model needs a
    large, frontal face). body_px: (33, 3) pixel x, y, visibility. Camera frame, mm."""
    if body_px is None or len(body_px) < 33:
        return None
    return _pose_points_mm(depth_mm, body_px, POSE_HEAD_IDX, intr, dmin, dmax, min_vis)


def pose_hand_points_mm(depth_mm, body_px, intr, dmin, dmax, min_vis=0.5) -> dict:
    """{'left'/'right': hand point (wrist + finger bases of the pose model) or None}. Camera frame, mm."""
    if body_px is None or len(body_px) < 33:
        return {}
    return {side: _pose_points_mm(depth_mm, body_px, idx, intr, dmin, dmax, min_vis)
            for side, idx in POSE_HAND_IDX.items()}


def put_text(img, text: str, xy, col=(255, 255, 255), scale: float = 0.55, thick: int = 1) -> None:
    """Text on a dark box. (An outline drawn as thick black text does not line up with the thin text in OpenCV 5.)"""
    import cv2
    (w, h), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thick)
    x, y = int(xy[0]), int(xy[1])
    cv2.rectangle(img, (x - 3, y - h - 4), (x + w + 3, y + base + 2), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, thick, cv2.LINE_AA)


def project(p_cam_mm: np.ndarray, intr) -> Optional[Tuple[int, int]]:
    if p_cam_mm[2] <= 1:
        return None
    return (int(round(intr.fx * p_cam_mm[0] / p_cam_mm[2] + intr.cx)),
            int(round(intr.fy * p_cam_mm[1] / p_cam_mm[2] + intr.cy)))


# ---------------------------------------------------------------------------------------------------------------
# Robot through the Jetson gateway relay
# ---------------------------------------------------------------------------------------------------------------
def connect_robot(enable_motion: bool):
    """XArmAPI through the gateway relay (saved `lite6 login`). enable_motion=False: read-only use."""
    sys.path.insert(0, str(ROBOT_CODE_DIR))
    import lite6
    from xarm.wrapper import XArmAPI
    try:
        gw = lite6.connect(quiet=True)
        r = gw.relay_allow()
    except lite6.Lite6Error as e:
        raise RuntimeError("%s (log in once: python robot_code/lite6.py login 10.13.37.101)" % e)
    log.info("relay open until %s", dt.datetime.fromtimestamp(r.get("until", 0)).strftime("%H:%M:%S"))
    arm = XArmAPI(gw.host, is_radian=False)
    if not arm.connected:
        raise RuntimeError("could not connect to the arm through %s" % gw.host)
    if enable_motion:
        code, (err, warn) = arm.get_err_warn_code()
        if err or warn:
            log.warning("controller error %s / warning %s: clearing it", err, warn)
            arm.clean_warn()
            arm.clean_error()
        arm.motion_enable(enable=True)
        arm.set_mode(0)
        arm.set_state(0)
    return arm


def tool_pose(arm) -> Optional[List[float]]:
    """[x, y, z, roll, pitch, yaw] (mm, degrees) or None."""
    code, pose = arm.get_position(is_radian=False)
    return [float(v) for v in pose[:6]] if code == 0 else None


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def in_safety_box(p: Sequence[float], box=(-500, 500, -500, 500, 0, 700), margin: float = 20.0) -> bool:
    x, y, z = p[:3]
    return (box[0] + margin <= x <= box[1] - margin and box[2] + margin <= y <= box[3] - margin
            and box[4] + margin <= z <= box[5] - margin)


# ---------------------------------------------------------------------------------------------------------------
# Lite 6 link points, workstation frame on the robot base, robot pose reader
# ---------------------------------------------------------------------------------------------------------------
# Modified DH (alpha_{i-1} [deg], a_{i-1} [mm], d_i [mm], theta offset [deg]); checked on 2026-10-07 against the
# controller's own forward kinematics: tool point within 4 mm (the controller adds its factory calibration).
LITE6_MDH = ((0, 0, 243.3, 0), (-90, 0, 0, -90), (180, 200, 0, -90), (90, 87, 227.6, 0), (90, 0, 0, 0), (-90, 0, 61.5, 0))


def lite6_points_mm(q_deg: Sequence[float]) -> Dict[str, np.ndarray]:
    """Robot frame points [mm] of the Lite 6 for joint angles [deg]: base, shoulder (J2 axis), elbow (J3 axis),
    wrist (J5 centre), flange (tool point without a TCP offset)."""
    T = np.eye(4)
    org = []
    for (al, a, d, off), qi in zip(LITE6_MDH, q_deg[:6]):
        ca, sa = np.cos(np.radians(al)), np.sin(np.radians(al))
        th = np.radians(qi + off)
        ct, st = np.cos(th), np.sin(th)
        T = T @ np.array([[ct, -st, 0, a], [st * ca, ct * ca, -sa, -sa * d], [st * sa, ct * sa, ca, ca * d], [0, 0, 0, 1]])
        org.append(T[:3, 3].copy())
    return {"base": np.zeros(3), "shoulder": org[1], "elbow": org[2], "wrist": org[4], "flange": org[5]}


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def workspace_frame_from_robot(rframe: "RobotFrame", towards_worker_robot_mm: Sequence[float]):
    """Workstation frame W (metres) on the REAL robot: origin = robot base, z = robot z (up), x = horizontal direction
    from the base to the given point (the assembly area / the worker), y = z x x. W is the robot frame turned about
    its vertical axis:  p_W = R_w_robot @ p_robot_mm / 1000.
    Returns (R_wc, origin_cam_m, R_w_robot, theta_deg) with  p_W = R_wc @ (p_cam_m - origin_cam_m)."""
    theta = float(np.arctan2(towards_worker_robot_mm[1], towards_worker_robot_mm[0]))
    R_w_robot = rot_z(-theta)
    R_wc = R_w_robot @ rframe.R
    origin_cam_m = -(rframe.R.T @ rframe.t) / 1000.0           # robot base seen from the camera
    return R_wc, origin_cam_m, R_w_robot, float(np.degrees(theta))


class RobotPoller:
    """Reads the real robot's joints and tool position in a background thread (READ ONLY: no motion, no state change)
    and keeps the newest link points in the robot frame [mm]. Reconnects on errors; never raises into the caller."""

    def __init__(self, hz: float = 10.0, full_pose: bool = True) -> None:
        """full_pose=False (live program): only the joints are read (one request per poll, half the relay traffic);
        the tool point is the flange from the kinematics and 'pose' has no orientation. True: tool pose incl. RPY
        (calibration tools that need the red tip)."""
        self.period = 1.0 / max(hz, 1.0)
        self.full_pose = full_pose
        self.read_ms = float('nan')
        self._lock = threading.Lock()
        self._last: Optional[Dict[str, Any]] = None
        self.status = "starting"
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="robot-poller", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        arm, prev = None, None
        while not self._stop.is_set():
            try:
                if arm is None:
                    self.status = "connecting"
                    arm = connect_robot(enable_motion=False)
                    self.status = "connected"
                t0 = time.monotonic()
                cq, q = arm.get_servo_angle(is_radian=False)
                cp, pose = arm.get_position(is_radian=False) if self.full_pose else (0, None)
                if cq not in (0, 1, 2) or cp not in (0, 1, 2):   # 1 / 2: controller error / warning flag, data valid
                    raise RuntimeError("robot read failed (codes %s, %s)" % (cq, cp))
                pts = lite6_points_mm(q)
                if pose is None:                                 # joints only: tool point = flange (kinematics; the
                    pose = list(pts["flange"]) + [0.0, 0.0, 0.0]    # controller's TCP is ~3 mm from it), no orientation
                pts["tool"] = np.array(pose[:3], dtype=float)
                t = 0.5 * (t0 + time.monotonic())
                speed, jspeed = 0.0, 0.0
                if prev is not None and t > prev[0]:
                    speed = float(np.linalg.norm(pts["tool"] - prev[1]) / (t - prev[0]) / 1000.0)   # m/s
                    jspeed = float(np.max(np.abs(np.asarray(q[:6], float) - prev[2])) / (t - prev[0]))   # deg/s
                prev = (t, pts["tool"], np.asarray(q[:6], float))
                with self._lock:
                    self._last = {"t": t, "pts": pts, "speed_mps": speed, "jspeed_dps": jspeed,
                                  "joints": [float(v) for v in q[:6]], "pose": [float(v) for v in pose[:6]]}
                self.status = "ok" if cq == 0 and cp == 0 else "ok (controller %s flag)" % ("error" if 1 in (cq, cp) else "warning")
                self.read_ms = 1000.0 * (time.monotonic() - t0)
            except Exception as exc:                            # noqa: BLE001  (network, relay, SDK)
                self.status = "error: %s" % exc
                log.warning("robot pose reader: %s (retrying)", exc)
                prev = None
                if arm is not None:                             # quick try first: the SDK reconnects its sockets
                    try:                                        # (the relay access is still valid)
                        arm.connect()
                        if arm.connected and arm.get_servo_angle(is_radian=False)[0] in (0, 1, 2):
                            log.warning("robot pose reader: reconnected (quick)")
                            continue
                    except Exception:                           # noqa: BLE001
                        pass
                    try:
                        arm.disconnect()
                    except Exception:                           # noqa: BLE001
                        pass
                arm = None                                      # then a full reconnect (relay + SDK) after 1 s
                self._stop.wait(1.0)
                continue
            self._stop.wait(max(0.0, self.period - (time.monotonic() - t0)))
        try:
            if arm is not None:
                arm.disconnect()
        except Exception:                                       # noqa: BLE001
            pass

    def latest(self) -> Optional[Dict[str, Any]]:
        """{'t' (time.monotonic), 'pts' {base, shoulder, elbow, wrist, flange, tool} [mm, robot frame], 'speed_mps',
        'joints' [deg], 'pose' [x, y, z mm, roll, pitch, yaw deg]} or None."""
        with self._lock:
            return self._last

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)
