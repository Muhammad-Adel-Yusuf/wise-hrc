"""Geometry helpers: EAR, MAR, head pose (solvePnP), depth lookup, deprojection."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np

# MediaPipe 478-point face mesh indices ------------------------------------
# EAR order: p1 outer corner, p2/p3 upper lid, p4 inner corner, p5/p6 lower lid
RIGHT_EYE = (33, 160, 158, 133, 153, 144)      # subject's right eye (image left)
LEFT_EYE = (362, 385, 387, 263, 373, 380)      # subject's left eye (image right)
# Inner-lip landmarks for MAR
MOUTH_LEFT, MOUTH_RIGHT = 308, 78
MOUTH_VERTICAL_PAIRS = ((82, 87), (13, 14), (312, 317))
FACE_LEFT, FACE_RIGHT, FOREHEAD, CHIN, NOSE_TIP = 454, 234, 10, 152, 1
# solvePnP correspondences: nose, chin, eye outer corners, mouth corners
PNP_INDICES = (1, 152, 263, 33, 291, 61)
# Generic face model in mm. Frame: x right, y down, z away from the camera,
# origin at the nose tip, so a frontal face has R = identity.
PNP_MODEL_POINTS = np.array([
    (0.0, 0.0, 0.0),          # nose tip
    (0.0, 71.7, 14.0),        # chin
    (48.9, -37.0, 29.3),      # left-eye outer corner (image right)
    (-48.9, -37.0, 29.3),     # right-eye outer corner (image left)
    (32.6, 32.6, 27.2),       # left mouth corner
    (-32.6, 32.6, 27.2),      # right mouth corner
], dtype=np.float64)


def eye_aspect_ratio(p: np.ndarray) -> float:
    """EAR = (|p2-p6| + |p3-p5|) / (2 |p1-p4|). p has shape (6, 2)."""
    horiz = np.linalg.norm(p[0] - p[3])
    if horiz < 1e-6:
        return float("nan")
    return float((np.linalg.norm(p[1] - p[5]) + np.linalg.norm(p[2] - p[4])) / (2.0 * horiz))


def mouth_aspect_ratio(face_px: np.ndarray) -> float:
    """MAR = mean vertical inner-lip opening / mouth width."""
    width = np.linalg.norm(face_px[MOUTH_LEFT] - face_px[MOUTH_RIGHT])
    if width < 1e-6:
        return float("nan")
    vert = np.mean([np.linalg.norm(face_px[a] - face_px[b]) for a, b in MOUTH_VERTICAL_PAIRS])
    return float(vert / width)


def face_width_px(face_px: np.ndarray, height_floor: float = 0.55) -> float:
    """Face width, floored by a fraction of face height (guards yaw shrinkage)."""
    w = float(np.linalg.norm(face_px[FACE_LEFT] - face_px[FACE_RIGHT]))
    h = float(np.linalg.norm(face_px[FOREHEAD] - face_px[CHIN]))
    return max(w, height_floor * h)


def eye_centers(face_px: np.ndarray) -> np.ndarray:
    """Centres of the two eyes (mean of the six EAR points), shape (2, 2)."""
    return np.array([face_px[list(RIGHT_EYE)].mean(axis=0), face_px[list(LEFT_EYE)].mean(axis=0)])


def angles_from_rotation(R: np.ndarray) -> Tuple[float, float, float]:
    """(yaw, pitch, roll) in degrees from a camera<-face rotation matrix.

    yaw   > 0: face turned toward image right;  pitch > 0: looking up.
    Derived from the face forward vector f = -R[:, 2] (model z points away).
    """
    f = -R[:, 2]
    yaw = math.degrees(math.atan2(f[0], -f[2]))
    pitch = math.degrees(math.atan2(-f[1], math.hypot(f[0], f[2])))
    up = R[:, 1] * -1.0                     # face 'up' axis (model y points down)
    roll = math.degrees(math.atan2(up[0], -up[1]))
    return yaw, pitch, roll


@dataclass
class HeadPose:
    yaw: float
    pitch: float
    roll: float
    tvec_mm: np.ndarray
    reproj_error_px: float


class HeadPoseEstimator:
    """solvePnP head pose with temporal initialisation."""

    def __init__(self, fx: float, fy: float, cx: float, cy: float, max_reproj_px: float = 8.0) -> None:
        import cv2  # lazy: keeps pure-logic tests independent of OpenCV
        self._cv2 = cv2
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.max_reproj_px = max_reproj_px
        self._rvec: Optional[np.ndarray] = None
        self._tvec: Optional[np.ndarray] = None

    def estimate(self, face_px: np.ndarray) -> Optional[HeadPose]:
        cv2 = self._cv2
        img = np.ascontiguousarray(face_px[list(PNP_INDICES)], dtype=np.float64)
        try:
            if self._rvec is not None:
                ok, rvec, tvec = cv2.solvePnP(PNP_MODEL_POINTS, img, self.K, None,
                                              self._rvec.copy(), self._tvec.copy(), True,
                                              cv2.SOLVEPNP_ITERATIVE)
            else:
                ok, rvec, tvec = cv2.solvePnP(PNP_MODEL_POINTS, img, self.K, None,
                                              flags=cv2.SOLVEPNP_ITERATIVE)
        except cv2.error:
            ok = False
        if not ok or tvec[2, 0] <= 0:
            self._rvec = self._tvec = None
            return None
        proj, _ = cv2.projectPoints(PNP_MODEL_POINTS, rvec, tvec, self.K, None)
        err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img, axis=1)))
        if err > self.max_reproj_px:
            self._rvec = self._tvec = None
            return None
        self._rvec, self._tvec = rvec, tvec
        R, _ = cv2.Rodrigues(rvec)
        yaw, pitch, roll = angles_from_rotation(R)
        return HeadPose(yaw, pitch, roll, tvec.reshape(3), err)


# Depth ---------------------------------------------------------------------
def median_depth_mm(depth_mm: np.ndarray, u: float, v: float, radius: int,
                    min_mm: float, max_mm: float, min_valid_fraction: float) -> Optional[float]:
    """Median of valid depth values in a (2r+1)^2 window; None if too few valid."""
    h, w = depth_mm.shape[:2]
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None
    patch = depth_mm[max(0, vi - radius):vi + radius + 1, max(0, ui - radius):ui + radius + 1]
    valid = patch[(patch >= min_mm) & (patch <= max_mm)]
    if patch.size == 0 or valid.size < min_valid_fraction * patch.size:
        return None
    return float(np.median(valid))


DEPTH_POINTS = (1, 168, 10, 50, 280, 152)      # nose tip, nose bridge, forehead, both cheeks, chin


def robust_face_depth_mm(depth_mm: np.ndarray, face_uv: np.ndarray, radius: int, min_mm: float,
                         max_mm: float, min_valid_fraction: float, max_spread_mm: float,
                         min_points: int) -> Tuple[Optional[float], int]:
    """Median depth over several face points (each a windowed median), outliers rejected.

    face_uv: (N, 2) pixel coords in the depth image. Returns (depth_mm | None, n_points_used).
    A single point (e.g. the nose tip) often falls on a depth hole; several points rarely all do.
    """
    zs = []
    for u, v in face_uv:
        z = median_depth_mm(depth_mm, float(u), float(v), radius, min_mm, max_mm, min_valid_fraction)
        if z is not None:
            zs.append(z)
    if len(zs) < min_points:
        return None, len(zs)
    med = float(np.median(zs))
    keep = [z for z in zs if abs(z - med) <= max_spread_mm]      # drop background / hair / edge hits
    if len(keep) < min_points:
        return None, len(keep)
    return float(np.median(keep)), len(keep)


def deproject(u: float, v: float, z_mm: float, fx: float, fy: float, cx: float, cy: float) -> np.ndarray:
    """Pinhole deprojection to camera coordinates (mm): x right, y down, z forward."""
    return np.array([(u - cx) * z_mm / fx, (v - cy) * z_mm / fy, z_mm])


# MediaPipe Pose (33 points) indices used for body presence / orientation --------------
POSE_NOSE, POSE_L_EAR, POSE_R_EAR, POSE_L_SHOULDER, POSE_R_SHOULDER = 0, 7, 8, 11, 12
POSE_DRAW = (0, 2, 5, 7, 8, 11, 12)


@dataclass
class BodyInfo:
    present: bool
    facing: str                    # front | side | back | none
    turn: Optional[float] = None   # (nose x - shoulder centre x) / shoulder width, image x (None if unknown)


def body_orientation(body: Optional[np.ndarray], cfg) -> BodyInfo:
    """Coarse body/head orientation from Pose landmarks: rows (x_px, y_px, visibility).

    present: >= 2 of nose/ears/shoulders visible.  back: nose not visible (back of the head to the camera).
    side: ears very asymmetric or nose far from the shoulder centre.  front: otherwise.
    Only used to decide 'present but turned away'; it gives NO yaw/pitch.
    """
    if body is None or len(body) < 13:
        return BodyInfo(False, "none")
    v = body[:, 2]
    mv = cfg["min_visibility"]
    pts = (POSE_NOSE, POSE_L_EAR, POSE_R_EAR, POSE_L_SHOULDER, POSE_R_SHOULDER)
    if sum(1 for i in pts if v[i] >= mv) < 2:
        return BodyInfo(False, "none")
    turn = None
    if v[POSE_L_SHOULDER] >= mv and v[POSE_R_SHOULDER] >= mv and v[POSE_NOSE] >= mv:
        sw = abs(body[POSE_L_SHOULDER, 0] - body[POSE_R_SHOULDER, 0])
        if sw > 1e-6:
            turn = float((body[POSE_NOSE, 0] - 0.5 * (body[POSE_L_SHOULDER, 0] + body[POSE_R_SHOULDER, 0])) / sw)
    if v[POSE_NOSE] < cfg["nose_visible_min"]:
        return BodyInfo(True, "back", turn)
    if abs(v[POSE_L_EAR] - v[POSE_R_EAR]) >= cfg["ear_asymmetry"] or (turn is not None and abs(turn) >= cfg["turn_ratio"]):
        return BodyInfo(True, "side", turn)
    return BodyInfo(True, "front", turn)
