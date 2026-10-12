"""MediaPipe Holistic Landmarker wrapper + landmark -> Observation features."""
from __future__ import annotations

import logging
import time
import urllib.request
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from . import geometry as geo
from .monitor import Observation

log = logging.getLogger(__name__)


def ensure_model(path: str, url: str, auto_download: bool = True) -> str:
    """Return a usable local model path, downloading it once if allowed."""
    p = Path(path)
    if p.exists() and p.stat().st_size > 1_000_000:
        return str(p)
    if not auto_download:
        raise FileNotFoundError(f"Model not found at {p}. Download it from {url} or enable auto_download.")
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(p.suffix + ".part")
    log.info("Downloading model %s -> %s", url, p)
    try:
        with urllib.request.urlopen(url, timeout=60) as resp, open(tmp, "wb") as fh:
            while True:
                chunk = resp.read(1 << 20)
                if not chunk:
                    break
                fh.write(chunk)
        if tmp.stat().st_size < 1_000_000:
            raise IOError("downloaded file is suspiciously small")
        tmp.replace(p)
    except Exception as exc:                                    # noqa: BLE001
        tmp.unlink(missing_ok=True)
        raise RuntimeError(f"Model download failed ({exc}). Download manually from {url} "
                           f"and set vision.model_path.") from exc
    return str(p)


def _body_array(lms: Any, w: int, h: int) -> np.ndarray:
    """Pose landmarks -> (N, 3) array of (x_px, y_px, visibility). Empty (0, 3) if no person."""
    if lms is None or len(lms) == 0:
        return np.empty((0, 3))
    if isinstance(lms[0], (list, tuple)):
        lms = lms[0]
    rows = []
    for l in lms:
        vis = getattr(l, "visibility", None)
        pres = getattr(l, "presence", None)
        v = min(float(vis) if vis is not None else 1.0, float(pres) if pres is not None else 1.0)
        rows.append((l.x * w, l.y * h, v))
    return np.array(rows, dtype=np.float64)


def _world_array(lms: Any) -> np.ndarray:
    """Pose WORLD landmarks -> (N, 4) array (x, y, z in metres, hip-centred; visibility). Empty (0, 4) if no person.
    Used only by the physical-fatigue algorithm (Algorithm 4)."""
    if lms is None or len(lms) == 0:
        return np.empty((0, 4))
    if isinstance(lms[0], (list, tuple)):
        lms = lms[0]
    rows = []
    for l in lms:
        vis = getattr(l, "visibility", None)
        rows.append((l.x, l.y, l.z, float(vis) if vis is not None else 1.0))
    return np.array(rows, dtype=np.float64)


def _lm_array(lms: Any, w: int, h: int) -> Optional[np.ndarray]:
    """Convert a MediaPipe landmark list (flat or nested) to an (N, 2) pixel array."""
    if lms is None or len(lms) == 0:
        return None
    if isinstance(lms[0], (list, tuple)):          # tolerate list-of-lists API variants
        lms = lms[0]
    return np.array([(l.x * w, l.y * h) for l in lms], dtype=np.float64)


class HolisticVision:
    """Runs the Holistic Landmarker (VIDEO mode) on BGR frames."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        self._mp = mp
        v = cfg["vision"]
        model = ensure_model(v["model_path"], v["model_url"], v["auto_download"])
        kwargs = dict(
            base_options=mp_python.BaseOptions(model_asset_path=model),
            running_mode=mp_vision.RunningMode.VIDEO,
            min_face_detection_confidence=v["min_face_detection_confidence"],
            min_face_landmarks_confidence=v["min_face_landmarks_confidence"],
            min_pose_detection_confidence=v["min_pose_detection_confidence"],
            min_pose_landmarks_confidence=v["min_pose_landmarks_confidence"],
            min_hand_landmarks_confidence=v["min_hand_landmarks_confidence"],
        )
        try:
            options = mp_vision.HolisticLandmarkerOptions(**kwargs)
        except TypeError:       # older/newer builds may lack some option names
            options = mp_vision.HolisticLandmarkerOptions(
                base_options=kwargs["base_options"], running_mode=kwargs["running_mode"])
            log.warning("HolisticLandmarkerOptions confidence options unsupported; using defaults")
        self._landmarker = mp_vision.HolisticLandmarker.create_from_options(options)
        self._last_ts = -1
        self.last_body: Optional[np.ndarray] = None
        self.last_body_world: Optional[np.ndarray] = None   # (N, 4) metres + visibility; None = pose not run; (0, 4) = no person

    def detect(self, bgr: np.ndarray, t: float) -> Tuple[Optional[np.ndarray], List[np.ndarray]]:
        """Return (face landmarks px or None, [hand landmark px arrays])."""
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        ts = max(int(t * 1000), self._last_ts + 1)             # must be strictly increasing
        self._last_ts = ts
        res = self._landmarker.detect_for_video(image, ts)
        face = _lm_array(res.face_landmarks, w, h)
        if face is not None and len(face) < 468:
            face = None
        hands = [a for a in (_lm_array(res.left_hand_landmarks, w, h),
                             _lm_array(res.right_hand_landmarks, w, h)) if a is not None]
        self.last_body = _body_array(getattr(res, "pose_landmarks", None), w, h)
        self.last_body_world = _world_array(getattr(res, "pose_world_landmarks", None))
        return face, hands

    def close(self) -> None:
        try:
            self._landmarker.close()
        except Exception:                                       # noqa: BLE001
            pass


class FaceHandVision:
    """Face Landmarker + Hand Landmarker (VIDEO mode). Same interface as HolisticVision.

    Uses two mature single-purpose graphs instead of the Holistic graph, which can hit a
    native CHECK failure (process abort) that Python cannot catch.
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        import mediapipe as mp
        from mediapipe.tasks import python as mp_python
        from mediapipe.tasks.python import vision as mp_vision
        self._mp = mp
        v = cfg["vision"]
        face_model = ensure_model(v["face_model_path"], v["face_model_url"], v["auto_download"])
        hand_model = ensure_model(v["hand_model_path"], v["hand_model_url"], v["auto_download"])
        mode = mp_vision.RunningMode.VIDEO
        self._face = mp_vision.FaceLandmarker.create_from_options(mp_vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=face_model), running_mode=mode, num_faces=1,
            min_face_detection_confidence=v["min_face_detection_confidence"],
            min_face_presence_confidence=v["min_face_landmarks_confidence"],
            min_tracking_confidence=v["min_face_landmarks_confidence"],
            output_face_blendshapes=False, output_facial_transformation_matrixes=False))
        self._hand = mp_vision.HandLandmarker.create_from_options(mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=hand_model), running_mode=mode, num_hands=2,
            min_hand_detection_confidence=v["min_hand_landmarks_confidence"],
            min_hand_presence_confidence=v["min_hand_landmarks_confidence"],
            min_tracking_confidence=v["min_hand_landmarks_confidence"]))
        self._last_ts = -1
        # Optional Pose Landmarker (body presence when the face is lost). Never fatal: if the model cannot
        # be loaded or keeps failing, the system runs exactly as before (face lost -> Unknown).
        self._bp = cfg["body_presence"]
        self._pose = None
        self._pose_fails = 0
        self.last_body: Optional[np.ndarray] = None     # None = pose not run on this frame
        self.last_body_world: Optional[np.ndarray] = None   # world landmarks (metres) for the physical algorithm; None = pose not run
        self.force_pose = False                          # True (posture calibration): run the pose model on every frame
        self.pose_only = False                           # True (physical-fatigue program): skip the face and hand models
        if self._bp["enabled"] or cfg.get("physical", {}).get("enabled", False):   # the physical algorithm needs the pose model too
            try:
                pose_model = ensure_model(v["pose_model_path"], v["pose_model_url"], v["auto_download"])
                self._pose = mp_vision.PoseLandmarker.create_from_options(mp_vision.PoseLandmarkerOptions(
                    base_options=mp_python.BaseOptions(model_asset_path=pose_model), running_mode=mode, num_poses=1,
                    min_pose_detection_confidence=v["min_pose_detection_confidence"],
                    min_pose_presence_confidence=v["min_pose_landmarks_confidence"],
                    min_tracking_confidence=v["min_pose_landmarks_confidence"],
                    output_segmentation_masks=False))
                log.info("Body-presence (pose) model loaded")
            except Exception as exc:                            # noqa: BLE001
                log.warning("Body-presence disabled: pose model unavailable (%s)", exc)
                self._pose = None

    @property
    def pose_available(self) -> bool:
        return self._pose is not None

    def detect(self, bgr: np.ndarray, t: float) -> Tuple[Optional[np.ndarray], List[np.ndarray]]:
        import cv2
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h, w = rgb.shape[:2]
        image = self._mp.Image(image_format=self._mp.ImageFormat.SRGB, data=np.ascontiguousarray(rgb))
        ts = max(int(t * 1000), self._last_ts + 1)             # must be strictly increasing
        self._last_ts = ts
        if self.pose_only:
            face, hands = None, []
        else:
            fres = self._face.detect_for_video(image, ts)
            face = _lm_array(fres.face_landmarks[0], w, h) if fres.face_landmarks else None
            if face is not None and len(face) < 468:
                face = None
            hres = self._hand.detect_for_video(image, ts)
            hands = [a for a in (_lm_array(hl, w, h) for hl in (hres.hand_landmarks or [])) if a is not None]
        self.last_body = None
        self.last_body_world = None
        if self._pose is not None and (face is None or self.force_pose
                                   or not self._bp["run_only_when_face_lost"]):
            try:
                pres = self._pose.detect_for_video(image, ts)
                self.last_body = _body_array(pres.pose_landmarks[0] if pres.pose_landmarks else None, w, h)
                self.last_body_world = _world_array(pres.pose_world_landmarks[0] if getattr(pres, "pose_world_landmarks", None) else None)
                self._pose_fails = 0
            except Exception as exc:                            # noqa: BLE001
                self._pose_fails += 1
                log.warning("Pose inference failed (%d): %s", self._pose_fails, exc)
                if self._pose_fails >= 30:
                    log.warning("Body-presence disabled after repeated pose failures")
                    self._close_pose()
        return face, hands

    def _close_pose(self) -> None:
        try:
            if self._pose is not None:
                self._pose.close()
        except Exception:                                       # noqa: BLE001
            pass
        self._pose = None

    def close(self) -> None:
        for lm in (getattr(self, "_face", None), getattr(self, "_hand", None), getattr(self, "_pose", None)):
            try:
                if lm is not None:
                    lm.close()
            except Exception:                                   # noqa: BLE001
                pass


def create_vision(cfg: Dict[str, Any]):
    """Factory: vision.backend = face_hand (default) | holistic."""
    return HolisticVision(cfg) if cfg["vision"]["backend"] == "holistic" else FaceHandVision(cfg)


class FeatureExtractor:
    """Landmarks -> Observation (EAR, MAR, head pose, hand geometry) with validity flags."""

    def __init__(self, cfg: Dict[str, Any], fx: float, fy: float, cx: float, cy: float) -> None:
        self.cfg = cfg
        self.pose = geo.HeadPoseEstimator(fx, fy, cx, cy, cfg["vision"]["max_reprojection_error_px"])
        self.rub_floor = cfg["rub"]["face_width_height_floor"]

    def extract(self, t: float, face_px: Optional[np.ndarray], hands_px: List[np.ndarray],
                image_size: Tuple[int, int], body: Optional[np.ndarray] = None) -> Observation:
        """body: pose landmarks (x_px, y_px, visibility) if the pose model ran on this frame, else None."""
        obs = Observation(t=t)
        obs.hands_px = hands_px
        if body is not None:
            bi = geo.body_orientation(body, self.cfg["body_presence"])
            obs.body_checked, obs.body_present, obs.body_facing, obs.body_turn = True, bi.present, bi.facing, bi.turn
            obs.body_px = body if len(body) else None
            obs.hands_px = hands_px
        if face_px is None:
            return obs
        v = self.cfg["vision"]
        w, h = image_size
        # plausibility: landmarks must lie (mostly) inside the image
        inside = np.mean((face_px[:, 0] > -0.05 * w) & (face_px[:, 0] < 1.05 * w)
                         & (face_px[:, 1] > -0.05 * h) & (face_px[:, 1] < 1.05 * h))
        if inside < 0.95:
            return obs
        obs.face_ok = True
        hp = self.pose.estimate(face_px)
        if hp is not None:
            obs.pose_ok = True
            obs.yaw, obs.pitch = hp.yaw, hp.pitch
        ear_r = geo.eye_aspect_ratio(face_px[list(geo.RIGHT_EYE)])
        ear_l = geo.eye_aspect_ratio(face_px[list(geo.LEFT_EYE)])
        ear = float(np.nanmean([ear_r, ear_l])) if not (np.isnan(ear_r) and np.isnan(ear_l)) else None
        lo, hi = v["ear_valid_range"]
        pmin, pmax = v["eye_pitch_range_deg"]
        if ear is not None and lo <= ear <= hi and hp is not None \
                and abs(hp.yaw) <= v["max_eye_yaw_deg"] and pmin <= hp.pitch <= pmax:
            obs.eyes_ok, obs.ear = True, ear
        elif ear is not None:
            obs.ear = ear                                       # kept for display only
        mar = geo.mouth_aspect_ratio(face_px)
        mlo, mhi = v["mar_valid_range"]
        if np.isfinite(mar) and mlo <= mar <= mhi and hp is not None and abs(hp.yaw) <= v["max_mouth_yaw_deg"]:
            obs.mouth_ok, obs.mar = True, mar
        obs.eye_centers_px = geo.eye_centers(face_px)
        obs.face_width_px = geo.face_width_px(face_px, self.rub_floor)
        obs.hands_px = hands_px
        return obs
