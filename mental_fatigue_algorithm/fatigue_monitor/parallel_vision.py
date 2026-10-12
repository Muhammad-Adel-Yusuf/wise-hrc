"""Asynchronous landmark models: face, hand and pose each run in their OWN PROCESS, continuously, on the newest frame.

MediaPipe's Python binding keeps the GIL during inference, so threads cannot run the models at the same time, and running
them one after another (face 20 + hand 50 + pose 30 ms) limits the camera program to ~10 fps. Here:
  * main loop  : detect(bgr, t) puts the frame into shared memory and returns AT ONCE the newest result of each model;
  * each model : takes the newest frame whenever it is free (it never queues old frames), so the loop runs at the camera
                 rate and every model updates as fast as the CPU allows. The face model is capped at face_max_hz (the
                 fatigue features need ~10 Hz) to leave CPU time to the hand and pose models that the safety geometry uses.
Same interface as vision.FaceHandVision: face_px, hands_px = detect(bgr, t); last_body, last_body_world; close().
rates() gives the measured update rate of each model [Hz]. Landmarks are in pixels of the frame given to detect().
"""
from __future__ import annotations

import logging
import multiprocessing as mpc
import queue
import time
from multiprocessing import shared_memory
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

log = logging.getLogger(__name__)
KINDS = ("face", "hand", "pose")
MAX_W, MAX_H = 1920, 1080


def _make_model(kind: str, cfg: Dict[str, Any]):
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    from .vision import ensure_model
    v = cfg["vision"]
    mode = mp_vision.RunningMode.VIDEO
    if kind == "face":
        return mp_vision.FaceLandmarker.create_from_options(mp_vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=ensure_model(v["face_model_path"], v["face_model_url"], v["auto_download"])),
            running_mode=mode, num_faces=1, min_face_detection_confidence=v["min_face_detection_confidence"],
            min_face_presence_confidence=v["min_face_landmarks_confidence"],
            min_tracking_confidence=v["min_face_landmarks_confidence"],
            output_face_blendshapes=False, output_facial_transformation_matrixes=False))
    if kind == "hand":
        return mp_vision.HandLandmarker.create_from_options(mp_vision.HandLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=ensure_model(v["hand_model_path"], v["hand_model_url"], v["auto_download"])),
            running_mode=mode, num_hands=2, min_hand_detection_confidence=v["min_hand_landmarks_confidence"],
            min_hand_presence_confidence=v["min_hand_landmarks_confidence"],
            min_tracking_confidence=v["min_hand_landmarks_confidence"]))
    return mp_vision.PoseLandmarker.create_from_options(mp_vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=ensure_model(v["pose_model_path"], v["pose_model_url"], v["auto_download"])),
        running_mode=mode, num_poses=1, min_pose_detection_confidence=v["min_pose_detection_confidence"],
        min_pose_presence_confidence=v["min_pose_landmarks_confidence"],
        min_tracking_confidence=v["min_pose_landmarks_confidence"], output_segmentation_masks=False))


def _worker(kind: str, cfg: Dict[str, Any], shm_name: str, lock, seq, meta, max_hz: float, stop, results) -> None:
    """One landmark model in its own process: always the NEWEST frame, results to the queue."""
    import mediapipe as mp
    try:                                                         # Windows: BELOW_NORMAL priority, the main loop first
        import ctypes
        ctypes.windll.kernel32.SetPriorityClass(ctypes.windll.kernel32.GetCurrentProcess(), 0x00004000)
    except (AttributeError, OSError):
        pass
    model = _make_model(kind, cfg)
    shm = shared_memory.SharedMemory(name=shm_name)
    buf = np.ndarray((MAX_H, MAX_W, 3), dtype=np.uint8, buffer=shm.buf)
    results.put((kind, "ready", 0, 0.0, None))
    last_seq, last_ts, last_run = 0, -1, 0.0
    period = 1.0 / max_hz if max_hz > 0 else 0.0
    while not stop.is_set():
        if seq.value == last_seq or (period and time.monotonic() - last_run < period):
            time.sleep(0.002)
            continue
        with lock:
            s, h, w, t = seq.value, int(meta[0]), int(meta[1]), float(meta[2])
            rgb = np.ascontiguousarray(buf[:h, :w])
        last_seq, last_run = s, time.monotonic()
        ts = max(int(t * 1000), last_ts + 1)
        last_ts = ts
        try:
            res = model.detect_for_video(mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb), ts)
            if kind == "face":
                out = np.array([(l.x * w, l.y * h) for l in res.face_landmarks[0]]) if res.face_landmarks else None
            elif kind == "hand":
                out = [np.array([(l.x * w, l.y * h) for l in hl]) for hl in (res.hand_landmarks or [])]
            else:
                if res.pose_landmarks:
                    body = np.array([(l.x * w, l.y * h, min(float(l.visibility if l.visibility is not None else 1.0),
                                                              float(l.presence if l.presence is not None else 1.0)))
                                     for l in res.pose_landmarks[0]])
                    wl = res.pose_world_landmarks[0] if getattr(res, "pose_world_landmarks", None) else []
                    world = np.array([(l.x, l.y, l.z, float(l.visibility if l.visibility is not None else 1.0)) for l in wl]) \
                        if wl else np.empty((0, 4))
                else:
                    body, world = np.empty((0, 3)), np.empty((0, 4))
                out = (body, world)
            results.put((kind, s, w, t, out))
        except Exception as exc:                                # noqa: BLE001
            results.put((kind, s, w, t, exc))
    model.close()
    shm.close()


class ParallelFaceHandVision:
    """Face, hand and pose landmarkers in three processes, asynchronous (see the module doc)."""

    def __init__(self, cfg: Dict[str, Any], face_max_hz: Optional[float] = None) -> None:
        ctx = mpc.get_context("spawn")
        v = cfg["vision"]
        face_hz = float(face_max_hz if face_max_hz is not None else v.get("face_max_hz", 10.0))
        self.shm = shared_memory.SharedMemory(create=True, size=MAX_W * MAX_H * 3)
        self.buf = np.ndarray((MAX_H, MAX_W, 3), dtype=np.uint8, buffer=self.shm.buf)
        self.lock = ctx.Lock()
        self.seq = ctx.Value("q", 0, lock=False)
        self.meta = ctx.Array("d", 3, lock=False)               # h, w, t
        self.stop_ev = ctx.Event()
        self.results = ctx.Queue()
        hz = {"face": face_hz, "hand": float(v.get("hand_max_hz", 0.0)), "pose": float(v.get("pose_max_hz", 0.0))}  # 0 = free
        self.procs = [ctx.Process(target=_worker, name="vision-" + k, daemon=True,
                                  args=(k, cfg, self.shm.name, self.lock, self.seq, self.meta, hz[k], self.stop_ev,
                                        self.results)) for k in KINDS]
        for p in self.procs:
            p.start()
        ready, t0 = set(), time.time()
        while len(ready) < len(KINDS):
            kind, tag, *_ = self.results.get(timeout=180)
            if tag == "ready":
                ready.add(kind)
        log.info("parallel vision: 3 processes ready in %.1f s (max rates: face %s, hand %s, pose %s)", time.time() - t0,
                 *[("%.0f Hz" % hz[k]) if hz[k] > 0 else "free" for k in KINDS])
        self._seq = 0
        self._w = None
        self._latest: Dict[str, Any] = {"face": None, "hand": [], "pose": (None, None)}
        self._t_res = {k: -1e9 for k in KINDS}                    # frame time of each model's newest result
        self.ages: Dict[str, float] = {k: 1e9 for k in KINDS}    # how old that result is at the current frame [s]
        self._count = {k: 0 for k in KINDS}
        self._t_rate = time.monotonic()
        self._rates = {k: 0.0 for k in KINDS}
        self.last_body: Optional[np.ndarray] = None
        self.last_body_world: Optional[np.ndarray] = None
        self.force_pose = False                                  # interface of FaceHandVision (pose runs continuously)
        self.pose_only = False

    @property
    def pose_available(self) -> bool:
        return True

    def detect(self, bgr: np.ndarray, t: float) -> Tuple[Optional[np.ndarray], List[np.ndarray]]:
        import cv2
        h, w = bgr.shape[:2]
        with self.lock:
            self.buf[:h, :w] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            self.meta[0], self.meta[1], self.meta[2] = h, w, t
            self._seq += 1
            self.seq.value = self._seq
        if self._w is not None and self._w != w:                 # frame size changed: drop old results
            self._latest = {"face": None, "hand": [], "pose": (None, None)}
        self._w = w
        while True:
            try:
                kind, s, rw, t_res, out = self.results.get_nowait()
            except queue.Empty:
                break
            if isinstance(out, Exception):
                log.warning("%s model failed: %s", kind, out)
                continue
            if s == "ready" or rw != w:
                continue
            self._latest[kind] = out
            self._t_res[kind] = t_res
            self._count[kind] += 1
        self.ages = {k: t - self._t_res[k] for k in KINDS}
        self.result_t = dict(self._t_res)                        # frame time each newest result was computed on
        now = time.monotonic()
        if now - self._t_rate >= 2.0:
            self._rates = {k: self._count[k] / (now - self._t_rate) for k in KINDS}
            self._count = {k: 0 for k in KINDS}
            self._t_rate = now
        face = self._latest["face"]
        if face is not None and len(face) < 468:
            face = None
        body, world = self._latest["pose"]
        self.last_body, self.last_body_world = body, world
        return (None if self.pose_only else face), ([] if self.pose_only else list(self._latest["hand"] or []))

    def rates(self) -> Dict[str, float]:
        return dict(self._rates)

    def close(self) -> None:
        self.stop_ev.set()
        for p in self.procs:
            p.join(timeout=3)
            if p.is_alive():
                p.terminate()
        self.shm.close()
        try:
            self.shm.unlink()
        except FileNotFoundError:
            pass
