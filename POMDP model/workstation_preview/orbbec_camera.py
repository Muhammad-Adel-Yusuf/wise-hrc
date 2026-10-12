"""Orbbec RGB-D acquisition (pyorbbecsdk v2 API) plus a plain-webcam fallback source.

Cannot be exercised without the physical camera; written against the public
pyorbbecsdk v2 examples (Pipeline / Config / AlignFilter / get_camera_param).
Every SDK call that differs between releases is wrapped defensively.
"""
from __future__ import annotations

import logging
import time
from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np

log = logging.getLogger(__name__)


@dataclass
class Intrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    def scaled(self, width: int, height: int) -> "Intrinsics":
        sx, sy = width / self.width, height / self.height
        return Intrinsics(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, width, height)


@dataclass
class RGBDFrame:
    t: float                              # host monotonic capture time (s)
    color_bgr: np.ndarray
    depth_mm: Optional[np.ndarray]        # aligned to colour, same HxW, or None
    depth_aligned: bool


def _profiles(plist: Any) -> List[Any]:
    """Enumerate a StreamProfileList across SDK versions."""
    try:
        return [plist[i] for i in range(len(plist))]
    except Exception:                                           # noqa: BLE001
        n = plist.get_count()
        return [plist.get_stream_profile_by_index(i) for i in range(n)]


def _video(p: Any) -> Any:
    return p.as_video_stream_profile() if hasattr(p, "as_video_stream_profile") else p


def _fmt(ob: Any, name: str) -> Any:
    return getattr(ob.OBFormat, name, None)


def frame_to_bgr(frame: Any, ob: Any) -> Optional[np.ndarray]:
    """Convert an SDK colour frame to a BGR image (RGB/BGR/MJPG/YUYV/UYVY/NV12/I420)."""
    import cv2
    w, h, fmt = frame.get_width(), frame.get_height(), frame.get_format()
    raw = np.frombuffer(frame.get_data(), dtype=np.uint8)
    if fmt == _fmt(ob, "RGB") and raw.size == w * h * 3:
        return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if fmt == _fmt(ob, "BGR") and raw.size == w * h * 3:
        return raw.reshape(h, w, 3).copy()
    if fmt == _fmt(ob, "MJPG"):
        return cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if fmt == _fmt(ob, "YUYV") and raw.size == w * h * 2:
        return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    if fmt == _fmt(ob, "UYVY") and raw.size == w * h * 2:
        return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_UYVY)
    if fmt == _fmt(ob, "NV12") and raw.size == w * h * 3 // 2:
        return cv2.cvtColor(raw.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV12)
    if fmt == _fmt(ob, "NV21") and raw.size == w * h * 3 // 2:
        return cv2.cvtColor(raw.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_NV21)
    if fmt == _fmt(ob, "I420") and raw.size == w * h * 3 // 2:
        return cv2.cvtColor(raw.reshape(h * 3 // 2, w), cv2.COLOR_YUV2BGR_I420)
    log.warning("Unsupported colour format %s", fmt)
    return None


class OrbbecRGBD:
    """Synchronised colour + depth, depth aligned to colour (HW D2C or SW AlignFilter)."""

    def __init__(self, cam_cfg: Dict[str, Any]) -> None:
        self.cfg = cam_cfg
        self._ob: Any = None
        self._pipeline: Any = None
        self._align: Any = None
        self.intrinsics: Optional[Intrinsics] = None
        self.color_profile_desc = ""
        self.depth_profile_desc = ""
        self.align_mode = "none"

    # -- profile selection ---------------------------------------------------
    def _pick(self, plist: Any, w: int, h: int, fps: int, formats: List[Any]) -> Any:
        cands = [_video(p) for p in _profiles(plist)]
        for want_fmt in formats + [None]:
            for p in cands:
                if (p.get_width(), p.get_height()) == (w, h) and p.get_fps() >= fps - 1 and p.get_fps() <= fps + 1 \
                        and (want_fmt is None or p.get_format() == want_fmt):
                    return p
        for p in cands:                                         # same size, any fps
            if (p.get_width(), p.get_height()) == (w, h):
                return p
        log.warning("Requested %dx%d@%d unavailable; using the closest profile", w, h, fps)
        return min(cands, key=lambda p: (abs(p.get_width() * p.get_height() - w * h), abs(p.get_fps() - fps)))

    def start(self) -> None:
        import pyorbbecsdk as ob                                 # imported here: optional dependency
        self._ob = ob
        c = self.cfg
        self._pipeline = ob.Pipeline()
        config = ob.Config()
        fps = int(c["fps"])
        fmts = [f for f in (_fmt(ob, n) for n in c["color_formats"]) if f is not None]

        color_list = self._pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
        color_p = self._pick(color_list, c["color_width"], c["color_height"], fps, fmts)
        self.color_profile_desc = f"{color_p.get_width()}x{color_p.get_height()}@{color_p.get_fps()} {color_p.get_format()}"

        use_depth = bool(c.get("use_depth", True))
        depth_p = None
        if use_depth and c.get("prefer_hw_d2c", False):
            try:
                hw = _profiles(self._pipeline.get_d2c_depth_profile_list(color_p, ob.OBAlignMode.HW_MODE))
                if hw:
                    depth_p = _video(hw[0])
                    config.set_align_mode(ob.OBAlignMode.HW_MODE)
                    self.align_mode = "hardware D2C"
            except Exception as exc:                            # noqa: BLE001
                log.warning("Hardware D2C unavailable (%s); falling back to software align", exc)
        if use_depth and depth_p is None:
            depth_list = self._pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
            y16 = _fmt(ob, "Y16")
            depth_p = self._pick(depth_list, c["depth_width"], c["depth_height"], fps,
                                 [y16] if y16 is not None else [])
            self._align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
            self.align_mode = "software AlignFilter (depth->color)"
        if use_depth:
            self.depth_profile_desc = f"{depth_p.get_width()}x{depth_p.get_height()}@{depth_p.get_fps()}"
        else:                                                    # colour only: no depth stream, no alignment, no frame sync
            self.depth_profile_desc, self.align_mode = "off (use_depth=false)", "no depth"

        config.enable_stream(color_p)
        if use_depth:
            config.enable_stream(depth_p)
        if use_depth and c.get("enable_frame_sync", True):
            try:
                self._pipeline.enable_frame_sync()
            except Exception as exc:                            # noqa: BLE001
                log.warning("Frame sync not available: %s", exc)
        self._pipeline.start(config)
        self._read_intrinsics(color_p)
        log.info("Orbbec started: colour %s | depth %s | %s", self.color_profile_desc,
                 self.depth_profile_desc, self.align_mode)

    def _read_intrinsics(self, color_p: Any) -> None:
        w, h = color_p.get_width(), color_p.get_height()
        try:
            param = self._pipeline.get_camera_param()
            i = param.rgb_intrinsic
            self.intrinsics = Intrinsics(float(i.fx), float(i.fy), float(i.cx), float(i.cy),
                                         int(i.width) or w, int(i.height) or h).scaled(w, h)
        except Exception as exc:                                # noqa: BLE001
            log.warning("Could not read intrinsics (%s); assuming ~90 deg HFOV", exc)
            f = 0.5 * w
            self.intrinsics = Intrinsics(f, f, w / 2.0, h / 2.0, w, h)

    # -- acquisition -----------------------------------------------------------
    def read(self) -> Optional[RGBDFrame]:
        ob = self._ob
        frames = self._pipeline.wait_for_frames(int(self.cfg["frame_timeout_ms"]))
        if frames is None:
            return None
        t = time.monotonic()
        if self._align is not None:
            aligned = self._align.process(frames)
            if aligned is None:
                return None
            frames = aligned.as_frame_set() if hasattr(aligned, "as_frame_set") else aligned
        cf = frames.get_color_frame()
        if cf is None:
            return None
        color = frame_to_bgr(cf, ob)
        if color is None:
            return None
        depth_mm, aligned_ok = None, False
        df = frames.get_depth_frame() if self._align is not None or self.cfg.get("use_depth", True) else None
        if df is not None:
            try:
                d = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(df.get_height(), df.get_width())
                scale = 1.0
                for name in ("get_depth_scale", "get_value_scale"):
                    if hasattr(df, name):
                        scale = float(getattr(df, name)())
                        break
                depth_mm = d.astype(np.float32) * scale
                aligned_ok = depth_mm.shape[:2] == color.shape[:2]
            except Exception as exc:                            # noqa: BLE001
                log.warning("Depth frame decode failed: %s", exc)
                depth_mm = None
        return RGBDFrame(t, color, depth_mm, aligned_ok)

    def stop(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as exc:                            # noqa: BLE001
                log.warning("Pipeline stop failed: %s", exc)
            self._pipeline = None


class OpenCVSource:
    """Plain webcam / video-file source (no depth) for development without the camera."""

    def __init__(self, source: Any = 0, width: int = 1280, height: int = 720) -> None:
        import cv2
        self._cap = cv2.VideoCapture(source)
        if isinstance(source, int):
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open video source {source!r}")
        w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or width
        h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or height
        f = 0.9 * w
        self.intrinsics = Intrinsics(f, f, w / 2.0, h / 2.0, w, h)
        self.color_profile_desc, self.depth_profile_desc, self.align_mode = f"{w}x{h}", "none", "no depth"

    def start(self) -> None:
        pass

    def read(self) -> Optional[RGBDFrame]:
        ok, frame = self._cap.read()
        return RGBDFrame(time.monotonic(), frame, None, False) if ok else None

    def stop(self) -> None:
        self._cap.release()
