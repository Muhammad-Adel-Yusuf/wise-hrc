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
    depth_t: Optional[float] = None       # capture time of depth_mm: == t when they came together, older when the depth
                                          # stream is slower than colour (newest depth image, see OrbbecRGBD.read)


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
    """Colour + depth, depth aligned to colour (HW D2C or SW AlignFilter).

    newest_depth=False (calibration and measuring tools): only colour frames that came WITH their own depth image are
    returned (exact pairs; the frame rate is then the depth rate).
    newest_depth=True (live program): every colour frame is returned, at the colour rate. The depth stream can be slower (Femto Bolt on a USB 2 link: the
    raw time-of-flight data limits it to ~6-7 fps whatever the depth resolution), so each frame carries the NEWEST
    aligned depth image with its own capture time depth_t (frame sync pairs it with the colour frame it came with);
    depth older than max_depth_age_s is not attached. Users that need the depth of a given moment pick the depth image
    nearest in time (main.py, DepthHistory)."""

    def __init__(self, cam_cfg: Dict[str, Any], newest_depth: bool = False) -> None:
        self.cfg = cam_cfg
        self.newest_depth = newest_depth
        self._ob: Any = None
        self._pipeline: Any = None
        self._align: Any = None
        self._depth: Optional[np.ndarray] = None                 # newest aligned depth [mm] and its capture time
        self._depth_t: Optional[float] = None
        self.connection = "?"
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

        try:
            self.connection = str(self._pipeline.get_device().get_device_info().get_connection_type())
        except Exception:                                         # noqa: BLE001
            self.connection = "?"
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
        if use_depth and not self.usb3:
            log.warning("CAMERA LINK %s: depth only ~6-7 fps (the raw depth data does not fit USB 2). Colour still runs "
                        "at %d fps with the newest depth; for 30 fps depth the link must be USB 3 - run "
                        "python camera_check.py and follow its advice.", self.connection, fps)
        else:
            log.info("camera connection: %s", self.connection)
        log.info("Orbbec started: colour %s | depth %s | %s", self.color_profile_desc,
                 self.depth_profile_desc, self.align_mode)

    @property
    def usb3(self) -> bool:
        return "USB3" in self.connection.upper().replace(".", "").replace(" ", "")

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
        """The next colour frame with the newest aligned depth image (see the class doc). None: no colour frame yet."""
        ob = self._ob
        frames = self._pipeline.wait_for_frames(int(self.cfg["frame_timeout_ms"]))
        if frames is None:
            return None
        t = time.monotonic()
        cf = frames.get_color_frame()
        df = frames.get_depth_frame() if self.cfg.get("use_depth", True) else None
        if df is not None:                                       # a new depth image: align it once, keep it
            if self._align is not None:                          # software alignment needs its colour frame
                aligned = self._align.process(frames) if cf is not None else None
                if aligned is not None:
                    aligned = aligned.as_frame_set() if hasattr(aligned, "as_frame_set") else aligned
                df = aligned.get_depth_frame() if aligned is not None else None
            d = self._decode_depth(df) if df is not None else None
            if d is not None:
                self._depth, self._depth_t = d, t
        if cf is None:
            return None
        if self.cfg.get("use_depth", True) and not self.newest_depth and self._depth_t != t:
            return None                                          # exact pairs only: this colour frame has no depth
        color = frame_to_bgr(cf, ob)
        if color is None:
            return None
        depth_mm, depth_t = self._depth, self._depth_t
        if depth_mm is None or depth_t is None or t - depth_t > float(self.cfg.get("max_depth_age_s", 0.5)):
            depth_mm, depth_t = None, None
        aligned_ok = depth_mm is not None and depth_mm.shape[:2] == color.shape[:2]
        return RGBDFrame(t, color, depth_mm, aligned_ok, depth_t)

    @staticmethod
    def _decode_depth(df: Any) -> Optional[np.ndarray]:
        try:
            d = np.frombuffer(df.get_data(), dtype=np.uint16).reshape(df.get_height(), df.get_width())
            scale = 1.0
            for name in ("get_depth_scale", "get_value_scale"):
                if hasattr(df, name):
                    scale = float(getattr(df, name)())
                    break
            return d.astype(np.float32) * scale
        except Exception as exc:                                # noqa: BLE001
            log.warning("Depth frame decode failed: %s", exc)
            return None

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


@dataclass
class DepthView:
    """The depth image of one moment, in the frame interface the 3-D code uses (depth_mm, depth_aligned)."""
    depth_mm: Optional[np.ndarray]
    depth_aligned: bool
    depth_t: Optional[float] = None
    skew_s: Optional[float] = None        # |depth capture time - requested time|


class DepthHistory:
    """Recent aligned depth images by capture time. at(t) gives the one NEAREST to t (the frame time a landmark model
    ran on): with depth slower than colour (USB 2) a landmark is measured in the depth image closest to its own moment,
    never further than max_skew_s away (otherwise no depth: the 3-D code then holds / falls back as for a dropout)."""

    def __init__(self, keep_s: float = 1.5, max_skew_s: float = 0.2) -> None:
        self.keep_s, self.max_skew_s = keep_s, max_skew_s
        self._t: List[float] = []
        self._d: List[np.ndarray] = []
        self.added = 0                                           # depth images added (for the rate in the perf log)
        self._skews: List[float] = []

    def add(self, frame: Any) -> None:
        dt = getattr(frame, "depth_t", None)
        if dt is None or not getattr(frame, "depth_aligned", False) or (self._t and dt <= self._t[-1]):
            return
        self._t.append(dt)
        self._d.append(frame.depth_mm)
        self.added += 1
        while self._t and self._t[0] < frame.t - self.keep_s:
            self._t.pop(0)
            self._d.pop(0)

    def at(self, t: Optional[float]) -> DepthView:
        if t is None or not self._t:
            return DepthView(None, False)
        i = min(range(len(self._t)), key=lambda k: abs(self._t[k] - t))
        skew = abs(self._t[i] - t)
        if skew > self.max_skew_s:
            return DepthView(None, False, self._t[i], skew)
        self._skews.append(skew)
        return DepthView(self._d[i], True, self._t[i], skew)

    def take_skew_ms(self) -> float:
        """Mean |skew| [ms] since the last call (perf log)."""
        m = 1000.0 * float(np.mean(self._skews)) if self._skews else float("nan")
        self._skews = []
        return m


class ThreadedSource:
    """Reads the camera in a background thread (colour + depth alignment overlap with the processing of the previous
    frame). read() returns the NEWEST frame not returned before (waits up to timeout_s for it; older frames are
    dropped, never queued). Every other attribute (intrinsics, color_profile_desc, ...) is the wrapped source's."""

    def __init__(self, source, timeout_s: float = 0.5) -> None:
        import threading
        self._src, self._timeout = source, timeout_s
        self._cond = threading.Condition()
        self._frame: Optional[RGBDFrame] = None
        self._n = self._taken = 0
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="camera-reader", daemon=True)
        self._thread.start()

    def __getattr__(self, name):
        return getattr(self._src, name)

    def _run(self) -> None:
        while not self._stop:
            try:
                f = self._src.read()
            except Exception as exc:                            # noqa: BLE001
                log.warning("camera read failed: %s", exc)
                time.sleep(0.05)
                continue
            if f is not None:
                with self._cond:
                    self._frame, self._n = f, self._n + 1
                    self._cond.notify_all()

    def start(self) -> None:
        pass

    def read(self) -> Optional[RGBDFrame]:
        with self._cond:
            if self._n == self._taken:
                self._cond.wait(self._timeout)
            if self._n == self._taken:
                return None
            self._taken = self._n
            return self._frame

    def stop(self) -> None:
        self._stop = True
        self._thread.join(timeout=2.0)
        self._src.stop()
