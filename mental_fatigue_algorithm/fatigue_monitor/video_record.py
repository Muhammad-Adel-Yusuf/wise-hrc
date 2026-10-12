"""Video of an experiment, recorded while MATLAB (liveHRC) says so.

liveHRC writes  POMDP model/videos/recording.json = {"recording": true/false, "dir": "<session folder>", "window": "HRC live"}:
  * true  (G START, R RESUME)  -> this recorder writes
        <dir>/camera.mp4 + camera_times.csv   the camera frames (CAMERA_FPS) with their wall-clock time, and
        <dir>/plots.mp4  + plot_times.csv     a screen capture of MATLAB's live window(s) (window titles starting with
                                              "HRC live"), WINDOW_FPS, in a background thread - MATLAB itself does not
                                              capture anything, so its 0.5 s decision loop keeps its pace;
  * false (P PAUSE, X STOP, Esc) -> the files are closed and <dir>/camera_done is written.
MATLAB writes <dir>/status.csv (one row per decision); make_experiment_video.py combines the three into one video.
The MATLAB window must be VISIBLE on the screen (not minimised, not covered) to be captured.
"""
from __future__ import annotations

import ctypes
import json
import logging
import sys
import threading
import time
from ctypes import wintypes
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

log = logging.getLogger("fatigue.video")
CAMERA_FPS = 15.0
WINDOW_FPS = 2.0
CHECK_EVERY_S = 0.3


# ---------------------------------------------------------------------------------------------------------------
# MATLAB window capture (Windows)
# ---------------------------------------------------------------------------------------------------------------
def _windows_with_title(prefix: str) -> List[int]:
    """Handles of the visible top-level windows whose title starts with prefix (left to right)."""
    user32 = ctypes.windll.user32
    found = []

    @ctypes.WINFUNCTYPE(ctypes.c_bool, wintypes.HWND, wintypes.LPARAM)
    def cb(hwnd, _):
        if user32.IsWindowVisible(hwnd) and not user32.IsIconic(hwnd):
            n = user32.GetWindowTextLengthW(hwnd)
            if n:
                buf = ctypes.create_unicode_buffer(n + 1)
                user32.GetWindowTextW(hwnd, buf, n + 1)
                if buf.value.startswith(prefix):
                    found.append(hwnd)
        return True

    user32.EnumWindows(cb, 0)
    return sorted(found, key=lambda h: _rect(h)[0])


def _rect(hwnd: int):
    """Window rectangle without the invisible shadow border (physical pixels)."""
    r = wintypes.RECT()
    try:
        ctypes.windll.dwmapi.DwmGetWindowAttribute(hwnd, 9, ctypes.byref(r), ctypes.sizeof(r))   # extended frame bounds
    except (AttributeError, OSError):
        ctypes.windll.user32.GetWindowRect(hwnd, ctypes.byref(r))
    return r.left, r.top, r.right, r.bottom


def grab_windows(prefix: str) -> Optional[np.ndarray]:
    """BGR screen capture of the windows whose title starts with prefix (side by side), or None."""
    from PIL import ImageGrab
    imgs = []
    for hwnd in _windows_with_title(prefix):
        l, t, r, b = _rect(hwnd)
        if r - l < 50 or b - t < 50:
            continue
        imgs.append(np.array(ImageGrab.grab(bbox=(l, t, r, b), all_screens=True))[:, :, ::-1])
    if not imgs:
        return None
    if len(imgs) == 1:
        return imgs[0]
    h = max(i.shape[0] for i in imgs)
    return np.hstack([cv2.copyMakeBorder(i, 0, h - i.shape[0], 0, 0, cv2.BORDER_CONSTANT) for i in imgs])


class WindowRecorder(threading.Thread):
    """Background capture of MATLAB's live window(s) -> plots.mp4 + plot_times.csv."""

    def __init__(self, d: Path, prefix: str, fps: float = WINDOW_FPS) -> None:
        super().__init__(name="window-recorder", daemon=True)
        self.d, self.prefix, self.period = d, prefix, 1.0 / fps
        self.fps = fps
        self._halt = threading.Event()
        self.frames = 0

    def run(self) -> None:
        if sys.platform == "win32":                             # physical pixels for both the rectangle and the grab
            try:
                ctypes.windll.user32.SetThreadDpiAwarenessContext(ctypes.c_void_p(-4))
            except (AttributeError, OSError):
                pass
        writer, size, times = None, None, open(self.d / "plot_times.csv", "w", encoding="utf-8")
        times.write("t_wall\n")
        try:
            while not self._halt.is_set():
                t0 = time.monotonic()
                try:
                    img = grab_windows(self.prefix)
                except Exception as exc:                        # noqa: BLE001 (screen locked, window closing ...)
                    log.warning("window capture failed: %s", exc)
                    img = None
                if img is not None:
                    if writer is None:
                        h, w = img.shape[:2]
                        size = (w - w % 2, h - h % 2)
                        writer = cv2.VideoWriter(str(self.d / "plots.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, size)
                    if (img.shape[1], img.shape[0]) != size:
                        img = cv2.resize(img, size, interpolation=cv2.INTER_AREA)
                    writer.write(img)
                    times.write("%.3f\n" % time.time())
                    self.frames += 1
                self._halt.wait(max(0.0, self.period - (time.monotonic() - t0)))
        finally:
            if writer is not None:
                writer.release()
            times.close()

    def stop(self) -> None:
        self._halt.set()
        self.join(timeout=5.0)


# ---------------------------------------------------------------------------------------------------------------
class CameraRecorder:
    def __init__(self, control_file: Path, fps: float = CAMERA_FPS) -> None:
        self.control = Path(control_file)
        self.period = 1.0 / fps
        self.fps = fps
        self.writer: Optional[cv2.VideoWriter] = None
        self.times = None
        self.window: Optional[WindowRecorder] = None
        self.dir: Optional[Path] = None
        self._checked = 0.0
        self._mtime = None
        self._want: Optional[Path] = None
        self._prefix = "HRC live"
        self._last_frame = -1e9
        self.frames = 0

    @property
    def recording(self) -> bool:
        return self.writer is not None

    def _read_control(self) -> None:
        try:
            m = self.control.stat().st_mtime
        except OSError:
            self._want = None
            return
        if m == self._mtime:
            return
        self._mtime = m
        try:
            d = json.loads(self.control.read_text(encoding="utf-8"))
            self._want = Path(d["dir"]) if d.get("recording") and d.get("dir") else None
            self._prefix = d.get("window", "HRC live")
        except (OSError, ValueError, KeyError):
            pass                                                # half-written: read again next time

    def update(self, t_wall: float, frame_bgr: Optional[np.ndarray]) -> None:
        now = time.monotonic()
        if now - self._checked >= CHECK_EVERY_S:
            self._checked = now
            self._read_control()
            if self._want != self.dir:
                self._close()
                if self._want is not None:
                    self._open(self._want, frame_bgr)
        if self.writer is None or frame_bgr is None or now - self._last_frame < self.period * 0.9:
            return
        self._last_frame = now
        h, w = self.size[1], self.size[0]
        img = frame_bgr if frame_bgr.shape[:2] == (h, w) else cv2.resize(frame_bgr, (w, h))
        self.writer.write(img)
        self.times.write("%.3f\n" % t_wall)
        self.frames += 1

    def _open(self, d: Path, frame_bgr: Optional[np.ndarray]) -> None:
        if frame_bgr is None:
            return
        d.mkdir(parents=True, exist_ok=True)
        h, w = frame_bgr.shape[:2]
        self.size = (w, h)
        self.writer = cv2.VideoWriter(str(d / "camera.mp4"), cv2.VideoWriter_fourcc(*"mp4v"), self.fps, self.size)
        if not self.writer.isOpened():
            log.warning("cannot write %s", d / "camera.mp4")
            self.writer = None
            return
        self.times = open(d / "camera_times.csv", "w", encoding="utf-8")
        self.times.write("t_wall\n")
        self.dir, self.frames = d, 0
        (d / "camera_done").unlink(missing_ok=True)
        if sys.platform == "win32":
            self.window = WindowRecorder(d, self._prefix)
            self.window.start()
        log.info("recording started (camera + MATLAB window '%s*'): %s", self._prefix, d)

    def _close(self) -> None:
        if self.writer is not None:
            self.writer.release()
            self.times.close()
            nwin = 0
            if self.window is not None:
                self.window.stop()
                nwin = self.window.frames
                self.window = None
            (self.dir / "camera_done").write_text("%d camera frames, %d window frames\n" % (self.frames, nwin),
                                                  encoding="utf-8")
            log.info("recording stopped: %d camera frames, %d MATLAB window frames in %s", self.frames, nwin, self.dir)
        self.writer, self.times, self.dir = None, None, None

    def close(self) -> None:
        self._close()
