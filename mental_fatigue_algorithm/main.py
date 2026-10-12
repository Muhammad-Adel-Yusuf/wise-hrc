#!/usr/bin/env python3
"""Real-time vision-based mental-fatigue RISK estimator (Orbbec RGB-D).

Not a medical diagnosis. The robot is VIRTUAL: this program sends the worker state (fatigue, attention, presence) and
both upper limbs in the workstation frame (workspace_calibration.py) by UDP to the MATLAB POMDP framework
(POMDP model/liveHRC.m), which decides and simulates the robot. Section pomdp_bridge in config.json.

Four algorithms: 1 task, 2 visual, 3 attention (combined by weights into the MENTAL load) and 4 PHYSICAL fatigue (LFFR, standalone,
shown next to the mental load and never weighted into it). Programs: main.py = 1-3 combined + 4 side by side; main_task.py,
main_visual.py, main_attention.py, main_physical.py = one algorithm alone. Options: --no-physical (main.py without algorithm 4).
Extra keys for algorithm 4: Y = baseline (Borg CR10 starting fatigue + 12 s neutral-posture capture: sit upright, arms relaxed, hands still),
0 on/off, Z (twice) reset the physical fatigue to 0. Option: --start-borg N (starting Borg CR10 rating, skips the question).

Keys: U load PowerPoint instructions (or --pptx), Up/Down (or Left/Right, N/P) step = instruction number,
      7/8/9 switch algorithm 1/2/3 on or off (combined program; every button in the window does the same as its key),
      S start the EAR/MAR baseline (it never starts by itself; --auto-baseline restores that),
      1/2/3 calibrate Assembly/Laptop/Storage, 4 calibrate the assembly WORK POSTURE (face-free, for leaning over the
      assembly area), K toggle the collaboration flag (testing without the robot; off = STANDBY), R (twice) erase calibration, F reset fatigue to Low, B relearn the EAR/MAR baseline,
      D toggle depth window, Q or Esc quit.
"""
from __future__ import annotations

import argparse
import json
import logging
import subprocess
import sys
import textwrap
import time
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import math
import cv2
import numpy as np

from fatigue_monitor import geometry as geo
from fatigue_monitor.attention import KEY_TO_STATION, STATION_LABELS
from fatigue_monitor.config import ConfigError, INDICATORS, load_config
from fatigue_monitor import dashboard
from fatigue_monitor.instruction_deck import InstructionDeck, ask_for_pptx
from fatigue_monitor.csv_logger import CsvLogger, MODE_COLUMNS
from fatigue_monitor.monitor import FatigueMonitor, MonitorState, Observation
from fatigue_monitor.orbbec_camera import Intrinsics, OpenCVSource, OrbbecRGBD, RGBDFrame
from fatigue_monitor.vision import FeatureExtractor, create_vision
from fatigue_monitor.borg_prompt import ask_borg
from fatigue_monitor.physical_runtime import LOG_COLUMNS as PHYS_LOG_COLUMNS, PhysicalAlgorithm, PhysicalUnavailable
from fatigue_monitor.pomdp_bridge import (ArmTracker, BridgeSender, HeadTracker, WorkspaceFrame, arm_status,
                                         build_message, robot_block)

log = logging.getLogger("fatigue")

class InstructionFeed:
    """Current laptop instruction number (+ optional action events) for the task-difficulty algorithm.

    Sources: the file given with --instruction-file (plain integer, or JSON such as
    {"instruction": 5, "action": "completed"}; re-read when it changes) and the keys N / P (next / previous
    instruction), A (action attempted), C (action completed). Without any source the task-difficulty
    algorithm stays inactive and reports 0.
    """

    def __init__(self, path: Optional[str] = None) -> None:
        self.path = Path(path) if path else None
        self.value: Optional[int] = None
        self._mtime: Optional[float] = None
        self._last_poll = 0.0
        self._action: Optional[str] = None
        self.limit: Optional[int] = None                 # number of slides when a PowerPoint deck is loaded

    def set_limit(self, n: Optional[int], start: bool = False) -> None:
        self.limit = n
        if n is not None and (start or self.value is None):
            self.value = 1
        if n is not None and self.value is not None:
            self.value = min(max(self.value, 1), n)

    def poll(self) -> None:
        now = time.monotonic()
        if self.path is None or now - self._last_poll < 0.2:
            return
        self._last_poll = now
        try:
            mtime = self.path.stat().st_mtime
            if mtime == self._mtime:
                return
            self._mtime = mtime
            data = json.loads(self.path.read_text(encoding="utf-8").strip())
        except (OSError, ValueError):
            return
        n, act = (data.get("instruction"), data.get("action")) if isinstance(data, dict) else (data, None)
        if isinstance(n, int) and not isinstance(n, bool):
            self.value = n
        if act in ("attempted", "completed", "corrective"):
            self._action = act

    def next(self) -> None:
        v = 1 if self.value is None else self.value + 1
        self.value = min(v, self.limit) if self.limit else v

    def prev(self) -> None:
        low = 1 if self.limit else 0
        self.value = 1 if self.value is None else max(low, self.value - 1)

    def action(self, kind: str) -> None:
        self._action = kind

    def take_action(self) -> Optional[str]:
        a, self._action = self._action, None
        return a


LEVEL_COLORS = {"Low": (60, 180, 60), "Medium": (0, 165, 255), "High": (40, 40, 220), "Unknown": (128, 128, 128)}
LABELS = {"perclos": "PERCLOS", "closure": "Prolonged closure", "brief_disengagement": "Brief look-away",
          "sustained_disengagement": "Sustained (>=2s)",
          "yawn": "Yawns (5 min)", "nod": "Nods (5 min)", "rub": "Eye rubs (5 min)"}


# Arrow-key codes returned by cv2.waitKeyEx: Windows (up/down/left/right) and Linux GTK.
ARROW_NEXT = (2621440, 2555904, 65364, 65363)      # Down, Right
ARROW_PREV = (2490368, 2424832, 65362, 65361)      # Up, Left


# Run modes: "all" = the three algorithms + combiner (final level); the others run ONE algorithm alone.
MODES = ("all", "task", "visual", "attention")      # mental-load programs (draw_ui modes)
PROGRAM_MODES = MODES + ("physical",)                  # + algorithm 4 alone (own window: dashboard.render_physical)
MODE_TITLE = {"task": "ALGORITHM 1 - TASK", "visual": "ALGORITHM 2 - VISUAL", "attention": "ALGORITHM 3 - ATTENTION"}
MODE_KEYS = {"task": ("task_score", "task_fatigue_level"), "visual": ("visual_score", "visual_fatigue_level"),
             "attention": ("attention_score", "attention_fatigue_level")}
MODE_INDICATORS = {"visual": ("perclos", "closure", "yawn", "nod", "rub"),
                   "attention": ("brief_disengagement", "sustained_disengagement")}
MODE_WARN_WORDS = {"visual": ("baseline", "quality", "warming"), "attention": ("baseline", "calibration", "quality", "warming"),
                   "task": ("baseline", "calibration", "quality", "warming", "instruction")}


def mode_result(st: "MonitorState", mode: str) -> Tuple[int, float, str]:
    """(level_code, score, label) shown as the headline: the final level in mode 'all', else that algorithm's own."""
    if mode == "all":
        return st.level_code, st.combined_score, st.level
    out = {"task": st.task, "visual": st.visual, "attention": st.attention_out}[mode]
    ks, kl = MODE_KEYS[mode]
    lv = int(out[kl])
    return lv, float(out[ks]), ("Low", "Medium", "High")[lv]


# Exit codes that must NOT trigger an automatic restart by the supervisor.
# (A native MediaPipe abort() exits with 3 on Windows / a signal on Linux, so 3 is deliberately not used.)
EXIT_CONFIG, EXIT_MISSING_DEP, EXIT_STARTUP = 2, 20, 21
NO_RESTART_CODES = (0, EXIT_CONFIG, EXIT_MISSING_DEP, EXIT_STARTUP)


class Perf:
    """Milliseconds per loop stage, averaged and logged every period_s (to see where the frame rate goes)."""

    def __init__(self, period_s: float = 10.0) -> None:
        self.period, self.sum, self.n, self.t_log, self.t = period_s, {}, 0, time.perf_counter(), None
        self.extra = None                                        # callable -> extra text for the log line

    def start(self) -> None:
        self.t = time.perf_counter()

    def mark(self, stage: str) -> None:
        now = time.perf_counter()
        if self.t is not None:
            self.sum[stage] = self.sum.get(stage, 0.0) + (now - self.t)
        self.t = now

    def end_frame(self) -> None:
        self.n += 1
        now = time.perf_counter()
        if now - self.t_log >= self.period and self.n:
            el = now - self.t_log
            parts = "  ".join("%s %.0f" % (k, 1000.0 * v / self.n) for k, v in self.sum.items())
            log.info("perf: %.1f fps | ms per frame: %s%s", self.n / el, parts, (" | " + self.extra()) if self.extra else "")
            self.sum, self.n, self.t_log = {}, 0, now


def supervise(argv: list, max_restarts: int, window_s: float = 600.0) -> int:
    """Run the monitor in a child process and restart it if it dies abnormally.

    A native crash inside MediaPipe/Orbbec cannot be caught with try/except; only a
    separate process survives it. The calibration file and CSV log persist across
    restarts; the 60 s EAR/MAR baseline is re-learned after each restart.
    """
    crashes: list = []
    cmd = [sys.executable, str(Path(__file__).resolve()), "--child", *argv]
    while True:
        try:
            code = subprocess.call(cmd)
        except KeyboardInterrupt:
            return 0
        if code in NO_RESTART_CODES:
            return code
        now = time.monotonic()
        crashes = [c for c in crashes if now - c < window_s] + [now]
        log.error("Monitor process crashed (exit code %s). Crash %d of %d allowed in %.0f s.",
                  code, len(crashes), max_restarts, window_s)
        if len(crashes) > max_restarts:
            log.error("Too many crashes; giving up. Check camera/USB and MediaPipe versions.")
            return 1
        time.sleep(min(2.0 * len(crashes), 10.0))
        log.info("Restarting monitor ...")


def fmt(v: Optional[float], spec: str = "{:.2f}") -> str:
    return "--" if v is None else spec.format(v)


# ---------------------------------------------------------------------------
# Depth / 3-D head position (display + logging only; never used for scoring)
# ---------------------------------------------------------------------------
class DepthTracker:
    """Head 3-D position from several face points, with a short hold across dropouts."""

    def __init__(self) -> None:
        self._last: Optional[Dict[str, Any]] = None
        self._last_t = -1e9

    def update(self, t: float, frame: RGBDFrame, face_px: Optional[np.ndarray], inf_size: Tuple[int, int],
               intr: Intrinsics, cfg: Dict[str, Any]) -> Dict[str, Any]:
        v = cfg["vision"]
        out: Dict[str, Any] = {"valid": False, "distance_m": None, "x_m": None, "y_m": None, "z_m": None,
                               "n_points": 0, "held": False, "why": ""}
        if face_px is None:
            out["why"] = "no face"
        elif frame.depth_mm is None or not frame.depth_aligned:
            out["why"] = "no aligned depth"
        else:
            h, w = frame.depth_mm.shape[:2]
            uv = face_px[list(geo.DEPTH_POINTS)] * np.array([w / inf_size[0], h / inf_size[1]])
            z, n = geo.robust_face_depth_mm(frame.depth_mm, uv, v["depth_roi_radius_px"],
                                            cfg["camera"]["depth_min_mm"], cfg["camera"]["depth_max_mm"],
                                            v["depth_min_valid_fraction"], v["depth_max_spread_mm"],
                                            v["depth_min_points"])
            out["n_points"] = n
            if z is None:
                out["why"] = f"only {n} valid depth points"
            else:
                nu, nv = uv[0]                                   # nose tip for the x/y direction
                p = geo.deproject(nu, nv, z, intr.fx, intr.fy, intr.cx, intr.cy) / 1000.0
                out.update(valid=True, distance_m=float(np.linalg.norm(p)),
                           x_m=float(p[0]), y_m=float(p[1]), z_m=float(p[2]))
                self._last, self._last_t = dict(out), t
                return out
        if self._last is not None and t - self._last_t <= v["depth_hold_s"]:
            held = dict(self._last)
            held["held"] = True
            return held
        return out


# ---------------------------------------------------------------------------
# Drawing
# ---------------------------------------------------------------------------
def put(img: np.ndarray, text: str, x: int, y: int, color=(255, 255, 255), scale: float = 0.55, thick: int = 1) -> None:
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, thick, cv2.LINE_AA)


def bar(img: np.ndarray, x: int, y: int, w: int, frac: Optional[float], color) -> None:
    cv2.rectangle(img, (x, y), (x + w, y + 10), (70, 70, 70), 1)
    if frac is not None:
        cv2.rectangle(img, (x, y), (x + int(w * max(0.0, min(1.0, frac))), y + 10), color, -1)


def stage_meter(img: np.ndarray, x: int, y: int, r: Optional[float], stages: int = 10, bw: int = 9, gap: int = 2,
                h: int = 10) -> None:
    """10-stage meter: stage i is lit when r >= i/10 (at least stage 1 for any r > 0).
    Stages 1-4 green, 5-7 yellow/orange, 8-10 red. Unavailable (None) shows empty boxes."""
    lit = 0
    if r is not None:
        lit = int(min(stages, max(0, math.ceil(max(0.0, min(1.0, r)) * stages - 1e-9))))
    for i in range(stages):
        x0 = x + i * (bw + gap)
        c = (60, 180, 60) if i < 4 else ((0, 200, 255) if i < 7 else (0, 0, 230))
        if i < lit:
            cv2.rectangle(img, (x0, y), (x0 + bw, y + h), c, -1)
        else:
            cv2.rectangle(img, (x0, y), (x0 + bw, y + h), (90, 90, 90), 1)


def overall_stage(st: MonitorState) -> Optional[int]:
    """Overall 10-stage meter, banded by the final level so colour = level:
    Low -> stages 1-4 (green), Medium -> 5-7 (yellow), High -> 8-10 (red). Position inside a band from the combined score."""
    if st.level == "Unknown":
        return None
    lo, hi = {"Low": (1, 4), "Medium": (5, 7), "High": (8, 10)}[st.level]
    return int(min(hi, max(lo, math.ceil(max(0.0, st.combined_score) * 10.0 - 1e-9))))


class HeldBody:
    """Display only: keeps drawing the last body points for `hold_s` seconds on frames where the pose model did not run
    (it runs on every n-th frame), so the skeleton does not blink. The algorithms never see the held points."""

    def __init__(self, hold_s: float = 0.5) -> None:
        self.hold_s, self._body, self._t = hold_s, None, -1e9

    def update(self, t: float, body: Optional[np.ndarray]) -> Optional[np.ndarray]:
        if body is not None:
            self._body, self._t = body, t
            return body
        return self._body if self._body is not None and 0.0 <= t - self._t <= self.hold_s else None


def draw_ui(color: np.ndarray, st: MonitorState, pos: Dict[str, Any], intr_desc: str, fps: float,
            face_px: Optional[np.ndarray], hands_px, inf_size: Tuple[int, int], depth_img: Optional[np.ndarray],
            ranges: Optional[Dict[str, str]] = None, body_px: Optional[np.ndarray] = None,
            mode: str = "all", ui: Optional[Dict[str, Any]] = None) -> np.ndarray:
    """Dashboard canvas (see fatigue_monitor/dashboard.py). ui receives the clickable buttons of this frame."""
    return dashboard.render(color, st, mode_result(st, mode), mode, ui if ui is not None else {},
                            face_px=face_px, hands_px=hands_px, body_px=body_px, inf_size=inf_size,
                            depth_img=depth_img, fps=fps, intr_desc=intr_desc)


def depth_visual(depth_mm: Optional[np.ndarray], lo: float, hi: float, width: int = 320) -> Optional[np.ndarray]:
    """Coloured depth preview, made SMALL first (the dashboard shows it as a thumbnail): ~1 ms instead of ~60 ms."""
    if depth_mm is None:
        return None
    h, w = depth_mm.shape[:2]
    d = cv2.resize(depth_mm, (width, max(1, h * width // w)), interpolation=cv2.INTER_NEAREST)
    vis = cv2.applyColorMap(cv2.convertScaleAbs(d - lo, alpha=255.0 / (hi - lo)), cv2.COLORMAP_JET)
    vis[d <= 0] = 0
    return vis


# ---------------------------------------------------------------------------
def build_source(args: argparse.Namespace, cfg: Dict[str, Any]):
    cam = cfg["camera"]
    if args.source == "orbbec":
        return OrbbecRGBD(cam, newest_depth=True)      # colour rate; depth matched by time (DepthHistory)
    return OpenCVSource(args.video if args.source == "video" else args.camera_index,
                        cam["color_width"], cam["color_height"])


def run_physical_only(args: argparse.Namespace, cfg: Dict[str, Any]) -> int:
    """main_physical.py: ONLY algorithm 4 (physical fatigue). No face / hand models, no mental algorithm, no combiner."""
    try:
        phys = PhysicalAlgorithm(cfg, Path(args.config).resolve().parent)
    except PhysicalUnavailable as exc:
        log.error("%s", exc)
        return EXIT_MISSING_DEP
    source = vision = plog = None
    try:
        cp = Path(cfg["logging"]["csv_path"])
        plog = CsvLogger(str(cp.with_name(f"{cp.stem}_physical{cp.suffix}")), cfg["logging"]["interval_s"], PHYS_LOG_COLUMNS)
        source = build_source(args, cfg)
        source.start()
        vision = create_vision(cfg)
        if not hasattr(vision, "pose_only"):
            raise RuntimeError("the physical program needs vision.backend = face_hand")
        vision.pose_only = True
        if getattr(vision, "pose_available", False) is False:
            raise RuntimeError("the pose model could not be loaded (see the log above); download it or check vision.pose_model_path")
    except ImportError as exc:
        log.error("Missing dependency: %s (see requirements.txt / README)", exc)
        return EXIT_MISSING_DEP
    except Exception as exc:                                    # noqa: BLE001
        log.error("Start-up failed: %s", exc)
        if source is not None:
            source.stop()
        return EXIT_STARTUP
    intr = source.intrinsics
    inf_w = min(cfg["vision"]["inference_width"], intr.width)
    inf_h = int(round(intr.height * inf_w / intr.width))
    win = "Physical fatigue - algorithm 4"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, cfg["display"]["window_width"], int(cfg["display"]["window_width"] * dashboard.CANVAS_H / dashboard.CANVAS_W))
    ui: Dict[str, Any] = {"buttons": [], "physical_bands": cfg["physical"]["bands"]}
    clicks: list = []
    cv2.setMouseCallback(win, lambda e, x, y, f, p: clicks.append(dashboard.button_at(ui, x, y)) if e == cv2.EVENT_LBUTTONDOWN else None)
    desc = f"{source.color_profile_desc}"
    n_pose, fps, last_t, fails, frame_i, body_px, last_z = cfg["physical"]["pose_every_n"], 0.0, None, 0, 0, None, None
    msg = ""
    if args.start_borg is not None:
        phys.set_start_rating(args.start_borg)
    log.info("Physical fatigue (algorithm 4) running alone. %s | %s", desc, phys.params_source)
    try:
        while True:
            frame = source.read()
            if frame is None:
                fails += 1
                if fails > 100:
                    log.error("No frames received for a long time; stopping.")
                    break
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
                continue
            fails = 0
            t = frame.t
            frame_i += 1
            if phys.enabled and (phys.capturing or frame_i % n_pose == 0):
                small = frame.color_bgr if frame.color_bgr.shape[1] == inf_w else cv2.resize(frame.color_bgr, (inf_w, inf_h), interpolation=cv2.INTER_AREA)
                try:
                    vision.detect(small, t)
                    body_px = getattr(vision, "last_body", None)
                    phys.feed(t, getattr(vision, "last_body_world", None))
                except Exception as exc:                        # noqa: BLE001
                    log.warning("Pose inference failed: %s", exc)
            st = phys.update(t)
            if plog.maybe_log(t, phys.log_row()):
                print(f"[physical] f={st['f']:.2f} (Borg-equiv {st['borg_equiv']:.1f}) {st['band_name']} mode={st['mode']} class={st['intensity'] or '-'}", flush=True)
            if last_t is not None and t > last_t:
                fps = 0.9 * fps + 0.1 / (t - last_t) if fps else 1.0 / (t - last_t)
            last_t = t
            view = dashboard.render_physical(frame.color_bgr, st, ui, body_px=body_px, inf_size=(inf_w, inf_h), fps=fps, intr_desc=desc)
            if phys.message:
                cv2.putText(view, phys.message[:70], (40, 746), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 255), 1, cv2.LINE_AA)
            cv2.imshow(win, view)
            kx = cv2.waitKeyEx(1)
            key = kx & 0xFF if kx != -1 else -1
            if clicks:
                key = clicks.pop(0)
                clicks.clear()
            if key in (27, ord("q"), ord("Q")):
                break
            if key in (ord("y"), ord("Y")):
                b0 = ask_borg()                                        # blocking: starting fatigue, then the neutral-posture capture
                if b0 is not None:
                    phys.set_start_rating(b0)
                phys.start_capture(t)
                phys.message = "Physical baseline: sit upright, arms relaxed, hands still"
            elif key == ord("0"):
                phys.enabled = not phys.enabled
                phys.message = "Physical fatigue " + ("ON" if phys.enabled else "OFF")
            elif key in (ord("z"), ord("Z")):
                if last_z is not None and t - last_z <= 3.0:
                    phys.reset(t)
                    last_z, phys.message = None, "Physical fatigue reset to 0"
                else:
                    last_z, phys.message = t, "Press Z again within 3 s to RESET the physical fatigue to 0"
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        log.info("Interrupted")
    except Exception:                                           # noqa: BLE001
        log.exception("Unexpected error in main loop")
        return 1
    finally:
        if vision is not None:
            vision.close()
        if source is not None:
            source.stop()
        if plog is not None:
            plog.close()
        cv2.destroyAllWindows()
        log.info("Resources released")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--source", choices=("orbbec", "webcam", "video"), default="orbbec")
    ap.add_argument("--camera-index", type=int, default=0)
    ap.add_argument("--video", help="video file when --source video")
    ap.add_argument("--instruction-file", help="file with the current laptop instruction number (integer or JSON), "
                                               "for the task-difficulty algorithm")
    ap.add_argument("--pptx", help="PowerPoint instruction file (slide number = instruction number); "
                                   "or press U in the window. Step with the arrow keys.")
    ap.add_argument("--auto-baseline", action="store_true",
                    help="old behaviour: start the ~60 s EAR/MAR baseline by itself (default: only when you press S)")
    ap.add_argument("--no-wait-for-slides", action="store_true",
                    help="start the baseline at once instead of waiting for the PowerPoint upload (modes all/task)")
    ap.add_argument("--mode", choices=PROGRAM_MODES, default="all",
                    help="all = mental load (algorithms 1-3 + combiner) AND the separate physical fatigue (algorithm 4) (default); "
                         "task | visual | attention | physical = one algorithm alone")
    ap.add_argument("--start-borg", type=float, help="physical fatigue: Borg CR10 rating (0-10) before the first task -> starting fatigue f0 = B0 / 10")
    ap.add_argument("--no-physical", action="store_true",
                    help="main.py without the physical-fatigue algorithm (mental load only, as before)")
    ap.add_argument("--log-level", default="INFO")
    ap.add_argument("--no-supervisor", action="store_true", help="run in this process (no auto-restart on crash)")
    ap.add_argument("--max-restarts", type=int, default=5, help="crashes tolerated per 10 min by the supervisor")
    ap.add_argument("--child", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    logging.basicConfig(level=args.log_level.upper(), format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:                                                    # warnings also to a file (e.g. why the robot pose is lost)
        log_dir = Path(__file__).resolve().parent / "logs"
        log_dir.mkdir(exist_ok=True)
        fh = logging.FileHandler(log_dir / ("camera_%s.log" % time.strftime("%Y%m%d")), encoding="utf-8")
        fh.setLevel(logging.WARNING)
        fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
        logging.getLogger().addHandler(fh)
    except OSError:
        pass
    if not (args.child or args.no_supervisor):
        raw = list(sys.argv[1:] if argv is None else argv)
        return supervise(raw, args.max_restarts)

    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        log.error("Configuration error: %s", exc)
        return EXIT_CONFIG

    if args.mode == "physical":
        return run_physical_only(args, cfg)

    source = vision = csv_log = None
    phys: Optional[PhysicalAlgorithm] = None
    phys_log: Optional[CsvLogger] = None
    try:
        source = build_source(args, cfg)
        source.start()
        if cfg["vision"].get("opencv_threads", 0):
            cv2.setNumThreads(int(cfg["vision"]["opencv_threads"]))   # leave CPU cores to the model processes
        if cfg["vision"].get("threaded_camera", True):
            from fatigue_monitor.orbbec_camera import ThreadedSource
            source = ThreadedSource(source)                      # camera + depth alignment in a background thread
        vision = create_vision(cfg)
        csv_path = cfg["logging"]["csv_path"]
        if args.mode != "all":                                   # separate log (own columns) per single algorithm
            cp = Path(csv_path)
            csv_path = str(cp.with_name(f"{cp.stem}_{args.mode}{cp.suffix}"))
        csv_log = CsvLogger(csv_path, cfg["logging"]["interval_s"], MODE_COLUMNS.get(args.mode))
        if args.mode == "all" and cfg["physical"]["enabled"] and not args.no_physical:   # algorithm 4: separate state, separate log
            try:
                phys = PhysicalAlgorithm(cfg, Path(args.config).resolve().parent)
                pp = Path(csv_path)
                phys_log = CsvLogger(str(pp.with_name(f"{pp.stem}_physical{pp.suffix}")), cfg["logging"]["interval_s"], PHYS_LOG_COLUMNS)
                if args.start_borg is not None:
                    phys.set_start_rating(args.start_borg)
                log.info("Physical fatigue (algorithm 4) active: %s | %s", phys.params_source, phys._baseline_info()["baseline_text"])
            except PhysicalUnavailable as exc:
                log.warning("Physical fatigue (algorithm 4) disabled: %s", exc)
    except ImportError as exc:
        log.error("Missing dependency: %s (see requirements.txt / README)", exc)
        return EXIT_MISSING_DEP
    except Exception as exc:                                    # noqa: BLE001
        log.error("Start-up failed: %s", exc)
        if source is not None:
            source.stop()
        return EXIT_STARTUP

    intr = source.intrinsics
    inf_w = min(cfg["vision"]["inference_width"], intr.width)
    inf_h = int(round(intr.height * inf_w / intr.width))
    ki = intr.scaled(inf_w, inf_h)
    # link to the MATLAB POMDP framework (virtual robot): worker state + upper limbs in the workstation frame W
    bcfg = cfg["pomdp_bridge"]
    bridge: Optional[BridgeSender] = None
    arms_tr: Optional[ArmTracker] = None
    arms: dict = {}
    head, head_src = None, ""
    head_tr: Optional[HeadTracker] = None
    robot_poller = None                                        # real robot pose (read only), when W is on the real robot
    recorder = None                                            # camera video while MATLAB records (videos/recording.json)
    wframe = WorkspaceFrame()
    if bcfg["enabled"] and args.mode == "all":
        wpath = Path(bcfg["workspace_file"])
        if not wpath.is_absolute():
            wpath = Path(args.config).resolve().parent / wpath
        wframe = WorkspaceFrame(str(wpath))
        bridge, arms_tr, head_tr = BridgeSender(bcfg), ArmTracker(bcfg), HeadTracker(bcfg)
        from fatigue_monitor.video_record import CameraRecorder
        recorder = CameraRecorder(wpath.parent / "videos" / "recording.json")
        log.info("POMDP link: sending to %s:%s | workstation frame %s", bcfg["host"], bcfg["port"],
                 ("calibrated on the REAL robot" if wframe.on_real_robot else "calibrated (stand-in robot object)")
                 if wframe.calibrated else "NOT calibrated - run workspace_calibration.py (limb points are not sent)")
        if wframe.on_real_robot and bcfg["real_robot"]["enabled"]:
            from fatigue_monitor.robot_frame import RobotPoller
            robot_poller = RobotPoller(bcfg["real_robot"]["poll_hz"], full_pose=False)   # joints only: lighter
            log.info("POMDP link: reading the real robot pose (read only) for MATLAB")
        if source.align_mode == "no depth":
            log.warning("POMDP link: no depth stream - limb points cannot be measured (use the Orbbec camera)")
    extractor = FeatureExtractor(cfg, ki.fx, ki.fy, ki.cx, ki.cy)
    monitor = FatigueMonitor(cfg)
    instr = InstructionFeed(args.instruction_file)
    deck = InstructionDeck()
    deck_win = "Instructions (PowerPoint)"

    def load_deck(path: Optional[str], t_now: Optional[float] = None) -> None:
        if not path:
            return
        try:
            n = deck.load(path)
        except ValueError as exc:
            monitor._calib_msg = f"Cannot open slides: {exc}"[:70]
            log.error("Cannot open %s: %s", path, exc)
            return
        instr.set_limit(n, start=True)
        gate["slides_ok"] = True
        started = t_now is not None and try_start_baseline(t_now)
        waiting = monitor.baseline_hold
        monitor._calib_msg = (f"Slides loaded ({n}): learning baseline ~60 s, keep eyes open" if started else
                              f"Slides loaded ({n}): press S when ready to start the baseline" if waiting else
                              f"Instructions: {Path(path).name} ({n} slides) - Up/Down = step")
        refresh_hold_reason()
        log.info("Instruction deck loaded: %s (%d slides)", path, n)

    # The ~60 s EAR/MAR baseline never starts by itself: it needs YOUR key press (S) and, in the task / all programs,
    # also the uploaded slides (unless the instruction source is a file/program or --no-wait-for-slides is given).
    need_slides = (args.mode in ("all", "task") and not args.instruction_file and not args.no_wait_for_slides)
    gate = {"go": bool(args.auto_baseline), "slides_ok": not need_slides}

    def refresh_hold_reason() -> None:
        todo = []
        if not gate["go"]:
            todo.append("press S to start the baseline")
        if not gate["slides_ok"]:
            todo.append("press U to load the instruction slides")
        monitor.baseline_hold_reason = "waiting for you: " + " and ".join(todo) + " - baseline not started"

    def try_start_baseline(t_now: float) -> bool:
        """Release the baseline hold once every gate is open; the ~60 s learning starts from this moment."""
        if monitor.baseline_hold and gate["go"] and gate["slides_ok"]:
            monitor.baseline_hold = False
            monitor.reset_baseline(t_now)
            return True
        return False

    monitor.baseline_hold = not (gate["go"] and gate["slides_ok"])
    refresh_hold_reason()
    if args.pptx:
        load_deck(args.pptx)
    posture_file = cfg["work_posture"]["file"]
    if monitor.posture.load(posture_file):
        log.info("Assembly work posture loaded from %s", posture_file)
    collab = bool(cfg["collaboration"]["start_active"])          # K toggles it (stands in for the cell's flag)
    was_capturing = False
    depth_tracker = DepthTracker()
    win = "Fatigue risk monitor" if args.mode == "all" else f"Fatigue - algorithm: {args.mode}"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, cfg["display"]["window_width"], int(cfg["display"]["window_width"] * dashboard.canvas_height(phys is not None) / dashboard.CANVAS_W))
    ui: Dict[str, Any] = {"buttons": [], "instruction": None, "slides_total": None, "show_depth": False}
    last_z, frame_i = None, 0
    held_body = HeldBody(0.5)
    clicks: list = []                                          # key codes produced by mouse clicks on buttons

    def on_mouse(event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            key = dashboard.button_at(ui, x, y)
            if key is not None:
                clicks.append(key)
    cv2.setMouseCallback(win, on_mouse)
    show_depth = cfg["display"]["show_depth"]
    desc = f"{source.color_profile_desc} | {source.align_mode}"
    fps, last_t, fails, det_fails, last_r = 0.0, None, 0, 0, None
    log.info("Running. %s", desc)

    perf = Perf()
    from fatigue_monitor.orbbec_camera import DepthHistory
    depths = DepthHistory(max_skew_s=float(cfg["camera"].get("max_depth_skew_s", 0.2)))   # depth by capture time
    depth_count = [0, time.perf_counter()]

    def perf_extra() -> str:
        now = time.perf_counter()
        rate = (depths.added - depth_count[0]) / max(1e-6, now - depth_count[1])
        depth_count[:] = [depths.added, now]
        txt = "depth %.1f fps (time offset to landmarks %.0f ms)" % (rate, depths.take_skew_ms())
        if hasattr(vision, "rates"):
            txt += " | models face %.1f  hand %.1f  pose %.1f Hz" % tuple(vision.rates().get(k, 0) for k in ("face", "hand", "pose"))
        return txt
    perf.extra = perf_extra
    last_ui = 0.0
    try:
        while True:
            perf.start()
            frame = source.read()
            perf.mark("camera")
            if frame is None:
                fails += 1
                if fails > 100:
                    log.error("No frames received for a long time; stopping.")
                    break
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
                continue
            fails = 0
            t = frame.t
            small = frame.color_bgr if frame.color_bgr.shape[1] == inf_w else \
                cv2.resize(frame.color_bgr, (inf_w, inf_h), interpolation=cv2.INTER_AREA)
            frame_i += 1
            try:
                # key 4: pose needed even when the face is seen; algorithm 4: pose on every n-th frame
                vision.force_pose = monitor.posture_capture_active or bool(phys is not None and phys.enabled and (phys.capturing or frame_i % cfg["physical"]["pose_every_n"] == 0)) \
                    or bool(bridge is not None and frame_i % bcfg["pose_every_n"] == 0)
                face_px, hands_px = vision.detect(small, t)
                body_px = getattr(vision, "last_body", None)
                det_fails = 0
                perf.mark("models")
            except Exception as exc:                            # noqa: BLE001
                det_fails += 1
                log.warning("Landmark inference failed (%d): %s", det_fails, exc)
                face_px, hands_px, body_px = None, [], None
                if det_fails >= cfg["vision"]["max_consecutive_failures"]:
                    log.warning("Re-creating the landmarker")
                    try:
                        vision.close()
                        vision = create_vision(cfg)
                    except Exception:                           # noqa: BLE001
                        log.exception("Could not re-create the landmarker")
                    det_fails = 0
            bp_cfg = cfg["body_presence"]                         # the mental algorithms see the body exactly as before algorithm 4 existed
            mental_wants_pose = bp_cfg["enabled"] and (face_px is None or monitor.posture_capture_active or not bp_cfg["run_only_when_face_lost"])
            obs: Observation = extractor.extract(t, face_px, hands_px, (inf_w, inf_h), body=(body_px if mental_wants_pose else None))
            if phys is not None:                                   # algorithm 4: own input (world landmarks), own state
                phys.feed(t, getattr(vision, "last_body_world", None))
                ui["physical"], ui["physical_bands"] = phys.update(t), cfg["physical"]["bands"]
                if phys_log is not None and phys_log.maybe_log(t, phys.log_row()):
                    ps = phys.state
                    print(f"[physical] f={ps['f']:.2f} (Borg-equiv {ps['borg_equiv']:.1f}) {ps['band_name']} mode={ps['mode']} class={ps['intensity'] or '-'}", flush=True)
            instr.poll()
            obs.instruction, obs.action_event = instr.value, instr.take_action()
            obs.collab_active = collab
            st = monitor.update(obs)
            if was_capturing and not monitor.posture_capture_active and monitor.posture.calibrated:
                try:
                    monitor.posture.save(posture_file)
                except OSError as exc:
                    log.warning("Could not save %s: %s", posture_file, exc)
            was_capturing = monitor.posture_capture_active

            perf.mark("fatigue")
            pos = depth_tracker.update(t, frame, face_px, (inf_w, inf_h), intr, cfg)
            if bridge is not None:                             # POMDP link: newest worker state + limbs to MATLAB
                # each model's landmarks are measured in the depth image nearest to the frame THEY were computed on
                depths.add(frame)
                rt = getattr(vision, "result_t", None) or {}
                fr = {k: depths.at(rt.get(k, t)) for k in ("face", "hand", "pose")}
                arms = arms_tr.update(t, fr["pose"], body_px, hands_px, (inf_w, inf_h), intr, wframe, cfg["camera"],
                                      hands_fresh=getattr(vision, "ages", {}).get("hand", 0.0) < 0.25,
                                      hand_frame=fr["hand"])
                head, head_src = head_tr.update(t, fr["face"], face_px, body_px, (inf_w, inf_h), intr, wframe, cfg["camera"],
                                                pose_frame=fr["pose"])
                rob = robot_block(robot_poller, wframe, bcfg["real_robot"]["max_age_s"])
                bridge.maybe_send(t, lambda seq: build_message(seq, st, obs, phys.state if phys is not None else None,
                                                               arms, wframe, collab, head, rob))
            if recorder is not None:                            # camera video while MATLAB records
                recorder.update(time.time(), frame.color_bgr)
            perf.mark("link+rec")

            if csv_log.maybe_log(t, {
                "level_code": st.level_code, "level": st.level, "combined_score": st.combined_score,
                "task_level": st.task["task_fatigue_level"], "task_score": st.task["task_score"],
                "visual_level": st.visual["visual_fatigue_level"], "visual_score": st.visual["visual_score"],
                "attention_level": st.attention_out["attention_fatigue_level"],
                "attention_score": st.attention_out["attention_score"],
                "attention_state": st.attention_out["attention_state"],
                "current_instruction": st.task["current_instruction"],
                "backtracking_count": st.task["backtracking_count"],
                "suspected_error_count": st.task["suspected_error_count"],
                "hesitation_count": st.task["hesitation_count"],
                "brief_disengagement_count": st.attention_out["brief_disengagement_count"],
                "sustained_disengagement_count": st.attention_out["sustained_disengagement_count"],
                "max_outside_s": st.attention_out["maximum_outside_duration_s"],
                "active_evidence": "|".join(st.active_evidence), "data_valid": st.data_valid,
                "data_warnings": "|".join(st.data_warnings), "quality": st.quality,
                "collab_active": st.collab_active, "task_engaged": st.task_engaged, "face_free": st.fallback,
                "attention": st.attention, "presence": st.presence, "immediate_distraction": st.immediate_distraction,
                "d_outside_s": st.d_outside_s, "perclos": st.raw["perclos"],
                "closure_weighted_60s": st.raw["closure"], "brief_events_60s": st.raw["brief_disengagement"],
                "sustained_max_60s_s": st.raw["sustained_disengagement"],
                "yawns_5min": st.raw["yawn"], "nods_5min_weighted": st.raw["nod"], "rubs_5min": st.raw["rub"],
                **{f"r_{k}": st.r.get(k) for k in INDICATORS},
                "ear": st.ear, "ear_threshold": st.ear_threshold, "mar": st.mar,
                "yaw_deg": st.yaw, "pitch_deg": st.pitch,
                "microsleep_count": st.microsleep_count, "microsleep_score": st.microsleep_score,
                "yawn_count": st.visual["yawn_count"], "nod_count": st.visual["nod_count"],
                "eye_rub_count": st.visual["eye_rub_count"],
                "depth_valid": pos["valid"], "distance_m": pos["distance_m"],
                "x_m": pos["x_m"], "y_m": pos["y_m"], "z_m": pos["z_m"],
            }):
                mc, ms, ml = mode_result(st, args.mode)
                print((f"level_code={mc} ({ml})" if args.mode == "all" else f"[{args.mode}] level_code={mc} ({ml}) score={ms:.2f}"),
                      flush=True)

            if last_t is not None and t > last_t:
                fps = 0.9 * fps + 0.1 / (t - last_t) if fps else 1.0 / (t - last_t)
            last_t = t
            perf.mark("log")
            ui_due = time.monotonic() - last_ui >= 1.0 / max(1.0, float(cfg["display"].get("ui_max_hz", 12.0)))
            if ui_due:
                last_ui = time.monotonic()
            dvis = None if not ui_due else depth_visual(frame.depth_mm, cfg["camera"]["depth_min_mm"], cfg["camera"]["depth_max_mm"]) \
                if show_depth else None
            ui.update(instruction=(instr.value if instr.value is not None and instr.value >= 0 else None),
                      slides_total=(instr.limit if deck.loaded else None), show_depth=show_depth)
            view = None if not ui_due else draw_ui(frame.color_bgr, st, pos, desc, fps, face_px, hands_px, (inf_w, inf_h), dvis,
                           {n: monitor.attention.range_text(n) for n in STATION_LABELS}, body_px=held_body.update(t, body_px),
                           mode=args.mode, ui=ui)
            if bridge is not None and view is not None:
                rates = vision.rates() if hasattr(vision, "rates") else {}
                if rates:
                    put(view, "loop %.0f fps | models: face %.0f  hand %.0f  pose %.0f Hz" % (
                        fps, rates.get("face", 0), rates.get("hand", 0), rates.get("pose", 0)),
                        12, view.shape[0] - 50, (0, 230, 255), 0.45)
                put(view, f"POMDP link -> {bcfg['host']}:{bcfg['port']}  sent {bridge.sent}  |  workstation frame: "
                          + (("REAL robot" if wframe.on_real_robot else "calibrated") if wframe.calibrated else
                             "NOT calibrated (run workspace_calibration.py, then restart main.py)")
                          + (f"  |  robot pose: {robot_poller.status}" if robot_poller is not None else "")
                          + (f"  |  head: {head_src or 'not seen'}" if wframe.calibrated else ""),
                    12, view.shape[0] - 30, (0, 230, 255) if wframe.calibrated else (60, 60, 255), 0.45)
                if wframe.calibrated:                          # which arm points reach MATLAB (all 4 of one arm needed)
                    ast = arm_status(arms)
                    complete = "SEWP" in ast or "SEWp" in ast
                    put(view, ast + ("" if complete else "   <- no complete arm: MATLAB cannot verify safety"),
                        12, view.shape[0] - 10, (0, 230, 255) if complete else (60, 60, 255), 0.45)
            perf.mark("draw")
            if view is not None:
                cv2.imshow(win, view)

            if deck.loaded and ui_due:
                cv2.imshow(deck_win, deck.frame(instr.value))
            kx = cv2.waitKeyEx(1)
            key = kx & 0xFF if kx != -1 else -1
            perf.mark("show")
            perf.end_frame()
            if kx in ARROW_NEXT or kx in ARROW_PREV:      # arrow keys (Windows and Linux codes)
                (instr.next if kx in ARROW_NEXT else instr.prev)()
                key = -1                                    # do not let the low byte act as a letter key
            if clicks:                                          # a button was clicked: same as pressing its key
                key = clicks.pop(0)
                clicks.clear()
            if key in (27, ord("q"), ord("Q")):
                break
            if key == -1:
                pass
            elif key in (ord("7"), ord("8"), ord("9")):
                name = {ord("7"): "task", ord("8"): "visual", ord("9"): "attention"}[key]
                if args.mode == "all":
                    monitor.set_algorithm_enabled(name, not monitor.enabled[name], t)
                else:
                    monitor._calib_msg = "Algorithms can be switched on/off in the combined program (main.py)"
            elif key == ord("0") or key in (ord("z"), ord("Z"), ord("y"), ord("Y")):
                if phys is None:
                    monitor._calib_msg = "Physical fatigue (algorithm 4) is not active (see config physical / --no-physical)"
                elif key in (ord("y"), ord("Y")):
                    b0 = ask_borg()                                    # blocking: starting fatigue, then the neutral-posture capture
                    if b0 is not None:
                        phys.set_start_rating(b0)
                    phys.start_capture(t)
                    monitor._calib_msg = "Physical baseline: sit upright, arms relaxed, hands still"
                elif key == ord("0"):
                    phys.enabled = not phys.enabled
                    monitor._calib_msg = "Physical fatigue " + ("ON" if phys.enabled else "OFF")
                elif last_z is not None and t - last_z <= 3.0:
                    phys.reset(t)
                    last_z = None
                    from fatigue_monitor import pomdp_bridge as _pb
                    _pb.FATIGUE_RESETS[0] += 1                         # MATLAB resets its fatigue monitor too
                    monitor._calib_msg = "Physical fatigue reset to 0"
                else:
                    last_z = t
                    monitor._calib_msg = "Press Z again within 3 s to RESET the physical fatigue to 0"
            elif key in (ord("u"), ord("U")):
                load_deck(ask_for_pptx(), t)
            elif key in (ord("s"), ord("S")):
                if not monitor.baseline_hold:
                    monitor._calib_msg = "Baseline already started (B = relearn it)"
                else:
                    gate["go"] = True
                    if not try_start_baseline(t):
                        monitor._calib_msg = "Load the instruction slides first (U), then the baseline starts"
                    refresh_hold_reason()
            elif key == ord("4"):
                monitor.begin_posture_calibration(t)
            elif key in (ord("k"), ord("K")):
                collab = not collab
                monitor._calib_msg = "Collaboration ACTIVE" if collab else "Collaboration INACTIVE (standby)"
            elif chr(key) in KEY_TO_STATION:
                monitor.begin_calibration(KEY_TO_STATION[chr(key)], t)
            elif key in (ord("r"), ord("R")):
                if last_r is not None and t - last_r <= 3.0:        # second press within 3 s confirms
                    monitor.reset_calibration()
                    try:
                        Path(posture_file).unlink()
                    except OSError:
                        pass
                    last_r = None
                else:
                    last_r = t
                    monitor._calib_msg = "Press R again within 3 s to ERASE the workstation calibration"
            elif key in (ord("f"), ord("F")):
                monitor.reset_fatigue(t)
                from fatigue_monitor import pomdp_bridge as _pb
                _pb.FATIGUE_RESETS[0] += 1                             # MATLAB resets its fatigue monitor too
            elif key in (ord("b"), ord("B")):
                monitor.reset_baseline(t)
            elif key in (ord("d"), ord("D")):
                show_depth = not show_depth
            elif key in (ord("n"), ord("N")):
                instr.next()
            elif key in (ord("p"), ord("P")):
                instr.prev()
            elif key in (ord("a"), ord("A")):
                instr.action("attempted")
            elif key in (ord("c"), ord("C")):
                instr.action("completed")
            if cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        log.info("Interrupted")
    except Exception:                                           # noqa: BLE001
        log.exception("Unexpected error in main loop")
        return 1
    finally:
        if vision is not None:
            vision.close()
        if source is not None:
            source.stop()
        if csv_log is not None:
            csv_log.close()
        if phys_log is not None:
            phys_log.close()
        if bridge is not None:
            bridge.close()
        if robot_poller is not None:
            robot_poller.close()
        if recorder is not None:
            recorder.close()
        deck.close()
        cv2.destroyAllWindows()
        log.info("Resources released")
    return 0


if __name__ == "__main__":
    sys.exit(main())