"""CALIBRATION WIZARD - run it at the start of every experiment (see CALIBRATION_GUIDE.md in the project folder).

    cd mental_fatigue_algorithm
    python calibration_wizard.py

It checks every calibration of the set-up, shows the live workstation (places and zones as MATLAB uses them) with the
status of each calibration next to it, and asks whether you are satisfied or want to redo a step:

  step  what                                   redo it when ...
  0     robot connection (gateway login)       the login expired / another computer
  1     camera placement                       you moved the camera (do 2 right after)
  2     robot-camera calibration  (ROBOT MOVES) the camera or the robot base moved; the live red-tip check is not GOOD
  3     workstation places                     the assembly area, storage, safe or handover point changed
  4     hands & head detection check           always a quick look; no file is changed

Keys in the window:  K / Enter  satisfied: finish     1 2 3 4  run that step     U  update the camera link of the
workstation file (offered after a new robot-camera calibration)     Q / Esc  quit without confirming

The places are stored in ROBOT coordinates, so moving the camera does NOT invalidate them: after step 2 the wizard
only updates the camera link of the workstation file (no new baselines needed), as long as the robot and the table
objects stayed where they were.
The robot moves ONLY in step 2, after you confirm with Y that nobody is within reach. Everything else only reads it.
"""
from __future__ import annotations

import datetime as dt
import json
import os
import subprocess
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from fatigue_monitor.config import load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.vision import FaceHandVision
from fatigue_monitor import robot_frame as rf
from workstation_view import WorkstationOverlay, detect_people, workspace_path

HERE = Path(__file__).resolve().parent
POSES_FILE = rf.PROJECT / "points_for_robot-human-calibration.txt"
WORKSTATION_POINTS = rf.PROJECT / "POMDP model" / "workstation_points.txt"
WIN = "CALIBRATION WIZARD"
VIEW_W, PANEL_W = 960, 560
TIP_GOOD_MM, TIP_BAD_MM = 25.0, 50.0       # live red-tip check: GOOD below, BAD above (CHECK between)
GREEN, YELLOW, RED, GREY, WHITE = (80, 220, 80), (0, 210, 255), (80, 80, 255), (170, 170, 170), (255, 255, 255)


# ---------------------------------------------------------------------------------------------------------------
# status of each calibration
# ---------------------------------------------------------------------------------------------------------------
def age_text(iso: str) -> str:
    try:
        d = dt.datetime.now() - dt.datetime.fromisoformat(iso)
    except (TypeError, ValueError):
        return "?"
    return "%.0f min ago" % (d.total_seconds() / 60) if d.days < 1 and d.total_seconds() < 5400 else \
        "%.1f h ago" % (d.total_seconds() / 3600) if d.days < 1 else "%d days ago" % d.days


def robot_link_status() -> list:
    """Step 0: gateway login, gateway state, robot state (read only)."""
    rows = []
    sys.path.insert(0, str(rf.ROBOT_CODE_DIR))
    try:
        import lite6
        cfg = lite6.load_config()
        prof = cfg["profiles"].get(cfg.get("current", "default"), {})
        exp = prof.get("expires")
        if not prof.get("token"):
            return [(RED, "not logged in: python robot_code/lite6.py login 10.13.37.101")]
        days = (exp - time.time()) / 86400 if exp else None
        rows.append((GREEN if days is None or days > 2 else YELLOW,
                     "gateway login valid until %s" % (time.strftime("%Y-%m-%d", time.localtime(exp)) if exp else "?")))
        st = lite6.connect(quiet=True).status()
        rob = st.get("robot", {})
        rows.append((GREEN if rob.get("reachable") else RED, "robot %s" % (
            "reachable (%.0f ms)" % rob.get("latency_ms", 0) if rob.get("reachable") else "NOT reachable: %s" % rob.get("error"))))
        if st.get("jobs", {}).get("halted"):
            rows.append((RED, "gateway HALTED after an emergency stop: python robot_code/lite6.py resume"))
        arm = rf.connect_robot(enable_motion=False)
        _, state = arm.get_state()
        _, (err, warn) = arm.get_err_warn_code()
        arm.disconnect()
        rows.append((RED if err else GREEN, "robot state %s%s%s" % (
            state, " (stopped - programs re-enable it)" if state == 4 else "",
            ", ERROR %s" % err if err else ", warning %s" % warn if warn else ", no error")))
    except Exception as exc:                                    # noqa: BLE001
        rows.append((RED, "robot / gateway: %s" % str(exc)[:90]))
    return rows


def link_offset(ws: dict, rframe: rf.RobotFrame):
    """(angle deg, shift mm) between the camera link stored in the workstation file and the one expected from the
    current robot-camera calibration."""
    Rwr = np.array(ws["R_w_robot"])
    R_exp, o_exp = Rwr @ rframe.R, -(rframe.R.T @ rframe.t) / 1000.0
    R_old, o_old = np.array(ws["R_wc"]), np.array(ws["origin_cam_m"])
    ang = float(np.degrees(np.arccos(np.clip((np.trace(R_old.T @ R_exp) - 1) / 2, -1, 1))))
    return ang, float(1000 * np.linalg.norm(o_exp - o_old))


def update_workspace_link(wpath: Path, rframe: rf.RobotFrame) -> str:
    """After a new robot-camera calibration: recompute the camera link (R_wc, origin_cam_m) of the workstation file;
    the places (robot frame) are kept."""
    ws = json.loads(wpath.read_text(encoding="utf-8"))
    th = float(ws.get("real_robot", {}).get("theta_deg", 0.0))
    R_wc, origin, R_w_robot, _ = rf.workspace_frame_from_robot(rframe, (np.cos(np.radians(th)), np.sin(np.radians(th)), 0.0))
    ws["R_wc"], ws["origin_cam_m"], ws["R_w_robot"] = R_wc.tolist(), origin.tolist(), R_w_robot.tolist()
    ws.setdefault("real_robot", {})["robot_camera_calibration_created"] = rframe.data.get("created")
    ws["camera_link_updated"] = dt.datetime.now().isoformat(timespec="seconds")
    wpath.write_text(json.dumps(ws, indent=2) + "\n", encoding="utf-8")
    return "camera link of the workstation file updated (places kept)"


def file_status(rframe: rf.RobotFrame, wpath: Path) -> dict:
    """Steps 2 and 3 from the files."""
    out = {"calib": [], "ws": [], "ws_obj": None, "link_stale": False}
    if not rframe.calibrated:
        out["calib"].append((RED, "no robot-camera calibration: do step 2"))
    else:
        d = rframe.data
        rms = d.get("rms_mm", float("nan"))
        out["calib"].append((GREEN if rms <= 10 else YELLOW, "saved %s (%s): fit error %.1f mm rms, %d samples" % (
            d.get("created", "?")[:16].replace("T", " "), age_text(d.get("created")), rms, len(d.get("samples", [])))))
    n_poses = 0
    if POSES_FILE.exists():
        n_poses = sum(1 for ln in POSES_FILE.read_text(encoding="utf-8").splitlines() if ln.strip().startswith("pos"))
    out["calib"].append((GREEN if n_poses >= 6 else RED, "%s: %d poses" % (POSES_FILE.name, n_poses)))
    if not wpath.exists():
        out["ws"].append((RED, "no workstation file: do step 3"))
        return out
    ws = json.loads(wpath.read_text(encoding="utf-8"))
    if "R_w_robot" not in ws:
        out["ws"].append((RED, "workstation file is not on the real robot (old stand-in method): do step 3"))
        return out
    out["ws_obj"] = ws
    out["ws"].append((GREEN, "saved %s (%s), %s" % (ws.get("created", "?")[:16].replace("T", " "), age_text(ws.get("created")),
                                                   ws.get("method", ""))))
    ab, sb = ws.get("assembly_baseline", {}), ws.get("storage_baseline", {})
    out["ws"].append((GREEN if ab else YELLOW, "assembly area: %s" % (
        "hand baseline, %s hand points" % ab.get("hand_points", "?") if ab else "no baseline")))
    out["ws"].append((WHITE, "storage: %s" % ws.get("places_W", {}).get("storage_mode", sb.get("method", "recorded points"))))
    z = ws.get("zones", {})
    out["ws"].append((WHITE, "zones: collaborative %.2f m, assistive %.3f m" % (z.get("dCollabMax", 0), z.get("assistRadius", 0.12))))
    for name, ok in ws.get("checks", {}).items():
        if not ok:
            out["ws"].append((YELLOW, "warning: " + name))
    if rframe.calibrated:
        ang, shift = link_offset(ws, rframe)
        if ang > 0.5 or shift > 5:
            out["link_stale"] = True
            out["ws"].append((RED, "camera link OUT OF DATE (%.1f deg, %.0f mm): press U" % (ang, shift)))
        else:
            out["ws"].append((GREEN, "camera link matches the robot-camera calibration"))
    return out


# ---------------------------------------------------------------------------------------------------------------
# live red-tip check: is the robot-camera calibration still right?
# ---------------------------------------------------------------------------------------------------------------
class TipCheck:
    """Predicted red tip (robot pose + calibration) vs the red tip the camera sees: 3-D error, rolling median."""

    def __init__(self, rframe: rf.RobotFrame, intr, cc) -> None:
        self.rframe, self.intr, self.cc = rframe, intr, cc
        self.offset = np.array(rframe.data.get("marker_offset_tool_mm", [0.0, 0.0, 0.0]))
        self.err = deque(maxlen=15)
        self.pred, self.seen = None, None

    def update(self, f, last) -> None:
        self.pred = self.seen = None
        if last is None or "pose" not in last or time.monotonic() - last["t"] > 1.0:
            return
        pose = last["pose"]
        tip = np.array(pose[:3]) + rf.rpy_matrix(*pose[3:6]) @ self.offset
        self.pred = rf.project(self.rframe.to_cam(tip), self.intr)
        if self.pred is None:
            return
        m = rf.red_mask(f.color_bgr)
        n, _, st, ce = cv2.connectedComponentsWithStats(m)
        best = None
        for i in range(1, n):
            if st[i, cv2.CC_STAT_AREA] >= 30:
                d = float(np.hypot(ce[i][0] - self.pred[0], ce[i][1] - self.pred[1]))
                if d <= 150 and (best is None or d < best[0]):
                    best = (d, ce[i])
        if best is None:
            return
        u, v = best[1]
        p = rf.pixel_point_mm(f.depth_mm, u, v, self.intr, 3, self.cc["depth_min_mm"], self.cc["depth_max_mm"], percentile=20)
        if p is None:
            return
        self.seen = (int(u), int(v))
        self.err.append(float(np.linalg.norm(self.rframe.to_robot(p) - tip)))

    def status(self):
        if len(self.err) < 5:
            return GREY, "red tip not seen near its predicted position (robot pose hides it, or the camera moved)"
        e = float(np.median(self.err))
        if e <= TIP_GOOD_MM:
            return GREEN, "live red-tip check GOOD: %.0f mm" % e
        if e <= TIP_BAD_MM:
            return YELLOW, "live red-tip check CHECK: %.0f mm (> %.0f)" % (e, TIP_GOOD_MM)
        return RED, "live red-tip check BAD: %.0f mm - camera moved? redo step 2" % e

    def draw(self, img) -> None:
        if self.pred:
            cv2.circle(img, self.pred, 16, (255, 0, 255), 2)
        if self.seen:
            cv2.drawMarker(img, self.seen, (0, 255, 0), cv2.MARKER_CROSS, 24, 2)


# ---------------------------------------------------------------------------------------------------------------
class Wizard:
    def __init__(self) -> None:
        self.cfg = load_config(str(HERE / "config.json"))
        self.cfg["body_presence"]["enabled"] = True
        self.cfg.setdefault("physical", {})["enabled"] = False
        self.cc = self.cfg["camera"]
        self.wpath = workspace_path(self.cfg)
        self.cam = self.vision = self.poller = None
        self.msg = ""
        self.refresh_status()

    # ---- resources (the camera can be used by one program at a time)
    def open(self) -> None:
        self.cam = OrbbecRGBD(self.cc)
        self.cam.start()
        self.intr = self.cam.intrinsics
        self.vision = FaceHandVision(self.cfg)
        self.poller = rf.RobotPoller(10)
        self.reload()

    def close(self) -> None:
        for obj in (self.poller, self.vision):
            try:
                if obj is not None:
                    obj.close()
            except Exception:                                   # noqa: BLE001
                pass
        if self.cam is not None:
            self.cam.stop()
        self.cam = self.vision = self.poller = None
        cv2.destroyAllWindows()

    def reload(self) -> None:
        self.rframe = rf.RobotFrame(rf.CALIB_FILE)
        self.files = file_status(self.rframe, self.wpath)
        self.overlay = WorkstationOverlay(self.files["ws_obj"], self.rframe, self.intr) \
            if self.files["ws_obj"] is not None and self.rframe.calibrated and self.cam is not None else None
        self.tip = TipCheck(self.rframe, self.intr, self.cc) if self.rframe.calibrated and self.cam is not None else None
        self.hand_seen, self.head_seen = deque(maxlen=60), deque(maxlen=60)

    def refresh_status(self) -> None:
        self.robot_rows = robot_link_status()

    # ---- steps
    def run_step(self, script: str, args=(), title: str = "") -> int:
        print("\n=== %s ===\n" % title)
        self.close()
        rc = subprocess.call([sys.executable, str(HERE / script)] + list(args), cwd=str(HERE))
        print("=== %s finished (exit code %s) ===" % (title, rc))
        self.refresh_status()
        self.open()
        return rc

    def confirm_robot_motion(self) -> bool:
        """Step 2 moves the robot: explicit confirmation in the window."""
        lines = ["STEP 2 - ROBOT-CAMERA CALIBRATION: THE ROBOT WILL MOVE",
                 "", "The robot visits the poses of %s" % POSES_FILE.name,
                 "at 10 deg/s (about 3 minutes) and the camera follows its red tip.", "",
                 "Before you press Y:",
                 "  - nobody in the camera image and nobody within reach of the robot",
                 "  - the camera is fixed in its final place (step 1)",
                 "  - someone can press STOP on the gateway page (or Esc in the calibration window)",
                 "", "Y = start the robot      any other key = cancel"]
        img = np.zeros((420, 900, 3), np.uint8)
        for i, ln in enumerate(lines):
            rf.put_text(img, ln, (20, 40 + 32 * i), RED if i == 0 else WHITE, 0.7, 2 if i == 0 else 1)
        cv2.imshow(WIN, img)
        return (cv2.waitKey(0) & 0xFF) in (ord("y"), ord("Y"))

    # ---- panel
    def panel(self, h: int, live_people: str) -> np.ndarray:
        img = np.full((h, PANEL_W, 3), 30, np.uint8)
        y = [24]

        def put(text, col=WHITE, scale=0.5, bold=False, indent=0):
            rf.put_text(img, text, (10 + indent, y[0]), col, scale, 2 if bold else 1)
            y[0] += 22 if not bold else 26

        put("CALIBRATION STATUS", WHITE, 0.65, True)
        put("0  Robot connection", WHITE, 0.55, True)
        for col, t in self.robot_rows:
            put(t, col, indent=14)
        put("1  Camera placement", WHITE, 0.55, True)
        put("camera %s, depth aligned" % (self.cam.color_profile_desc if self.cam else "?"), GREEN, indent=14)
        put("redo when the robot, storage or your hands are not in the view", GREY, 0.45, indent=14)
        put("2  Robot-camera calibration (robot moves)", WHITE, 0.55, True)
        for col, t in self.files["calib"]:
            put(t, col, indent=14)
        if self.tip is not None:
            col, t = self.tip.status()
            put(t, col, indent=14)
            put("magenta circle = predicted tip, green cross = seen", GREY, 0.45, indent=14)
        put("3  Workstation places", WHITE, 0.55, True)
        for col, t in self.files["ws"]:
            put(t, col, indent=14)
        put("4  Hands & head detection", WHITE, 0.55, True)
        put(live_people, GREEN if "hands" in live_people and "no " not in live_people else YELLOW, indent=14)
        y[0] += 6
        problems = [t for col, t in self.robot_rows + self.files["calib"] + self.files["ws"] if col == RED]
        if self.tip is not None and self.tip.status()[0] == RED:
            problems.append("red-tip check")
        put("VERDICT: %s" % ("ready - press K if you are satisfied" if not problems else
                             "%d item(s) need attention (red)" % len(problems)), GREEN if not problems else RED, 0.6, True)
        put("K/Enter satisfied   1 camera   2 robot-camera   3 places", WHITE, 0.5)
        put("4 hands & head   U update camera link   Q quit", WHITE, 0.5)
        if self.msg:
            put(self.msg, YELLOW, 0.48)
        return img

    def report(self) -> None:
        print("\nCALIBRATION STATUS")
        for title, rows in (("0 robot connection", self.robot_rows), ("2 robot-camera calibration", self.files["calib"]),
                            ("3 workstation places", self.files["ws"])):
            print("  " + title)
            for col, t in rows:
                print("     %s %s" % ({GREEN: "OK  ", YELLOW: "WARN", RED: "NO  "}.get(col, "    "), t))

    # ---- main loop
    def run(self) -> int:
        self.open()
        self.report()
        k = 0
        try:
            while True:
                f = self.cam.read()
                if f is None or f.depth_mm is None or not f.depth_aligned:
                    if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                        return 1
                    continue
                k += 1
                self.vision.force_pose = (k % 2 == 0)
                palms, head = detect_people(f, self.vision, self.rframe, self.intr, self.cc)
                self.hand_seen.append(len(palms) > 0)
                self.head_seen.append(head is not None)
                last = self.poller.latest()
                tool = last["pts"]["tool"] if last is not None and time.monotonic() - last["t"] < 1.0 else None
                if self.tip is not None:
                    self.tip.update(f, last)
                img = f.color_bgr.copy()
                rows = self.overlay.draw(img, palms, tool, head) if self.overlay is not None else []
                if self.tip is not None:
                    self.tip.draw(img)
                img = cv2.resize(img, (VIEW_W, int(img.shape[0] * VIEW_W / img.shape[1])))
                if self.overlay is not None:
                    rf.put_text(img, self.overlay.legend(), (8, 20), WHITE, 0.42)
                    rf.put_text(img, self.overlay.places_text(), (8, 40), WHITE, 0.42)
                for i, r in enumerate(rows[:4]):
                    rf.put_text(img, r, (8, 60 + 20 * i), WHITE, 0.45)
                people = "last 60 frames: hands in %d%%, head in %d%%" % (
                    100 * np.mean(self.hand_seen), 100 * np.mean(self.head_seen))
                if not any(self.hand_seen):
                    people += " - no hands seen yet"
                canvas = np.hstack([img, self.panel(img.shape[0], people)])
                cv2.imshow(WIN, canvas)
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    print("\nquit without confirming the calibration")
                    return 1
                if key in (13, ord("k")):
                    self.report()
                    print("\nCALIBRATION CONFIRMED - next: python main.py (this folder), then liveHRC in MATLAB (POMDP model).")
                    return 0
                if key == ord("1"):
                    self.run_step("camera_preview.py", title="STEP 1 camera placement (Q closes it)")
                    self.msg = "camera moved? then do step 2 (robot-camera calibration)"
                elif key == ord("2"):
                    if not self.confirm_robot_motion():
                        self.msg = "step 2 cancelled - the robot did not move"
                        continue
                    before = rf.RobotFrame(rf.CALIB_FILE).data.get("created")
                    self.run_step("robot_camera_calibration.py", ["--red-tip", "--auto", "--poses", str(POSES_FILE)],
                                  "STEP 2 robot-camera calibration (Esc in its window stops the robot)")
                    if self.rframe.calibrated and self.rframe.data.get("created") != before and self.wpath.exists():
                        self.msg = update_workspace_link(self.wpath, self.rframe)
                        self.reload()
                    elif self.rframe.data.get("created") == before:
                        self.msg = "step 2: no new calibration saved (see the terminal)"
                elif key == ord("3"):
                    self.run_step("define_workstation.py", title="STEP 3 workstation places (B, S or R, then V; Q closes)")
                elif key == ord("4"):
                    self.run_step("robot_frame_live.py", title="STEP 4 hands & head check (Q closes)")
                elif key == ord("u"):
                    if self.files["ws_obj"] is not None and self.rframe.calibrated:
                        self.msg = update_workspace_link(self.wpath, self.rframe)
                        self.reload()
        finally:
            self.close()


def main() -> int:
    os.chdir(HERE)
    print(__doc__)
    return Wizard().run()


if __name__ == "__main__":
    sys.exit(main())
