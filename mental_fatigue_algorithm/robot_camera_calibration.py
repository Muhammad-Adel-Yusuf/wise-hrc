"""Calibrate the depth camera with respect to the REAL robot (UFactory Lite 6): eye-to-hand calibration.

    python robot_camera_calibration.py            the program moves the robot through 12 poses around its start pose
    python robot_camera_calibration.py --manual   the program never moves the robot: you move it (UFactory Studio)
                                                  and press SPACE at each pose
    python robot_camera_calibration.py --red-tip --auto
                                                  no marker: the red tip of the tool is the target; the program
                                                  lifts the tool START_LIFT_MM, runs all poses, saves if good, quits

Result: POMDP model/robot_camera_calibration.json  (p_robot_mm = R @ p_cam_mm + t). Check it live with
    python robot_frame_live.py

Before you start
  1. Print aruco_marker_6x6_id23.png (written next to this file on the first run; black square about 5 cm), glue it
     on a flat card and fix it FIRMLY on the robot tool, facing the camera. It may sit anywhere on the tool: the
     offset between the marker and the tool point is part of the fit. No marker? Click the tool tip in the image
     before each capture instead (less accurate).
  2. Put the robot (UFactory Studio, or a task file) in a pose where the marker is well inside the image, about
     10 cm above anything below the tool. That is the START pose: the program never goes lower than it.
  3. Nobody within reach of the robot. The gateway STOP button / `python robot_code/lite6.py estop` stops it.

Keys (window "robot-camera calibration")
  N  move to the next planned pose and capture        A  all remaining planned poses (auto)
  SPACE  capture at the current pose                  click  use this pixel instead of the marker (next capture)
  U  undo the last sample    H  back to the start pose    S  save    Q / Esc  stop the robot and quit
The robot moves at 50 mm/s. Esc during a motion stops it at once.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from fatigue_monitor.config import ConfigError, load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor import geometry as geo
from fatigue_monitor import robot_frame as rf

# ======================= settings you can change =========================
MARKER_DICT = cv2.aruco.DICT_6X6_250
MARKER_ID = 23
SPEED_MM_S = 50                 # Cartesian speed of the straight-up lift (--auto)
ACC_MM_S2 = 300
JOINT_SPEED_DEG_S = 10          # joint speed of the calibration poses
JOINT_ACC_DEG_S2 = 100
CAPTURE_FRAMES = 15             # frames averaged per sample
MIN_SAMPLES = 6                 # save allowed from this many samples
MIN_SPREAD_DEG = 15.0           # tool rotation needed to fit the marker offset
# planned poses = joint changes from the START joints [deg] (J1..J6). Joint moves never hit a kinematic error (21).
# Every pose and 4 points on its way are checked with the controller's kinematics: the tool never goes lower than
# the START pose and stays in the safety box; a pose that fails is tried with J2/J3 mirrored, else skipped.
JOINT_OFFSETS = [(0, 0, 0, 0, 0, 0),
                 (10, 0, 0, 0, 0, 0), (-10, 0, 0, 0, 0, 0),
                 (0, -6, -6, 0, 0, 0), (5, -8, 0, 0, 10, 20), (-5, -8, 0, 0, -10, -20),
                 (8, 0, -8, 10, 0, -30), (-8, 0, -8, -10, 0, 30), (0, -4, -10, 0, 15, 0),
                 (0, -10, -4, 0, -15, 0), (12, -5, -5, 0, 0, 40), (-12, -5, -5, 0, 0, -40)]
MIN_PATH_Z_MM = 80.0            # --poses: the tool never below this on the way (checked with the kinematics)
START_LIFT_MM = 80              # --auto: the START pose is the current pose lifted this much (straight up)
AUTO_SAVE_RMS_MM = 12.0         # --auto: save only below this fit error
RED_MIN_AREA_PX = 60
DISPLAY_SCALE = 0.75
# =========================================================================

HERE = Path(__file__).resolve().parent
MARKER_PNG = HERE / ("aruco_marker_6x6_id%d.png" % MARKER_ID)
WIN = "robot-camera calibration"
log = logging.getLogger("robot_cam_calib")


def write_marker_png() -> None:
    if MARKER_PNG.exists():
        return
    d = cv2.aruco.getPredefinedDictionary(MARKER_DICT)
    m = cv2.aruco.generateImageMarker(d, MARKER_ID, 600)
    page = np.full((900, 800), 255, np.uint8)
    page[100:700, 100:700] = m
    cv2.putText(page, "ArUco 6x6 id %d - print the black square ~5 cm wide" % MARKER_ID, (40, 780),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, 0, 2)
    cv2.imwrite(str(MARKER_PNG), page)
    print("marker to print: %s" % MARKER_PNG)


class MarkerFinder:
    def __init__(self) -> None:
        d = cv2.aruco.getPredefinedDictionary(MARKER_DICT)
        self.det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())

    def find(self, bgr: np.ndarray) -> Optional[np.ndarray]:
        """The 4 corners (4,2) of the marker, or None (also when it is seen twice)."""
        corners, ids, _ = self.det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
        if ids is None:
            return None
        hits = [c.reshape(4, 2) for c, i in zip(corners, ids.ravel()) if i == MARKER_ID]
        return hits[0] if len(hits) == 1 else None


class RedTipFinder:
    """The red tool tip, without a marker: the red blob that is NOT where red was before the last robot move
    (table edges, skin, cables stay where they are; only the tool moves)."""

    def __init__(self) -> None:
        self.static: Optional[np.ndarray] = None

    @staticmethod
    def mask(bgr: np.ndarray) -> np.ndarray:
        return rf.red_mask(bgr)

    def remember(self, bgr: np.ndarray) -> None:
        """Call with the robot standing still, right before it moves."""
        self.static = cv2.dilate(self.mask(bgr), np.ones((25, 25), np.uint8))

    def find(self, bgr: np.ndarray, near: Optional[tuple] = None) -> Optional[tuple]:
        """Largest new red blob; with near (predicted pixel), the new blob closest to it (within 100 px)."""
        if self.static is None:
            return None
        m = cv2.bitwise_and(self.mask(bgr), cv2.bitwise_not(self.static))
        n, _, stats, cent = cv2.connectedComponentsWithStats(m)
        blobs = [(stats[i, cv2.CC_STAT_AREA], cent[i]) for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= RED_MIN_AREA_PX]
        if not blobs:
            return None
        if near is not None:
            d, c = min((float(np.hypot(c[0] - near[0], c[1] - near[1])), c) for _, c in blobs)
            if d > 100:
                return None
        else:
            area, c = max(blobs, key=lambda b: b[0])
        return int(round(c[0])), int(round(c[1]))


def marker_point_mm(depth: np.ndarray, corners: np.ndarray, intr, dmin, dmax) -> Optional[np.ndarray]:
    """Marker centre in the camera (mm): median depth over the inner 60 % of the marker."""
    c = corners.mean(axis=0)
    inner = (c + 0.6 * (corners - c)).astype(np.int32)
    mask = np.zeros(depth.shape[:2], np.uint8)
    cv2.fillConvexPoly(mask, inner, 1)
    vals = depth[(mask > 0) & (depth >= dmin) & (depth <= dmax)]
    if vals.size < 20:
        return None
    h, w = depth.shape[:2]
    sx, sy = w / intr.width, h / intr.height
    return geo.deproject(float(c[0]), float(c[1]), float(np.median(vals)),
                            intr.fx * sx, intr.fy * sy, intr.cx * sx, intr.cy * sy)


class Calibrator:
    def __init__(self, cam: OrbbecRGBD, cam_cfg, arm, manual: bool, red_tip: bool = False, start=None,
                 poses_file: Optional[str] = None) -> None:
        self.cam, self.cc, self.arm, self.manual = cam, cam_cfg, arm, manual
        self.poses_file = poses_file
        self.joint_speed = JOINT_SPEED_DEG_S
        self.finder = MarkerFinder()
        self.red = RedTipFinder() if red_tip else None
        self.tip: Optional[tuple] = None                         # red-tip pixel of the current frame
        self.predict_uv: Optional[tuple] = None                  # where the fit expects the target (capture)
        self.image_dir = rf.CALIB_FILE.parent / "robot_camera_calibration_images"
        self.samples: List[dict] = []
        self.fit: Optional[dict] = None
        self.click: Optional[tuple] = None
        self.msg = ""
        self.frame = None
        self.corners = None
        self.start = start or rf.tool_pose(arm)
        self.plan: List[dict] = [] if start else self._plan()   # --auto: planned after the lift
        self.next_i = 0
        self.quit = False

    # ---- planned poses
    def joints(self) -> Optional[List[float]]:
        code, q = self.arm.get_servo_angle(is_radian=False)
        return [float(v) for v in q[:6]] if code == 0 else None

    def fk(self, q) -> Optional[List[float]]:
        code, pose = self.arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
        return [float(v) for v in pose[:6]] if code == 0 else None

    def _file_plan(self) -> Optional[List[dict]]:
        """Poses of a task file (robot_code format, e.g. tasks/calibration_poses.txt), in order. The whole way from
        the current joints through every pose is checked (21 points per move): inside the safety box and never
        below MIN_PATH_Z_MM. None (nothing moves) if any point fails."""
        sys.path.insert(0, str(rf.ROBOT_CODE_DIR))
        import robot_code
        try:
            task = robot_code.read_task_file(self.poses_file)
        except ValueError as e:
            print("pose file: %s" % e)
            return None
        self.joint_speed = task["speed"]
        plan, prev = [], self.joints()
        for name, q in zip(task["names"], task["positions"]):
            for f in np.linspace(0, 1, 21):
                w = self.fk([a + f * (b - a) for a, b in zip(prev, q)])
                if w is None or w[2] < MIN_PATH_Z_MM or not rf.in_safety_box(w, margin=0):
                    print("NOT moving: on the way to %s the tool would reach %s (min z %.0f mm, safety box)" % (
                        name, w and [round(v) for v in w[:3]], MIN_PATH_Z_MM))
                    return None
            plan.append({"joints": q, "pose": self.fk(q), "name": name})
            prev = q
        print("%d poses from %s at %g deg/s, way checked" % (len(plan), self.poses_file, self.joint_speed))
        return plan

    def _plan(self) -> List[dict]:
        """Joint targets around the START joints, checked along the way (see JOINT_OFFSETS)."""
        q0, z0 = self.joints(), self.start[2]
        plan = []
        for d in JOINT_OFFSETS:
            for dd in (d, (d[0], -d[1], -d[2]) + tuple(d[3:])):
                q = [a + b for a, b in zip(q0, dd)]
                way = [self.fk([a + f * b for a, b in zip(q0, dd)]) for f in (0.25, 0.5, 0.75, 1.0)]
                if all(w is not None and w[2] >= z0 - 1.0 and rf.in_safety_box(w) for w in way):
                    plan.append({"joints": q, "pose": way[-1]})
                    break
            else:
                print("joint offsets %s skipped: the tool would go lower than the start or leave the box" % (d,))
        return plan

    # ---- camera + window
    def grab(self):
        f = self.cam.read()
        if f is not None and f.depth_mm is not None and f.depth_aligned:
            self.frame = f
            if self.red is not None:
                self.tip = self.red.find(f.color_bgr, self.predict_uv)
            else:
                self.corners = self.finder.find(f.color_bgr)
        return self.frame

    def show(self, status: str = "") -> int:
        f = self.frame
        if f is None:
            return cv2.waitKey(30) & 0xFF
        img = f.color_bgr.copy()
        intr = self.cam.intrinsics
        if self.corners is not None:
            cv2.polylines(img, [self.corners.astype(np.int32)], True, (0, 255, 0), 3)
        if self.click is not None:
            cv2.drawMarker(img, self.click, (0, 0, 255), cv2.MARKER_CROSS, 30, 3)
        if self.tip is not None:
            cv2.circle(img, self.tip, 10, (0, 255, 0), 2)
        if self.fit is not None:                                 # where the fit puts the tool point / marker
            frame = rf.RobotFrame(None)
            frame.R, frame.t = self.fit["R"], self.fit["t"]
            pose = rf.tool_pose(self.arm)
            if pose:
                Rt = rf.rpy_matrix(*pose[3:])
                for p, col in ((np.array(pose[:3]), (255, 0, 255)),
                               (np.array(pose[:3]) + Rt @ self.fit["offset_mm"], (0, 255, 255))):
                    uv = rf.project(frame.to_cam(p), intr)
                    if uv:
                        cv2.circle(img, uv, 12, col, 3)
        lines = [
            "marker: %s   samples: %d   next planned pose: %s" % (
                "FOUND" if self.corners is not None or self.tip is not None else ("click point set" if self.click else "not found"),
                len(self.samples), "-" if self.manual else "%d/%d" % (min(self.next_i + 1, len(self.plan)), len(self.plan))),
            ("SPACE capture  click=use pixel  U undo  S save  Q quit" if self.manual else
             "N next pose  A all poses  SPACE capture  click=use pixel  U undo  H start pose  S save  Q quit"),
        ]
        if self.fit is not None:
            lines.append("fit: rms %.1f mm  max %.1f mm  offset marker->tool [%.0f %.0f %.0f] mm%s   (circles: magenta tool,"
                         " yellow marker)" % (self.fit["rms_mm"], self.fit["max_mm"], *self.fit["offset_mm"],
                                              "" if self.fit["offset_fitted"] else " NOT FITTED"))
        if status or self.msg:
            lines.append(status or self.msg)
        y = 30
        for ln in lines:
            rf.put_text(img, ln, (12, y), scale=0.6)
            y += 26
        cv2.imshow(WIN, cv2.resize(img, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE))
        return cv2.waitKey(1) & 0xFF

    def on_mouse(self, event, x, y, flags, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.click = (int(x / DISPLAY_SCALE), int(y / DISPLAY_SCALE))

    # ---- robot
    def stop_robot(self) -> None:
        self.arm.set_state(4)
        time.sleep(0.3)
        self.arm.set_mode(0)
        self.arm.set_state(0)

    def move(self, target) -> bool:
        """target: plan entry {"joints", "pose"} (joint move) or a Cartesian pose (straight move). The window stays
        alive; Esc/Q stops. True when the target was reached."""
        q = target["joints"] if isinstance(target, dict) else None
        pose = target["pose"] if isinstance(target, dict) else target
        if self.red is not None:                                 # red seen now (robot still) = not the target
            for _ in range(3):
                self.grab()
            self.red.remember(self.frame.color_bgr)
        if q is not None:
            code = self.arm.set_servo_angle(angle=q, speed=self.joint_speed, mvacc=JOINT_ACC_DEG_S2, wait=False)
        else:
            code = self.arm.set_position(*pose, speed=SPEED_MM_S, mvacc=ACC_MM_S2, wait=False)
        if code == 10:
            self.msg = "the gateway guard refused the motion (code 10)"
            return False
        if code not in (0, 1, 2):
            self.msg = "motion command failed (code %s)" % code
            return False
        t0, idle = time.time(), None
        while True:
            self.grab()
            k = self.show("moving to [%.0f %.0f %.0f] ... Esc stops" % tuple(pose[:3]))
            if k in (27, ord("q")):
                self.stop_robot()
                self.msg = "STOPPED by the user"
                return False
            _, (err, _) = self.arm.get_err_warn_code()
            if err:
                self.stop_robot()
                self.msg = "ROBOT ERROR %s: check the robot (clear it in Studio), then go on" % err
                return False
            _, state = self.arm.get_state()
            if state != 1 and time.time() - t0 > 0.4:
                idle = idle or time.time()
                if time.time() - idle > 0.5:
                    now = rf.tool_pose(self.arm)
                    if q is not None:
                        qn = self.joints()
                        if qn and max(abs(a - b) for a, b in zip(qn, q)) < 0.5:
                            return True
                    elif now and np.linalg.norm(np.array(now[:3]) - pose[:3]) < 2.0:
                        return True
                    self.msg = "the robot stopped short of the pose (%s)" % (now and [round(v) for v in now[:3]])
                    return False
            else:
                idle = None
            if time.time() - t0 > 30:
                self.stop_robot()
                self.msg = "move timeout"
                return False

    # ---- samples
    def capture(self) -> bool:
        pose_a = rf.tool_pose(self.arm)
        self.predict_uv = None
        if self.fit is not None and len(self.samples) >= 6 and pose_a:
            frame = rf.RobotFrame(None)
            frame.R, frame.t = self.fit["R"], self.fit["t"]
            target = np.array(pose_a[:3]) + rf.rpy_matrix(*pose_a[3:]) @ self.fit["offset_mm"]
            self.predict_uv = rf.project(frame.to_cam(target), self.cam.intrinsics)
        try:
            return self._capture(pose_a)
        finally:
            self.predict_uv = None

    def _capture(self, pose_a) -> bool:
        pts = []
        for _ in range(CAPTURE_FRAMES * 3):
            f = self.grab()
            self.show("capturing ...")
            if f is None:
                continue
            p = None
            if self.click is not None:
                p = rf.pixel_point_mm(f.depth_mm, self.click[0], self.click[1], self.cam.intrinsics, 3,
                                      self.cc["depth_min_mm"], self.cc["depth_max_mm"], percentile=20)
            elif self.red is not None:
                if self.tip is not None:
                    p = rf.pixel_point_mm(f.depth_mm, self.tip[0], self.tip[1], self.cam.intrinsics, 3,
                                          self.cc["depth_min_mm"], self.cc["depth_max_mm"], percentile=20)
            elif self.corners is not None:
                p = marker_point_mm(f.depth_mm, self.corners, self.cam.intrinsics,
                                    self.cc["depth_min_mm"], self.cc["depth_max_mm"])
            if p is not None:
                pts.append(p)
            if len(pts) >= CAPTURE_FRAMES:
                break
        pose_b = rf.tool_pose(self.arm)
        if len(pts) < CAPTURE_FRAMES // 2:
            self.msg = "capture failed: marker / depth not seen (%d frames)" % len(pts)
            return False
        if pose_a is None or pose_b is None or np.linalg.norm(np.subtract(pose_a[:3], pose_b[:3])) > 1.0:
            self.msg = "capture failed: the robot moved during the capture"
            return False
        pts = np.array(pts)
        p = np.median(pts, axis=0)
        spread = float(np.max(np.linalg.norm(pts - p, axis=1)))
        if spread > 15:
            self.msg = "capture failed: unstable depth (%.0f mm spread) - is the marker flat and facing the camera?" % spread
            return False
        source = "click" if self.click else ("red tip" if self.red is not None else "marker")
        self.samples.append({"cam_mm": p.tolist(), "tool_pose": pose_b, "source": source, "spread_mm": round(spread, 1)})
        self._save_image(p)
        self.click = None
        self.msg = "sample %d: camera [%.0f %.0f %.0f] mm, tool [%.0f %.0f %.0f] mm" % (
            len(self.samples), *p, *pose_b[:3])
        print(self.msg)
        self.refit()
        return True

    def _save_image(self, p_cam) -> None:
        """The frame of each sample with the target used: to check the detections afterwards."""
        img = self.frame.color_bgr.copy()
        uv = rf.project(p_cam, self.cam.intrinsics)
        if uv:
            cv2.drawMarker(img, uv, (0, 255, 0), cv2.MARKER_CROSS, 40, 2)
        self.image_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.image_dir / ("sample_%02d.jpg" % len(self.samples))), img)

    def refit(self) -> None:
        if len(self.samples) < 4:
            self.fit = None
            return
        cam = np.array([s["cam_mm"] for s in self.samples])
        pos = np.array([s["tool_pose"][:3] for s in self.samples])
        rot = np.array([rf.rpy_matrix(*s["tool_pose"][3:]) for s in self.samples])
        spread = rf.rotation_spread_deg(rot)
        fit_offset = spread >= MIN_SPREAD_DEG and len(self.samples) >= 6
        self.fit = rf.solve_eye_to_hand(cam, pos, rot, fit_offset=fit_offset)
        self.fit.update(offset_fitted=fit_offset, rotation_spread_deg=spread)
        print("fit with %d samples: rms %.1f mm, max %.1f mm, errors %s" % (
            len(self.samples), self.fit["rms_mm"], self.fit["max_mm"], np.round(self.fit["errors_mm"], 1).tolist()))

    def save(self) -> None:
        if self.fit is None or len(self.samples) < MIN_SAMPLES:
            self.msg = "need at least %d samples to save" % MIN_SAMPLES
            return
        if not self.fit["offset_fitted"]:
            self.msg = "NOT saved: rotate the tool more (spread %.0f deg < %.0f) so the marker offset can be fitted" % (
                self.fit["rotation_spread_deg"], MIN_SPREAD_DEG)
            return
        frame = rf.RobotFrame(None)
        frame.R, frame.t = self.fit["R"], self.fit["t"]
        intr = self.cam.intrinsics
        frame.save(rf.CALIB_FILE, {
            "rms_mm": round(self.fit["rms_mm"], 2), "max_mm": round(self.fit["max_mm"], 2),
            "marker_offset_tool_mm": np.round(self.fit["offset_mm"], 1).tolist(),
            "rotation_spread_deg": round(self.fit["rotation_spread_deg"], 1),
            "errors_mm": np.round(self.fit["errors_mm"], 2).tolist(),
            "camera": {"profile": self.cam.color_profile_desc, "fx": intr.fx, "fy": intr.fy, "cx": intr.cx, "cy": intr.cy},
            "target": "red tool tip" if self.red is not None else "ArUco 6x6_250 id %d" % MARKER_ID,
            "samples": self.samples})
        self.msg = "SAVED %s (rms %.1f mm)" % (rf.CALIB_FILE, self.fit["rms_mm"])
        print(self.msg)

    # ---- automatic run (--auto)
    def run_auto(self) -> bool:
        """Lift to START, every planned pose, drop outliers, back to START, save if the fit is good."""
        cv2.namedWindow(WIN)
        if self.poses_file:
            return self._run_poses()
        here = rf.tool_pose(self.arm)
        if np.linalg.norm(np.subtract(here[:3], self.start[:3])) > 1.0:
            print("lifting the tool to the start pose %s" % [round(v, 1) for v in self.start])
            if not self.move(self.start):
                print(self.msg)
                return False
        else:
            print("start pose = current pose %s" % [round(v, 1) for v in here])
        self.plan = self._plan()
        if not self._visit_all():
            return False
        print("back to the start pose")
        self.move(self.plan[0] if self.plan else self.start)
        return self._finish()

    def _run_poses(self) -> bool:
        """--poses: the recorded poses in order; the robot stays at the last one."""
        self.plan = self._file_plan()
        if not self.plan or not self._visit_all():
            return False
        return self._finish()

    def _visit_all(self) -> bool:
        for i, target in enumerate(self.plan):
            here = self.joints()
            if max(abs(a - b) for a, b in zip(here, target["joints"])) > 0.3:   # pose 0 = start: no move needed
                print("pose %d/%d: joints %s -> tool %s" % (i + 1, len(self.plan), [round(v, 1) for v in target["joints"]],
                                                          [round(v, 1) for v in target["pose"][:3]]))
                if not self.move(target):
                    print("aborted: %s" % self.msg)
                    return False
            time.sleep(0.5)                                      # let the arm settle
            if not self.capture():
                print("pose %d: %s" % (i + 1, self.msg))
        return True

    def _finish(self) -> bool:
        self.drop_outliers()
        if self.fit is None or self.fit["rms_mm"] > AUTO_SAVE_RMS_MM:
            print("NOT saved: fit error %s mm (limit %.0f)" % (self.fit and round(self.fit["rms_mm"], 1), AUTO_SAVE_RMS_MM))
            return False
        self.save()
        return rf.CALIB_FILE.exists() and self.msg.startswith("SAVED")

    def drop_outliers(self) -> None:
        while self.fit is not None and len(self.samples) > MIN_SAMPLES:
            e = self.fit["errors_mm"]
            worst = int(np.argmax(e))
            if e[worst] <= max(15.0, 3 * float(np.median(e))):
                break
            print("dropping sample %d (error %.1f mm)" % (worst + 1, e[worst]))
            self.samples.pop(worst)
            self.refit()

    # ---- main loop
    def run(self) -> None:
        cv2.namedWindow(WIN)
        cv2.setMouseCallback(WIN, self.on_mouse)
        print("start pose: %s" % [round(v, 1) for v in self.start])
        while not self.quit:
            self.grab()
            k = self.show()
            if k in (27, ord("q")):
                break
            elif k == ord(" "):
                self.capture()
            elif k == ord("u") and self.samples:
                self.samples.pop()
                self.msg = "last sample removed"
                self.refit()
            elif k == ord("s"):
                self.save()
            elif self.manual:
                continue
            elif k == ord("h") and self.plan:
                self.move(self.plan[0])
            elif k in (ord("n"), ord("a")):
                while self.next_i < len(self.plan):
                    if not self.move(self.plan[self.next_i]):
                        break
                    self.next_i += 1
                    time.sleep(0.5)                              # let the arm settle
                    self.capture()
                    if k == ord("n"):
                        break
                if self.next_i >= len(self.plan):
                    self.msg = (self.msg + "  |  all planned poses done: press S to save, H to go back").strip(" |")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--manual", action="store_true", help="never move the robot; move it yourself and press SPACE")
    ap.add_argument("--red-tip", action="store_true", help="no marker: the red tip of the tool is the target")
    ap.add_argument("--auto", action="store_true", help="lift, run all poses, save if good and quit (no keys)")
    ap.add_argument("--poses", help="--auto: visit the poses of this task file in order (e.g. ../tasks/calibration_poses.txt)")
    ap.add_argument("--lift", type=float, default=START_LIFT_MM,
                    help="--auto: straight-up lift before the poses [mm] (0: start where the tool is)")
    ap.add_argument("--config", default=str(HERE / "config.json"))
    args = ap.parse_args(argv)
    poses = os.path.abspath(args.poses) if args.poses else None   # relative to where you started it
    os.chdir(HERE)                                               # model / config paths are relative
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(name)s: %(message)s")
    write_marker_png()
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print("config error: %s" % exc)
        return 1
    cam = OrbbecRGBD(cfg["camera"])
    cam.start()
    try:
        arm = rf.connect_robot(enable_motion=not args.manual)
    except RuntimeError as exc:
        cam.stop()
        print("robot: %s" % exc)
        return 1
    start = None
    if args.auto:
        start = rf.tool_pose(arm)
        start[2] += max(args.lift, 0.0)
    cal = Calibrator(cam, cfg["camera"], arm, args.manual, red_tip=args.red_tip, start=start, poses_file=poses)
    ok = True
    try:
        if args.auto and not args.manual:
            ok = cal.run_auto()
        else:
            cal.run()
    finally:
        if not args.manual:
            arm.set_state(4)
        arm.disconnect()
        cam.stop()
        cv2.destroyAllWindows()
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
