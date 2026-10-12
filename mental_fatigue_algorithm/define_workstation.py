"""Workstation places for the REAL robot, checked on the live camera image.

    python define_workstation.py

SAFE and HANDOVER come from robot positions you recorded (POMDP model/workstation_points.txt). STORAGE too, unless
you do the 30 s STORAGE baseline (key S): move your hands around / into the storage; the storage region is where they
went (same rule as the assembly area) and the robot's storage point is its top centre + STORAGE_ABOVE_MM. The storage
must be inside the camera view for this. Or the 30 s ROBOT storage baseline (key R): move the robot by hand (its
manual mode) so that its red tip goes around / into the storage. The robot's own positions are recorded (no camera
needed); red tip = tool pose + the tip offset of the robot-camera calibration. Storage region = where the red tip went;
robot storage point = the recorded tool position closest to the middle of the motion (the robot was there: reachable;
its joints are saved too). The tool starts with the areas of the saved workspace file (if it was made by this tool),
so one baseline can be redone without the other.
The ASSEMBLY AREA comes from a 20 s HAND BASELINE (BASELINE_S): press B and work normally (assemble, type on the keyboard).
Every position the hand model measures for your hands is recorded in robot coordinates; the assembly area is the region
they covered (2nd to 98th percentile of x and y, so a few stray points do not stretch it, plus BASELINE_MARGIN_MM; top =
98th percentile of the hand height plus the margin). Only hands the camera actually sees are used: hands hidden behind
the laptop screen cannot be measured.

Drawn on the live camera image (robot frame):
  * assembly area: hand trail (cyan dots), covered footprint on the table (cyan outline), area box (green);
    before the baseline, the box through your recorded corners is shown in grey for reference only;
  * SAFE (white), HANDOVER (orange), STORAGE pick point (grey; its box is outside this camera's view);
  * your hands now (green / red: inside / outside the area) and the robot tool (magenta; the robot is only READ).
Saves POMDP model/workspace_calibration.json for main.py and MATLAB (liveHRC); W = the robot base frame in metres.
The baseline points are also written to POMDP model/workstation_preview/assembly_baseline_points.csv.

Without camera:  python define_workstation.py --set-area XMIN XMAX YMIN YMAX [TOP]   (mm, robot frame = MATLAB's top view
x 1000 * metres) changes only the assembly area of the saved file, recomputes the checks and saves.

Keys:  B  start / stop the 20 s ASSEMBLY baseline     S  30 s STORAGE baseline with your hands
       R  30 s STORAGE baseline with the robot's red tip (move the robot by hand)     C  clear all
       1/2 x min -/+   3/4 x max -/+   5/6 y min -/+   7/8 y max -/+   9/0 top -/+   (10 mm per press, after the baseline)
       V  save    Q / Esc  quit
"""
from __future__ import annotations

import csv
import datetime as dt
import json
import os
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from fatigue_monitor.config import load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.pomdp_bridge import WorkspaceFrame
from fatigue_monitor.vision import FaceHandVision
from fatigue_monitor import robot_frame as rf
import workspace_calibration as wc

# ======================= settings you can change =========================
POINTS_FILE = rf.PROJECT / "POMDP model" / "workstation_points.txt"
ROBOT_POINTS_FILE = rf.PROJECT / "tasks" / "robot_points.txt"     # points of the robot's action files
# MATLAB's places that the REAL robot must reach (task layer: FETCH -> GRIP within 1 cm of the storage point, MOVE ->
# WORK within 1 cm of the collaborative work point): always the tool position of the action point the robot's action
# file drives to, never a place derived from the camera baselines
ACTION_POINTS = {"storage": "pos2", "collab": "pos12"}
BASELINE_FILE = rf.PROJECT / "POMDP model" / "workstation_preview" / "assembly_baseline_points.csv"
BASELINE_S = 20.0               # assembly hand baseline duration [s] (was 60)
STORAGE_BASELINE_S = 30.0       # storage hand baseline duration
STORAGE_ABOVE_MM = 50.0         # robot storage point: this far above the top of the storage hand region
STORAGE_BASELINE_FILE = rf.PROJECT / "POMDP model" / "workstation_preview" / "storage_baseline_points.csv"
ROBOT_STORAGE_S = 30.0          # robot storage baseline duration (red tip moved by hand)
ROBOT_STORAGE_MARGIN_MM = 20.0  # storage region = red tip positions (2nd-98th percentile) + this margin
LITE6_ARM_MM = 200.0 + 243.7    # Lite 6 shoulder (J2) -> wrist centre, fully stretched (a2 + |a3, d4|)
LITE6_D1_MM, LITE6_D6_MM = 243.3, 61.5
REACH_MARGIN_MM = 20.0          # keep the storage point this much inside the reach
BASELINE_PCT = (2.0, 98.0)      # area = these percentiles of the hand positions (x and y) ...
BASELINE_MARGIN_MM = 30.0       # ... plus this margin on every side and on top
BASELINE_MIN_POINTS = 100       # fewer hand points: no area (keep the hands visible to the camera)
HAND_Z_RANGE_MM = (-30.0, 600.0)  # hand points outside this height (robot frame) are rejected (depth errors)
HAND_MAX_RANGE_MM = 1500.0      # ... and farther than this from the robot base
STORAGE_BACK_Y_MM = -600.0      # storage box about 200 mm long: back edge
NUDGE_MM = 10.0
LITE6_SHOULDER_M = 0.2433
DISPLAY_SCALE = 0.75
# =========================================================================
HERE = Path(__file__).resolve().parent
WIN = "workstation places (robot frame)"
SAFE_CHECK = "robot at the safe position clear of the assembly area"
BASE_CHECK = "robot base column clear of the assembly area (fixed, any pose)"
BOX_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def rel(path: Path) -> str:
    """Path relative to the project folder when it is inside it."""
    try:
        return str(Path(path).resolve().relative_to(rf.PROJECT))
    except ValueError:
        return str(path)


def read_points(path: Path):
    """(name -> [x, y, z] mm, name -> 6 joint angles deg) from the name:[J1..J6, x, y, z] lines."""
    pts, joints = {}, {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.split("#")[0].strip()
        m = re.match(r"(\w+)\s*:\s*\[(.*)\]", line)
        if m:
            nums = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", m.group(2))]
            if len(nums) >= 3:
                pts[m.group(1)] = np.array(nums[-3:])
            if len(nums) == 9:
                joints[m.group(1)] = nums[:6]
    return pts, joints


def recorded_box(P: dict) -> dict:
    """Reference only: the box through the recorded corners."""
    names = ("upper_left_corner", "upper_right_corner", "lower_right_corner", "lower_left_corner",
             "other_side_upper_left_corner")
    q = np.array([P[n] for n in names if n in P])
    return {"xmin": q[:, 0].min(), "xmax": q[:, 0].max(), "ymin": q[:, 1].min(), "ymax": q[:, 1].max(),
            "zmax": q[:, 2].max()}


def baseline_area(points: np.ndarray, margin_mm: float = None) -> dict:
    lo = np.percentile(points, BASELINE_PCT[0], axis=0)
    hi = np.percentile(points, BASELINE_PCT[1], axis=0)
    m = BASELINE_MARGIN_MM if margin_mm is None else margin_mm
    return {"xmin": lo[0] - m, "xmax": hi[0] + m, "ymin": lo[1] - m, "ymax": hi[1] + m,
            "zmax": max(hi[2] + m, 100.0)}


def footprint(points: np.ndarray, area: dict) -> np.ndarray:
    """Convex hull (x, y) of the hand points inside the area (top view)."""
    q = points[(points[:, 0] >= area["xmin"]) & (points[:, 0] <= area["xmax"]) &
               (points[:, 1] >= area["ymin"]) & (points[:, 1] <= area["ymax"])][:, :2]
    if len(q) < 3:
        return np.zeros((0, 2))
    return cv2.convexHull(q.astype(np.float32)).reshape(-1, 2)


def box_corners(b: dict, zmin: float = 0.0):
    return [np.array([x, y, z]) for x in (b["xmin"], b["xmax"]) for y in (b["ymin"], b["ymax"]) for z in (zmin, b["zmax"])]


def real_safe_clearance(q_safe, box: dict):
    """(moving arm, fixed base column) clearance [m] between the REAL arm at the safe joints (Lite 6 links as
    capsules) and the assembly box."""
    L = rf.lite6_points_mm(q_safe)
    pts = [L["base"], L["shoulder"], L["elbow"], L["wrist"], L["flange"]]
    radii = (0.08, 0.06, 0.05, 0.05)
    lo = np.array([box["xmin"], box["ymin"], 0.0]) / 1000.0
    hi = np.array([box["xmax"], box["ymax"], box["zmax"]]) / 1000.0
    d = [wc.segment_box_distance(a / 1000.0, b / 1000.0, lo, hi) - r for a, b, r in zip(pts[:-1], pts[1:], radii)]
    return min(d[1:]), d[0]


def reach_tool_down_mm(z_mm: float) -> float:
    """Farthest distance from the base axis at which the Lite 6 can put its tool point at height z with the tool
    pointing down (wrist centre LITE6_D6_MM above it)."""
    dz = z_mm + LITE6_D6_MM - LITE6_D1_MM
    return float(np.sqrt(max(LITE6_ARM_MM ** 2 - dz ** 2, 0.0))) - REACH_MARGIN_MM


def storage_point(P: dict, st_area=None, rb_pick=None) -> np.ndarray:
    """Robot storage point [mm]: middle of the edge of the storage hand region that is nearest to the robot base, at
    its top + STORAGE_ABOVE_MM (the centre of a storage box may be out of the robot's reach), else the middle of the
    recorded front edge (reachable). With a robot storage baseline: the recorded robot tool position (rb_pick)."""
    if rb_pick is not None:
        return np.array(rb_pick["pos"], dtype=float)
    if st_area is not None:
        cx, cy = 0.5 * (st_area["xmin"] + st_area["xmax"]), 0.5 * (st_area["ymin"] + st_area["ymax"])
        edges = [(cx, st_area["ymin"]), (cx, st_area["ymax"]), (st_area["xmin"], cy), (st_area["xmax"], cy)]
        x, y = min(edges, key=lambda e: np.hypot(*e))
        return np.array([x, y, st_area["zmax"] + STORAGE_ABOVE_MM])
    return 0.5 * (P["storage_right_front_corner"] + P["storage_left_front_corner"])


def places_w(P: dict, b: dict, st_area=None, rb_pick=None) -> dict:
    """Places in W (metres) for workspace_calibration.derive and MATLAB."""
    sf, sl = P["storage_right_front_corner"], P["storage_left_front_corner"]
    storage = storage_point(P, st_area, rb_pick)
    m = lambda p: [round(float(v) / 1000.0, 4) for v in p]
    return {"base": [0.0, 0.0, 0.0],
            "assembly_area": {"x": m([b["xmin"], b["xmax"]]), "y": m([b["ymin"], b["ymax"]]), "height": round(b["zmax"] / 1000.0, 4)},
            "handover": m(P["handling_point"]), "storage": m(storage), "safe": m(P["safe_pose"]),
            "robot_height": LITE6_SHOULDER_M, "robot_height_measured": LITE6_SHOULDER_M,
            "storage_object_height": round(float(min(sf[2], sl[2]) if st_area is None else st_area["zmax"]) / 1000.0, 4),
            "handover_mode": "recorded with the robot",
            "storage_mode": ("robot red-tip baseline" if rb_pick is not None else
                             "hand baseline" if st_area is not None else "recorded with the robot")}


def tip_mm(pose, offset) -> np.ndarray:
    """Red tip position [mm]: tool position + tool rotation @ tip offset (robot-camera calibration)."""
    return np.array(pose[:3], dtype=float) + rf.rpy_matrix(*pose[3:6]) @ np.asarray(offset, dtype=float)


def robot_pick(flanges: np.ndarray, joints: list, poses: list) -> dict:
    """The recorded sample whose tool position is closest to the median of the motion."""
    i = int(np.argmin(np.linalg.norm(flanges - np.median(flanges, axis=0), axis=1)))
    return {"pos": [float(v) for v in flanges[i]], "joints": [float(v) for v in joints[i]],
            "pose": [float(v) for v in poses[i]]}


def load_saved(path: Path):
    """(assembly area, storage area, whole file) of a workspace file written by this tool; areas in mm, robot frame."""
    try:
        d = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None, None, {}
    area = d.get("assembly_baseline", {}).get("area_robot_mm")
    st = d.get("storage_baseline", {}).get("area_robot_mm")
    return (dict(area) if area else None), (dict(st) if st else None), d


def load_saved_pick(d: dict):
    """Robot storage pick of a saved robot storage baseline, else None."""
    sb = d.get("storage_baseline", {})
    return {"pos": sb["pick_tool_mm"], "joints": sb.get("pick_joints_deg"), "pose": sb.get("pick_pose")} \
        if "pick_tool_mm" in sb else None


class View:
    def __init__(self, rframe: rf.RobotFrame, intr) -> None:
        self.rframe, self.intr = rframe, intr

    def uv(self, p_mm):
        return rf.project(self.rframe.to_cam(np.asarray(p_mm, dtype=float)), self.intr)

    def box(self, img, corners, col, thick=2):
        q = [self.uv(c) for c in corners]
        for a, b in BOX_EDGES:
            if q[a] and q[b]:
                cv2.line(img, q[a], q[b], col, thick, cv2.LINE_AA)

    def polygon(self, img, pts_mm, col, thick=2):
        q = [self.uv(p) for p in pts_mm]
        if len(q) >= 3 and all(q):
            cv2.polylines(img, [np.array(q, np.int32)], True, col, thick, cv2.LINE_AA)

    def point(self, img, p_mm, col, label, r=9, stem=True):
        q = self.uv(p_mm)
        if q is None:
            return
        if stem:                                                # line down to the table: shows the height
            f = self.uv([p_mm[0], p_mm[1], 0.0])
            if f:
                cv2.line(img, f, q, col, 1, cv2.LINE_AA)
        cv2.circle(img, q, r, col, -1)
        if label:
            rf.put_text(img, label, (q[0] + 12, q[1] - 8), col, 0.75, 2)


def visible_hands_mm(f, vision, rframe, intr, cc, inf_w=640):
    """Palm centres (robot frame, mm) of the hands the hand model sees in this frame, with plausible depth."""
    h, w = f.color_bgr.shape[:2]
    s = inf_w / w
    _, hands = vision.detect(cv2.resize(f.color_bgr, (inf_w, int(h * s))), f.t)
    out = []
    for hd in hands:
        p = rframe.to_robot(rf.palm_point_mm(f.depth_mm, hd / s, intr, cc["depth_min_mm"], cc["depth_max_mm"]))
        if p is not None and HAND_Z_RANGE_MM[0] <= p[2] <= HAND_Z_RANGE_MM[1] and np.linalg.norm(p) <= HAND_MAX_RANGE_MM:
            out.append(p)
    return out


def action_point_poses() -> dict:
    """{'storage': W [m], 'collab': W [m]} = tool (flange) position of the robot's action points (ACTION_POINTS in
    tasks/robot_points.txt, forward kinematics); empty if the file or a point is missing."""
    out = {}
    try:
        text = ROBOT_POINTS_FILE.read_text(encoding="utf-8")
    except OSError:
        return out
    for place, name in ACTION_POINTS.items():
        m = re.search(r"(?m)^%s\s*:\s*\[([^\]]+)\]" % re.escape(name), text)
        if m:
            q = [float(v) for v in m.group(1).split(",")[:6]]
            out[place] = np.round(rf.lite6_points_mm(q)["flange"] / 1000.0, 4)
    return out


def evaluate(P: dict, J: dict, area: dict, st_area, rb_pick, cfg: dict):
    """Places (W) and all checks for an assembly area (mm, robot frame): (pl, cfx, poses, zones, checks, storage point)."""
    pl = places_w(P, area, st_area, rb_pick)
    cfx = dict(cfg["pomdp_bridge"]["calibration"], area_height_m=pl["assembly_area"]["height"])
    poses, zones, checks = wc.derive(pl, cfx)
    for place, p in action_point_poses().items():        # where the real robot's action files actually go
        poses[place] = p
        if place == "storage":
            pl["storage"] = [float(v) for v in p]
            pl["storage_mode"] = "robot action point %s (tasks/robot_points.txt)" % ACTION_POINTS["storage"]
    if "safe_pose" in J:
        arm_c, base_c = real_safe_clearance(J["safe_pose"], area)
        need = cfx["safe_clearance_m"]
        checks[SAFE_CHECK] = (arm_c >= need, "%.2f m (>= %.2f, real arm links)" % (arm_c, need), True)
        checks[BASE_CHECK] = (base_c >= need, "%.2f m (>= %.2f)" % (base_c, need), False)
    # MATLAB live rule (zoneClassifyHRC with the fixed assistive zone): a hand counts as collaborative
    # within dCollabMax of the robot tool and as assistive within assistRadius (top view) of the handover
    # point. With the robot idle at SAFE, a hand in the assembly area is "separated" when the area is
    # farther than dCollabMax from SAFE and does not touch the assistive cylinder.
    old_key = [k2 for k2 in checks if k2.startswith("idle robot counts as separated")]
    for k2 in old_key:                              # old rule (assistive = dAssistMax from the tool)
        del checks[k2]
    lo = np.array([area["xmin"], area["ymin"], 0.0]) / 1000.0
    hi = np.array([area["xmax"], area["ymax"], area["zmax"]]) / 1000.0
    d_safe = wc.box_distance(np.array(pl["safe"]), lo, hi)
    hv = np.array(pl["handover"][:2])
    d_assist = float(np.linalg.norm(np.maximum(np.maximum(lo[:2] - hv, 0.0), hv - hi[:2])))
    r_assist, r_collab = cfx.get("assist_radius_m", 0.12), cfx.get("collab_radius_m", 0.30)
    checks["idle robot at SAFE: hand in the assembly area counts as separated"] = (
        d_safe > r_collab and d_assist > r_assist,
        "area %.2f m from SAFE (> collaborative %.2f), %.3f m from the handover point (> assistive %.3f)%s" % (
            d_safe, r_collab, d_assist, r_assist,
            "" if d_assist > r_assist else " -> the area corner nearest the handover point is assistive"),
        False)
    stp = storage_point(P, st_area, rb_pick)
    reach, limit = float(np.linalg.norm(stp[:2])), reach_tool_down_mm(stp[2])
    inbox = rf.in_safety_box(stp, margin=0)
    checks["storage point within the robot's reach (tool pointing down) and the safety box"] = (
        (reach <= limit or rb_pick is not None) and inbox,
        "%.0f mm from the base axis (limit %.0f at this height)%s%s" % (
            reach, limit, " - reached by the robot itself" if rb_pick is not None else "" if reach <= limit else
            " -> move the storage about %.0f mm closer to the robot" % (reach - limit),
            "" if inbox else ", OUTSIDE the gateway box +-500 mm"), False)
    return pl, cfx, poses, zones, checks, stp


def set_area(values) -> int:
    """--set-area XMIN XMAX YMIN YMAX [TOP] (mm, robot frame): change only the assembly area of the saved workspace
    file, recompute everything that depends on it, print the checks, save unless a red check blocks. No camera."""
    cfg = load_config(str(HERE / "config.json"))
    out_path = Path(cfg["pomdp_bridge"]["workspace_file"])
    if not out_path.is_absolute():
        out_path = (HERE / out_path).resolve()
    area, st_area, prev = load_saved(out_path)
    if area is None:
        print("no saved assembly area in %s: run define_workstation.py first" % out_path)
        return 1
    v = [float(x) for x in values]
    new = {"xmin": v[0], "xmax": v[1], "ymin": v[2], "ymax": v[3], "zmax": v[4] if len(v) > 4 else area["zmax"]}
    if new["xmin"] >= new["xmax"] or new["ymin"] >= new["ymax"] or new["zmax"] <= 0:
        print("invalid area %s" % new)
        return 1
    P, J = read_points(POINTS_FILE)
    rb_pick = load_saved_pick(prev)
    pl, cfx, poses, zones, checks, stp = evaluate(P, J, new, st_area, rb_pick, cfg)
    print("assembly area: x %.0f..%.0f  y %.0f..%.0f  top %.0f mm (was x %.0f..%.0f  y %.0f..%.0f  top %.0f)" % (
        new["xmin"], new["xmax"], new["ymin"], new["ymax"], new["zmax"],
        area["xmin"], area["xmax"], area["ymin"], area["ymax"], area["zmax"]))
    for k, (ok, val, block) in checks.items():
        print("  %s %s: %s" % ("OK  " if ok else ("NO  " if block else "WARN"), k, val))
    if any(not ok for ok, _, block in checks.values() if block):
        print("NOT saved: a red (NO) check blocks saving")
        return 2
    prev["places_W"]["assembly_area"] = pl["assembly_area"]
    prev["robot_poses_W"] = {k2: [round(float(x), 4) for x in val] for k2, val in poses.items()}
    prev["zones"] = {k2: round(float(val), 4) for k2, val in zones.items()}
    prev["checks"] = {k2: bool(val[0]) for k2, val in checks.items()}
    ab = prev.setdefault("assembly_baseline", {})
    ab.setdefault("area_baseline_mm", ab.get("area_robot_mm"))   # keep what the hand baseline gave
    ab["area_robot_mm"] = {k2: round(float(val), 1) for k2, val in new.items()}
    ab["manual_override"] = dt.datetime.now().isoformat(timespec="seconds")
    prev["created"] = dt.datetime.now().isoformat(timespec="seconds")
    out_path.write_text(json.dumps(prev, indent=2) + "\n", encoding="utf-8")
    print("SAVED %s" % out_path)
    return 0


def main() -> int:
    if "--set-area" in sys.argv:
        i = sys.argv.index("--set-area")
        vals = sys.argv[i + 1:i + 6]
        if len(vals) < 4:
            print("usage: python define_workstation.py --set-area XMIN XMAX YMIN YMAX [TOP]   (mm, robot frame)")
            return 1
        return set_area([x for x in vals if not x.startswith("--")])
    os.chdir(HERE)
    rframe = rf.RobotFrame(rf.CALIB_FILE)
    if not rframe.calibrated:
        print("no robot-camera calibration: run robot_camera_calibration.py first")
        return 1
    P, J = read_points(POINTS_FILE)
    need = ["handling_point", "safe_pose", "storage_right_front_corner", "storage_left_front_corner"]
    missing = [n for n in need if n not in P]
    if missing:
        print("missing in %s: %s" % (POINTS_FILE, ", ".join(missing)))
        return 1
    cfg = load_config(str(HERE / "config.json"))
    cfg["body_presence"]["enabled"] = False                     # hand model only: hands the camera really sees
    cfg.setdefault("physical", {})["enabled"] = False
    cc = cfg["camera"]
    cam = OrbbecRGBD(cc)
    cam.start()
    intr = cam.intrinsics
    vision = FaceHandVision(cfg)
    poller = rf.RobotPoller(10)
    view = View(rframe, intr)
    ref_box = recorded_box(P) if "upper_left_corner" in P else None
    points: list = []                                           # baseline hand points (robot frame, mm)
    times: list = []
    base_t0, area, hull = None, None, np.zeros((0, 2))
    st_points, st_times, st_t0, st_area = [], [], None, None   # storage baseline (hands)
    rb_tips, rb_flanges, rb_joints, rb_poses, rb_times, rb_t0, rb_pick, rb_last_t = [], [], [], [], [], None, None, None
    tip_offset = rframe.data.get("marker_offset_tool_mm", [0.0, 0.0, 0.0])   # red tip on the tool
    msg = "press B and work normally for %.0f s (assemble, type): your hands define the assembly area" % BASELINE_S
    out_path = Path(cfg["pomdp_bridge"]["workspace_file"])
    if not out_path.is_absolute():
        out_path = (HERE / out_path).resolve()
    area, st_area, saved = load_saved(out_path)
    rb_pick = load_saved_pick(saved)
    if area is not None:
        msg = "loaded the saved assembly area%s - B / S redo a baseline, V saves" % (
            " and storage area" if st_area is not None else "")
    sf, sl = P["storage_right_front_corner"], P["storage_left_front_corner"]
    try:
        while True:
            f = cam.read()
            if f is None or f.depth_mm is None or not f.depth_aligned:
                continue
            img = f.color_bgr.copy()
            hands_mm = visible_hands_mm(f, vision, rframe, intr, cc)
            now = time.time()
            if base_t0 is not None:                             # baseline running
                points += hands_mm
                times += [now - base_t0] * len(hands_mm)
                left = BASELINE_S - (now - base_t0)
                if len(points) >= 10:
                    area = baseline_area(np.array(points))
                msg = "BASELINE: work normally ... %2.0f s left, %d hand points" % (max(left, 0), len(points))
                if left <= 0:
                    base_t0 = None
                    if len(points) >= BASELINE_MIN_POINTS:
                        area = baseline_area(np.array(points))
                        hull = footprint(np.array(points), area)
                        msg = "baseline done: %d hand points -> assembly area set (check it, nudge with 1-0, V saves)" % len(points)
                    else:
                        area = None
                        msg = "baseline: only %d hand points (need %d) - keep your hands visible and press B again" % (
                            len(points), BASELINE_MIN_POINTS)

            if st_t0 is not None:                               # storage baseline running
                st_points += hands_mm
                st_times += [now - st_t0] * len(hands_mm)
                left = STORAGE_BASELINE_S - (now - st_t0)
                if len(st_points) >= 10:
                    st_area = baseline_area(np.array(st_points))
                msg = "STORAGE BASELINE: move your hands around / into the storage ... %2.0f s left, %d hand points" % (
                    max(left, 0), len(st_points))
                if left <= 0:
                    st_t0 = None
                    if len(st_points) >= BASELINE_MIN_POINTS // 2:
                        st_area = baseline_area(np.array(st_points))
                        msg = "storage baseline done: %d hand points -> storage set (V saves)" % len(st_points)
                    else:
                        st_area = None
                        msg = ("storage baseline: only %d hand points - is the storage inside the camera view? "
                               "(the recorded storage is kept)" % len(st_points))

            last = poller.latest()
            if rb_t0 is not None:                               # robot storage baseline running
                if last is not None and last["t"] != rb_last_t and "pose" in last:
                    rb_last_t = last["t"]
                    rb_tips.append(tip_mm(last["pose"], tip_offset))
                    rb_flanges.append(np.array(last["pose"][:3]))
                    rb_joints.append(last["joints"])
                    rb_poses.append(last["pose"])
                    rb_times.append(now - rb_t0)
                left = ROBOT_STORAGE_S - (now - rb_t0)
                if len(rb_tips) >= 5:
                    st_area = baseline_area(np.array(rb_tips), ROBOT_STORAGE_MARGIN_MM)
                msg = "ROBOT STORAGE BASELINE: move the robot's red tip around / into the storage ... %2.0f s left, %d robot poses" % (
                    max(left, 0), len(rb_tips))
                if left <= 0:
                    rb_t0 = None
                    if len(rb_tips) >= 20:
                        st_area = baseline_area(np.array(rb_tips), ROBOT_STORAGE_MARGIN_MM)
                        rb_pick = robot_pick(np.array(rb_flanges), rb_joints, rb_poses)
                        msg = "robot storage baseline done: %d robot poses -> storage set (V saves)" % len(rb_tips)
                    else:
                        msg = "robot storage baseline: only %d robot poses - is the robot connected? (R again)" % len(rb_tips)

            # ---- drawing (robot frame -> image)
            o = view.uv([0, 0, 0])
            for axis, col in (([150.0, 0, 0], (0, 0, 255)), ([0, 150.0, 0], (0, 255, 0)), ([0, 0, 150.0], (255, 0, 0))):
                e = view.uv(axis)
                if o and e:
                    cv2.arrowedLine(img, o, e, col, 2, tipLength=0.2)
            if area is None and ref_box is not None:
                view.box(img, box_corners(ref_box), (150, 150, 150), 1)
            for p in points[-1500:]:
                q = view.uv(p)
                if q:
                    cv2.circle(img, q, 2, (255, 255, 0), -1)
            if len(hull) >= 3:
                view.polygon(img, [[x, y, 0.0] for x, y in hull], (255, 255, 0), 2)
            if area is not None:
                view.box(img, box_corners(area), (60, 220, 60), 3)
                q = view.uv(box_corners(area)[7])
                if q:
                    rf.put_text(img, "ASSEMBLY AREA", (q[0] + 10, q[1]), (60, 220, 60), 0.8, 2)
            for p in st_points[-800:]:
                q = view.uv(p)
                if q:
                    cv2.circle(img, q, 2, (220, 220, 220), -1)
            if st_area is not None:
                view.box(img, box_corners(st_area), (220, 220, 220), 2)
            for p in rb_tips[-600:]:
                q = view.uv(p)
                if q:
                    cv2.circle(img, q, 3, (0, 0, 255), -1)
            view.point(img, storage_point(P, st_area, rb_pick), (220, 220, 220), "STORAGE (robot pick)")
            view.point(img, P["safe_pose"], (255, 255, 255), "SAFE")
            view.point(img, P["handling_point"], (0, 165, 255), "HANDOVER")
            for p in hands_mm:
                inside = area is not None and (area["xmin"] <= p[0] <= area["xmax"] and area["ymin"] <= p[1] <= area["ymax"]
                                               and p[2] <= area["zmax"])
                view.point(img, p, (0, 255, 0) if inside else (0, 0, 255),
                           "hand %s [%.0f %.0f %.0f]" % ("IN" if inside else "out", *p), r=8, stem=False)
            if last is not None and time.monotonic() - last["t"] < 1.0:
                q = view.uv(last["pts"]["tool"])
                if q:
                    cv2.circle(img, q, 14, (255, 0, 255), 3)

            # ---- text panel (after resizing: sharp)
            img = cv2.resize(img, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)
            lines = [("B %.0f s assembly baseline   S %.0f s storage (hands)   R 30 s storage (robot red tip)   C clear"
                      "   1/2 3/4 5/6 7/8 9/0 nudge   V save   Q quit") % (BASELINE_S, STORAGE_BASELINE_S), msg]
            checks = {}
            if area is not None:
                pl, cfx, poses, zones, checks, stp = evaluate(P, J, area, st_area, rb_pick, cfg)
                lines.append("ASSEMBLY x %.0f..%.0f  y %.0f..%.0f  z 0..%.0f mm   |   SAFE %s  HANDOVER %s  STORAGE %s (%s)" % (
                    area["xmin"], area["xmax"], area["ymin"], area["ymax"], area["zmax"],
                    np.round(P["safe_pose"]).astype(int).tolist(), np.round(P["handling_point"]).astype(int).tolist(),
                    np.round(stp).astype(int).tolist(), pl["storage_mode"]))
                for k, (ok, val, block) in checks.items():
                    lines.append("%s %s: %s" % ("OK  " if ok else ("NO  " if block else "WARN"), k, val))
            y = 22
            for ln in lines:
                col = (80, 80, 255) if ln.startswith("NO") else (0, 210, 255) if ln.startswith("WARN") else \
                    (80, 220, 80) if ln.startswith("OK") else (255, 255, 255)
                rf.put_text(img, ln, (8, y), col, 0.5)
                y += 20
            if base_t0 is not None or st_t0 is not None or rb_t0 is not None:    # progress bar
                frac = (min((now - base_t0) / BASELINE_S, 1.0) if base_t0 is not None
                        else min((now - st_t0) / STORAGE_BASELINE_S, 1.0) if st_t0 is not None
                        else min((now - rb_t0) / ROBOT_STORAGE_S, 1.0))
                cv2.rectangle(img, (8, img.shape[0] - 24), (8 + int(frac * (img.shape[1] - 16)), img.shape[0] - 10), (255, 255, 0), -1)
            cv2.imshow(WIN, img)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break
            busy = base_t0 is not None or st_t0 is not None or rb_t0 is not None
            if k == ord("r"):
                if not busy:
                    rb_tips, rb_flanges, rb_joints, rb_poses, rb_times = [], [], [], [], []
                    st_area, rb_pick, rb_t0, st_points = None, None, time.time(), []
                elif rb_t0 is not None:
                    rb_t0, msg = None, "robot storage baseline stopped early (%d robot poses)" % len(rb_tips)
                    if len(rb_tips) >= 20:
                        st_area = baseline_area(np.array(rb_tips), ROBOT_STORAGE_MARGIN_MM)
                        rb_pick = robot_pick(np.array(rb_flanges), rb_joints, rb_poses)
            elif k == ord("s"):
                if not busy:
                    st_points, st_times, st_area, st_t0, rb_pick, rb_tips = [], [], None, time.time(), None, []
                elif st_t0 is not None:
                    st_t0, msg = None, "storage baseline stopped early (%d hand points)" % len(st_points)
                    st_area = baseline_area(np.array(st_points)) if len(st_points) >= BASELINE_MIN_POINTS // 2 else None
            elif k == ord("b") and st_t0 is None and rb_t0 is None:
                if base_t0 is None:
                    points, times, hull, area, base_t0 = [], [], np.zeros((0, 2)), None, time.time()
                else:
                    base_t0, msg = None, "baseline stopped early (%d hand points) - B to start again" % len(points)
                    if len(points) >= BASELINE_MIN_POINTS:
                        area = baseline_area(np.array(points))
                        hull = footprint(np.array(points), area)
            elif k == ord("c"):
                points, times, hull, area, base_t0, msg = [], [], np.zeros((0, 2)), None, None, "baselines cleared"
                st_points, st_times, st_area, st_t0 = [], [], None, None
                rb_tips, rb_flanges, rb_joints, rb_poses, rb_times, rb_t0, rb_pick = [], [], [], [], [], None, None
            elif k in map(ord, "1234567890") and area is not None:
                key, sign = {"1": ("xmin", -1), "2": ("xmin", 1), "3": ("xmax", -1), "4": ("xmax", 1), "5": ("ymin", -1),
                             "6": ("ymin", 1), "7": ("ymax", -1), "8": ("ymax", 1), "9": ("zmax", -1), "0": ("zmax", 1)}[chr(k)]
                area[key] += sign * NUDGE_MM
                msg = "%s = %.0f mm" % (key, area[key])
            elif k == ord("v"):
                if area is None:
                    msg = "NOT saved: do the hand baseline first (B)"
                    continue
                if any(not ok for ok, _, block in checks.values() if block):
                    msg = "NOT saved: a red (NO) check blocks saving"
                    continue
                wf = WorkspaceFrame()
                wf.R, wf.origin, wf.R_w_robot, theta = rf.workspace_frame_from_robot(rframe, (1.0, 0.0, 0.0))
                pts_arr = np.array(points) if points else None
                _, _, prev = load_saved(out_path)
                extra = {"method": "real robot (recorded robot points + hand baseline)",
                         "places_W": {k2: pl[k2] for k2 in ("base", "assembly_area", "handover", "storage", "safe", "handover_mode",
                                                            "storage_mode")},
                         "robot": {"shoulder_height": LITE6_SHOULDER_M, "object_height": LITE6_SHOULDER_M},
                         "storage_object_height": pl["storage_object_height"],
                         "robot_poses_W": {k2: [round(float(x), 4) for x in v] for k2, v in poses.items()},
                         "zones": {k2: round(float(v), 4) for k2, v in zones.items()},
                         "checks": {k2: bool(v[0]) for k2, v in checks.items()},
                         "settings": cfx, "camera": {"color": cam.color_profile_desc, "depth_align": cam.align_mode},
                         "assembly_baseline": (dict(prev.get("assembly_baseline", {}),
                                                    area_robot_mm={k2: round(float(v), 1) for k2, v in area.items()})
                                               if pts_arr is None else {"duration_s": BASELINE_S, "hand_points": len(points),
                                               "percentiles": list(BASELINE_PCT), "margin_mm": BASELINE_MARGIN_MM,
                                               "area_robot_mm": {k2: round(float(v), 1) for k2, v in area.items()},
                                               "hand_z_p50_mm": round(float(np.median(pts_arr[:, 2])), 1),
                                               "footprint_robot_mm": [[round(float(x), 1), round(float(y), 1)] for x, y in hull],
                                               "points_file": rel(BASELINE_FILE)}),
                         "storage_baseline": ({"method": "robot red tip moved by hand", "duration_s": ROBOT_STORAGE_S,
                                               "robot_poses": len(rb_tips), "percentiles": list(BASELINE_PCT),
                                               "margin_mm": ROBOT_STORAGE_MARGIN_MM, "tip_offset_tool_mm": list(tip_offset),
                                               "area_robot_mm": {k2: round(float(v), 1) for k2, v in st_area.items()},
                                               "pick_tool_mm": [round(v, 1) for v in rb_pick["pos"]],
                                               "pick_joints_deg": [round(v, 2) for v in rb_pick["joints"]],
                                               "pick_pose": [round(v, 2) for v in rb_pick["pose"]],
                                               "points_file": rel(STORAGE_BASELINE_FILE)}
                                              if rb_pick is not None and rb_tips else
                                              dict(prev.get("storage_baseline", {}))
                                              if rb_pick is not None else
                                              {} if st_area is None else
                                              dict(prev.get("storage_baseline", {}),
                                                   area_robot_mm={k2: round(float(v), 1) for k2, v in st_area.items()})
                                              if not st_points else
                                              {"duration_s": STORAGE_BASELINE_S, "hand_points": len(st_points),
                                               "percentiles": list(BASELINE_PCT), "margin_mm": BASELINE_MARGIN_MM,
                                               "above_mm": STORAGE_ABOVE_MM,
                                               "area_robot_mm": {k2: round(float(v), 1) for k2, v in st_area.items()},
                                               "points_file": rel(STORAGE_BASELINE_FILE)}),
                         "action_points": dict(ACTION_POINTS, handover="pos4", safe="pos16"),
                         "real_robot": {"theta_deg": theta, "points_file": POINTS_FILE.name,
                                        "robot_camera_calibration_created": rframe.data.get("created"),
                                        "use": "p_W_m = R_w_robot @ p_robot_mm / 1000 (W = robot frame in metres)"}}
                wf.save(str(out_path), extra)
                BASELINE_FILE.parent.mkdir(parents=True, exist_ok=True)
                for fname, tt, pp in ((BASELINE_FILE, times, points), (STORAGE_BASELINE_FILE, st_times, st_points),
                                      (STORAGE_BASELINE_FILE, rb_times, rb_tips)):
                    if pp:                                      # only the baselines done in this session
                        with open(fname, "w", newline="", encoding="utf-8") as fh:
                            wr = csv.writer(fh)
                            wr.writerow(["t_s", "x_mm", "y_mm", "z_mm"])
                            wr.writerows([[round(t, 3)] + [round(float(v), 1) for v in p] for t, p in zip(tt, pp)])
                msg = "SAVED %s" % out_path
                print(msg)
    finally:
        poller.close()
        vision.close()
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
