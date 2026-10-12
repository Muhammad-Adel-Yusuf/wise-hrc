#!/usr/bin/env python3
"""Workstation calibration with REAL OBJECTS standing in for the robot (Orbbec RGB-D). Run it BEFORE main.py.

  1  ASSEMBLY     drag a rectangle over the (empty) table area in front of you where you assemble (press, move, release).
                  The depth inside this rectangle also measures the TABLE, so keep hands and objects out of it.
  2  ROBOT        click the object that stands for the robot (e.g. a bottle):
                  robot base = centre of its foot on the table, safe position = its tip (the robot's initial position),
                  virtual robot height = the object's height
  3  STORAGE      click the object that stands for the tool storage (e.g. a tissue packet)
  4  HANDOVER     automatic: between the robot and the assembly area, just outside the area, handover_height_m above
                  the table. Click the table to move it, or press H and hold your hand at the exact point for ~3 s.
How an object is found: every depth pixel near the click that stands higher than object_min_height_m above the table is
dropped vertically onto the table; the pixels whose footprints touch the footprint of the clicked part form the object.
So the object is found whole even when the camera has no depth on parts of it (a shiny or partly transparent bottle
gives broken depth). Press D to see what the camera sees above the table (coloured by height). If the camera sees only a
part of the bottle (clear plastic: often only the label), the measured height is too small: wrap paper around the bottle,
or write its height in config.json, pomdp_bridge.calibration.robot_height_override_m (0 = use the measured height).
The workstation frame W: origin = robot base on the table, z = up (table plane), x = towards the assembly area (the
worker), y = to the worker's right; metres.

Checks: must pass (red blocks saving)
  * the robot (object) at its safe position keeps safe_clearance_m from the assembly area
  * the handover point is outside the assembly area
  * the straight robot paths safe -> storage -> handover -> safe do not cross the assembly area
Warnings (shown in yellow, saving allowed)
  * the safe position is inside the assistive distance: the idle robot then counts as "assistive", not "separated"
    (the robot is more cautious while you work)
  * the handover point is farther than reach_max_m from the assembly area
Everything is written to workspace_calibration.json (config pomdp_bridge.workspace_file, default ../POMDP model/).

Keys: 1-4 redo a step, D show the height above the table, H handover from your hand, V save, Q / Esc quit.

REAL ROBOT (when POMDP model/robot_camera_calibration.json exists, from robot_camera_calibration.py): W is built on the
real robot instead of a stand-in object: origin = robot base, z = robot z (up), x = from the base towards the assembly
area. Step 2 takes the SAFE position from the robot itself: put the robot in its safe / idle pose and press 2 (the
robot is only read, never moved). The workspace file then also holds R_w_robot (W <-> robot frame), so main.py sends
the real robot pose and MATLAB measures the distances to the real robot. --stand-in forces the object method.
"""
from __future__ import annotations

import argparse
import logging
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

from fatigue_monitor.config import ConfigError, load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.pomdp_bridge import PALM_IDX, WorkspaceFrame, point_3d
from fatigue_monitor.vision import create_vision
from fatigue_monitor import robot_frame as rf

log = logging.getLogger("fatigue.workspace")
STEP_PROMPT = {1: "drag a rectangle over the ASSEMBLY area (empty table: it also measures the table)",
               2: "click the object that is the ROBOT (e.g. the bottle)",
               3: "click the object that is the TOOL STORAGE (e.g. the tissue packet)",
               4: "handover is automatic - click the table to move it, H = your hand, V = save"}
ROBOT_ELBOW_LIFT = 0.25                # same as stateDefinitionHRC (robotPointsHRC)
ROBOT_RADII = (0.08, 0.06, 0.05)       # column, upper link, forearm + tool
HAND_HEIGHT = 0.10                     # typical hand height above the table while assembling (m)
LITE6_SHOULDER_M = 0.2433              # real robot: height of the J2 axis above the base (m)
ROBOT_STEP_PROMPT = "put the robot in its SAFE (idle) pose and press 2 - its tool position is read (no motion)"
FOOT_CELL = 0.01                       # top-view grid for the object footprints (m)


# ---------------------------------------------------------------------------------------------------------------
# depth helpers (camera coordinates, metres)
# ---------------------------------------------------------------------------------------------------------------
def depth_points(depth_mm, u0, u1, v0, v1, intr, cam):
    """3-D points and pixel coordinates of the valid depth pixels in a window."""
    h, w = depth_mm.shape[:2]
    u0, u1, v0, v1 = max(0, u0), min(w, u1), max(0, v0), min(h, v1)
    vv, uu = np.mgrid[v0:v1, u0:u1]
    z = depth_mm[v0:v1, u0:u1].astype(float)
    ok = (z >= cam["depth_min_mm"]) & (z <= cam["depth_max_mm"])
    z = z / 1000.0
    P = np.stack([(uu - intr.cx) * z / intr.fx, (vv - intr.cy) * z / intr.fy, z], axis=-1)
    return P, ok, (u0, v0)


def table_patch(depth_mm, u, v, intr, cam, r=25) -> Optional[np.ndarray]:
    if depth_mm is None:
        return None
    P, ok, _ = depth_points(depth_mm, int(u) - r, int(u) + r + 1, int(v) - r, int(v) + r + 1, intr, cam)
    return P[ok] if ok.sum() > 0.3 * ok.size else None


def plane_fit(P: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Table plane (point, unit normal towards the camera): RANSAC (largest set of points within 1 cm of a plane, so
    hands, objects or the worker inside the rectangle are ignored), then least squares on those points."""
    rng = np.random.default_rng(0)
    if len(P) > 30000:
        P = P[rng.choice(len(P), 30000, replace=False)]
    best, best_n = None, -1
    for _ in range(300):
        a, b, c = P[rng.choice(len(P), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((P - a) @ n) < 0.01
        if inl.sum() > best_n:
            best, best_n = inl, int(inl.sum())
    Q = P[best]
    for _ in range(2):
        c = Q.mean(axis=0)
        n = np.linalg.svd(Q - c, full_matrices=False)[2][2]
        Q = P[np.abs((P - c) @ n) < 0.008]
    c = Q.mean(axis=0)
    return c, (n if np.dot(n, -c) > 0 else -n)


def ray_plane(u, v, intr, p0, n) -> Optional[np.ndarray]:
    ray = np.array([(u - intr.cx) / intr.fx, (v - intr.cy) / intr.fy, 1.0])
    den = float(np.dot(n, ray))
    return None if abs(den) < 1e-9 else ray * (np.dot(n, p0) / den)


def table_basis(n) -> Tuple[np.ndarray, np.ndarray]:
    e1 = np.cross(n, [1.0, 0.0, 0.0] if abs(n[0]) < 0.9 else [0.0, 1.0, 0.0])
    e1 /= np.linalg.norm(e1)
    return e1, np.cross(n, e1)


def segment_object(depth_mm, u, v, plane, intr, cam, c, max_radius: float = 1.0,
                   exclude: Optional[Tuple[np.ndarray, float]] = None) -> Optional[Dict]:
    """The object at the clicked pixel. Pixels higher than object_min_height_m above the table are dropped vertically
    onto the table (footprint, FOOT_CELL grid); the object = all pixels whose footprint is connected to the footprint
    of the clicked part. Gaps in the depth image (shiny, partly transparent) therefore do not cut the object.
    max_radius: the footprint is also limited to this radius (m) around the clicked part (re-centred twice on the
    object), so things standing next to it (a box behind the bottle, your arm) are not taken with it.
    exclude: (foot point in camera coordinates, radius) of an object already found (the robot bottle when the
    storage box is clicked): its pixels are left out."""
    if depth_mm is None:
        return None
    r = int(c["object_window_px"])
    P, ok, (u0, v0) = depth_points(depth_mm, int(u) - r, int(u) + r + 1, int(v) - r, int(v) + r + 1, intr, cam)
    p0, n = plane
    H = np.where(ok, (P - p0) @ n, 0.0)
    above = ok & (H > c["object_min_height_m"]) & (H < 1.0)
    if above.sum() < 40:
        return None
    vv, uu = np.nonzero(above)
    # seed: the clicked pixel or the nearest raised pixel within 15 px
    du, dv = uu - (int(u) - u0), vv - (int(v) - v0)
    d2 = du * du + dv * dv
    k = int(np.argmin(d2))
    if d2[k] > 15 * 15:
        return None
    pts = P[vv, uu]
    e1, e2 = table_basis(n)
    q = np.stack([pts @ e1, pts @ e2], axis=1)
    if exclude is not None:
        xc = np.array([exclude[0] @ e1, exclude[0] @ e2])
        keep = np.linalg.norm(q - xc, axis=1) > exclude[1]
        if keep.sum() < 40:
            return None
        vv, uu, pts, q = vv[keep], uu[keep], pts[keep], q[keep]
        du, dv = uu - (int(u) - u0), vv - (int(v) - v0)
        d2 = du * du + dv * dv
        k = int(np.argmin(d2))
        if d2[k] > 40 * 40:                                     # clicked on the robot object: nearest other pixel
            return None
    cell = np.floor((q - q.min(axis=0)) / FOOT_CELL).astype(int)
    grid = np.zeros(tuple(cell.max(axis=0) + 1), np.uint8)
    grid[cell[:, 0], cell[:, 1]] = 1
    grid = cv2.dilate(grid, np.ones((3, 3), np.uint8))          # bridges 1-2 cm gaps between footprint cells
    _, lab = cv2.connectedComponents(grid)
    pick = lab[cell[k, 0], cell[k, 1]]
    mem = lab[cell[:, 0], cell[:, 1]] == pick
    centre = q[k]
    for _ in range(3):                                          # stay on the clicked object
        near = mem & (np.linalg.norm(q - centre, axis=1) <= max_radius)
        if near.sum() < 40:
            break
        centre = np.median(q[near], axis=0)
    mem = mem & (np.linalg.norm(q - centre, axis=1) <= max_radius)
    if mem.sum() < 40:
        return None
    pix = np.stack([uu[mem] + u0, vv[mem] + v0], axis=1).astype(np.int32)
    return {"pts": pts[mem], "contour": cv2.convexHull(pix), "n_px": int(mem.sum()),
            "touches_window": bool(uu[mem].min() == 0 or vv[mem].min() == 0 or
                                   uu[mem].max() == above.shape[1] - 1 or vv[mem].max() == above.shape[0] - 1)}


def height_overlay(img, depth_mm, plane, intr, cam, c) -> None:
    """Key D: colour every pixel by its height above the table (blue low ... red high, up to 30 cm)."""
    if depth_mm is None or plane is None:
        return
    h, w = depth_mm.shape[:2]
    P, ok, _ = depth_points(depth_mm, 0, w, 0, h, intr, cam)
    H = np.where(ok, (P - plane[0]) @ plane[1], 0.0)
    m = ok & (H > c["object_min_height_m"]) & (H < 1.0)
    col = cv2.applyColorMap(np.clip(H / 0.30 * 255, 0, 255).astype(np.uint8), cv2.COLORMAP_JET)
    if col.shape[:2] != img.shape[:2]:
        col = cv2.resize(col, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST)
        m = cv2.resize(m.astype(np.uint8), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    img[m] = (0.3 * img[m] + 0.7 * col[m]).astype(img.dtype)
    nodepth = ~ok
    if nodepth.shape != img.shape[:2]:
        nodepth = cv2.resize(nodepth.astype(np.uint8), (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0
    img[nodepth] = (0.4 * img[nodepth] + 0.6 * np.array([255, 0, 255])).astype(img.dtype)


def object_geometry(obj: Dict, plane) -> Tuple[np.ndarray, float]:
    """(foot centre on the table in camera coordinates, height of the top above the table).
    The camera sees the top and the near side of the object only, so the mean of the points would be pulled towards
    the camera; the centre of the smallest rectangle around the points projected on the table is not."""
    p0, n = plane
    h = (obj["pts"] - p0) @ n
    top = float(np.percentile(h, 95))
    foot = obj["pts"] - np.outer(h, n)
    e1, e2 = table_basis(n)
    q = np.stack([foot @ e1, foot @ e2], axis=1)
    r = np.linalg.norm(q - np.median(q, axis=0), axis=1)
    q = q[r <= np.percentile(r, 98)]                    # drop stray depth pixels at the object edge
    (cx, cy), _, _ = cv2.minAreaRect(q.astype(np.float32))
    p = cx * e1 + cy * e2
    return p + (np.dot(p0 - p, n)) * n, top


# ---------------------------------------------------------------------------------------------------------------
# geometry in W and checks
# ---------------------------------------------------------------------------------------------------------------
def box_distance(p, lo, hi) -> float:
    return float(np.linalg.norm(np.maximum(np.maximum(lo - p, 0.0), p - hi)))


def segment_box_distance(a, b, lo, hi, n=60) -> float:
    return min(box_distance(a + s * (b - a), lo, hi) for s in np.linspace(0.0, 1.0, n))


def robot_segments(tool, shoulder_z) -> List[Tuple[np.ndarray, np.ndarray, float]]:
    base, sh = np.zeros(3), np.array([0.0, 0.0, shoulder_z])
    el = 0.5 * (sh + tool) + np.array([0.0, 0.0, ROBOT_ELBOW_LIFT])
    return [(base, sh, ROBOT_RADII[0]), (sh, el, ROBOT_RADII[1]), (el, tool, ROBOT_RADII[2])]


def auto_handover(area: Dict, c: Dict) -> np.ndarray:
    """Just outside the assembly area on the robot side, on the line from the area centre to the robot base."""
    x0, x1 = area["x"]; y0, y1 = area["y"]
    cen = np.array([0.5 * (x0 + x1), 0.5 * (y0 + y1)])
    u = -cen / max(np.linalg.norm(cen), 1e-6)
    s = []
    for k, (lo, hi) in enumerate(((x0, x1), (y0, y1))):
        if u[k] < -1e-9:
            s.append((lo - cen[k]) / u[k])
        elif u[k] > 1e-9:
            s.append((hi - cen[k]) / u[k])
    edge = cen + min(s) * u
    xy = edge + c["handover_offset_m"] * u
    return np.array([xy[0], xy[1], c["handover_height_m"]])


def derive(pl: Dict, c: Dict) -> Tuple[Dict, Dict, Dict]:
    a = pl["assembly_area"]
    lo = np.array([a["x"][0], a["y"][0], 0.0])
    hi = np.array([a["x"][1], a["y"][1], c["area_height_m"]])
    corners = [np.array([x, y, HAND_HEIGHT]) for x in a["x"] for y in a["y"]]
    centre_y = 0.5 * (a["y"][0] + a["y"][1])
    collab = np.array([a["x"][0] + 0.05, centre_y, c["collab_height_m"]])
    handover, storage, safe = np.array(pl["handover"]), np.array(pl["storage"]), np.array(pl["safe"])
    d_collab = float(c.get("collab_radius_m", 0.30))   # collaborative radius around the robot tool [m] (config)
    d_assist = max(d_collab + 0.10, max(np.linalg.norm(handover - q) for q in corners) + 0.05)
    poses = {"safe": safe, "storage": storage, "handover": handover, "collab": collab}
    zones = {"dProhibit": float(c.get("prohibit_radius_m", 0.05)), "dCollabMax": d_collab, "dAssistMax": d_assist,
             "assistRadius": float(c.get("assist_radius_m", 0.12))}   # fixed assistive zone around the handover point
    shz = pl["robot_height"]
    arm_clear = min(segment_box_distance(s, e, lo, hi) - r for s, e, r in robot_segments(safe, shz))
    safe_to_area = box_distance(safe, np.array([lo[0], lo[1], 0]), np.array([hi[0], hi[1], HAND_HEIGHT]))
    inside = lo[0] <= handover[0] <= hi[0] and lo[1] <= handover[1] <= hi[1]
    gap = box_distance(handover, np.array([lo[0], lo[1], HAND_HEIGHT]), np.array([hi[0], hi[1], HAND_HEIGHT]))
    path = min(segment_box_distance(p, q, lo, hi) for p, q in ((safe, storage), (storage, handover), (handover, safe))) - ROBOT_RADII[2]
    # (passed, value text, blocking)
    checks = {
        "robot at the safe position clear of the assembly area": (arm_clear >= c["safe_clearance_m"], f"{arm_clear:.2f} m (>= {c['safe_clearance_m']:.2f})", True),
        "handover point outside the assembly area": (not inside, "inside!" if inside else "ok", True),
        "robot paths do not cross the assembly area": (path >= 0.02, f"{path:.2f} m", True),
        "idle robot counts as separated (else: assistive)": (safe_to_area >= d_assist, f"{safe_to_area:.2f} m (>= {d_assist:.2f})", False),
        "handover point within reach": (gap <= c["reach_max_m"], f"{gap:.2f} m (<= {c['reach_max_m']:.2f})", False),
    }
    return poses, zones, checks


# ---------------------------------------------------------------------------------------------------------------
class Calibrator:
    def __init__(self, cfg: Dict, intr, rframe: Optional["rf.RobotFrame"] = None, arm=None) -> None:
        self.cfg, self.c, self.intr, self.cam = cfg, cfg["pomdp_bridge"]["calibration"], intr, cfg["camera"]
        self.rframe, self.arm = rframe, arm                 # real robot mode: camera -> robot calibration + robot (read)
        self.safe_robot_mm: Optional[np.ndarray] = None     # real robot mode: safe tool position, robot frame
        self.theta_deg = 0.0
        self.table_z = None                                 # real robot mode: table height in W (should be ~0)
        self.step = 1
        self.depth = None
        self.plane = None                               # table plane from the assembly rectangle
        self.robot: Optional[Dict] = None
        self.storage: Optional[Dict] = None
        self.area: Optional[Dict] = None                # {"px": ((u0, v0), (u1, v1))}
        self.handover: Optional[Tuple] = None           # None = automatic, ("click", (u, v)) or ("hand", p_cam)
        self.frame: Optional[WorkspaceFrame] = None
        self.drag = None
        self.show_height = False
        self.msg = "Step 1: " + STEP_PROMPT[1]

    # -- mouse --------------------------------------------------------------------------------------------------
    def mouse(self, event, x, y, flags, param) -> None:
        if self.step == 1:
            if event == cv2.EVENT_LBUTTONDOWN:
                self.drag = ((x, y), (x, y))
            elif event == cv2.EVENT_MOUSEMOVE and self.drag is not None:
                self.drag = (self.drag[0], (x, y))
            elif event == cv2.EVENT_LBUTTONUP and self.drag is not None:
                (u0, v0), _ = self.drag
                self.drag = None
                if abs(x - u0) > 10 and abs(y - v0) > 10:
                    self.set_area_drag(((u0, v0), (x, y)))
        elif event == cv2.EVENT_LBUTTONUP:
            if self.step == 2 and self.real_robot:
                self.msg = ROBOT_STEP_PROMPT
            elif self.step in (2, 3):
                self.set_object(self.step, x, y)
            elif self.step == 4:
                if table_patch(self.depth, x, y, self.intr, self.cam) is None:
                    self.msg = "handover: no depth on the table there - click a nearby spot"
                else:
                    self.handover = ("click", (x, y))
                    self.msg = "handover point moved (click again to move it, V to save)"

    @property
    def real_robot(self) -> bool:
        return self.rframe is not None

    def prompt(self, step: int) -> str:
        return ROBOT_STEP_PROMPT if step == 2 and self.real_robot else STEP_PROMPT[step]

    def next_step(self) -> None:
        robot_done = self.safe_robot_mm if self.real_robot else self.robot
        todo = [s for s, ok in ((1, self.area), (2, robot_done), (3, self.storage)) if ok is None]
        self.step = todo[0] if todo else 4
        self.msg += "   ->   step %d: %s" % (self.step, self.prompt(self.step))

    def read_safe_pose(self) -> None:
        """Real robot mode, step 2: the robot's current tool position is the safe position (read only)."""
        pose = rf.tool_pose(self.arm) if self.arm is not None else None
        if pose is None:
            self.msg = "could not read the robot tool position - is the robot on and the relay open?"
            return
        self.safe_robot_mm = np.array(pose[:3])
        self.msg = "SAFE position = robot tool at [%.0f %.0f %.0f] mm (robot frame)" % tuple(self.safe_robot_mm)
        self.update_frame()
        self.next_step()

    def find_object(self, u: int, v: int, key: str) -> Optional[Dict]:
        radius = self.c["robot_max_radius_m"] if key == "robot" else self.c["storage_max_radius_m"]
        exclude = None
        if key == "storage" and self.robot is not None:                 # leave the robot bottle out of the storage
            exclude = (object_geometry(self.robot, self.plane)[0], self.c["robot_max_radius_m"])
        obj = segment_object(self.depth, u, v, self.plane, self.intr, self.cam, self.c, radius, exclude)
        if obj is not None:
            obj["click"] = (u, v)
        return obj

    def set_object(self, step: int, u: int, v: int) -> None:
        if self.plane is None:
            self.msg = "step 1 first: drag the assembly area (it measures the table)"
            return
        name = "ROBOT" if step == 2 else "STORAGE"
        obj = self.find_object(u, v, "robot" if step == 2 else "storage")
        if obj is None:
            self.msg = f"no {name} object found there - click on the object itself (D shows what the camera sees)"
            return
        setattr(self, "robot" if step == 2 else "storage", obj)
        _, top = object_geometry(obj, self.plane)
        self.msg = f"{name} object found: height {100 * top:.0f} cm"
        if obj["touches_window"]:
            self.msg += " (object larger than object_window_px - increase it in config.json)"
        elif step == 2 and top < 0.08 and self.c["robot_height_override_m"] <= 0:
            self.msg += " - too small? the camera may see only a part of it (D), or set robot_height_override_m"
        self.update_frame()
        self.next_step()

    def set_area_drag(self, px) -> None:
        (a0, b0), (a1, b1) = px
        u0, u1, v0, v1 = min(a0, a1), max(a0, a1), min(b0, b1), max(b0, b1)
        if self.depth is None:
            self.msg = "no depth image - is the depth stream running?"
            return
        P, ok, _ = depth_points(self.depth, u0, u1 + 1, v0, v1 + 1, self.intr, self.cam)
        if ok.sum() < 500:
            self.msg = "assembly area: almost no depth in that rectangle - drag again over the table"
            return
        self.plane = plane_fit(P[ok])
        self.area = {"px": px}
        self.msg = "ASSEMBLY area and table set"
        for key in ("robot", "storage"):                # objects found before: measure again on the new table
            obj = getattr(self, key)
            if obj is not None:
                setattr(self, key, self.find_object(*obj["click"], key))
        self.update_frame()
        self.next_step()

    def area_centre_cam(self) -> Optional[np.ndarray]:
        p0, n = self.plane
        (a, b) = self.area["px"]
        return ray_plane(0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]), self.intr, p0, n)

    def update_frame(self) -> None:
        if self.real_robot:                             # W on the real robot base, x towards the assembly area
            if self.area is None or self.plane is None:
                self.frame = None
                return
            towards = self.rframe.to_robot(1000.0 * self.area_centre_cam())
            f = WorkspaceFrame()
            f.R, f.origin, f.R_w_robot, self.theta_deg = rf.workspace_frame_from_robot(self.rframe, towards)
            self.frame = f
            self.table_z = float(f.to_w(self.plane[0])[2])
            return
        if self.robot is None or self.area is None or self.plane is None:
            self.frame = None
            return
        foot, _ = object_geometry(self.robot, self.plane)
        self.frame = WorkspaceFrame.from_table(foot, self.plane[1], self.area_centre_cam())

    # -- places in W ----------------------------------------------------------------------------------------------
    def table_w(self, u, v) -> np.ndarray:
        p0, n = self.plane
        q = self.frame.to_w(ray_plane(u, v, self.intr, p0, n))
        q[2] = 0.0
        return q

    def places(self) -> Optional[Dict]:
        if self.frame is None or self.storage is None:
            return None
        if self.real_robot and self.safe_robot_mm is None:
            return None
        c = self.c
        if self.real_robot:
            robot_h = measured_h = LITE6_SHOULDER_M
        else:
            _, robot_h = object_geometry(self.robot, self.plane)
            measured_h = robot_h
            if c["robot_height_override_m"] > 0:
                robot_h = float(c["robot_height_override_m"])
        (a, b) = self.area["px"]
        q = [self.table_w(*a), self.table_w(*b), self.table_w(a[0], b[1]), self.table_w(b[0], a[1])]
        xs, ys = [p[0] for p in q], [p[1] for p in q]
        area = {"x": [float(min(xs)), float(max(xs))], "y": [float(min(ys)), float(max(ys))], "height": c["area_height_m"]}
        sfoot, sh = object_geometry(self.storage, self.plane)
        storage = self.frame.to_w(sfoot) * np.array([1, 1, 0]) + np.array([0, 0, sh + c["storage_above_object_m"]])
        safe = (self.frame.robot_to_w(self.safe_robot_mm) if self.real_robot   # real robot: its safe tool position
                else np.array([0.0, 0.0, robot_h]))                    # tip of the robot object
        if self.handover is None:
            hand = auto_handover(area, c)
        elif self.handover[0] == "click":
            hand = self.table_w(*self.handover[1]) + np.array([0, 0, c["handover_height_m"]])
        else:
            hand = self.frame.to_w(self.handover[1])
        return {"base": [0.0, 0.0, 0.0], "assembly_area": area, "handover": [float(v) for v in hand],
                "storage": [float(v) for v in storage], "safe": [float(v) for v in safe],
                "robot_height": robot_h, "robot_height_measured": measured_h, "storage_object_height": sh,
                "handover_mode": "automatic" if self.handover is None else self.handover[0]}

    # -- drawing --------------------------------------------------------------------------------------------------
    def project(self, p_w) -> Optional[Tuple[int, int]]:
        pc = self.frame.R.T @ np.asarray(p_w, dtype=float) + self.frame.origin
        if pc[2] <= 0.05:
            return None
        return int(self.intr.fx * pc[0] / pc[2] + self.intr.cx), int(self.intr.fy * pc[1] / pc[2] + self.intr.cy)

    def poly(self, img, pts_w, color, thick=2, fill=None) -> None:
        q = [self.project(p) for p in pts_w]
        if any(x is None for x in q):
            return
        arr = np.array(q, np.int32)
        if fill is not None:
            ov = img.copy()
            cv2.fillPoly(ov, [arr], fill)
            cv2.addWeighted(ov, 0.25, img, 0.75, 0, img)
        cv2.polylines(img, [arr], True, color, thick, cv2.LINE_AA)

    def draw(self, img) -> Optional[Dict]:
        if self.show_height:
            height_overlay(img, self.depth, self.plane, self.intr, self.cam, self.c)
        for obj, col, name in ((self.robot, (255, 255, 255), "ROBOT"), (self.storage, (200, 200, 200), "STORAGE")):
            if obj is not None:
                cv2.polylines(img, [obj["contour"].astype(np.int32)], True, col, 2, cv2.LINE_AA)
                x, y = obj["contour"].reshape(-1, 2).min(axis=0)
                cv2.putText(img, name, (int(x), int(y) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        if self.drag is not None:
            cv2.rectangle(img, self.drag[0], self.drag[1], (0, 255, 0), 2)
        if self.frame is None:
            return None
        o = self.project(np.zeros(3))
        if o:
            for axis, col in ((np.array([0.15, 0, 0]), (0, 0, 255)), (np.array([0, 0.15, 0]), (0, 255, 0))):
                e = self.project(axis)
                if e:
                    cv2.arrowedLine(img, o, e, col, 2, tipLength=0.2)
        pl = self.places()
        if pl is None:
            return None
        poses, zones, checks = derive(pl, self.c)
        a = pl["assembly_area"]
        rect = [np.array([x, y, 0.0]) for x, y in ((a["x"][0], a["y"][0]), (a["x"][1], a["y"][0]), (a["x"][1], a["y"][1]), (a["x"][0], a["y"][1]))]
        self.poly(img, rect, (60, 200, 60), 2, fill=(60, 200, 60))
        th = np.linspace(0, 2 * np.pi, 72)
        for centre, rad, col in ((poses["handover"], zones["dAssistMax"], (0, 200, 255)), (poses["collab"], zones["dCollabMax"], (0, 128, 255))):
            self.poly(img, [centre + rad * np.array([np.cos(t), np.sin(t), 0.0]) for t in th], col, 1)
        for name, key, col in (("handover" + (" (auto)" if self.handover is None else ""), "handover", (255, 160, 0)),
                               ("storage (tool)", "storage", (200, 200, 200)), ("collab. work point", "collab", (0, 128, 255)),
                               ("safe position (robot tip)", "safe", (255, 255, 255))):
            p, base = self.project(poses[key]), self.project(poses[key] * np.array([1, 1, 0]))
            if p and base:
                cv2.line(img, base, p, col, 1)
                cv2.circle(img, p, 7, col, -1)
                cv2.putText(img, name, (p[0] + 9, p[1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.55, col, 2, cv2.LINE_AA)
        return {"places": pl, "poses": poses, "zones": zones, "checks": checks}


def text(img, s, x, y, col=(0, 255, 255), scale=0.6) -> None:
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, 1, cv2.LINE_AA)


def panel(img, y0, y1) -> None:
    y0, y1 = max(0, y0), min(img.shape[0], y1)
    img[y0:y1] = (0.35 * img[y0:y1]).astype(img.dtype)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default="config.json")
    ap.add_argument("--stand-in", action="store_true", help="ignore the robot-camera calibration: robot = clicked object")
    args = ap.parse_args(argv)
    logging.basicConfig(level="INFO", format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        log.error("Configuration error: %s", exc)
        return 2
    bc = cfg["pomdp_bridge"]
    out = Path(bc["workspace_file"])
    if not out.is_absolute():
        out = Path(args.config).resolve().parent / out
    cam = cfg["camera"]
    source = OrbbecRGBD(cam)                                    # depth is required
    source.start()
    cfg["physical"]["enabled"] = False                          # hand model only (key H)
    cfg["body_presence"]["enabled"] = False
    vision = create_vision(cfg)
    intr = source.intrinsics
    inf_w = min(cfg["vision"]["inference_width"], intr.width)
    inf_h = int(round(intr.height * inf_w / intr.width))
    rframe, arm = None, None
    if not args.stand_in:
        rframe = rf.RobotFrame(rf.CALIB_FILE)
        if rframe.calibrated:
            try:
                arm = rf.connect_robot(enable_motion=False)        # read only: the safe tool position (step 2)
            except RuntimeError as exc:
                log.warning("robot not reachable (%s): step 2 cannot read the safe position", exc)
            log.info("REAL ROBOT mode: W on the robot base (robot-camera calibration %s, rms %.1f mm)",
                     rframe.data.get("created"), rframe.data.get("rms_mm", float("nan")))
        else:
            rframe = None
            log.info("no robot-camera calibration: the robot is a clicked stand-in object")
    cal = Calibrator(cfg, intr, rframe, arm)
    win = "Workstation calibration - click on the objects"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, cal.mouse)
    hand: Optional[Dict] = None                                 # running hand capture (key H)
    try:
        while True:
            frame = source.read()
            if frame is None:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                continue
            cal.depth = frame.depth_mm if frame.depth_aligned else None
            img = frame.color_bgr.copy()
            if hand is not None:
                small = cv2.resize(frame.color_bgr, (inf_w, inf_h))
                _, hands = vision.detect(small, frame.t)
                palms = []
                for hd in hands[:2]:
                    pts = [point_3d(cal.depth, hd[i], (inf_w, inf_h), intr, 4, cam["depth_min_mm"], cam["depth_max_mm"]) for i in PALM_IDX]
                    pts = [p for p in pts if p is not None]
                    if len(pts) >= 2:
                        palms.append(np.median(np.array(pts), axis=0))
                if palms:
                    hand["p"].append(palms)
                if frame.t - hand["t0"] >= 3.0:
                    if len(hand["p"]) >= 10:
                        cal.handover = ("hand", np.median(np.array([p[0] for p in hand["p"]]), axis=0))
                        cal.msg = "handover point from your hand: OK (V to save)"
                    else:
                        cal.msg = "hand not seen or no depth - try again"
                    hand = None
            res = cal.draw(img)
            panel(img, 0, 100 if hand is not None else 72)
            if res is not None:
                panel(img, img.shape[0] - 40 - 26 * len(res["checks"]), img.shape[0])
            text(img, f"step {cal.step}: {cal.prompt(cal.step)}   |   1-4 redo, D height view, H hand = handover, V save, Q quit", 16, 30)
            text(img, cal.msg, 16, 60, (255, 255, 255))
            if hand is not None:
                text(img, "hold your hand still at the handover point ...", 16, 90, (0, 200, 255))
            can_save = False
            if res is not None:
                can_save = all(ok for ok, _, block in res["checks"].values() if block)
                y = img.shape[0] - 20 - 26 * (len(res["checks"]) - 1)
                for i, (name, (ok, val, block)) in enumerate(res["checks"].items()):
                    col, tag = ((80, 220, 80), "OK  ") if ok else (((60, 60, 255), "NO  ") if block else ((0, 210, 255), "WARN"))
                    text(img, f"{tag} {name}: {val}", 16, y + 26 * i, col, 0.55)
                z = res["zones"]
                if cal.real_robot:
                    tz = cal.table_z if cal.table_z is not None else float("nan")
                    text(img, f"REAL robot frame: x turned {cal.theta_deg:.0f} deg from the robot x  |  table {100 * tz:+.1f} cm "
                              f"from the robot base height" + ("  <- more than 3 cm: is the robot on this table?" if abs(tz) > 0.03 else ""),
                         16, img.shape[0] - 46 - 26 * len(res["checks"]), (0, 230, 255) if abs(tz) <= 0.03 else (60, 60, 255), 0.55)
                text(img, f"robot height {100 * res['places']['robot_height']:.0f} cm  |  collaborative distance {z['dCollabMax']:.2f} m, "
                          f"assistive {z['dAssistMax']:.2f} m, prohibited {z['dProhibit']:.2f} m (fixed)",
                     16, img.shape[0] - 20 - 26 * len(res["checks"]), (255, 255, 255), 0.55)
            cv2.imshow(win, img)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            if ord("1") <= key <= ord("4"):
                cal.step = key - ord("0")
                if cal.step == 4:
                    cal.handover = None
                cal.msg = f"step {cal.step}: {cal.prompt(cal.step)}"
                if cal.step == 2 and cal.real_robot:
                    cal.read_safe_pose()
            elif key in (ord("d"), ord("D")):
                cal.show_height = not cal.show_height
                cal.msg = ("height view: coloured = above the table (blue low, red 30 cm+), magenta = no depth"
                           if cal.show_height else "height view off")
                if cal.show_height and cal.plane is None:
                    cal.msg = "height view needs the table: do step 1 first"
            elif key in (ord("h"), ord("H")):
                if cal.frame is None:
                    cal.msg = "do steps 1 and 2 first"
                else:
                    hand = {"kind": "H", "t0": frame.t, "p": []}
            elif key in (ord("v"), ord("V")):
                if res is None:
                    cal.msg = "steps 1-3 are needed before saving"
                elif not can_save:
                    cal.msg = "a red check blocks saving - move the object / redo that step"
                else:
                    pl = res["places"]
                    extra = {"method": "real robot" if cal.real_robot else "objects", "places_W": {
                                 "base": pl["base"], "assembly_area": pl["assembly_area"], "handover": pl["handover"],
                                 "storage": pl["storage"], "safe": pl["safe"], "handover_mode": pl["handover_mode"]},
                             "robot": {"shoulder_height": round(float(pl["robot_height"]), 4),
                                       "object_height": round(float(pl["robot_height_measured"]), 4)},
                             "storage_object_height": round(float(pl["storage_object_height"]), 4),
                             "robot_poses_W": {k: [round(float(x), 4) for x in v] for k, v in res["poses"].items()},
                             "zones": {k: round(float(v), 4) for k, v in res["zones"].items()},
                             "checks": {k: bool(v[0]) for k, v in res["checks"].items()},
                             "settings": dict(cal.c), "camera": {"color": source.color_profile_desc, "depth_align": source.align_mode}}
                    if cal.real_robot:
                        extra["real_robot"] = {"theta_deg": round(cal.theta_deg, 2), "table_z_m": round(float(cal.table_z), 4),
                                               "safe_tool_robot_mm": [round(float(v), 1) for v in cal.safe_robot_mm],
                                               "robot_camera_calibration": rf.CALIB_FILE.name,
                                               "robot_camera_calibration_created": cal.rframe.data.get("created"),
                                               "use": "p_W_m = R_w_robot @ p_robot_mm / 1000"}
                    cal.frame.save(str(out), extra)
                    cal.msg = f"Saved {out}"
                    log.info("Saved %s", out)
    finally:
        vision.close()
        source.stop()
        if arm is not None:
            arm.disconnect()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
