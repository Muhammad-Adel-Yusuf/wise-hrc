"""Live view of the workstation exactly as MATLAB (liveHRC) uses it: places and zones on the camera image.

    python workstation_view.py

Reads POMDP model/workspace_calibration.json (define_workstation.py, W = robot frame in metres) and draws:
  * ASSEMBLY AREA (green box)
  * ASSISTIVE zone (orange): MATLAB's fixed assistive zone = vertical cylinder of radius zones.assistRadius around the
    HANDOVER point (top view), from the table to the handover height
  * COLLABORATIVE zone (blue ring, radius dCollabMax) and PROHIBITED zone (red ring, radius dProhibit) around the
    robot tool, live (they move with the robot)
  * SAFE position (white), HANDOVER point (orange), STORAGE pick point (grey; an arrow at the image edge when it is
    outside the camera view), collaborative work point (blue), STORAGE BOXES (light blue, define_storage_boxes.py)
  * your hands with the zone MATLAB would give them (0 prohibited, 1 collaborative, 2 assistive, 3 separated), their
    distance to the robot tool, and your head with its distance to the robot tool.
The robot is only READ (tool position), never moved. Keys: Q / Esc quit.
The drawing is the class WorkstationOverlay, also used by calibration_wizard.py.
"""
from __future__ import annotations

import json
import os
import sys
import time
from pathlib import Path
from typing import List, Optional

import cv2
import numpy as np

from fatigue_monitor.config import load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.vision import FaceHandVision
from fatigue_monitor import robot_frame as rf

# same values as POMDP model/stateDefinitionHRC.m (not stored in the calibration file)
DPROHIBIT_DEFAULT_M = 0.05      # prm.dProhibit when the workstation file has no zones.dProhibit
ASSIST_RADIUS_DEFAULT_M = 0.12  # prm.assistZone.radius when the workstation file has no zones.assistRadius
DISPLAY_SCALE = 0.75
HERE = Path(__file__).resolve().parent
WIN = "workstation as MATLAB sees it (robot frame)"
ZONE_NAMES = {0: "PROHIBITED", 1: "COLLABORATIVE", 2: "ASSISTIVE", 3: "separated"}
ZONE_COLOURS = {0: (0, 0, 255), 1: (255, 120, 0), 2: (0, 165, 255), 3: (180, 180, 180)}
BOX_EDGES = [(0, 1), (2, 3), (4, 5), (6, 7), (0, 2), (1, 3), (4, 6), (5, 7), (0, 4), (1, 5), (2, 6), (3, 7)]


def workspace_path(cfg) -> Path:
    p = Path(cfg["pomdp_bridge"]["workspace_file"])
    return p if p.is_absolute() else (HERE / p).resolve()


class WorkstationOverlay:
    """Places and zones of a real-robot workspace file, drawn on a camera image (robot frame, mm)."""

    def __init__(self, ws: dict, rframe: rf.RobotFrame, intr) -> None:
        self.rframe, self.intr = rframe, intr
        Rwr = np.array(ws["R_w_robot"])
        to_robot = lambda p_w: Rwr.T @ (np.asarray(p_w, dtype=float) * 1000.0)    # W [m] -> robot [mm]
        pl, poses, zones = ws["places_W"], ws["robot_poses_W"], ws["zones"]
        self.area = a = pl["assembly_area"]
        self.d_collab = float(zones["dCollabMax"])
        self.assist_r = float(zones.get("assistRadius", ASSIST_RADIUS_DEFAULT_M))
        self.d_prohibit = float(zones.get("dProhibit", DPROHIBIT_DEFAULT_M))
        self.handover = to_robot(poses["handover"])
        self.safe, self.storage, self.collab = to_robot(poses["safe"]), to_robot(poses["storage"]), to_robot(poses["collab"])
        self.area_c = [to_robot([x, y, z]) for x in a["x"] for y in a["y"] for z in (0.0, a["height"])]
        sb = ws.get("storage_baseline", {})                       # storage boxes (define_storage_boxes.py), robot mm
        self.storage_boxes = [(b["name"].replace("storage_", ""), [[x, y, z] for z in (0.0, float(sb.get("rim_mm", 100.0)))
                                                                   for x, y in b["corners_robot_mm"]])
                              for b in sb.get("boxes", [])]
        self.th = np.linspace(0, 2 * np.pi, 73)

    def uv(self, p):
        return rf.project(self.rframe.to_cam(np.asarray(p, dtype=float)), self.intr)

    def ring(self, img, centre, radius_mm, z, col, thick=2):
        q = [self.uv([centre[0] + radius_mm * np.cos(t), centre[1] + radius_mm * np.sin(t), z]) for t in self.th]
        q = [p for p in q if p]
        if len(q) > 3:
            cv2.polylines(img, [np.array(q, np.int32)], True, col, thick, cv2.LINE_AA)

    def place(self, img, p, col, label):
        q, f = self.uv(p), self.uv([p[0], p[1], 0.0])
        h, w = img.shape[:2]
        if q and 0 <= q[0] < w and 0 <= q[1] < h:
            if f:
                cv2.line(img, f, q, col, 1, cv2.LINE_AA)
            cv2.circle(img, q, 10, col, -1)
            rf.put_text(img, label, (q[0] + 12, q[1] - 8), col, 0.75, 2)
        elif q:                                                  # outside the image: arrow at the edge
            c = np.array([w / 2, h / 2])
            d = np.array(q, float) - c
            s = min((w / 2 - 40) / max(abs(d[0]), 1e-6), (h / 2 - 40) / max(abs(d[1]), 1e-6))
            e = (c + d * s).astype(int)
            cv2.arrowedLine(img, tuple((c + d * s * 0.85).astype(int)), tuple(e), col, 4, tipLength=0.4)
            rf.put_text(img, label + " (outside the view)", (int(min(e[0], w - 420)), int(e[1]) - 14), col, 0.7, 2)

    def zone(self, p_mm, tool) -> Optional[int]:
        """MATLAB zone of a palm point (zoneClassifyHRC with the fixed assistive zone)."""
        if tool is None:
            return None
        d = np.linalg.norm(p_mm - tool) / 1000.0
        if d < self.d_prohibit:
            return 0
        if d <= self.d_collab:
            return 1
        if np.linalg.norm(p_mm[:2] - self.handover[:2]) <= self.assist_r * 1000.0:
            return 2
        return 3

    def in_area(self, p_mm) -> bool:
        a = self.area
        return a["x"][0] <= p_mm[0] / 1000 <= a["x"][1] and a["y"][0] <= p_mm[1] / 1000 <= a["y"][1]

    def draw(self, img, hands_mm: List[np.ndarray], tool, head_mm=None) -> List[str]:
        """Draw everything on img (full resolution); returns text rows about the hands / head."""
        q = [self.uv(c) for c in self.area_c]
        ov = img.copy()
        top = [q[i] for i in (1, 3, 7, 5)]
        if all(top):
            cv2.fillPoly(ov, [np.array(top, np.int32)], (60, 220, 60))
        cv2.addWeighted(ov, 0.18, img, 0.82, 0, img)
        for i, j in BOX_EDGES:
            if q[i] and q[j]:
                cv2.line(img, q[i], q[j], (60, 220, 60), 3, cv2.LINE_AA)
        if q[7]:
            rf.put_text(img, "ASSEMBLY AREA", (q[7][0] + 10, q[7][1]), (60, 220, 60), 0.8, 2)
        for name, c in self.storage_boxes:                        # storage boxes: wireframe, light blue
            q = [self.uv(p) for p in c]
            for i, j in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
                if q[i] and q[j]:
                    cv2.line(img, q[i], q[j], (255, 200, 80), 2, cv2.LINE_AA)
            top = [p for p in q[4:] if p]
            if top:
                rf.put_text(img, "STORAGE " + name, (min(p[0] for p in top), min(p[1] for p in top) - 8), (255, 200, 80), 0.6, 2)
        r = self.assist_r * 1000.0                               # assistive cylinder around the handover point
        hv = self.handover
        self.ring(img, hv, r, 0.0, (0, 165, 255), 2)
        self.ring(img, hv, r, hv[2], (0, 165, 255), 2)
        for t in np.linspace(0, 2 * np.pi, 9)[:-1]:
            a0 = self.uv([hv[0] + r * np.cos(t), hv[1] + r * np.sin(t), 0.0])
            a1 = self.uv([hv[0] + r * np.cos(t), hv[1] + r * np.sin(t), hv[2]])
            if a0 and a1:
                cv2.line(img, a0, a1, (0, 165, 255), 1, cv2.LINE_AA)
        if tool is not None:                                     # zones around the robot tool (live)
            self.ring(img, tool, self.d_collab * 1000.0, tool[2], (255, 120, 0), 2)
            self.ring(img, tool, self.d_prohibit * 1000.0, tool[2], (0, 0, 255), 2)
            qt = self.uv(tool)
            if qt:
                cv2.circle(img, qt, 6, (255, 0, 255), -1)
                rf.put_text(img, "robot tool", (qt[0] + 10, qt[1] + 22), (255, 0, 255), 0.65, 2)
        self.place(img, self.safe, (255, 255, 255), "SAFE")
        self.place(img, self.handover, (0, 165, 255), "HANDOVER / ASSISTIVE ZONE")
        self.place(img, self.storage, (200, 200, 200), "STORAGE")
        self.place(img, self.collab, (255, 120, 0), "collab. work point")
        rows = []
        for p in hands_mm:
            z = self.zone(p, tool)
            d = np.linalg.norm(p - tool) / 1000.0 if tool is not None else float("nan")
            col = ZONE_COLOURS.get(z, (255, 255, 255))
            qh = self.uv(p)
            if qh:
                cv2.circle(img, qh, 9, col, -1)
                rf.put_text(img, "hand: zone %s %s, %.2f m to tool%s" % (
                    z if z is not None else "?", ZONE_NAMES.get(z, ""), d, " | in assembly area" if self.in_area(p) else ""),
                    (qh[0] + 12, qh[1] + 6), col, 0.65, 2)
            rows.append("hand [%.0f %.0f %.0f] mm: zone %s %s" % (*p, z, ZONE_NAMES.get(z, "")))
        if head_mm is not None:
            d = np.linalg.norm(head_mm - tool) / 1000.0 if tool is not None else float("nan")
            qh = self.uv(head_mm)
            if qh:
                cv2.circle(img, qh, 9, (0, 200, 255), -1)
                rf.put_text(img, "head: %.2f m to tool" % d, (qh[0] + 12, qh[1] + 6), (0, 200, 255), 0.65, 2)
            rows.append("head [%.0f %.0f %.0f] mm: %.2f m to the robot tool" % (*head_mm, d))
        return rows

    def legend(self) -> str:
        return ("zones as in MATLAB: 0 prohibited < %.2f m, 1 collaborative <= %.2f m (around the robot tool), "
                "2 assistive = %.3f m around the handover point, 3 separated" % (self.d_prohibit, self.d_collab, self.assist_r))

    def places_text(self) -> str:
        a = self.area
        return "assembly x %.0f..%.0f  y %.0f..%.0f  top %.0f mm  |  safe %s  handover %s  storage %s  (robot frame mm)" % (
            1000 * a["x"][0], 1000 * a["x"][1], 1000 * a["y"][0], 1000 * a["y"][1], 1000 * a["height"],
            np.round(self.safe).astype(int).tolist(), np.round(self.handover).astype(int).tolist(),
            np.round(self.storage).astype(int).tolist())


def detect_people(f, vision, rframe, intr, cc, want_head=True):
    """(visible palms [mm, robot frame], head [mm] or None) of one frame."""
    h, w = f.color_bgr.shape[:2]
    s = 640 / w
    face, hands = vision.detect(cv2.resize(f.color_bgr, (640, int(h * s))), f.t)
    dmin, dmax = cc["depth_min_mm"], cc["depth_max_mm"]
    palms = [rframe.to_robot(rf.palm_point_mm(f.depth_mm, hd / s, intr, dmin, dmax)) for hd in hands]
    palms = [p for p in palms if p is not None]
    head = None
    if want_head:
        if face is not None:
            head = rf.head_point_mm(f.depth_mm, face / s, intr, dmin, dmax)
        body = getattr(vision, "last_body", None)
        if head is None and body is not None and len(body) >= 33:
            b = body.copy()
            b[:, :2] /= s
            head = rf.pose_head_point_mm(f.depth_mm, b, intr, dmin, dmax)
        head = rframe.to_robot(head)
    return palms, head


def main() -> int:
    os.chdir(HERE)
    cfg = load_config(str(HERE / "config.json"))
    wpath = workspace_path(cfg)
    ws = json.loads(wpath.read_text(encoding="utf-8"))
    if "R_w_robot" not in ws:
        print("%s is not on the real robot: run define_workstation.py first" % wpath)
        return 1
    rframe = rf.RobotFrame(rf.CALIB_FILE)
    cfg["body_presence"]["enabled"] = True                      # pose model: head when the face model misses it
    cfg.setdefault("physical", {})["enabled"] = False
    cc = cfg["camera"]
    cam = OrbbecRGBD(cc)
    cam.start()
    intr = cam.intrinsics
    vision = FaceHandVision(cfg)
    poller = rf.RobotPoller(10)
    ov = WorkstationOverlay(ws, rframe, intr)
    k = 0
    try:
        while True:
            f = cam.read()
            if f is None or f.depth_mm is None or not f.depth_aligned:
                continue
            k += 1
            vision.force_pose = (k % 2 == 0)
            palms, head = detect_people(f, vision, rframe, intr, cc)
            last = poller.latest()
            tool = last["pts"]["tool"] if last is not None and time.monotonic() - last["t"] < 1.0 else None
            img = f.color_bgr.copy()
            rows = ov.draw(img, palms, tool, head)
            img = cv2.resize(img, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)
            lines = ["Q quit  |  " + ov.legend(), ov.places_text(),
                     "robot tool: %s" % (np.round(tool).astype(int).tolist() if tool is not None else poller.status)] + rows
            y = 20
            for ln in lines:
                rf.put_text(img, ln, (8, y), (255, 255, 255), 0.48)
                y += 19
            cv2.imshow(WIN, img)
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        poller.close()
        vision.close()
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
