"""Storage area = the BLUE storage boxes, found by colour + depth and anchored on robot-recorded box centres.

    python define_storage_boxes.py              measure, save to the workspace file, write the check image
    python define_storage_boxes.py --dry-run    measure and write the check image only

Inputs
  * POMDP model/workstation_points.txt, lines  storage_box1_centre:[J1..J6, x, y, z]  storage_box2_centre:[...]
    (robot moved by hand to the middle of each box; joints in degrees, tool x, y, z in mm, robot frame);
  * the robot-camera calibration (POMDP model/robot_camera_calibration.json);
  * the camera (nothing may stand between it and the boxes; the robot is not used).
Method
  1. 30 frames of colour + depth. The depth uses the camera's WIDE mode (WFOV 512x512): the normal mode (NFOV) does
     not cover the right part of the colour image, where the boxes are. Its distances differ by ~5 mm from the
     normal mode: no matter for the box outline (the hand tracking keeps the normal mode).
  2. Blue pixels (HSV), the blob(s) under the projected box centres; pixels with unstable or inconsistent depth
     (flying pixels at edges) are dropped; the rest -> robot frame.
  3. Walls: histogram peaks of the upper (rim / wall) points - along the line through the box centres the outer walls
     and the dividers; across it the near and the far wall line (shared by the boxes of the row: the strongest peak on
     each side of the recorded centres); rim height = 95th percentile of the height.
  4. The measured boxes are shifted along that line so that their middles match the recorded centres (the robot
     positions are accurate to ~1 mm; the camera may be off by a few cm at the image edge). The shift is reported.
Output (POMDP model/workspace_calibration.json, a backup is written first): storage_baseline = the boxes (corners,
rim height, recorded centres) + their combined area; storage_object_height = rim height. The robot's storage POINT
(places_W.storage = the action point pos2 that action 0 drives to; MATLAB's FETCH -> GRIP needs the real tool there)
is NOT changed - a warning says when that point is not inside a box.
Check image: POMDP model/workstation_preview/storage_boxes.png (camera view + top view).
"""
from __future__ import annotations

import argparse
import datetime as dt
import json
import re
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, List

import cv2
import numpy as np

from fatigue_monitor import robot_frame as rf
from fatigue_monitor.config import load_config
from fatigue_monitor.orbbec_camera import Intrinsics, frame_to_bgr

HERE = Path(__file__).resolve().parent
POINTS_FILE = rf.PROJECT / "POMDP model" / "workstation_points.txt"
PREVIEW = rf.PROJECT / "POMDP model" / "workstation_preview"
BLUE_HUE = (105, 125)                    # OpenCV hue of the blue boxes (0..179)
BLUE_SAT_MIN, BLUE_VAL_MIN = 110, 40     # the worker's grey-blue sleeves are paler (saturation ~105)
N_FRAMES = 30
MAX_SPREAD_MM, MAX_LOCAL_MM = 25.0, 15.0  # depth stable over time / consistent with its 5x5 neighbourhood


def workspace_path(cfg) -> Path:
    p = Path(cfg["pomdp_bridge"]["workspace_file"])
    return p if p.is_absolute() else (HERE / p).resolve()


def read_centres() -> List[Dict]:
    out = []
    for ln in POINTS_FILE.read_text(encoding="utf-8").splitlines():
        m = re.match(r"\s*(storage_box\d+)_centre\s*:\s*\[([^\]]+)\]", ln)
        if m:
            v = [float(s) for s in m.group(2).split(",")]
            out.append({"name": m.group(1), "joints_deg": v[:6], "centre_mm": np.array(v[6:9])})
    if not out:
        sys.exit("no 'storage_boxN_centre:[J1..J6, x, y, z]' lines in %s" % POINTS_FILE)
    return sorted(out, key=lambda b: b["name"])


def capture(cam_cfg) -> tuple:
    """(colour BGR, depth mm aligned to colour (0 = none), depth spread mm, colour intrinsics)."""
    import pyorbbecsdk as ob
    pipe = ob.Pipeline()
    cfg = ob.Config()
    cp = pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR).get_video_stream_profile(
        cam_cfg["color_width"], cam_cfg["color_height"], ob.OBFormat.MJPG, 30)
    cfg.enable_stream(cp)
    cfg.enable_stream(pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_video_stream_profile(
        512, 512, ob.OBFormat.Y16, 15))
    pipe.enable_frame_sync()
    pipe.start(cfg)
    i = pipe.get_camera_param().rgb_intrinsic
    w, h = cp.get_width(), cp.get_height()
    intr = Intrinsics(float(i.fx), float(i.fy), float(i.cx), float(i.cy), int(i.width) or w, int(i.height) or h).scaled(w, h)
    al = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
    ds, col = [], None
    t_end, warm = time.time() + 20, time.time() + 2
    try:
        while time.time() < t_end and len(ds) < N_FRAMES:
            fs = pipe.wait_for_frames(300)
            if fs is None or fs.get_depth_frame() is None or fs.get_color_frame() is None or time.time() < warm:
                continue
            a = al.process(fs)
            if a is None:
                continue
            a = a.as_frame_set() if hasattr(a, "as_frame_set") else a
            d = a.get_depth_frame()
            ds.append(np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).copy())
            col = frame_to_bgr(a.get_color_frame(), ob)
    finally:
        pipe.stop()
    if len(ds) < 10:
        sys.exit("only %d depth frames from the camera" % len(ds))
    n = len(ds)
    D = np.sort(np.stack(ds), axis=0)                            # zeros (no depth) first
    nv = (D > 0).sum(0)
    pick = lambda q: np.take_along_axis(D, np.clip(n - nv + (q * nv) // 10, 0, n - 1)[None], 0)[0].astype(np.float32)
    med, spread = pick(5), pick(9) - pick(1)
    med[nv < n // 2] = 0.0
    return col, med, spread, intr


def blue_points(col, depth, spread, intr, rframe, boxes):
    """Robot-frame points [mm] of the blue blob(s) under the box centres + the masks (selected, good)."""
    hsv = cv2.cvtColor(col, cv2.COLOR_BGR2HSV)
    m = ((hsv[..., 0] >= BLUE_HUE[0]) & (hsv[..., 0] <= BLUE_HUE[1]) & (hsv[..., 1] >= BLUE_SAT_MIN)
         & (hsv[..., 2] >= BLUE_VAL_MIN)).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, lab, stats, _ = cv2.connectedComponentsWithStats(m)
    keep = set()
    for b in boxes:
        uv = rf.project(rframe.to_cam(b["centre_mm"]), intr)
        b["uv"] = uv
        if uv is None or not (0 <= uv[0] < lab.shape[1] and 0 <= uv[1] < lab.shape[0]):
            sys.exit("%s centre is outside the camera image - turn the camera so that the boxes are in view" % b["name"])
        best, bd = 0, 1e9                                       # blob under the centre, else the nearest within 60 px
        for k in range(1, n):
            ys, xs = np.nonzero(lab[max(0, uv[1] - 60):uv[1] + 61, max(0, uv[0] - 60):uv[0] + 61] == k)
            if len(xs) and stats[k, cv2.CC_STAT_AREA] > 500:
                dd = np.min(np.hypot(xs + max(0, uv[0] - 60) - uv[0], ys + max(0, uv[1] - 60) - uv[1]))
                if dd < bd:
                    best, bd = k, dd
        if not best:
            sys.exit("no blue box found near the projected %s centre %s" % (b["name"], uv))
        keep.add(best)
    sel = np.isin(lab, sorted(keep))
    loc = cv2.medianBlur(np.clip(depth, 0, 65535).astype(np.uint16), 5).astype(np.float32)
    good = sel & (depth > 200) & (spread < MAX_SPREAD_MM) & (np.abs(depth - loc) < MAX_LOCAL_MM)
    vs, us = np.nonzero(good)
    z = depth[vs, us]
    pc = np.stack([(us - intr.cx) * z / intr.fx, (vs - intr.cy) * z / intr.fy, z], 1)
    return (rframe.R @ pc.T).T + rframe.t, sel, good


def peak_near(vals, target, lo, hi, step=5.0):
    """Histogram peak of vals in [lo, hi] (a wall seen from above); target if there are no points."""
    v = vals[(vals >= lo) & (vals <= hi)]
    if len(v) < 20:
        return target
    h, e = np.histogram(v, bins=np.arange(lo, hi + step, step))
    h = np.convolve(h, [1, 2, 1], mode="same")
    i = int(np.argmax(h))
    return float((e[i] + e[i + 1]) / 2)


def fit_boxes(pts, boxes):
    """Rectangles (robot frame, mm) of the boxes, side by side along the line through their recorded centres."""
    c = np.array([b["centre_mm"][:2] for b in boxes])
    u = (c[-1] - c[0]) / np.linalg.norm(c[-1] - c[0]) if len(c) > 1 else np.array([1.0, 0.0])
    v = np.array([-u[1], u[0]])
    o = c.mean(0)
    rim = float(np.percentile(pts[:, 2], 95))
    up = pts[pts[:, 2] > 0.45 * rim]                             # walls and rims (not the floors / contents)
    pu, pv = (up[:, :2] - o) @ u, (up[:, :2] - o) @ v
    cu = (c - o) @ u
    sp = float(np.min(np.diff(cu))) if len(c) > 1 else 130.0
    walls = [peak_near(pu, cu[0] - sp / 2, cu[0] - 1.0 * sp, cu[0] - 0.15 * sp)]
    walls += [peak_near(pu, (cu[i] + cu[i + 1]) / 2, cu[i] + 0.2 * sp, cu[i + 1] - 0.2 * sp) for i in range(len(c) - 1)]
    walls += [peak_near(pu, cu[-1] + sp / 2, cu[-1] + 0.15 * sp, cu[-1] + 1.0 * sp)]
    mids = np.array([(walls[i] + walls[i + 1]) / 2 for i in range(len(c))])
    # across the row: the boxes stand in one row, so they share the near and the far wall line; the recorded centres
    # lie between them -> strongest wall peak on each side of the centres (the open front lip is low, the back high)
    inrow = (pu > walls[0]) & (pu < walls[-1])
    pr, vc = pv[inrow], float(np.mean((c - o) @ v))
    lo, hi = np.percentile(pr, [0.5, 99.5])
    v0, v1 = peak_near(pr, lo, lo, vc - 10.0), peak_near(pr, hi, vc + 10.0, hi)
    shift = float(np.mean(cu - mids))                           # anchor on the robot-recorded centres
    out = []
    for i, b in enumerate(boxes):
        a0, a1 = walls[i] + shift, walls[i + 1] + shift
        inb = (pu > walls[i]) & (pu < walls[i + 1])
        corners = [o + a * u + w * v for a, w in ((a0, v0), (a1, v0), (a1, v1), (a0, v1))]
        out.append({"name": b["name"], "centre_tool_mm": [round(float(x), 1) for x in b["centre_mm"]],
                    "joints_deg": b["joints_deg"],
                    "corners_robot_mm": [[round(float(p[0]), 1), round(float(p[1]), 1)] for p in corners],
                    "size_mm": [round(float(a1 - a0)), round(float(v1 - v0))], "n_points": int(inb.sum())})
    return out, rim, shift, u


def inside(p_xy, corners) -> bool:
    return cv2.pointPolygonTest(np.array(corners, np.float32), (float(p_xy[0]), float(p_xy[1])), False) >= 0


def draw(col, sel, good, pts, fitted, rim, boxes, rframe, intr, pick_mm) -> np.ndarray:
    img = col.copy()
    img[sel & ~good] = (img[sel & ~good] * 0.5 + np.array([255, 0, 255]) * 0.5).astype(np.uint8)
    for f in fitted:
        q = [rf.project(rframe.to_cam(np.array([x, y, z])), intr) for z in (0.0, rim) for x, y in f["corners_robot_mm"]]
        for i, j in [(0, 1), (1, 2), (2, 3), (3, 0), (4, 5), (5, 6), (6, 7), (7, 4), (0, 4), (1, 5), (2, 6), (3, 7)]:
            if q[i] and q[j]:
                cv2.line(img, q[i], q[j], (0, 255, 255), 2, cv2.LINE_AA)
    for b in boxes:
        if b.get("uv"):
            cv2.circle(img, b["uv"], 7, (255, 255, 255), 2)
            rf.put_text(img, b["name"].replace("storage_", ""), (b["uv"][0] - 40, b["uv"][1] - 12), (255, 255, 255), 0.6, 2)
    tv = np.full((600, 600, 3), 255, np.uint8)                   # top view, 1 mm = 1 px around the boxes
    allc = np.array([c for f in fitted for c in f["corners_robot_mm"]])
    org = allc.mean(0)
    P = lambda x, y: (int(300 + (x - org[0])), int(300 - (y - org[1])))
    for g in range(-300, 301, 50):
        cv2.line(tv, (300 + g, 0), (300 + g, 599), (235, 235, 235), 1)
        cv2.line(tv, (0, 300 + g), (599, 300 + g), (235, 235, 235), 1)
    for x, y, z in pts[np.argsort(pts[:, 2])]:
        p = P(x, y)
        if 0 <= p[0] < 600 and 0 <= p[1] < 600:
            t = float(np.clip(z / max(rim, 1), 0, 1))
            tv[p[1], p[0]] = (int(255 * (1 - t)), int(150 * (1 - t)), int(255 * t))
    for f in fitted:
        cv2.polylines(tv, [np.array([P(*c) for c in f["corners_robot_mm"]], np.int32)], True, (0, 160, 0), 2)
        cx, cy = np.mean(f["corners_robot_mm"], 0)
        cv2.putText(tv, "%s %dx%d mm" % (f["name"].replace("storage_", ""), *f["size_mm"]), (P(cx, cy)[0] - 60, P(cx, cy)[1]),
                    0, 0.45, (0, 120, 0), 1, cv2.LINE_AA)
    for b in boxes:
        cv2.circle(tv, P(*b["centre_mm"][:2]), 5, (0, 0, 0), 2)
    if pick_mm is not None:
        cv2.drawMarker(tv, P(*pick_mm[:2]), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        cv2.putText(tv, "robot storage point (action 0)", (P(*pick_mm[:2])[0] + 8, P(*pick_mm[:2])[1]), 0, 0.4, (0, 0, 255), 1)
    cv2.putText(tv, "top view, robot frame (x right, y up), grid 50 mm, colour = height", (8, 18), 0, 0.45, (80, 80, 80), 1)
    h = img.shape[0]
    tv = cv2.resize(tv, (h, h))
    return np.hstack([img, tv])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="do not change the workspace file")
    a = ap.parse_args()
    cfg = load_config()
    rframe = rf.RobotFrame(rf.CALIB_FILE)
    if not rframe.calibrated:
        sys.exit("no robot-camera calibration: run robot_camera_calibration.py first")
    boxes = read_centres()
    wpath = workspace_path(cfg)
    ws = json.loads(wpath.read_text(encoding="utf-8"))
    Rwr = np.array(ws["R_w_robot"])
    pick_mm = Rwr.T @ (np.array(ws["robot_poses_W"]["storage"]) * 1000.0)
    print("capturing %d frames (wide depth mode) ..." % N_FRAMES, flush=True)
    col, depth, spread, intr = capture(cfg["camera"])
    pts, sel, good = blue_points(col, depth, spread, intr, rframe, boxes)
    print("blue pixels %d, with good depth %d (%.0f %%)" % (sel.sum(), good.sum(), 100.0 * good.sum() / max(1, sel.sum())))
    if good.sum() < 2000:
        sys.exit("too few blue points with depth - is the view of the boxes free?")
    fitted, rim, shift, u = fit_boxes(pts, boxes)
    for f in fitted:
        print("  %s: %d x %d mm (along x' across), corners %s, %d wall points" % (
            f["name"], *f["size_mm"], f["corners_robot_mm"], f["n_points"]))
    allc = np.array([c for f in fitted for c in f["corners_robot_mm"]])
    area = {"xmin": float(allc[:, 0].min()), "xmax": float(allc[:, 0].max()), "ymin": float(allc[:, 1].min()),
            "ymax": float(allc[:, 1].max()), "zmax": rim}
    print("  rim height %.0f mm | combined area x %.0f..%.0f  y %.0f..%.0f mm" % (rim, area["xmin"], area["xmax"], area["ymin"], area["ymax"]))
    print("  camera vs recorded centres along the row of boxes: %+.0f mm (boxes shifted onto the recorded centres)" % shift)
    pick_in = any(inside(pick_mm, f["corners_robot_mm"]) for f in fitted)
    if not pick_in:
        print("  WARNING: the robot's storage point (action 0 -> pos2, [%.0f %.0f %.0f] mm) is NOT inside a storage box."
              " Update pos0-pos2 in robot_code/tasks/robot_points.txt to reach into a box." % tuple(pick_mm))
    PREVIEW.mkdir(parents=True, exist_ok=True)
    png = PREVIEW / "storage_boxes.png"
    cv2.imwrite(str(png), draw(col, sel, good, pts, fitted, rim, boxes, rframe, intr, pick_mm))
    print("check image: %s" % png)
    if a.dry_run:
        print("dry run: workspace file not changed")
        return 0
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    shutil.copy2(wpath, PREVIEW / ("workspace_calibration.json.bak_%s" % stamp))
    ws["storage_baseline"] = {
        "method": "blue storage boxes: colour + wide-mode depth, anchored on robot-recorded box centres",
        "created": dt.datetime.now().isoformat(timespec="seconds"),
        "boxes": fitted, "rim_mm": round(rim, 1), "area_robot_mm": {k: round(v, 1) for k, v in area.items()},
        "camera_shift_mm": round(shift, 1), "points_file": str(POINTS_FILE.relative_to(rf.PROJECT)),
        "check_image": str(png.relative_to(rf.PROJECT))}
    ws["storage_object_height"] = round(rim / 1000.0, 4)
    ws.setdefault("checks", {})["robot storage point inside a storage box"] = bool(pick_in)
    wpath.write_text(json.dumps(ws, indent=2) + "\n", encoding="utf-8")
    print("saved: %s (backup in %s)" % (wpath, PREVIEW))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
