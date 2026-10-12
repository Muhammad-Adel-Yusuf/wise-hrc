"""Live: where are the worker's hands and head with respect to the REAL robot?

    python robot_frame_live.py

Needs POMDP model/robot_camera_calibration.json (robot_camera_calibration.py). The robot is only READ (tool position),
never moved. Shown on the camera image and printed twice a second:
  * head (face surface) and each palm in the robot base frame [mm],
  * their distance to the robot base and to the robot tool,
  * a magenta circle where the calibration puts the robot tool: it must sit on the tool. If it does not, re-calibrate.
Keys: Q / Esc quit.
"""
from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np

from fatigue_monitor.config import ConfigError, load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.vision import FaceHandVision
from fatigue_monitor import robot_frame as rf

HERE = Path(__file__).resolve().parent
WIN = "hands and head in the robot frame"
DISPLAY_SCALE = 0.75
PRINT_EVERY_S = 0.5
FACE_OUTLINE = (10, 338, 297, 332, 284, 251, 389, 356, 454, 323, 361, 288, 397, 365, 379, 378, 400, 377, 152, 148, 176,
                149, 150, 136, 172, 58, 132, 93, 234, 127, 162, 21, 54, 103, 67, 109)   # face-mesh oval


def fmt(p):
    return "[%5.0f %5.0f %5.0f]" % tuple(p) if p is not None else "[  ---   ---   ---]"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=str(HERE / "config.json"))
    ap.add_argument("--no-robot", action="store_true", help="do not read the robot tool position")
    args = ap.parse_args(argv)
    os.chdir(HERE)                                               # model / config paths are relative
    logging.basicConfig(level=logging.WARNING, format="%(levelname)s %(name)s: %(message)s")
    frame_rb = rf.RobotFrame(rf.CALIB_FILE)
    if not frame_rb.calibrated:
        print("no calibration: run  python robot_camera_calibration.py  first (%s)" % rf.CALIB_FILE)
        return 1
    print("calibration %s: rms %.1f mm, %d samples" % (frame_rb.data.get("created"), frame_rb.data.get("rms_mm", -1),
                                                        len(frame_rb.data.get("samples", []))))
    try:
        cfg = load_config(args.config)
    except ConfigError as exc:
        print("config error: %s" % exc)
        return 1
    cfg["body_presence"]["enabled"] = True                      # pose model: head and hands when the face / hand
    cfg.setdefault("physical", {})["enabled"] = False           # models miss them (small or turned face)
    cc = cfg["camera"]
    cam = OrbbecRGBD(cc)
    cam.start()
    intr = cam.intrinsics
    vision = FaceHandVision(cfg)
    vision.force_pose = True                                     # pose model on every frame
    arm = None
    if not args.no_robot:
        try:
            arm = rf.connect_robot(enable_motion=False)
        except RuntimeError as exc:
            print("robot not read (%s): tool distances not shown" % exc)
    inf_w = int(cfg["vision"]["inference_width"])
    last_print = 0.0
    try:
        while True:
            f = cam.read()
            if f is None or f.depth_mm is None or not f.depth_aligned:
                continue
            img = f.color_bgr
            h, w = img.shape[:2]
            s = inf_w / w
            face, hands = vision.detect(cv2.resize(img, (inf_w, int(h * s))), f.t)
            face = face / s if face is not None else None
            hands = [hd / s for hd in hands]
            body = vision.last_body
            if body is not None and len(body) >= 33:
                body = body.copy()
                body[:, :2] /= s
            else:
                body = None
            dmin, dmax = cc["depth_min_mm"], cc["depth_max_mm"]
            # head: face model (precise) or pose model (nose, eyes, ears; works at a distance / turned away)
            head, head_uv, head_src = None, None, ""
            if face is not None:
                head, head_uv, head_src = rf.head_point_mm(f.depth_mm, face, intr, dmin, dmax), face[168], "face"
            if head is None and body is not None:
                vis = [i for i in rf.POSE_HEAD_IDX if body[i, 2] >= 0.5]
                if vis:
                    head = rf.pose_head_point_mm(f.depth_mm, body, intr, dmin, dmax)
                    head_uv, head_src = body[vis, :2].mean(axis=0), "pose"
            head = frame_rb.to_robot(head)
            # hands: hand model (palm) or pose model (wrist + finger bases) for each hand it missed
            palms = []
            for hd in hands:
                p = frame_rb.to_robot(rf.palm_point_mm(f.depth_mm, hd, intr, dmin, dmax))
                palms.append((hd[list(rf.PALM_IDX)].mean(axis=0), p, "palm"))
            if body is not None and len(palms) < 2:
                for side, p in rf.pose_hand_points_mm(f.depth_mm, body, intr, dmin, dmax).items():
                    idx = [i for i in rf.POSE_HAND_IDX[side] if body[i, 2] >= 0.5]
                    if not idx:
                        continue
                    uv = body[idx, :2].mean(axis=0)
                    if any(np.hypot(*(uv - q[0])) < 80 for q in palms):   # the hand model already has this hand
                        continue
                    palms.append((uv, frame_rb.to_robot(p), side + " hand (pose)"))
            tool = rf.tool_pose(arm) if arm is not None else None
            tool_p = np.array(tool[:3]) if tool else None

            out = img.copy()

            def label(uv, text, col):
                uv = (int(uv[0]), int(uv[1]))
                cv2.circle(out, uv, 8, col, -1)
                rf.put_text(out, text, (uv[0] + 12, uv[1]), col, 0.7, 2)

            def dists(p):
                if p is None:
                    return "no depth"
                txt = "base %.0f" % np.linalg.norm(p)
                if tool_p is not None:
                    txt += "  tool %.0f" % np.linalg.norm(p - tool_p)
                return txt + " mm"

            # what each model sees: face outline (orange), hand points (green), pose head/hand points (cyan)
            if face is not None:
                for i in FACE_OUTLINE:
                    cv2.circle(out, (int(face[i, 0]), int(face[i, 1])), 2, (0, 165, 255), -1)
            for hd in hands:
                for u, v in hd[:, :2]:
                    cv2.circle(out, (int(u), int(v)), 3, (0, 255, 0), -1)
            if body is not None:
                for i in rf.POSE_HEAD_IDX + rf.POSE_HAND_IDX["left"] + rf.POSE_HAND_IDX["right"]:
                    if body[i, 2] >= 0.5:
                        cv2.circle(out, (int(body[i, 0]), int(body[i, 1])), 5, (255, 255, 0), 2)
            rows = ["detected: face model %s | hand model %d hand(s) | pose model %s" % (
                "YES" if face is not None else "no", len(hands), "YES" if body is not None else "no")]
            if head_uv is not None:
                label(head_uv, "head %s" % dists(head), (0, 200, 255))
                rows.append("head (%s) %s  %s" % (head_src, fmt(head), dists(head)))
            for i, (uv, p, name) in enumerate(palms):
                label(uv, "hand %s" % dists(p), (0, 255, 0))
                rows.append("%s %s  %s" % (name, fmt(p), dists(p)))
            if tool_p is not None:
                uv = rf.project(frame_rb.to_cam(tool_p), intr)
                if uv:
                    cv2.circle(out, uv, 14, (255, 0, 255), 3)
                rows.append("tool  %s" % fmt(tool_p))
            out = cv2.resize(out, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)   # text after the resize: sharp
            y = 24
            for ln in ["robot frame [mm]  (Q quit)   orange: face model  green: hand model  cyan: pose model"] + rows:
                rf.put_text(out, ln, (10, y))
                y += 22
            cv2.imshow(WIN, out)
            if time.time() - last_print > PRINT_EVERY_S and len(rows) > 1:
                last_print = time.time()
                print(" | ".join(rows[1:]))
            if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                break
    finally:
        vision.close()
        cam.stop()
        if arm is not None:
            arm.disconnect()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
