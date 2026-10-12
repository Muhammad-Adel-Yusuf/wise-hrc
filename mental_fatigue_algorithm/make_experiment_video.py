"""Make the experiment video from one recording session (started automatically by MATLAB liveHRC at P / X / Esc).

    python make_experiment_video.py "<...>/POMDP model/videos/session_<date>_<time>"

Inputs in the session folder:  status.csv (MATLAB, one row per decision), and from main.py:
                                camera.mp4 + camera_times.csv (15 frames/s) and plots.mp4 + plot_times.csv
                                (screen capture of MATLAB's live window, 2 frames/s).
Output: experiment_video.mp4 (1920 x 1080, 15 frames/s, MPEG-4) in the same folder:
  header (title, date, elapsed time, REC) | worker camera | MATLAB live decision window |
  status tiles (mode, robot action, mental fatigue, attention, hand-robot distance and zone, safety) |
  session timeline (robot action, mental fatigue, hand zone; cursor = now).
The streams are synchronised by their wall-clock time stamps.
"""
from __future__ import annotations

import csv
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1920, 1080, 15.0
FONT_DIR = Path("C:/Windows/Fonts")
BG, CARD, BORDER = (243, 245, 248), (255, 255, 255), (220, 225, 232)
NAVY, INK, MUTED = (11, 37, 69), (24, 32, 44), (110, 120, 135)
RED = (208, 58, 58)
ACTION = {0: ("tr", "Task-layer motion", "slow"), 1: ("sc", "Solo - normal speed", "normal"), 2: ("ss", "Solo - slow", "slow"),
          3: ("ap", "Approach the worker", "slow"), 4: ("ah", "Assistive hold (handover)", "stop"),
          5: ("cn", "Collaborative - normal", "normal"), 6: ("cs", "Collaborative - slow", "slow"),
          7: ("ch", "Collaborative hold", "stop"), 8: ("ra", "Assist / reallocate", "slow"), 9: ("rt", "Retreat to safe", "slow"),
          10: ("ps", "PROTECTIVE STOP", "protective"), 11: ("wt", "Wait at safe position", "stop")}
SPEED_COL = {"normal": (30, 90, 168), "slow": (110, 168, 225), "stop": (242, 176, 30), "protective": (208, 58, 58)}
LEVEL = {0: ("Low", (46, 158, 79)), 1: ("Medium", (242, 176, 30)), 2: ("High", (208, 58, 58))}
ATT = {0: ("Focused", (46, 158, 79)), 1: ("Uncertain", (242, 176, 30)), 2: ("Distracted", (208, 58, 58))}
ZONE = {0: ("Prohibited", (208, 58, 58)), 1: ("Collaborative", (232, 128, 90)), 2: ("Assistive", (242, 176, 30)),
        3: ("Separated", (46, 158, 79))}
RISK = {0: ("No risk", (46, 158, 79)), 1: ("Potential risk", (242, 176, 30)), 2: ("ACTIVE RISK", (208, 58, 58))}
GREY = (170, 176, 186)


def font(size, bold=False):
    name = "segoeuib.ttf" if bold else "segoeui.ttf"
    try:
        return ImageFont.truetype(str(FONT_DIR / name), size)
    except OSError:
        return ImageFont.load_default()


F = {k: font(*v) for k, v in {"title": (34, True), "sub": (19, False), "cap": (17, True), "tile_t": (15, True),
                                "tile_v": (30, True), "tile_s": (16, False), "clock": (30, True), "small": (14, False),
                                "band": (15, True)}.items()}


def num(x):
    try:
        v = float(x)
        return None if math.isnan(v) else v
    except (TypeError, ValueError):
        return None


def read_times(p: Path):
    if not p.exists():
        return np.zeros(0)
    with open(p, encoding="utf-8") as f:
        next(f, None)
        return np.array([float(x) for x in f if x.strip()])


def read_status(p: Path):
    with open(p, encoding="utf-8") as f:
        return list(csv.DictReader(f))


def clock(s):
    s = max(0, int(s))
    return "%02d:%02d:%02d" % (s // 3600, (s % 3600) // 60, s % 60)


def rounded(d, box, r, fill, outline=None, width=1):
    d.rounded_rectangle(box, radius=r, fill=fill, outline=outline, width=width)


def fit(img, w, h):
    ih, iw = img.shape[:2]
    s = min(w / iw, h / ih)
    return cv2.resize(img, (max(1, int(iw * s)), max(1, int(ih * s))), interpolation=cv2.INTER_AREA)


class Composer:
    def __init__(self, d: Path) -> None:
        self.d = d
        self.status = read_status(d / "status.csv")
        self.t_plot = read_times(d / "plot_times.csv")
        self.t_cam = read_times(d / "camera_times.csv")
        self.t0 = float(self.status[0]["t_wall"]) if self.status else self.t_plot[0]
        self.t1 = (float(self.status[-1]["t_wall"]) if self.status else self.t_plot[-1]) + 0.5
        self.t_status = np.array([float(r["t_wall"]) for r in self.status])
        self.started = time.strftime("%Y-%m-%d  %H:%M", time.localtime(self.t0))
        self.mode = max(set(r["mode"] for r in self.status), key=lambda m: sum(r["mode"] == m for r in self.status))
        # layout
        self.cam_box = (24, 104, 24 + 820, 104 + 462)                    # camera card (image 800 x 450)
        self.fig_box = (868, 104, W - 24, 566 + 36)                      # MATLAB window card
        self.tiles_y = (622, 790)
        self.tl_box = (24, 808, W - 24, H - 18)                          # timeline card
        self.base = self._static()
        self.timeline = self._timeline_layer()

    # ---------- static parts
    def _static(self):
        img = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(img)
        d.rectangle((0, 0, W, 84), fill=NAVY)
        d.text((28, 10), "Fatigue-aware Human-Robot Collaboration", font=F["title"], fill=(255, 255, 255))
        d.text((30, 52), "POMDP decision layer  |  UFactory Lite 6  |  live experiment  |  mode at start: %s" % self.mode,
               font=F["sub"], fill=(178, 196, 222))
        d.text((W - 430, 50), "started " + self.started, font=F["sub"], fill=(178, 196, 222))
        for box, cap in ((self.cam_box, "WORKER CAMERA  -  Orbbec RGB-D"), (self.fig_box, "LIVE DECISION WINDOW  -  MATLAB POMDP (2 updates / s)")):
            rounded(d, (box[0] + 3, box[1] + 3, box[2] + 3, box[3] + 3), 12, (226, 230, 236))
            rounded(d, box, 12, CARD, BORDER)
            d.text((box[0] + 16, box[3] - 30), cap, font=F["cap"], fill=MUTED)
        rounded(d, (self.tl_box[0] + 3, self.tl_box[1] + 3, self.tl_box[2] + 3, self.tl_box[3] + 3), 12, (226, 230, 236))
        rounded(d, self.tl_box, 12, CARD, BORDER)
        d.text((self.tl_box[0] + 16, self.tl_box[1] + 10), "SESSION TIMELINE", font=F["cap"], fill=MUTED)
        return img

    def _x(self, t):
        x0, x1 = self.tl_box[0] + 190, self.tl_box[2] - 24
        return x0 + (x1 - x0) * (t - self.t0) / max(self.t1 - self.t0, 1e-6)

    def _timeline_layer(self):
        img = Image.new("RGB", (W, H), BG)
        img.paste(self.base)
        d = ImageDraw.Draw(img)
        bands = [("Robot action", lambda r: SPEED_COL[ACTION.get(int(num(r["final"]) or 0), ACTION[0])[2]]),
                 ("Mental fatigue", lambda r: LEVEL.get(int(num(r["Fm"])) if num(r["Fm"]) is not None else -1, (None, GREY))[1]),
                 ("Hand zone", lambda r: ZONE.get(int(num(r["zone"])) if num(r["zone"]) is not None else -1, (None, GREY))[1])]
        y = self.tl_box[1] + 42
        for name, colour in bands:
            d.text((self.tl_box[0] + 18, y + 6), name, font=F["band"], fill=INK)
            for i, r in enumerate(self.status):
                ta = self.t_status[i]
                tb = self.t_status[i + 1] if i + 1 < len(self.status) else self.t1
                d.rectangle((self._x(ta), y, self._x(tb) + 1, y + 34), fill=colour(r))
            y += 50
        # time axis
        span = self.t1 - self.t0
        step = next(s for s in (10, 15, 30, 60, 120, 300, 600, 1200, 3600) if span / s <= 12)
        for k in range(0, int(span) + 1, step):
            x = self._x(self.t0 + k)
            d.line((x, y - 4, x, y + 2), fill=MUTED)
            d.text((x - 18, y + 4), "%d:%02d" % (k // 60, k % 60), font=F["small"], fill=MUTED)
        # legend
        lx = self.tl_box[0] + 190
        for label, col in (("normal speed", SPEED_COL["normal"]), ("slow", SPEED_COL["slow"]), ("stopped / hold", SPEED_COL["stop"]),
                           ("protective stop", SPEED_COL["protective"]), ("low", LEVEL[0][1]), ("medium", LEVEL[1][1]),
                           ("high", LEVEL[2][1]), ("unknown", GREY)):
            d.rectangle((lx, y + 30, lx + 14, y + 44), fill=col)
            d.text((lx + 20, y + 28), label, font=F["small"], fill=INK)
            lx += 30 + int(d.textlength(label, font=F["small"])) + 18
        return np.array(img)[:, :, ::-1].copy()                        # BGR for OpenCV

    # ---------- per status row
    def tiles(self, r):
        layer = Image.new("RGB", (W, self.tiles_y[1] - self.tiles_y[0] + 8), BG)
        d = ImageDraw.Draw(layer)
        a = int(num(r["final"]) or 0)
        short, name, speed = ACTION.get(a, ACTION[0])
        fm, fp, att = num(r["Fm"]), num(r["Fp"]), num(r["attention"])
        zone, risk, hand = num(r["zone"]), num(r["risk"]), num(r["hand_m"])
        lev = lambda v, table: table.get(int(v), ("Unknown", GREY)) if v is not None else ("Unknown", GREY)
        robot_cmd = num(r.get("robot_action"))
        tiles = [
            ("MODE  /  PHASE", r["mode"].capitalize(), r["phase"], (30, 90, 168)),
            ("ROBOT ACTION", "%s  %s" % (short, name.split(" - ")[0] if len(name) > 22 else name), "planner: %s%s" % (
                r["planner"], "   |   real robot: action %d" % robot_cmd if robot_cmd is not None and robot_cmd >= 0 else ""),
             SPEED_COL[speed]),
            ("MENTAL FATIGUE", lev(fm, LEVEL)[0], "physical: %s" % lev(fp, LEVEL)[0], lev(fm, LEVEL)[1]),
            ("ATTENTION", lev(att, ATT)[0], "camera estimate", lev(att, ATT)[1]),
            ("HAND  -  ROBOT TOOL", "%.2f m" % hand if hand is not None else "-", "zone: %s" % lev(zone, ZONE)[0],
             lev(zone, ZONE)[1]),
            ("SAFETY", lev(risk, RISK)[0] if num(r["verified"]) else "Not verified",
             (r["supervisor"] or "monitoring").replace("_", " ")[:34], lev(risk, RISK)[1] if num(r["verified"]) else GREY),
        ]
        n, gap = len(tiles), 16
        tw = (W - 48 - gap * (n - 1)) / n
        for i, (title, value, sub, col) in enumerate(tiles):
            x0 = 24 + i * (tw + gap)
            box = (x0, 0, x0 + tw, self.tiles_y[1] - self.tiles_y[0])
            rounded(d, (box[0] + 3, box[1] + 3, box[2] + 3, box[3] + 3), 12, (226, 230, 236))
            rounded(d, box, 12, CARD, BORDER)
            d.rounded_rectangle((box[0], box[1], box[0] + 8, box[3]), radius=4, fill=col)
            d.text((x0 + 22, 14), title, font=F["tile_t"], fill=MUTED)
            val_font = F["tile_v"]
            while d.textlength(value, font=val_font) > tw - 36 and val_font.size > 16:
                val_font = font(val_font.size - 2, True)
            d.text((x0 + 22, 44), value, font=val_font, fill=col if col != GREY else INK)
            d.text((x0 + 22, 104), sub, font=F["tile_s"], fill=INK)
        return np.array(layer)[:, :, ::-1].copy()

    # ---------- render
    def run(self, out: Path) -> None:
        cam = cv2.VideoCapture(str(self.d / "camera.mp4")) if (self.d / "camera.mp4").exists() else None
        plots = cv2.VideoCapture(str(self.d / "plots.mp4")) if (self.d / "plots.mp4").exists() else None
        vw = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))
        ci, pi, si = -1, -1, -1
        cam_img = plot_img = tile_img = None
        n = int((self.t1 - self.t0) * FPS)
        cx0, cy0 = self.cam_box[0] + 10, self.cam_box[1] + 10
        cw, chh = self.cam_box[2] - self.cam_box[0] - 20, self.cam_box[3] - self.cam_box[1] - 50
        fx0, fy0 = self.fig_box[0] + 10, self.fig_box[1] + 10
        fw, fh = self.fig_box[2] - self.fig_box[0] - 20, self.fig_box[3] - self.fig_box[1] - 50
        tstart = time.time()
        for k in range(n):
            t = self.t0 + k / FPS
            while cam is not None and ci + 1 < len(self.t_cam) and self.t_cam[ci + 1] <= t:
                ok, fr = cam.read()
                ci += 1
                if ok:
                    cam_img = fit(fr, cw, chh)
            while plots is not None and pi + 1 < len(self.t_plot) and self.t_plot[pi + 1] <= t:
                ok, fr = plots.read()
                pi += 1
                if ok:
                    plot_img = fit(fr, fw, fh)
            while si + 1 < len(self.status) and self.t_status[si + 1] <= t:
                si += 1
                tile_img = None
            if si < 0:
                si = 0
            if tile_img is None:
                tile_img = self.tiles(self.status[si])
            frame = self.timeline.copy()
            frame[self.tiles_y[0]:self.tiles_y[0] + tile_img.shape[0]] = tile_img
            for img, (x0, y0, ww, hh) in ((cam_img, (cx0, cy0, cw, chh)), (plot_img, (fx0, fy0, fw, fh))):
                if img is None:
                    if not (cam is None and x0 == cx0):                  # (no camera at all: one message below)
                        cv2.putText(frame, "waiting for the stream ...", (x0 + 20, y0 + hh // 2), cv2.FONT_HERSHEY_SIMPLEX,
                                    0.8, MUTED, 2)
                    continue
                ox, oy = x0 + (ww - img.shape[1]) // 2, y0 + (hh - img.shape[0]) // 2
                frame[oy:oy + img.shape[0], ox:ox + img.shape[1]] = img
            if cam is None:
                cv2.putText(frame, "camera not recorded (main.py not running?)", (cx0 + 20, cy0 + chh // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (60, 60, 200), 2)
            # cursor, elapsed time, REC
            x = int(self._x(t))
            cv2.line(frame, (x, self.tl_box[1] + 36), (x, self.tl_box[1] + 192), (40, 40, 40), 2)
            pil = Image.fromarray(frame[0:84, W - 470:W, ::-1].copy())
            d = ImageDraw.Draw(pil)
            if int(t * 2) % 2 == 0:
                d.ellipse((10, 14, 30, 34), fill=RED)
            d.text((40, 6), "REC   " + clock(t - self.t0), font=F["clock"], fill=(255, 255, 255))
            frame[0:84, W - 470:W] = np.array(pil)[:, :, ::-1]
            vw.write(frame)
            if k % 150 == 0:
                print("  %5.1f %%  (%s of %s)" % (100.0 * k / max(n, 1), clock(t - self.t0), clock(self.t1 - self.t0)), flush=True)
        vw.release()
        if cam is not None:
            cam.release()
        if plots is not None:
            plots.release()
        print("done in %.0f s: %s" % (time.time() - tstart, out))


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    d = Path(sys.argv[1])
    print("experiment video for", d)
    for _ in range(40):                                             # main.py closes the camera file within ~0.5 s
        if (d / "camera_done").exists() or not (d / "camera.mp4").exists():
            break
        time.sleep(0.5)
    if not (d / "status.csv").exists():
        print("missing status.csv in", d)
        return 1
    Composer(d).run(d / "experiment_video.mp4")
    return 0


if __name__ == "__main__":
    sys.exit(main())
