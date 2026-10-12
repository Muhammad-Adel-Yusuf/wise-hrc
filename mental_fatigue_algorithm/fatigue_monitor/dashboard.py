"""Dashboard window: one fixed-size canvas (1280 x 760) with the camera view, status, the three algorithm cards, the
instruction tracker and clickable buttons. Pure drawing (OpenCV); no logic of the fatigue algorithms lives here.

Every button has a keyboard key; a mouse click on a button is turned into that key by the main program, so keys and
buttons always do exactly the same thing.
"""
from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from . import geometry as geo

PANEL_IMAGE = True                      # ui["panel_image"] (e.g. a slide) replaces the camera picture
CANVAS_W, CANVAS_H = 1280, 760
PHYS_H = 150                            # extra strip below the dashboard for the physical-fatigue algorithm (Algorithm 4)

# colours (BGR)
BG = (30, 27, 24)
PANEL = (46, 42, 38)
PANEL_HI = (58, 53, 48)
LINE = (78, 72, 66)
TEXT = (238, 236, 232)
MUTED = (158, 152, 146)
DIM = (104, 99, 94)
ACCENT = (255, 168, 82)
GREEN = (112, 196, 92)
AMBER = (48, 184, 250)
RED = (84, 84, 236)
LEVEL_COLOR = {"Low": GREEN, "Medium": AMBER, "High": RED, "Unknown": DIM}
LEVEL_NAME = ("Low", "Medium", "High")
FONT = cv2.FONT_HERSHEY_SIMPLEX

Button = Tuple[int, int, int, int, int]          # x0, y0, x1, y1, key code


# ------------------------------------------------------------------ drawing primitives
def _text(img, s, x, y, scale=0.5, color=TEXT, thick=1):
    cv2.putText(img, s, (int(x), int(y)), FONT, scale, color, thick, cv2.LINE_AA)


def _text_w(s, scale=0.5, thick=1) -> int:
    return cv2.getTextSize(s, FONT, scale, thick)[0][0]


def _text_c(img, s, cx, cy, scale=0.5, color=TEXT, thick=1):
    w, h = cv2.getTextSize(s, FONT, scale, thick)[0]
    _text(img, s, cx - w / 2, cy + h / 2, scale, color, thick)


def _rrect(img, x0, y0, x1, y1, color, r=10, filled=True, thick=1):
    x0, y0, x1, y1 = int(x0), int(y0), int(x1), int(y1)
    r = max(0, min(r, (x1 - x0) // 2, (y1 - y0) // 2))
    if filled:
        cv2.rectangle(img, (x0 + r, y0), (x1 - r, y1), color, -1)
        cv2.rectangle(img, (x0, y0 + r), (x1, y1 - r), color, -1)
        for cx, cy in ((x0 + r, y0 + r), (x1 - r, y0 + r), (x0 + r, y1 - r), (x1 - r, y1 - r)):
            cv2.circle(img, (cx, cy), r, color, -1, cv2.LINE_AA)
    else:
        cv2.rectangle(img, (x0 + r, y0), (x1 - r, y1), color, thick, cv2.LINE_AA)
        cv2.rectangle(img, (x0, y0 + r), (x1, y1 - r), color, thick, cv2.LINE_AA)


def _segments(img, x, y, w, frac: Optional[float], h=10, n=10, color=None, off=False):
    """10-segment meter; segment i is lit when frac >= i/10. Colour by position (green / amber / red) unless given."""
    gap = 3
    sw = (w - gap * (n - 1)) / n
    lit = 0 if frac is None else int(min(n, max(0, math.ceil(max(0.0, min(1.0, frac)) * n - 1e-9))))
    for i in range(n):
        x0 = x + i * (sw + gap)
        c = color or (GREEN if i < 4 else (AMBER if i < 7 else RED))
        if off:
            c = DIM
        _rrect(img, x0, y, x0 + sw, y + h, c if (i < lit and not off) else LINE, r=3)


def _chip(img, x, y, label, bg, fg=(24, 22, 20), scale=0.45, pad=10, h=24) -> int:
    w = _text_w(label, scale, 1) + 2 * pad
    _rrect(img, x, y, x + w, y + h, bg, r=h // 2)
    _text_c(img, label, x + w / 2, y + h / 2, scale, fg, 1)
    return int(w)


def _button(img, ui, rect, label, key: int, hint: str, active: bool = False, color=None, enabled: bool = True,
            scale=0.42) -> None:
    x0, y0, x1, y1 = rect
    bg = color or (PANEL_HI if not active else ACCENT)
    fg = TEXT if not active else (24, 22, 20)
    if color is not None:
        fg = (24, 22, 20)
    if not enabled:
        bg, fg = PANEL, DIM
    _rrect(img, x0, y0, x1, y1, LINE, r=8)                     # 1 px outline = slightly larger filled shape behind
    _rrect(img, x0 + 1, y0 + 1, x1 - 1, y1 - 1, bg, r=7)
    if hint:                                                   # hot key, small, top-left corner
        _text(img, hint, x0 + 7, y0 + 13, 0.32, MUTED if not active and color is None else fg, 1)
    _text_c(img, label, (x0 + x1) / 2, y1 - 12, scale, fg, 1)
    if enabled:
        ui["buttons"].append((int(x0), int(y0), int(x1), int(y1), int(key)))


def button_at(ui: Dict[str, Any], x: int, y: int) -> Optional[int]:
    """Key code of the button under (x, y) (canvas coordinates), else None."""
    for x0, y0, x1, y1, key in ui.get("buttons", []):
        if x0 <= x <= x1 and y0 <= y <= y1:
            return key
    return None


def _fit(image: np.ndarray, w: int, h: int) -> Tuple[np.ndarray, int, int]:
    """Scale to fit into (w, h) keeping the aspect ratio; returns (image, x offset, y offset) inside the box."""
    ih, iw = image.shape[:2]
    s = min(w / iw, h / ih)
    nw, nh = max(1, int(iw * s)), max(1, int(ih * s))
    return cv2.resize(image, (nw, nh), interpolation=cv2.INTER_AREA), (w - nw) // 2, (h - nh) // 2


# ------------------------------------------------------------------ panels
def _status(st, ui) -> Tuple[str, Tuple[int, int, int]]:
    warn = " ".join(st.data_warnings)
    if st.standby:
        return "STANDBY  collaboration inactive", AMBER
    if "waiting for you" in warn or "waiting for the instruction" in warn:
        return "PRESS  S  TO START BASELINE", ACCENT
    if "learning personal baseline" in warn:
        return f"LEARNING BASELINE {st.baseline_progress * 100:.0f}%", ACCENT
    if "workstation calibration" in warn:
        return "CALIBRATE  1 / 2 / 3", ACCENT
    if not st.data_valid:
        return "CHECKING DATA QUALITY", AMBER
    return "RUNNING", GREEN


def _header(img, st, head, mode, ui):
    m_code, m_score, m_label = head
    cv2.rectangle(img, (0, 0), (CANVAS_W, 80), PANEL, -1)
    cv2.line(img, (0, 80), (CANVAS_W, 80), LINE, 1)
    _text(img, "Mental Fatigue Monitor", 24, 38, 0.85, TEXT, 2)
    sub = {"all": "mental load: 3 algorithms + combiner", "task": "Algorithm 1 only - task", "visual": "Algorithm 2 only - visual",
           "attention": "Algorithm 3 only - attention"}[mode]
    _text(img, sub + "  |  vision-based risk estimate, not a medical diagnosis", 24, 62, 0.4, MUTED, 1)
    txt, col = _status(st, ui)
    _chip(img, 640, 26, txt, col, scale=0.5, h=28)
    # level badge
    col = LEVEL_COLOR.get(m_label, DIM)
    _rrect(img, 960, 10, 1256, 70, col, r=14)
    title = "FINAL LEVEL" if mode == "all" else "LEVEL"
    _text(img, title, 978, 30, 0.4, (24, 22, 20), 1)
    _text(img, m_label.upper(), 978, 58, 0.95, (24, 22, 20), 2)
    _text_c(img, f"{m_code}", 1226, 40, 1.3, (24, 22, 20), 2)


def _camera(img, color, st, face_px, hands_px, body_px, inf_size, depth_img, fps, intr_desc, ui):
    X0, Y0, BW, BH = 24, 96, 840, 472
    _rrect(img, X0 - 2, Y0 - 2, X0 + BW + 2, Y0 + BH + 2, LINE, r=10)
    panel = ui.get("panel_image")                   # e.g. the PowerPoint slide: shown instead of the camera view
    if panel is not None:
        small, ox, oy = _fit(panel, BW, BH)
    else:                                           # resize first, then draw the landmarks on the small image (no full-size copy)
        small, ox, oy = _fit(color, BW, BH)
        sx, sy = small.shape[1] / inf_size[0], small.shape[0] / inf_size[1]
        if face_px is not None:
            for idx in geo.RIGHT_EYE + geo.LEFT_EYE + (geo.NOSE_TIP, geo.MOUTH_LEFT, geo.MOUTH_RIGHT):
                cv2.circle(small, (int(face_px[idx, 0] * sx), int(face_px[idx, 1] * sy)), 2, (0, 255, 0), -1)
        for hp in hands_px:
            for p in hp:
                cv2.circle(small, (int(p[0] * sx), int(p[1] * sy)), 2, (255, 200, 0), -1)
        if body_px is not None and len(body_px) > 12:
            for i in geo.POSE_DRAW:
                if body_px[i, 2] >= 0.5:
                    cv2.circle(small, (int(body_px[i, 0] * sx), int(body_px[i, 1] * sy)), 4, (255, 0, 255), -1)
    img[Y0 + oy:Y0 + oy + small.shape[0], X0 + ox:X0 + ox + small.shape[1]] = small
    # overlay chips
    att = st.attention if not st.standby else "Standby"
    chip_col = {"Outside working area": RED, "Unknown": DIM}.get(att, GREEN)
    x = X0 + 12
    x += _chip(img, x, Y0 + 12, f"Looking at: {att}", chip_col, scale=0.45) + 8
    if st.task_engaged:
        x += _chip(img, x, Y0 + 12, "Task engaged (face hidden)", AMBER, scale=0.42) + 8
    if panel is None:
        _chip(img, X0 + 12, Y0 + 44, f"yaw {_f(st.yaw, '{:.0f}')}  pitch {_f(st.pitch, '{:.0f}')}", PANEL, TEXT, 0.38, h=20)
    _text(img, f"{fps:4.1f} FPS  {intr_desc}"[:70], X0 + 12, Y0 + BH - 10, 0.36, (220, 220, 220), 1)
    if st.standby:
        _rrect(img, X0 + 180, Y0 + BH // 2 - 30, X0 + BW - 180, Y0 + BH // 2 + 30, (20, 18, 16), r=12)
        _text_c(img, "STANDBY - collaboration inactive", X0 + BW // 2, Y0 + BH // 2, 0.8, AMBER, 2)
    if depth_img is not None:
        dh = 120
        dw = int(depth_img.shape[1] * dh / depth_img.shape[0])
        d = cv2.resize(depth_img, (dw, dh))
        img[Y0 + BH - dh - 28:Y0 + BH - 28, X0 + BW - dw - 12:X0 + BW - 12] = d


def _f(v, spec="{:.2f}"):
    return "--" if v is None else spec.format(v)


def _controls(img, st, ui, mode):
    """Calibration and session buttons + the message line under the camera."""
    X0, Y0 = 24, 580
    _rrect(img, X0, Y0, 864, 750, PANEL, r=12)
    bw, bh, g = 101, 36, 6
    waiting = "waiting for" in " ".join(st.data_warnings)
    _text(img, "CALIBRATION", X0 + 14, Y0 + 22, 0.4, MUTED, 1)
    row = [("Start baseline", ord("s"), "S", waiting), ("Relearn baseline", ord("b"), "B", False),
           ("Assembly", ord("1"), "1", False), ("Laptop", ord("2"), "2", False), ("Storage", ord("3"), "3", False),
           ("Work posture", ord("4"), "4", False), ("Reset calib.", ord("r"), "R", False)]
    x = X0 + 14
    for label, key, hint, active in row:
        _button(img, ui, (x, Y0 + 30, x + bw, Y0 + 30 + bh), label, key, hint, active=active, scale=0.36)
        x += bw + g
    _text(img, "SESSION", X0 + 14, Y0 + 88, 0.4, MUTED, 1)
    row2 = [("Reset fatigue", ord("f"), "F", False, None),
            ("Collab: " + ("ON" if st.collab_active else "OFF"), ord("k"), "K", False, GREEN if st.collab_active else AMBER),
            ("Load slides", ord("u"), "U", False, None),
            (("Slide / camera", ord("v"), "V", bool(ui.get("slide_view")), None) if "slide_view" in ui
             else ("Depth view", ord("d"), "D", ui.get("show_depth", False), None))]
    x = X0 + 14
    for label, key, hint, active, col in row2:
        _button(img, ui, (x, Y0 + 96, x + bw + 14, Y0 + 96 + bh), label, key, hint, active=active, color=col, scale=0.4)
        x += bw + 14 + g
    # message line
    msg, col = "", MUTED
    if st.calibration_status:
        msg, col = st.calibration_status, (0, 230, 255)
    elif st.notices:
        msg, col = st.notices[0], AMBER
    elif st.data_warnings:
        msg = st.data_warnings[0]
        col = AMBER
    elif st.presence:
        msg = f"presence: {st.presence}"
    _text(img, msg[:95], X0 + 14, Y0 + 158, 0.42, col, 1)


def _instruction_card(img, ui, st):
    x0, y0, x1, y1 = 884, 96, 1256, 208
    _rrect(img, x0, y0, x1, y1, PANEL, r=12)
    _text(img, "INSTRUCTION", x0 + 16, y0 + 24, 0.4, MUTED, 1)
    val = ui.get("instruction")
    total = ui.get("slides_total")
    num = "--" if val is None else str(val)
    _text(img, num, x0 + 16, y0 + 84, 2.2, TEXT, 4)
    w = _text_w(num, 2.2, 4)
    if total:
        _text(img, f"/ {total}", x0 + 28 + w, y0 + 84, 0.8, MUTED, 2)
    else:
        _text(img, "no slides (press U)", x0 + 28 + w, y0 + 84, 0.42, MUTED, 1)
    bx = x0 + 200
    _button(img, ui, (bx, y0 + 14, bx + 78, y0 + 50), "Prev", ord("p"), "P", scale=0.42)
    _button(img, ui, (bx + 86, y0 + 14, bx + 164, y0 + 50), "Next", ord("n"), "N", scale=0.42, color=ACCENT)
    _button(img, ui, (bx, y0 + 60, bx + 78, y0 + 96), "Tried", ord("a"), "A", scale=0.42)
    _button(img, ui, (bx + 86, y0 + 60, bx + 164, y0 + 96), "Done", ord("c"), "C", scale=0.42)


def _algo_card(img, ui, y, name, tag, key, st, enabled, toggle):
    """One algorithm card. key: 'task' | 'visual' | 'attention'."""
    x0, x1, h = 884, 1256, 112
    _rrect(img, x0, y, x1, y + h, PANEL if enabled else (38, 35, 32), r=12)
    out = {"task": st.task, "visual": st.visual, "attention": st.attention_out}[key]
    ks, kl = {"task": ("task_score", "task_fatigue_level"), "visual": ("visual_score", "visual_fatigue_level"),
              "attention": ("attention_score", "attention_fatigue_level")}[key]
    lv, sc = int(out[kl]), float(out[ks])
    tcol = TEXT if enabled else DIM
    _text(img, f"{tag}  {name}", x0 + 16, y + 26, 0.6, tcol, 1)
    if toggle:
        _button(img, ui, (x1 - 84, y + 8, x1 - 12, y + 36), "ON" if enabled else "OFF", ord({"task": "7", "visual": "8", "attention": "9"}[key]),
                {"task": "7", "visual": "8", "attention": "9"}[key], color=GREEN if enabled else None, scale=0.45)
    if not enabled:
        _text(img, "OFF - not used in the final level", x0 + 16, y + 62, 0.42, DIM, 1)
        if key == "attention":
            _text(img, "SAFETY: look-away rule (2 s / 3 s) inactive", x0 + 16, y + 88, 0.42, RED, 1)
        return
    name_lv = LEVEL_NAME[lv]
    _chip(img, x0 + 16, y + 38, name_lv.upper(), LEVEL_COLOR[name_lv], scale=0.42, h=22)
    _text(img, f"score {sc:.2f}", x0 + 100, y + 54, 0.45, TEXT, 1)
    now = st.instant.get(key)
    if now is not None:
        _text(img, f"now {now:.2f}", x0 + 190, y + 54, 0.4, MUTED, 1)
    _segments(img, x0 + 16, y + 68, 340, sc, h=9)
    if key == "task":
        t = st.task
        line = f"review {t['review_count']}   errors {t['suspected_error_count']}   hesitation {t['hesitation_count']}"
    elif key == "visual":
        v = st.visual
        line = (f"PERCLOS {_r10(st.r.get('perclos'))}  closure {_r10(st.r.get('closure'))}  "
                f"yawns {v['yawn_count']}  nods {v['nod_count']}  rubs {v['eye_rub_count']}")
    else:
        a = st.attention_out
        line = (f"{_STATE_TXT.get(a['attention_state'], a['attention_state'].lower())}  out {st.d_outside_s:.1f}s  "
                f"brief {a['brief_disengagement_count']}  sust. {a['sustained_disengagement_count']}")
    _text(img, line[:60], x0 + 16, y + 98, 0.36, MUTED, 1)


_STATE_TXT = {"SUSTAINED_DISENGAGEMENT": "look-away", "OUTSIDE_BRIEF": "glancing away", "FOCUSED": "focused",
              "TASK_ENGAGED": "task engaged", "ATTENTION_UNVERIFIED": "unverified", "UNKNOWN": "unknown",
              "STANDBY": "standby", "OFF": "off"}


def _r10(r):
    return "--" if r is None else f"{int(math.ceil(max(0.0, min(1.0, r)) * 10 - 1e-9))}/10"


def _combined_card(img, st, mode, ui):
    x0, y0, x1, y1 = 884, 588, 1256, 750
    _rrect(img, x0, y0, x1, y1, PANEL, r=12)
    if mode == "all":
        _text(img, "COMBINED", x0 + 16, y0 + 24, 0.4, MUTED, 1)
        _text(img, f"score {st.combined_score:.2f}", x0 + 16, y0 + 50, 0.6, TEXT, 1)
        _text(img, f"data quality {st.quality:.2f}   baseline {'done' if st.baseline_done else f'{st.baseline_progress * 100:.0f}%'}",
              x0 + 150, y0 + 50, 0.4, MUTED, 1)
        _segments(img, x0 + 16, y0 + 62, 340, st.combined_score, h=9)
        y = y0 + 92
        for ev in (st.active_evidence or ["no active evidence"])[:4]:
            _text(img, "- " + ev[:52], x0 + 16, y, 0.4, MUTED if ev.startswith("no ") else TEXT, 1)
            y += 18
    else:
        _text(img, "INDICATORS  (raw | score)", x0 + 16, y0 + 24, 0.4, MUTED, 1)
        from .config import INDICATORS
        names = {"visual": ("perclos", "closure", "yawn", "nod", "rub"),
                 "attention": ("brief_disengagement", "sustained_disengagement"), "task": ()}[mode]
        y = y0 + 48
        for k in names:
            _text(img, f"{k.replace('_', ' ')}", x0 + 16, y, 0.42, TEXT, 1)
            _text(img, f"{_f(st.raw.get(k))}  |  {_r10(st.r.get(k))}", x0 + 210, y, 0.42, MUTED, 1)
            y += 20
        if mode == "task":
            td = st.task_diag
            _text(img, f"normal duration {td.get('normal_duration_s', 0):.0f}s "
                       f"({'calibrated' if td.get('calibrated') else 'default'})", x0 + 16, y0 + 48, 0.42, TEXT, 1)


# ------------------------------------------------------------------ entry point
def render(color: np.ndarray, st, head: Tuple[int, float, str], mode: str, ui: Dict[str, Any], *,
           face_px=None, hands_px: Sequence = (), body_px=None, inf_size: Tuple[int, int] = (640, 360),
           depth_img: Optional[np.ndarray] = None, fps: float = 0.0, intr_desc: str = "") -> np.ndarray:
    """Draw the whole dashboard. ui: {"instruction": int|None, "slides_total": int|None, "show_depth": bool,
    "buttons": [...] (cleared and refilled here)}. Returns the canvas (CANVAS_H x CANVAS_W x 3)."""
    ui["buttons"] = []
    phys = ui.get("physical")                       # state dict of the physical algorithm: adds a separate strip below
    img = np.full((CANVAS_H + (PHYS_H if phys is not None else 0), CANVAS_W, 3), BG, np.uint8)
    _header(img, st, head, mode, ui)
    _camera(img, color, st, face_px, hands_px, body_px, inf_size, depth_img, fps, intr_desc, ui)
    _controls(img, st, ui, mode)
    if mode in ("all", "task"):
        _instruction_card(img, ui, st)
    if mode == "all":
        y = 218
        for key, name, tag in (("task", "Task", "1"), ("visual", "Visual", "2"), ("attention", "Attention", "3")):
            _algo_card(img, ui, y, name, tag, key, st, st.enabled.get(key, True), toggle=True)
            y += 122
    else:
        tag = {"task": "1", "visual": "2", "attention": "3"}[mode]
        y = 218 if mode == "task" else 96
        _algo_card(img, ui, y, mode.capitalize(), tag, mode, st, True, toggle=False)
    _combined_card(img, st, mode, ui)
    if phys is not None:
        _physical_strip(img, ui, phys, 768)
    return img


def canvas_height(with_physical: bool) -> int:
    return CANVAS_H + (PHYS_H if with_physical else 0)


# ------------------------------------------------------------------ physical fatigue (Algorithm 4) - always a SEPARATE panel
BAND_COLOR = {"Low": GREEN, "Medium": AMBER, "High": RED}
PHYS_NOTE = "operational 0-1 state, not a physiological measurement"


def _phys_plot(img, x0, y0, x1, y1, hist, bands, span_min):
    _rrect(img, x0, y0, x1, y1, (38, 35, 32), r=8)
    for v, col in ((bands[0], GREEN), (bands[1], AMBER)):
        yy = int(y1 - 6 - v * (y1 - y0 - 12))
        cv2.line(img, (x0 + 4, yy), (x1 - 4, yy), col if False else LINE, 1, cv2.LINE_AA)
        _text(img, f"{v:.1f}", x0 + 6, yy - 3, 0.3, DIM, 1)
    if len(hist) >= 2:
        t_end = hist[-1][0]
        t_beg = min(hist[0][0], t_end - 1e-6)
        span = max(span_min, t_end - t_beg) if t_end - t_beg < span_min else (t_end - t_beg)
        pts = []
        for tm, f in hist:
            px = x1 - 6 - (t_end - tm) / span * (x1 - x0 - 40)
            py = y1 - 6 - max(0.0, min(1.0, f)) * (y1 - y0 - 12)
            pts.append((int(px), int(py)))
        cv2.polylines(img, [np.array(pts, np.int32)], False, ACCENT, 2, cv2.LINE_AA)
        _text(img, f"last {min(span, t_end - t_beg):.0f} min", x0 + 40, y1 - 8, 0.3, DIM, 1)


def _phys_gauge(img, ui, ph, x0, y0, w):
    """Value, band, bar, mode and workload class of the physical state (shared by the strip and the stand-alone view)."""
    f = float(ph.get("f", 0.0))
    band = ph.get("band_name", "Low")
    off = not ph.get("enabled", True)
    _text(img, "--" if off else f"{f:.2f}", x0, y0 + 40, 1.5, DIM if off else TEXT, 3)
    _text(img, "f (0-1)", x0 + 2, y0 + 58, 0.38, MUTED, 1)
    _text(img, "OFF" if off else f"Borg CR10 equiv. {10 * f:.1f}", x0 + 130, y0 + 18, 0.45, MUTED, 1)
    if not off:
        _chip(img, x0 + 130, y0 + 28, band.upper(), BAND_COLOR[band], scale=0.45, h=24)
        mode = ph.get("mode", "recovery")
        seen = ph.get("seen", True)
        label, col = (("NO PERSON SEEN", DIM) if not seen else (("WORK", AMBER) if mode == "work" else ("RECOVERY", GREEN)))
        if mode == "work" and seen and ph.get("intensity"):
            label += f"  class {ph['intensity']}"
        _chip(img, x0 + 130 + 90, y0 + 28, label, col, scale=0.42, h=24)
    _segments(img, x0, y0 + 70, w, None if off else f, h=9, off=off)


def _phys_features(ph) -> str:
    ft = ph.get("features") or {}
    g = lambda k, spec: "--" if ft.get(k) is None else spec.format(ft[k])
    return (f"arm {g('arm_elev_p90_deg', '{:.0f}')} deg  trunk {g('trunk_incl_p90_deg', '{:.0f}')} deg  "
            f"reps {g('repetition_rate_per_min', '{:.0f}')}/min  static {g('static_posture_s', '{:.0f}')} s  "
            f"valid {100 * ph.get('valid_fraction', 0.0):.0f}%")


def _phys_capture_banner(img, ph, x0, y0, x1, y1):
    """While the neutral posture is captured (key Y): a clear instruction and a progress bar over the plot area."""
    p = ph.get("capture_progress")
    if p is None:
        msg = ph.get("message", "")
        if msg:
            _text(img, msg[:78], x0 + 8, y1 - 6 - 0, 0.34, AMBER, 1)
        return
    _rrect(img, x0, y0, x1, y1, (24, 22, 20), r=8)
    _text_c(img, "SIT UPRIGHT - ARMS RELAXED - HANDS STILL", (x0 + x1) // 2, y0 + 22, 0.55, ACCENT, 2)
    _rrect(img, x0 + 20, y1 - 26, x1 - 20, y1 - 14, LINE, r=5)
    _rrect(img, x0 + 20, y1 - 26, x0 + 20 + int((x1 - x0 - 40) * max(0.02, min(1.0, p))), y1 - 14, GREEN, r=5)


def _physical_strip(img, ui, ph, y0):
    x0, x1, y1 = 24, 1256, y0 + PHYS_H - 12
    _rrect(img, x0, y0, x1, y1, PANEL, r=12)
    _text(img, "4  PHYSICAL FATIGUE", x0 + 16, y0 + 26, 0.6, TEXT, 1)
    _text(img, "separate - not weighted into the mental load", x0 + 250, y0 + 26, 0.36, MUTED, 1)
    _phys_gauge(img, ui, ph, x0 + 16, y0 + 36, 340)
    _phys_plot(img, x0 + 400, y0 + 40, x0 + 880, y1 - 12, ph.get("history", []), ui.get("physical_bands", (0.3, 0.6)), 10.0)
    _text(img, _phys_features(ph)[:62], x0 + 570, y0 + 26, 0.34, MUTED, 1)
    pc = ph.get("params", "")
    _text(img, pc[:44], x0 + 900, y0 + 70, 0.38, AMBER if "UNCAL" in pc else GREEN, 1)
    _text(img, ph.get("baseline_text", "")[:48], x0 + 900, y0 + 88, 0.34, GREEN if ph.get("baseline_complete") else AMBER, 1)
    _text(img, PHYS_NOTE, x0 + 900, y0 + 106, 0.32, DIM, 1)
    off = not ph.get("enabled", True)
    _button(img, ui, (x1 - 280, y0 + 8, x1 - 196, y0 + 40), "Baseline", ord("y"), "Y", color=None if ph.get("baseline_complete") else ACCENT, scale=0.4)
    _button(img, ui, (x1 - 188, y0 + 8, x1 - 104, y0 + 40), "ON" if not off else "OFF", ord("0"), "0", color=GREEN if not off else None, scale=0.42)
    _button(img, ui, (x1 - 96, y0 + 8, x1 - 12, y0 + 40), "Reset", ord("z"), "Z", scale=0.42)
    _phys_capture_banner(img, ph, x0 + 400, y0 + 40, x0 + 880, y1 - 12)


def render_physical(color: np.ndarray, ph: Dict[str, Any], ui: Dict[str, Any], *, body_px=None,
                    inf_size: Tuple[int, int] = (640, 360), fps: float = 0.0, intr_desc: str = "") -> np.ndarray:
    """Stand-alone view of Algorithm 4 (main_physical.py): camera with the skeleton, the physical state, its history."""
    ui["buttons"] = []
    img = np.full((CANVAS_H, CANVAS_W, 3), BG, np.uint8)
    cv2.rectangle(img, (0, 0), (CANVAS_W, 80), PANEL, -1)
    cv2.line(img, (0, 80), (CANVAS_W, 80), LINE, 1)
    _text(img, "Physical Fatigue Monitor", 24, 38, 0.85, TEXT, 2)
    _text(img, "Algorithm 4 only - camera-assisted LFFR estimate  |  " + PHYS_NOTE, 24, 62, 0.4, MUTED, 1)
    X0, Y0, BW, BH = 24, 96, 840, 472
    _rrect(img, X0 - 2, Y0 - 2, X0 + BW + 2, Y0 + BH + 2, LINE, r=10)
    small, ox, oy = _fit(color, BW, BH)
    if body_px is not None and len(body_px) > 24:
        sx, sy = small.shape[1] / inf_size[0], small.shape[0] / inf_size[1]
        pt = lambda i: (int(body_px[i, 0] * sx), int(body_px[i, 1] * sy))
        for a, b in ((11, 12), (11, 13), (13, 15), (12, 14), (14, 16), (11, 23), (12, 24), (23, 24)):
            if body_px[a, 2] >= 0.5 and body_px[b, 2] >= 0.5:
                cv2.line(small, pt(a), pt(b), (255, 0, 255), 2, cv2.LINE_AA)
        for i in (11, 12, 13, 14, 15, 16, 23, 24):
            if body_px[i, 2] >= 0.5:
                cv2.circle(small, pt(i), 5, (255, 200, 0), -1)
    img[Y0 + oy:Y0 + oy + small.shape[0], X0 + ox:X0 + ox + small.shape[1]] = small
    _text(img, f"{fps:4.1f} FPS  {intr_desc}"[:70], X0 + 12, Y0 + BH - 10, 0.36, (220, 220, 220), 1)
    # right card
    x0, y0, x1, y1 = 884, 96, 1256, 750
    _rrect(img, x0, y0, x1, y1, PANEL, r=12)
    _text(img, "PHYSICAL FATIGUE", x0 + 16, y0 + 26, 0.6, TEXT, 1)
    _phys_gauge(img, ui, ph, x0 + 16, y0 + 40, 340)
    y = y0 + 150
    ft = ph.get("features") or {}
    rows = (("Upper-arm elevation (p90)", "arm_elev_p90_deg", "{:.0f} deg"), ("Trunk inclination (p90)", "trunk_incl_p90_deg", "{:.0f} deg"),
            ("Wrist speed", "wrist_speed_mean_mps", "{:.2f} m/s"), ("Elbow speed", "elbow_speed_mean_dps", "{:.0f} deg/s"),
            ("Repetition rate", "repetition_rate_per_min", "{:.0f} /min"), ("Static posture", "static_posture_s", "{:.0f} s"),
            ("Reach (p90)", "reach_norm_p90", "{:.2f}"))
    _text(img, "CAMERA FEATURES (last window)", x0 + 16, y, 0.4, MUTED, 1)
    y += 24
    for label, k, spec in rows:
        _text(img, label, x0 + 16, y, 0.42, TEXT, 1)
        _text(img, "--" if ft.get(k) is None else spec.format(ft[k]), x0 + 230, y, 0.42, MUTED, 1)
        y += 22
    sc = ph.get("workload_score")
    _text(img, f"workload score {'--' if sc is None else f'{sc:.2f}'}   valid frames {100 * ph.get('valid_fraction', 0.0):.0f}%", x0 + 16, y + 8, 0.4, MUTED, 1)
    pc = ph.get("params", "")
    _text(img, "parameters: " + pc, x0 + 16, y + 40, 0.4, AMBER if "UNCAL" in pc else GREEN, 1)
    _text(img, "Run calibrate.py on the pilot data and set", x0 + 16, y + 62, 0.36, DIM, 1)
    _text(img, "physical.params_file to use calibrated rates.", x0 + 16, y + 78, 0.36, DIM, 1)
    _text(img, ph.get("baseline_text", "")[:46], x0 + 16, y + 106, 0.38, GREEN if ph.get("baseline_complete") else AMBER, 1)
    off = not ph.get("enabled", True)
    _button(img, ui, (x0 + 16, y1 - 108, x0 + 190, y1 - 68), "Baseline (rating + posture)", ord("y"), "Y", color=None if ph.get("baseline_complete") else ACCENT, scale=0.36)
    _button(img, ui, (x0 + 16, y1 - 60, x0 + 120, y1 - 20), "ON" if not off else "OFF", ord("0"), "0", color=GREEN if not off else None, scale=0.45)
    _button(img, ui, (x0 + 130, y1 - 60, x0 + 260, y1 - 20), "Reset fatigue", ord("z"), "Z", scale=0.42)
    # history
    _rrect(img, 24, 580, 864, 750, PANEL, r=12)
    _text(img, "FATIGUE HISTORY (f, 0-1)", 38, 602, 0.4, MUTED, 1)
    _phys_plot(img, 38, 612, 850, 738, ph.get("history", []), ui.get("physical_bands", (0.3, 0.6)), 10.0)
    _phys_capture_banner(img, ph, 38, 612, 850, 738)
    return img
