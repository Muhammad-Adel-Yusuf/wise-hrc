"""Small blocking window that asks for ONE Borg CR10 rating (0-10) - the starting fatigue of the physical-fatigue algorithm.
Click a number (or press 0-9, 1 then 0 is not needed: click 10), then Enter / OK. Esc cancels (returns None). Nothing is pre-selected."""
from __future__ import annotations

from typing import Optional, Tuple

import cv2
import numpy as np

W, H = 760, 300
BG, PANEL, TEXT, MUTED, ACCENT, GREEN, LINE = (30, 27, 24), (46, 42, 38), (238, 236, 232), (158, 152, 146), (255, 168, 82), (112, 196, 92), (90, 84, 78)
ANCHORS = {0: "nothing at all", 1: "very light", 2: "light", 3: "moderate", 5: "strong", 7: "very strong", 10: "extremely strong"}


def cell_rect(v: int) -> Tuple[int, int, int, int]:
    w = (W - 60) / 11
    x0 = int(30 + v * w)
    return x0, 110, int(x0 + w - 4), 164


def ok_rect() -> Tuple[int, int, int, int]:
    return W - 190, H - 62, W - 30, H - 18


def cell_at(x: int, y: int) -> Optional[int]:
    for v in range(11):
        x0, y0, x1, y1 = cell_rect(v)
        if x0 <= x <= x1 and y0 <= y <= y1:
            return v
    return None


def render(selected: Optional[int], title: str = "Starting fatigue (Borg CR10)") -> np.ndarray:
    F = cv2.FONT_HERSHEY_SIMPLEX
    img = np.full((H, W, 3), BG, np.uint8)
    cv2.putText(img, title, (30, 40), F, 0.8, TEXT, 2, cv2.LINE_AA)
    cv2.putText(img, "How strong is your PHYSICAL exertion / bodily fatigue right now?", (30, 70), F, 0.5, MUTED, 1, cv2.LINE_AA)
    cv2.putText(img, "Arms, shoulders, trunk - not mental effort. Click one number.", (30, 90), F, 0.45, MUTED, 1, cv2.LINE_AA)
    for v in range(11):
        x0, y0, x1, y1 = cell_rect(v)
        cv2.rectangle(img, (x0, y0), (x1, y1), ACCENT if selected == v else PANEL, -1)
        cv2.rectangle(img, (x0, y0), (x1, y1), LINE, 1)
        lab = str(v)
        tw = cv2.getTextSize(lab, F, 0.6, 1)[0][0]
        cv2.putText(img, lab, ((x0 + x1) // 2 - tw // 2, 146), F, 0.6, (24, 22, 20) if selected == v else TEXT, 1, cv2.LINE_AA)
        if v in ANCHORS:
            t = ANCHORS[v]
            s = 0.3
            tw = cv2.getTextSize(t, F, s, 1)[0][0]
            cv2.putText(img, t, (max(4, min(W - tw - 4, (x0 + x1) // 2 - tw // 2)), 184), F, s, MUTED, 1, cv2.LINE_AA)
    x0, y0, x1, y1 = ok_rect()
    cv2.rectangle(img, (x0, y0), (x1, y1), GREEN if selected is not None else (60, 56, 52), -1)
    lab = "OK" if selected is not None else "choose a number"
    tw = cv2.getTextSize(lab, F, 0.55, 1)[0][0]
    cv2.putText(img, lab, ((x0 + x1) // 2 - tw // 2, (y0 + y1) // 2 + 6), F, 0.55, (24, 22, 20) if selected is not None else MUTED, 1, cv2.LINE_AA)
    cv2.putText(img, "Esc = cancel", (30, H - 30), F, 0.45, MUTED, 1, cv2.LINE_AA)
    return img


def ask_borg(title: str = "Starting fatigue (Borg CR10)", window: str = "Borg CR10") -> Optional[int]:
    """Blocks until OK or Esc. Returns 0..10 or None."""
    state = {"sel": None, "done": False}

    def on_mouse(event, x, y, flags, param) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        v = cell_at(x, y)
        if v is not None:
            state["sel"] = v
        else:
            x0, y0, x1, y1 = ok_rect()
            if x0 <= x <= x1 and y0 <= y <= y1 and state["sel"] is not None:
                state["done"] = True

    cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window, on_mouse)
    result: Optional[int] = None
    try:
        while True:
            cv2.imshow(window, render(state["sel"], title))
            k = cv2.waitKey(30) & 0xFF
            if k == 27:
                break
            if ord("0") <= k <= ord("9"):
                state["sel"] = k - ord("0")
            elif k in (13, 10) and state["sel"] is not None:
                state["done"] = True
            if state["done"]:
                result = state["sel"]
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                cv2.namedWindow(window, cv2.WINDOW_AUTOSIZE)              # a closed window is re-created, the rating is still required
                cv2.setMouseCallback(window, on_mouse)
    finally:
        try:
            cv2.destroyWindow(window)
        except cv2.error:
            pass
        cv2.waitKey(1)
    return result
