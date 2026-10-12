"""Live camera preview to place the camera (Orbbec, same settings as main.py).

    python camera_preview.py

Shows the colour image with
  * red pixels painted green (the red ring of the robot tool must show up as a solid green spot),
  * a centre cross with its depth,
  * the largest red spots with their size and distance.
Keys: R red overlay on/off, D depth view on/off, G grid on/off, Q / Esc quit.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import cv2
import numpy as np

from fatigue_monitor.config import load_config
from fatigue_monitor.orbbec_camera import OrbbecRGBD
from fatigue_monitor.robot_frame import put_text, red_mask

HERE = Path(__file__).resolve().parent
WIN = "camera preview (Q quit, R red, D depth, G grid)"
DISPLAY_SCALE = 0.75


def text(img, s, xy, col=(255, 255, 255), scale=0.6):
    put_text(img, s, xy, col, scale)


def main() -> int:
    os.chdir(HERE)
    cfg = load_config(str(HERE / "config.json"))
    cam = OrbbecRGBD(cfg["camera"])
    cam.start()
    show_red, show_depth, show_grid = True, False, True
    try:
        while True:
            f = cam.read()
            if f is None:
                continue
            img = f.color_bgr.copy()
            h, w = img.shape[:2]
            d = f.depth_mm if f.depth_aligned else None
            if show_depth and d is not None:
                dn = np.clip((d - 300) / (2500 - 300) * 255, 0, 255).astype(np.uint8)
                img = cv2.applyColorMap(255 - dn, cv2.COLORMAP_JET)
                img[d <= 0] = 0
            if show_grid:
                for i in (1, 2):
                    cv2.line(img, (w * i // 3, 0), (w * i // 3, h), (200, 200, 200), 1)
                    cv2.line(img, (0, h * i // 3), (w, h * i // 3), (200, 200, 200), 1)
            cv2.drawMarker(img, (w // 2, h // 2), (0, 255, 255), cv2.MARKER_CROSS, 30, 2)
            lines = []
            if d is not None:
                c = d[h // 2 - 4:h // 2 + 5, w // 2 - 4:w // 2 + 5]
                c = c[c > 0]
                lines.append("centre depth: %s" % ("%.0f mm" % np.median(c) if c.size else "no depth"))
            if show_red:
                m = red_mask(f.color_bgr)
                img[m > 0] = (0, 255, 0)
                n, _, st, ce = cv2.connectedComponentsWithStats(m)
                blobs = sorted(((st[i, cv2.CC_STAT_AREA], ce[i]) for i in range(1, n) if st[i, cv2.CC_STAT_AREA] >= 40),
                               key=lambda b: -b[0])[:5]
                for k, (area, c) in enumerate(blobs):
                    u, v = int(c[0]), int(c[1])
                    z = d[v, u] if d is not None else 0
                    cv2.circle(img, (u, v), 14, (0, 0, 255), 2)
                    text(img, "%d: %d px%s" % (k + 1, area, ", %.0f mm" % z if z > 0 else ""), (u + 16, v), (0, 255, 255), 0.7)
            img = cv2.resize(img, None, fx=DISPLAY_SCALE, fy=DISPLAY_SCALE)
            text(img, "Q quit  R red overlay  D depth view  G grid", (10, 24))
            for i, s in enumerate(lines):
                text(img, s, (10, 50 + 24 * i))
            cv2.imshow(WIN, img)
            k = cv2.waitKey(1) & 0xFF
            if k in (27, ord("q")):
                break
            if k == ord("r"):
                show_red = not show_red
            elif k == ord("d"):
                show_depth = not show_depth
            elif k == ord("g"):
                show_grid = not show_grid
            if cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
