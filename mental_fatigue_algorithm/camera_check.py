"""Camera link check: USB connection type + measured colour and depth frame rates of the Femto Bolt.

    python camera_check.py           one check (about 10 s)
    python camera_check.py --watch   checks again every few seconds: unplug / replug the camera into another port or
                                     with another cable and watch the result (Ctrl+C to end)

Why: the Femto Bolt sends RAW time-of-flight data and the depth image is computed on this PC. Over a USB 2 link the raw
data limits depth to ~6-7 fps whatever depth resolution is chosen (colour alone still reaches 30 fps). The live program
then runs at the colour rate with the newest depth, but 30 fps depth needs a USB 3 (SuperSpeed, 5 Gbit/s) link.
Uses the same camera settings as the live program (config.json "camera").
"""
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent

ADVICE_USB2 = """
  The camera is connected at USB 2 speed. Depth cannot exceed ~6-7 fps on this link.
  To get USB 3 (check again after each step with  python camera_check.py --watch):
    1. Plug the camera DIRECTLY into a USB 3 port of the laptop (SS logo / blue inside), no hub, no extension.
       On this Lenovo IdeaPad (Y50-70) the USB 3 ports are normally the ones on the LEFT side - try each of them.
    2. Use the original Orbbec USB-C cable (or a cable marked USB 3.x / SuperSpeed, 5 Gbit/s or faster).
       Phone / charging USB-C cables are usually USB 2 only - that alone gives exactly this result.
    3. Turn the USB-C plug at the camera end upside down and plug it in again (a known fix for a USB-C link that
       falls back to USB 2).
    4. Connect the camera's own power supply as well, if it has one plugged into a separate port.
"""


def measure(cam_cfg: dict, seconds: float = 6.0) -> dict:
    import pyorbbecsdk as ob
    pipe = ob.Pipeline()
    info = pipe.get_device().get_device_info()
    out = {"device": info.get_name(), "serial": info.get_serial_number(), "firmware": info.get_firmware_version(),
           "connection": str(info.get_connection_type())}
    cfg = ob.Config()
    cl = pipe.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
    dl = pipe.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR)
    fps = int(cam_cfg["fps"])
    try:
        cp = cl.get_video_stream_profile(cam_cfg["color_width"], cam_cfg["color_height"], ob.OBFormat.MJPG, fps)
    except Exception:                                           # noqa: BLE001
        cp = cl.get_default_video_stream_profile()
    dp = dl.get_video_stream_profile(cam_cfg["depth_width"], cam_cfg["depth_height"], ob.OBFormat.Y16, fps)
    cfg.enable_stream(cp)
    cfg.enable_stream(dp)
    pipe.enable_frame_sync()
    out["colour_mode"] = "%dx%d@%d %s" % (cp.get_width(), cp.get_height(), cp.get_fps(), str(cp.get_format()).split(".")[-1])
    out["depth_mode"] = "%dx%d@%d" % (dp.get_width(), dp.get_height(), dp.get_fps())
    pipe.start(cfg)
    try:
        warm, end = time.time() + 2.0, time.time() + 2.0 + seconds
        nc = nd = 0
        while time.time() < end:
            fs = pipe.wait_for_frames(300)
            if fs is None or time.time() < warm:
                continue
            nc += fs.get_color_frame() is not None
            nd += fs.get_depth_frame() is not None
    finally:
        pipe.stop()
    out["colour_fps"], out["depth_fps"] = nc / seconds, nd / seconds
    return out


def report(r: dict, target: float) -> bool:
    usb3 = "USB3" in r["connection"].upper().replace(".", "").replace(" ", "")
    ok = usb3 and r["depth_fps"] >= target
    print("\n  %s  serial %s  firmware %s" % (r["device"], r["serial"], r["firmware"]))
    print("  link        : %s %s" % (r["connection"], "(SuperSpeed - good)" if usb3 else "(TOO SLOW for depth)"))
    print("  colour      : %-22s measured %5.1f fps" % (r["colour_mode"], r["colour_fps"]))
    print("  depth       : %-22s measured %5.1f fps" % (r["depth_mode"], r["depth_fps"]))
    if ok:
        print("  RESULT      : OK - colour and depth at full rate.")
    elif usb3:
        print("  RESULT      : USB 3 link, but depth is %.1f fps (< %.0f). Close other camera programs; put the laptop on "
              "its charger and the 'High performance' power plan; check again." % (r["depth_fps"], target))
    else:
        print("  RESULT      : depth %.1f fps - limited by the USB 2 link." % r["depth_fps"])
        print(ADVICE_USB2)
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--watch", action="store_true", help="check again every few seconds (replug and watch)")
    ap.add_argument("--target", type=float, default=20.0, help="depth fps counted as OK (default 20)")
    a = ap.parse_args()
    cam_cfg = json.loads((HERE / "config.json").read_text(encoding="utf-8"))["camera"]
    while True:
        try:
            ok = report(measure(cam_cfg), a.target)
        except Exception as exc:                                # noqa: BLE001 (camera unplugged / busy)
            ok = False
            print("\n  camera not available (%s) - plugged in? another program using it?" % exc)
        if not a.watch:
            return 0 if ok else 1
        try:
            print("  ... next check in 3 s (Ctrl+C to end)")
            time.sleep(3.0)
        except KeyboardInterrupt:
            return 0


if __name__ == "__main__":
    raise SystemExit(main())
