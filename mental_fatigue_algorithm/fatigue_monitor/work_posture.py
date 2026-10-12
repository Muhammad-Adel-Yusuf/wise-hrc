"""Face-free fallback for ALGORITHM 3: the "assembly work posture" and the face-loss decision.

Problem: when the worker leans over the assembly area (e.g. to write) the face landmarks disappear, so head
yaw/pitch is unavailable and the attention label would be UNKNOWN. This module decides, WITHOUT the face, whether the
worker is *engaged in the assembly task*. It NEVER claims that the worker is looking at the assembly: the result is
TASK_ENGAGED ("present and working, gaze unknown"), a different state from FOCUSED.

SAFETY RULE (the look-away rule of algorithm 3 is not weakened). The fallback may only convert frames that would
otherwise be UNKNOWN. Priorities, highest first (the caller applies 1 and 2 before this module is consulted):

  1. face visible and pointing outside the working area  -> OUTSIDE   (hands / task context are ignored)
  2. face lost, body detected turned away (side / back)  -> OUTSIDE   (hands / task context are ignored)
  3. face lost, body facing the camera, both hands in the calibrated assembly zone and moving, the body in the
     calibrated work posture, recent step activity, and the time limit not exceeded -> TASK_ENGAGED
  4. otherwise                                            -> UNKNOWN  ("attention unverified" after a few seconds)
  5. face visible at a station                            -> FOCUSED (normal path, not this module)

Additional guards: hands never reset or interrupt an outside timer; if the last known state was OUTSIDE, engagement is
blocked until the face has been seen again at a station; engagement is time-limited (max_engaged_s).

Face-free calibration (key 4): the worker takes the normal assembly posture (leaning, as when writing) and the
system stores, from MediaPipe Pose + Hand landmarks only: the shoulder width and centre, and the box in which the
hand centres move, in shoulder-width units. No face, depth or head pose is needed.

INITIAL ENGINEERING PARAMETERS (config.json -> work_posture); all values need experimental calibration.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from . import geometry as geo

TASK_ENGAGED = "TASK_ENGAGED"


@dataclass
class PostureFeatures:
    """Face-free geometry of one frame (pixels; hands in shoulder-width units relative to the shoulder centre)."""
    sw: float                                   # shoulder width, px
    cx: float                                   # shoulder centre, px
    cy: float
    hands_rel: List[Tuple[float, float]]        # hand centres, ((x - cx)/sw, (y - cy)/sw)
    hands_abs: List[Tuple[float, float]]        # hand centres in px


def extract_features(body: Optional[np.ndarray], hands_px: Sequence[np.ndarray],
                     min_visibility: float) -> Optional[PostureFeatures]:
    """None unless both shoulders are visible (no torso -> no posture decision)."""
    if body is None or len(body) < 13:
        return None
    ls, rs = body[geo.POSE_L_SHOULDER], body[geo.POSE_R_SHOULDER]
    if ls[2] < min_visibility or rs[2] < min_visibility:
        return None
    sw = float(math.hypot(ls[0] - rs[0], ls[1] - rs[1]))
    if sw < 1e-3:
        return None
    cx, cy = 0.5 * float(ls[0] + rs[0]), 0.5 * float(ls[1] + rs[1])
    cents = [np.asarray(h, float).mean(axis=0) for h in hands_px if h is not None and len(h)]
    return PostureFeatures(sw, cx, cy, [((float(c[0]) - cx) / sw, (float(c[1]) - cy) / sw) for c in cents],
                           [(float(c[0]), float(c[1])) for c in cents])


@dataclass
class PostureRef:
    """The calibrated assembly work posture."""
    sw: float
    cx: float
    cy: float
    x_lo: float                                 # hand-zone box in shoulder-width units
    x_hi: float
    y_lo: float
    y_hi: float
    n_samples: int = 0

    def to_dict(self) -> Dict[str, Any]:
        return {k: getattr(self, k) for k in ("sw", "cx", "cy", "x_lo", "x_hi", "y_lo", "y_hi", "n_samples")}

    @staticmethod
    def from_dict(d: Dict[str, Any]) -> "PostureRef":
        return PostureRef(float(d["sw"]), float(d["cx"]), float(d["cy"]), float(d["x_lo"]), float(d["x_hi"]),
                          float(d["y_lo"]), float(d["y_hi"]), int(d.get("n_samples", 0)))


class PostureCapture:
    """Collects samples while the worker holds the assembly work posture (key 4)."""

    def __init__(self, t0: float, cfg: Dict[str, Any]) -> None:
        self.t0 = t0
        self.cfg = cfg
        self.samples: List[PostureFeatures] = []
        self.hand_points: List[Tuple[float, float]] = []
        self.frames = 0
        self.facings: Dict[str, int] = {}

    def add(self, feat: Optional[PostureFeatures], facing: Optional[str] = None) -> None:
        self.frames += 1
        if facing:
            self.facings[facing] = self.facings.get(facing, 0) + 1
        if feat is None:
            return
        self.samples.append(feat)
        self.hand_points.extend(feat.hands_rel)

    def done(self, t: float) -> bool:
        c = self.cfg
        if t - self.t0 >= c["capture_timeout_s"]:
            return True
        return (t - self.t0 >= c["capture_s"] and len(self.samples) >= c["capture_min_samples"]
                and len(self.hand_points) >= c["capture_min_hand_samples"])

    def progress(self, t: float) -> float:
        return min(1.0, (t - self.t0) / self.cfg["capture_s"])

    def summary(self) -> str:
        return (f"{len(self.samples)}/{self.frames} frames with both shoulders, "
                f"{len(self.hand_points)} hand samples")

    def result(self) -> Tuple[Optional[PostureRef], str]:
        """(PostureRef | None, message). A failure explains what to fix."""
        c = self.cfg
        if len(self.samples) < c["capture_min_samples"]:
            return None, ("too few frames with both shoulders visible (" + self.summary() +
                          "): sit so both shoulders are in view")
        if len(self.hand_points) < c["capture_min_hand_samples"]:
            return None, ("too few hand samples (" + self.summary() +
                          "): keep both hands working in the assembly area in view of the camera")
        sw = float(np.median([s.sw for s in self.samples]))
        cx = float(np.median([s.cx for s in self.samples]))
        cy = float(np.median([s.cy for s in self.samples]))
        pts = np.asarray(self.hand_points, float)
        lo, hi = np.percentile(pts, 5, axis=0), np.percentile(pts, 95, axis=0)
        ref = PostureRef(sw, cx, cy, float(lo[0]), float(hi[0]), float(lo[1]), float(hi[1]), len(self.samples))
        return ref, "assembly work posture calibrated (" + self.summary() + ")"


@dataclass
class PostureCheck:
    in_posture: bool = False                    # shoulders at the calibrated place / scale
    hands_in_zone: int = 0
    reason: str = ""


class WorkPosture:
    """Holds the reference posture, checks frames against it and measures hand motion (face-free)."""

    def __init__(self, cfg: Dict[str, Any], min_visibility: float = 0.5) -> None:
        self.cfg = cfg                          # the "work_posture" section
        self.min_visibility = min_visibility
        self.ref: Optional[PostureRef] = None
        self._prev: List[Tuple[float, float]] = []
        self._prev_t: Optional[float] = None
        self._moving_until = -1e9
        self.speed = 0.0

    @property
    def calibrated(self) -> bool:
        return self.ref is not None

    def features(self, body: Optional[np.ndarray], hands_px: Sequence[np.ndarray]) -> Optional[PostureFeatures]:
        return extract_features(body, hands_px, self.min_visibility)

    def reset(self) -> None:
        self.ref = None
        self.reset_motion()

    def reset_motion(self) -> None:
        self._prev, self._prev_t, self._moving_until, self.speed = [], None, -1e9, 0.0

    def check(self, feat: Optional[PostureFeatures]) -> PostureCheck:
        c, r = self.cfg, self.ref
        if r is None:
            return PostureCheck(False, 0, "assembly posture not calibrated (press 4)")
        if feat is None:
            return PostureCheck(False, 0, "shoulders not visible")
        scale = feat.sw / r.sw
        if abs(scale - 1.0) > c["scale_tol"]:
            return PostureCheck(False, 0, f"shoulder scale {scale:.2f} outside the calibrated posture")
        shift = math.hypot(feat.cx - r.cx, feat.cy - r.cy) / r.sw
        if shift > c["position_tol_sw"]:
            return PostureCheck(False, 0, f"body moved {shift:.2f} shoulder widths from the calibrated posture")
        m = c["zone_margin_sw"]
        n = sum(1 for (x, y) in feat.hands_rel
                if r.x_lo - m <= x <= r.x_hi + m and r.y_lo - m <= y <= r.y_hi + m)
        return PostureCheck(True, n, "")

    def update_motion(self, t: float, feat: Optional[PostureFeatures]) -> bool:
        """Hand motion in shoulder widths per second (face-free); True while hands moved recently."""
        c = self.cfg
        cents = feat.hands_abs if feat is not None else []
        speed = 0.0
        if feat is not None and self._prev_t is not None and cents and self._prev:
            dt = t - self._prev_t
            if 0.0 < dt <= 0.5:
                for p in cents:
                    d = min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in self._prev)
                    speed = max(speed, d / (feat.sw * dt))
        self._prev, self._prev_t = cents, t
        self.speed = 0.5 * self.speed + 0.5 * speed
        if self.speed >= c["hand_speed_min_sw_s"]:
            self._moving_until = t + c["hand_hold_s"]
        return t <= self._moving_until

    # -- persistence -----------------------------------------------------------------------
    def save(self, path: str) -> None:
        if self.ref is not None:
            with open(path, "w", encoding="utf-8") as fh:
                json.dump(self.ref.to_dict(), fh, indent=2)

    def load(self, path: str) -> bool:
        try:
            with open(path, "r", encoding="utf-8") as fh:
                self.ref = PostureRef.from_dict(json.load(fh))
            return True
        except (OSError, ValueError, KeyError, TypeError):
            return False


class FaceLossFallback:
    """Timeline of one face-loss period: hold -> evaluate -> decide (durations: work_posture.hold_s / decision_s).

    update() is called on frames WITHOUT a usable face and not turned away (priorities 1 and 2 are the caller's).
    Returns (label, reason): label is the held last label, TASK_ENGAGED, or None (= UNKNOWN).
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.reset()

    def reset(self) -> None:
        self._last_label: Optional[str] = None      # last known face-based public label
        self._blocked = False                       # last known state was OUTSIDE: no engagement until a station is seen
        self._last_step_t = -1e9
        self._lost_since: Optional[float] = None
        self._cond_since: Optional[float] = None
        self._cond_last: Optional[float] = None
        self._confirmed = False
        self.reason = ""

    # -- notifications from the monitor --------------------------------------------------------
    def note_face(self, t: float, public_label: str) -> None:
        """A valid-face frame (public label: LAPTOP / ASSEMBLY / STORAGE / WORKING_AREA / OUTSIDE_WORKING_AREA)."""
        self._lost_since = None
        self._cond_since = self._cond_last = None
        self._confirmed = False
        if public_label == "UNKNOWN":
            return
        self._last_label = public_label
        if public_label == "OUTSIDE_WORKING_AREA":
            self._blocked = True
        else:
            self._blocked = False
            if public_label in ("LAPTOP", "ASSEMBLY", "STORAGE"):
                self._last_step_t = t

    def note_outside(self, t: float) -> None:
        """Outside was decided without a valid face (body turned away): same guard as a visible look-away."""
        self._last_label = "OUTSIDE_WORKING_AREA"
        self._blocked = True
        self._lost_since = None
        self._cond_since = self._cond_last = None
        self._confirmed = False

    def note_step_activity(self, t: float) -> None:
        """Instruction change / attempted / completed event."""
        self._last_step_t = t

    # -- per-frame decision --------------------------------------------------------------------
    def update(self, t: float, body_front: bool, posture: PostureCheck, hands_moving: bool
               ) -> Tuple[Optional[str], str]:
        c = self.cfg
        if self._lost_since is None:
            self._lost_since = t
            self._cond_since = self._cond_last = None
            self._confirmed = False
        lost = t - self._lost_since
        if lost < c["hold_s"]:
            self.reason = "holding last attention state"
            return self._last_label, self.reason
        why = ""
        if not c["enabled"]:
            why = "fallback disabled"
        elif self._blocked:
            why = "last known state was outside: engagement not allowed"
        elif lost > c["max_engaged_s"]:
            why = "engagement time limit"
        elif not body_front:
            why = "body not facing the camera"
        elif not posture.in_posture:
            why = posture.reason or "not in assembly posture"
        elif posture.hands_in_zone < c["min_hands_in_zone"]:
            why = f"hands in assembly zone: {posture.hands_in_zone}/{c['min_hands_in_zone']}"
        elif not hands_moving:
            why = "hands not moving"
        elif t - self._last_step_t > c["step_context_s"]:
            why = "no recent step activity"
        cond = not why
        if cond:
            if self._cond_since is None:
                self._cond_since = t
            self._cond_last = t
            if t - self._cond_since >= max(0.0, c["decision_s"] - c["hold_s"]):
                self._confirmed = True
        elif self._cond_last is not None and t - self._cond_last > c["condition_gap_s"]:
            self._cond_since, self._cond_last, self._confirmed = None, None, False
        if self._confirmed and (cond or (self._cond_last is not None and t - self._cond_last <= c["condition_gap_s"])):
            self.reason = "task engaged (face hidden, gaze unknown)"
            return TASK_ENGAGED, self.reason
        self.reason = why or "confirming task engagement"
        return None, self.reason
