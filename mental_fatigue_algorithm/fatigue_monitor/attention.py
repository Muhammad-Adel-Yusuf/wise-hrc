"""Workstation attention classification with fuzzy angular membership.

Angles are head-pose azimuth (yaw) and elevation (pitch) in degrees, in the
camera frame. yaw > 0: face turned toward the right side of the image;
pitch > 0: looking up (looking down is negative).
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from statistics import median, pstdev
from typing import Any, Dict, List, Optional, Tuple

STATION_LABELS = {"assembly": "Assembly", "laptop": "Laptop", "storage": "Storage"}
# Attention labels (besides the three station labels above):
LABEL_AREA = "Working area"                  # inside the calibrated working area, not at a specific station
LABEL_OUTSIDE = "Outside working area"       # valid head pose, outside the calibrated boundary -> disengagement
LABEL_UNKNOWN = "Unknown"                    # no/invalid pose or calibration incomplete -> never disengagement
KEY_TO_STATION = {"1": "assembly", "2": "laptop", "3": "storage"}


def fuzzy_membership(alpha: float, a_min: float, a_max: float) -> float:
    """Raised-cosine membership f(alpha): 1 inside a_min, 0 beyond a_max."""
    a = abs(alpha)
    if a <= a_min:
        return 1.0
    if a > a_max:
        return 0.0
    return 0.5 * (1.0 + math.cos(math.pi * (a - a_min) / (a_max - a_min)))


def wrap_deg(a: float) -> float:
    """Wrap an angle difference to [-180, 180)."""
    return (a + 180.0) % 360.0 - 180.0


Point = Tuple[float, float]


def convex_hull(points: List[Point]) -> List[Point]:
    """Andrew's monotone chain; returns the hull in counter-clockwise order."""
    pts = sorted(set(points))
    if len(pts) <= 2:
        return pts

    def cross(o: Point, a: Point, b: Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower: List[Point] = []
    for q in pts:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(q)
    upper: List[Point] = []
    for q in reversed(pts):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(q)
    return lower[:-1] + upper[:-1]


def distance_to_polygon(pt: Point, poly: List[Point]) -> float:
    """0 if pt is inside (or on) the convex CCW polygon, else the distance to its boundary (degrees)."""
    n = len(poly)
    if n == 0:
        return math.inf
    inside = n >= 3
    best = math.inf
    for i in range(n):
        a, b = poly[i], poly[(i + 1) % n]
        ex, ey = b[0] - a[0], b[1] - a[1]
        if n >= 3 and ex * (pt[1] - a[1]) - ey * (pt[0] - a[0]) < 0:
            inside = False                                     # right of a CCW edge -> outside
        seg2 = ex * ex + ey * ey
        u = 0.0 if seg2 == 0 else max(0.0, min(1.0, ((pt[0] - a[0]) * ex + (pt[1] - a[1]) * ey) / seg2))
        best = min(best, math.hypot(pt[0] - (a[0] + u * ex), pt[1] - (a[1] + u * ey)))
    return 0.0 if inside else best


@dataclass
class AttentionResult:
    label: str                                   # Assembly/Laptop/Storage/None/Unknown
    memberships: Dict[str, Optional[float]] = field(default_factory=dict)
    reason: str = ""


class AttentionClassifier:
    """Stores per-station reference directions (+ spread) and classifies head direction.

    Each station has an elliptical/box "core" (membership 1) around its calibrated
    centre whose half-widths scale with the spread measured during calibration
    (``k_core`` * std, clipped to sane limits), followed by a soft fade-out. When the
    regions of two stations overlap, the station whose centre is *closest in
    normalised units* wins (not simply the first one), and a small hysteresis
    margin stops flicker on the boundary.
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg["attention"]
        self.path = Path(self.cfg["calibration_file"])
        self.refs: Dict[str, Tuple[float, float]] = {}
        self.spreads: Dict[str, Tuple[float, float]] = {}     # (yaw_std, pitch_std) per station
        self.ear_refs: Dict[str, float] = {}                  # open-eye EAR seen while looking at the station
        self._current: Optional[str] = None                   # last winner (hysteresis)
        self.load()

    # -- persistence -------------------------------------------------------
    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            stations = {k: v for k, v in data.get("stations", {}).items() if k in STATION_LABELS}
            self.refs = {k: (float(v["yaw"]), float(v["pitch"])) for k, v in stations.items()}
            self.spreads = {k: (float(v["yaw_std"]), float(v["pitch_std"]))
                            for k, v in stations.items() if "yaw_std" in v and "pitch_std" in v}
            self.ear_refs = {k: float(v["ear_ref"]) for k, v in stations.items() if v.get("ear_ref") is not None}
        except (OSError, ValueError, KeyError, TypeError):
            self.refs, self.spreads, self.ear_refs = {}, {}, {}

    def save(self) -> None:
        stations: Dict[str, Dict[str, float]] = {}
        for k, v in self.refs.items():
            stations[k] = {"yaw": v[0], "pitch": v[1]}
            if k in self.spreads:
                stations[k]["yaw_std"], stations[k]["pitch_std"] = self.spreads[k]
            if k in self.ear_refs:
                stations[k]["ear_ref"] = self.ear_refs[k]
        payload: Dict[str, Any] = {"stations": stations}
        if self.is_complete:      # informational copy; it is re-derived from the stations on load
            payload["working_area"] = {
                "margin_deg": self.cfg["working_area_margin_deg"],
                "reference_yaw_deg": self._yaw_ref(),
                "polygon_yaw_rel_pitch": [[round(a, 3), round(b, 3)] for a, b in self.working_area_polygon()],
            }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    def reset(self) -> None:
        self.refs, self.spreads, self._current = {}, {}, None
        self.ear_refs.clear()                                  # keep the same dict object (shared with StationEarNorm)
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass

    def set_reference(self, station: str, yaw: float, pitch: float,
                      yaw_std: Optional[float] = None, pitch_std: Optional[float] = None,
                      ear_ref: Optional[float] = None) -> None:
        self.refs[station] = (yaw, pitch)
        if ear_ref is not None:
            self.ear_refs[station] = float(ear_ref)
        if yaw_std is not None and pitch_std is not None:
            self.spreads[station] = (float(yaw_std), float(pitch_std))
        else:
            self.spreads.pop(station, None)                    # fall back to fixed config widths
        self.save()

    @property
    def is_complete(self) -> bool:
        return all(s in self.refs for s in STATION_LABELS)

    # -- region geometry ---------------------------------------------------
    def widths(self, name: str) -> Tuple[float, float, float, float]:
        """(theta_min, theta_max, phi_min, phi_max): core / outer half-widths in degrees."""
        if name not in self.spreads:                           # old calibration without spread
            p = self.cfg["stations"][name]
            return p["theta_min"], p["theta_max"], p["phi_min"], p["phi_max"]
        r = self.cfg["ranges"]
        sy, sp = self.spreads[name]
        ov = (self.cfg.get("overrides") or {}).get(name) or {}   # manual half-widths from config.json

        def one(sd: float, axis: str) -> Tuple[float, float]:
            core = min(max(r["k_core"] * sd, r["min_core_deg"][axis]), r["max_core_deg"][axis])
            if ov.get(f"{axis}_core_deg") is not None:
                core = float(ov[f"{axis}_core_deg"])
            fade = min(max(r["fade_k"] * sd, r["min_fade_deg"]), r["max_fade_deg"])
            return core, core + fade

        ty0, ty1 = one(sy, "yaw")
        tp0, tp1 = one(sp, "pitch")
        return ty0, ty1, tp0, tp1

    def range_text(self, name: str) -> str:
        """Short text of the calibrated range (core, membership = 1) for the on-screen display."""
        if name not in self.refs:
            return "not calibrated"
        ry, rp = self.refs[name]
        ty0, _, tp0, _ = self.widths(name)
        return f"Y {ry - ty0:.0f}..{ry + ty0:.0f}  P {rp - tp0:.0f}..{rp + tp0:.0f}"

    def distance(self, name: str, yaw: float, pitch: float) -> float:
        """Normalised distance to the station centre (1.0 = edge of the core)."""
        ry, rp = self.refs[name]
        ty0, _, tp0, _ = self.widths(name)
        return math.hypot(wrap_deg(yaw - ry) / ty0, (pitch - rp) / tp0)

    # -- working area ------------------------------------------------------
    def _yaw_ref(self) -> float:
        """Reference yaw so relative yaws never straddle the +-180 wrap."""
        return self.refs["assembly"][0] if "assembly" in self.refs else next(iter(self.refs.values()))[0]

    def working_area_polygon(self) -> List[Point]:
        """Convex boundary (yaw relative to `_yaw_ref`, pitch) of the calibrated working area.

        It is the convex hull of the calibrated 'core' rectangle of every workstation
        (centre +- core half-widths, i.e. the directions actually used when looking at that
        station), so it also contains the normal visual region *between* the stations.
        `working_area_margin_deg` is added when testing a point (see `in_working_area`).
        """
        y0 = self._yaw_ref()
        pts: List[Point] = []
        for name, (ry, rp) in self.refs.items():
            ty0, _, tp0, _ = self.widths(name)
            cy = wrap_deg(ry - y0)
            pts += [(cy + sy * ty0, rp + sp * tp0) for sy in (-1, 1) for sp in (-1, 1)]
        return convex_hull(pts)

    def in_working_area(self, yaw: float, pitch: float) -> bool:
        if not self.refs:
            return False
        d = distance_to_polygon((wrap_deg(yaw - self._yaw_ref()), pitch), self.working_area_polygon())
        return d <= self.cfg["working_area_margin_deg"]

    # -- classification ----------------------------------------------------
    def memberships(self, yaw: float, pitch: float) -> Dict[str, Optional[float]]:
        out: Dict[str, Optional[float]] = {}
        for name in STATION_LABELS:
            if name not in self.refs:
                out[name] = None
                continue
            ty0, ty1, tp0, tp1 = self.widths(name)
            ry, rp = self.refs[name]
            a_theta = fuzzy_membership(wrap_deg(yaw - ry), ty0, ty1)
            a_phi = fuzzy_membership(pitch - rp, tp0, tp1)
            out[name] = a_theta * a_phi          # A_i = f_theta * f_phi
        return out

    def classify(self, yaw: Optional[float], pitch: Optional[float]) -> AttentionResult:
        if yaw is None or pitch is None or not (math.isfinite(yaw) and math.isfinite(pitch)):
            return AttentionResult(LABEL_UNKNOWN, {}, "no head pose")
        mem = self.memberships(yaw, pitch)
        if not self.is_complete:
            return AttentionResult(LABEL_UNKNOWN, mem, "workstation calibration incomplete")
        thr = self.cfg["membership_threshold"]
        cands = [n for n in STATION_LABELS if (mem[n] or 0.0) > thr]
        if not cands:
            self._current = None
            if self.in_working_area(yaw, pitch):               # between the stations
                return AttentionResult(LABEL_AREA, mem)
            return AttentionResult(LABEL_OUTSIDE, mem)
        # Overlap: nearest station centre (in normalised units) wins ...
        dist = {n: self.distance(n, yaw, pitch) for n in cands}
        best = min(cands, key=lambda n: (dist[n], -(mem[n] or 0.0)))
        # ... unless the previous station is still nearly as good (hysteresis).
        cur = self._current
        if cur in dist and cur != best and dist[cur] <= dist[best] + self.cfg["switch_margin"]:
            best = cur
        self._current = best
        return AttentionResult(STATION_LABELS[best], mem)


class CalibrationCapture:
    """Collects head-pose samples until `duration_s` seconds of VALID data were gathered
    (or `timeout_s` elapsed), then stores median and std.

    Counting valid time instead of wall-clock time means a low frame rate or a face that is
    briefly lost does not make the calibration fail; the capture just takes a little longer.
    """

    def __init__(self, station: str, t_start: float, duration_s: float, min_samples: int,
                 timeout_s: Optional[float] = None) -> None:
        self.station = station
        self.t_start = t_start
        self.duration_s = duration_s
        self.min_samples = min_samples
        self.timeout_s = timeout_s if timeout_s is not None else 3.0 * duration_s
        self.yaws: List[float] = []
        self.pitches: List[float] = []
        self.ears: List[float] = []
        self.valid_s = 0.0
        self._last_valid_t: Optional[float] = None

    def add(self, yaw: float, pitch: float, t: Optional[float] = None, ear: Optional[float] = None) -> None:
        self.yaws.append(yaw)
        self.pitches.append(pitch)
        if ear is not None and math.isfinite(ear):
            self.ears.append(float(ear))
        if t is not None:
            if self._last_valid_t is not None:
                self.valid_s += min(max(t - self._last_valid_t, 0.0), 0.25)
            self._last_valid_t = t

    def done(self, t: float) -> bool:
        return self.valid_s >= self.duration_s or t - self.t_start >= self.timeout_s

    def progress(self) -> float:
        return min(1.0, self.valid_s / self.duration_s) if self.duration_s > 0 else 1.0

    def summary(self, t: float) -> str:
        return f"{len(self.yaws)} samples, {self.valid_s:.1f}/{self.duration_s:.0f} s valid, {t - self.t_start:.1f} s elapsed"

    def ear_ref(self) -> Optional[float]:
        """Open-eye EAR at this station (75th percentile of the valid samples), None if too few."""
        if len(self.ears) < 5:
            return None
        return float(sorted(self.ears)[int(0.75 * (len(self.ears) - 1))])

    def result(self) -> Optional[Tuple[float, float, float, float]]:
        """(yaw_median, pitch_median, yaw_std, pitch_std)."""
        if len(self.yaws) < self.min_samples:
            return None
        return median(self.yaws), median(self.pitches), pstdev(self.yaws), pstdev(self.pitches)
