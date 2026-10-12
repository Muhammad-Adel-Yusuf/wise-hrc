"""Hardware-independent temporal logic: rolling windows and state machines.

Every class takes explicit timestamps (seconds) so it can be unit-tested with
synthetic signals. Missing data is NEVER interpreted as closed eyes,
distraction or fatigue: invalid samples pause or abort detectors instead.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from statistics import median
from typing import Any, Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

from .attention import LABEL_OUTSIDE

NAN = float("nan")


# --------------------------------------------------------------------------
# Generic helpers
# --------------------------------------------------------------------------
class RollingEventCounter:
    """Weighted event counter over a rolling time window."""

    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self._events: Deque[Tuple[float, float]] = deque()

    def add(self, t: float, weight: float = 1.0) -> None:
        self._events.append((t, weight))

    def count(self, t: float) -> float:
        while self._events and self._events[0][0] < t - self.window_s:
            self._events.popleft()
        return float(sum(w for _, w in self._events))

    def clear(self) -> None:
        self._events.clear()


class BaselineLearner:
    """Learns a personal reference value from valid samples (percentile)."""

    def __init__(self, duration_s: float, min_valid_s: float, percentile: float) -> None:
        self.duration_s = duration_s
        self.min_valid_s = min_valid_s
        self.percentile = percentile
        self.t_start: Optional[float] = None
        self._last_t: Optional[float] = None
        self._values: List[float] = []
        self._valid_s = 0.0
        self.value: Optional[float] = None
        self.sigma: Optional[float] = None

    @property
    def done(self) -> bool:
        return self.value is not None

    def progress(self, t: float) -> float:
        if self.done:
            return 1.0
        if self.t_start is None:
            return 0.0
        return min(1.0, max((t - self.t_start) / self.duration_s, self._valid_s / self.min_valid_s * 0.999))

    def update(self, t: float, sample: Optional[float]) -> bool:
        """Feed one sample (None = invalid). Returns True when learning completes now."""
        if self.done:
            return False
        if self.t_start is None:
            self.t_start = t
        dt = 0.0 if self._last_t is None else min(t - self._last_t, 0.25)
        self._last_t = t
        if sample is not None and np.isfinite(sample):
            self._values.append(float(sample))
            self._valid_s += dt
        if t - self.t_start >= self.duration_s and self._valid_s >= self.min_valid_s:
            arr = np.asarray(self._values)
            self.value = float(np.percentile(arr, self.percentile))
            low = arr[arr <= self.value] if self.percentile < 50 else arr[arr >= self.value]
            mad = float(np.median(np.abs(low - np.median(low)))) if low.size else 0.0
            self.sigma = 1.4826 * mad
            self._values = []
            return True
        return False


# --------------------------------------------------------------------------
# 1. PERCLOS (time-weighted, recomputed from stored EAR with current threshold)
# --------------------------------------------------------------------------
class PerclosTracker:
    """PERCLOS = closed time / valid eye-observation time in a rolling window."""

    def __init__(self, window_s: float, min_valid_s: float, max_dt: float = 0.25) -> None:
        self.window_s = window_s
        self.min_valid_s = min_valid_s
        self.max_dt = max_dt
        self._buf: Deque[Tuple[float, float, float]] = deque()  # (t, dt, ear or nan)
        self._last_t: Optional[float] = None

    def update(self, t: float, ear: Optional[float], valid: bool) -> None:
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if dt < 0 or dt > self.max_dt:      # clock glitch or gap: not observation time
            dt = 0.0
        self._buf.append((t, dt, float(ear) if (valid and ear is not None) else NAN))
        while self._buf and self._buf[0][0] < t - self.window_s:
            self._buf.popleft()

    def value(self, threshold: float, min_valid_s: Optional[float] = None) -> Tuple[Optional[float], float]:
        """Return (PERCLOS or None if too little valid data, valid seconds)."""
        need = self.min_valid_s if min_valid_s is None else min_valid_s
        valid_s = closed_s = 0.0
        for _, dt, ear in self._buf:
            if ear == ear:                      # not NaN
                valid_s += dt
                if ear < threshold:
                    closed_s += dt
        if valid_s < need or valid_s <= 0:
            return None, valid_s
        return closed_s / valid_s, valid_s


class StationEarNorm:
    """Open-eye EAR reference per head-pitch bin -> scale factor mapping EAR to the global-baseline scale.

    Looking down lowers the EAR without the eyes being closed. The reference of a pitch bin is the
    75th percentile of its EAR samples in the last window_s seconds (needs min_valid_s of samples), else the
    value seeded from the station calibration (seed_fn), else the nearest bin within `neighbor_bins`.
    scale = clip(global_baseline / reference, scale_min, scale_max); 1.0 when no reference exists.
    Known limit: if the eyes are closed more than ~75 % of the time at one pitch, its reference drifts down.
    """

    def __init__(self, cfg: Dict[str, Any], seed_fn: Optional[Any] = None) -> None:
        self.cfg = cfg
        self.seed_fn = seed_fn
        self._buf: Dict[int, Deque[Tuple[float, float, float]]] = {}
        self._last_t: Optional[float] = None

    def reset(self) -> None:
        self._buf.clear()
        self._last_t = None

    def bin_of(self, pitch: float) -> int:
        return int(np.floor(pitch / self.cfg["pitch_bin_deg"]))

    def update(self, t: float, pitch: Optional[float], ear: Optional[float], valid: bool) -> None:
        dt = 0.0 if self._last_t is None else min(max(t - self._last_t, 0.0), 0.25)
        self._last_t = t
        if pitch is None or not valid or ear is None or not np.isfinite(ear) or not np.isfinite(pitch):
            return
        buf = self._buf.setdefault(self.bin_of(pitch), deque())
        buf.append((t, dt, float(ear)))
        while buf and buf[0][0] < t - self.cfg["window_s"]:
            buf.popleft()

    def _bin_ref(self, b: int, seeds: Dict[int, float]) -> Optional[float]:
        buf = self._buf.get(b)
        if buf and sum(dt for _, dt, _ in buf) >= self.cfg["min_valid_s"]:
            return float(np.percentile([e for _, _, e in buf], self.cfg["percentile"]))
        return seeds.get(b)

    def reference(self, pitch: Optional[float]) -> Optional[float]:
        if pitch is None or not np.isfinite(pitch):
            return None
        seeds = self.seed_fn() if self.seed_fn else {}
        b0 = self.bin_of(pitch)
        for d in range(0, int(self.cfg["neighbor_bins"]) + 1):
            for b in ((b0,) if d == 0 else (b0 - d, b0 + d)):
                ref = self._bin_ref(b, seeds)
                if ref is not None:
                    return ref
        return None

    def scale(self, pitch: Optional[float], global_base: Optional[float]) -> float:
        if not self.cfg["enabled"] or global_base is None:
            return 1.0
        ref = self.reference(pitch)
        if ref is None or ref <= 0:
            return 1.0
        return float(np.clip(global_base / ref, self.cfg["scale_min"], self.cfg["scale_max"]))


class HandActivity:
    """True while a hand is moving normally (fast enough centroid motion, relative to face width)."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self._prev: List[np.ndarray] = []
        self._prev_t: Optional[float] = None
        self._active_until = -1e9
        self.speed = 0.0                                   # smoothed, face-widths / s (for display/debug)

    def reset(self) -> None:
        self._prev, self._prev_t, self._active_until, self.speed = [], None, -1e9, 0.0

    def update(self, t: float, hands_px: Sequence[np.ndarray], face_width_px: Optional[float]) -> bool:
        if not self.cfg["enabled"]:
            return False
        cents = [np.asarray(h, float).mean(axis=0) for h in hands_px if h is not None and len(h)]
        speed = 0.0
        if self._prev_t is not None and cents and self._prev and face_width_px and face_width_px > 0:
            dt = t - self._prev_t
            if 0.0 < dt <= 0.5:
                for c in cents:
                    d = min(float(np.linalg.norm(c - p)) for p in self._prev)
                    speed = max(speed, d / (face_width_px * dt))
        self._prev, self._prev_t = cents, t
        self.speed = 0.5 * self.speed + 0.5 * speed
        if self.speed >= self.cfg["speed_min_fw_s"]:
            self._active_until = t + self.cfg["hold_s"]
        return self.active(t)

    def active(self, t: float) -> bool:
        return t <= self._active_until


# --------------------------------------------------------------------------
# 2. Eye-closure duration classification
# --------------------------------------------------------------------------
@dataclass
class ClosureEvent:
    start: float
    end: float
    duration: float
    category: str        # blink | long_blink | prolonged | microsleep


def classify_closure(duration: float, cfg: Dict[str, Any]) -> str:
    if duration < cfg["blink_max_s"]:
        return "blink"
    if duration < cfg["long_blink_max_s"]:
        return "long_blink"
    if duration < cfg["prolonged_max_s"]:
        return "prolonged"
    return "microsleep"


class EyeClosureTracker:
    """Tracks closure episodes from a per-frame closed flag."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.events: Deque[ClosureEvent] = deque()
        self._start: Optional[float] = None
        self._last_closed_t: Optional[float] = None
        self._prev_open_t: Optional[float] = None
        self._gap_start: Optional[float] = None
        self.counts: Dict[str, int] = {"blink": 0, "long_blink": 0, "prolonged": 0, "microsleep": 0}

    def reset_events(self) -> None:
        self.events.clear()
        self._start = self._last_closed_t = self._gap_start = None
        self.counts = {k: 0 for k in self.counts}

    def _abort(self) -> None:
        self._start = None
        self._gap_start = None

    def update(self, t: float, valid: bool, closed: bool, occluded: bool = False) -> Optional[ClosureEvent]:
        if occluded:                      # hand rubbing eye: not a real closure
            self._abort()
            self._prev_open_t = None
            return None
        if not valid:
            if self._start is not None:
                if self._gap_start is None:
                    self._gap_start = t
                if t - self._gap_start > self.cfg["gap_tolerance_s"]:
                    self._abort()
            return None
        if closed:
            if self._start is None:
                self._start = t if self._prev_open_t is None else 0.5 * (self._prev_open_t + t)
            self._gap_start = None
            self._last_closed_t = t
            return None
        # eye open
        event = None
        if self._start is not None:
            end = 0.5 * ((self._last_closed_t or t) + t)
            dur = max(0.0, end - self._start)
            cat = classify_closure(dur, self.cfg)
            event = ClosureEvent(self._start, end, dur, cat)
            self.counts[cat] += 1
            if dur >= self.cfg["count_min_s"]:
                self.events.append(event)
            self._abort()
        self._prev_open_t = t
        return event

    def in_progress_duration(self, t: float) -> float:
        if self._start is None or self._gap_start is not None:
            return 0.0
        return max(0.0, t - self._start)

    def weighted_count(self, t: float) -> float:
        """Closures >= count_min_s in the rolling window (microsleep counts extra)."""
        c = self.cfg
        while self.events and self.events[0].end < t - c["window_s"]:
            self.events.popleft()
        total = 0.0
        for ev in self.events:
            total += c["weight_microsleep"] if ev.category == "microsleep" else c["weight_prolonged"]
        d = self.in_progress_duration(t)
        if d >= c["count_min_s"]:
            total += c["weight_microsleep"] if d >= c["prolonged_max_s"] else c["weight_prolonged"]
        return total

    def microsleep_count(self, t: float) -> int:
        """Number of microsleep-like closures (>= prolonged_max_s) in the rolling window.

        A closure still in progress counts as soon as it reaches prolonged_max_s.
        (Named 'microsleep-like': without EEG/physiology this is a visual proxy only.)
        """
        c = self.cfg
        while self.events and self.events[0].end < t - c["window_s"]:
            self.events.popleft()
        n = sum(1 for ev in self.events if ev.category == "microsleep")
        if self.in_progress_duration(t) >= c["prolonged_max_s"]:
            n += 1
        return n

    def prolonged_weighted(self, t: float) -> float:
        """Prolonged closures only (0.5 s <= d < prolonged_max_s), microsleep-like ones excluded."""
        c = self.cfg
        self.microsleep_count(t)                       # prunes the window
        total = c["weight_prolonged"] * sum(1 for ev in self.events if ev.category == "prolonged")
        d = self.in_progress_duration(t)
        if c["count_min_s"] <= d < c["prolonged_max_s"]:
            total += c["weight_prolonged"]
        return total

    def long_closure_overlaps(self, t0: float, t1: float, now: float) -> bool:
        """True if a closure >= count_min_s overlaps [t0, t1] (incl. one in progress)."""
        for ev in self.events:
            if ev.start <= t1 and ev.end >= t0:
                return True
        if self._start is not None and self._start <= t1 and \
                self.in_progress_duration(now) >= self.cfg["count_min_s"]:
            return True
        return False


# --------------------------------------------------------------------------
# 3. Brief vs sustained disengagement (looking outside the working area)
# --------------------------------------------------------------------------
class DistractionTracker:
    """Tracks continuous 'outside the working area' events and classifies each one exactly once.

    'Outside' = valid head pose, outside the calibrated working area (label "Outside working area").
    Workstations and "Working area" (the region between them) are NOT outside. "Unknown" labels
    (no/invalid pose, uncalibrated) never count as outside time.

    Classification of one continuous event (duration d, measured over outside frames only):
      * d <  sustained_threshold_s and the worker RETURNED to the working area -> BRIEF event
        (counted once, when the return is confirmed; an event ended by lost data is not counted);
      * d >= sustained_threshold_s -> SUSTAINED event, never counted as brief.
    A short return (< reentry_debounce_s) does not split one continuous event into two.
    """

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self._outside_since: Optional[float] = None
        self._station_since: Optional[float] = None
        self._last_outside_t: Optional[float] = None
        self._unknown_since: Optional[float] = None
        self._brief: Deque[Tuple[float, float]] = deque()          # (end time, duration) of finished brief events
        self._sustained: Deque[Tuple[float, float]] = deque()      # (end time, duration) of finished sustained events
        self.brief_total = 0                                       # lifetime counters (diagnostics)
        self.sustained_total = 0
        self._current_counted = False
        self._prev_t: Optional[float] = None
        self._dt = 0.0                                             # last frame interval (capped)

    # -- internal -----------------------------------------------------------------
    def clear_current(self) -> None:
        """Forget the event in progress without classifying it (used when collaboration becomes inactive)."""
        self._outside_since = self._station_since = self._unknown_since = self._last_outside_t = None
        self._current_counted = False
        self._prev_t = None

    def _finish(self, end: float, returned: bool) -> None:
        c = self.cfg
        if self._outside_since is not None:
            dur = end - self._outside_since + self._dt              # + one frame: first..last sample spans n-1 gaps
            if dur >= c["sustained_threshold_s"]:
                self._sustained.append((end, dur))                 # (already counted in sustained_total)
            elif returned and dur >= c["min_record_s"]:
                self._brief.append((end, dur))
                self.brief_total += 1
        self._outside_since = None
        self._station_since = None
        self._current_counted = False

    def update(self, t: float, label: str, since: Optional[float] = None) -> None:
        """since: optional back-dated start of an outside episode (e.g. face turned away, confirmed later)."""
        c = self.cfg
        self._dt = 0.0 if self._prev_t is None else min(0.25, max(0.0, t - self._prev_t))
        self._prev_t = t
        if label == LABEL_OUTSIDE:
            self._unknown_since = None
            self._station_since = None
            if self._outside_since is None:
                self._outside_since = t if since is None else min(t, since)
            self._last_outside_t = t
            if not self._current_counted and t - self._outside_since >= c["sustained_threshold_s"]:
                self._current_counted = True
                self.sustained_total += 1
        elif label == "Unknown":
            if self._outside_since is not None:
                if self._unknown_since is None:
                    self._unknown_since = t
                if t - self._unknown_since > c["unknown_tolerance_s"]:
                    self._finish(self._last_outside_t or t, returned=False)   # lost data is not a return
                    self._unknown_since = None
        else:  # a workstation or the region between them
            self._unknown_since = None
            if self._outside_since is not None:
                if self._station_since is None:
                    self._station_since = t
                if t - self._station_since >= c["reentry_debounce_s"]:
                    self._finish(self._last_outside_t or t, returned=True)

    def _prune(self, t: float) -> None:
        c = self.cfg
        lo = t - max(c["brief_window_s"], c["history_window_s"])
        while self._brief and self._brief[0][0] < lo:
            self._brief.popleft()
        while self._sustained and self._sustained[0][0] < t - c["history_window_s"]:
            self._sustained.popleft()

    # -- queries ------------------------------------------------------------------
    def d_outside(self, t: float) -> float:
        """Continuous outside duration; unknown (missing-data) time is not counted."""
        if self._outside_since is None or self._last_outside_t is None:
            return 0.0
        if self._unknown_since is not None or self._station_since is not None:
            # missing data / a glance back at the working area is not outside time
            return max(0.0, self._last_outside_t - self._outside_since)
        return max(0.0, t - self._outside_since)

    def immediate(self, t: float) -> bool:
        """True while the CURRENT continuous outside time is >= sustained_threshold_s."""
        return self.d_outside(t) >= self.cfg["sustained_threshold_s"]

    def brief_count(self, t: float) -> int:
        """Brief events (returned before sustained_threshold_s) in the last brief_window_s.
        The event in progress is NOT included: it may still become sustained."""
        self._prune(t)
        return sum(1 for e, _ in self._brief if e >= t - self.cfg["brief_window_s"])

    def sustained_count(self, t: float) -> int:
        """Finished sustained events in the history window plus the one in progress."""
        self._prune(t)
        return len(self._sustained) + (1 if self.immediate(t) else 0)

    def max_outside_s(self, t: float) -> float:
        """Longest continuous outside stay (brief or sustained, finished or in progress) in the history window."""
        self._prune(t)
        best = max([d for _, d in self._brief] + [d for _, d in self._sustained], default=0.0)
        return max(best, self.d_outside(t))

    def sustained_max_s(self, t: float) -> float:
        """Longest sustained event (>= sustained_threshold_s) in the last history_window_s,
        including the one in progress; 0.0 if there is none."""
        self._prune(t)
        best = max((d for _, d in self._sustained), default=0.0)
        cur = self.d_outside(t)
        if cur >= self.cfg["sustained_threshold_s"]:
            best = max(best, cur)
        return best


# --------------------------------------------------------------------------
# 4. Yawning
# --------------------------------------------------------------------------
@dataclass
class YawnEvent:
    start: float
    end: float
    duration: float
    peak_mar: float


class YawnDetector:
    """Open -> sustained open -> close state machine on MAR (personal baseline)."""

    CLOSED, OPEN, WAIT_CLOSE = 0, 1, 2

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self.base: Optional[float] = None
        self.sigma: float = cfg["sigma_floor"]
        self.state = self.CLOSED
        self._t_start = 0.0
        self._samples: List[float] = []
        self._closed_since: Optional[float] = None
        self._armed_t = -1e9
        self._gap_start: Optional[float] = None
        self._last_event_t = -1e9

    def set_baseline(self, base: float, sigma: float) -> None:
        self.base = base
        self.sigma = max(sigma, self.cfg["sigma_floor"])

    @property
    def open_threshold(self) -> Optional[float]:
        if self.base is None:
            return None
        return self.base + max(self.cfg["delta_abs"], self.cfg["z_score"] * self.sigma)

    @property
    def release_threshold(self) -> Optional[float]:
        if self.base is None:
            return None
        return self.base + self.cfg["release_fraction"] * (self.open_threshold - self.base)

    def update(self, t: float, mar: Optional[float]) -> Optional[YawnEvent]:
        c = self.cfg
        if self.base is None:
            return None
        hi, lo = self.open_threshold, self.release_threshold
        if mar is None:                                  # invalid frame
            if self.state == self.OPEN:
                if self._gap_start is None:
                    self._gap_start = t
                if t - self._gap_start > c["gap_tolerance_s"]:
                    self.state = self.CLOSED
                    self._closed_since, self._armed_t = None, -1e9
            return None
        self._gap_start = None
        if self.state == self.CLOSED:
            if mar < lo:
                if self._closed_since is None:
                    self._closed_since = t
                elif t - self._closed_since >= c["min_closed_before_s"]:
                    self._armed_t = t                  # mouth demonstrably closed
            else:
                self._closed_since = None
            if mar >= hi and t - self._armed_t <= c["arm_hold_s"] and \
                    t - self._last_event_t >= c["refractory_s"]:
                self.state = self.OPEN
                self._t_start = t
                self._samples = [mar]
            return None
        if self.state == self.WAIT_CLOSE:
            if mar < lo:
                self.state = self.CLOSED
                self._closed_since, self._armed_t = t, -1e9
            return None
        # OPEN
        self._samples.append(mar)
        if t - self._t_start > c["max_duration_s"]:      # e.g. mouth left open
            self.state = self.WAIT_CLOSE
            return None
        if mar < lo:                                     # closing -> evaluate
            dur = t - self._t_start
            s = np.asarray(self._samples)
            plateau = float(np.mean(s >= hi))
            cv = float(np.std(s) / max(np.mean(s), 1e-6))
            self.state = self.CLOSED
            self._closed_since, self._armed_t = t, -1e9
            if c["min_duration_s"] <= dur <= c["max_duration_s"] and \
                    plateau >= c["plateau_min_fraction"] and cv <= c["max_cv"]:
                self._last_event_t = t
                return YawnEvent(self._t_start, t, dur, float(s.max()))
        return None


# --------------------------------------------------------------------------
# 5. Head nodding
# --------------------------------------------------------------------------
@dataclass
class NodEvent:
    start: float
    end: float
    peak_drop_deg: float
    confirmed_by_closure: bool
    weight: float


class NodDetector:
    """Downward pitch excursion + return relative to a recent pitch baseline."""

    IDLE, DOWN, REARM = 0, 1, 2

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self._hist: Deque[Tuple[float, float]] = deque()
        self.state = self.IDLE
        self._t0 = 0.0
        self._peak = 0.0
        self._yaw0 = 0.0
        self._yaw_dev = 0.0
        self._peak_membership = 0.0
        self._gap_start: Optional[float] = None
        self._last_event_t = -1e9
        self._pending: List[Tuple[float, float, float]] = []   # (start, end, peak)
        self.baseline: Optional[float] = None

    def _update_baseline(self, t: float) -> Optional[float]:
        """Median pitch over [t-window, t-lag]; lag keeps the excursion out of the baseline."""
        c = self.cfg
        lo, hi = t - c["baseline_window_s"], t - c["baseline_lag_s"]
        vals = [p for (tt, p) in self._hist if lo <= tt <= hi]
        if len(vals) < 5 or not self._hist or (t - self._hist[0][0]) < c["baseline_min_s"]:
            return None
        return float(median(vals))

    def update(self, t: float, pitch: Optional[float], yaw: Optional[float],
               station_membership: float, closure: EyeClosureTracker) -> List[NodEvent]:
        c = self.cfg
        out: List[NodEvent] = []
        # resolve pending candidates once the confirmation wait has passed
        still: List[Tuple[float, float, float]] = []
        for (s, e, peak) in self._pending:
            if t - e >= c["confirm_wait_s"]:
                conf = closure.long_closure_overlaps(s - 0.5, e + 0.5, t)
                out.append(NodEvent(s, e, peak, conf,
                                    c["confirmed_weight"] if conf else c["unconfirmed_weight"]))
            else:
                still.append((s, e, peak))
        self._pending = still

        if pitch is None or yaw is None:
            if self.state == self.DOWN:
                if self._gap_start is None:
                    self._gap_start = t
                if t - self._gap_start > c["gap_tolerance_s"]:
                    self.state = self.REARM
            return out
        self._gap_start = None
        self._hist.append((t, pitch))
        while self._hist and self._hist[0][0] < t - c["baseline_window_s"]:
            self._hist.popleft()
        self.baseline = self._update_baseline(t)
        if self.baseline is None:
            return out
        dp = self.baseline - pitch                       # >0 : head lower than baseline
        return_deg = c["excursion_deg"] * c["return_fraction"]

        if self.state == self.IDLE:
            if dp >= c["excursion_deg"] and t - self._last_event_t >= c["refractory_s"]:
                self.state = self.DOWN
                self._t0, self._peak, self._yaw0 = t, dp, yaw
                self._yaw_dev, self._peak_membership = 0.0, station_membership
        elif self.state == self.DOWN:
            if dp > self._peak:
                self._peak, self._peak_membership = dp, station_membership
            self._yaw_dev = max(self._yaw_dev, abs(yaw - self._yaw0))
            dur = t - self._t0
            if dur > c["max_duration_s"]:
                self.state = self.REARM                   # sustained lowering, not a nod
            elif dp <= return_deg:
                self.state = self.IDLE
                ok = (c["min_duration_s"] <= dur <= c["max_duration_s"]
                      and self._yaw_dev <= c["max_yaw_change_deg"]
                      and self._peak_membership < c["exclude_workstation_membership"])
                if ok:
                    self._last_event_t = t
                    self._pending.append((self._t0, t, self._peak))
        else:  # REARM
            if dp <= return_deg:
                self.state = self.IDLE
        return out


# --------------------------------------------------------------------------
# 6. Eye rubbing
# --------------------------------------------------------------------------
def count_reversals(points: np.ndarray, min_amplitude: float) -> int:
    """Count direction reversals of a 2-D track along its principal axis."""
    if len(points) < 3:
        return 0
    centered = points - points.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    proj = centered @ vt[0]
    pivot, direction, rev = proj[0], 0, 0
    for p in proj[1:]:
        if direction == 0:
            if p - pivot >= min_amplitude:
                direction, pivot = 1, p
            elif pivot - p >= min_amplitude:
                direction, pivot = -1, p
        elif direction == 1:
            if p > pivot:
                pivot = p
            elif pivot - p >= min_amplitude:
                direction, pivot, rev = -1, p, rev + 1
        else:
            if p < pivot:
                pivot = p
            elif p - pivot >= min_amplitude:
                direction, pivot, rev = 1, p, rev + 1
    return rev


@dataclass
class RubEvent:
    start: float
    end: float
    reversals: int


class EyeRubDetector:
    """Hand near an eye (normalised by face width) with oscillating motion."""

    def __init__(self, cfg: Dict[str, Any]) -> None:
        self.cfg = cfg
        self._active = False
        self._t0 = 0.0
        self._last_near = -1e9
        self._track: List[Tuple[float, float]] = []
        self._counted = False
        self._last_event_t = -1e9
        self.last_distance_norm: Optional[float] = None

    def occluded(self, t: float) -> bool:
        """True while a hand is near an eye (+ tail): eye samples are unreliable."""
        return t - self._last_near <= self.cfg["occlusion_tail_s"]

    def update(self, t: float, eye_centers: Optional[np.ndarray], face_width: Optional[float],
               hands: Sequence[np.ndarray]) -> Optional[RubEvent]:
        c = self.cfg
        near = False
        if eye_centers is not None and face_width and face_width > 1e-6 and len(hands) > 0:
            best = (1e9, None, None)                       # dist, eye idx, hand idx
            for hi, pts in enumerate(hands):
                for ei in range(len(eye_centers)):
                    d = float(np.min(np.linalg.norm(pts - eye_centers[ei], axis=1))) / face_width
                    if d < best[0]:
                        best = (d, ei, hi)
            self.last_distance_norm = best[0]
            near = best[0] <= c["near_norm_distance"]
            if near:
                _, ei, hi = best
                centroid = hands[hi].mean(axis=0)
                rel = (centroid - eye_centers[ei]) / face_width
        else:
            self.last_distance_norm = None
        if near:
            self._last_near = t
            if not self._active:
                self._active, self._t0, self._track, self._counted = True, t, [], False
            self._track.append((float(rel[0]), float(rel[1])))
            if not self._counted and t - self._t0 >= c["min_duration_s"] and \
                    t - self._last_event_t >= c["refractory_s"]:
                rev = count_reversals(np.asarray(self._track), c["min_amplitude_norm"])
                if rev >= c["min_reversals"]:
                    self._counted = True
                    self._last_event_t = t
                    return RubEvent(self._t0, t, rev)
        elif self._active and t - self._last_near > c["gap_tolerance_s"]:
            self._active = False
            self._track = []
        return None
