"""FatigueMonitor: hardware-independent orchestration.

Input : one `Observation` per frame (landmark-derived features, optional laptop instruction number).
Detectors (blink/closure, PERCLOS, yawn, nod, eye rub, head pose / station, body presence) feed THREE
independent algorithms - task_fatigue (1), visual_fatigue (2), attention_fatigue (3) - and the combiner
fatigue_fusion produces the only reported result: the final mental-fatigue level 0 / 1 / 2.
Depth/robot distance is NOT an input, and nothing here controls the robot or talks to a decision layer.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

from .attention import (LABEL_UNKNOWN, LABEL_OUTSIDE, AttentionClassifier, AttentionResult, CalibrationCapture, STATION_LABELS)

LABEL_TO_KEY = {v: k for k, v in STATION_LABELS.items()}
from .attention_fatigue import (AttentionFatigueEstimator, LABEL_TASK_ENGAGED, TASK_ENGAGED, _TO_TRACKER, normalize_label)
from .work_posture import FaceLossFallback, PostureCapture, PostureCheck, WorkPosture
from .config import INDICATORS
from .events import (BaselineLearner, EyeClosureTracker, EyeRubDetector,
                     HandActivity, NodDetector, PerclosTracker, RollingEventCounter, StationEarNorm, YawnDetector)
from .fatigue_fusion import FatigueFusion
from .scoring import QualityTracker, level_code, microsleep_score
from .task_fatigue import TaskFatigueEstimator
from .visual_fatigue import VisualFatigueEstimator


@dataclass
class Observation:
    """Per-frame features extracted from landmarks (None = unavailable)."""
    t: float
    face_ok: bool = False
    pose_ok: bool = False
    eyes_ok: bool = False
    mouth_ok: bool = False
    ear: Optional[float] = None
    mar: Optional[float] = None
    yaw: Optional[float] = None
    pitch: Optional[float] = None
    eye_centers_px: Optional[np.ndarray] = None
    face_width_px: Optional[float] = None
    hands_px: List[np.ndarray] = field(default_factory=list)
    # body presence (Pose Landmarker); only filled when the pose model ran on this frame
    body_checked: bool = False
    body_present: bool = False
    body_facing: Optional[str] = None      # front | side | back | none
    body_turn: Optional[float] = None
    body_px: Optional[np.ndarray] = None
    # laptop / task information for the task-difficulty algorithm (None = not available)
    instruction: Optional[int] = None      # instruction number currently shown on the laptop
    action_event: Optional[str] = None     # None | 'attempted' | 'completed' | 'corrective'
    # collaboration flag from the cell: False = the worker has left / deactivated the collaboration (STANDBY)
    collab_active: bool = True


@dataclass
class MonitorState:
    t: float
    level: str                      # Low | Medium | High (Unknown only if report_unknown_as_minus_one)
    level_code: int                 # FINAL mental-fatigue level: 0 Low, 1 Medium, 2 High
    combined_score: float
    active_evidence: List[str]
    task: Dict[str, Any]            # output of algorithm 1 (task_fatigue)
    visual: Dict[str, Any]          # output of algorithm 2 (visual_fatigue)
    attention_out: Dict[str, Any]   # output of algorithm 3 (attention_fatigue)
    data_valid: bool                # False while baseline learning / warming up / low data quality
    data_warnings: List[str]
    raw: Dict[str, Optional[float]]  # indicator raw values (display / log)
    r: Dict[str, Optional[float]]    # indicator scores 0..1 (display / log)
    quality: float
    attention: str                  # workstation label of this frame
    memberships: Dict[str, Optional[float]]
    immediate_distraction: bool     # currently outside the working area for >= sustained_threshold_s
    d_outside_s: float
    ear: Optional[float]
    ear_threshold: float
    mar: Optional[float]
    yaw: Optional[float]
    pitch: Optional[float]
    perclos_valid_s: float
    closure_counts: Dict[str, int]
    baseline_progress: float
    baseline_done: bool
    calibration_status: str
    eyes_occluded: bool
    microsleep_count: int = 0          # microsleep-like closures (>= 1 s) in the rolling window
    microsleep_score: float = 0.0      # 1 - exp(-growth * count)
    ear_scale: float = 1.0             # per-pitch EAR normalisation factor applied to this frame
    hands_active: bool = False         # hands moving normally -> eye closures ignored
    presence: str = ""                 # face | body turned away | body facing camera | not detected | face lost
    task_diag: Dict[str, Any] = field(default_factory=dict)      # extra diagnostics of algorithm 1
    fusion_diag: Dict[str, Any] = field(default_factory=dict)    # extra diagnostics of the combiner
    standby: bool = False              # collaboration inactive: nothing is measured
    collab_active: bool = True
    fallback: str = ""                 # face-free fallback decision / reason (only while the face is lost)
    task_engaged: bool = False         # face hidden, assembly posture + hands confirm task engagement (gaze unknown)
    hands_in_zone: Optional[int] = None   # hands inside the calibrated assembly zone (None = torso/posture unavailable)
    posture: str = ""                  # assembly-posture check (calibrated? in posture? hands in zone)
    notices: List[str] = field(default_factory=list)             # non-blocking notices (e.g. nobody detected)
    enabled: Dict[str, bool] = field(default_factory=lambda: {"task": True, "visual": True, "attention": True})
    instant: Dict[str, float] = field(default_factory=dict)      # instantaneous score "now" of each algorithm (before accumulation)


class FatigueMonitor:
    def __init__(self, cfg: Dict[str, Any], attention: Optional[AttentionClassifier] = None) -> None:
        self.cfg = cfg
        self.attention = attention or AttentionClassifier(cfg)
        self.baseline_hold_reason = "waiting for the instruction file (press U) - baseline not started"
        self.baseline_hold = False        # True: do not start the EAR/MAR baseline yet (main.py: waiting for the PowerPoint)
        b = cfg["baseline"]
        self.ear_base = BaselineLearner(b["duration_s"], b["min_valid_s"], b["ear_percentile"])
        self.mar_base = BaselineLearner(b["duration_s"], b["min_valid_s"], b["mar_percentile"])
        pc = cfg["perclos"]
        self.perclos = PerclosTracker(pc["window_s"], pc["min_valid_s"])
        self.closure = EyeClosureTracker(cfg["closure"])
        self.yawn = YawnDetector(cfg["yawn"])
        self.nod = NodDetector(cfg["nod"])
        self.rub = EyeRubDetector(cfg["rub"])
        self.yawn_counter = RollingEventCounter(cfg["yawn"]["window_s"])
        self.nod_counter = RollingEventCounter(cfg["nod"]["window_s"])
        self.rub_counter = RollingEventCounter(cfg["rub"]["window_s"])
        # three independent algorithms + the combiner (the only source of the reported level)
        self.task = TaskFatigueEstimator(cfg["task_fatigue"])
        self.visual = VisualFatigueEstimator(cfg["visual_fatigue"])
        self.attn_alg = AttentionFatigueEstimator(cfg["attention_fatigue"])
        self.fusion = FatigueFusion(cfg["fatigue_fusion"])
        q = cfg["quality"]
        self.quality = QualityTracker(q["window_s"], q["max_gap_s"])
        self.ear_threshold = pc["ear_initial_threshold"]
        self._eye_closed = False
        self._raw: Dict[str, Optional[float]] = {k: None for k in INDICATORS}
        self._r: Dict[str, Optional[float]] = {k: None for k in INDICATORS}
        self._fus_out = self.fusion.output()
        self._task_out = self.task.output()
        self._vis_out = self.visual.output()
        self._att_out = self.attn_alg.output()
        self._data_valid, self._warnings = False, ["starting"]
        self._perclos_valid_s = 0.0
        self.station_norm = StationEarNorm(cfg["eye_norm"], self._ear_seeds)
        self.hand_activity = HandActivity(cfg["hand_activity"])
        self.enabled: Dict[str, bool] = {"task": True, "visual": True, "attention": True}
        self._off: Dict[str, Dict[str, Any]] = {}
        self._tap: List[Dict[str, Any]] = []                  # detected visual events, drained by pop_events()
        self._hands_zone: Optional[int] = None
        self._body_away_since: Optional[float] = None
        self._away_now = False
        self.posture = WorkPosture(cfg["work_posture"], cfg["body_presence"]["min_visibility"])
        self.fallback = FaceLossFallback(cfg["work_posture"])
        self._pcap: Optional[PostureCapture] = None
        self._standby = False
        self._last_instr: Optional[int] = None
        self._fb_reason = ""
        self._posture_txt = ""
        self._engaged = False
        self._nobody_since: Optional[float] = None
        self._presence = ""
        self._calib_msg_seen = ("", -1e9)
        self._ear_scale = 1.0
        self._perclos_grace_until = -1e9
        self._perclos_grace_min = 1.0
        self._capture: Optional[CalibrationCapture] = None
        self._calib_msg = ""

    @property
    def distraction(self):
        """The outside-working-area event tracker of the attention algorithm."""
        return self.attn_alg.tracker

    # -- workstation calibration (keys 1/2/3/R) ------------------------------
    def begin_calibration(self, station: str, t: float) -> None:
        a = self.cfg["attention"]
        self._capture = CalibrationCapture(station, t, a["capture_s"], a["capture_min_samples"],
                                           a.get("capture_timeout_s"))
        self._calib_msg = f"Capturing {STATION_LABELS[station]}... keep looking at it"

    def pop_events(self) -> List[Dict[str, Any]]:
        """Drain the detector events produced since the last call (for data collection; read-only tap).
        Each item: {"type", "start", "end", "detail"}; type in blink | long_blink | prolonged_closure |
        microsleep_like_closure | yawn | nod | eye_rub. Nothing here changes any algorithm or threshold."""
        out, self._tap = self._tap, []
        return out

    def set_algorithm_enabled(self, name: str, on: bool, t: float) -> None:
        """Switch one of the three algorithms on / off at run time.

        Off: the algorithm is not evaluated, its output is frozen at zero and the combiner ignores it (its weight is
        removed and renormalised, its level cannot raise the final level). Switching it ON starts it from a clean state.
        Switching ATTENTION off removes the look-away (2 s / 3 s) safety rule from the final level: the window says so."""
        if name not in self.enabled:
            raise KeyError(name)
        if bool(on) == self.enabled[name]:
            return
        self.enabled[name] = bool(on)
        est = {"task": self.task, "visual": self.visual, "attention": self.attn_alg}[name]
        est.reset()
        if name == "attention":
            self.fallback.reset()
        self._off[name] = dict(est.output())
        if name == "attention":
            self._off[name]["attention_state"] = "OFF"
        self._calib_msg = f"Algorithm {name}: {'ON' if on else 'OFF'}"

    @property
    def posture_capture_active(self) -> bool:
        """True while key 4 is capturing: the vision layer must run the pose model on every frame."""
        return self._pcap is not None

    def begin_posture_calibration(self, t: float) -> None:
        """Key 4: capture the assembly work posture (face-free: shoulders + hands only)."""
        self._pcap = PostureCapture(t, self.cfg["work_posture"])
        self._calib_msg = "Capturing assembly work posture... lean and work as you do when writing, hands in view"

    def reset_calibration(self) -> None:
        self.attention.reset()
        self.posture.reset()
        self.fallback.reset()
        self._pcap = None
        self._capture = None
        self._calib_msg = "Workstation calibration reset"

    def _ear_seeds(self) -> Dict[int, float]:
        """EAR seen at station calibration, keyed by the head-pitch bin of that station."""
        out: Dict[int, List[float]] = {}
        for name, ear in self.attention.ear_refs.items():
            if name in self.attention.refs:
                out.setdefault(self.station_norm.bin_of(self.attention.refs[name][1]), []).append(ear)
        return {b: float(np.mean(v)) for b, v in out.items()}

    def reset_fatigue(self, t: float) -> None:
        """Manual reset (key F): fatigue level back to Low.

        Clears the three algorithms' cumulative states, the combiner and every rolling window (PERCLOS,
        closures incl. microsleep-like ones, yawns, nods, rubs, disengagement). KEEPS the workstation calibration
        and the personal EAR/MAR baselines. If the worker is still showing fatigue signs the level
        will rise again. For PERCLOS, 1 s of valid data is enough during the first min_valid_s
        seconds after the reset (perclos.grace_min_valid_s, instead of waiting for a full window).
        """
        cfg = self.cfg
        pc = cfg["perclos"]
        self.perclos = PerclosTracker(pc["window_s"], pc["min_valid_s"])
        self._perclos_grace_until = t + pc["min_valid_s"]
        self._perclos_grace_min = 1.0                     # manual reset: Low again within ~2 s
        self.hand_activity.reset()
        self._body_away_since = None
        self.fallback.reset()
        self.closure.reset_events()
        self.yawn_counter = RollingEventCounter(cfg["yawn"]["window_s"])
        self.nod_counter = RollingEventCounter(cfg["nod"]["window_s"])
        self.rub_counter = RollingEventCounter(cfg["rub"]["window_s"])
        self.task.reset()                             # keeps the worker's calibrated normal durations
        self.visual.reset()
        self.attn_alg.reset()
        self.fusion.reset()
        self._fus_out = self.fusion.output()
        self._calib_msg = "Fatigue reset to Low (calibration and baselines kept)"

    def reset_baseline(self, t: float) -> None:
        """Manual baseline reset (key B): relearn the personal EAR/MAR baselines from scratch (~60 s).

        Also clears the per-station eye references learned online and everything reset_fatigue clears.
        The workstation calibration (yaw/pitch ranges and the EAR seen at calibration) is kept.
        The level is Unknown while learning; look at your normal working area with eyes open and mouth closed.
        """
        b = self.cfg["baseline"]
        self.ear_base = BaselineLearner(b["duration_s"], b["min_valid_s"], b["ear_percentile"])
        self.mar_base = BaselineLearner(b["duration_s"], b["min_valid_s"], b["mar_percentile"])
        self.ear_threshold = self.cfg["perclos"]["ear_initial_threshold"]
        self.yawn = YawnDetector(self.cfg["yawn"])
        self.station_norm.reset()
        self._eye_closed = False
        self._ear_scale = 1.0
        self.reset_fatigue(t)
        self._calib_msg = "Baseline reset: learning EAR/MAR for ~60 s, keep eyes open, mouth closed"

    # -- helpers ---------------------------------------------------------------
    def _update_ear_threshold(self) -> None:
        pc = self.cfg["perclos"]
        if self.ear_base.done:
            thr = self.ear_base.value * pc["ear_threshold_ratio"]
            self.ear_threshold = float(np.clip(thr, pc["ear_threshold_min"], pc["ear_threshold_max"]))

    # -- main update -------------------------------------------------------------
    def update(self, obs: Observation) -> MonitorState:
        t = obs.t
        cfg = self.cfg
        if not obs.collab_active:
            return self._standby_update(obs)
        if self._standby:
            self._leave_standby(t)
        # 1. hand near eye -> eye samples unreliable (rubbing is NOT a closure)
        rub_ev = self.rub.update(t, obs.eye_centers_px if obs.face_ok else None,
                                 obs.face_width_px, obs.hands_px if obs.face_ok else [])
        if rub_ev is not None:
            self.rub_counter.add(t)
            self._tap.append({"type": "eye_rub", "start": rub_ev.start, "end": rub_ev.end,
                              "detail": f"reversals={rub_ev.reversals}"})
        occluded = self.rub.occluded(t)

        eye_valid = bool(obs.face_ok and obs.eyes_ok and obs.ear is not None
                         and np.isfinite(obs.ear) and not occluded)
        mouth_valid = bool(obs.face_ok and obs.mouth_ok and obs.mar is not None and np.isfinite(obs.mar))
        pose_valid = bool(obs.face_ok and obs.pose_ok and obs.yaw is not None and obs.pitch is not None)
        # 2. attention label first (head pose only): the eye reference depends on the station
        att = self.attention.classify(obs.yaw if pose_valid else None, obs.pitch if pose_valid else None)
        since = self._body_presence(t, obs, pose_valid)
        att, since, engaged, unverified_ok = self._face_free_attention(t, obs, pose_valid, att, since)
        # A confirmed "face turned away" frame is a KNOWN state (worker present, looking away), so it does not
        # lower data quality; otherwise every look-away would push Q below the limit and show Unknown.
        self.quality.add(t, (eye_valid and pose_valid) or since is not None or engaged)
        if self.enabled["attention"]:
            att_out = self.attn_alg.update_label(t, att.label, since=since,
                                                 unverified_ok=unverified_ok)   # algorithm 3 (every frame)
        else:
            att_out = self._off["attention"]

        # 3. personal baselines (first ~60 s of valid data); not started while baseline_hold (waiting for the slides)
        just_done = False
        if not self.baseline_hold:
            just_done = self.ear_base.update(t, obs.ear if eye_valid else None)
            if self.mar_base.update(t, obs.mar if mouth_valid else None):
                self.yawn.set_baseline(self.mar_base.value, self.mar_base.sigma or 0.0)
        if just_done:
            self._update_ear_threshold()
            self.closure.reset_events()          # discard closures judged with the provisional threshold
            self.station_norm.reset()
            pc = cfg["perclos"]                  # samples stored during learning used the provisional scale
            self.perclos = PerclosTracker(pc["window_s"], pc["min_valid_s"])
            self._perclos_grace_until = t + pc["min_valid_s"]
            self._perclos_grace_min = pc["grace_min_valid_s"]   # after baseline: wait for enough data

        # 4. eye state with hysteresis, PERCLOS storage and closure episodes.
        #    EAR is first rescaled with the station's own open-eye reference (looking down lowers EAR).
        pitch = obs.pitch if pose_valid else None
        self.station_norm.update(t, pitch, obs.ear, eye_valid)
        self._ear_scale = self.station_norm.scale(pitch, self.ear_base.value) if self.ear_base.done else 1.0
        ear_eff = obs.ear * self._ear_scale if (eye_valid and obs.ear is not None) else obs.ear
        active = self.hand_activity.update(t, obs.hands_px if obs.face_ok else [], obs.face_width_px)
        thr_open = self.ear_threshold * cfg["perclos"]["reopen_hysteresis"]
        if eye_valid:
            if active:                                   # hands moving normally: awake, ignore closures
                self._eye_closed = False
            else:
                self._eye_closed = ear_eff < (thr_open if self._eye_closed else self.ear_threshold)
        ear_store = max(ear_eff, thr_open) if (active and eye_valid and ear_eff is not None) else ear_eff
        self.perclos.update(t, ear_store, eye_valid)
        cev = self.closure.update(t, eye_valid, self._eye_closed, occluded=occluded or (active and eye_valid))
        if cev is not None:
            kind = {"blink": "blink", "long_blink": "long_blink", "prolonged": "prolonged_closure",
                    "microsleep": "microsleep_like_closure"}.get(cev.category, cev.category)
            self._tap.append({"type": kind, "start": cev.start, "end": cev.end,
                              "detail": f"ear_threshold={self.ear_threshold:.3f}"})

        # assembly work posture capture (key 4): shoulders + hands only, the face is not needed
        if self._pcap is not None:
            self._pcap.add(self.posture.features(obs.body_px, obs.hands_px), obs.body_facing if obs.body_checked else None)
            if self._pcap.done(t):
                ref, msg = self._pcap.result()
                if ref is not None:
                    self.posture.ref = ref
                self._calib_msg = ("Assembly posture: " + msg) if ref is not None else ("Assembly posture FAILED: " + msg)
                self._pcap = None
            else:
                self._calib_msg = (f"Capturing assembly posture: {self._pcap.progress(t) * 100:.0f}% "
                                   f"({self._pcap.summary()})")

        # workstation calibration capture (also records the open-eye EAR at that station)
        if self._capture is not None:
            if pose_valid:
                self._capture.add(obs.yaw, obs.pitch, t, obs.ear if eye_valid else None)
            name = STATION_LABELS[self._capture.station]
            if self._capture.done(t):
                res = self._capture.result()
                if res is None:
                    self._calib_msg = f"{name}: too few valid samples ({self._capture.summary(t)}); face/pose lost?"
                else:
                    self.attention.set_reference(self._capture.station, *res, ear_ref=self._capture.ear_ref())
                    self._calib_msg = f"{name} calibrated (yaw {res[0]:.1f}, pitch {res[1]:.1f}, {self._capture.summary(t)})"
                self._capture = None
            else:
                self._calib_msg = f"Capturing {name}: {self._capture.progress() * 100:.0f}% ({len(self._capture.yaws)} samples)"

        # 5. yawn
        yev = self.yawn.update(t, obs.mar if mouth_valid else None)
        if yev is not None:
            self.yawn_counter.add(t)
            self._tap.append({"type": "yawn", "start": yev.start, "end": yev.end,
                              "detail": f"peak_mar={yev.peak_mar:.3f}"})

        # 6. nod
        best_mem = max([m for m in att.memberships.values() if m is not None], default=0.0)
        for ev in self.nod.update(t, obs.pitch if pose_valid else None,
                                  obs.yaw if pose_valid else None, best_mem, self.closure):
            self.nod_counter.add(ev.end, ev.weight)
            self._tap.append({"type": "nod", "start": ev.start, "end": ev.end,
                              "detail": f"drop_deg={ev.peak_drop_deg:.1f};closure_confirmed={ev.confirmed_by_closure}"})

        # 7. algorithm 1 (task), algorithm 2 (visual), combiner - every frame
        task_label = "ASSEMBLY" if engaged else att.label       # engaged = working at the assembly area (gaze unknown)
        task_out = (self.task.update(t, obs.instruction, task_label, obs.action_event)
                    if self.enabled["task"] else self._off["task"])
        self._evaluate(t, att_out, task_out)
        return self._state(t, obs, att)

    def _face_free_attention(self, t: float, obs: Observation, pose_valid: bool, att: AttentionResult,
                             since: Optional[float]):
        """Apply the face-free rules. Returns (att, since, engaged, unverified_ok).

        Priority: 1 face visible -> normal path (outside stays outside); 2 face lost + body turned away -> OUTSIDE;
        3 face lost + posture/hands/context -> TASK_ENGAGED (only UNKNOWN frames can become this); 4 else UNKNOWN."""
        fb = self.fallback
        if obs.instruction is not None and obs.instruction != self._last_instr:
            fb.note_step_activity(t)
        if obs.instruction is not None:
            self._last_instr = obs.instruction
        if obs.action_event:
            fb.note_step_activity(t)
        feat = self.posture.features(obs.body_px, obs.hands_px)
        moving = self.posture.update_motion(t, feat) if not pose_valid else False
        chk = self.posture.check(feat)
        self._hands_zone = chk.hands_in_zone if (feat is not None and self.posture.calibrated) else None
        self._posture_txt = (f"posture {'OK' if chk.in_posture else 'no'}, hands in zone {chk.hands_in_zone}"
                             + (f" ({chk.reason})" if chk.reason else "")) if self.posture.calibrated \
            else "posture not calibrated (press 4)"
        self._engaged, self._fb_reason = False, ""
        if pose_valid:                                                     # priority 1 / 5
            fb.note_face(t, normalize_label(att.label))
            return att, since, False, False
        complete = self.attention.is_complete
        if (since is not None or self._away_now) and complete:             # priority 2: body turned away
            fb.note_outside(t)
            if since is not None:
                att = AttentionResult(LABEL_OUTSIDE, {}, "face turned away (body detected)")
            self._fb_reason = "body turned away: outside wins, hands ignored"
            return att, since, False, False
        if not complete:
            return att, since, False, False
        body_front = bool(obs.body_checked and obs.body_present and obs.body_facing == "front")
        label, reason = fb.update(t, body_front, chk, moving)              # priority 3 / 4
        self._fb_reason = reason
        if label == TASK_ENGAGED:
            self._engaged = True
            return AttentionResult(LABEL_TASK_ENGAGED, {}, reason), since, True, False
        if label is not None and label != "UNKNOWN":                       # 0..hold_s: last valid state
            return AttentionResult(_TO_TRACKER[label], {}, reason), since, False, False
        return att, since, False, True                                     # UNKNOWN (present, attention unverified)

    def _standby_update(self, obs: Observation) -> MonitorState:
        """Collaboration inactive: the worker has left / deactivated the cell. Nothing is measured or accumulated."""
        t = obs.t
        if not self._standby:
            self._standby = True
            self.attn_alg.suspend()
            self.task.pause(t)
            self.fallback.reset()
            self.posture.reset_motion()
            self.hand_activity.reset()
            self._body_away_since = None
            self._eye_closed = False
            self.closure.update(t, False, False, occluded=True)
        self._presence = "standby (collaboration inactive)"
        self._fb_reason, self._engaged, self._posture_txt = "", False, ""
        reasons = ["collaboration inactive (standby)"]
        self._data_valid, self._warnings = False, reasons
        self._att_out = self.attn_alg.output()
        self._fus_out = self.fusion.update(t, self._task_out, self._vis_out, self._att_out, data_valid=False,
                                           warnings=reasons)
        return self._state(t, obs, AttentionResult(LABEL_UNKNOWN, {}, "standby"))

    def _leave_standby(self, t: float) -> None:
        self._standby = False
        self.attn_alg.resume(t)
        self.task.resume(t)
        self.fallback.reset()
        self.quality = QualityTracker(self.cfg["quality"]["window_s"], self.cfg["quality"]["max_gap_s"])
        self._calib_msg = "Collaboration active again: monitoring resumed"

    def _body_presence(self, t: float, obs: Observation, pose_valid: bool) -> Optional[float]:
        """Return the (back-dated) start time of a confirmed 'face turned away' episode, else None."""
        bp = self.cfg["body_presence"]
        away = bool(bp["enabled"] and not pose_valid and obs.body_checked and obs.body_present
                    and obs.body_facing in ("side", "back"))
        if away and bp["suppress_when_hands_active"] and self.hand_activity.active(t):
            away = False
        if pose_valid:
            self._presence = "face"
        elif not obs.body_checked:
            self._presence = "face lost (body check off)"
        elif away:
            self._presence = f"body, face turned away ({obs.body_facing})"
        elif obs.body_present:
            self._presence = "body facing camera, face not found"
        else:
            self._presence = "not detected"
        self._away_now = away
        if not away:
            self._body_away_since = None
            return None
        if self._body_away_since is None:
            self._body_away_since = t
        return self._body_away_since if t - self._body_away_since >= bp["confirm_s"] else None

    def _evaluate(self, t: float, att_out: Dict[str, Any], task_out: Dict[str, Any]) -> None:
        cfg = self.cfg
        en = self.enabled
        ready = (not en["visual"]) or (self.ear_base.done and self.mar_base.done)
        pv, self._perclos_valid_s = (
            self.perclos.value(self.ear_threshold,
                               self._perclos_grace_min if t < self._perclos_grace_until else None)
            if self.ear_base.done else (None, 0.0))
        closure_w = self.closure.prolonged_weighted(t) if self.ear_base.done else 0.0
        n_ms = self.closure.microsleep_count(t) if self.ear_base.done else 0
        yawns = self.yawn_counter.count(t) if self.mar_base.done else 0.0
        nods, rubs = self.nod_counter.count(t), self.rub_counter.count(t)
        vis_out = (self.visual.update(t, microsleep_count=n_ms, prolonged_closure_weighted=closure_w,
                                      yawn_count=yawns, nod_count=nods, eye_rub_count=rubs,
                                      perclos=pv, valid=ready) if en["visual"] else self._off["visual"])
        reasons: List[str] = []
        if not en["visual"]:
            pass                                            # no EAR/MAR baseline needed
        elif self.baseline_hold:
            reasons.append(self.baseline_hold_reason)
        elif not ready:
            reasons.append(f"learning personal baseline ({self.ear_base.progress(t) * 100:.0f}%)")
        q = self.quality
        if q.observed_s() < cfg["quality"]["min_observed_s"]:
            reasons.append("warming up")
        elif q.value() < cfg["quality"]["min_ratio"]:
            reasons.append(f"data quality Q={q.value():.2f} < {cfg['quality']['min_ratio']}")
        if (en["attention"] or en["task"]) and not self.attention.is_complete:
            reasons.append("workstation calibration incomplete (press 1/2/3)")
        self._data_valid, self._warnings = not reasons, reasons
        self._fus_out = self.fusion.update(t, task_out, vis_out, att_out, data_valid=not reasons, warnings=reasons,
                                           enabled=en)
        self._task_out, self._vis_out, self._att_out = task_out, vis_out, att_out

        sc = self.visual.diagnostics().get("scores", {})
        ad = self.attn_alg.diagnostics()
        att_ok = self.attention.is_complete
        self._raw = {
            "perclos": pv if en["visual"] else None, "closure": closure_w if (self.ear_base.done and en["visual"]) else None,
            "brief_disengagement": float(ad["brief_events_in_window"]) if att_ok else None,
            "sustained_disengagement": ad["current_outside_s"] if att_ok else None,
            "yawn": yawns if self.mar_base.done else None, "nod": nods, "rub": rubs,
        }
        self._r = {
            "perclos": sc.get("perclos") if (pv is not None and en["visual"]) else None,
            "closure": sc.get("closure") if (self.ear_base.done and en["visual"]) else None,
            "brief_disengagement": ad["brief_score"] if att_ok else None,
            "sustained_disengagement": ad["sustained_score"] if att_ok else None,
            "yawn": sc.get("yawn") if self.mar_base.done else None,
            "nod": sc.get("nod"), "rub": sc.get("eye_rub"),
        }

    def _instants(self) -> Dict[str, float]:
        """Instantaneous ('now') score of each enabled algorithm, before it is accumulated over time."""
        out: Dict[str, float] = {}
        for name, est in (("task", self.task), ("visual", self.visual), ("attention", self.attn_alg)):
            if self.enabled[name]:
                d = est.diagnostics()
                v = d.get("instant_score")
                out[name] = float(v) if v is not None else 0.0
        return out

    def _notices(self, t: float) -> List[str]:
        """Non-blocking notices. 'Nobody detected while collaboration is active' is reported, never hidden."""
        out: List[str] = []
        nobody = (not self._standby and self._presence == "not detected")
        if nobody:
            if self._nobody_since is None:
                self._nobody_since = t
            if t - self._nobody_since >= self.cfg["collaboration"]["nobody_notice_s"]:
                out.append("collaboration active but nobody detected")
        else:
            self._nobody_since = None
        if self._att_out.get("attention_state") == "ATTENTION_UNVERIFIED":
            out.append("attention unverified (face not seen, no task-engagement evidence)")
        return out

    def _state(self, t: float, obs: Observation, att) -> MonitorState:
        f = self._fus_out
        if self._calib_msg != self._calib_msg_seen[0]:              # show status messages for 8 s only
            self._calib_msg_seen = (self._calib_msg, t)
        msg = self._calib_msg if (self._capture is not None or t - self._calib_msg_seen[1] <= 8.0) else ""
        n_ms = self.closure.microsleep_count(t) if self.ear_base.done else 0
        lvl = f["final_mental_fatigue_level"]
        return MonitorState(
            t=t, level=f["final_mental_fatigue_label"], level_code=level_code(f["final_mental_fatigue_label"]),
            combined_score=f["combined_score"], active_evidence=list(f["active_evidence"]),
            task=dict(self._task_out), visual=dict(self._vis_out), attention_out=dict(self._att_out),
            data_valid=self._data_valid, data_warnings=list(self._warnings),
            raw=dict(self._raw), r=dict(self._r),
            quality=self.quality.value(), attention=att.label, memberships=att.memberships,
            immediate_distraction=self.distraction.immediate(t),
            d_outside_s=self.distraction.d_outside(t),
            ear=obs.ear, ear_threshold=self.ear_threshold, mar=obs.mar, yaw=obs.yaw, pitch=obs.pitch,
            perclos_valid_s=self._perclos_valid_s, closure_counts=dict(self.closure.counts),
            baseline_progress=min(self.ear_base.progress(t), self.mar_base.progress(t)),
            baseline_done=self.ear_base.done and self.mar_base.done,
            calibration_status=msg, eyes_occluded=self.rub.occluded(t),
            microsleep_count=n_ms,
            microsleep_score=microsleep_score(n_ms, self.cfg["visual_fatigue"]["microsleep_growth"]),
            ear_scale=self._ear_scale, hands_active=self.hand_activity.active(t),
            presence=self._presence, standby=self._standby, collab_active=not self._standby,
            enabled=dict(self.enabled), instant=self._instants(),
            fallback=self._fb_reason, task_engaged=self._engaged, hands_in_zone=self._hands_zone, posture=self._posture_txt,
            notices=self._notices(t), task_diag=self.task.diagnostics(), fusion_diag=self.fusion.diagnostics(),
        )
