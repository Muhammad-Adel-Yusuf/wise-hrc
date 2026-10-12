"""Configuration defaults, loading and validation.

ALL numeric thresholds/weights of the system live here (and in config.json,
which is generated from these defaults). Values are LITERATURE-INFORMED
PROTOTYPE PARAMETERS and require experimental validation. Nothing here is a
medical diagnostic criterion.
"""
from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from typing import Any, Dict, Optional

# Indicator rows shown on screen / logged (their scores live in the three algorithms).
INDICATORS = ("perclos", "closure", "brief_disengagement", "sustained_disengagement", "yawn", "nod", "rub")
STATIONS = ("assembly", "laptop", "storage")

FACE_MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/face_landmarker/"
                  "face_landmarker/float16/latest/face_landmarker.task")
HAND_MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/hand_landmarker/"
                  "hand_landmarker/float16/latest/hand_landmarker.task")
POSE_MODEL_URL = ("https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
                  "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task")
MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/holistic_landmarker/"
    "holistic_landmarker/float16/latest/holistic_landmarker.task"
)

DEFAULT_CONFIG: Dict[str, Any] = {
    "_notice": (
        "Literature-informed PROTOTYPE parameters; they require experimental "
        "validation in the target workplace. Vision-based risk estimate, not a "
        "medical diagnosis. All values in task_fatigue, visual_fatigue, attention_fatigue and "
        "fatigue_fusion are INITIAL ENGINEERING PARAMETERS requiring experimental validation."
    ),
    "camera": {
        "color_width": 1280, "color_height": 720,
        "depth_width": 640, "depth_height": 576,
        "fps": 30,
        "color_formats": ["RGB", "BGR", "MJPG", "YUYV", "NV12"],
        "use_depth": True,                  # False = colour stream only (faster; nothing in the study uses depth)
        "prefer_hw_d2c": True,               # alignment in the camera (faster); falls back to software
        "enable_frame_sync": True,
        "frame_timeout_ms": 200,
        "depth_min_mm": 200.0, "depth_max_mm": 4000.0,
    },
    "vision": {
        # "face_hand" = Face Landmarker + Hand Landmarker (default, stable);
        # "holistic"  = Holistic Landmarker (can abort the whole process with
        #               "packet.cc Check failed: holder_ != nullptr" on some builds).
        "backend": "face_hand",
        "face_model_path": "models/face_landmarker.task",
        "face_model_url": FACE_MODEL_URL,
        "hand_model_path": "models/hand_landmarker.task",
        "hand_model_url": HAND_MODEL_URL,
        "pose_model_path": "models/pose_landmarker_lite.task",   # body presence when the face is lost
        "pose_model_url": POSE_MODEL_URL,
        "max_consecutive_failures": 30,      # then the landmarker is re-created
        "model_path": "models/holistic_landmarker.task",
        "model_url": MODEL_URL,
        "auto_download": True,
        "inference_width": 640,
        "min_face_detection_confidence": 0.15,
        "min_face_landmarks_confidence": 0.15,
        "min_pose_detection_confidence": 0.5,
        "min_pose_landmarks_confidence": 0.5,
        "min_hand_landmarks_confidence": 0.5,
        "max_eye_yaw_deg": 55.0, "max_mouth_yaw_deg": 60.0,
        "eye_pitch_range_deg": [-55.0, 45.0],
        "max_reprojection_error_px": 8.0,
        "ear_valid_range": [0.05, 0.60],
        "mar_valid_range": [0.0, 2.0],
        "depth_roi_radius_px": 8,
        "depth_min_valid_fraction": 0.2,
        "depth_min_points": 2,          # face points (of 6) that must give a valid depth
        "depth_max_spread_mm": 80.0,    # points further than this from the median are rejected
        "depth_hold_s": 0.5,            # keep the last good value this long across dropouts
    },
    "baseline": {
        "duration_s": 60.0, "min_valid_s": 20.0,
        "ear_percentile": 75.0, "mar_percentile": 30.0,
    },
    "attention": {
        "calibration_file": "calibration.json",
        "capture_s": 5.0,
        "capture_min_samples": 15,      # valid samples needed; the capture itself lasts capture_s of VALID time
        "capture_timeout_s": 15.0,      # give up if capture_s of valid data was not reached in this time
        # Working area = convex hull of the calibrated station regions, grown by this margin (deg).
        "working_area_margin_deg": 5.0,
        "membership_threshold": 0.35,
        # Per-station range learned from the calibration spread (std):
        #   core half-width  = clip(k_core * std, min_core_deg, max_core_deg)   -> membership 1
        #   fade  half-width = clip(fade_k * std, min_fade_deg, max_fade_deg)   -> membership 1 -> 0
        # k_core = 2.5 is a little wider than the +-2 std ranges suggested by the calibration printout.
        "ranges": {
            "k_core": 2.5, "fade_k": 1.0,
            "min_core_deg": {"yaw": 5.0, "pitch": 4.0},
            "max_core_deg": {"yaw": 25.0, "pitch": 10.0},
            "min_fade_deg": 3.0, "max_fade_deg": 10.0,
        },
        # When two station regions overlap, keep the previous station unless another one is
        # closer by more than this many normalised units (1.0 = core edge).
        "switch_margin": 0.25,
        # Optional manual override of a station's core half-width in degrees, e.g.
        #   "overrides": {"assembly": {"yaw_core_deg": 12.0, "pitch_core_deg": 10.0}}
        # Anything not listed keeps the value learned from the calibration spread.
        "overrides": {},
        "stations": {
            "assembly": {"theta_min": 8.0, "theta_max": 25.0, "phi_min": 8.0, "phi_max": 20.0},
            "laptop":   {"theta_min": 8.0, "theta_max": 25.0, "phi_min": 8.0, "phi_max": 20.0},
            "storage":  {"theta_min": 8.0, "theta_max": 25.0, "phi_min": 8.0, "phi_max": 20.0},
        },
    },
    # Per-station eye-openness normalisation. Looking down at the bench lowers the EAR (the upper lid follows
    # the gaze and the camera sees the eye from above) without the eyes being closed. Each station therefore
    # gets its own open-eye EAR reference and the EAR is rescaled to the global baseline before any threshold.
    "eye_norm": {
        "enabled": True,
        "window_s": 120.0, "min_valid_s": 15.0, "percentile": 75.0,
        "scale_min": 0.6, "scale_max": 1.8,
        # The reference is kept per head-PITCH bin (so an unintentional glance down at any object is
        # compensated, not only the calibrated stations). Stations seed the bins with the EAR seen at calibration.
        "pitch_bin_deg": 10.0, "neighbor_bins": 2,
    },
    # Hand activity: while a hand is moving normally (assembly work) the worker is clearly awake, so eye
    # closures are not counted (no closure/microsleep-like episode, PERCLOS samples are never 'closed').
    # Body presence (MediaPipe Pose Landmarker). When the FACE is lost but the BODY is still seen and turned
    # away from the camera (side or back), the worker is present and looking away -> "Outside working area"
    # instead of "Unknown". A body that still faces the camera (e.g. looking steeply down at the bench)
    # stays "Unknown". No yaw/pitch is taken from the body. INITIAL ENGINEERING PARAMETERS (not validated).
    "body_presence": {
        "enabled": True,
        "run_only_when_face_lost": True,   # run the pose model only on frames without a face (saves CPU)
        "min_visibility": 0.5,             # landmark visibility needed to count a point as seen
        "nose_visible_min": 0.5,           # nose visibility below this = back of the head to the camera
        "ear_asymmetry": 0.5,              # |vis(left ear) - vis(right ear)| >= this = head turned sideways
        "turn_ratio": 0.35,                # |nose x - shoulder centre x| / shoulder width >= this = sideways
        "confirm_s": 0.8,                  # turned-away body must be seen this long (event is back-dated)
        "suppress_when_hands_active": False,   # True: moving hands veto the turned-away decision
    },
    # Algorithm 4 - PHYSICAL fatigue (standalone, LFFR model; code in the sibling folder physical_fatigue_model). It is estimated
    # ALONGSIDE the mental load and is never weighted or added into it. The default rates are INITIAL values until
    # physical_params.json (calibrate.py of the pilot study) exists. Operational 0-1 state, not a physiological measurement.
    "physical": {
        "enabled": True,
        "package_dir": "../physical_fatigue_model",   # folder that contains physical_fatigue/ (relative to this config file)
        "params_file": "physical_params.json",       # lambda_L, lambda_M, lambda_H, mu from calibrate.py (missing = initial defaults)
        "window_s": 30.0,                            # camera-feature window for the workload class
        "update_s": 2.0,                             # fatigue state update interval
        "pose_every_n": 2,                           # run the pose model on every n-th frame
        "min_visibility": 0.5,
        "absent_is_recovery": True,                  # nobody seen in the whole window = recovery (not a physical task)
        "bands": [0.3, 0.6],                         # f < 0.3 Low, < 0.6 Medium, else High (display only; operational, not validated)
        "history_min": 30.0,                         # minutes of fatigue history in the plot
        "estimator": {},                             # overrides of physical_fatigue.estimator.DEFAULT_ESTIMATOR (f_full, rest_*, ...)
        "baseline_file": "physical_baseline.json",   # neutral posture saved by key Y (reloaded at the next start: same camera / seat)
        "neutral_capture_s": 12.0,                   # sit upright, arms relaxed, hands still for this long (key Y)
        "start_borg": None,                          # Borg CR10 before the first task (0-10) -> f0 = B0 / 10; None = ask with key Y / --start-borg
        "baseline": {},                              # overrides of physical_fatigue.baseline.DEFAULT_BASELINE_CFG
    },
    "work_posture": {                    # face-free fallback (work_posture.py): leaning over the assembly area
        "enabled": True,
        "file": "work_posture.json",       # saved by key 4 (assembly work posture calibration)
        "capture_s": 6.0, "capture_min_samples": 40, "capture_min_hand_samples": 20, "capture_timeout_s": 25.0,
        "hold_s": 0.5,                     # 0 .. hold_s after the face is lost: hold the last valid attention state
        "decision_s": 1.0,                 # hold_s .. decision_s: evaluate torso/hands/context; engaged only if confirmed by decision_s
        "max_engaged_s": 30.0,             # engagement is time-limited; afterwards UNKNOWN (-> attention unverified)
        "step_context_s": 60.0,            # a station was looked at / a step event happened within this time
        "min_hands_in_zone": 2,            # hands that must be inside the calibrated hand zone
        "zone_margin_sw": 0.35,            # hand-zone margin, in shoulder widths
        "position_tol_sw": 0.6,            # shoulder-centre shift tolerated from the calibrated posture, in shoulder widths
        "scale_tol": 0.30,                 # |shoulder width / calibrated - 1| tolerated (leaning changes the scale)
        "hand_speed_min_sw_s": 0.3, "hand_hold_s": 1.5,    # hands must have moved (shoulder widths / s) within hand_hold_s
        "condition_gap_s": 0.3,            # a dropout of the engagement conditions shorter than this is ignored
    },
    # Link to the MATLAB POMDP framework (POMDP model/liveHRC.m): the worker state and both upper limbs (in the
    # workstation frame W from workspace_calibration.py) are sent by UDP; the robot itself is virtual (MATLAB).
    "pomdp_bridge": {
        "enabled": True,
        "host": "127.0.0.1", "port": 5005,          # MATLAB liveHRC.m listens on this port
        "send_hz": 15.0,                           # messages per second (MATLAB uses the newest one every 0.5 s)
        "pose_every_n": 2,                         # pose model on every n-th frame (shoulders / elbows / wrists)
        "arm_hold_s": 0.4,                         # each arm point is kept this long after it was last measured; older -> not sent
        "palm_from_wrist_m": 0.08,                 # hand model missed a hand: palm estimated this far beyond the wrist
        "min_visibility": 0.4,                     # pose landmark visibility needed for a 3-D point
        "depth_radius_px": 4,                      # depth median window around each landmark (depth pixels)
        "hand_match_px": 80,                       # hand wrist within this distance of a pose wrist (pixels at 960 wide) = same arm
        "workspace_file": "../POMDP model/workspace_calibration.json",   # written by workspace_calibration.py, read by MATLAB too
        "real_robot": {                            # W on the real robot (robot_camera_calibration.py + workspace_calibration.py)
            "enabled": True,                       # read the real robot pose (read only) and send it to MATLAB
            "poll_hz": 10.0,                       # robot pose reads per second (through the Jetson gateway relay)
            "max_age_s": 0.3,                      # an older robot pose is not sent (MATLAB then uses its virtual robot)
        },
        "calibration": {                           # workspace_calibration.py (real objects: robot = bottle, storage = packet)
            "object_window_px": 250,               # search window (+- px) around a clicked object
            "robot_height_override_m": 0.0,        # > 0: use this robot (bottle) height instead of the measured one
            "object_min_height_m": 0.015,          # pixels this far above the table belong to an object
            "robot_max_radius_m": 0.06,            # robot object (bottle): footprint at most this radius around the click
            "storage_max_radius_m": 0.15,          # storage object (packet / box): footprint at most this radius
            "handover_height_m": 0.20,             # robot holds the tool this high above the table at the handover point
            "handover_offset_m": 0.10,             # automatic handover point: this far outside the assembly area (robot side)
            "storage_above_object_m": 0.05,        # robot tool this far above the top of the storage object
            "collab_height_m": 0.16,               # robot tool height at the collaborative work point
            "area_height_m": 0.25,                 # height of the assembly area (workpiece + hands) for the checks
            "safe_clearance_m": 0.15,              # robot arm at the safe position: at least this far from the assembly area
            "reach_max_m": 0.45,                   # warning if the handover point is farther than this from the assembly area
            "collab_radius_m": 0.35,               # collaborative zone: hand within this distance of the robot tool (MATLAB dCollabMax)
            "assist_radius_m": 0.065,
            "prohibit_radius_m": 0.01,             # prohibited zone: hand within this distance of the robot tool -> protective stop               # assistive zone: hand within this horizontal distance of the handover point
        },
    },
    "collaboration": {
        "start_active": True,              # initial state of the collaboration flag (key K toggles it in the window)
        "nobody_notice_s": 3.0,            # collaboration active but nobody detected this long -> notice
    },
    "hand_activity": {
        "enabled": True,
        "speed_min_fw_s": 0.5,     # hand centroid speed in face-widths per second
        "hold_s": 2.0,             # activity is kept this long after the last fast movement
    },
    "perclos": {
        "window_s": 60.0,
        "grace_min_valid_s": 5.0,      # after a baseline/fatigue reset PERCLOS needs only this much valid data
        "ear_initial_threshold": 0.20,
        "ear_threshold_ratio": 0.67,
        "ear_threshold_min": 0.12, "ear_threshold_max": 0.28,
        "reopen_hysteresis": 1.10,
        "min_valid_s": 18.0,
        "limits": {"low_below": 0.15, "high_at": 0.30},
    },
    "closure": {
        "blink_max_s": 0.20, "long_blink_max_s": 0.50, "prolonged_max_s": 1.00,
        "count_min_s": 0.50, "window_s": 60.0,
        "weight_prolonged": 1.0, "weight_microsleep": 2.0,
        "gap_tolerance_s": 0.15,
        # "Microsleep-like closure" = eye closure >= prolonged_max_s (no EEG, so it is NOT a confirmed microsleep).
    },
    "yawn": {
        "window_s": 300.0,
        "delta_abs": 0.30, "z_score": 6.0, "sigma_floor": 0.02,
        "release_fraction": 0.5,
        "min_duration_s": 1.5, "max_duration_s": 10.0,
        "min_closed_before_s": 0.3, "arm_hold_s": 2.0,
        "plateau_min_fraction": 0.6, "max_cv": 0.35,
        "gap_tolerance_s": 0.5, "refractory_s": 5.0,
    },
    "nod": {
        "window_s": 300.0,
        "baseline_window_s": 20.0, "baseline_lag_s": 3.0, "baseline_min_s": 5.0,
        "excursion_deg": 15.0, "return_fraction": 0.4,
        "min_duration_s": 0.5, "max_duration_s": 3.0,
        "max_yaw_change_deg": 20.0,
        "exclude_workstation_membership": 0.7,
        "confirm_wait_s": 1.0,
        "unconfirmed_weight": 0.5, "confirmed_weight": 1.0,
        "gap_tolerance_s": 0.5, "refractory_s": 2.0,
    },
    "rub": {
        "window_s": 300.0,
        "near_norm_distance": 0.35,
        "min_duration_s": 0.7, "min_reversals": 2,
        "min_amplitude_norm": 0.08,
        "gap_tolerance_s": 0.3, "occlusion_tail_s": 0.4,
        "refractory_s": 2.0, "face_width_height_floor": 0.55,
    },
    # ------------------------------------------------------------------------------------------------
    # THE THREE INDEPENDENT ALGORITHMS AND THE COMBINER.
    # ALL VALUES BELOW ARE INITIAL ENGINEERING PARAMETERS THAT REQUIRE EXPERIMENTAL VALIDATION.
    # ------------------------------------------------------------------------------------------------
    "task_fatigue": {                    # Algorithm 1: task difficulty / instruction behaviour
        "medium_threshold": 0.40, "high_threshold": 0.70,     # score_to_level
        "accumulation_rate": 0.02, "recovery_rate": 0.01,     # per second
        "recovery_score_threshold": 0.10,                     # recovery_signal = 1 when the instantaneous score is below
        "weights": {"backtracking": 0.30, "error": 0.30, "prolonged_viewing": 0.20, "hesitation": 0.20},
        "window_s": 60.0,                                     # rolling window of review / error evidence
        "big_backward_steps": 2,                              # backward trips longer than this are counted as big jumps (diagnostic)
        "review_min_dwell_s": 2.0,                            # a backward trip counts once the worker stays this long on its end
        "error_min_backward_steps": 1,                        # smallest backward trip that can be a suspected error (rework)
        "error_unit": 1.0,                                    # units of error evidence of one suspected error
        "viewed_min_s": 2.0,                                  # laptop time after which an instruction counts as viewed
        "attempt_min_assembly_s": 3.0,                        # assembly time after which the step counts as attempted
        "backtracking_full_units": 4.0,                       # units in the window for backtracking score 1.0
        "unviewed_backward_unit": 0.5, "repeated_return_unit": 0.5,
        "error_full_units": 2.0, "corrective_unit": 0.5,
        "default_normal_duration_s": 30.0,                    # used until the worker is calibrated
        "calibration_min_instructions": 3,                    # completed instructions needed to calibrate (median)
        "duration_factor": 1.5,                               # hesitation/viewing limit = factor * calibrated normal
        "normal_view_s": 10.0, "prolonged_view_full_s": 30.0,
        "hesitation_min_alternations": 3, "hesitation_full_s": 30.0,
    },
    "visual_fatigue": {                  # Algorithm 2: visual fatigue / alertness
        "medium_threshold": 0.40, "high_threshold": 0.70,
        "microsleep_growth": 1.2,                             # microsleep_score = 1 - exp(-growth * count)
        "accumulation_rate": 0.03, "recovery_rate": 0.01,
        "recovery_score_threshold": 0.10,
        "smoothing_tau_s": 4.0,                               # exponential smoothing of the instantaneous score
        "weights": {"microsleep": 0.45, "yawn": 0.20, "nod": 0.15, "eye_rub": 0.10, "perclos": 0.10},
        "normalization": {                                    # counts / values -> [0, 1]
            "perclos": [[0.0, 0.0], [0.15, 0.35], [0.30, 1.0]],
            "closure": [[0.0, 0.0], [1.0, 0.4], [3.0, 1.0]],  # prolonged (0.5 s .. microsleep limit) weighted count
            "yawn":    [[0.0, 0.0], [1.0, 0.4], [2.0, 0.7], [3.0, 1.0]],
            "nod":     [[0.0, 0.0], [1.0, 0.5], [2.0, 1.0]],
            "rub":     [[0.0, 0.0], [1.0, 0.4], [3.0, 1.0]],
        },
        "single_microsleep_forces_high": False,               # explicit opt-in: one isolated event -> High level
        "microsleep_high_score": 0.6,                         # only used when the flag above is true
    },
    "attention_fatigue": {               # Algorithm 3: working-area attention / sustained disengagement
        # high_threshold 0.99 (not 0.70): with the linear ramp 0.5 + 0.5*progress a 0.70 threshold would give High at
        # 2.4 s; 0.99 makes High start at 3 s as specified (2 s -> score 0.5 >= medium 0.40).
        "medium_threshold": 0.40, "high_threshold": 0.99,
        "sustained_threshold_s": 2.0,                         # >= this outside: sustained, score 0.5 (Medium evidence)
        "high_threshold_s": 3.0,                              # >= this outside: score 1.0 (High evidence), capped
        "sustained_growth": 0.0,                              # 0 = linear 0.5 + 0.5*progress; > 0 = exponential (e.g. 1.2)
        "accumulation_rate": 0.03, "recovery_rate": 0.01,
        "recovery_score_threshold": 0.10,
        "brief_window_s": 60.0, "brief_full_count": 5.0,      # brief events in the window for brief score 1.0
        "brief_weight": 0.5,                                  # brief-only evidence is capped at this (moderate)
        "history_window_s": 60.0,                             # window of maximum_outside_duration_s / sustained memory
        "reentry_debounce_s": 0.3,                            # a return shorter than this does not split an event
        "unknown_tolerance_s": 1.0,                           # unknown data longer than this ends an event (not a return)
        "min_record_s": 0.3,                                  # outside episodes shorter than this are noise
        # Collaboration active + face not seen + no task-engagement evidence: the worker is present but attention is
        # unverified. After unverified_after_s the score is floored at unverified_score (Medium, always below High).
        "unverified_enabled": True, "unverified_after_s": 5.0, "unverified_score": 0.5,
    },
    "fatigue_fusion": {                  # Fourth file: final mental-fatigue level (0 Low, 1 Medium, 2 High)
        "weights": {"task": 0.25, "visual": 0.35, "attention": 0.40},   # combined_score weights (renormalised)
        "medium_threshold": 0.40, "high_threshold": 0.70,
        "medium_persistence_s": 5.0, "high_persistence_s": 3.0,
        "recovery_threshold": 0.25, "recovery_persistence_s": 15.0,
        "use_max_rule": True,                                 # final level >= max(component levels)
        "attention_sustained_bypasses_persistence": True,     # see fatigue_fusion.py
        "report_unknown_as_minus_one": False,                 # True: -1 while data is not valid instead of holding the level
    },
    "quality": {"window_s": 60.0, "min_ratio": 0.8, "max_gap_s": 0.5, "min_observed_s": 10.0},
    "logging": {"csv_path": "logs/fatigue_log.csv", "interval_s": 5.0},
    "display": {"window_width": 1280, "show_depth": True},
}


class ConfigError(ValueError):
    """Raised when the configuration is inconsistent."""


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """Return base updated recursively with override (inputs are not mutated)."""
    out = copy.deepcopy(base)
    for key, val in override.items():
        if isinstance(val, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], val)
        else:
            out[key] = copy.deepcopy(val)
    return out


def _check(cond: bool, msg: str) -> None:
    if not cond:
        raise ConfigError(msg)


def _validate_fatigue_sections(cfg: Dict[str, Any]) -> None:
    """Checks of the three algorithms and the combiner."""
    for name in ("task_fatigue", "visual_fatigue", "attention_fatigue", "fatigue_fusion"):
        sec = cfg[name]
        _check(0.0 < sec["medium_threshold"] < sec["high_threshold"] <= 1.0,
               f"{name}: 0 < medium_threshold < high_threshold <= 1")
    for name in ("task_fatigue", "visual_fatigue", "attention_fatigue"):
        sec = cfg[name]
        _check(sec["accumulation_rate"] >= 0 and sec["recovery_rate"] >= 0, f"{name}: rates must be >= 0")
        _check(0.0 <= sec["recovery_score_threshold"] <= 1.0, f"{name}: recovery_score_threshold in [0,1]")
    tf = cfg["task_fatigue"]
    _check(set(tf["weights"]) == {"backtracking", "error", "prolonged_viewing", "hesitation"},
           "task_fatigue.weights need backtracking, error, prolonged_viewing, hesitation")
    _check(all(v >= 0 for v in tf["weights"].values()) and sum(tf["weights"].values()) > 0, "task weights invalid")
    _check(tf["big_backward_steps"] >= 0 and tf["window_s"] > 0, "task_fatigue: big_backward_steps >= 0, window_s > 0")
    _check(tf["backtracking_full_units"] > 0 and tf["error_full_units"] > 0, "task_fatigue: *_full_units > 0")
    _check(tf["review_min_dwell_s"] >= 0 and tf["error_min_backward_steps"] >= 1 and tf["error_unit"] > 0,
           "task_fatigue: review_min_dwell_s >= 0, error_min_backward_steps >= 1, error_unit > 0")
    _check(tf["calibration_min_instructions"] >= 1 and tf["duration_factor"] > 0, "task_fatigue calibration invalid")
    _check(tf["prolonged_view_full_s"] > 0 and tf["hesitation_full_s"] > 0, "task_fatigue: *_full_s > 0")
    vf = cfg["visual_fatigue"]
    _check(vf["microsleep_growth"] > 0, "visual_fatigue.microsleep_growth > 0")
    _check(set(vf["weights"]) == {"microsleep", "yawn", "nod", "eye_rub", "perclos"},
           "visual_fatigue.weights need microsleep, yawn, nod, eye_rub, perclos")
    _check(all(v >= 0 for v in vf["weights"].values()) and sum(vf["weights"].values()) > 0, "visual weights invalid")
    _check(set(vf["normalization"]) == {"perclos", "closure", "yawn", "nod", "rub"},
           "visual_fatigue.normalization needs perclos, closure, yawn, nod, rub")
    for name, knots in vf["normalization"].items():
        xs, rs = [k[0] for k in knots], [k[1] for k in knots]
        _check(len(knots) >= 2 and all(b > a for a, b in zip(xs, xs[1:])), f"{name}: knot x must strictly increase")
        _check(all(b >= a for a, b in zip(rs, rs[1:])) and all(0.0 <= r <= 1.0 for r in rs), f"{name}: knot r invalid")
    _check(vf["smoothing_tau_s"] >= 0 and 0.0 <= vf["microsleep_high_score"] <= 1.0, "visual_fatigue smoothing/high score invalid")
    af = cfg["attention_fatigue"]
    _check(0 < af["sustained_threshold_s"] < af["high_threshold_s"], "attention_fatigue: 0 < sustained_threshold_s < high_threshold_s")
    _check(af["sustained_growth"] >= 0, "attention_fatigue.sustained_growth >= 0")
    _check(af["brief_window_s"] > 0 and af["history_window_s"] > 0 and af["brief_full_count"] > 0,
           "attention_fatigue windows / brief_full_count must be > 0")
    _check(0.0 <= af["brief_weight"] <= 1.0, "attention_fatigue.brief_weight in [0,1]")
    _check(af["unverified_after_s"] >= 0 and af["medium_threshold"] <= af["unverified_score"] < af["high_threshold"],
           "attention_fatigue: medium_threshold <= unverified_score < high_threshold (unverified must never be High)")
    ff = cfg["fatigue_fusion"]
    _check(set(ff["weights"]) == {"task", "visual", "attention"} and sum(ff["weights"].values()) > 0
           and all(v >= 0 for v in ff["weights"].values()), "fatigue_fusion.weights need task, visual, attention")
    _check(0.0 <= ff["recovery_threshold"] < ff["medium_threshold"], "fatigue_fusion: 0 <= recovery_threshold < medium_threshold")
    _check(all(ff[k] >= 0 for k in ("medium_persistence_s", "high_persistence_s", "recovery_persistence_s")),
           "fatigue_fusion persistence values must be >= 0")


def validate_config(cfg: Dict[str, Any]) -> None:
    """Raise ConfigError if the configuration is inconsistent."""
    _validate_fatigue_sections(cfg)

    cl = cfg["closure"]
    _check(0 < cl["blink_max_s"] < cl["long_blink_max_s"] < cl["prolonged_max_s"], "closure limits must ascend")
    _check(cl["count_min_s"] >= cl["blink_max_s"], "count_min_s must be >= blink_max_s")

    pc = cfg["perclos"]
    _check(0 < pc["ear_threshold_min"] < pc["ear_threshold_max"] < 1, "EAR clamp range invalid")
    _check(0 < pc["ear_threshold_ratio"] < 1, "ear_threshold_ratio in (0,1)")
    _check(pc["reopen_hysteresis"] >= 1.0, "reopen_hysteresis >= 1")
    _check(pc["limits"]["low_below"] < pc["limits"]["high_at"], "PERCLOS limits must ascend")

    at = cfg["attention"]
    _check(set(at["stations"]) == set(STATIONS), f"stations must be {STATIONS}")
    for name, s in at["stations"].items():
        _check(0 <= s["theta_min"] < s["theta_max"], f"{name}: theta_min < theta_max")
        _check(0 <= s["phi_min"] < s["phi_max"], f"{name}: phi_min < phi_max")
    _check(0 < at["membership_threshold"] < 1, "membership_threshold in (0,1)")
    _check(at["capture_min_samples"] >= 1 and at["capture_s"] > 0, "capture_s > 0, capture_min_samples >= 1")
    _check(at["working_area_margin_deg"] >= 0, "working_area_margin_deg >= 0")
    _check(at["capture_timeout_s"] >= at["capture_s"], "capture_timeout_s >= capture_s")
    rg = at["ranges"]
    _check(rg["k_core"] > 0 and rg["fade_k"] >= 0, "ranges.k_core > 0, fade_k >= 0")
    for ax in ("yaw", "pitch"):
        _check(0 < rg["min_core_deg"][ax] <= rg["max_core_deg"][ax], f"ranges: {ax} min_core <= max_core")
    _check(0 < rg["min_fade_deg"] <= rg["max_fade_deg"], "ranges: min_fade_deg <= max_fade_deg")
    _check(at["switch_margin"] >= 0, "switch_margin >= 0")
    for st_name, ov in at["overrides"].items():
        _check(st_name in STATIONS, f"attention.overrides: unknown station {st_name}")
        for k, v in ov.items():
            _check(k in ("yaw_core_deg", "pitch_core_deg") and (v is None or v > 0),
                   f"attention.overrides.{st_name}.{k} must be yaw_core_deg/pitch_core_deg > 0")

    _check(cfg["vision"]["backend"] in ("face_hand", "holistic"), "vision.backend: face_hand | holistic")
    _check(cfg["vision"]["max_consecutive_failures"] >= 1, "vision.max_consecutive_failures >= 1")

    bp = cfg["body_presence"]
    _check(all(0.0 < bp[k] <= 1.0 for k in ("min_visibility", "nose_visible_min", "ear_asymmetry")),
           "body_presence visibilities must be in (0, 1]")
    _check(bp["turn_ratio"] > 0 and bp["confirm_s"] >= 0, "body_presence: turn_ratio > 0, confirm_s >= 0")
    ph = cfg["physical"]
    _check(ph["window_s"] > 0 and ph["update_s"] > 0 and ph["pose_every_n"] >= 1 and 0 < ph["min_visibility"] <= 1.0
           and ph["history_min"] > 0, "physical: window_s, update_s, history_min > 0; pose_every_n >= 1; 0 < min_visibility <= 1")
    _check(ph["neutral_capture_s"] >= 3 and (ph["start_borg"] is None or 0 <= ph["start_borg"] <= 10),
           "physical: neutral_capture_s >= 3; start_borg None or 0..10")
    _check(len(ph["bands"]) == 2 and 0 < ph["bands"][0] < ph["bands"][1] < 1, "physical.bands must be two increasing values in (0, 1)")
    wp = cfg["work_posture"]
    _check(0 <= wp["hold_s"] <= wp["decision_s"] <= wp["max_engaged_s"], "work_posture: 0 <= hold_s <= decision_s <= max_engaged_s")
    _check(wp["min_hands_in_zone"] >= 0 and wp["capture_s"] > 0 and wp["capture_min_samples"] >= 1
           and wp["capture_timeout_s"] >= wp["capture_s"], "work_posture: capture / hands settings invalid")
    _check(wp["scale_tol"] > 0 and wp["position_tol_sw"] > 0 and wp["zone_margin_sw"] >= 0
           and wp["hand_speed_min_sw_s"] > 0 and wp["step_context_s"] >= 0 and wp["condition_gap_s"] >= 0,
           "work_posture: tolerances must be positive")
    ha = cfg["hand_activity"]
    _check(ha["speed_min_fw_s"] > 0 and ha["hold_s"] >= 0, "hand_activity: speed_min_fw_s > 0, hold_s >= 0")
    _check(cfg["perclos"]["grace_min_valid_s"] > 0, "perclos.grace_min_valid_s > 0")
    en = cfg["eye_norm"]
    _check(en["pitch_bin_deg"] > 0 and en["neighbor_bins"] >= 0, "eye_norm: pitch_bin_deg > 0, neighbor_bins >= 0")
    _check(en["window_s"] > 0 and en["min_valid_s"] > 0 and 0 < en["percentile"] <= 100, "eye_norm window/percentile invalid")
    _check(0 < en["scale_min"] <= 1.0 <= en["scale_max"], "eye_norm: scale_min <= 1 <= scale_max")
    q = cfg["quality"]
    _check(0 < q["min_ratio"] <= 1, "quality.min_ratio in (0,1]")
    y = cfg["yawn"]
    _check(y["min_duration_s"] < y["max_duration_s"], "yawn durations must ascend")
    n = cfg["nod"]
    _check(n["min_duration_s"] < n["max_duration_s"], "nod durations must ascend")
    _check(n["excursion_deg"] > 0, "nod.excursion_deg > 0")
    cam = cfg["camera"]
    _check(all(cam[k] > 0 for k in ("color_width", "color_height", "depth_width", "depth_height", "fps")),
           "camera dimensions/fps must be positive")
    for name in ("closure", "yawn", "nod", "rub", "quality"):
        _check(all(not (isinstance(v, float) and (math.isnan(v) or v < 0))
                   for v in cfg[name].values() if isinstance(v, (int, float))),
               f"{name}: negative/NaN value")


_LEGACY_SECTIONS = ("scoring", "distraction")


def _migrate_legacy(user: Dict[str, Any]) -> Dict[str, Any]:
    """Older config files used the single MFR scorer ("scoring") and a "distraction" section. Those sections
    are no longer used (three independent algorithms + combiner); drop them so an old file still loads, and
    carry over the values that still exist under a new name."""
    user = copy.deepcopy(user)
    dn = user.get("distraction", {})
    af = user.setdefault("attention_fatigue", {}) if dn else user.get("attention_fatigue", {})
    for old, new in (("working_area_min_duration_s", "sustained_threshold_s"), ("min_duration_s", "sustained_threshold_s"),
                     ("sustained_high_duration_s", "high_threshold_s"), ("brief_window_s", "brief_window_s"),
                     ("sustained_window_s", "history_window_s"), ("reentry_debounce_s", "reentry_debounce_s"),
                     ("unknown_tolerance_s", "unknown_tolerance_s"), ("min_record_s", "min_record_s")):
        if old in dn:
            af.setdefault(new, dn[old])
    for k in _LEGACY_SECTIONS:
        user.pop(k, None)
    cl = user.get("closure", {})
    if "microsleep_growth" in cl:
        user.setdefault("visual_fatigue", {}).setdefault("microsleep_growth", cl.pop("microsleep_growth"))
    return user


def load_config(path: Optional[str] = None) -> Dict[str, Any]:
    """Load a JSON config (merged over defaults) and validate it."""
    cfg = copy.deepcopy(DEFAULT_CONFIG)
    if path:
        p = Path(path)
        if not p.exists():
            raise ConfigError(f"Config file not found: {p}")
        try:
            user = json.loads(p.read_text(encoding="utf-8"))
        except json.JSONDecodeError as exc:
            raise ConfigError(f"Invalid JSON in {p}: {exc}") from exc
        cfg = deep_merge(cfg, _migrate_legacy(user))
    validate_config(cfg)
    return cfg


if __name__ == "__main__":  # python -m fatigue_monitor.config config.json
    import sys
    target = sys.argv[1] if len(sys.argv) > 1 else "config.json"
    validate_config(DEFAULT_CONFIG)
    Path(target).write_text(json.dumps(DEFAULT_CONFIG, indent=2) + "\n", encoding="utf-8")
    print(f"Wrote {target}")
