"""Periodic CSV logging (default: every 5 s), flushed after every row."""
from __future__ import annotations

import csv
import datetime as dt
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO

COLUMNS: List[str] = [
    "timestamp_iso", "t_s", "level_code", "level", "combined_score",
    "task_level", "task_score", "visual_level", "visual_score", "attention_level", "attention_score",
    "attention_state", "active_evidence", "data_valid", "data_warnings", "quality",
    "current_instruction", "backtracking_count", "suspected_error_count", "hesitation_count",
    "brief_disengagement_count", "sustained_disengagement_count", "max_outside_s",
    "collab_active", "task_engaged", "face_free",
    "attention", "presence", "immediate_distraction", "d_outside_s",
    "perclos", "closure_weighted_60s", "brief_events_60s", "sustained_max_60s_s",
    "yawns_5min", "nods_5min_weighted", "rubs_5min",
    "r_perclos", "r_closure", "r_brief_disengagement", "r_sustained_disengagement", "r_yawn", "r_nod", "r_rub",
    "ear", "ear_threshold", "mar", "yaw_deg", "pitch_deg",
    "depth_valid", "distance_m", "x_m", "y_m", "z_m",
    "microsleep_count", "microsleep_score", "yawn_count", "nod_count", "eye_rub_count",
]


# Column sets of the single-algorithm programs (main_task.py / main_visual.py / main_attention.py).
MODE_COLUMNS: Dict[str, List[str]] = {
    "task": ["timestamp_iso", "t_s", "task_level", "task_score", "current_instruction", "backtracking_count",
             "suspected_error_count", "hesitation_count", "attention", "quality"],
    "visual": ["timestamp_iso", "t_s", "visual_level", "visual_score", "microsleep_count", "microsleep_score",
               "yawn_count", "nod_count", "eye_rub_count", "perclos", "closure_weighted_60s", "ear", "ear_threshold",
               "mar", "pitch_deg", "quality"],
    "attention": ["timestamp_iso", "t_s", "attention_level", "attention_score", "attention_state", "attention",
                  "presence", "collab_active", "task_engaged", "face_free", "brief_disengagement_count", "sustained_disengagement_count", "max_outside_s",
                  "d_outside_s", "yaw_deg", "pitch_deg", "quality"],
}


def _fmt(v: Any) -> Any:
    if v is None:
        return ""
    if isinstance(v, float):
        return f"{v:.4f}"
    if isinstance(v, bool):
        return int(v)
    return v


class CsvLogger:
    def __init__(self, path: str, interval_s: float, columns: Optional[List[str]] = None) -> None:
        self.columns = list(columns) if columns else COLUMNS
        self.path = Path(path)
        self.interval_s = interval_s
        self._last: Optional[float] = None
        self.path.parent.mkdir(parents=True, exist_ok=True)
        if self.path.exists() and self.path.stat().st_size > 0:      # header changed -> keep old file aside
            with open(self.path, newline="", encoding="utf-8") as fh:
                first = fh.readline().strip()
            if first != ",".join(self.columns):
                self.path.rename(self.path.with_name(
                    f"{self.path.stem}_old_{dt.datetime.now():%Y%m%d_%H%M%S}{self.path.suffix}"))
        new = not self.path.exists() or self.path.stat().st_size == 0
        self._fh: TextIO = open(self.path, "a", newline="", encoding="utf-8")
        self._w = csv.DictWriter(self._fh, fieldnames=self.columns, extrasaction="ignore")
        if new:
            self._w.writeheader()
            self._fh.flush()

    def maybe_log(self, t: float, row: Dict[str, Any]) -> bool:
        if self._last is not None and t - self._last < self.interval_s:
            return False
        self._last = t
        row = dict(row)
        row["timestamp_iso"] = dt.datetime.now().isoformat(timespec="seconds")
        row["t_s"] = t
        self._w.writerow({k: _fmt(row.get(k)) for k in self.columns})
        self._fh.flush()
        return True

    def close(self) -> None:
        try:
            self._fh.close()
        except OSError:
            pass
