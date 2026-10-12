"""
experiment_logger.py
--------------------
Records everything that happens in one live run (from pressing G until stop)
and saves it as:

    results/<mode>_experiment_<N>.csv    one row per loop step (for plotting)
    results/<mode>_experiment_<N>.json   run info + timestamped event log

<mode> is "solo", "assistive" or "collaborative".
<N> is chosen automatically: 1 for the first run of that mode, then 2, 3, ...

Rows are written to disk as they are logged, so data is not lost if the
program crashes or the window is closed.
"""

import csv
import json
import os
import re
import time
from datetime import datetime

VALID_MODES = ("solo", "assistive", "collaborative")


def _to_plain(value):
    """Convert numpy values/arrays and other objects to CSV/JSON-friendly types."""
    if hasattr(value, "tolist"):          # numpy arrays and numpy scalars
        value = value.tolist()
    if isinstance(value, (list, tuple, dict)):
        return json.dumps(value)
    return value


class ExperimentLogger:
    def __init__(self, mode, results_dir="results"):
        mode = mode.lower().strip()
        if mode not in VALID_MODES:
            raise ValueError(f"mode must be one of {VALID_MODES}, got '{mode}'")
        self.mode = mode
        self.results_dir = results_dir
        self.running = False
        self.name = None
        self._csv_file = None
        self._writer = None
        self._columns = None
        self._t0 = None
        self._step = 0
        self._events = []
        self._info = {}

    # ---------- file naming ----------
    def _next_index(self):
        os.makedirs(self.results_dir, exist_ok=True)
        pattern = re.compile(rf"^{self.mode}_experiment_(\d+)\.(csv|json)$")
        used = [int(m.group(1)) for f in os.listdir(self.results_dir)
                if (m := pattern.match(f))]
        return max(used, default=0) + 1

    # ---------- run control ----------
    def start(self, **run_info):
        """Call when G is pressed. Extra keyword arguments (participant,
        task name, parameters...) are saved in the .json file."""
        if self.running:
            return
        idx = self._next_index()
        self.name = f"{self.mode}_experiment_{idx}"
        self._csv_path = os.path.join(self.results_dir, self.name + ".csv")
        self._json_path = os.path.join(self.results_dir, self.name + ".json")
        self._csv_file = open(self._csv_path, "w", newline="")
        self._writer = None
        self._columns = None
        self._t0 = time.perf_counter()
        self._step = 0
        self._events = []
        self._info = {
            "name": self.name,
            "mode": self.mode,
            "start_wall_time": datetime.now().isoformat(timespec="seconds"),
            **{k: _to_plain(v) for k, v in run_info.items()},
        }
        self.running = True
        self.log_event("start")
        print(f"[logger] Recording {self.name}")

    def elapsed(self):
        return time.perf_counter() - self._t0 if self.running else 0.0

    def log_step(self, **values):
        """Call once per loop iteration with whatever you want recorded,
        e.g. log_step(cognitive_load=0.42, hr_distance=0.85, action='WAIT')."""
        if not self.running:
            return
        row = {"t": round(self.elapsed(), 4), "step": self._step}
        row.update({k: _to_plain(v) for k, v in values.items()})

        if self._writer is None:              # first row fixes the columns
            self._columns = list(row.keys()) + ["extra"]
            self._writer = csv.DictWriter(self._csv_file, fieldnames=self._columns)
            self._writer.writeheader()

        extra = {k: row.pop(k) for k in list(row) if k not in self._columns}
        row["extra"] = json.dumps(extra) if extra else ""
        self._writer.writerow(row)
        self._csv_file.flush()
        self._step += 1

    def log_event(self, name, **details):
        """Mark a moment, e.g. log_event('tool_request', tool='screwdriver')."""
        if not self.running:
            return
        event = {"t": round(self.elapsed(), 4), "step": self._step, "event": name}
        event.update({k: _to_plain(v) for k, v in details.items()})
        self._events.append(event)
        self._write_json()                    # keep the json up to date too

    def stop(self, **summary):
        """Call when the run is stopped. Safe to call more than once."""
        if not self.running:
            return
        self.log_event("stop")
        self._info["end_wall_time"] = datetime.now().isoformat(timespec="seconds")
        self._info["duration_s"] = round(self.elapsed(), 3)
        self._info["n_steps"] = self._step
        self._info.update({k: _to_plain(v) for k, v in summary.items()})
        self._write_json()
        self._csv_file.close()
        self.running = False
        print(f"[logger] Saved {self._csv_path} and {self._json_path}")

    def _write_json(self):
        with open(self._json_path, "w") as f:
            json.dump({"info": self._info, "events": self._events}, f, indent=2)

    # allows: with ExperimentLogger("solo") as log: ...
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.stop()
