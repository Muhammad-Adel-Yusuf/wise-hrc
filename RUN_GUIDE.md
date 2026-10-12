# Live test: real worker in front of the camera, virtual robot

> **Real robot:** before every experiment run the calibration wizard (`cd mental_fatigue_algorithm`, `python calibration_wizard.py`). See **CALIBRATION_GUIDE.md**. It replaces section 1 below (workstation calibration with stand-in objects) when the real Lite 6 is used.

The camera measures the real worker: mental and physical fatigue, attention, presence, and both arms in 3-D. The robot is simulated in MATLAB.
The MATLAB POMDP framework decides every 0.5 s exactly as in the simulation: planner, fatigue monitor, fatigue adaptation and safety supervisor.

```
pilot_physical_full_code/
  mental_fatigue_algorithm/   camera program (main.py) + workstation calibration by clicking (workspace_calibration.py)
  physical_fatigue_model/     physical-fatigue model used by main.py (algorithm 4)
  POMDP model/                MATLAB framework + live loop (liveHRC.m)
  RUN_GUIDE.md                this file
```

## 0. Install once (Windows, Python 3.11)
```
cd mental_fatigue_algorithm
pip install -r requirements.txt
```
MATLAB R2020b or newer (it uses `udpport`). If Windows asks whether to allow MATLAB / Python on the network, allow it; both run on the same PC (127.0.0.1).

## 1. Calibrate the workstation (once per set-up, and again whenever the camera or table moves)
No marker is needed: you click on the camera image and the depth camera turns every click into a 3-D point on the table.
```
cd mental_fatigue_algorithm
python workspace_calibration.py
```
In the window, define the five fixed places (the line at the top always says what to do next):
1. **Robot base**: click the table where the (virtual) robot stands.
2. **Assembly area**: press the mouse at one corner of the area where the assembly is done, drag to the opposite corner and release. A green box appears.
3. **Handover point**: click the table under the point where the robot should hold the tool for the worker. The tool is held 20 cm above it.
   Or press **H** and hold your hand at the exact point for about 3 s.
4. **Tool storage**: click where the tools are.
5. **Safe position**: click where the robot waits when it is idle, when the worker rests or when the session is stopped.

The window then draws everything on the image:
- the assembly area (green box);
- the handover point;
- the storage;
- the collaborative work point (inside the assembly area, on the robot side);
- a grey "shadow" of the robot at the safe position;
- the collaborative and assistive distances.

It checks five things:
- the robot arm at the safe position keeps at least 15 cm from the assembly area;
- the safe position is outside the assistive distance, so a robot waiting there is "separated";
- the handover point is outside the assembly area;
- the handover point is within reach of the assembly area;
- the robot paths (safe → storage → handover → safe) do not cross the assembly area.

Each check is green or red. To redo a place, press its number (**1–5**) and click again. **V** saves (only when everything is green), **Q** quits.
This writes `POMDP model/workspace_calibration.json`. MATLAB reads the places and the distances from it, and `main.py` reads the workstation frame.
The prohibited distance (0.05 m) is not calibrated: it is set in `config.json` (`pomdp_bridge.calibration.prohibit_radius_m`).
Heights and the clearance can be changed in `config.json`, section `pomdp_bridge.calibration`.

The camera must see the table at each place you click. On a black or glossy spot, the program says "no depth" and you click nearby.

## 2. Start the camera program
First check the camera link (about 10 s):
```
cd mental_fatigue_algorithm
python camera_check.py
```
It must say `link: USB3...` and depth about 30 fps. On a USB 2 link depth is limited to ~6-7 fps (the camera sends raw depth data). The program still runs at ~25 fps with the newest depth, but hand/arm distances are measured in a depth image up to ~0.1 s older. `python camera_check.py --watch` re-checks every few seconds while you try other ports and cables.

```
python main.py
```
The log line `perf: .. fps | .. | depth .. fps (time offset to landmarks .. ms) | models face .. hand .. pose .. Hz` appears every 10 s.
In its window, as before: **U** load the instruction slides, **S** start the 60 s baseline, **1/2/3** calibrate assembly / laptop / storage, **Y** physical baseline.
The bottom line of the window must say `POMDP link -> 127.0.0.1:5005 ... workstation frame: calibrated`.

What it sends about 15 times per second:
- physical fatigue level (0/1/2);
- final mental fatigue level (0/1/2);
- attention: focused / uncertain (outside the working area for less than 2 s) / distracted (2 s or more);
- face detected, presence, estimate confidence;
- task counters;
- shoulder, elbow, wrist and palm of both arms in metres, in the workstation frame.

A value that is not available yet is sent as "unknown". Examples:
- mental level before the baseline is finished;
- physical level during its first 30 s;
- attention before the station calibration.

The planner then works from its prior belief.

## 3. Start MATLAB
```
cd "POMDP model"
liveHRC            % (run valueIterationHRC first only if PHRC.mat / VHRC.mat are missing)
```
Click the **HRC live** window. The robot waits at its safe position until you press **G**.

| key | action |
|---|---|
| G / X | START / STOP the session (STOP sends the robot to its safe position; it is not an emergency stop) |
| P / R | PAUSE (rest) / RESUME (continue after a warning, or back to work after a rest; only accepted while the worker is at the workstation) |
| T / K | bring me a tool / I took the tool (normally the camera detects that the hand reached the tool) |
| 1 / 2 / 3 | solo / assistive / collaborative mode |
| E | emergency stop on / off (simulated) |
| Esc | quit (the log `liveLog_<date>.csv` is saved; it is also saved every minute) |

The window shows:
- **left:** the top view, with the assembly area (green), the places, the virtual robot and its zones (red prohibited, orange collaborative, yellow assistive) and the worker's arms from the camera;
- **right:** session, mode and phase, robot action (final and planner), safety reason, worker state, fatigue level and dialogue, and the message for the worker.

## When the virtual robot stops
The safety geometry counts as **verified** only when all of these hold:
- the workstation is calibrated;
- the newest camera message is less than 0.25 s old;
- every tracked arm is complete: shoulder, elbow, wrist and palm, with depth.

If the hand model misses a hand, its palm is estimated 8 cm beyond the wrist.

The safety supervisor stops the robot in these cases:
- the geometry is not verified, for example an arm is partly hidden or the camera program stopped;
- the hand is too close to the robot;
- the whole arm is too close to the robot;
- the worker has been distracted for 2 s while near the robot.

When the camera sees nobody, the work cell counts as empty.

## Notes
- The robot is virtual: this tests the sensing and the decisions, not the reactions of people to a real moving robot, nor real stopping distances.
- All zone sizes and thresholds are illustrative values for these tests, **not validated safety distances**.
- Removed from this package because this stage does not need them. They are still in your original archive:
  - the pilot validation study;
  - the calibration and pilot tools of the physical model;
  - the single-algorithm programs;
  - all tests and logs;
  - the MATLAB simulation and test scripts and their figures.
