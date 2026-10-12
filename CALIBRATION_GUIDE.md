# Calibration guide: real robot (UFactory Lite 6), depth camera and worker

The calibrations that link the camera, the real robot and the worker, and the order to do them in. At the start of
every experiment, run the **calibration wizard**. It checks all of them, shows the live workstation, and asks whether
you are satisfied or want to redo a step.

```
cd mental_fatigue_algorithm
python calibration_wizard.py
```

Press **K** (or Enter) when you are satisfied, then start the experiment as usual (`python main.py`, then `liveHRC`
in MATLAB).

---

## 1. The calibrations at a glance

| Step | Calibration | Why it is needed | Redo it when | Robot moves? | Time | Result file |
|---|---|---|---|---|---|---|
| 0 | **Robot connection** (gateway login) | The programs reach the robot through the Jetson gateway | The login expired (valid 30 days), or on another computer | no | 1 min | `~/.lite6/config.json` (token only, no password) |
| 1 | **Camera placement** | The camera must see the robot's red tool tip, the assembly area, the storage and your hands and head | Before step 2, whenever the camera is moved | no | 2 min | none |
| 2 | **Robot–camera calibration** (eye-to-hand) | Converts every camera point (hands, head) into **robot coordinates** | The **camera** or the **robot base** moved, or the live red-tip check is not GOOD | **YES** | 3 min | `POMDP model/robot_camera_calibration.json` |
| 3 | **Workstation places** | Where the assembly area, storage, safe and handover points are, plus the zone sizes MATLAB uses | The **layout** changed (storage, assembly place, safe or handover point) | no (read only) | 1–3 min | `POMDP model/workspace_calibration.json` |
| 4 | **Hands & head detection check** | Confirms the camera finds your hands and head in robot coordinates | Every session (a quick look) | no | 1 min | none |
| (main.py) | **Fatigue-monitor baselines** (60 s baseline, station calibration, physical baseline) | Per-participant baselines of the fatigue algorithms | Every session / participant | no | ~3 min | see `RUN_GUIDE.md` |

**Moving the camera.** The places of step 3 are stored in **robot coordinates**, so they stay valid when only the
camera moves, as long as the robot and the table objects stay where they are. After a camera move, do **step 1 and
step 2** only. The wizard then updates the camera link of the workstation file automatically; no new baselines are
needed.

---

## 2. Start-of-experiment procedure (wizard)

1. Switch on the robot and the camera; the PC must be on KFUPM-WiFi.
2. Run `python calibration_wizard.py` in the `mental_fatigue_algorithm` folder.
3. The window shows the **live workstation** on the left and the **status of every calibration** on the right:
   - **left**: assembly area (green), assistive zone around the handover point (orange cylinder), collaborative and
     prohibited zones around the robot tool (blue and red rings), safe, handover and storage points, and your hands
     and head with their zone and distance to the robot;
   - **right**: green = OK, yellow = warning, red = needs attention, and a **VERDICT** line.
4. Use the information below to decide. Then press **K** if you are satisfied, or the number of the step to redo.
   After a step, the wizard comes back with the updated status.

| Information shown | Good | Act when |
|---|---|---|
| Live **red-tip check** (robot pose + calibration predict where the red tip is; the camera measures where it is) | GOOD ≤ 25 mm | CHECK 25–50 mm: watch it. BAD > 50 mm: the camera moved, redo **2**. "Not seen": move the robot so its red tip is visible, then look again |
| Robot-camera calibration fit error | ≤ 10 mm rms | > 10 mm: redo **2** with the camera fixed and nobody in view |
| Camera link of the workstation file | "matches" | "OUT OF DATE": press **U** |
| Workstation warnings | none | storage out of reach: move the storage closer and redo **3**. Other warnings: see the text |
| Hands / head seen (last 60 frames) | hands and head most of the time | rarely seen: redo **1** (camera placement) |
| Robot state / gateway | reachable, no error | not logged in, halted or error: see step 0 |

Keys: **K / Enter** satisfied · **1** camera · **2** robot–camera · **3** places · **4** hands & head ·
**U** update the camera link · **Q / Esc** quit without confirming.

---

## 3. The steps in detail

### Step 0: Robot connection
- Once per computer, and again when the login expires (the wizard shows the date):
  `python robot_code/lite6.py login 10.13.37.101` (asks for the gateway password; only a token is stored).
- If the gateway is **halted** after an emergency stop: `python robot_code/lite6.py resume`.
- "Robot state 4 (stopped)" is normal after a program ended; the programs re-enable the robot when they start.

### Step 1: Camera placement (`camera_preview.py`)
- Red pixels are painted green: the robot's red tool ring must be one solid spot. **D** shows the depth, **G** a grid.
- Place the camera so that the **robot tool over its whole working range**, the **assembly area**, the **storage**
  and your **hands and head** are in the image, about 0.7–1.5 m away. Then **fix it**.
- If the camera moved, do **step 2** next.

### Step 2: Robot–camera calibration (`robot_camera_calibration.py`, **the robot moves**)
- The robot visits the poses of **`points_for_robot-human-calibration.txt`** (project folder) at 10 °/s, and the camera
  locates its red tool ring at each pose. The fit gives the camera position in robot coordinates and the position of
  the ring on the tool.
- **Before confirming with Y:** nobody in the image or within reach of the robot, the camera fixed, and someone at the
  STOP button (gateway web page). **Esc** in the calibration window stops the robot at once.
- Saved only if the fit error is below 12 mm. Pictures of every sample are in
  `POMDP model/robot_camera_calibration_images/`; the green cross must sit on the red ring in each.
- Afterwards the wizard updates the camera link of the workstation file (places kept).
- **The poses file:** `posN:[J1,J2,J3,J4,J5,J6, x,y,z]` (joints in degrees; the tool x, y, z in mm, for information),
  plus `speed=slow` and `loop:false`. Record new poses by moving the robot by hand when the camera has moved so much
  that the red tip is no longer visible at most of them. Use about 12–14 poses spread over the working space, with
  different tool orientations.

### Step 3: Workstation places (`define_workstation.py`)
| Place | How it is defined |
|---|---|
| Safe position, handover point | Robot positions recorded by moving the robot by hand: `POMDP model/workstation_points.txt` |
| **Assembly area** | **B**: 60 s hand baseline. Work normally; the area is where your hands went (2nd–98th percentile + 3 cm) |
| **Storage** | **S**: 30 s hand baseline around the storage, or **R**: 30 s with the robot's red tip moved by hand into the storage. The robot's storage point is on the storage edge nearest the robot (hands) or a pose the robot actually reached (robot) |
| Zone sizes | `config.json` → `pomdp_bridge.calibration`: `prohibit_radius_m` (0.01), `collab_radius_m` (0.35) and `assist_radius_m` (0.065) |

- Fine-tune the area with **1/2 3/4 5/6 7/8 9/0** (10 mm per press). **V** saves; red checks block saving.
- The storage must be **within the robot's reach**: about 40 cm from the robot's vertical axis with the tool pointing
  down. The check says how far to move it.
- Only hands the camera sees are used; hands hidden behind the laptop screen cannot be measured.

### Step 4: Hands & head check (`robot_frame_live.py`)
- Shows your head and hands in robot coordinates, their distance to the robot base and tool, and which model found
  them (face, hand or pose model). Move your hands towards the robot tool: the tool distance must shrink smoothly.

---

## 4. Files

| File | Content | Written by |
|---|---|---|
| `points_for_robot-human-calibration.txt` | Robot poses for step 2 (**keep and maintain this file**) | you (robot moved by hand) |
| `POMDP model/workstation_points.txt` | Safe, handover and storage-corner robot positions | you (robot moved by hand) |
| `POMDP model/robot_camera_calibration.json` | Camera → robot transform, fit error, samples | step 2 |
| `POMDP model/workspace_calibration.json` | Places, zones, checks, camera link (read by `main.py` and MATLAB) | step 3 (camera link: wizard **U** / after step 2) |
| `POMDP model/workstation_preview/*.csv` | Hand / robot points of the baselines | step 3 |
| `mental_fatigue_algorithm/config.json` | Camera settings, zone sizes, real-robot link settings | you |

---

## 5. Safety
- The robot moves **only** in step 2 (after **Y**) and when you move it by hand. All other tools only read it.
- Keep everybody out of the robot's reach while it moves; the gateway STOP button and
  `python robot_code/lite6.py estop` stop it at once.
- Zone sizes and distances are illustrative values for these tests, **not validated safety distances**.

## 6. Open points (state on 2026-10-08)
- **Storage out of reach:** the saved storage point is 552 mm from the robot's axis (reach about 416 mm at that
  height). Move the blue bin about 15 cm closer to the robot and redo step 3 (**S**), or use **R** with the robot.
- The assistive cylinder (6.5 cm since 2026-10-08; was 7.5 cm) touched a 1.7 cm corner of the assembly area; nudge the area with **2** or **6**
  in step 3 if this matters.
- Prohibited distance 0.01 m everywhere (2026-10-09; was 0.05, then 0.02) (config → workspace file → MATLAB). With it, the real robot runs slower
  (`robot_code.py`: slow 5, medium 15, fast 30 °/s) so that it travels less than about 3.5 cm in the ~0.85 s it takes
  to react (camera → MATLAB every 0.5 s → actions.md → robot).
