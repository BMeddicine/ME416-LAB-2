# Arm tools: firmware, servo calibration and IK

Two desktop GUIs and the ESP32 firmware they talk to. Together they take an arm from "servos wired up" to "drag the end point and the arm follows":

1. **Firmware** (`firmware/`) drives the three servos with smooth, limited moves and takes text commands over USB serial.
2. **Servo angle GUI** (`tools/servo_angle_gui.py`) finds the servo fit `s = a·θ + b` for each joint.
3. **IK GUI** (`tools/ik_gui.py`) uses those fits to move the arm by its end point in the (r, z) plane, and records where the real mechanism collides.

## Setup

- Python 3 with `pyserial` (`pip install pyserial`). Tkinter comes with the standard Windows Python install.
- PlatformIO (VS Code extension) to build and upload the firmware.
- Only one program can hold the serial port. Close the PlatformIO serial monitor (Ctrl+C in its terminal, or the trash icon on its tab) and the other GUI before connecting.

---

## Firmware (`firmware/`)

ESP32-S3 Super Mini, PlatformIO project. Open `firmware/` in PlatformIO, then Build / Upload.

### Joint table

All per-arm servo settings live in `kJoints` at the top of `firmware/src/main.cpp`:

| Joint | Name | Pin | Travel | Limits (servo deg) | Home |
|---|---|---|---|---|---|
| 1 | base 300 | 10 | 300° | 0 to 300 | 250 |
| 2 | 180 #1 | 9 | 180° | 0 to 180 | 45 |
| 3 | 180 #2 | 11 | 180° | 0 to 180 | 27 |

Angles here are **servo angles**, not the θ used in kinematics: 0 is 500 µs, full travel is 2500 µs. Commands outside the limits are rejected, not clamped. Both GUIs read this table at start-up, so changing a limit here changes the sliders and the IK workspace too.

### Serial commands (115200 baud)

| Command | Effect |
|---|---|
| `<joint> <angle>` | Move one joint, e.g. `2 90` |
| `goto <a> <b> <c>` | Move all three joints, arriving at the same time |
| `home` | Move to the home pose |
| `stop` | Freeze where the arm is now |
| `speed <deg/s>` / `accel <deg/s²>` | Motion limits for the next move |
| `here <a> <b> <c>` | Tell the controller where the arm is, without moving |
| `relax` / `hold` | Stop / restart the servo pulses |
| `status` | Print positions, targets, limits and settings |
| `test` | Sweep each joint through its range |

Every move follows a trapezoidal speed profile, updated once per PWM frame, so the servos ramp instead of jumping.

### Start-up and the saved pose

Hobby servos can't report where they are. The firmware saves the last pose to flash once the arm has been still for 0.5 s. At boot it assumes the arm is still there, sends that pose as the very first pulse, then glides to home.

> **Before reflashing with a changed joint table, press Home first** and let the arm stop. If the saved pose doesn't match the new table, the first pulse can snap a servo to the wrong angle at full speed. When the pin order was corrected, the flash key was renamed (`kPoseKey = "pose2"`) for this reason: old saves are ignored and the arm is assumed to be at home.

---

## Servo angle GUI (`servo_angle_gui.py`)

```
python tools/servo_angle_gui.py
```

Use it to find, for each joint, the servo angle at which the link points along +r, plus the scale factor `a`.

- One slider per joint, limited to the firmware range. Moves are sent on release, or while dragging if you tick the box. Next to each slider: a number box (Enter to send) and ±0.5 / ±1 / ±5° nudge buttons.
- **STOP** (or **Esc**), **Go Home**, **Sync from arm**, speed setting.
- "at:" shows the angle the firmware is actually commanding.

### Recording points

1. Move a joint until its link lines up with a direction you can see, e.g. horizontal (θ = 0) or vertical (θ = 90).
2. Pick the joint, enter θ (degrees from +r, CCW positive) and click **Record point**. The GUI asks the arm for its position first, saves all three servo angles, and refuses if the arm is still moving.
3. Repeat at two or more θ values, approaching from both sides.

Each joint is fit as `s = a·θ + b`; `b` is the servo angle along +r. Points save automatically to `logs/servo_angle_points_<timestamp>.csv`.

### Current fits (Bryson's arm)

From `logs/servo_angle_points_corrected.csv`:

| Joint | Angle | Fit | rms | θ range within limits |
|---|---|---|---|---|
| 2 (180 #1) | θ₁, link AD from +r | s = −1.076·θ₁ + 150.5 | 0.17° | −27.4° to 139.9° |
| 3 (180 #2) | θ₂, link DI from +r | s = 0.891·θ₂ + 61.8 | 0.64° | −69.4° to 132.7° |

θ₂ is the **absolute** angle of the forearm from +r, not the angle relative to link 1. Joint 3 points taken at two different joint 2 positions fall on one line, which confirms this.

---

## IK GUI (`ik_gui.py`)

```
python tools/ik_gui.py [calibration.csv]
```

The calibration CSV defaults to `logs/servo_angle_points_corrected.csv`. You can also load one with the button in the GUI.

### Model

The arm in its (r, z) plane is treated as a two-link arm. A is the shoulder at the origin, D the elbow and I the end of the forearm, with a1 = |AD| = 80 mm and a2 = |DI| = 80 mm. The textbook relative elbow angle is θ2L = θ₂ − θ₁. Inverse kinematics, elbow up (θ2L ≤ 0):

```
cos θ2L = (r² + z² − a1² − a2²) / (2·a1·a2)
θ₁      = atan2(z, r) − atan2(a2·sin θ2L, a1 + a2·cos θ2L)
θ₂      = θ₁ + θ2L
```

The servo commands come from the fits: s₂ = a·θ₁ + b and s₃ = a·θ₂ + b. A pose is sent only if both are within the firmware limits. The base (joint 1) only sets which way r points, so it has its own slider in servo degrees.

### Using it

- **Click or drag** anywhere on the plot to move I. If you drag past what the arm can reach, I slides along the edge, and a red ✕ at the cursor says why ("out of reach", "joint 3 limit", ...).
- The **green shading** is every point with an elbow-up solution inside both servos' limits.
- The **dashed grey arm** is where the firmware last reported the arm.
- Moves go out as one `goto`, so all joints arrive together: on release by default, or throttled while dragging.
- **Go to (mm)** moves to a typed r, z. The readout shows r, z, θ₁, θ₂, θ2L and all three servo values.
- **STOP** (or **Esc**), **Go Home**, **Sync from arm** (also runs on connect), speed setting.

### Tracing mechanism limits

The servo limits don't match where the linkage actually collides. To map the real safe region:

1. Drive the arm slowly toward a limit. When it binds or touches, press **Esc**.
2. Press **✕ Mark limit here** or the **M** key. The GUI asks the arm where it froze and marks that spot (when not connected, it marks the GUI target instead).
3. Right-click an ✕ to delete it. **Undo last** and **Clear all** are next to the button.

Marks are saved to `logs/ik_limit_marks.csv` (r, z, all three servo angles, time) and reload on start-up. For now they are only drawn on the plot; they don't block moves.

### Not modelled yet

- The tool point **P** (offset **d** from I). The GUI moves I.
- The table: the shaded region extends below z = 0.
- Linkage collisions, other than through the limit marks.

---

## Reach test (`move_to_tag.py`)

Drives the tool point to an AprilTag found in a photo, then logs how far it actually landed from the tag. The logged misses give the base servo fit and show how far off the reach along the arm is.

### Before the first run

Fill in the `null` values in `config/arm_bryson.json`:

- `shoulderOffset_mm`: `[r, z]` of the shoulder A from the base origin. The base origin is on the base axis at plate level, so z is the shoulder's height above the plate.
- `baseFit`: a starting guess for `s1 = a·θ0 + b`. θ0 is the direction the arm points, measured from +U toward +V, and +U runs parallel to tag 0's x edge (corner 0 → 1). Get it with `servo_angle_gui.py`: jog joint 1 until the arm points along +U. That servo angle is `b`. Then nudge it up a few degrees. If the arm turns counterclockwise seen from above, `a ≈ +1`; otherwise `a ≈ −1`. The reach test refines both.

`toolOffset_mm = [27.724, 12.5, −32.912]` is the offset from I to the tool point in the arm frame: r outward, s 90° counterclockwise from r seen from above, z up. The end plate is assumed to stay level. The 12.5 mm side offset turns the base by `atan2(12.5, R)` so the tool point, not I, lands on the target.

### Each trial

1. Photo with tag 0 and one other tag on the plate (same photo rules as the vision code). Copy it into `photos/`.
2. In MATLAB R2024b, from the repo root:
   ```matlab
   addpath(genpath('src'))
   locateTagTarget(5)          % 5 = ID of the target tag; newest photo in photos/
   ```
   This writes `logs/tagtarget_<time>.json` and an annotated `.jpg`. Check that the red U / green V axes start at the base axis. If they don't, `baseTag.centerUV_mm` in `config/layout.json` is wrong.
3. Run the arm:
   ```
   python tools/move_to_tag.py --dry-run          # print the plan only
   python tools/move_to_tag.py --port COM5        # newest tagtarget_*.json
   ```
   It moves to 40 mm above the target, then down to `--hover` mm above the tag centre (default 5), and waits until the firmware reports it has arrived. Ctrl+C sends `stop`.
4. Measure the tool point's miss from the tag centre (actual − desired) along U and V, with the ruler parallel to tag 0's edges. Type `dU dV` (and optionally `dW`). The row is appended to `logs/tag_reach_trials.csv`, then the arm lifts back to the approach point.

Use targets spread over a range of base angles; ±30° or more gives a good slope for the fit.

### Webcam loop

With the webcam (calibrated in `calibration/webcamParams.mat`, 640×480) mounted over the plate, steps 1–4 run in a loop:

```
python tools/move_to_tag.py --webcam 2 --port COM6 --hover 20
```

1. A live view opens showing which tags OpenCV sees, and whether tag 0 and the target are both visible. This is only a framing aid. Press **Space** to take the photo, or **Q** to quit.
2. The frame is saved to `photos/webcam_<time>.png` and passed to `locateTagTarget` in MATLAB R2024b with the webcam calibration. That takes about 20 s, because MATLAB starts fresh each time. It also writes the usual JSON and annotated image to `logs/`.
3. Confirm, move, measure, as above. Afterwards the arm parks at home, out of the next photo. Move the tag and repeat.

- `--camera` picks the device: 0 is the laptop's own camera, 1 the USB webcam on this PC.
- `--dry-run` runs the photo and plan steps without moving.
- Needs `opencv-python` (`pip install opencv-python`).
- MATLAB has no webcam support package here, so Python does the capture.

### Fitting

```
python tools/move_to_tag.py --fit
```

For every measured trial, it recovers the base angle the arm actually reached and splits the miss into:

- **radial**: reach error along the arm. Points to the shoulder offset, the tool offset or the joint 2/3 fits.
- **tangential**: base angle error × reach. Points to `baseFit`.

It also prints the `reachCorrection_mm` the trials call for. This is how far short along r the tool lands, and the code sends I that much further out. It accounts for the correction each trial was run with. If the per-trial values spread widely, the reach error is a scale error, not a constant offset, so look at the link lengths or the joint 2/3 fits instead.

Then it fits `s1 = a·θ0 + b` from the reached angles (only `b` if they span less than 5°) and prints a `baseFit` line to paste into the arm config.

---

## Files

| Path | What |
|---|---|
| `firmware/src/main.cpp` | Servo controller, joint table, serial commands |
| `tools/servo_angle_gui.py` | Joint sliders and servo-fit data collection |
| `tools/ik_gui.py` | (r, z) IK GUI and limit marks (imports `SerialLink` and helpers from `servo_angle_gui.py`) |
| `logs/servo_angle_points_corrected.csv` | Calibration points used for the current fits |
| `logs/servo_angle_points_incorrect_joints.csv`, `..._182141.csv`, `..._181529.csv` | Points recorded before the pin swap was fixed (joints 1 and 3 mislabelled) |
| `logs/ik_limit_marks.csv` | Traced mechanism limit points |
| `tools/move_to_tag.py` | Reach test: tag photo → tool point → measured miss → base servo fit |
| `config/arm_bryson.json` | Per-arm geometry (shoulder offset, tool offset) and base servo fit |
| `src/vision/locateTagTarget.m` | MATLAB: photo → one tag's position in the base frame (`logs/tagtarget_*.json`) |
| `logs/tag_reach_trials.csv` | Reach test trials: commanded pose, servo values and measured miss |
