# ME416 Lab 2: Vision-Guided Stamp Machine

A RASTICrane (3-joint hobby-servo arm) bolted next to a curved stamp holder stamps letters onto paper. A webcam over the plate finds the holder from AprilTags. The arm then picks a rubber stamp up with a magnet, inks it, presses it onto the paper and slides it back into its well. Nothing about the stamp positions is measured by hand at run time.

The full write-up is in [`docs/Robotics_Lab_2_Report.pdf`](docs/Robotics_Lab_2_Report.pdf).

## Results

| Arm | Letters | Average miss |
|---|---|---|
| Bryson's | B, R, Y | 5.29 mm (4.88, 5.81, 5.19) |
| Aviva's | A | 19.7 mm (not fully calibrated in the time available) |

The error budget predicts 4.1 mm, dominated by tag corner error and servo-fit residual (report, section C).

---

## How it works

```
USB webcam (640×480, calibrated)
   │  photo + OpenCV tag hints
   ▼
src/stamp_hover_gui.py ──matlab -batch──► vision/locateStampHolder.m
   │                                         tag 0 → camera pose → H_h per height
   │                                         tags 1, 3 → holder frame → 26 stamp tops (CAD offsets)
   │  ◄── logs/stampholder_<time>.json ──────┘
   ▼
click stamps / type a word
   │
   ▼
Arm model (config/arm_bryson.json)
   base angle + two-link IK, shoulder and tool offsets,
   calibrated servo fits s = a·θ + b, reach corrections
   │
   ▼
legs: lift → retreat → rotate → extend → grip → ink → press → put back → park
   │  "goto s1 s2 s3" over USB serial
   ▼
firmware/ (ESP32-S3): trapezoidal moves, joint limits ──► servos
```

The PC does the vision and kinematics; the microcontroller only runs smooth servo moves on command. The PC has MATLAB's AprilTag detector and the calibration, and every intermediate result (JSON, annotated photo, logs) can be inspected there.

## Repository layout

```
.
├── src/          # Python program: stamp_hover_gui.py (main) + calibration tools   → src/README.md
├── vision/       # MATLAB: photo → stamp tops / tag position in the base frame       → vision/README.md
├── firmware/     # ESP32-S3 servo controller (PlatformIO); documented in src/README.md
├── config/       # arm calibration, tag layout, stamp holder CAD, stamp job defaults → config/README.md
├── calibration/  # webcam calibration (used) and iPhone calibration (first plan)    → calibration/README.md
├── tests/        # synthetic MATLAB tests for vision/
├── logs/         # calibration points and trial records                             → logs/README.md
├── photos/       # webcam photos land here (not committed)
└── docs/         # report and Q&A (the result video is too large for GitHub)
```

## Setup

- **Python 3** with `pip install pyserial opencv-python numpy` (Tkinter ships with the Windows installer).
- **MATLAB R2024b** with the Computer Vision Toolbox. The Python tools call `C:\Program Files\MATLAB\R2024b\bin\matlab.exe` by default (`--matlab` to change it).
- **PlatformIO** (VS Code extension) to build and upload `firmware/`.
- Only one program can hold the serial port, so close the PlatformIO monitor and any other GUI first.

## Calibrating an arm

Done once per arm, in this order (details in [src/README.md](src/README.md)):

1. **Firmware**: set the joint table (`kJoints` in `firmware/src/main.cpp`) and upload it.
2. **Servo fits**: `python src/servo_angle_gui.py`. Record each joint at known angles to get `s = a·θ + b`.
3. **IK check**: `python src/ik_gui.py`. Drag the end point, and mark where the real linkage collides.
4. **Reach trials**: `python src/move_to_tag.py --webcam <tag> --port COMx`. Measure each miss, then `--fit` for the base servo fit and the reach correction. Paste the result into `config/arm_bryson.json`.
5. **Stamp centring**: in the main GUI's Position check, hover over stamps and jog until centred. The recorded offsets (`logs/stamp_offsets.csv`) give the per-angle base and reach corrections.

## Stamping a word

```
python src/stamp_hover_gui.py --port COM6
```

1. **Connect**, then **Park** so the arm is out of the shot. **Take photo**: MATLAB locates the holder (about 20 s), and the stamps appear as clickable rings.
2. On the **Stamp job** tab, set the paper spot and letter spacing, and enter the stamps in order (e.g. `L3 U7 L3 U12`) or click them.
3. **Stamp word**. Each letter is picked, inked, pressed and put back, then the arm parks. The whole route is planned first, and nothing moves if any leg is out of reach.

## Safety measures

- Unreachable poses (outside the workspace, the joint limits or below the minimum tool height) are rejected, not clamped.
- Every move is split so the tool path stays within 1–2 mm of a straight line, and the firmware ramps every move.
- Base moves always finish turning the same way (with a 5° overshoot), because the base servo stops ~3° apart depending on direction.
- Travel is at a safe height (or the carry height with a stamp) that clears the holder. The arm pauses before going down so it doesn't swing into a well.
- **STOP** / **Esc** freezes the arm at any time.

## Tests

```
"C:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch "addpath(genpath('vision')); runtests('tests')"
```

## AI use

Claude was used to outline the project from the assignment and to write the Python programs (servo calibration GUI, arm position GUI, vision + stamping GUI). We then tested each one on the arm and iterated with it. See the report's contribution statement.

## Authors

- Bryson Medd
- Aviva Harman
