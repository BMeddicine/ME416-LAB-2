# ME416 Lab 2: Vision-Guided Stamp Machine

A RASTICrane (3-joint arm) bolted to the stamp rack spells a name, one letter at a time: it picks a rubber stamp from its holder, inks it, presses it onto the paper, then puts it back in its holder and wipes the magnet off. An overhead camera finds the stamps, ink pad and paper from AprilTags; nothing is measured by hand at run time.

The task is run **twice**, once with each teammate's arm on the same rack, each spelling its owner's name. The code and layout stay the same; only the per-arm config file changes.

> **Status:** in progress. Hard deadline: **Tue Oct 13, midnight**.

---

## How it works

```
Phone photo (iPhone, 0.5x lens, copied into photos/)
   │
   ▼
AprilTag detection ──► tag corners, undistorted with the calibration
   │
   ▼
Base tag (+ optional OnShape holder tags) ──► H ──► camera pose R, t via K
   │
   ▼
H_h = K[r1 r2 (h·r3 + t)] per tag height ──► every tag's [U, V] and yaw
   │
   ▼
Targets [U, V, W]: tag position + offset in the tag's frame (e.g. tag → disc)
   │
   ▼
Motion sequencer, per letter:
   pick stamp → lift → ink → press on arc → return to holder → wipe outward (release)
   │
   ▼
Inverse kinematics (rasticraneIk) ──► joint angles θ
   │
   ▼
Servo fit (s = aθ + b, per joint, per arm) ──► servo commands s
   │
   ▼
MQTT publish ──► arm
```

Detection and homography run on the PC; the microcontroller only receives servo commands over MQTT. (The PC vs microcontroller choice is justified in the report.)

Letters go on an arc at a fixed ρ from the base, with ϕ increasing per letter. Each letter comes out rotated by the yaw it was pressed at.

---

## Repository layout

```
.
├── config/           # layout.json (shared rack layout) + one file per arm (see config/notes.md)
├── calibration/      # iPhone 0.5x camera calibration (cameraParams .mat)
├── photos/           # drop the rack photo here (not committed)
├── src/
│   ├── main.m        # loads configs, locates the rack, spells the name
│   ├── vision/       # photo → tag poses → targets in the base frame (locateScene.m)
│   ├── kinematics.m  # rasticraneIk, joint limits
│   ├── control.m     # servo fit θ → s, sending commands to the arm
│   ├── sequence.m    # pick / ink / press / return / wipe
│   └── logging.m     # Problem 2 table
├── tests/            # synthetic tests for src/vision
├── firmware/         # ESP32-S3 servo controller (PlatformIO)
├── tools/            # servo angle calibration GUI
├── logs/             # scene logs + annotated images, Problem 2 table output
├── report/           # videos, sheet scans, figures, report
└── README.md
```

Only the per-arm config changes between runs. Everything about the rack lives in the shared layout file.

---

## Setup

<!-- TODO: fill in dependencies, build steps and broker details -->

MATLAB (same language as HW1). Needs the Computer Vision Toolbox (`readAprilTag`, `undistortPoints`, `insertShape`). On this PC it is installed in **R2024b**, not R2025b.

1. Camera calibration: done (`calibration/iphone-0.5-cameraparams.mat`, 0.31 px mean reprojection error).
2. Fill in the `null` values in `config/layout.json` (tag IDs, base tag offset and yaw from the base axis, tag → disc offset, heights). The program stops and names any required value that is still `null`.
3. Place the ink pad and paper by hand anywhere on the plate.
4. Confirm how commands reach the arm (broker address and topic, if MQTT).

### Taking the photo

- iPhone main camera app at **0.5x**, same resolution as the calibration (5712×4284), and JPEG ("Most Compatible" in Settings → Camera → Formats). The code checks the image size and refuses a photo taken with another lens or setting.
- Base tag, every holder tag, ink pad tag and paper tag all in frame. Hold roughly overhead.
- Copy the original file into `photos/`. The newest file is used. The program reads it the same way calibration did, ignoring the EXIF rotation, so don't rotate or edit it.

**Code style:** camelCase, with a unit suffix where it helps (`targetYaw_deg`, `sampleTime_ms`).

## Running

Locate the rack (vision only, for now):

```matlab
addpath(genpath('src'))
scene = locateScene(RequiredLetters="BRYSON");   % newest photo in photos/
```

This prints every target ([U, V, W], ρ, ϕ, yaw) and writes `logs/scene_<time>.json` plus an annotated `.jpg`. In the image, the yellow 50 mm grid and the red U / green V axes are the base frame drawn back onto the photo. If they don't line up with the plate, the base tag offset or yaw in the layout is wrong. Tags outlined in red failed the side-length check, which usually means the wrong `height_mm` for that tag.

Tests: `matlab -batch "addpath(genpath('src')); runtests('tests')"` (use the R2024b `matlab.exe`).

```
TODO: full run command, with the arm's config file as an argument
```

1. Run with Bryson's arm mounted and its config.
2. Swap the arm on the rack and run with Aviva's config. No code or layout changes.
3. Relocation clip (either arm): move the paper and ink pad, run again for at least two letters.

---

## Calibration notes

- **Servo fit:** 6–10 points per joint across the working range, approached from both directions. The gap between directions is backlash and is logged separately.
- **Elbow angle θ₂ is absolute** (measured from horizontal), not relative to the upper arm.
- **Tool length changes:** the tip is the magnet when empty and the stamp face when loaded. Ink and press heights use the loaded length.
- **Tag size:** use the measured printed square (nominal 50 mm), not the nominal size.
- **Stamp disc height:** the disc sits above the plate, so use H_h or account for the parallax.
- **Stamp holder positions:** from the OnShape document or from their tags; the choice is justified in the report.

## Safety

- Unreachable targets are rejected, not clamped into a guess.
- W is clamped above a floor height; stalling a servo into the table can destroy it.
- Hobby servos report no position, so the code waits after each command and breaks long moves into small steps so a loaded stamp does not swing off the magnet.
- Transport moves lift high enough that a loaded stamp clears the other holders.
- The release wipe goes outward along ρ at constant ϕ; any other direction runs the stamp into the side of the holder.
- Ink pad and paper positions come from tags at run time; nothing is hardcoded.

---

## Logging (Problem 2)

For every letter and both arms, one table with:

- Target tag pixel center and the [U, V] computed from it
- Commanded press [U, V], joint angles θ and servo commands s
- Landed [U, V], measured with a ruler from the base tag after the run, and the miss in mm

## Error budget

The predicted miss combines, by root-sum-square: camera calibration reprojection error, tag corner error through H, parallax at the disc height, and the servo fit residual (plus backlash and load sag). Base rotation error dominates: 1° at a 200 mm radius is roughly 3.5 mm at the tip. Predicted vs measured results are in the report.

---

## Deliverables

- **Video:** one unbroken take per arm, first command to last letter, with arm and paper in frame the whole time; plus a short relocation clip (paper and ink pad moved, at least two letters).
- **Sheets:** all three stamped sheets, labelled with the arm, base tag position marked, and the miss in mm written next to every letter.
- **Report** (4 pages max + code and conversation-history annex):
  - Problem 1 answers
  - Problem 2 table with the miss for every letter
  - Error budget: predicted vs measured miss, and which term dominates
  - Letter orientations compared with Problem 1(b)
  - Arm-vs-arm comparison of misses, and what the data says about the cause
  - AI contribution statement: what the model wrote, what we wrote, what we changed

## Authors

- Bryson Medd
- Aviva Harman
