# ME416 Lab 2: Vision-Guided Stamp Machine

A RASTICrane (3-joint arm) bolted to the stamp rack spells a name, one letter at a time: it picks a rubber stamp from its holder, inks it, presses it onto the paper, then puts it back in its holder and wipes the magnet off. An overhead camera finds the stamps, ink pad and paper from AprilTags; nothing is measured by hand at run time.

The task is run **twice**, once with each teammate's arm on the same rack, each spelling its owner's name. The code and layout stay the same; only the per-arm config file changes.

> **Status:** in progress. Hard deadline: **Tue Oct 13, midnight**.

---

## How it works

```
Camera frame
   │
   ▼
AprilTag detection ──► pixel centers + yaw per tag
   │
   ▼
Homography (H for the table plane, H_h for raised planes such as the stamp discs)
   │
   ▼
Table coordinates [U, V] (+ tag-to-disc offset rotated by tag yaw)
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
├── config/           # one file per arm + one shared layout file (see config/notes.md)
├── calibration/      # camera intrinsics K, distortion, reprojection error
├── src/
│   ├── main.cpp      # loads configs, locates the rack, spells the name
│   ├── vision.cpp    # tag detection, homography (H, H_h), pixel → [U, V]
│   ├── kinematics.cpp  # rasticraneIk, joint limits
│   ├── control.cpp   # servo fit θ → s, sending commands to the arm
│   ├── sequence.cpp  # pick / ink / press / return / wipe
│   └── logging.cpp   # Problem 2 table
├── logs/             # Problem 2 table output
├── report/           # videos, sheet scans, figures, report
└── README.md
```

Only the per-arm config changes between runs. Everything about the rack lives in the shared layout file.

---

## Setup

<!-- TODO: fill in dependencies, build steps and broker details -->

C++ (same language as HW1).

1. Install dependencies: `TODO`
2. Calibrate the camera and save K and distortion to `calibration/`.
3. Place the ink pad and paper by hand anywhere on the plate. Mount the camera so it sees the base tag, stamp holders, ink pad and paper at once.
4. Confirm how commands reach the arm (broker address and topic, if MQTT).

**Code style:** camelCase, with a unit suffix where it helps (`targetYaw_deg`, `sampleTime_ms`).

## Running

```
TODO: run command, with the arm's config file as an argument
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
