# Vision: photo → positions in the arm's base frame

MATLAB code that takes one webcam photo of the plate and returns positions in the arm's base frame [U, V, W] (mm). The Python program in [`src/`](../src/README.md) calls it with `matlab -batch`, then reads the JSON it writes to `logs/`. There are two entry points:

| Function | Used by | Needs in the photo | Writes |
|---|---|---|---|
| `locateStampHolder` | `src/stamp_hover_gui.py`, `src/check_stamp_holder.py` | base tag 0 and holder tags 1 and 3 (tag 2 by the ink pad if stamping) | `logs/stampholder_<time>.json` + annotated `.jpg` |
| `locateTagTarget` | `src/move_to_tag.py` (reach trials) | base tag 0 and the target tag | `logs/tagtarget_<time>.json` + annotated `.jpg` |

## Running it by hand

In MATLAB **R2024b** (the install with the Computer Vision Toolbox), from the repo root:

```matlab
addpath(genpath('vision'))
result = locateStampHolder("photos/webcam_<time>.png", Calibration="calibration/webcamParams.mat", ...
                           TagHints="photos/webcam_<time>.tags.json");
result = locateTagTarget(5, "photos/webcam_<time>.png");   % 5 = ID of the target tag
```

With no photo argument, both use the newest photo in `photos/`. The default calibration is `layout.calibrationFile`, the webcam. Other options: `LayoutPath=`, `HolderPath=` (stamp holder), `TargetHeight_mm=` (tag target), `Save=`, `Show=`.

The photo must come from the calibrated webcam at 640×480. The code compares the photo size with the calibration and stops if they differ.

### Stamp holder: tags 1 and 3 → 26 stamp tops

`locateStampHolder` maps the base tag and the curved holder's two tags (1 left, 3 right, seen from the tag side). It maps both holder tag centres at their height (5.5 mm, on pads tilted ~9°), builds the holder frame from them, and applies the CAD offsets in [`config/stamp_holder.json`](../config/stamp_holder.json) to get every stamp top (seat + 30 mm). Any other tag in the photo (e.g. tag 2 by the ink pad) is located at `otherTagHeight_mm` (default 0) and saved in `tags` with role `"other"`. Each saved stamp has `topPx`, the stamp top's pixel on the raw photo, which the GUI uses to pick a stamp by clicking on it. The JSON also stores `radialDistortion`, so Python can project world points onto the raw photo.

On the real rack `readAprilTag` misses tag 3. The holder tags have only a thin white line between their black border and the black pad, so the border merges into the pad. The Python webcam tools write OpenCV's detections to `photos/<photo>.tags.json` and pass them as `TagHints=`. For any hinted tag the first pass missed, `detectTags` whites out the photo outside the hinted outline and reads it again. Only `readAprilTag`'s corners are used, because OpenCV's outline sits on the outer edge of the white line, about 5 px outside the tag's border.

The annotated image draws, for each of the 26 positions, the well opening at rim height, the stamp top, and a line from the seat up to the top (lower row cyan, upper row green). It also draws the holder tags and the base-frame grid.

### Tag target: one tag

`locateTagTarget(id)` returns that tag's centre, ρ, ϕ and yaw. `move_to_tag.py` drives the tool point there and logs the miss.

## How it works

```
photo ─► readAprilTag ─► corner pixels ─► undistortPoints
                                              │
                        base tag 0 corners ───┤
                                              ▼
                         H (DLT) ─► split with K ─► camera pose R, t ─► refine
                                                            │
                         for each tag at height h:          ▼
                         H_h = K [r1  r2  h·r3 + t] ─► corners on plane W = h
                                                            │
                                                            ▼
                         centre, yaw (corner 0 → 1) ─► + CAD offsets ─► targets
```

1. **Detect** (`detectTags`): readAprilTag finds the tag36h11 tags, with the hint pass described above. Each tag's 4 corner pixels are undistorted with the calibration, so all later math can assume an ideal pinhole camera.
2. **Reference homography** (`findHomography`): the base tag's world corners are known, because its centre and yaw relative to the base axis were measured. The HW4 DLT, extended to N points, fits H from those correspondences.
3. **Camera pose** (`estimateTablePose`): with K known, H splits into the plane's pose, `[r1 r2 t] = λ K⁻¹ H` with `r3 = r1 × r2`. The result is snapped to the nearest true rotation, then refined by Gauss–Newton on pixel reprojection error.
   - **Handedness check:** if the camera comes out below the table, the corner order or yaw convention is mirrored, and the program stops instead of returning flipped coordinates.
4. **Any height** (`homographyAtHeight`): once the pose is known, `H_h = K [r1 r2 (h·r3 + t)]` is the homography for the plane W = h (the handout's H_h). Each tag is mapped back through the H_h for its own height. Raised tags and the stamp tops therefore get no parallax error.
5. **Locate** (`locateTag`): the tag centre is the mean of its 4 corners on that plane. Its yaw is the direction of corner 0 → 1, the same convention as HW4. The mapped side length is checked against `tagSize_mm`.
6. **Targets** (`solveStampHolder`, `solveTagTarget`): the photo-independent cores, tested directly.

## Configuration

[`config/layout.json`](../config/layout.json) holds the camera and tag settings:

| Key | Meaning |
|---|---|
| `calibrationFile` | default camera calibration (`calibration/webcamParams.mat`) |
| `photoDir`, `logDir` | where photos are read from and results are written |
| `tagFamily`, `tagSize_mm` | tag36h11, measured black square of the printed tags |
| `baseTag` | `id`, `centerUV_mm` and `yaw_deg` of tag 0 in the base frame, `height_mm` |
| `tagCornerWinding` | `"ccw"` for readAprilTag (verified). Only change it if the program says the frame is mirrored |
| `sideLengthTolerance` | fraction of `tagSize_mm` before a tag is flagged (default 0.03) |

The holder geometry is in [`config/stamp_holder.json`](../config/stamp_holder.json).

**Conventions**
- Lengths are in mm and angles in degrees.
- World frame: origin on the base axis at plate level, W up. U, V are parallel to tag 0's x, y edges.
- Tag frame: x runs along detected corner 0 → 1, and y is 90° counterclockwise from x, seen from above.
- Yaw is the direction of the tag's x axis, measured from +U toward +V.

## Checking the result

- **Yellow 50 mm grid, red U axis and green V axis:** the base frame drawn back onto the photo, including lens distortion. If the axes don't start at the arm's base, `baseTag.centerUV_mm` is wrong.
- **Magenta outlines:** detected tags, labelled with ID and role. A **red** outline means the side-length check failed, usually because of the wrong tag height or a bad detection.
- **Printed numbers:** reprojection RMS should be a fraction of a pixel, and the base tag check should be about 0.

## Files

| File | Purpose |
|---|---|
| `locateStampHolder.m`, `solveStampHolder.m`, `loadStampHolder.m` | stamp holder: photo → 26 stamp tops (entry point, core, config loader) |
| `locateTagTarget.m`, `solveTagTarget.m` | reach trials: photo → one tag (entry point, core) |
| `detectTags.m` | readAprilTag wrapper (from HW4) with undistorted corners and the tag-hint pass |
| `findHomography.m` | normalized DLT for N ≥ 4 points (from HW4, generalized) |
| `estimateTablePose.m` | H → R, t with K, reprojection refinement, handedness check |
| `homographyAtHeight.m` | pose → H_h for the plane W = h |
| `locateTag.m` | one tag → centre, yaw, side-length check at a given height |
| `tagCornersWorld.m` | tag centre, yaw, size and height → its 4 world corners in detector order |
| `projectWorldToPixel.m` | world → pixel, optionally with lens distortion (for drawing) |
| `applyHomography.m` | apply a 3×3 homography to N points |
| `drawScene.m` | annotated photo: tags, base-frame grid, optional target |
| `loadLayout.m`, `requireValue.m`, `repoRoot.m` | config loading and checks |

## Tests

```
"C:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch "addpath(genpath('vision')); runtests('tests')"
```

The tests project known tags through a known camera and check that the pipeline recovers them:

- [`tests/testStampHolder.m`](../tests/testStampHolder.m): every stamp top from tilted holder tags; wrong tag IDs and a missing holder tag are caught.
- [`tests/testVision.m`](../tests/testVision.m): a tag target from the base tag alone; a mirrored corner order is caught; the 4-point homography is exact.
