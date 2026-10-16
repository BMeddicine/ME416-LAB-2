# Vision: photo → targets in the base frame

This module takes one phone photo of the stamp rack and returns, in the arm's base frame [U, V, W] (mm):

- every stamp's steel disc, where the magnet should land
- the ink pad
- the paper

Each target also gets its ρ, ϕ and yaw. This is the input to IK. Nothing is measured by hand at run time; only the rack layout (fixed geometry from OnShape) is configured once.

## Quick start

1. Fill in the `null` values in [`config/layout.json`](../../config/layout.json) (see [Configuration](#configuration)).
2. Take the photo (see [Photo rules](#photo-rules)) and copy the original file into `photos/`.
3. In MATLAB **R2024b** (the install that has the Computer Vision Toolbox), from the repo root:

```matlab
addpath(genpath('src'))
scene = locateScene(RequiredLetters="BRYSON");
```

`locateScene` uses the newest photo in `photos/`. Pass a path to use another one: `locateScene("photos/IMG_2001.jpeg")`. Other options are `LayoutPath=`, `Save=false` and `Show=true`.

Example output:

```
== Scene from photos\IMG_2001.jpeg ==
Reference tags 0: 4 corners, reprojection RMS 0.29 px (0.06 mm on the plane)
Camera at [187, 51, 286] mm
Base tag check: 0.00 mm, 0.06 deg from its layout position
Ink pad: [  100.9,     5.4,   0.0] mm  rho  101.0  phi    3.0  yaw   91.6
Paper:   [   33.4,   144.0,   0.0] mm  rho  147.8  phi   76.9  yaw   80.7
Stamp A: disc [   49.5,  -220.7,  30.0] mm  rho  226.2  phi  -77.4  [tag]
```

Each run also writes two files to `logs/`:
- `scene_<time>.json`: the full result, for the Problem 2 table and the report
- `scene_<time>.jpg`: the annotated photo (see [Checking the result](#checking-the-result))

### Reach test: one tag only

`locateTagTarget(id)` needs only the base tag and the tag with that ID in the photo. It writes `logs/tagtarget_<time>.json` for `tools/move_to_tag.py` (see [tools/README.md](../../tools/README.md#reach-test-move_to_tagpy)). Options: `TargetHeight_mm=`, `LayoutPath=`, `Save=`, `Show=`.

### Stamp holder: tags 1 and 3 -> 26 stamp tops

`locateStampHolder()` needs the base tag and the curved holder's two tags (1 left, 3 right, seen from the tag side). It maps both tag centres at their height (5.5 mm, on pads tilted ~9°), builds the holder frame from them, and applies the CAD offsets in [`config/stamp_holder.json`](../../config/stamp_holder.json) to get every stamp top (seat + 30 mm). It writes `logs/stampholder_<time>.json` and an annotated `.jpg` with each well opening and stamp top drawn. Options: `Calibration=`, `HolderPath=`, `LayoutPath=`, `TagHints=`, `Save=`, `Show=`. Any other tags in the photo (e.g. tag 2 by the ink pad) are located at `otherTagHeight_mm` (default 0) and saved in `tags` with role `"other"`. The JSON also stores `radialDistortion`, so Python can project world points onto the raw photo.

On the real rack `readAprilTag` misses tag 3: the holder tags have only a thin white line between their black border and the black pad, so the border merges into the pad. The Python webcam tools write OpenCV's detections to `photos/<photo>.tags.json` and pass them as `TagHints=`. For any hinted tag the first pass missed, `detectTags` whites out the photo outside the hinted outline and reads it again. Only `readAprilTag`'s corners are used: OpenCV's outline sits on the outer edge of the white line, about 5 px outside the tag's border. With the webcam: `python tools/check_stamp_holder.py`. Each saved stamp also has `topPx`, the stamp top's pixel on the raw photo, which `tools/stamp_hover_gui.py` uses to pick a stamp by clicking on it.

## Using the result

```matlab
s = scene.stamps([scene.stamps.letter] == 'B');
pickTarget_mm  = s.disc_UVW_mm;        % [U V W], where the magnet lands
inkTarget_mm   = scene.inkPad.UVW_mm;
paperCenter_mm = scene.paper.UVW_mm;   % plus scene.paper.yaw_deg
```

| Field | Contents |
|---|---|
| `scene.stamps(k)` | `letter`, `tagId`, `disc_UVW_mm`, `rho_mm`, `phi_deg`, `yaw_deg`, `source` (`"tag"` or `"onshape"`), `detected`, `onshapeResidual_mm`, `onshapeYawResidual_deg` |
| `scene.inkPad`, `scene.paper`, `scene.arm` | `tagId`, `UVW_mm`, `rho_mm`, `phi_deg`, `yaw_deg` (`arm` is `[]` if not configured or not seen) |
| `scene.tags(k)` | every detected tag: `id`, `role`, `centerUV_mm`, `yaw_deg`, `centerPx` (raw pixel, for Problem 2(a)), side-length check |
| `scene.pose` | camera pose `R`, `t`, `K`, table homography `H`, `cameraCenter_mm`, reprojection RMS, reference tag IDs |
| `scene.baseCheck` | base tag position recovered through the fitted pose vs its layout value (should be ≈ 0) |
| `scene.warnings` | non-fatal problems (holder not seen, side-length check failed, …) |

Stamps have no tags of their own, so the place to return a stamp to is the same `disc_UVW_mm` it was picked from.

## Photo rules

The camera calibration (`calibration/iphone-0.5-cameraparams.mat`) is only valid for the exact camera mode it was made with. Every photo must use that same mode:

- **iPhone 17 Pro, 0.5x lens, 5712×4284**, JPEG. In Settings → Camera → Formats, choose "Most Compatible".
- **Send the original file.** Don't crop, rotate or edit it. The iPhone stores photos sideways with an EXIF rotation flag. MATLAB's `imread` ignores that flag, both here and during calibration, so the two match.
- **All the tags must be in frame:** the base tag, every holder tag you need, the ink pad tag and the paper tag. Roughly overhead is best, but any angle where the tags are readable works.

The code compares the photo size with the calibration and stops with an explanation if they differ. That catches a 1x photo, a 48 MP photo, or a photo that was rotated during transfer.

## How it works

```
photo ─► readAprilTag ─► corner pixels ─► undistortPoints
                                              │
   base tag corners (+ OnShape holder tags) ──┤
                                              ▼
                         H (DLT) ─► split with K ─► camera pose R, t ─► refine
                                                            │
                         for each tag at height h:          ▼
                         H_h = K [r1  r2  h·r3 + t] ─► corners on plane W = h
                                                            │
                                                            ▼
                         centre, yaw (corner 0 → 1) ─► + offset in tag frame ─► target
```

1. **Detect** (`detectTags`): readAprilTag finds the tag36h11 tags. Each tag's 4 corner pixels are undistorted with the calibration, so all later math can assume an ideal pinhole camera.
2. **Reference homography** (`findHomography`): the base tag's world corners are known, because its centre and yaw relative to the base axis come from OnShape. The HW4 DLT, extended to N points, fits H from those correspondences. With `useHolderTagsAsReference`, holder tags that have OnShape positions are added as extra points. A larger reference spread means less extrapolation error far from the base tag.
3. **Camera pose** (`estimateTablePose`): with K known, H splits into the plane's pose, `[r1 r2 t] = λ K⁻¹ H` with `r3 = r1 × r2`. The result is snapped to the nearest true rotation, then refined by Gauss–Newton on pixel reprojection error.
   - **Handedness check:** if the camera comes out below the table, the corner order or yaw convention is mirrored, and the program stops instead of returning flipped coordinates.
4. **Any height** (`homographyAtHeight`): once the pose is known, `H_h = K [r1 r2 (h·r3 + t)]` is the homography for the plane W = h (the handout's H_h). Each tag is mapped back through the H_h for its own configured height. Raised tags therefore get no parallax error, and neither do targets at a different height, such as the stamp discs.
5. **Locate** (`locateTag`): the tag centre is the mean of its 4 corners on that plane. Its yaw is the direction of corner 0 → 1, the same convention as HW4. The mapped side length is checked against `tagSize_mm`.
6. **Targets** (`solveScene`): target = tag centre + the offset in the tag's frame, rotated by the tag's yaw (`applyTagOffset`, as in `hw4B3.m`). For stamps the offset is the tag → disc offset from Problem 1(a), at `discHeight_mm`.

## Configuration

Everything about the rack lives in [`config/layout.json`](../../config/layout.json). It is shared by both arms and never changes between runs. `null` means not measured yet; if a required value is `null`, the program stops and names it.

| Key | Meaning |
|---|---|
| `tagSize_mm` | measured black square of the printed tags (not the nominal size) |
| `baseTag` | `id`, `centerUV_mm` and `yaw_deg` of the base tag in the base frame (OnShape), `height_mm` |
| `inkPad`, `paper` | `id`, tag `height_mm`, `offsetTag_mm` (tag centre → point of interest, in the tag frame), `targetHeight_mm` (defaults to the tag height) |
| `arm` | optional tag on the arm, used only as a cross-check |
| `stamp` | `tagToDisc_mm` (Problem 1(a)), `discHeight_mm` |
| `stampHolders` | one entry per letter: `tagId`, `tagHeight_mm`, optional `onshapeCenterUV_mm` and `onshapeYaw_deg` |
| `holderSource` | `"tags"`: holders measured from the photo. `"onshape"`: holders from CAD, with their tags only cross-checked |
| `useHolderTagsAsReference` | also fit H to holder tags with OnShape positions (they must be at the base tag's height) |
| `tagCornerWinding` | `"ccw"` for readAprilTag (verified). Only change it if the program says the frame is mirrored |
| `sideLengthTolerance` | fraction of `tagSize_mm` before a tag is flagged (default 0.03) |

**Conventions**
- Lengths are in mm and angles in degrees.
- World frame: origin on the base axis at plate level, W up.
- Tag frame: x runs along detected corner 0 → 1, and y is 90° counterclockwise from x, seen from above.
- Yaw is the direction of the tag's x axis, measured from +U toward +V.

## Checking the result

The annotated photo in `logs/` is the fastest way to tell whether to trust a run:

- **Yellow 50 mm grid, red U axis and green V axis:** the base frame drawn back onto the photo, including lens distortion. If the grid isn't square with the plate, or the axes don't start at the arm's base, the base tag offset or yaw in the layout is wrong.
- **Magenta outlines:** detected tags, labelled with ID and role. A **red** outline means the side-length check failed. That usually means the wrong `height_mm` for that tag, because a raised tag looks bigger, or else a bad detection.
- **Coloured circles:** targets projected from their own heights: stamp discs in cyan, ink pad in orange, paper in blue. The disc circles should sit on the discs.

The printed numbers matter too:

- **Reprojection RMS:** a fraction of a pixel is normal.
- **Base tag check:** should be about 0.
- **Tag vs OnShape residual:** printed for each holder that has both. Use it in the report to justify `holderSource`, and as a term in the error budget.

## Files

| File | Purpose |
|---|---|
| `locateScene.m` | entry point: loads the photo, layout and calibration, then runs everything, prints and saves |
| `solveScene.m` | photo-independent core: tags + layout → pose, located tags, targets. Tested directly |
| `detectTags.m` | readAprilTag wrapper (from HW4), adds undistorted corners |
| `findHomography.m` | normalized DLT for N ≥ 4 points (from HW4, generalized) |
| `estimateTablePose.m` | H → R, t with K, reprojection refinement, handedness check |
| `homographyAtHeight.m` | pose → H_h for the plane W = h |
| `locateTag.m` | one tag → centre, yaw, side-length check at a given height |
| `applyTagOffset.m` | offset in a tag's frame → world [U, V] |
| `tagCornersWorld.m` | tag centre, yaw, size and height → its 4 world corners in detector order |
| `projectWorldToPixel.m` | world → pixel, optionally with lens distortion (for drawing) |
| `applyHomography.m` | apply a 3×3 homography to N points |
| `drawScene.m` | annotated photo |
| `locateStampHolder.m`, `solveStampHolder.m`, `loadStampHolder.m` | curved stamp holder: tags 1 and 3 -> 26 stamp tops (geometry in `config/stamp_holder.json`) |
| `loadLayout.m`, `requireValue.m`, `repoRoot.m` | config loading and checks |

## Tests

```
"C:\Program Files\MATLAB\R2024b\bin\matlab.exe" -batch "addpath(genpath('src')); runtests('tests')"
```

[`tests/testVision.m`](../../tests/testVision.m) projects a known layout through a known camera and checks that the pipeline:

- recovers every position and yaw exactly, including raised tags
- shows that adding a reference tag reduces error when the corners are noisy
- catches a mirrored corner order
- catches a missing ink pad tag
- catches a missing letter
- catches a wrong tag height
- catches an unfilled layout value

All 10 tests pass (including `solveTagTarget` with only the base tag and one other tag). The full photo path was also run on a real tag photo from HW4. That run checked the code end to end, not accuracy, since that photo was not taken with the calibrated lens.

## Not done yet

- **Real values in `layout.json`:** tag IDs, base tag offset and yaw, tag → disc offset, heights, measured tag size.
- **Accuracy check on the bench:** tape tags at ruler-measured positions, run, and compare. Also raise one tag on a block to confirm the parallax correction.
- **IK, sequencing and servo commands:** these consume `scene` and are not part of this module.
