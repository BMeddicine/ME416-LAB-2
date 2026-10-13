# Config

- **One file per arm** (Bryson's, Aviva's): servo fit (a, b) per joint, fit residual, backlash, joint limits, the name it spells. This is the only thing that changes between the two runs.
- **`layout.json`, shared**: everything about the rack, read by `src/vision/loadLayout.m`. `null` means not measured yet.
  - `baseTag`: ID, tag centre `[U, V]` and yaw in the base frame (from OnShape). This tag defines the world frame.
  - `inkPad`, `paper`, `arm`: tag ID, tag height, offset from the tag centre to the point of interest (in the tag's frame), and target height.
  - `stamp`: tag → disc offset (Problem 1(a)) and disc height.
  - `stampHolders`: one entry per letter, with its tag ID, tag height and optional OnShape position and yaw.
  - `holderSource`: `"tags"` (measured from the photo) or `"onshape"` (from CAD; tags are then only a cross-check). Justify the choice in the report. Whenever both are available, the scene log reports the tag-vs-OnShape residual.
  - `useHolderTagsAsReference`: also fit H to holder tags with OnShape positions. This reduces extrapolation error far from the base tag, but only works for holder tags at the same height as the base tag.
  - `tagSize_mm`: the measured printed square, not the nominal size.
  - `tagCornerWinding`: `"ccw"` for readAprilTag. Leave it alone unless the program says the frame is mirrored.

Conventions: lengths in mm, angles in deg. A tag's frame has x along detected corner 0 → 1 and y 90° counterclockwise seen from above. Yaw is the direction of that x axis, measured from +U toward +V.
