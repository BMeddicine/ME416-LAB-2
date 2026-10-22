# Config

All lengths are in mm and all angles in degrees. Each file starts with a `_notes` array that explains its fields and records where the numbers came from.

| File | What | Read by |
|---|---|---|
| `arm_bryson.json` | Bryson's arm: shoulder offset (t1), tool offset from the forearm end I, base servo fit `s1 = a·θ0 + b` with a per-angle correction, reach table and reach correction `c(W)`, minimum tool height, and which `logs/servo_angle_points_*.csv` the joint 2/3 fits come from | `src/move_to_tag.py`, `src/stamp_hover_gui.py` (`--arm` picks another file) |
| `layout.json` | Camera and tag settings: default calibration, tag family and size, and tag 0's position in the base frame (which defines the world frame) | `vision/loadLayout.m`, `src/stamp_hover_gui.py` (tag 0 centre) |
| `stamp_holder.json` | Curved stamp holder from the OnShape CAD: holder tag IDs and heights, and the seat position and well size of all 26 stamps in the holder frame | `vision/loadStampHolder.m` |
| `stamp_job.json` | Stamp job: ink pad offset from tag 2, paper edge from tag 0, letter spacing, and press defaults (depth, dabs, rocking, dwell, press speed) | `src/stamp_hover_gui.py` |

Only Bryson's arm has a committed config file. Aviva's arm was not fully calibrated in the time available (see the report's arm comparison).

**Conventions:** world frame [U, V, W] = the arm's base frame. The origin is on the base axis at plate level, W is up, and U, V are parallel to tag 0's x, y edges. A tag's frame has x along detected corner 0 → 1 and y 90° counterclockwise seen from above. Yaw is the direction of that x axis, measured from +U toward +V.
