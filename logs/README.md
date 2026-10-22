# Logs

Calibration data and trial records from building and tuning Bryson's arm. The per-run vision results (`stampholder_*`, `tagtarget_*` JSON and annotated images) are written here too, but aren't committed.

| File | Written by | What |
|---|---|---|
| `servo_angle_points_corrected.csv` | `src/servo_angle_gui.py` | Joint 2/3 calibration points used for the final fits (`servoCalibration` in `config/arm_bryson.json`) |
| `servo_angle_points_base_servo.csv` | `src/servo_angle_gui.py` | Joint 1 (base) points behind the first `baseFit` |
| `servo_angle_points_incorrect_joints.csv` | `src/servo_angle_gui.py` | Points recorded before the pin swap was fixed (joints 1 and 3 mislabelled); kept for the record |
| `ik_limit_marks.csv` | `src/ik_gui.py` | Traced mechanism limit points; reloaded and drawn by the IK GUI |
| `tag_reach_trials.csv` | `src/move_to_tag.py` | Reach trials: commanded pose, servo values and measured miss; input to `move_to_tag.py --fit` |
| `tag_reach_trials_old.csv` | `src/move_to_tag.py` | Earlier trials, moved aside automatically when the CSV columns changed |
| `base_offsets.csv` | `src/stamp_hover_gui.py` | Base-only centring jogs over the stamps, behind `baseFit` and `baseCorrection` |
| `stamp_offsets.csv` | `src/stamp_hover_gui.py` | Base and reach centring jogs over the stamps, behind `reachTable` |
