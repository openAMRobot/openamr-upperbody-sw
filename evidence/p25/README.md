# P25: the placeholder geometry (21 Oct 2026)

What V01 found by looking at the renders of the lift model, and what was done about each. The record in words is the last section of
`docs/v01_visual_check.md`; this directory holds the numbers and the pictures behind it. **Nothing here changes a mount pose, a stop height, a joint or a frame.**

| file | what |
|---|---|
| `measure_p25.py`, `p25_measurements.json` | the overlap of the column's two stages, the arm profile against the column, the arm mounts against the profile, and the re-parent experiment, at six configurations (three lift stops x two base plates), computed from the expanded URDF. `python3 measure_p25.py out.json` with the workspace sourced |
| `rviz/rviz_L1000_bp050.png`, `.json` | real RViz2 (the `capture_rviz.py` of P19) at `L1000`, `bp050`; the lift joint read back from `/joint_states` (0.000 m); stack stopped, 0 processes left |
| `gazebo/gazebo_L1000_bp050.png`, `.json` | the real Gazebo GUI (`capture_gazebo.py`) at the same configuration, camera 1.05 / -0.90 / 1.00 looking at 0.2 / 0 / 0.8; lift joint 0.000 m; 0 processes left. `first_far_*` and `second_mid_*` are the two earlier, more distant cameras of the same run, kept because the first one had too few box pixels to measure |
| `colour_compare.py`, `colour_compare.json` | the colour of the boxes in the two captures against the URDF's rgba, and the P19 capture (red) before |
| `after_material_vs_p24.txt`, `compare_after_p24.py` | the three A6 and three A7 reports of P25 against the three and three of P24, check by check |
| `rviz_capture.log`, `gazebo_capture.log` | what the capture scripts printed |

The Gazebo capture drew the boxes the same hue as RViz and the URDF but greyer and darker (saturation 0.23 against 0.42): see `colour_compare.json` and the V01 record. The
P19 captures in `../renders/` are not replaced; they show the boxes red, which is the "before".
