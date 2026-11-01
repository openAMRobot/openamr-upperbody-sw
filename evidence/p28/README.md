# P28: the lift column geometry (28 Oct 2026)

What was measured and recorded for the geometry items of PR #14 after Alex's answer of 22 Oct. The record in words is the last section of `docs/v01_visual_check.md`
and the CHANGELOG entry of P28; this directory holds the numbers and the pictures behind it.

| file | what |
|---|---|
| `measure_p28.py`, `v01_geometry.json` | the stages, the adapter plate, the arm profile and the mounts at six configurations (three lift stops x two base plates) from the expanded URDF (exact: every box is axis-aligned), the P25 figures beside them, and V01 item 3: MoveIt's collision check at each configuration, lift held at the stop, every other joint at its default, over all ENABLED pairs. `python3 measure_p28.py out.json ../p25/p25_measurements.json` with the workspace sourced |
| `rviz/rviz_<stop>_<bp>.png`, `.json` | six real RViz2 screenshots (`../renders/rviz/capture_rviz.py`, the P19 and P25 script): L1000, L1175, L1350 x bp000, bp050. Each `.json` has the lift joint read back from `/joint_states` (all six equal to the stop asked for) and the number of processes left after the stop (0) |
| `rviz_capture.log` | what the capture script printed |
| `run_exit_codes.txt` | the six runner exit codes and the first, abandoned attempt |
| `a7_run2_failures.txt` | the two FAIL lines of A7 run 2 |

The reports are in `../reports/` (`20261028T002351Z_dev.json` to `20261028T005455Z_a7.json`).

**V01 item 3 (written record of `v01_geometry.json`, `collision_check_item_3`):** 0 of 567 enabled pairs colliding at all six configurations (L1000, L1175, L1350 x bp000, bp050), with the lift held at the stop and every other joint at its default.
