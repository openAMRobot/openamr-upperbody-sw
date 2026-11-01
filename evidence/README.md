# Evidence: the final set (P-03 rev20.1 item 25, D9)

This directory keeps only the evidence that something in the repository still reads, names or matches. The rest of what PR #14 brought in
(passes P1 to P30) was removed from `main`; nothing of it is lost.

**Full history.** Every pass from P1 to P30, with every file this directory ever held, is on GitHub at `refs/pull/14/head`, commit
`462907556cbb9c770eb814dda436cc3abca29741`:

```
git fetch origin refs/pull/14/head
git show 462907556cbb9c770eb814dda436cc3abca29741:evidence/<path>
```

Links in `CHANGELOG.md`, `docs/pr/PR14_BODY.md` and `docs/verification.md` to files that are no longer here point into that history; read
them at that commit.

Everything here is A6 (static and mock) or A7 (simulation) development evidence, or a picture for a person to look at. None of it is
hardware, physical acceptance or release evidence.

## What stays, and what needs it

| Path | Read, named or matched by |
|---|---|
| `reports/*.json` (every committed A6 `*_dev.json` and A7 `*_a7.json`) | `tools/check_evidence_backing.py`: it loads every report matching `evidence/reports/*.json`, backs verdict counts against any committed report, and evaluates the `json_count` and `launch_exits` globs of `docs/pr/claim_backing.yaml` (for example `20261018T*_a7.json`, `20261028T*_a7.json`) over them. Removing a report changes those counts and gives MISMATCH. `test_a7_process_exits.py` names the `*_a7.json` pattern; `docs/issues/ros_gz-bridge_teardown_segfault.md` and `ros2/src/openamrobot_upperbody_moveit/README.md` name single reports |
| `reports/README.md` | `tools/check_evidence_backing.py` (`ROW_DOCS`: its table rows name reports next to verdict counts); `ros2/src/openamrobot_upperbody_gazebo/README.md` and `tools/README.md` |
| `p23/a7_run1_console_tail.txt` | a `record` entry in `docs/pr/claim_backing.yaml`; `docs/issues/ros_gz-bridge_teardown_segfault.md` |
| `p28/README.md` | a `record` entry in `docs/pr/claim_backing.yaml` |
| `shutdown_p17/orphan_count_36_runs.json` | a `record` entry in `docs/pr/claim_backing.yaml` |
| `shutdown_p17/logs/bare_empty_world_service_stop_30.txt`, `shutdown_p17/scripts/`, `shutdown_p17/timing_*.jsonl` | `docs/issues/gz-sim_server_teardown_segfault_and_hang.md` (the upstream issue draft and its reproduction) |
| `v01/` | `ros2/src/openamrobot_upperbody_description/scripts/build_evidence.py` (the V01 details of every A6 and A7 report: `evidence/v01/*.json`); `docs/pr/claim_backing.yaml` (`item_3_collision_counts.json`, `item_8_goal_vs_real_ee.json`) |
| `renders/` | `build_evidence.py` (`evidence/renders/`) |
| `renders/rviz/` | `.github/workflows/model-tests.yml` runs `capture_rviz.py` in place for the V01 RViz captures of job a7; `tools/v01_rear_captures/` derives from `capture.launch.py`, `capture.rviz` and `xgrab.py` |
| `p28/v01_geometry.json` | `build_evidence.py`; `docs/v01_visual_check.md` |
| `p28/measure_p28.py`, `p28/rviz/` (the latest RViz captures: the six configurations of the final geometry) | `docs/v01_visual_check.md` (the P28 record and the sheet for a person) |
| `p25/` | `reports/README.md` names the directory; `docs/v01_visual_check.md` names `p25/measure_p25.py`, `p25/p25_measurements.json`, `p25/rviz/` and `p25/gazebo/` |
| `m02_bisect/RESULTS.md` | `ros2/src/openamrobot_upperbody_bringup/test/a7_support.py` (why the joints at an upstream limit at rest are so by construction) |
| `w03/` | `ros2/src/openamrobot_upperbody_moveit/openamrobot_upperbody_moveit/named_poses.py` and the moveit package's `CMakeLists.txt` |

## What was removed

Files that no code, test, workflow, `docs/pr/claim_backing.yaml` glob or record, `docs/decision_citations.yaml`, `reports/README.md`,
`docs/issues/`, README or `docs/v01_visual_check.md` names or matches: the pass directories `p22_checks/`, `p23/` (except the console tail
above), `p24/`, `p27/`, `p29/` and the run logs of `p28/`; `frozen_a6_993b000/`; `m07/`; the raw runs and scripts of `m02_bisect/`; and two
unreferenced files of `shutdown_p17/`. `docs/decision_citations.yaml` mentions `evidence/` only to exclude it from its scan, and the
`evidence/` paths in `.github/workflows/model-tests.yml` other than `capture_rviz.py` are the workspace's output directory, not this one.
