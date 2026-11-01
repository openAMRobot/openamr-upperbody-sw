<!-- SPDX-License-Identifier: MIT -->
# M07: the per-joint rest table Alex asked for (P26)

Alex asked for this on 16 Oct in his "M07 resting-state decision needed" comment on PR #14 (as P26 quotes it; this file was written from the P26 text, the comment itself was not
re-read). It is **input for Alex and Om, not a proposal**: no rule is suggested here, and no test, datum, margin or URDF was changed to produce it. Nothing was simulated for this file:
every reading is from a run recorded earlier (section 3 says which, and what that costs).

Conventions. Joint positions in rad, raw `/arm/joint_states` (the joint_state_broadcaster's own topic, not the clamped copy). **Distance** is M07's own definition
(`test_m07_rest_position_margin.py`, `rest_rows`): `min(lowest reading - lower limit, upper limit - highest reading)`, so **negative means a reading past the limit**. "Nearest limit" says which
limit that minimum is against. Limits come from the **expanded** `combined_robot.xacro` (`hardware_profile:=gazebo`) of each body, stored here as
`expanded_new_gazebo_ba5b4ea.urdf` and `expanded_old_7daa2ab_gazebo.urdf`; they are equal to each other and to upstream's own expansion on all 18 OpenArm joints that have limits
(`rest_readings.json`, key `all_openarm_joints_with_limits_identical_in_the_three_expansions`).

## 1. The table

Current body = this branch at `ba5b4ea` (code `ecdfcf2`; the three runs were made on `1165166`, no later code change), lift column, Gazebo Harmonic, Bullet-Featherstone, headless, platform-sw `dfd44a5`.
Old body = `7daa2ab` (the mast body before the lift), same engine, platform-sw `174731c`.

**Window and sample count, stated.**
Current: after the five controllers were active, **6.0 s of simulated time to settle, then a 6.0 s window, 600 readings per joint** (all 16 expected joints in every message; M07 requires at least 50 per joint), **3 runs**;
nothing commanded. The extremes below are the lowest and the highest of those 3 x 600 readings.
Old: **25 s of simulated time to settle, then 300 readings per joint**, **3 runs**, nothing commanded (P17's effort probe, `evidence/m02_bisect/scripts/effort_reader.py`). **The two bodies were not read with the same window**
(see section 3): the old 300 readings span about 3 s at the 100 readings per simulated second that M07's 600-in-6.0-s implies, not the 6 s that `RESULTS.md` says; the duration was not recorded.

<!-- BEGIN table1 -->
| body | joint | URDF lower [rad] | URDF upper [rad] | lowest raw reading, 3 runs [rad] | highest raw reading, 3 runs [rad] | readings per joint, runs | nearest limit | signed distance to it, smallest .. largest over the 3 runs [rad] (negative = past the limit) | startup position set by |
|---|---|---|---|---|---|---|---|---|---|
| current body (ba5b4ea, lift) | `openarm_left_finger_joint1` | +0.0000 | +0.7854 | +1.329e-04 | +4.163e-04 | 600 x 3 runs | lower | +1.329e-04 .. +4.160e-04 | `ros2_control.xacro:43`, `initial_value` 0.0 (see section 2) |
| current body (ba5b4ea, lift) | `openarm_right_finger_joint1` | -0.7854 | +0.0000 | +2.195e-08 | +2.755e-08 | 600 x 3 runs | upper | -2.755e-08 .. -2.654e-08 | `ros2_control.xacro:43`, `initial_value` 0.0 (see section 2) |
| current body (ba5b4ea, lift) | `openarm_left_joint4` | +0.0000 | +2.4435 | +1.002e-04 | +1.010e-04 | 600 x 3 runs | lower | +1.002e-04 .. +1.009e-04 | `ros2_control.xacro:43`, `initial_value` 0.0 (see section 2) |
| current body (ba5b4ea, lift) | `openarm_right_joint4` | +0.0000 | +2.4435 | +9.479e-05 | +9.544e-05 | 600 x 3 runs | lower | +9.479e-05 .. +9.530e-05 | `ros2_control.xacro:43`, `initial_value` 0.0 (see section 2) |
| old body (7daa2ab, mast) | `openarm_left_finger_joint1` | +0.0000 | +0.7854 | +5.001e-06 | +8.428e-05 | 300 x 3 runs | lower | +5.001e-06 .. +8.425e-05 | `ros2_control.xacro:42` at 7daa2ab, `initial_value` 0.0 (see section 2) |
| old body (7daa2ab, mast) | `openarm_right_finger_joint1` | -0.7854 | +0.0000 | -2.436e-05 | -1.385e-05 | 300 x 3 runs | upper | +1.385e-05 .. +2.430e-05 | `ros2_control.xacro:42` at 7daa2ab, `initial_value` 0.0 (see section 2) |
| old body (7daa2ab, mast) | `openarm_left_joint4` | +0.0000 | +2.4435 | -7.353e-07 | +1.401e-07 | 300 x 3 runs | lower | -7.353e-07 .. -4.588e-07 | `ros2_control.xacro:42` at 7daa2ab, `initial_value` 0.0 (see section 2) |
| old body (7daa2ab, mast) | `openarm_right_joint4` | +0.0000 | +2.4435 | +4.475e-06 | +4.794e-06 | 300 x 3 runs | lower | +4.475e-06 .. +4.722e-06 | `ros2_control.xacro:42` at 7daa2ab, `initial_value` 0.0 (see section 2) |
<!-- END table1 -->

Evidence behind each column.

| column | current body | old body (7daa2ab) |
|---|---|---|
| URDF lower / upper | `joint_limits.json` and `expanded_new_gazebo_ba5b4ea.urdf` (`<limit lower upper>` of the four joints) | `joint_limits.json` and `expanded_old_7daa2ab_gazebo.urdf`; limits that coincide with the eight named poses are also in `evidence/w03/old_body_7daa2ab.json` |
| lowest / highest raw reading, readings, window | `evidence/reports/20261021T085748Z_a7.json`, `...T090537Z_a7.json`, `...T091325Z_a7.json`, M07 `details.joints[]` (`min_rad`, `max_rad`, `readings`), `details.window_sim_s`, `settle_sim_s`, `messages_in_window` | `evidence/m02_bisect/effort_old1.json`, `effort_old2.json`, `effort_old3.json`, `joints.<name>.position.{min,max}`, `.n`, `.messages` |
| nearest limit, distance | computed in `build_table.py` from the two columns above (M07's formula); per run in `rest_readings.json` and section 5 | same |
| startup position set by | section 2 | section 2 |

What the table says, without interpretation: in the current body **three of the four joints rest 9.5e-5 to 4.2e-4 rad inside their limit, and the right finger rests 2.2e-8 to 2.8e-8 rad past its upper limit (0)**, in all
three runs (and in the three runs of `15109f9`, section 5). In the old body the right finger rests 1.4e-5 to 2.4e-5 rad inside it, the left `joint4` rests past its lower limit by up to 7.4e-7 rad, and the
other two are 5.0e-6 to 8.4e-5 (left finger) and 4.5e-6 to 4.8e-6 (right `joint4`) inside. The M07 margin is `joint_rest_margin_rad` = 1e-3 rad (datum, `placeholder`): every row above is inside it, in both bodies.
Within one run the spread between lowest and highest reading is at most 2.8e-7 rad for the current body (median 1.3e-7 to 1.7e-7) and 8.6e-7 for the old (median 6.5e-8); between runs it is larger for the left finger (1.3e-4 .. 4.2e-4 current, 5.0e-6 .. 8.4e-5 old). All from `rest_readings.json`.

## 2. Where the startup position comes from

The same answer for all four joints, and for both bodies:

1. **A file sets the value, to zero.** The Gazebo profile of `ros2_control.xacro` declares, for each of the sixteen arm joints, `<state_interface name="position"><param name="initial_value">0.0</param>`
   (macro `openamr_gazebo_arm_joint`, line 43; used for these four joints at lines 110, 114, 118, 122). The old body has the same line at `ros2_control.xacro:42` (`git show 7daa2ab:ros2/src/openamrobot_upperbody_description/urdf/ros2_control.xacro`).
   It copies upstream: `openarm.ros2_control.xacro` (openarm_description @ 14ff67b) `configure_joint`, with `initial_position="0.0"` for `joint4` (line 93) and `finger_joint1` (line 99), `initial_value` at line 76.
2. **No other file sets any joint position at startup.** A search of `*.xacro`, `*.yaml`, `*.py`, `*.urdf`, `*.srdf`, `*.sdf` under `src/` for `initial_value` / `initial_pos` finds, besides these, only upstream's own xacros and platform-sw's Nav2 `initial_pose` (the base's map pose, not a joint); the spawn (`sim.launch.py:350-352`)
   passes `-name -topic -x -y -z` and no joint values; M07 commands nothing, and the SRDF poses (`home`, `park`, `open`, `closed`) are not applied at startup.
3. **What is not set by any file is the offset from zero.** The readings are 1e-8 to 4e-4 rad away from the 0.0 that every file says, always on the side the limit allows, except the right finger and the old left `joint4`, which
   read past it. That offset is where the solver and the position-holding controller settle against the limit stop: the solver's resting state.
4. **Not verified:** whether gz_ros2_control pushes `initial_value` into the physics joint or only seeds the state interface (the plugin library contains the string "found initial value"; its source was not read; with 0.0 it is indistinguishable from the SDF default of zero),
   and what the `JointTrajectoryController` / `GripperActionController` hold at activation (their source was not read). Either way every file gives zero, and zero is the limit of these four joints.

## 3. What the two bodies' windows do and do not allow

* Same topic, same engine, same four joints, same controllers file. Different harness (M07's test, 6 s settle + 6 s window, against P17's probe, 25 s settle + 300 readings), different platform-sw pin
  (`dfd44a5` against `174731c`), different sample count. A difference of 1e-5 rad between the bodies is inside the between-run scatter of the current body's own right finger (9e-6, `RESULTS.md`), so these rows
  show *that* each body rests within about 4e-4 rad of the stop, not that one body rests nearer than the other for any row with a gap of that size.
* **A same-window, same-harness measurement of both bodies was not made.** I tried to run three fresh headless runs of the current body with P17's probe (to remove the harness difference on the current side); the launch was refused
  by this session's permission classifier ("Interfere With Workloads") and I did not retry or work around it. The old body would also need an overlay build of `7daa2ab`. Both are one command each if wanted.

## 4. Upstream's joint definition and the open target (kept separate from the table)

**4.1 The joint definitions are upstream's, unchanged.** In upstream's own expansion (`openarm_v20.urdf.xacro` @ 14ff67b, `expanded_upstream_openarm_v20_14ff67b.urdf`):
`finger_joint1` is [0, +0.7854] on the left hand and [-0.7854, 0] on the right, `joint4` is [0, +2.4435] on both arms. Our current and old expansions give identical limits for all 18 joints with limits
(`rest_readings.json`). So the stop each joint rests on is upstream's, not something the composition introduced.

**4.2 The startup value and the arm rest pose are upstream's too.** Upstream's `configure_joint` starts every joint, `joint4` and `finger_joint1` included, at 0.0 (section 2, item 1). Upstream's own SRDF
(`openarm_bimanual.srdf` @ 4e837e1) has `home` with `joint4` = 0 (lines 44-61; `joint4` at :48 and :57) and `hands_up` with `joint4` = 1.5 (`joint4` at :67 and :76): upstream's named home for the elbow is its lower limit. Our `home` is a copy of it (`openamrobot.srdf:50-67`, "imported from upstream").
Upstream's real-hardware plugin drives the arm joints to 0.0 **on activation** (`on_activate` calls `return_to_zero()`, `openarm_simple_hardware.cpp:232`, `:305-318`). For the gripper the same function sends the constant
`GRIPPER_JOINT_0_POSITION` = **0.044** (`:319`, `:345`; `openarm_simple_hardware.hpp:107`), which is the prismatic parallel gripper's stroke in metres; what that does to the pinch gripper, whose joint is in rad, was not traced. So upstream's own startup target
for the gripper is not documented as 0 on the real robot, and the simulator's 0.0 is not a copy of it.

**4.3 Upstream defines no gripper open or closed target at all.** Its SRDF has no gripper `group_state` (the only `group_state`s are `home` at lines 44 and 53 and `hands_up` at 63 and 72, all arms); a search of `openarm_ros2` and `openarm_description` for
gripper open/close states finds none for the pinch gripper. The pose called `open` is **ours**: `openamrobot.srdf:152` (left) and `:158` (right), both **0**, with `closed` at **+0.7854 / -0.7854** (`:155`, `:161`).
The SRDF comment at lines 147-151 gives the reason ("`open` is the end both ranges share (0)"). ADR-009 point 5's "upstream's convention" refers to the **opposite signs** of the two hands' ranges, not to which end is open.
Upstream's hardware code says nothing about the pinch gripper's direction: its comment for it is stale ("joint 0-1.5708 rad -> motor 0-1.5708", `openarm_simple_hardware.cpp:377`, while the joint range is 0.7854), and its only open/closed statement
("Joint 0=closed -> motor 0 rad, Joint 0.044=open", `:397`, inside commented-out code) is for the other end effector, the prismatic parallel gripper.

**4.4 What the model's own geometry shows about which end is which.** (`finger_gap.py`, `finger_gap_output.txt`, `finger_open_close.png`; forward kinematics of the two finger collision meshes of upstream's `pinch_gripper`, with `finger_joint2`
following `finger_joint1` by multiplier 1, the same numbers from upstream's own expansion and from ours.) The distance between the two fingertips (centroids of the 2 mm of each finger farthest from its pivot):

| `finger_joint1` (left hand; right hand is mirrored, -q gives the same numbers) | fingertip separation |
|---|---|
| 0 (the stop both hands rest on) | **7.25 mm** |
| 0.1963 | 47.19 mm |
| 0.3927 | 86.70 mm |
| 0.7854 (the far end) | **158.44 mm** |

In the model, **q = 0 is the end where the two fingers are together, and |q| = 0.7854 is the wide-apart end.** This is geometry computed here from the collision meshes; it is not a measurement of the real gripper and not a statement in upstream's
documentation, and I did not confirm it against a photograph (upstream's images are not in the checkout). If it is right, the stop the fingers rest on at startup is the **closed** end of the model's travel, and the SRDF word `open` (0) names the
end at which the model's fingers touch. That is for someone with the hardware, or Enactic, to confirm; it is reported here because P26 and the register wording speak of "the open stop".

**4.5 So: upstream's design, or our composition's effect?** Split in two, because the evidence differs.

* **That each of the four joints sits on a stop at rest is upstream's design, by construction.** Upstream's expansion has the limit at 0 for all four joints, starts them at 0.0, and calls `joint4` = 0 its `home`. Our composition changes none of that
  (sections 2 and 4.1), and the old body, with the same upstream pins, rests within 8.4e-5 rad of the same four stops (the largest old distance in the table; `RESULTS.md` and the PR #14 body say 2.5e-5, which its own P17 table contradicts for the left finger: 8.4e-5 and 4.7e-5 in runs 1 and 2). What the pose is *called* at the finger (`open`) is our choice (4.3), and 4.4 says the label may be on the wrong end.
* **Where within about 1e-4 rad of the stop it settles, and on which side, differs between the bodies**: the right finger moved from 1.4e-5..2.4e-5 inside to 2.2e-8..2.8e-8 past; the left `joint4` from 4.6e-7..7.4e-7 past to 1.0e-4 inside; the left finger from 5.0e-6..8.4e-5 to 1.3e-4..4.2e-4 inside
  (all with the window caveats of section 3). That is an effect of the composition (or of the platform-sw pin, which also differs between the two sets of runs; not separated). **Its cause is not known**: P17 measured the finger effort in both bodies and found the same sign,
  and `RESULTS.md` records "cause unknown". Not measured: upstream's own bimanual model at rest in Gazebo without our base and lift, which is what would say whether a sub-1e-3 offset is there too.

## 5. Every run, for the record

Readings per joint: 600 (current, 6.0 s window) and 300 (old). Both sets of current runs are on platform-sw `dfd44a5`; the `15109f9` set is the P24 runs, before the P25 material change (A7 reports record the git hash).

<!-- BEGIN table2 -->
| set, run | joint | lowest [rad] | highest [rad] | readings | nearest limit | signed distance [rad] | file |
|---|---|---|---|---|---|---|---|
| current, 1165166 run 1 | `openarm_left_finger_joint1` | +1.329e-04 | +1.331e-04 | 600 | lower | +1.329e-04 | `evidence/reports/20261021T085748Z_a7.json` |
| current, 1165166 run 1 | `openarm_right_finger_joint1` | +2.389e-08 | +2.755e-08 | 600 | upper | -2.755e-08 | `evidence/reports/20261021T085748Z_a7.json` |
| current, 1165166 run 1 | `openarm_left_joint4` | +1.002e-04 | +1.003e-04 | 600 | lower | +1.002e-04 | `evidence/reports/20261021T085748Z_a7.json` |
| current, 1165166 run 1 | `openarm_right_joint4` | +9.479e-05 | +9.495e-05 | 600 | lower | +9.479e-05 | `evidence/reports/20261021T085748Z_a7.json` |
| current, 1165166 run 2 | `openarm_left_finger_joint1` | +3.319e-04 | +3.321e-04 | 600 | lower | +3.319e-04 | `evidence/reports/20261021T090537Z_a7.json` |
| current, 1165166 run 2 | `openarm_right_finger_joint1` | +2.438e-08 | +2.711e-08 | 600 | upper | -2.711e-08 | `evidence/reports/20261021T090537Z_a7.json` |
| current, 1165166 run 2 | `openarm_left_joint4` | +1.009e-04 | +1.010e-04 | 600 | lower | +1.009e-04 | `evidence/reports/20261021T090537Z_a7.json` |
| current, 1165166 run 2 | `openarm_right_joint4` | +9.530e-05 | +9.544e-05 | 600 | lower | +9.530e-05 | `evidence/reports/20261021T090537Z_a7.json` |
| current, 1165166 run 3 | `openarm_left_finger_joint1` | +4.160e-04 | +4.163e-04 | 600 | lower | +4.160e-04 | `evidence/reports/20261021T091325Z_a7.json` |
| current, 1165166 run 3 | `openarm_right_finger_joint1` | +2.195e-08 | +2.654e-08 | 600 | upper | -2.654e-08 | `evidence/reports/20261021T091325Z_a7.json` |
| current, 1165166 run 3 | `openarm_left_joint4` | +1.008e-04 | +1.010e-04 | 600 | lower | +1.008e-04 | `evidence/reports/20261021T091325Z_a7.json` |
| current, 1165166 run 3 | `openarm_right_joint4` | +9.522e-05 | +9.539e-05 | 600 | lower | +9.522e-05 | `evidence/reports/20261021T091325Z_a7.json` |
| current, 15109f9 run 1 | `openarm_left_finger_joint1` | +4.134e-04 | +4.136e-04 | 600 | lower | +4.134e-04 | `evidence/reports/20261020T120539Z_a7.json` |
| current, 15109f9 run 1 | `openarm_right_finger_joint1` | +2.115e-08 | +2.707e-08 | 600 | upper | -2.707e-08 | `evidence/reports/20261020T120539Z_a7.json` |
| current, 15109f9 run 1 | `openarm_left_joint4` | +1.002e-04 | +1.003e-04 | 600 | lower | +1.002e-04 | `evidence/reports/20261020T120539Z_a7.json` |
| current, 15109f9 run 1 | `openarm_right_joint4` | +9.483e-05 | +9.495e-05 | 600 | lower | +9.483e-05 | `evidence/reports/20261020T120539Z_a7.json` |
| current, 15109f9 run 2 | `openarm_left_finger_joint1` | +3.694e-04 | +3.697e-04 | 600 | lower | +3.694e-04 | `evidence/reports/20261020T121325Z_a7.json` |
| current, 15109f9 run 2 | `openarm_right_finger_joint1` | +1.967e-08 | +2.675e-08 | 600 | upper | -2.675e-08 | `evidence/reports/20261020T121325Z_a7.json` |
| current, 15109f9 run 2 | `openarm_left_joint4` | +1.009e-04 | +1.010e-04 | 600 | lower | +1.009e-04 | `evidence/reports/20261020T121325Z_a7.json` |
| current, 15109f9 run 2 | `openarm_right_joint4` | +9.529e-05 | +9.541e-05 | 600 | lower | +9.529e-05 | `evidence/reports/20261020T121325Z_a7.json` |
| current, 15109f9 run 3 | `openarm_left_finger_joint1` | +4.160e-04 | +4.163e-04 | 600 | lower | +4.160e-04 | `evidence/reports/20261020T122112Z_a7.json` |
| current, 15109f9 run 3 | `openarm_right_finger_joint1` | +2.195e-08 | +2.654e-08 | 600 | upper | -2.654e-08 | `evidence/reports/20261020T122112Z_a7.json` |
| current, 15109f9 run 3 | `openarm_left_joint4` | +1.008e-04 | +1.010e-04 | 600 | lower | +1.008e-04 | `evidence/reports/20261020T122112Z_a7.json` |
| current, 15109f9 run 3 | `openarm_right_joint4` | +9.522e-05 | +9.539e-05 | 600 | lower | +9.522e-05 | `evidence/reports/20261020T122112Z_a7.json` |
| old, 7daa2ab run 1 | `openarm_left_finger_joint1` | +8.425e-05 | +8.428e-05 | 300 | lower | +8.425e-05 | `evidence/m02_bisect/effort_old1.json` |
| old, 7daa2ab run 1 | `openarm_right_finger_joint1` | -1.393e-05 | -1.385e-05 | 300 | upper | +1.385e-05 | `evidence/m02_bisect/effort_old1.json` |
| old, 7daa2ab run 1 | `openarm_left_joint4` | -7.353e-07 | +1.209e-07 | 300 | lower | -7.353e-07 | `evidence/m02_bisect/effort_old1.json` |
| old, 7daa2ab run 1 | `openarm_right_joint4` | +4.722e-06 | +4.787e-06 | 300 | lower | +4.722e-06 | `evidence/m02_bisect/effort_old1.json` |
| old, 7daa2ab run 2 | `openarm_left_finger_joint1` | +4.724e-05 | +4.727e-05 | 300 | lower | +4.724e-05 | `evidence/m02_bisect/effort_old2.json` |
| old, 7daa2ab run 2 | `openarm_right_finger_joint1` | -2.436e-05 | -2.430e-05 | 300 | upper | +2.430e-05 | `evidence/m02_bisect/effort_old2.json` |
| old, 7daa2ab run 2 | `openarm_left_joint4` | -7.092e-07 | +1.312e-07 | 300 | lower | -7.092e-07 | `evidence/m02_bisect/effort_old2.json` |
| old, 7daa2ab run 2 | `openarm_right_joint4` | +4.475e-06 | +4.794e-06 | 300 | lower | +4.475e-06 | `evidence/m02_bisect/effort_old2.json` |
| old, 7daa2ab run 3 | `openarm_left_finger_joint1` | +5.001e-06 | +5.022e-06 | 300 | lower | +5.001e-06 | `evidence/m02_bisect/effort_old3.json` |
| old, 7daa2ab run 3 | `openarm_right_finger_joint1` | -1.451e-05 | -1.446e-05 | 300 | upper | +1.446e-05 | `evidence/m02_bisect/effort_old3.json` |
| old, 7daa2ab run 3 | `openarm_left_joint4` | -4.588e-07 | +1.401e-07 | 300 | lower | -4.588e-07 | `evidence/m02_bisect/effort_old3.json` |
| old, 7daa2ab run 3 | `openarm_right_joint4` | +4.708e-06 | +4.749e-06 | 300 | lower | +4.708e-06 | `evidence/m02_bisect/effort_old3.json` |
<!-- END table2 -->

## 6. What could not be measured, and what was not done

* A fresh same-window measurement of either body: refused (section 3); everything above is from recorded runs.
* Whether gz_ros2_control applies `initial_value` to the physics joint, and what the controllers hold at activation (section 2, item 4).
* Which direction opens the real gripper (section 4.4): geometry of the model only.
* Upstream's own model at rest in Gazebo (section 4.5), and the cause of the between-body offsets (P17: unknown).
* The duration of the old body's 300 readings: not recorded (section 1).
* Alex's 16 Oct comment itself: P26's description of it was used.

## 7. Files and how to reproduce

`build_table.py` (reads only committed files; `python3 evidence/m07/build_table.py` from the repository root regenerates the tables above and `rest_readings.json`, `joint_limits.json`), `rest_readings.json`, `joint_limits.json`,
`expanded_*.urdf` (`ros2 run xacro xacro .../combined_robot.xacro hardware_profile:=gazebo` for the current body, the same for the `7daa2ab` tree in an overlay, and `openarm_v20.urdf.xacro` for upstream),
`finger_gap.py` + `finger_gap_output.txt` + `finger_open_close.png` (its meshes are read from `src/upstream/openarm_description` @ 14ff67b by absolute path).
