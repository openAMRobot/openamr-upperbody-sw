# Sim-to-real protocol for the bench week (Proposed)

Status: **Proposed** (20 Nov 2026). Nothing here is accepted until the owners named in "Report" accept it. Scripts: [tools/sim2real/](../tools/sim2real/README.md).
Bench window: the power-on of 30 Nov to 4 Dec 2026.

## 1. Purpose and evidence class

The protocol compares one simulation reference with one bench measurement for five motions: lift, base straight, base rotation, arm tracking and gripper.

- **Simulation reference.** What the combined model does in Gazebo on a named commit and physics engine, measured with Gazebo ground truth (`gz model -m openamrobot -p` / `-l <link>`, the `- Pose` block). It describes the model, not the robot.
- **Bench measurement.** What the delivered robot does, measured with a tape, calipers, floor marks and a stopwatch, plus what its drivers report.
- **Neither is safety evidence.** A pass band here says a difference is small enough not to investigate. It says nothing about whether a motion is safe. Simulation results are not safety evidence (platform-fw #20, last paragraph), and the model permits no statement about a physical robot or its limits (ADR-009, `ros2/src/openamrobot_upperbody_bringup/launch/moveit.launch.py:32`).
- **Odometry is never the reference.** It is recorded so that its error can be measured. On DART, `/odom` over-reports base motion: V01 item 5 on `a3a93c6`, and T2/T3 below.

## 2. Preconditions and safety

In this order, before the first motion of the day:

1. **Hardwired E-stop tested first.** Press it, check that the arms, base and lift drop power, then release and reset. The scripts never touch the stop chain. This functional check does not replace the E-01 chain (platform-fw #20: "the hardwired E-01 safety chain stays independent of it").
2. **One operator at the stop, area clear.** One named operator holds the E-stop for the whole run (`--operator` is required with `--target robot`). Nobody inside the reach of the arms, or in the path of the base, during a motion.
3. **Arms in park before any lift or base motion (AD-30: base and arm never move together).** `t1_lift.py`, `t2_base_straight.py` and `t3_base_rotate.py` read `/joint_states` and refuse to start unless every arm joint is within 0.05 rad of SRDF `park` (`ros2/src/openamrobot_upperbody_moveit/config/openamrobot.srdf:156-190`: all zero, joint 4 at 0.1). For AD-30 in this repository, see `ros2/src/openamrobot_upperbody_bringup/launch/view.launch.py:23` and `ros2/src/openamrobot_upperbody_moveit/README.md:35`. `park` is a placeholder pose (`openamrobot.srdf:151-155`).
4. **Lift only through the authoritative controller interlock.** The real profile has no lift interface: "real nothing: no command interface, no state interface. The lift node firmware owns the lift (CTL-017)" (`ros2/src/openamrobot_upperbody_description/urdf/ros2_control.xacro:21-22`; `ros2/src/openamrobot_upperbody_control/config/arm_controllers.yaml:32-35`). The interlock (base stopped and both arms in a safe pose, enforced by the base controller firmware) is platform-fw #20, open, owner `Evans-Kwashie`. **`t1_lift.py --target robot` sends nothing.** The operator moves the lift through that controller and presses Enter at the start and end of each move. If #20 is not implemented on the bench robot, T1 on the robot is NOT RUN.
5. **Speed caps** (`tools/sim2real/common.py`, constants; a request above a cap is clamped down and the clamp is written to `session.yaml`):

   | what | cap |
   |---|---|
   | base linear | ≤ 0.2 m/s |
   | base angular | ≤ 0.5 rad/s |
   | arm velocity scaling (fraction of each joint's URDF velocity limit) | ≤ 0.1 |
   | gripper `max_effort` | ≤ 5 N |

6. **What the scripts do and do not do.**
   - They send goals and Twists only through existing controllers: the arm and gripper controllers of `arm_controllers.yaml` and the base velocity topic.
   - They never bypass a stop.
   - With `--target robot`, they ask the operator to type `yes` before **every** motion; anything else aborts.
   - On Ctrl+C they cancel their action goals and publish a zero Twist (`common.StopGuard`, `rosio.Io.cancel_all` / `zero_twist`).
   - `--dry-run` sends nothing and logs every intended send to `sent.jsonl` (tested: `tools/sim2real/tests/test_dry_run.py`).

## 3. Interfaces

Each name is read from this repository at `a3a93c6` unless marked otherwise. A name the real profile does not define is **TO BE CONFIRMED ON THE BENCH**. The scripts take it as a required argument with no default.

| what | sim (Gazebo) | robot | source |
|---|---|---|---|
| arm controllers | `/left_arm_controller`, `/right_arm_controller` (FollowJointTrajectory, position command) | same names: one controller file for mock, gazebo and real | `arm_controllers.yaml:28-30, 44-48, 59-89` |
| gripper controllers | `/left_gripper_controller/gripper_cmd`, `/right_gripper_controller/gripper_cmd` (GripperCommand), joint `openarm_<side>_finger_joint1` | same names | `arm_controllers.yaml:50-54, 91-97` |
| arm driver (real) | n/a | upstream `openarm_hardware/OpenArmHW` (CAN-FD), `ee_type="pinch_gripper"` | `ros2_control.xacro:9, 62-77` |
| lift | `/lift_controller/follow_joint_trajectory`, `lift_joint` 0 / 0.175 / 0.350 m | **none in this repository**; command through the authoritative lift controller, state topic TO BE CONFIRMED ON THE BENCH (owner: `Evans-Kwashie`, platform-fw #20) | `ros2_control.xacro:17-22, 140-141`; `openamrobot.srdf:135-143` |
| base velocity | `/cmd_vel` (platform-sw gz bridge) | TO BE CONFIRMED ON THE BENCH (owner: platform-sw code owners `skunal3318`, `panthera-momagdii`). platform-sw's real-robot doc names `/cmd_vel` on the micro-ROS agent | platform-sw `ros2/src/openamrobot_gazebo/config/gz_bridge.yaml:30-34`; platform-sw `docs/real_robot/00_overview.md:70` @8e7a454 |
| base odometry (recorded only) | `/odom` (DiffDrive) | TO BE CONFIRMED ON THE BENCH (same owners). platform-sw's doc names `/odom/unfiltered` (agent) and `/odom` (EKF) | `gz_bridge.yaml:16-20`; `00_overview.md:70, 73` |
| joint states | `/joint_states` (joint_state_broadcaster) | `/joint_states`; arm effort is the motor torque, gripper velocity and effort are reported as **0** by the pinned driver | openarm_ros2 `4e837e1` `openarm_hardware/src/openarm_simple_hardware.cpp:263-264, 277-278` |

## 4. Tests

Every pass band below is **Proposed (owner to accept)**. They live in `tools/sim2real/bands.yaml` with a reason each. Run each script with `--dry-run` first and read its `sent.jsonl`.

### T1 lift

- **Commanded:** L1000 → L1175 → L1350 → L1000 (lift_joint 0, 0.175, 0.350, 0 m; `openamrobot.srdf:135-143`; `upperbody_configuration.yaml:62-64`; stroke 0.350 m, `upperbody_configuration.yaml:31`), both arms in park. In sim, one goal per stop, timed at the datum speed `lift_max_velocity_dold_v4_350` = 0.018 m/s (`datums.yaml:71-77`). On the robot, the operator commands the lift through the authoritative controller (section 2, item 4).
- **Measured on the bench:**
  - travel time per move (stopwatch, or the script's Enter presses);
  - final height at each stop by steel tape, from the floor to the **top face of `adapter_plate_link`** (the face `arm_profile_link` stands on; plate section 0.425 × 0.100 m, `datums.yaml:105-106`, thickness 0.010 m, `upperbody_configuration.yaml:68`), at the plate's front edge and at its rear edge;
  - the lift state topic if one exists.
- **Sim reference:** `lift_joint` from `/joint_states`, and the Gazebo pose of `lift_carriage_link`. `adapter_plate_link` sits on a fixed joint and is lumped into the carriage, so its top face = carriage z + `adapter_plate_joint` z + half the plate height, read from `/robot_description`.
- **Metrics:**
  - speed = Δheight / travel time, against 0.018 m/s. That figure is a no-load catalogue speed; the datum note says the real column is slower under load (`datums.yaml:77`). Effort datum 800 N (`datums.yaml:79-85`) for reference.
  - position error per stop, relative to the L1000 reading (stroke error);
  - **sag** with the arms in park = rear edge height − front edge height.
- **Proposed bands:** speed ratio 0.5 to 1.05; stroke error ±5 mm; sag ±3 mm.

### T2 base straight

- **Commanded:** 1.0 m forward, pause, 1.0 m back, at 0.2 m/s (Twist for distance / speed s of the node clock, then zero), both arms in park.
- **Measured on the bench:**
  - floor marks under the **front face centre of `base_link`'s collision box** before and after each leg. The box is 0.780 × 0.580 × 0.240 m and centred on `base_link`, so the face centre is x +0.390 m, y 0 (expanded URDF at `a3a93c6`). The model has no bumper frame.
  - the distance between the marks along the start heading, by tape;
  - lateral drift, perpendicular to the start heading;
  - odometry, recorded only.
- **Metrics:** real / commanded and sim / commanded distance; lateral drift; odometry / ground truth (sim) and odometry / tape (bench).
- **Proposed bands:** real / commanded 0.95 to 1.05; lateral drift ±0.05 m over 1 m.

### T3 base rotation

- **Commanded:** +90° then +180° in place at 0.5 rad/s, arms in park.
- **Measured on the bench:** floor marks under the two front corners of `base_link`'s collision box, A (x +0.390, y +0.290) and B (x +0.390, y −0.290), before and after each turn. Mark coordinates are taken on a taped floor grid.
- **Angle by trigonometry.** With u₀ = B₀ − A₀ and u₁ = B₁ − A₁:

  θ = atan2(u₀ₓ·u₁ᵧ − u₀ᵧ·u₁ₓ, u₀ₓ·u₁ₓ + u₀ᵧ·u₁ᵧ)

  This is signed, counter-clockwise positive. For the 180° turn a result near −π is read as +π. The formula is `common.floor_turn_angle` and `compare.floor_turn_angle`, and it is tested.
- **Metrics:** turn ratio, real / commanded and sim / commanded; odometry / ground truth. Compare with #75's DART finding that, with the arms in park, every spin turns 4.0 to 4.1 % less than r(ωR − ωL)/track (#75, "Base yaw", on #75's branch).
- **Proposed band:** turn ratio 0.95 to 1.05. A 4 % deficit sits inside this band, so the band alone cannot say whether the real base shares it: read the bench value next to the sim value.

### T4 arm tracking

- **Commanded:** per arm, SRDF `home` → `hands_up` → `park` (`openamrobot.srdf:51-87, 156-173`). One FollowJointTrajectory goal per segment, timed so the slowest joint moves at 0.1 of its URDF velocity limit (the meaning of MoveIt's velocity scaling for one straight segment).
- **Measured:**
  - the controller's own reference against feedback (`~/controller_state`), and `/joint_states`;
  - per joint and segment: RMS and max |reference − feedback|; settle time after the trajectory end until every joint is within 1e-3 rad of the goal;
  - at park: the M07 rest margin of every arm joint (distance to the nearer URDF limit) against the datum `joint_rest_margin_rad` = 1e-3 rad, a placeholder (`datums.yaml:173-176`). M07 itself is the Gazebo check of `docs/verification.md:110`.
  - at each held pose: the mean `/joint_states` effort per joint (driver-reported motor torque on the robot), next to the **static model gravity torque** from the URDF masses and centres of mass at the measured joint positions (`common.gravity_torques`).
- **A12 input for #73.** The effort-against-model table (`t4_holds.csv`) is the A12 input for #73's gravity feed-forward. In sim the two differ by at most 0.109 N·m over the 42 joint-pose rows (largest at `openarm_right_joint1`, park), which checks the model-torque computation against the simulator. The residual is not explained here: the joint friction the simulator profile of `37b93f5` adds is a candidate, not checked. On the bench the same columns give driver torque against model torque.
- **Proposed bands:** RMS ≤ 0.01 rad; max ≤ 0.02 rad (V01 item 6's tolerance, `docs/v01_visual_check.md:97-100`); settle ≤ 2 s; M07 margin at park ≥ 1e-3 rad.

### T5 gripper

- **Commanded:** both hands, `open` / half / `closed` (fractions 1, 0.5, 0 of each hand's far limit), `max_effort` 5. Left `open` = +0.7854 rad, right `open` = −0.7854 rad, `closed` = 0 (`openamrobot.srdf:191-208`).
- **Measured on the bench:**
  - jaw opening between the fingertips with a vernier caliper at each command;
  - the sign per side (the hand opens for the far-limit command);
  - time to settle;
  - the **raw command values sent** (in `sent.jsonl` and `t5_gripper.csv`).
- **Units, #79.** #79 was opened as: model revolute 0 to 0.785 rad, pinned openarm_ros2 driver prismatic 0 to 0.044 m.
  - Decision (Alex, 17 Nov 2026, on #79): radians everywhere.
  - The real profile passes `ee_type="pinch_gripper"` (`ros2_control.xacro:72`). With it the pinned driver maps finger radians to motor radians 1:1; the 0 to 0.044 m mapping is the `parallel_link` branch only (`openarm_simple_hardware.cpp:375-392`).
  - **Still open on #79:** the motor-angle against finger-angle table measured on the real gripper (5 or more points; the linkage may not be 1:1). Run `t5_gripper.py --fractions 1,0.75,0.5,0.25,0` for it.
  - Also open on #79: `return_to_zero()` sends 0.044 as the gripper target, which is 0.044 rad in `pinch_gripper` mode.
  - The raw values are recorded as sent, in radians.
- **Sim reference:**
  - `/joint_states` finger joints and the Gazebo roll of `openarm_<side>_ee_link1`;
  - the model jaw opening at the measured angle, as the smallest distance between the distal thirds of the two finger collision meshes (the M05 method of `ros2/src/openamrobot_upperbody_moveit/test/test_gripper_sign.py:69`; SRDF comment: 0.1 mm at 0, 139 mm at the far end, `openamrobot.srdf:194`).
  - DART does not move either gripper on `a3a93c6` (#101; cause: the finger `<dynamics>` of `37b93f5`), so T5 is also run on Bullet-Featherstone.
- **Proposed bands:** sign correct on every command; settle ≤ 2 s; caliper − model jaw opening ±5 mm.

## 5. Data and file layout

- **One session directory per run:** `<session-root>/<UTC stamp>_<test>_<target>/`.
- **`session.yaml` in each one:**
  - target, dry-run flag, operator, argv;
  - the **`config_id`** of the URDF in use from `ros2/src/openamrobot_upperbody_description/scripts/print_config_id.py` (`hardware_profile:=gazebo` for sim, `:=real` for the robot), and the command that produced it;
  - the git SHA and dirty flag of this repository;
  - `ROS_DOMAIN_ID`, `GZ_PARTITION`, the caps, every clamp applied, and the AD-30 park check.
- **The other files:**
  - `sent.jsonl` lists every goal and Twist sent (or that would have been sent);
  - `metrics.csv` (test, metric, value, unit, source) is what `compare.py` reads;
  - per test there is one CSV of steps, plus `joint_states.csv` (and `odom.csv` for T2 and T3).
- **Manual measurements:** one CSV template per test in `tools/sim2real/templates/` (`t1_lift_manual.csv` … `t5_gripper_manual.csv`). Copy them into the session's `manual/` directory and fill them in at the bench; each names its instrument.
- **Robot-side record:** add the SHAs of the other repositories running on the robot (platform-sw, firmware) to `session.yaml`'s `note` until the scripts can read them.

## 6. Report

```bash
python3 tools/sim2real/compare.py --sim <sim sessions> --robot <robot sessions> --manual <manual dir> --out report.md
```

`report.md` lists, for each banded metric:
- the sim value, the robot value and the bench value;
- the delta (bench or robot − sim);
- the Proposed band and the verdict: PASS, FAIL, or MISSING. A missing file or metric is never a PASS.

It then lists every metric. Exit 0 means all PASS, 1 a FAIL, 2 something MISSING.

**Who signs off (Proposed):**
- the operator signs the session;
- the model owner (Tushar Goel) signs the comparison with the model;
- the bands are accepted, changed or rejected by this repository's code owners (CODEOWNERS);
- T1's robot side additionally needs the platform-fw #20 owner;
- T5's #79 table needs the #79 owners (Om, driver and real profile; Manas, model and MuJoCo side; #79 decision comment of 17 Nov).

## 7. Simulation reference on `a3a93c6`, DART (T5 also Bullet)

Source:
- **Build:** a fresh clone of `a3a93c6` with the pins of `upstream_pins.yaml`: platform-sw `8e7a454`, openarm_description `14ff67b`, openarm_ros2 `4e837e1`.
- **Launch:** `sim.launch.py headless:=true base_plate:=bp000`, private `ROS_DOMAIN_ID` and `GZ_PARTITION`.
- **Measurement:** scripts of this change, ground truth from `gz model`. Gazebo prints poses to 1 mm and 1 mrad, which limits T1's ground-truth heights.
- **Evidence:** the session directories, kept with the P14 evidence (not committed).

| test | engine | metric | value |
|---|---|---|---|
| T1 | DART | lift leaves L1000 | **NOT RUN**: no. `lift_joint` −6.2e−15 m and carriage z 0.834 m at every stop; the goals returned no result in time and the simulator nearly stopped (sim time 9.1 → 13.7 s over two goal timeouts). The cause is #75 (L1000 is the lower limit; DART skips the velocity servo while gravity loads the joint into it). Unblocked by #75 |
| T1 | Bullet-Featherstone (extra, not DART) | speed / 0.018 m/s, L1000→L1175, L1175→L1350, L1350→L1000 | 0.9776, 0.9779, 0.9877 |
| T1 | Bullet-Featherstone | `lift_joint` at L1175 / L1350 / L1000 | 0.17500 / 0.35000 / 0.00000 m |
| T1 | Bullet-Featherstone | plate top face from the floor (ground truth) at L1000 / L1175 / L1350 | 0.844 / 1.019 / 1.194 m |
| T2 | DART | sim / commanded distance, forward / back (1.000 m commanded) | 0.7015 / 0.8960 |
| T2 | DART | lateral drift, forward / back | −1.4e−5 / −8.5e−7 m |
| T2 | DART | odometry / ground truth, forward / back | 1.342 / 1.000 |
| T3 | DART | sim / commanded turn, 90° / 180° | 0.8365 / 0.8890 |
| T3 | DART | odometry / ground truth yaw, 90° / 180° | 1.080 / 1.033 |
| T3 | DART | position drift of the model origin during the turn, 90° / 180° | 0.0066 / 0.0163 m |
| T4 | DART | max per-joint RMS \|ref − fb\|, home / hands_up / park (both arms) | 2.2e−13 / 0.0065 / 0.0065 rad |
| T4 | DART | max \|ref − fb\|, home / hands_up / park (both arms) | 2.3e−13 / 0.0067 / 0.0066 rad |
| T4 | DART | settle to 1e-3 rad after the trajectory end, hands_up / park (left; right) | 0.039 / 0.034 s; 0.045 / 0.029 s |
| T4 | DART | smallest M07 margin at park (joint 4, both arms) | 0.1000 rad (datum 1e-3) |
| T4 | DART | joint 4 effort vs model gravity torque, hands_up / park | 4.387 vs 4.393 / 0.413 vs 0.454 N·m |
| T4 | DART | joint 1 effort vs model gravity torque, hands_up (left; right) | −4.230 vs −4.304; 4.230 vs 4.304 N·m |
| T5 | DART | right and left finger_joint1 after open / half / closed | 0 (1.3e−14) for every command: neither gripper moves; open and half stalled (#101) |
| T5 | Bullet-Featherstone | finger_joint1 after open / half / closed, left | 0.78540 / 0.39270 / 0.00000 rad |
| T5 | Bullet-Featherstone | finger_joint1 after open / half / closed, right | −0.78540 / −0.39270 / 0.00000 rad |
| T5 | Bullet-Featherstone | model jaw opening at the measured angle, open / half / closed (both hands) | 0.1395 / 0.0728 / 0.00008 m |
| T5 | Bullet-Featherstone | settle to 1e-3 rad, open / half / closed (left; right) | 0.24 / 0.41 / 0.41 s; 0.48 / 0.40 / 0.43 s |

**Not run in simulation.**
- **T1 on DART:** blocked by #75. The Bullet values are an extra reference on another engine, not the DART reference.
- **T5 on DART:** it ran and is recorded, but it is no reference (#101).
- **T2 and T3:** DART only. platform-sw tuned the base on DART (`sim.launch.py:40-41`), and #56 and #75 are open on the base's friction and yaw.
