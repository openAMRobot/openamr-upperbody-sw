# Build 1 arm failures: diagnosis (16 Nov 2026 CI run)

Scope: the two failures of CI job "firmware_sil arms, grasp, single writer" on PR #70 head `77bbf18`
(workflow run 43950927203, job 133592880607): `test_scn03_arms_reach` and `test_scn04_grasp`. Order of work as
decision D9: (1) forward kinematics, (2) frames and joint mapping, (3) commanded against actual, (4) gripper path.
Simulation only. Nothing here is safety evidence or a measurement of a delivered arm.

Inputs that could not be read: the `build1-*-results` artifacts (recorded trajectories and the scenario JSON files).
Their download host was refused by this session's network policy. The evidence below comes from the job logs
(GitHub API `actions/jobs/{id}/logs`), the code at the pins, and two off-line models of the same URDF.

## Summary

| Failure | Cause, one sentence |
|---|---|
| scn03: right arm "3.0 rad from park" (fixed: both arms return; right reach still displaced, see cycle 2) | The return move was never planned: MoveIt refused the start state three times (error -26, START_STATE_INVALID, log lines 14990, 15003, 15016) because `openarm_right_joint1` read -1.39639 rad, 9.1e-5 rad past its lower limit -1.3963, which is less than one step (3.8e-4 rad) of the Damiao position field the driver reads it through, and the start-state clamp only allowed 1e-5 rad. |
| scn04: gripper stays at 0.785 rad, MoveIt -10 on grasp_lift (fixed: PASS in cycle 2; cycles 3 and 4 grasp, lift and hold the item and fail only the new tracking check, see below) | No gravity torque reaches the arm motors (the MIT frame's tau is 0), so with the driver's gains the hand settles about 7 cm below the grasp pose, its finger lands on the shelf board, friction holds the finger where it is when the close command arrives, and MoveIt then refuses the lift because the measured start state is in collision with the board (error -10, START_STATE_IN_COLLISION). |

Neither failure is a joint-order, left/right, sign, offset or wrap-around error, and in both the command reached the
motor emulator. The 3.0 rad is the reach pose itself, held because no return trajectory was ever sent.

## D9 step 1: forward kinematics agree

Same joint vectors, base and lift frozen at 0, poses in `base_footprint`. Pinocchio 4.1.0 on the xacro expansion of
the combined model (bp000, `hardware_profile:=gazebo`, expansion SHA-256 `5d9a1b51...0ea`) against MuJoCo 4.0.1 on the
MJCF generated from the same expansion by PR #69's generator. 212 vectors: activation (all 0), park, joint 1 of the
right arm on its lower limit, the 9 MJCF keyframes, 200 random vectors inside the limits.
Script: `tools/build1_diagnosis/fk_compare.py`.

| Frame | Largest position difference | Largest rotation difference |
|---|---|---|
| `openarm_left_ee_base_link` (hand base, the IK target of scn03 and scn04) | 4.5e-13 mm | 3.8e-14 deg |
| `openarm_left_grasp_frame` (TCP of the model) | 4.5e-13 mm | 4.3e-14 deg |
| `openarm_right_ee_base_link` | 4.6e-13 mm | 4.1e-14 deg |
| `openarm_right_grasp_frame` | 4.6e-13 mm | 4.9e-14 deg |

MoveIt and Gazebo load the same expansion (`build1_robot.urdf.xacro` includes `combined_robot.xacro` unchanged).
In CI the driver's joint states and the Gazebo joint states differ by at most 3.7e-4 rad
(`L0_{left,right}_driver_state_minus_physics`, log lines 14899 and 14939 of job 133592880607): the emulator, the
codec and the driver report the physics joint by joint, by name.

## D9 step 2: frames, sides, joint order

- Target frame: both scenarios planned for `openarm_{side}_ee_base_link`, the hand base, not for the TCP. scn04 placed
  the hand base at an estimated `GRASP_DEPTH = 0.105` m behind the item (`scenarios.py` line 331 at `77bbf18`). The
  model's TCP, `openarm_{side}_grasp_frame`, is 0.138 m from the hand base along -z and 1.43 mm along -x. With the
  estimate the item sits against the finger roots: once the arm held its pose (CI cycle 1 below) it was pushed 90 mm
  before the fingers closed, as it was pushed 52 mm in `fast` on 16 Nov. Fixed: the item goes to the TCP, read from /tf.
- Orientation: scn03 sent a position-only goal, so any hand orientation was allowed. The plan that CI recorded for
  the right arm ends with joint 1 on its limit (-1.3963) and joint 2 at 3.012 rad, the arm over the top; the left
  arm's ends with joint 7 on its limit (-1.5708) and joint 3 at 1.549 of 1.5708 (cycle 1 log, `L0_* commanded end
  point`). Fixed: the reach uses the side-grasp hand orientation of scn04 (tolerance 0.3 rad). With it pinocchio IK
  gives mirrored solutions well inside the limits (joint 1 -1.306 and 1.306, joint 4 0.439, the rest under 0.2 rad).
- Left and right: driver joint names come from `arm_prefix` (`openarm_{left_,right_}joint1..7`, `finger_joint1`),
  motor ids 0x01 to 0x08 on `vcan_arm_l` and `vcan_arm_r`. The emulator maps the same ids to the same names
  (`arm_emulator.cpp` line 35). The right arm's joint 1 limits [-1.3963, 3.4907] are the mirror of the left's
  [-3.4907, 1.3963] (axis -y against +y). No swap.
- Joint order: MoveIt, the trajectory controllers and `/joint_states` are matched by name; the per-joint driver
  against physics difference above would show any mix-up.

## D9 step 3: commanded against actual

The pinned driver (`openarm_ros2` 4e837e1, `openarm_simple_hardware.cpp` line 291) sends each arm motor
`{kp, kd, q, dq, tau}` with `tau` taken from the joint's effort command interface. The trajectory controllers claim
only the position interfaces (`openamrobot_upperbody_control/config/arm_controllers.yaml`), so `tau` is always 0.
The emulator applies `kp (q* - q) + kd dq* + tau` (`motor_model.cpp` line 181). With kp 70, 70, 70, 60, 10, 10, 10
N m/rad (`openarm_simple_hardware.hpp` line 104, the same as upstream `control_gains.yaml`) nothing else holds the
arm against gravity, and it settles where `kp` times the error equals the gravity torque.

Off-line check with the same gains: the MJCF's arm actuators are the driver's MIT law (kp and kd of the driver, no
gravity term). The joint vector that puts the hand base at the scn04 `grasp_reach` pose (pinocchio IK) was commanded
and held for 6 s. Script: `tools/build1_diagnosis/sag_mujoco.py`, `check_gravity.py`.

| Commanded | Joint error after 6 s (rad), joints 1 to 7 | Hand base z, commanded 0.925 m |
|---|---|---|
| driver gains only (tau 0) | 0.145, 0.001, 0.001, -0.073, 0.005, -0.052, 0.001 | 0.851 m: 74 mm low, 15 deg tilted |
| driver gains and tau = gravity torque | 0.000 on every joint | 0.925 m |

Joint 1 needs 10.7 N m at that pose. The tote wall stands 45 mm below the planned hand base, and the fingertips of a
hand tilted 15 deg reach the board (top at 0.82 m). The same sag explains the scn03 guard readings of 0.028 and
0.037 m (`L0_*_position_error_from_tf`) and the earlier misses of up to 86 mm.

The `fast` profile is the control experiment: there Gazebo's position servo holds the arm, no driver is in the loop,
and in the same CI run `scn15_grasp_fast` (the same scenario function) closed the fingers on the item at 0.334 rad
and planned `grasp_lift` with success (job 133592881063).

## D9 step 4: the gripper path, traced

`left_gripper_controller` (position adapter, `hardware_interface_adapter.hpp` line 86) writes the goal to
`openarm_left_finger_joint1/position`. The driver converts it with `joint_to_motor_radians`, the identity for
`ee_type="pinch_gripper"` (`openarm_simple_hardware.cpp` line 378), and sends MIT `{5.0, 0.1, q, 0, 0}` to motor
0x08 (line 297). The emulator applies it as effort on `finger_joint1`; `finger_mimic_follower` drives `finger_joint2`.
The open command proved the path and its sign in the same run: 0.044 to 0.764 rad in the physics
(`finger_open_position`). No unit conversion (0.044 m) is involved at this point: 0.044 is only sent at activation.

Why the close did nothing: at log line 15767 (during `grasp_approach`) the driver reads the finger at 0.78565 rad,
past its open limit: something pushed it open. After `grasp_reach` MoveIt finds `openarm_left_ee_link1` (that
finger) in contact with `work_shelf_board_0` (line 15829). The close goal was accepted (line 15813) and the finger
stayed at 0.7854 rad against a motor torque of about 3.9 N m: it was held by the board. `stalled = True` is not a
cause. The driver reports finger velocity as 0 (`openarm_simple_hardware.cpp` line 277), so the controller declares a
stall one second into any move that has not reached its goal (`gripper_action_controller_impl.hpp` lines 182 to 197);
it aborts the goal but does not change the command, which keeps reaching the motor.

MoveIt error -10 is `START_STATE_IN_COLLISION`, not a planning failure: `CheckStartStateCollision` refused the
measured state with the finger in the board (line 15829).

## CI cycle 1 (head `9e13615`, run 43976060527)

| Check | 16 Nov (`77bbf18`) | Cycle 1 |
|---|---|---|
| scn04 tracking, `grasp_approach`, `grasp_reach`, `grasp_lift` | not measured (hand 74 mm low off-line) | 0.014, 0.015, 0.012 rad |
| scn04 fingers on the item | 0.7854 rad, did not move | 0.0 rad: closed fully, on nothing (item pushed 90 mm first) |
| scn04 `grasp_lift` | -10 START_STATE_IN_COLLISION | 1 SUCCESS |
| scn03 left reach tracking | not measured | 0.016 rad |
| scn03 right reach tracking | not measured | 0.332 rad (joint 6), over-the-top plan |
| scn03 return moves | right: -26, never sent | both 1 SUCCESS (no reading outside the codec step) |
| scn03 result | FAIL | BLOCKED: CTL-017 lift fault at real-time factor 0.46 (runner too slow) |
| scn09 single writer | PASS | PASS, with the effort claims of the map |

## CI cycle 2 (head `cff74e7`, run 43979440489)

| Check | Cycle 2 |
|---|---|
| scn04 tracking, `grasp_approach`, `grasp_reach`, `grasp_lift` | 0.016, 0.007, 0.015 rad |
| scn04 fingers on the item | 0.130 rad (physics), 0.130 rad (driver): closed on the item |
| scn04 item lifted | 54.5 mm, inside 40 to 120 mm |
| scn04 result | **PASS** |
| scn03 left reach | tracking 0.018 rad, hand base 19.8 mm from the target (16 Nov: 36.8 mm) |
| scn03 right reach | plan inside the limits now (joint 1 1.253, joint 2 0.134); reading off by up to 0.48 rad, joint 6 on its limit 0.785, hand base 178 mm from the target. Physics and driver agree to 3.8e-4 rad: the physical arm is displaced |
| scn03 return moves | both SUCCESS, both arms back within 0.026 rad of park |
| scn03 result | BLOCKED: CTL-017 lift fault at real-time factor 0.43 |
| drive, lift, cancel; faults | 2 PASS and 1 BLOCKED each (scn02, scn05: CTL-017 at low real-time factor), as on #70 |

The right reach at level 0: the same commanded vector held in MuJoCo with the driver's gains and the gravity torque
ends 0.000 rad from the command on every joint, with no contact other than the wheels. The MuJoCo model has no shelf,
tote or payload. The displacement in Gazebo therefore comes from something the off-line model does not contain:
a contact with the scene, or a self-contact MoveIt's allowed-collision matrix permits. Cycle 3 logs the hand base
against the target per axis and the ground-truth poses of tote, item and payload around each reach to tell which.

## CI cycle 3 (head `795ab94`, run 43988726353)

| Check | Cycle 3 |
|---|---|
| scn04 | grasp held: fingers 0.174 rad on the item, item not pushed (0.0 mm), lifted 82 mm, drift 0.01 mm in 3 s. FAIL only on the 0.02 rad limit: joint 5 0.0212 rad (0.21 N m at kp 10) during `grasp_lift` |
| scn03 right reach | 0.011 rad, hand base 19.5 mm from the target |
| scn03 left reach | displaced: up to 0.67 rad (joint 7), hand base 0.13 m low and 0.09 m outward, physics and driver agree to 3.4e-4 rad. Commanded vector inside the limits (joint 1 -1.304, the mirror of cycle 2's right) |
| scn03 objects | tote, item and payload at their start poses before and after the reach |
| scn03 result | BLOCKED: CTL-017 lift fault at real-time factor 0.32 |
| drive, lift, cancel; faults; single writer | as cycle 2: 2 PASS and 1 BLOCKED each; scn09 PASS |

So one of the two level-0 reaches is displaced in each run, the right one in cycle 2 and the left one in cycle 3,
with sound plans and the other arm tracking to 0.02 rad. The scene objects do not move. Not explained yet (open
item 6 below).

## CI cycle 4 (head `d7c6ffd`, run 43991379633)

| Check | Cycle 4 |
|---|---|
| scn04 | item lifted 71 mm and held. FAIL on `grasp_lift_servo_torque_max` 0.75 N m (joint 1, 0.0107 rad) |
| scn04, why | the held item is not in the gravity model: 0.15 kg x 9.81 m/s^2 x about 0.5 m from joint 1 = 0.74 N m. The check finds the missing payload term; the unloaded moves read 0.15 and 0.26 N m |
| scn03 left reach | 0.20 N m, hand base 13.7 mm from the target |
| scn03 right reach | displaced again: joint 7 -1.21 rad, joint 2 0.18 rad (12.8 N m at kp 70), hand base 0.13 m off; plan inside the limits |
| scn03 result | BLOCKED: CTL-017 lift fault at real-time factor 0.32 |
| profiles fast and fine | 8 of 8 PASS, scn15_grasp_fast included (it failed on 16 Nov: item pushed 52 mm) |
| drive, lift, cancel; faults; single writer | 2 PASS and 1 BLOCKED each, as on #70; scn09 PASS |

## Cycle 5: Baseline 1 lift_object and the return to park, from contact forces (17 Nov)

Inputs: Baseline 1 runs 44114692549 and 44114738411 (upperbody-sw `d1c88be`) and 44100053272 (`dad65e7`); scene-test
runs on `9301130`, `2c0b7c1` and `d1c88be`; and the new workflow `build1-arm-diagnostics.yml`, which runs scn04's
grasp-and-lift with the Baseline 1 retreat (`diag_grasp`) and scn03 at lift stop 0 (`diag_reach_return`) five times in
DART with gz contact sensors (`diagnostics.py`, `diagnostics_live.py`, table by `scripts/diag_table.py`). Diagnostic
runs: 44131608383 (`3b8e537`), 44135128051 (`ab79db0`), 44137806328 (`d3368c3`, first with contact forces, the
baseline below), 44138101782 (`e1301bc`), the run of `bbd1346`, 44144252655 (`d139f76`) and the run of `696458d`.

Contact sensors on the 18 robot links were advertised and silent in DART, with collision names that match the
converted model (gz sdf, printed by the workflow). The sensors that work are in the world file: one each on the item,
the tote and the shelf fixtures, which report the touching robot link. The model has no `self_collide`, so Gazebo
computes no contact between two robot links: an arm cannot hit the robot body in this simulation.

### lift_object: the item rose 0.0386 m

Forward kinematics of the driver's joint readings at the end of `grasp_reach` and `grasp_lift` (`gravity.py` on the
expanded model), with the item carried rigidly by the TCP:

| Run | Hand base rise | TCP rise | Hand turned | Rigid-grasp item rise | Measured item rise | Difference (slip) |
|---|---|---|---|---|---|---|
| Baseline 1 run B, 44114738411 | 0.0642 m | 0.0436 m | 10.9 deg | 0.0439 m | 0.0386 m | 0.0053 m |
| Baseline 1 run A, 44114692549 | 0.0905 m | 0.0485 m | 28.9 deg | 0.0516 m | 0.0519 m | -0.0003 m |
| Baseline 1, 44100053272 | 0.0756 m | 0.0484 m | 12.6 deg | 0.0491 m | 0.0488 m | 0.0003 m |
| scene tests `d1c88be` | 0.0794 m | 0.0684 m | 14.3 deg | 0.0689 m | 0.0473 m | 0.0216 m |
| scene tests `9301130` | 0.0692 m | 0.0948 m | 10.7 deg | 0.0954 m | 0.0015 m | 0.0939 m |

The hand did not rise 0.08 m at the item. The lift was a pose goal on the hand base with the nominal side-grasp
orientation, a 0.015 m sphere and 0.3 rad per axis, and MoveIt ended with the hand turned 11 to 29 degrees: the TCP,
0.138 m from the hand base, rose 0.044 to 0.049 m in the three Baseline 1 runs. Where the grasp held, the item followed
the TCP to within 5.3 mm. The region of that goal allows a TCP rise below 0 (`arm_frames.worst_tcp_rise`).

Contact forces, diagnostics baseline (`d3368c3`, 5 runs; per finger, the item's sensor, mean over lift and hold):

| | Range |
|---|---|
| Normal force, finger 1 / finger 2 | 0.67 to 1.80 N / 1.00 to 3.97 N |
| Needed per finger, m g / (2 mu), mass 0.15 kg and mu 1.0 (both estimated) | 0.736 N (0.98 N at the 3.25 m/s^2 peak) |
| Tangential force on the item per finger | 0.72 to 3.31 N |
| Contact points per message, finger 1 / finger 2 | 1.2 to 10.1 / 1.2 to 2.4 (mesh on box) |
| Hand turned during the lift | 7.7 to 22.4 deg |
| Item slip in the TCP frame | 9.8 to 44.2 mm, onset 0.09 to 0.52 s after the lift starts |
| Finger joint 1 on a position limit | 0 of the samples in all five; finger joint 2 minus 1: 0.021 to 0.092 rad |
| Arm emulator torque on finger joint 1 (kp 4.884 x error) | 0.48 to 1.58 N m |
| Gazebo's transmitted torque about the finger axis | 0.07 to 0.24 N m |

So the grip holds the item with a factor of 0.9 to 5.4 over the static need, and the friction is near its limit during
a turning lift. The finger does not rest on its limit when it holds the item; it does when it closed on nothing (0.55
and 0.87 of the samples in those runs). The gripper command (GripperCommand max_effort 3.0 N m) does not set the grip:
the position adapter and the MIT kp of 5 do. The torque the emulator reports is 5 to 7 times the torque that the
contact forces balance at the TCP's 0.07 m from the finger axis; this is not resolved here (Proposed, below).

### Return to park: 46 N m with joint 6 on its limit, and 21.5 N m on joint 2

Diagnostics baseline, 10 retreats and 20 scn03 reaches and returns. Contacts with the world, largest force per pair:

| Move | Runs with a finger on `work_shelf_board_0` | Force | Other contacts |
|---|---|---|---|
| Baseline 1 retreat of the left arm (tote and item out of the planning scene) | 4 of 5 | 18 to 89 N | tote front and right walls 84 to 162 N in 2 runs, item 0.7 to 21 N |
| scn03 reach, left or right | 6 of 10 | 9.8 to 76 N | item under the left target: 0.5 to 1.2 N |
| scn03 return to park | 8 of 10 | 3.8 to 68 N | item 17 N once |

The retreat that ended 0.785 rad from park (joint 6 on its upper limit, joint 1 at 36.7 N m) had a finger on the board
at 89 N: the 46 N m case of 44100053272. A reach that ends displaced (0.13 to 0.19 m at the hand, joint 2 at 18 to 21 N m,
the 21.5 N m case) has a finger on the board at 42 to 76 N. When it wedges there, MoveIt refuses the next start state
(-10 START_STATE_IN_COLLISION, three attempts) and the arm stays 0.92 to 1.57 rad from park. The plans are inside the
joint limits (margin 0.039 to 0.109 rad). MoveIt plans with zero padding, and the executed hand base leaves its planned
path by up to 0.009 m where it touches nothing (`hand_path_deviation_m`, runs of `bbd1346` and `d139f76`). Not a gravity feed-forward error: its
command was never zero after start-up, and the gains were not touched. Not a collision with the robot body: none is
simulated.

### Fixes

| Fix | Test (fails when reverted) |
|---|---|
| scn04 reach and lift are straight lines of the hand with the orientation the approach ended with (`/compute_cartesian_path`, `/execute_trajectory`; `arm_frames.grasp_line`, `straight_lift`) | `test_arm_frames.py`: grasp line, lift on straight lines, old goal region |
| Only the item leaves the planning scene; on every path out of the grasp the gripper opens, the hand withdraws 0.10 m along the fingers, the payload is cleared, then the item returns (`planning_scene.excluded_from_planning`, `_withdraw`) | `test_arm_frames.py`: exclusion on every path, order of release, withdraw and return; `test_gravity.py`: payload cleared in the finally block |
| 0.015 m planning clearance on every fixed box, planning scene only (`FIXTURE_PADDING_M`, loader parameter `fixture_padding_m`) | `test_arm_frames.py`: padded boxes, measured deviation, lift and grasp still feasible (0.04 m is refused) |
| The straight reach runs 0.02 m above the grasp height (0.03 m above the item centre, unchanged) and comes down last; approach tolerance 0.12 rad per axis (was 0.3); from the gripper's collision meshes over the tote's front wall and under the board above | `test_arm_frames.py`: hand above the wall on the way in, lifted hand below the padded board above |

The 40 mm requirement, friction, masses, gains, limits and the gripper values are unchanged. 0.04 m of clearance was
tried (`bbd1346`): it closed the 0.25 m between the work shelf boards and the lift and the level-0 reaches were refused
(Cartesian fractions 0.52 to 0.88, MoveIt 99999).

### After the fixes

| Head | Grasps passing | Item rise | Item minus TCP rise | Hand turned in the lift | Retreats and scn03 returns at park | Board contacts in reaches and returns |
|---|---|---|---|---|---|---|
| `d3368c3` (before) | 3 of 5 | 0.0166 to 0.0822 m | -42.8 to -8.5 mm | 7.7 to 22.4 deg | 19 of 20 | 18 of 30 moves |
| `d139f76` | 4 of 5 | 0.0797 to 0.0803 m (one 0: wall caught in the reach) | -3.4 to -2.5 mm | 0.7 to 0.8 deg | 20 of 20 | 0 of 30 moves (tote wall 8 to 48 N in 3) |
| `696458d` (grasp 0.05 m above the centre, reverted) | 0 of 5: lift refused, Cartesian fraction 0.71 to 0.82 | - | - | - | 20 of 20 | 0 of 30 moves |
| HEAD_ROW | | | | | | |

## Grippers in the Gazebo video run (run 43978020109, branch viz/scene-video)

Both grippers read 0.000 after the close commands. By the convention of #66 (0 = closed, far limit = open) that is the
commanded state, not a fault, and it says nothing about whether the fingers moved. The open half decides it. In the
direct profiles with DART the package README records that the grippers do not move at all (Known limits, "Engine"),
and in this work the only gripper failure with the driver in the loop was the board contact above; with the arm
held (cycle 2) the driver path closed on the item. The video run's lift (0.000 m with a successful action) is out
of scope here; the trace of this work found no shared cause with the arm path (the lift has no ros2_control
interface with devices and no gravity feed-forward).

## Fixes in this change

1. **Gravity feed-forward, the missing command** (Proposed). `openamrobot_build1_scene/gravity.py` computes the static
   holding torque of the 14 arm joints from the URDF's masses, centres of mass and axes. It agrees with pinocchio's
   `computeGeneralizedGravity` to 1.1e-14 N m on 303 vectors. `scripts/gravity_feedforward.py` sends it from the
   driver's own joint states to one `effort_controllers/JointGroupEffortController` per arm on the real driver's
   `/controller_manager`; OpenArmHW puts it in the tau field of the same MIT frame as the trajectory position. Launch
   argument `gravity_feedforward` (default `true`, device profiles only). The single-writer map now lists the effort
   claim on the arm joints and the two command topics. No gain, damping, mimic, limit or physics parameter changed.
2. **Start-state resolution with devices.** `joint_state_clamp` gets `epsilon` = one step of the 16-bit Damiao position
   field, 2 x 12.5 / 65535 = 3.8148e-4 rad, when the joint states cross the codec. The field truncates: a joint at 0
   reads -1.907e-4 rad, a joint on a lower limit up to one step below it. Readings further out are still passed
   through and logged. The default 1e-5 rad of the other profiles is unchanged.
3. **Commanded against actual in the scenarios.** Every MoveIt move of scn03 and scn04 logs the last trajectory point,
   the reading minus the command per joint, `*_tracking_error_max` in rad and `*_servo_torque_max`, the largest
   driver kp times the error: the torque the joint's servo is still pushing with. Limit 0.5 N m for the required
   moves (7 % of the smallest motor peak). On the old behaviour joint 1 alone gives about 10 N m. Cycles 1 to 3 used
   a 0.02 rad limit on every joint; joint 5 (kp 10) read 0.016 to 0.023 rad in every arm move, loaded or not, which
   is 0.2 N m, so the limit is now set in torque.
4. **Hand orientation and TCP (D9 step 2).** scn03 reaches with the side-grasp orientation instead of a position-only
   goal; scn04 puts the item at the model's TCP read from /tf (`openamrobot_build1_scene/arm_frames.py`).

Tests: `test/test_gravity.py` (reference torques, controller file, launch defaults, codec step, clamp) and
`test/test_command_map.py` (effort claim found missing or misplaced). Both fail with the fix reverted.

## Remaining risk

1. The held item (0.15 kg in scn04, about 0.75 N m on joint 1) is now added as a point mass at the model's TCP once
   the gripper reports the grasp, and cleared after the release (Proposed). Its centre is taken at the TCP; the
   grasp puts the TCP 0.03 m above the item's centre. Not yet seen in a CI run.
2. The feed-forward runs as a ROS node at 100 Hz of ROS time, not in the real-time loop. When the joint states of an
   arm are older than `max_state_age_s` (0.2 s) or never arrived, it sends tau 0 for that arm every cycle, because the
   effort controller keeps its last command (docs/architecture/d4_mit_frame_field_ownership.md, Proposed). That is a
   functional command timeout, not a safety function. On hardware it would need the software lead's decision.
3. A MoveIt goal can still end within a step of a joint limit and a joint can rest on its stop; readings more than
   one step past a limit are refused, as before.
4. The finger geometry near the finger roots is inferred from the push and the closing angle seen in `fast`, not
   measured from the meshes.
5. Emulator effects (torque quantisation of the 12-bit tau field, the damping filter above 200 Hz) are inside the
   loop and were not separated here.
6. scn03 level-0 reach: one arm per run ends displaced by up to 1.21 rad with a sound plan (right in cycles 2 and 4,
   left in cycle 3). Explained in cycle 5: a finger on the edge of work_shelf_board_0 (42 to 76 N). Fixed there by a
   planning clearance. Still open: the left target is 0.015 m above the item's top, and the hand base touches the item
   (0.4 to 16 N) in most runs; once that held the servo torque at 0.73 N m.
7. scn03 never reaches level 1 in CI: the lift move faults the CTL-017 model below a real-time factor of 0.55.
