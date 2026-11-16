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
| scn03: right arm "3.0 rad from park" | The return move was never planned: MoveIt refused the start state three times (error -26, START_STATE_INVALID, log lines 14990, 15003, 15016) because `openarm_right_joint1` read -1.39639 rad, 9.1e-5 rad past its lower limit -1.3963, which is less than one step (3.8e-4 rad) of the Damiao position field the driver reads it through, and the start-state clamp only allowed 1e-5 rad. |
| scn04: gripper stays at 0.785 rad, MoveIt -10 on grasp_lift | No gravity torque reaches the arm motors (the MIT frame's tau is 0), so with the driver's gains the hand settles about 7 cm below the grasp pose, its finger lands on the shelf board, friction holds the finger where it is when the close command arrives, and MoveIt then refuses the lift because the measured start state is in collision with the board (error -10, START_STATE_IN_COLLISION). |

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
3. **Commanded against actual in the scenarios.** Every MoveIt move of scn03 and scn04 now logs the last trajectory
   point, the reading minus the command per joint, and `*_tracking_error_max` with a limit of 0.02 rad for the
   required moves. On the old behaviour this reads about 0.15 rad and fails. scn04 also logs the driver's finger
   reading next to the physics one.

4. **Hand orientation and TCP (D9 step 2).** scn03 reaches with the side-grasp orientation instead of a position-only
   goal; scn04 puts the item at the model's TCP read from /tf (`openamrobot_build1_scene/arm_frames.py`).

Tests: `test/test_gravity.py` (reference torques, controller file, launch defaults, codec step, clamp) and
`test/test_command_map.py` (effort claim found missing or misplaced). Both fail with the fix reverted.

## Remaining risk

1. The gravity model has no payload: an item in the hand is not compensated (0.15 kg in scn04).
2. The feed-forward runs as a ROS node at 100 Hz of ROS time, not in the real-time loop. On hardware it would need
   the software lead's decision, a payload model and a review of what happens when the joint states stop (today:
   the effort controller keeps its last command).
3. A MoveIt goal can still end within a step of a joint limit and a joint can rest on its stop; readings more than
   one step past a limit are refused, as before.
4. The finger geometry near the finger roots is inferred from the push and the closing angle seen in `fast`, not
   measured from the meshes.
5. Emulator effects (torque quantisation of the 12-bit tau field, the damping filter above 200 Hz) are inside the
   loop and were not separated here.
