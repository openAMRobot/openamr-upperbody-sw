# Build 1 scene: lift and base diagnosis in Gazebo DART

Gate C of the digital twin: the basic robot moves correctly in Gazebo DART, the reference engine (D1). This page covers
two symptoms of the scene video of 16 Nov 2026 and what was done about each. Simulation only. Nothing here is
hardware, physical acceptance or safety evidence.

Inputs: openamr-upperbody-sw #70 at `77bbf18` (the Build 1 scene), openamr-platform-sw main `69847ab`, Gazebo Harmonic
with `ros-jazzy-gz-ros2-control` 1.2.20, `ros-jazzy-gz-physics-vendor` 0.0.10 and `ros-jazzy-gz-dartsim-vendor` 0.0.3
(DART 6.13), as installed on the CI runner.

## 1. The lift does not move in DART, and the action succeeds

### Symptom

Scene video, Actions run 43978020109, job log, profile `fast`, engine `gz-physics-dartsim-plugin`, model sha256
`0bc1382b...`:

```
VIDEO_TASK {"name": "lift_to_stop_2_L1350_attempt_1", "sim_s": 46.584, "succeeded": true, "lift_m": -1.3960186979930254e-14}
VIDEO_TASK {"name": "lift_to_stop_2_L1350_attempt_2", "sim_s": 71.934, "succeeded": true, "lift_m": -1.3960195449608016e-14}
```

The joint sits 1.4e-14 m below its lower limit, the same value on both attempts, and `lift_controller` reports success.

### Command path in the profiles without devices

`rig.lift_move` > `/lift_controller/follow_joint_trajectory` > `joint_trajectory_controller` (`arm_controllers.yaml`,
command interface `position`) > `gz_ros2_control/GazeboSimSystem` of the system `openamr_lift_gazebo` > Gazebo. No
CTL-017 emulator and no gz plugin command the lift in `fast` and `fine`. With devices the lift has no ros2_control
interface; `ctl017_gazebo` commands a velocity through the gz `JointController` plugin (SW-06).

Joint and load, read from the expanded gazebo-profile model: `lift_joint` prismatic on z, limits 0.0 to 0.350 m,
effort 800 N, velocity 0.018 m/s (datums `lift_max_effort_dold_v4_350`, `lift_max_velocity_dold_v4_350`). The child
subtree of `lift_joint` (carriage, equipment, both arms) is 19.8 kg, 194 N of weight: a quarter of the effort limit.
The load is not the cause.

### Cause

Three facts, each read from the installed sources:

1. `gz_ros2_control` turns a position command into a joint velocity command:
   `target_vel = -position_proportional_gain * (q - q_cmd) * update_rate` written to `JointVelocityCmd`
   (`gz_system.cpp`, `GazeboSimSystem::write`).
2. gz-physics applies a velocity command in DART as a servo actuator (`dartsim/src/JointFeatures.cc`,
   `SetJointVelocityCommand` sets `Joint::SERVO`).
3. DART builds one joint constraint per joint with the priority position limit, then velocity limit, then servo
   (`dart/constraint/JointConstraint.cpp`, `JointConstraint::update`). When the joint is below its lower limit, or at
   it with a velocity into it, the limit branch is taken and the loop `continue`s: the servo row is never added for
   that step.

L1000 is joint position 0.0, the lower limit. Gravity loads the carriage into that limit at every step, so the limit
branch is active at every step and the servo is never applied, whatever the commanded velocity. The joint stays on the
limit at a constant rounding residue (-1.39e-14 m). Whether a run moved depended on where the landing left the joint:
a hair above the limit and still, the servo branch is reached; on or below it, never. That is the intermittency of the
earlier local runs, and the A7 landing finding of #48 (pushed past the lower stop before the controller was active).

The success: `lift_controller` has no `constraints` in `arm_controllers.yaml`. A goal tolerance of 0 is not checked by
`joint_trajectory_controller`, so the action succeeds when the trajectory time has passed. `rig.lift_move` returned the
action status alone.

In one sentence: the lift's rest stop is its lower position limit, and DART skips a velocity servo on a joint loaded
into its limit, so the position interface of `gz_ros2_control` cannot lift the carriage off L1000, while the trajectory
controller without a goal tolerance reports success anyway.

### Fix (profiles `fast` and `fine`)

- `build1.launch.py` adds an effort command interface to `lift_joint` of `openamr_lift_gazebo` in the plant model it
  spawns (`launch_support.lift_effort_plant`). The description package is not edited. No link, joint, limit, mass or
  inertia changes. The 0.350 m travel, the 0.018 m/s and 800 N limits and the three stops are unchanged.
- `config/lift_controller_gazebo.yaml`, handed to `gz_ros2_control` after `arm_controllers.yaml` (and after
  `sim_overlay.yaml` at the fine step): `lift_controller` commands `effort` with PID gains p 30000 N/m, i 60000 N/(m*s),
  d 1200 N*s/m (UNVERIFIED working values), and has a goal tolerance of 3 mm with a goal time of 2 s. An effort command
  is a joint force in the dynamics; the limit constraint only stops motion below the limit, so the carriage leaves
  L1000. The action now aborts when the joint is not at the stop.
- `rig.lift_move` returns arrival only when the action succeeded and `/joint_states` shows the joint within 3 mm.
- `config/command_map.yaml`: without devices `lift_controller` claims `lift_joint/effort`; the arms and grippers still
  claim position. The single-writer checks read the per-group claim.

Nothing sets a joint position directly, and no command is replaced by a pose reset.

### Test

`test_scn16_lift_fast` (CI job "fast lift stops and base yaw"): stops 2, 0, 1, 2, 0 through `lift_controller`. Each move
checks the position on `/joint_states` within 3 mm and fails when the action succeeded without arrival. The sequence
returns onto the lower limit and leaves it again.

Before the fix, PR #75 head `94bca56`, Actions run 43990377881, job 133737659293:

| Move | Action | `/joint_states` | Check |
|---|---|---|---|
| 1, to stop 2 (0.350 m) | succeeded, "Goal successfully reached!" | -1.39e-14 m | FAIL, success without arrival |
| 2, to stop 0 | succeeded | -1.39e-14 m | pass (no travel) |
| 3, to stop 1 (0.175 m) | succeeded | -1.39e-14 m | FAIL, success without arrival |
| 4, to stop 2 | succeeded | -1.39e-14 m | FAIL, success without arrival |
| 5, to stop 0 | succeeded | -1.39e-14 m | pass (no travel) |

After the fix, PR #75 head `92540ba`, Actions run 43994537341, job 133743203833, real-time factor 0.75, PASS:

| Move | Action | `/joint_states` | Gazebo time | Check |
|---|---|---|---|---|
| 1, to stop 2 (0.350 m) | succeeded | 0.35000 m | 25.6 s | pass |
| 2, to stop 0 (onto the lower limit) | succeeded | 0.00001 m | 25.4 s | pass |
| 3, to stop 1 (0.175 m, off the lower limit) | succeeded | 0.17490 m | 13.7 s | pass |
| 4, to stop 2 | succeeded | 0.34973 m | 13.7 s | pass |
| 5, to stop 0 | succeeded | 0.00000 m | 25.4 s | pass |

The log of `lift_controller` reads "Command interfaces are [effort]". Travel at the planned 0.015 m/s: 0.350 m in
23.3 s plus the 2 s margin of the goal.

Static tests: `test_launch_support.py` (the plant edit adds exactly one interface, the overlay has a non-zero goal
tolerance and goal time, the fine-step values survive the merge), `test_command_map.py` (the lift claims effort without
devices, a position claim on it is found).

### What remains for the lift

- **Device profiles.** `ctl017_gazebo` commands a velocity through the gz `JointController`, the same DART servo.
  In CI (job 133729114649) the lift left L1000 and stopped at 0.072 m on a CTL-017 stall fault (F02, Hall rate) at a
  real-time factor of 0.56: the clock rule of the emulator, addressed by openamr-platform-sw #158. The servo limit
  still applies there whenever the lift rests on L1000 under load, for example after a move back to stop 0. The
  candidate is the force mode of the same plugin (`use_force_commands` with a velocity PID), not applied here because
  no device run can pass on the runner yet to show it.
- **Standalone simulation.** `openamrobot_upperbody_gazebo` still drives the lift through the position interface and
  carries the same limit. Not changed here.
- **Grippers on DART.** The finger joints rest on a position limit (left 0 of 0 to 0.785, right 0 of -0.785 to 0) and
  read 2.3e-15 rad beyond it in the video run (`left_gripper_close` -2.325e-15, `right_gripper_close` 2.325e-15). This
  fits the same DART behaviour. Not investigated further here. Note: the video task commands +0.7854 rad to open the
  right gripper, whose range is -0.785 to 0.
- The gains are working values. No bench recording of the column exists.

## 2. Base yaw in DART

### Symptom

Scene video, same run:

```
VIDEO_TASK {"name": "drive_forward_0.2_m_s_for_8_s", "truth": [1.6010805922006122, -2.000675142025947e-05, -1.2628490585739611e-05]}
VIDEO_TASK {"name": "turn_right_0.5_rad_s_for_3.14_s", "truth": [1.6100226539345954, 0.007947154011543422, -1.5081009344876761]}
```

The command of the turn was `(0.0, -0.5)`: angular -0.5 rad/s, a right turn. A right turn is negative yaw, so the sign
is correct. The magnitude is 1.508 rad for 1.570 rad of command (-4.0 %). Nothing in that run recorded the wheels.

### Scenario

`test_scn17_base_yaw_fast`, order D8 as far as this repository can take it: the model of openamr-platform-sw main as it
is, drive wheels and the four castors present (fixed joints, frictionless spheres, `mu` 0). `/cmd_vel` to the DiffDrive
plugin. Equal wheel speeds (0.2 m/s for 8 s), then equal and opposite ones (0.5 rad/s for pi s left and right, 0.25
rad/s for 2 pi s left), each from standstill and to standstill. For each: ground truth yaw, the kinematic expectation
r * (dR - dL) / track from the wheel joint positions, and `/odom`. Radius 0.100 m (wheel collision cylinder) and track
0.400 m (wheel joint offsets) are read from `/robot_description`, equal to the DiffDrive values. The track is the
platform's placeholder and is not changed. The loaded radius 0.098 m is a design input that the pinned model does not
carry yet (openamr-platform-sw #150 sets it).

### Numbers

Run A: PR #75 head `94bca56` (before the lift fix), Actions run 43990377881, job 133737659293, real-time factor 0.61,
arms in the activation pose, segments in the order straight, spin left, spin right, slow spin:

| Segment | Wheels dL, dR (rad) | Kinematic | Ground truth | Odometry | Truth vs kinematic | Odometry vs truth |
|---|---|---|---|---|---|---|
| straight 0.2 m/s, 8 s | 16.200, 16.200 | 1.6200 m | 1.6202 m, yaw -1e-5 rad | 1.6200 m | +0.01 % | -0.01 % |
| spin left 0.5 rad/s | -3.186, 3.186 | 1.5930 rad | 1.6094 rad | 1.5930 rad | +1.03 % | -1.02 % |
| spin right 0.5 rad/s | 3.172, -3.172 | -1.5860 rad | -1.6025 rad | -1.5860 rad | +1.04 % | -1.03 % |
| spin left 0.25 rad/s | -3.181, 3.181 | 1.5905 rad | 1.6075 rad | 1.5905 rad | +1.07 % | -1.06 % |

Run B: PR #75 head `92540ba` (lift fix), Actions run 43994537341, job 133743203833, real-time factor 0.69, same order:

| Segment | Wheels dL, dR (rad) | Kinematic | Ground truth | Odometry | Truth vs kinematic | Odometry vs truth |
|---|---|---|---|---|---|---|
| straight 0.2 m/s, 8 s | 16.140, 16.140 | 1.6140 m | 1.6142 m, yaw -1e-5 rad | 1.6140 m | +0.01 % | -0.01 % |
| spin left 0.5 rad/s | -3.209, 3.209 | 1.6045 rad | 1.6106 rad | 1.6045 rad | +0.38 % | -0.38 % |
| spin right 0.5 rad/s | 3.242, -3.242 | -1.6210 rad | -1.6272 rad | -1.6210 rad | +0.38 % | -0.38 % |
| spin left 0.25 rad/s | -3.149, 3.149 | 1.5745 rad | 1.5823 rad | 1.5745 rad | +0.50 % | -0.49 % |

Run B also ran the video sequence last, after the spins. Facing +y it drove 1.6 m from (1.6, 0) into the 0.1 m post
`obstacle_post` at (1.5, 1.2) of `config/scene.yaml` and then turned -1.045 rad of ground truth for -1.573 rad of
wheels. That segment measured a collision, not the wheel contact; the drive distance was not logged. The scenario now
runs the video sequence first, from the spawn pose and heading of the video, and logs the drive distance and both poses.

Each spin moves the base 12.4 to 13.1 mm. The base rests nose down by 0.0093 rad (front castors loaded) in every segment.
Odometry equals the kinematic expectation exactly: DiffDrive integrates the same wheel positions with the same radius
and separation. The wheels turned 1 to 1.4 % more than the command because the scenario's `/cmd_vel` publisher runs on
the wall clock; the comparison is made against the wheels, not against the command.

### Video sequence

Run C: the video sequence first, from the spawn pose. Numbers pending the CI run of the scenario change.

### Reading

- The sign is right in both directions and the error is the same at 0.25 and 0.5 rad/s and in both directions.
- From standstill the body turns 0.4 to 1.1 % more than the wheels say (two runs, no seed set), not 4 % less. The
  video deficit is not reproduced from standstill. The video sequence segment answers it; see "Video sequence" below.
- The castors of the pinned model are frictionless on fixed joints. They carry normal load and no tangential force, so
  they cannot brake or drive the yaw directly. What they change is the share of the weight left on the drive wheels.
- The drive wheels are cylinders 46 mm wide on a rigid floor. DART puts contact points across the width, at radii
  between 0.177 and 0.223 m from the centre of the turn. A wheel that rolls without slip at its mid-plane must scrub at
  both edges in a spin, and where it grips decides the yaw rate: anywhere from 0.400/0.446 to 0.400/0.354 of the
  kinematic value. +1 % sits inside that band: the inner edges grip a little more. This is the drive wheel contact
  model, not the track.
- Verdict for this step of D8: the measured yaw error comes from the drive wheel contact, not from the castor model.
  openamr-platform-sw #150 (castor swivel, sphere contact, loaded radius 0.098 m) is the next D8 step: once castors
  grip and swivel, they add their own drag and the yaw has to be measured again with this scenario. Nothing in the
  platform model is edited here.
- The older evidence has the other sign: odometry 171.9 degrees for 168.3 of ground truth (6 s at 0.5 rad/s after a
  drive, 15 Nov), and 9.4 degrees of over-report in the Nav2 spin of `scn14_drive_fast`. From standstill at 0.25 and
  0.5 rad/s odometry reports about 1 % less than the body turns. The slip of the wheel contact depends on how the turn
  is entered (speed, acceleration, a drive just before); the video sequence segment tests that case.

## What remains

1. Lift over CAN3: a passing run needs openamr-platform-sw #158 (clock contract) on the runner, then the force mode of
   the lift `JointController` if the lift sticks on L1000 there.
2. `openamrobot_upperbody_gazebo`: the same lift fix, if the owner adopts this one.
3. Base: rerun `scn17_base_yaw_fast` on openamr-platform-sw #150 when it is adopted; then the lift, arms and payload
   steps of D8.
4. Grippers on DART: check whether the same limit behaviour stops the fingers.
