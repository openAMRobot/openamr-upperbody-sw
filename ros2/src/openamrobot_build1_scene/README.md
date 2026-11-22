# openamrobot_build1_scene

The Build 1 scene. The full OpenAMRobot 2.0 combined model stands in one Gazebo Harmonic world. Every device of the
robot is on a virtual CAN bus with its emulator. Nav2 and MoveIt 2 run on top. Scenarios, including faults, run
headless and write numbers.

Simulation only. Nothing here is hardware, physical acceptance or safety evidence. No part of this package is a safety
function. Emulators are protocol evidence only.

What is modelled, assumed and not modelled: [`docs/build1_scene.md`](../../../docs/build1_scene.md).

## Run it

```bash
ros2 launch openamrobot_build1_scene build1.launch.py rviz:=true
```

That is the run profile `firmware_sil`. It needs the five virtual CAN interfaces:

```bash
ros2 run openamrobot_build1_scene setup_vcan_all.sh
```

The script needs root or passwordless sudo, `iproute2` and the `vcan` kernel module. On Ubuntu the module is in
`linux-modules-extra-$(uname -r)`. Without virtual CAN the launch prints `BLOCKED` and stops. On such a machine the
profiles `fast` and `fine` still run:

```bash
ros2 launch openamrobot_build1_scene build1.launch.py profile:=fast rviz:=true
```

Run one scenario against a running launch:

```bash
ros2 run openamrobot_build1_scene run_scenario.py --list
```

```bash
ros2 run openamrobot_build1_scene run_scenario.py scn01_drive
```

## Workspace

The workspace of the repository README, plus two things. They are the pins of
[`config/pins.yaml`](config/pins.yaml). CI uses the same layout
([`build1-scene-tests.yml`](../../../.github/workflows/build1-scene-tests.yml)).

| Input | Pin | How it is used |
|---|---|---|
| openamr-upperbody-sw main | `37b93f5` | merged in (branch point `b15bcb6`) |
| openamr-upperbody-sw #66 (open) | `6fe5e9c` | merged into this branch by an explicit merge commit |
| openamr-upperbody-sw #65 (open) | `d1515b1` | read only, not merged |
| openamr-platform-sw main | `69847ab` | checked out whole (Nav2, description, docking unchanged since `0741778`): description, Nav2, perception, docking model |
| openamr-platform-sw #145 (open, provisional) | `8c5bd52` | only `ros2/src/openamrobot_sil`. Contains #141, #144 and main (#139 merged as `69847ab`) |
| openamr-platform-sw #105 (open) | `acc427d` | not checked out. The base RGB-D sensor block follows it |
| audits `audit/2026-09-28` | `be17653` | Use Case 1 baseline, M-01, E-01, phase 1 baseline |
| openarm_ros2, openarm_can, openarm_description | as `sil_pins.yaml` | the unmodified upstream driver |

`openamr-platform-sw` is not edited. `navigation_status_node` of its Nav2 launch needs the package
`openamr_nav_msgs`, which is in neither repository. That one process exits at start. Nothing here uses it.

## Run profiles

The names are those of the phase 1 baseline (audits, `digital-twin/phase1/run_profiles.md`). Every result names its
profile, the baseline lock, the hash of the expanded model and the engine.

| Profile | Devices | Step | What drives the joints |
|---|---|---|---|
| `firmware_sil` (default) | emulators on virtual CAN, host test client or firmware in Renode, real arm driver | 1 ms. `step:=fine` gives 1/1500 s | see the single-writer map |
| `faults` | as `firmware_sil` | as `firmware_sil` | as `firmware_sil`. Faults are injected, no owner changes |
| `fast` | none | 1 ms | `gz_ros2_control` position controllers, the base DiffDrive plugin |
| `fine` | none | 1/1500 s, with the 750 Hz overlay of #66 | as `fast` |
| `geometry` | none | no physics | refused here. `openamrobot_upperbody_bringup` runs it |

`fine` is the default physics profile of `config/scene.yaml`. The baseline puts `firmware_sil` at 1 ms.
`firmware_sil` with `step:=fine` is a new combination and not a baseline profile. CI reports both steps.

## Launch arguments

| Argument | Values | Default | Meaning |
|---|---|---|---|
| `profile` | `firmware_sil`, `faults`, `fast`, `fine` | `firmware_sil` | run profile |
| `step` | `auto`, `fast`, `fine` | `auto` | physics step of the device profiles |
| `stm32` | `host`, `renode_readonly` | `host` | what sits in the STM32 slot, as `openamrobot_sil` |
| `physics_engine` | a gz-physics plugin | `gz-physics-dartsim-plugin` | see Known limits |
| `station` | `origin`, `work_shelf`, `aisle_entry` | `origin` | where the robot is spawned |
| `sensors`, `extra_sensors` | `true`, `false` | `true` | rendered sensors, and the cameras this package adds |
| `nav2`, `moveit`, `rviz`, `headless` | `true`, `false` | `true`, `true`, `false`, `false` | |
| `create_vcan` | `true`, `false` | `true` | run `setup_vcan_all.sh` first |
| `driver_rate` | `auto`, Hz | `auto` | real driver's controller manager. Every second physics step |
| `gravity_feedforward` | `true`, `false` | `true` | device profiles: the arms' gravity torque through the real driver's effort interface (Proposed, see `docs/build1_arm_diagnosis.md`). Tau 0 for an arm whose joint state is stale; a held item as an optional payload (`docs/architecture/d4_mit_frame_field_ownership.md`) |
| `planning_pose_source` | `map`, `ground_truth` | `map` | where the MoveIt planning scene's objects are placed from: the robot's own estimate (AMCL map, or spawn pose and odom before it), or Gazebo's ground truth, a simulation oracle that every run using it declares in `/build1/run_info` (`planning_pose_source`), in each scenario JSON (`run.planning_pose_source`) and in its log. Environment `BUILD1_PLANNING_POSE_SOURCE` sets it for the scenario tests (`test/build1_harness.py`). Proposed |
| `object_pose_source` | `ground_truth` | `ground_truth` | where the tote, item and payload poses (planning scene) and the grasp targets (scenarios) come from. `ground_truth` is Gazebo's pose of each object, a perception stand-in and the only implementation today: no run with it counts as the perception profile until an object detector replaces it. Declared in `/build1/run_info` (`object_pose_source`), in each scenario JSON (`run.object_pose_source`) and in the launch, loader and scenario logs; a scenario whose run_info lacks it is refused. Environment `BUILD1_OBJECT_POSE_SOURCE` sets it for the scenario tests. Proposed |
| `driver_clock` | `sim`, `wall` | `sim` | clock of the real driver's controller manager |
| `sim_state_rate` | Hz, `step` | `100` | rate of `/sim/joint_states`. `step` is every physics step and costs real-time factor |
| `replay_drive`, `replay_bms`, `replay_aux`, `replay_arm_l`, `replay_arm_r` | a `candump -L` file | empty | replay hook |

## Single-writer map

Machine-readable form: [`config/command_map.yaml`](config/command_map.yaml). `test_command_map.py` checks it against
the model. `single_writer_check.py` and scenario 9 check the running system against it.

**Gazebo owns the rigid-body dynamics of every joint in every profile.** The arm emulators emulate the CAN motor
devices only: they turn MIT frames into joint effort and joint state into reply frames.

### One command path per joint and per profile

| Joint | `fast`, `fine` | `firmware_sil`, `faults` |
|---|---|---|
| `left_wheel_joint`, `right_wheel_joint` | `/cmd_vel` > control bridge > DiffDrive plugin of the base | `/cmd_vel` > host test client > `vcan_drive` (SDO 60FFh, or RPDO 0x201 with `stm32:=renode_readonly`) > ZLAC8015D emulator > `/sil/drive/gz_cmd_vel` > `wheel_command_adapter` > one velocity input per wheel |
| `lift_joint` | `lift_controller` > effort interface of `gz_ros2_control` (`config/lift_controller_gazebo.yaml`) | `/sil/lift/command` and `/sil/lift/permit` > host test client > `vcan_aux` (RPDO1 0x210, permit 0x410) > CTL-017 emulator > velocity input of the lift |
| `openarm_{left,right}_joint1..7` | `{side}_arm_controller` > position interface of `gz_ros2_control` | `{side}_arm_controller` in `/controller_manager` > `openarm_hardware/OpenArmHW` > `vcan_arm_{l,r}` (CAN FD) > `{side}_arm_emulator` in `/sim/controller_manager` > effort interface. The tau field of the same MIT frame: `gravity_feedforward` > `{side}_arm_gravity_controller` > effort interface of the real driver |
| `openarm_{left,right}_finger_joint1` | `{side}_gripper_controller` > position interface | `{side}_gripper_controller` > real driver > `vcan_arm_{l,r}` (CAN FD) > `{side}_arm_emulator` > effort interface |
| `openarm_{left,right}_finger_joint2` | mimic joint, no controller. Owner by engine: the mimic constraint of Bullet Featherstone, else `finger_mimic_follower` | the same |
| castor joints | fixed in the description. Nothing moves them | the same |

### One writer per topic

| Topic | `fast`, `fine` | `firmware_sil`, `faults` |
|---|---|---|
| `/clock` | control bridge | control bridge |
| `/odom` | control bridge, from the DiffDrive plugin | `base_state_adapter`, from wheel speeds read over CAN1 |
| `/tf` `odom` > `base_footprint` | control bridge, from the DiffDrive plugin | `base_state_adapter` |
| `/tf` `map` > `odom` | `amcl` | `amcl` |
| `/tf` every other frame | `robot_state_publisher` | `robot_state_publisher` |
| `/joint_states` | `joint_state_publisher` | `joint_state_publisher` |
| `/base/joint_states` | control bridge, wheel states of Gazebo | `base_state_adapter`: wheels from CAN1, lift from CAN3 |
| `/arm/joint_states` | `joint_state_broadcaster` in Gazebo | `joint_state_broadcaster` of the real driver |
| `/sim/joint_states` | none | `joint_state_broadcaster` in `/sim`: what the physics does |
| `/cmd_vel` | `collision_monitor` | `collision_monitor` |
| `/cmd_vel_nav` | `controller_server` and `behavior_server`. Nav2 design, the behaviour tree runs one at a time | the same |
| `/sil/drive/gz_cmd_vel` | none | ZLAC8015D emulator |
| `/build1/drive/{left,right}_wheel_cmd_vel` | none | `wheel_command_adapter` |
| `/sil/lift/gz_cmd_vel` | none | CTL-017 emulator |
| `/build1/fingers/{left,right}_finger2_cmd_pos` | `finger_mimic_follower`, when the engine has no mimic | the same |
| `/local_costmap/footprint`, `/global_costmap/footprint` | `footprint_publisher` | `footprint_publisher` |
| `/planning_scene` | `planning_scene_loader` | `planning_scene_loader` |
| `/{left,right}_arm_gravity_controller/commands` | none | `gravity_feedforward` |
| `/sil/lift/command`, `/sil/lift/permit`, `/sil/*/inject`, `/sil/stm32/can1_silent`, `/sil/lift/permit_silent` | none | operator inputs. Nothing in the launch writes them |

Limit of the check: rclpy in Jazzy does not say which node sent a message. Frame ownership on `/tf` is checked
through the set of publishing nodes and through the rule that no frame has two parents.

### Conflicts of the phase 1 single-writer map

Source: audits `be17653`, `digital-twin/phase1/single_writer_map.md`.

| Conflict | In this scene | Single owner, and why | Test | Follow-up outside this repository |
|---|---|---|---|---|
| SW-01 `/cmd_vel` has two publishers | resolved by composition | `collision_monitor`. The docking nodes are not started: `dock_trigger` writes `/cmd_vel` too. The dock and its tags are in the world as a target only | `test_command_map.py::test_sw01_*`, scenario 9 | openamr-platform-sw: one arbiter for `/cmd_vel` (its #90 was not read) |
| SW-03 the ZLAC emulator and the DiffDrive plugin both shape the wheel command | fixed | the ZLAC8015D emulator. In the device profiles the DiffDrive plugin is taken out of the model and each wheel has one velocity input. `wheel_command_adapter` unfolds the emulator's twist with the same radius and separation, exactly. No second ramp, limit or geometry. The rig fails a run in which the emulator leaves Gazebo feedback for its internal plant | `test_sw03_*` in three test files, scenarios 1 and 9 | openamr-platform-sw #141: publish the per-wheel demand directly, and report the plant switch as an error |
| SW-06 `lift_joint` has a different owner in every stack | fixed per profile | device profiles: CTL-017 over CAN3, the only path. The lift leaves `ros2_control` in the plant model, `lift_controller` is not loaded and is not in MoveIt's controller list. `fast` and `fine`: `lift_controller`, because no CTL-017 exists there | `test_sw06_*`, scenarios 2, 3 and 9 | none |
| SW-08 firmware_sil has no complete `/tf`, `/odom` or `/joint_states` | fixed | `base_state_adapter` for `/odom`, `odom` > `base_footprint` and the base and lift joint states. `joint_state_publisher` for `/joint_states`. With `stm32:=renode_readonly` the wheel speeds come from `/sil/fw/wheel_speeds` | `test_sw08_*`, `test_odometry.py`, scenarios 1 and 9 | openamr-platform-sw #141: the host client integrates odometry on the host clock |
| SW-09 `finger_joint2` has no physics constraint | fixed | by engine: the mimic constraint of Bullet Featherstone, else `finger_mimic_follower` through a position controller on the joint. Never both | `test_sw09_*`, scenarios 4 and 9 | this repository: grippers on DART in `fast` and `fine`, see Known limits |
| SW-10 the odometry frame chain differs between simulation and the real path | documented | one chain in every profile: `odom` > `base_footprint` > `base_link`, the chain of the model. `/odom` is raw wheel odometry, no filter. The Gate A real path (`odom` > `base_link`, then a static `base_link` > `base_footprint`) cannot be published next to this model: `base_link` would have two parents. No 2.0 real bringup exists to copy | `test_sw10_*`, scenario 9 | openamr-platform-sw: decide the 2.0 chain and make `real_bringup` follow the model |

## Time

Time follows `docs/architecture/sil-topology.md` section 6 of openamr-platform-sw #141. Three clocks exist and they
are not synchronised.

| Clock | Used here for |
|---|---|
| Gazebo time on `/clock` | every ROS node of this launch, the real driver's controller manager, every limit of a scenario |
| host monotonic time | every timer inside the device emulators and inside the host test client |
| Renode virtual time | everything inside the firmware image, `stm32:=renode_readonly` only |

- The Gazebo real-time factor is measured by `rtf_monitor.py` and is in every scenario log.
- A Renode time ratio is a separate number. `openamrobot_sil` reports it in `/sil/fw/status`.
- A scenario waits in Gazebo time. Its wall limit is twenty times longer. A slow machine does not fail a check.
- One device rule cannot hold on a slow machine: the CTL-017 model faults when the Hall rate falls below half
  of nominal, which is a real-time factor of about 0.5. A lift scenario that meets this reports BLOCKED, not FAIL.
- `base_state_adapter` integrates wheel speeds over Gazebo time at local receipt. It does not use the pose of
  `/sil/base/odom`, which the host test client integrates on the host clock.

## Emulator limits

State these wherever a result depends on them. Each scenario log does.

- The ZLAC8015D emulator follows Manual 1.03 and CANopen Routine 1.08 as filed in platform-hw. The V4.2 package is
  still missing.
- The Daly BMS emulator follows a 2019 protocol copy. The protocol is not confirmed for the delivered unit.
- The OpenArm motor emulator has an assumed power-up state, assumed thresholds and assumed latency. Its README lists
  them.
- The CTL-017 emulator models a proposal. The permit COB-ID, period, timeout and refusal code are SIL test values.
- A virtual CAN bus has no bit rate, no arbitration, no error frame and no bus-off.

## Scenarios

Each is a launch test with its own timeout and writes `BUILD1_LOG_DIR/<name>.json`. PASS, FAIL or BLOCKED. BLOCKED
is never PASS.

| Test | Profile | What it measures |
|---|---|---|
| `scn01_drive` | `firmware_sil` | Nav2 drives 2 m and turns 90 degrees. Odometry against ground truth |
| `scn02_lift_permit` | `firmware_sil` | three lift heights. Without permit: refused, no motion. With permit: position |
| `scn03_arms_reach` | `firmware_sil` | MoveIt moves both arms to a shelf pose at L1000 and at L1350 |
| `scn04_grasp` | `firmware_sil` | open, reach, close on the tote item, lift it. Corrected open and closed states of #66 |
| `scn05_bus_loss` | `faults` | loss on each of the five buses. What the stack does today |
| `scn06_stale_sensors` | `faults` | every sensor topic stops while Nav2 drives |
| `scn07_cancel_resume` | `firmware_sil` | cancel a drive and an arm goal, stop, resume |
| `scn08_arm_power_cut` | `faults` | arm motor power cut while holding a pose |
| `scn09_single_writer` | `firmware_sil` | the running system against the map. Frame kinds on the five buses |
| `scn10_profiles`, `scn11_profile_sil_fine` | `firmware_sil` | real-time factor, controller rates, missed updates at 1 ms and 1/1500 s |
| `scn12_profile_fast`, `scn13_profile_fine` | `fast`, `fine` | single writers, real-time factor, controller rate, missed updates |
| `scn14_drive_fast` | `fast` | scenario 1 without devices |
| `scn15_grasp_fast` | `fast`, Bullet Featherstone | scenario 4 without devices |
| `scn16_lift_fast` | `fast`, DART | the lift to stops 2, 0, 1, 2, 0 through `lift_controller`. Position on `/joint_states`; an action success without arrival fails |
| `scn17_base_yaw_fast` | `fast`, DART | equal, then equal and opposite wheel speeds with the castors present. Ground truth yaw against r * (wR - wL) / track and `/odom`. Diagnosis: see `docs/build1_lift_and_base_diagnosis.md` |
| `scn18_stop_during_nav` | `firmware_sil` | platform #90 software Stop engaged during a Nav2 drive: time and distance to stop, no motion while locked or with a goal sent into the Stop, release, drive finished. Functional stop, not the hardwired E-stop |
| `scn19_stop_gate_kill` | `firmware_sil` | the Stop gate process ended with SIGKILL during a Nav2 drive: the base has to stop and stay stopped while the goal still commands. Functional check, not safety evidence |

Fault scenarios record observations. Next to each stands the entry of
[`config/expected_reactions.yaml`](config/expected_reactions.yaml), a simulation-only reference. Those entries are
expected behaviour of emulators and of functional software. They are not certified safety behaviour. A difference
between observation and reference is reported and does not fail the test.

Bus loss is injected with the hooks of the emulators and of the host test client. The interfaces stay up.
Stale sensors are injected by stopping the process of the sensor bridge (`inject_stale_sensors.py`).

## CAN FD

The arm buses are CAN FD. Nothing here assumes classic CAN behaviour.

- `setup_vcan_all.sh` sets MTU 72 on `vcan_arm_l` and `vcan_arm_r` and MTU 16 on the three base buses, and fails with
  BLOCKED if it cannot. It assumes no default: a new vcan interface has MTU 2060 (CAN XL) on recent kernels, as on the
  CI runner, and 16 on older ones.
- The launch refuses arm buses without MTU 72. It does not fall back to classic CAN.
- `test_canfd_vcan.py` sends FD frames with bit rate switch, reads them back as 72 bytes, shows that a classic-only
  socket does not see them, and shows that the base buses refuse them.
- Scenario 9 listens on all five buses of the running system: only FD frames on the arm buses, only classic frames on
  the base buses.

## Replay hook

`replay_<bus>:=<file>` plays a `candump -L` trace into that bus in place of the bus's emulator. The emulator of that
bus is not started. `can_replay.py` is the tool. It refuses an interface whose name does not start with `vcan`.

`test_replay_vcan.py` plays the synthetic trace `traces/synthetic_bms_26v4_55pct.log` into `vcan_bms`. The unchanged
host test client then reports 26.4 V and 55 %, values that only the trace holds.

## RViz checklist

`rviz:=true` opens `rviz/build1.rviz`. Fixed frame `map`. Tick each line.

| Check | Display | What to see |
|---|---|---|
| TF tree | TF | one tree: `map` > `odom` > `base_footprint` > `base_link` > lift > arms. No frame flickers |
| Joint states | RobotModel | wheels turn when the base drives, the lift carriage rises, the fingers open |
| Footprint | Footprint (local costmap) | the octagon of the base. It grows when an arm reaches past it |
| Odometry | Odometry | arrow on the robot. Compare with the ground truth arrow: the gap is the odometry error |
| Scan | LaserScan | `/scan_filtered`: shelf posts, obstacles, dock. No return on the robot's own body |
| Depth | Base depth, Head depth | depth images of the base and head cameras. Simulation cameras, unverified parameters |
| Camera topics | Dock camera, Wrist cameras | `/rgb_image` shows the dock tags from the origin. Wrist images follow the hands |
| Collision objects | PlanningScene | shelves, tote, item, payload, obstacles, in the right place relative to the robot |
| Planning scene | MotionPlanning | plan `left_arm` to `hands_up`: the plan avoids the work shelf |
| Costmaps | Local costmap, Global costmap | the global map shows the scene. The local map shows what the scan sees now |

## Known limits

- **Engine.** With the pinned models neither engine runs the whole robot. Measured on 15 Nov 2026, `fast` profile:

  | Engine | Base: 6 s at 0.2 m/s, then 6 s at 0.5 rad/s | Grippers in `fast` and `fine` | `finger_joint2` |
  |---|---|---|---|
  | DART (default) | 1.200 m ground truth for 1.200 m odometry, 168.3 degrees for 171.9 | do not move | `finger_mimic_follower` |
  | Bullet Featherstone | 0.866 m for 1.200 m, 0.0 degrees for 171.7 | move | mimic constraint |

  Three contact experiments did not change the Bullet result. The launch prints the limit of the chosen engine.
  Use DART for every run that drives. Use Bullet Featherstone for gripper work in `fast` and `fine`, at a station.
- **Castors.** The description holds the castors on fixed joints. They are frictionless sliders, not swivel castors.
- **Speed.** On a 4-core machine without a graphics card the full stack ran at a real-time factor of about 0.25.
- **Docking.** The dock and its three tags are in the world. No docking node runs. See SW-01.
- **MoveIt and the base.** MoveIt's model has a fixed base. `planning_scene_loader` places the scene objects relative
  to the robot from its pose estimate and sends them again when the base has moved.
- **Lift in MoveIt.** In the device profiles MoveIt cannot move the lift. The lift moves between calibrated stops
  through CTL-017 only.
- **Seeds.** No seed is set. A repeated run is not guaranteed to be identical.
- **Emulator rate at the 1/1500 s step.** `gz_ros_control` reports that the controller period is faster than the
  simulation period: the step is written 0.00066667 s, 10 ns longer than 1/1500 s. Measured on the CI runner, the
  simulator's controller manager then publishes at 750 Hz, every second step, not at 1500 Hz. At the 1 ms step it
  published at 616 Hz of the 1000 Hz asked for, at a real-time factor of 0.27.
- **MoveIt from the activation pose.** With the real driver, joint 4 rests on its lower limit and reads about
  0.0002 rad below it: half a step of the Damiao position field, which truncates. MoveIt refuses a start state outside
  the limits. With devices `joint_state_clamp` allows one step (3.8e-4 rad) and no more. The scenarios still leave
  the activation pose by a joint command to the park pose first.
- **Host test client on a slow machine.** The drive thread of `openamrobot_sil` (#141) ends on an SDO reply it does
  not expect. The base then cannot be commanded. The rig detects it and reports BLOCKED.
- **Gripper through the real driver.** The upstream driver reports finger velocity as 0, so the gripper controller
  reports a stall one second into every move that has not reached its goal. The flag is all it changes: the command
  stays and reaches the motor. The fingers that did not close in `firmware_sil` on 16 Nov 2026 were held by the
  shelf board under a hand that had settled 7 cm low (`docs/build1_arm_diagnosis.md`).
- **Arm accuracy through the real driver.** Without the gravity feed-forward the driver's gains leave the arm below
  every pose: 0.145 rad on joint 1 and 74 mm at the hand for the grasp pose (MuJoCo, the same gains). With it the
  scenarios check commanded against read joint positions, weighed by the driver's kp: at most 0.5 N m of servo
  torque left (`*_servo_torque_max`; `*_tracking_error_max` is recorded in rad).
- **Lift in DART.** Through the position interface of `gz_ros2_control` the lift could not leave its lower limit
  (L1000) in `fast` and `fine`, and `lift_controller` still reported success. Cause and fix:
  [`docs/build1_lift_and_base_diagnosis.md`](../../../docs/build1_lift_and_base_diagnosis.md). `fast` and `fine` now
  drive the lift through an effort interface with a goal tolerance (`config/lift_controller_gazebo.yaml`), added by the
  launch to the plant model only. The device profiles keep the CTL-017 velocity input; the same DART limit applies
  there when the lift rests on its lower limit (see that document).
- **Lift on a slow machine.** On the CI runner the full device stack ran at a real-time factor of 0.26 to 0.50.
  The CTL-017 model needs about 0.5 while the lift moves. Lift checks report BLOCKED there.

## Unverified parameters

| Parameter | Value | Status |
|---|---|---|
| floor friction | 0.8 | estimated, `config/scene.yaml` |
| aisle width, shelf size, tote, item, obstacles | `config/scene.yaml` | estimated. The Use Case 1 baseline states none of them |
| wheel and castor friction | openamr-platform-sw #135 | simulation estimates there |
| drive track 0.400 m | openamr-platform-sw | placeholder there |
| base, head and wrist camera sensors | `urdf/build1_robot.urdf.xacro` | field of view, resolution, rate and range unverified. The head camera is one RGB-D sensor for a stereo unit |
| finger follower gains | P 20, D 0.5, limit 7 N*m | working values |
| footprint padding of arm points | 0.08 m | estimated |
| grasp depth of the hand | 0.105 m | estimated from the model |
| joint damping and friction of the arms | openamr-upperbody-sw #66 | sourced there, no bench recording |

## Tests

```bash
colcon test --packages-select openamrobot_build1_scene --ctest-args -R "test_(scene|world_and_map|launch_support|command_map|footprint|odometry|candump|docs_and_pins)$"
```

Those eight need no simulator and no virtual CAN. `test_canfd_vcan` and `test_replay_vcan` need virtual CAN.
The `test_scn*` tests need Gazebo and several minutes each. `-DBUILD1_SCENARIOS=OFF` leaves them out.
