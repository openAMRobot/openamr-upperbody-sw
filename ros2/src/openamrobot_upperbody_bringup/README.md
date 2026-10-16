# openamrobot_upperbody_bringup

Launch files for the upper body: the model viewer, the mock bringup, and the MoveIt launch (A7). None of them starts the mobile base on its
own (`sim:=gazebo` below starts it through the simulator).

## View the model

```bash
ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000
```

`robot_state_publisher`, `joint_state_publisher_gui` and RViz2 on the combined description (it moved here from
`openamrobot_upperbody_description`). Source the workspace overlay first; the meshes are `package://` URIs. Args:
`column`, `base_plate`, `head_pitch`, `base_tilt` (option B's configuration IDs), `arm_package`, `enable_planar_base`. The viewer's
`joint_state_publisher_gui` has a slider for `lift_joint`.

## Mock bringup

```bash
ros2 launch openamrobot_upperbody_bringup mock.launch.py [base_plate:=bp000] [rviz:=true]
```

`robot_state_publisher` on the combined description with `hardware_profile:=mock`, `ros2_control_node` with
`openamrobot_upperbody_control/config/arm_controllers.yaml`, and spawners for `joint_state_broadcaster`,
`left_arm_controller`, `right_arm_controller`, `left_gripper_controller` and `right_gripper_controller`. `rviz` defaults to
`false`. The arms run on `mock_components/GenericSystem`, which reflects commands back as state.

```bash
ros2 control list_controllers
ros2 control list_hardware_interfaces
ros2 action send_goal /left_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [openarm_left_joint1, openarm_left_joint2, openarm_left_joint3, openarm_left_joint4, openarm_left_joint5, openarm_left_joint6, openarm_left_joint7], points: [{positions: [0.2, 0, 0, 0.3, 0, 0, 0], time_from_start: {sec: 2}}]}}"
```

## Test

`test/test_l01_mock_controllers.py` (verification check **L01**, `launch_testing`) starts `mock.launch.py` headless, waits
up to 30 s for all five controllers to be `active`, sends one trajectory to `left_arm_controller` and checks that
`/joint_states` follows it within 0.02 rad while the right arm stays put. It runs in its own random `ROS_DOMAIN_ID`,
localhost only. Not covered: a real launch (the `real` profile is checked by expansion in the description package), and tracking
performance, which mock hardware cannot show. The Gazebo launch and its tests (G01, G02) are in `openamrobot_upperbody_gazebo`.

## MoveIt

```bash
ros2 launch openamrobot_upperbody_bringup moveit.launch.py [base_plate:=bp000] [sim:=mock|gazebo] [rviz:=false] [lift_position:=0.0] \
                                                           [headless:=false] [physics_engine:=<gz-physics plugin>]
```

| Argument | Default | |
|---|---|---|
| `column`, `base_plate`, `head_pitch`, `base_tilt` | `dold_v4_350`, `bp000`, `hp25`, `bt10` | option B's configuration IDs (its baseline); `column:=tl3_400` does not expand (no supplier data) |
| `lift_position` | `0.0` | `sim:=mock` only: the position (m) of `lift_joint` the stub reports; `0.0` / `0.175` / `0.350` are the calibration stops `L1000` / `L1175` / `L1350` |
| `sim` | `mock` | `mock`: `mock.launch.py`, a stub for the wheel and lift joints, then `move_group`. `gazebo`: the gazebo package's `sim.launch.py`, then `move_group` on the simulator's clock |
| `rviz` | `false` | RViz2 with the MotionPlanning display (`openamrobot_upperbody_moveit/config/moveit.rviz`). With `sim:=gazebo` it replaces the simulator's own RViz |
| `headless` | `false` | `sim:=gazebo` only: `true` runs `gz sim -s`, as the checks do |
| `physics_engine` | empty | `sim:=gazebo` only, passed to `sim.launch.py`. Empty means that launch's own default, **DART**, on which a `closed` gripper pose may not close. Pass `gz-physics-bullet-featherstone-plugin` for the grippers to close (M02 does) |

`base_joint_states_stub.py` publishes the base's two wheel joints at zero and `lift_joint` at `lift_position` on `/joint_states`, for
`mock` only: neither the base nor the lift is under `ros2_control` (ADR-003; the lift is a joint we have no authority to command), so
nothing there reports them, MoveIt would warn every second that the robot's state is incomplete, and `robot_state_publisher` publishes
nothing above the lift. The base does not move during manipulation (AD-30), so zero is the true state of the wheels. The lift is not at
zero by nature: its position is the operating point of the run (a calibration stop, a calibration ID and not a configuration ID). In
Gazebo the wheel states come from the simulator and `lift_joint` from `joint_state_publisher`'s default, zero, which is the simulator's
own resting state (the unactuated carriage sits on its lower limit) but is not read from the physics.

## A7 tests

`test/test_m01_mock_named_poses.py` (**M01**, once per lift stop `L1000`, `L1175`, `L1350`: 3 runs, the stub's `lift_position` set from
the stop) and `test/test_m02_gazebo_named_poses.py` (**M02**, the baseline, lift at `L1000`)
send every SRDF named pose to `move_group` (both arms: `hands_up`, `home`, `hands_up`, `park`; both grippers: `closed`, `open`, each with its
own value), which plans and executes through the trajectory execution manager into the real controllers, and require a **final joint error
under 0.01 rad** after a one second settle. The record (a sidecar per run) holds every pose's outcome, planning and wall time and error, and
the joint-state clamp's counters (see the MoveIt package README, "The joint-state clamp": MoveIt refuses a start state any amount past a limit,
and a simulated joint resting on its limit reads back a hair past it, so `move_group` plans from a copy of `/joint_states` with that noise taken
off, at most 1e-5 rad of it, and the sidecar carries how often that happened and the largest amount). There is **no retry**: a refused request
is a failed pose. M02 starts a headless Gazebo server in its own ROS domain and Gazebo partition and
passes `bullet-featherstone` explicitly. `simulation` class throughout.

They **skip unless `OPENAMR_A7=1`** (`run_a7_regression.sh` sets it): `verify_a6.sh` runs this package's whole `colcon test`, and it must neither
start Gazebo nor depend on an A7 check. A skipped M01 or M02 is never a result; the A7 report treats a missing sidecar as `NOT_RUN`.

```bash
ros2 run openamrobot_upperbody_bringup run_a7_regression.sh      # about 15 minutes; G01, G02, M01-M06, writes evidence/<ts>_a7.json
```

`run_a7_regression.sh` runs the gazebo, moveit and bringup tests, then `build_a7_evidence.py` writes `<workspace>/evidence/<UTC timestamp>_a7.json`
(`profile: "a7"`, `evidence_class: "simulation"`). A tree with uncommitted changes prints its results and writes **no** file (P08's rule: a
report records a commit). Exit status: **1** if any check is `FAIL` or `NOT_RUN`; **3** if nothing failed but the tree is dirty (nothing recorded); 2 if the
evidence cannot be assembled; 0 otherwise. `A7_M01_STOPS=L1000,L1350` reduces M01 to those lift stops and the report says so.

## Not here

- The Gazebo launch alone is `ros2 launch openamrobot_upperbody_gazebo sim.launch.py` (the simulator and the base are started there).
- No lift controller: the lift is option B's prismatic joint, with effort 0 and velocity 0, and nothing here commands it.

See [`openamrobot_upperbody_description`](../openamrobot_upperbody_description/README.md).
