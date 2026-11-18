# Simulators: what each one integrates (demo capability matrix)

Status on 18 Nov 2026, branch `demo/grocery-mujoco-isaac` (stacked on #80 and #69). One row per simulator, one column per
capability. Simulation is development evidence only: nothing on this page is hardware, physical acceptance or safety evidence.

Overall status, one of four:

- **fully integrated**: the robot model, ROS 2, controllers and the navigation and manipulation stacks run together in this simulator.
- **partially integrated**: some of those run; the cells say which.
- **visual-only**: draws the robot and data from elsewhere; no physics of its own.
- **BLOCKED**: cannot run in the available environment; not a pass.

Cell values: **yes** (runs, with evidence), **partial** (runs with a stated gap), **no** (not built), **BLOCKED** (built, not run).

## Matrix

| | Gazebo Harmonic (DART) | MuJoCo 4.0.1 | Isaac Sim 5.1 | Renode / firmware SIL | RViz |
|---|---|---|---|---|---|
| **Overall** | **fully integrated** (simulation) | **partially integrated** (model and physics only) | **BLOCKED** (no GPU) | **partially integrated** (proposals) | **visual-only** |
| Model source | yes: `combined_robot.xacro`, `hardware_profile:=gazebo` [G1] | yes: MJCF generated from the same expansion, hash checked against the lock [M1] | partial: import job (URDF of the same expansion, one declared 0.1 kg root placeholder) [I1]; USD import BLOCKED | no robot model: the firmware image runs against emulated devices [R1] | yes: `robot_state_publisher` on the same xacro [V1] |
| ROS 2 integration | yes: `gz_ros2_control`, `ros_gz_bridge` [G1] [G2] | **no** | **no** (no ROS bridge configured) | partial: micro-ROS from the firmware, proposed [R2] | yes (display only) |
| Firmware path | partial: OpenArm SIL on virtual CAN with the real driver and emulated motors [G3]; base and lift emulators on virtual CAN, draft [G4] | **no** | **no** | partial: STM32 image boots in Renode, read-only, proposed [R1] [R2] | no |
| Controllers | yes: ros2_control trajectory, gripper and lift controllers [G1] [G2] | partial: the MJCF position and velocity actuators, driven by a script (`pick.py`); no ros2_control | BLOCKED: USD drives from the job [I1], never run | partial: the firmware's own loops against emulators | no |
| Sensors | partial: LiDAR on `lidar_link`; base camera bound to the legacy `camera_link`, not `base_camera_link` [A1] | partial: four MJCF cameras rendered offscreen; base camera on the legacy `camera_link` [A1]; no LiDAR, IMU or range sensors | BLOCKED: RTX cameras at `base_camera_link` (Gemini 336L), head and wrist links, RTX LiDAR mount at `lidar_link`, written to the stage [D1]; render never run | partial: emulated device data on CAN | yes (draws Gazebo topics) |
| Contact | partial: DART; robot-link contact sensors do not report, finger forces come from the item's sensor [G2] | yes: soft contact, elliptic cone, convex hulls of the meshes; grasp-and-lift sweep [D1] | BLOCKED: PhysX, never run | no | no |
| Nav2 | yes [G1] [G2] | **no** | **no** | no | yes (display) |
| MoveIt | yes: scn03 and scn04 plan with MoveIt [G2] | **no** (the pick uses a script IK) | **no** | no | yes (display, planning plugin) [V1] |
| CI | Build 1 scene workflow on #70 and #73 [G2] | `Simulators` (generation, parity, physics) [M1]; `Grocery MuJoCo and Isaac` (this branch) [D1] | `isaac` jobs BLOCKED without a GPU runner [I1] [D1] | proposals' own workflows [R1] [R2] | none |

## What the grocery demo adds (this branch)

- MuJoCo: a grocery dark store around the generated robot MJCF, included byte for byte; a grasp-and-lift contact experiment; a
  recording. The base drive in the recording is kinematic; the pick is controlled in MuJoCo physics by a script. Nothing here is ROS,
  ros2_control, MoveIt, Nav2 or firmware. See [grocery_scene.md](grocery_scene.md).
- Isaac Sim: the store as a USD stage that references the robot USD of the #80 importer unchanged and puts RTX camera and LiDAR prims
  on the authoritative frames. Checked on the CPU with usd-core; the import and the render are BLOCKED without a GPU.

## Known differences that matter for the demo

- Base camera frame (P12 of the digital-twin parity audit): the Gazebo sensor and the generated MJCF camera sit on the legacy
  `camera_link`, 157 mm above `base_camera_link` and pitched differently. The Isaac stage of this branch uses `base_camera_link`,
  the Gemini 336L frame of the decisions register (CAMERAS). The MuJoCo recording uses free cameras only, so it shows no robot
  camera view.
- Lift speed (P07): the generated MJCF does not enforce 18 mm/s. The grocery scripts ramp the lift setpoint at 18 mm/s in the
  controller; the model is unchanged.
- Centre of mass (P13): 17 mm behind `mass_summary.json` at L1000 in every model, cause open. Not changed here.

## Evidence

- [G1] Gazebo on `main`: [docs/v01_visual_check.md](../v01_visual_check.md) (`sim.launch.py`, RViz, lift planning group);
  openamr-platform-sw Nav2 and docking packages.
- [G2] Build 1 world, draft PRs [#70](https://github.com/openAMRobot/openamr-upperbody-sw/pull/70) and
  [#73](https://github.com/openAMRobot/openamr-upperbody-sw/pull/73); run
  [43991379633](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/43991379633) (scn04 lifted the item 71 mm,
  the job failed on its servo-torque check), diagnosis in #73 `docs/build1_arm_diagnosis.md`.
- [G3] OpenArm SIL, merged: [#67](https://github.com/openAMRobot/openamr-upperbody-sw/pull/67).
- [G4] Base and lift SIL emulators (ZLAC8015D, CTL-017, Daly BMS) on virtual CAN, draft branch in openamr-platform-sw,
  `docs/architecture/sil-topology.md` (Proposed).
- [M1] MuJoCo generator and parity: [#69](https://github.com/openAMRobot/openamr-upperbody-sw/pull/69), run
  [43750447835](https://github.com/openAMRobot/openamr-upperbody-sw/actions/runs/43750447835) on `4270e53` (mujoco job:
  87 tests passed; generation `hash in lock, inputs changed`).
- [I1] Isaac import job and single articulation: [#69](https://github.com/openAMRobot/openamr-upperbody-sw/pull/69),
  [#80](https://github.com/openAMRobot/openamr-upperbody-sw/pull/80); the `isaac` job is BLOCKED (no GPU runner).
- [R1] [openamr-platform-fw#51](https://github.com/openAMRobot/openamr-platform-fw/pull/51) (telemetry image boots in Renode,
  proposal), [openamr-platform-fw#29](https://github.com/openAMRobot/openamr-platform-fw/pull/29) (experimental Renode boot in CI).
- [R2] [openamr-platform-sw#141](https://github.com/openAMRobot/openamr-platform-sw/pull/141) and
  [#144](https://github.com/openAMRobot/openamr-platform-sw/pull/144) (firmware in Renode on the drive bus, micro-ROS; proposals).
- [V1] RViz captures in `evidence/p25/rviz/` and `evidence/p28/rviz/`; `docs/v01_visual_check.md`.
- [A1] Digital-twin parity audit (internal), checks P07, P12 and P13.
- [D1] This branch: [grocery_scene.md](grocery_scene.md), workflow `.github/workflows/grocery-mujoco-isaac.yml`.
