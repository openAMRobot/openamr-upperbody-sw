# Digital Twin Control Room

The control room shows a running OpenAMRobot 2.0 simulation (Build 1 scene, Baseline 1 mission, or any launch that
uses the same topic names) and writes an offline HTML report of a recorded run. It lives in
`ros2/src/openamrobot_control_room`.

**It is read only.** It holds no command authority and never goes around the command arbiter of the stack: it
publishes nothing outside `/control_room/`, sends no action goal, calls no service and writes no parameter. Everything
it shows is simulation data: never hardware, physical acceptance or safety evidence.

One demonstration, three views of the same run: **live** next to the running mission (`control_room.launch.py`),
**REPLAY** of that run's recording on its own simulation time (`replay.launch.py`), and the **offline report** of the
same recording (`mission_report.py`). All three read the same topics and the same mission JSON; none adds a component
to the stack. What each part of the run is (Gazebo physics, emulator, host stand-in, unmodified driver, virtual CAN,
Proposed topic) is shown in the status field `provenance`, the report's components table and
[can_signal_paths.md](can_signal_paths.md).

## What it shows and where it comes from

| Display | Source topics | Where |
|---|---|---|
| Live scene | robot model, TF, map, costmaps, footprint, planning scene; observer camera `/baseline1/observer/image` (spawned by the mission's recorder), head, base depth, wrist and dock cameras | RViz, Lichtblick 3D and Image panels |
| Mission state and phases | `/baseline1/mission_status` (**Proposed**, published by the Baseline 1 mission from this branch on); the grocery mission's `/grocery_demo/phase` and `/grocery_demo/events` (volatile: a phase published before the control room started is not seen); otherwise only activity inferred from the action status topics, labelled as inferred | `/control_room/status` field `mission` (with its `source`) |
| What ran | `/build1/run_info` of the launch (profile, `stm32`, engine, pose source) and the writers of `/cmd_vel`, `/cmd_vel_nav`, `/cmd_vel_smoothed`, `/sil/lift/command`, `/sil/lift/permit` read from the ROS graph | status `provenance`: each component labelled emulator, host stand-in, real ROS driver, virtual CAN or Gazebo physics; a warning when `/cmd_vel` has a writer other than `collision_monitor` |
| Nav2 goal, plans | `/plan` (its last pose is republished as `/control_room/nav_goal`; the status adds ground truth against it), `/local_plan` | RViz, status `nav` |
| Robot pose and localization error | `/amcl_pose` and the `map` to `base_footprint` transform against Gazebo ground truth `/build1/ground_truth/odom` | `/control_room/localization/position_error`, `yaw_error`, `tf_position_error`; RViz shows AMCL, odometry and ground truth arrows |
| Station error | ground truth against the station pose of `config/scene.yaml` (default `work_shelf`) | `/control_room/station_error`, status `station` (distance, along heading, left, yaw) |
| MoveIt trajectories | `/display_planned_path`, `/monitored_planning_scene`, `/move_action` and `/execute_trajectory` status | RViz MoveIt Trajectory and Planning Scene displays |
| Joint states, lift position | `/joint_states` (`lift_joint`), `/sil/gz/joint_states` (Gazebo physics) | `/control_room/lift/position`, status `lift` |
| Camera and LiDAR | `/scan_filtered`, `/scan`, the camera topics above | RViz, Lichtblick |
| Controller states | CTL-017 emulator: `/sil/lift/status` (the host stand-in's decode of TPDO1 and TPDO2), `/sil/lift/emulator_state`, `/sil/lift/permit` (read); ZLAC8015D emulator: `/sil/drive/status` (stand-in), `/sil/drive/emulator_state`; Daly BMS emulator: `/sil/bms/*`; `/sil/fw/status` exists only with Renode firmware (`stm32:=renode*`) and is shown as absent with that reason otherwise | status `lift`, `drive`, `bms`, `firmware` |
| Virtual CAN | receive-only sockets on `vcan_drive`, `vcan_aux`, `vcan_bms`, `vcan_arm_l`, `vcan_arm_r`; payloads decoded by `can_decode.py` (signal paths and layouts: [can_signal_paths.md](can_signal_paths.md)) | `/control_room/can/summary` (frames, rate per wall second and per Gazebo second, CiA 301 classes, top IDs), `/control_room/can/decoded` (latest engineering value of every signal, last raw frame per ID), `/control_room/can/<bus>/rate_hz` |
| Contacts, grasp, object motion | `/build1/contacts/*` (only with `BUILD1_CONTACT_SENSORS=1`), gripper action status, `/build1/ground_truth/finger_joint_states`, `/build1/ground_truth/item/pose` | `/control_room/contacts/max_force`, `/control_room/item/speed`, status `contacts`, `grasp`, `item` |
| Simulation time, real-time factor | `/clock`, `/build1/rtf`, `/build1/rtf_stats`, `/build1/run_info` | status `sim_time`, `rtf`, `run_info`; Lichtblick plot |
| Test results | the mission JSON (`baseline1_mission.json`) | offline report: phase timeline and phase table; live only through the Proposed mission status topic |

## Run it next to a simulation

Build once, from `ros2/` of a workspace that already builds the Build 1 scene:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to openamrobot_control_room openamrobot_build1_scene
source install/setup.bash
```

Terminal 1, the simulation (Baseline 1 configuration: firmware SIL, Nav2, MoveIt):

```bash
ros2 run openamrobot_build1_scene setup_vcan_all.sh
ros2 launch openamrobot_build1_scene build1.launch.py profile:=firmware_sil nav2:=true moveit:=true
```

Terminal 2, the control room (RViz and the monitors):

```bash
ros2 launch openamrobot_control_room control_room.launch.py
```

Terminal 3, the Baseline 1 mission against the running stack:

```bash
export BUILD1_LOG_DIR=$PWD/build1_logs
python3 -c "import sys; from openamrobot_build1_scene import baseline1, rig; r = rig.run('baseline1_mission', baseline1.mission, needs_devices=True, nav2=True, moveit=True); sys.exit(0 if r[0] == 'PASS' else 1)"
```

For the grocery dark store, start its launch in terminal 1 and the control room with `scene:=grocery_scene.yaml`
(its `packing` station is known only to that scene); the status warns when the scene differs from the run's
`/build1/run_info`. Pick the station for the station error with `station:=<name of the scene's stations>`.

Launch arguments: `rviz` (true), `foxglove` (false), `foxglove_port` (8765), `foxglove_address` (127.0.0.1),
`vcan` (true), `monitor` (true), `use_sim_time` (true), `station` (work_shelf), `scene` (scene.yaml).

### In a browser (Lichtblick)

```bash
ros2 launch openamrobot_control_room control_room.launch.py rviz:=false foxglove:=true
```

Open Lichtblick (MPL-2.0, a community fork of Foxglove Studio), connect with "Foxglove WebSocket" to
`ws://127.0.0.1:8765` and import the layout `ros2/src/openamrobot_control_room/config/lichtblick_layout.json`.
The layout has a 3D scene, the observer and head cameras, plots of localization error, station error, lift
position, real-time factor, CAN rates, contact force and item speed, the mission status, control room status and
CAN summary, and the log. It has no Teleop, Publish, Call Service or Parameters panel.

The bridge (`config/foxglove_bridge.yaml`) offers only the `connectionGraph` capability: no `clientPublish`,
`services`, `parameters`, `parametersSubscribe` or `assets`, and the allowlists of those paths match nothing. The 3D
panel's publish buttons therefore have nowhere to publish. Without `assets` the browser cannot fetch meshes from
`package://` URLs, so the robot is drawn from its primitives only.

### The Gazebo window

The control room does not start the Gazebo GUI: its world controls (pause, step, reset) and transform tools write to
the simulation. For a picture of the scene it uses camera topics instead.

## Offline mission report

```bash
ros2 run openamrobot_control_room mission_report.py --bag $BUILD1_LOG_DIR/bag \
  --mission $BUILD1_LOG_DIR/baseline1_mission.json --candump "$BUILD1_LOG_DIR/record_candump_*.log" \
  -o $BUILD1_LOG_DIR/control_room_report.html
```

One static HTML file, no external resources: phase timeline and phase table from the mission JSON, station error,
localization error, left and right arm joint states, lift position (joint states and Gazebo physics), item height,
real-time factor, CAN frame rate (candump wall time), the decoded CAN signals (drive wheel targets and actuals in
m/s, lift position and velocity, left arm MIT command and feedback positions, BMS SOC and voltage, and the first raw
frame of every bus and ID with its decode, all on candump wall time), the components of the run from its
`/build1/run_info`, the mission status messages, and contact force (when contact sensors were recorded), on the
Gazebo time base with the phases as shaded bands. The station pose comes from `config/scene.yaml` of
`openamrobot_build1_scene` (`--station X Y YAW` or `--scene PATH` to override). The verdicts are the mission's; the
page computes none.

`--synthetic DIR` writes a small synthetic bag and mission JSON first and reports them; `--synthetic-plain` renders
the same fixture without ROS. The CI job `control-room-tests.yml` writes the synthetic report on every run and, when
dispatched with a Baseline 1 acceptance run id, the report of that run's recorded bag; both are in the job's artifact.

![Report from the synthetic fixture](media/control_room_report_synthetic.png)

The picture is the synthetic fixture: invented numbers, not a robot run.

## REPLAY of a recorded run

```bash
ros2 launch openamrobot_control_room replay.launch.py bag:=$BUILD1_LOG_DIR/bag \
  mission:=$BUILD1_LOG_DIR/baseline1_mission.json [rate:=1.0] [start:=SIM_S] [end:=SIM_S]
```

`control_room_replay.py` plays one or more bags (comma separated, merged in time order) on their recorded simulation
time and publishes every recorded non-command topic on `/control_room/replay/<topic>`, the recorded clock on
`/control_room/replay/clock`, and on `/control_room/replay/status` the mode `REPLAY`, the recorded simulation time
and the mission phase and verdict at that time from the mission JSON; `/control_room/replay/label` puts a REPLAY text
with the phase above the recorded base pose. RViz2 runs with `rviz/control_room_replay.rviz` and `/tf`, `/tf_static`
and `/clock` remapped to `/control_room/replay/`, so it draws the recording and never the live graph, even next to a
running simulation. Command topics of the recording (`/cmd_vel*`, controller commands, the lift command and permit,
action topics) are never replayed and are listed in the status. Lichtblick can open the bag file directly with
`config/lichtblick_layout.json`; that view is a file, not live data.

## How read only is enforced and tested

- Every publisher goes through `readonly.guard_publisher_topic`, which refuses any topic outside `/control_room/`.
  The nodes are created without parameter services and create no service or action client.
- `test_monitor_graph.py` reads both monitor nodes from a second node on a live ROS graph: publishers only under
  `/control_room/` (plus `/rosout`, `/parameter_events`), no service client, no action client or server, no
  parameter service. The monitor does not subscribe to any `_action/feedback` topic, because the ROS graph counts
  such a subscriber as an action client.
- `test_foxglove_readonly.py` starts the bridge with the control room configuration and connects a WebSocket client:
  no write capability is offered, a client advertise and messages on `/cmd_vel` create no ROS publisher and deliver
  nothing, a parameter write leaves the probe node's parameter unchanged, an asset request returns no asset.
- `test_vcan_listen.py` sends frames on a test vcan interface and counts every frame on it with a second socket: the
  count equals what was sent, so the monitor transmitted nothing. `ReceiveOnlyBus` has no send method.
- `test_static.py` checks the RViz configuration (view, select, focus and measure tools only; no 2D Goal Pose,
  2D Pose Estimate, Publish Point, Interact or MotionPlanning), the layout and the bridge configuration, that the
  launch file starts only the four monitoring components, and that no source calls a client, goal or parameter API.
- The CI job also starts the launch file and fails if any node it started publishes outside `/control_room/` or has
  a service or action client, and compares the installed package with its source tree (`tools/check_installed.py`).
- `test_topic_audit.py` checks every topic the monitor reads against the topic lists of two recorded runs
  (`test/fixtures/runs`); a topic no run has must be labelled absent with its reason. `test_can_decode.py` decodes
  frames recorded in those runs and checks them against the same runs' status topics and the driver's gains.
- `test_replay.py` plays a bag with a recorded `/cmd_vel` and checks that only `/control_room/replay/` topics come out
  and that `/cmd_vel` is not replayed.

## Proposed

- `/baseline1/mission_status` (std_msgs/String JSON, latched), published by the Baseline 1 mission from its phase
  tracker (`baseline1_phases.Tracker(on_status=...)`, `baseline1.status_publisher`): current phase, state and the
  verdict of every phase, at each phase begin and end and once at the end. Implemented on this branch as Proposed;
  not accepted. Contract change request: openamr-upperbody-sw issue 82. The grocery mission keeps its own
  `/grocery_demo/phase`.

## Limitations

- The live Baseline 1 phase comes from the Proposed topic; a launch without it shows only inferred activity. The
  grocery phase topic is volatile: a control room started late misses the phases before it.
- AMCL against ground truth assumes the map frame equals the Gazebo world frame, as in Baseline 1 (AMCL starts at the
  spawn pose at the world origin). A launch with another map origin shows an offset.
- Contact forces exist only when the scene runs with `BUILD1_CONTACT_SENSORS=1`; the Baseline 1 bag does not record
  them.
- The CAN decode follows the emulators' and the host stand-in's frame layouts (can_signal_paths.md). They are not
  confirmed device protocols: the CTL-017 dictionary is a proposal, the Daly protocol is unconfirmed.
- The CAN plots of the report use candump wall time, not Gazebo time.
- The replay has no map (the recorders do not record `/map`) and no MoveIt displays.
- Lichtblick draws the robot without meshes (no `assets` capability).
- The foxglove_bridge node keeps its own ROS parameter services (a ROS 2 node default). They are reachable from the
  ROS graph, not from the browser; the bridge's settings are read at start.
- No Nav2 feedback (distance remaining, recoveries): reading it would make the monitor look like an action client.
- RViz and Lichtblick were not rendered headlessly for this page; the only picture is the report of the synthetic
  fixture.
