# CAN signal paths of the firmware_sil simulation

Where every command and every feedback value travels in the Baseline 1 and grocery runs (`openamrobot_build1_scene`
profile `firmware_sil`, `stm32: host`), from the producer to the Gazebo joint and back. Derived from the code, not
from a datasheet:

- openamr-upperbody-sw `integration/baseline1` (024ccb8): `openamrobot_build1_scene` (build1.launch.py,
  launch_support.py, base_state_adapter.py, wheel_command_adapter.py, baseline1.py, urdf/build1_robot.urdf.xacro),
  `openamrobot_openarm_emulator` (#67: codec.cpp, arm_emulator.cpp, config/sim_controllers.yaml).
- openamr-platform-sw `integration/baseline1` 931e631: `openamrobot_sil` (sil.launch.py can_nodes, stm32_host.py,
  zlac_gazebo.py, ctl017_gazebo.py, daly_bms.py, vendor/can_sil, config/sil.yaml), `openamrobot_nav2`
  (navigation_launch.py, nav2_params.yaml).
- enactic/openarm_ros2 4e837e1 (openarm_hardware, the unmodified driver) with openarm_can bfd4e52 and
  openarm_description 14ff67b, as `config/pins.yaml`.
- The frames themselves: candump logs of Baseline 1 run 44309817813 and grocery run 44299879412, cut into
  `ros2/src/openamrobot_control_room/test/fixtures/runs` and decoded by `openamrobot_control_room/can_decode.py`
  (`test/test_can_decode.py` checks the decode against the runs' own status topics).

Nothing on these buses is a real device. `stm32: host` means the STM32 base controller is the host test stand-in
`stm32_host_client` (openamrobot_sil stm32_host.py), not firmware, and not firmware in Renode. Simulation evidence
only: never hardware, physical acceptance or safety evidence.

## What each part is

| Part | What it is in these runs |
|---|---|
| STM32 base controller | host test stand-in `stm32_host_client` (Python, host process). Renode firmware runs only with `stm32:=renode` or `renode_readonly`, not in these runs |
| ZLAC8015D drive | emulator `zlac8015d_emulator` (vendored zlac8015d_emulator.py with zlac_gazebo.py), host process |
| CTL-017 lift controller | emulator `ctl017_emulator` (vendored ctl017_emulator.py, CTL-017 v1 proposal of openamr-upperbody-fw #15, with ctl017_gazebo.py), host process |
| Daly BMS | emulator `daly_bms_emulator` (daly_bms.py), protocol unconfirmed for the delivered unit |
| OpenArm Damiao motors | emulated by `OpenArmCanEmulator`, a ros2_control controller in the simulator's controller manager (`/sim`), at the 1 kHz physics step |
| OpenArm driver | the real ROS 2 driver, unmodified: openarm_ros2 `openarm_hardware` with openarm_can, in the root controller manager |
| CAN | Linux virtual CAN (vcan): no bitrate, no arbitration delay, no electrical layer. The arm emulator adds its own bus timing model to its replies (bus_timing.cpp) |
| Command arbiter | none launched. `/cmd_vel` has one writer, Nav2's `collision_monitor`; the control room lists the writers from the live graph |
| Plant | Gazebo Harmonic physics (DART), 1 ms step |

## Clock sources

| Producer | Clock |
|---|---|
| Gazebo | simulation time on `/clock` |
| host stand-in, ZLAC8015D, CTL-017 and BMS emulator nodes, base_state_adapter, wheel_command_adapter | Gazebo time (`use_sim_time`, clock contract C1 and C4 of sil-topology.md): their cycles stop while Gazebo is paused |
| openarm_ros2 driver | Gazebo time at 500 Hz (`driver_clock: gazebo`, `driver_rate_hz: 500` in `/build1/run_info`) |
| arm emulator | the physics step, 1000 Hz |
| candump time stamps | host wall clock. Not Gazebo time: the real-time factor of these runs is about 0.05 to 0.2 |
| bag receive times | Gazebo time (`ros2 bag record --use-sim-time`) |

## Signal paths

### Base drive (vcan_drive, CAN1 slot)

| Step | Value |
|---|---|
| Command producer | Nav2 `controller_server` and `behavior_server` (backup) on `cmd_vel_nav` > `velocity_smoother` > `cmd_vel_smoothed` > `collision_monitor` > `/cmd_vel` (geometry_msgs/Twist) |
| Into CAN | host stand-in subscribes `/cmd_vel`, converts to wheel r/min with wheel radius 0.098 m and separation 0.400 m (sil.yaml) and rounds to whole r/min: 1 r/min is 0.0103 m/s at the rim |
| Transport | vcan_drive, CANopen node 1, expedited SDO. 601h (stand-in to drive): bring-up 60FFh:1 and :2 = 0, 6060h = 3 (profile velocity), 2000h = 250 ms (link loss), 6040h 06h, 07h, 0Fh; then every 20 ms of Gazebo time 60FFh:1 (left) and 60FFh:2 (right) target velocity int32 r/min, read 606Ch:1 and :2 (0.1 r/min), every fifth cycle read 6041h statusword and 603Fh error code. 581h: confirmations and read replies. Heartbeat 701h every 100 ms. RPDO1 201h and RPDO2 301h exist only with `renode_readonly` |
| Emulator | ZLAC8015D emulator: reduced CiA 402 state machine, ramp, 2000h link-loss stop |
| Controller state | 6041h statusword (recorded: 55 = 37h, ready, switched on, operation enabled, voltage, no quick stop), 603Fh fault bits (recorded: 0), NMT state on 701h (operational) |
| To the plant | emulator demand > `/sil/drive/gz_cmd_vel` (Twist) > `wheel_command_adapter` > `/build1/drive/{left,right}_wheel_cmd_vel` (Float64, rad/s) > ros_gz_bridge > `/build1/gz/{left,right}_wheel_cmd_vel` > gz-sim JointController, velocity mode |
| Plant | Gazebo joints `left_wheel_joint`, `right_wheel_joint` |
| Feedback producer | Gazebo `/sil/gz/joint_states` (JointStatePublisher system, bridged) > emulator wheel feedback > 606Ch > SDO read reply 581h > host stand-in |
| ROS feedback | `/sil/drive/status` (JSON: configured, statusword, fault_code, target_rpm, wheel_rad_s, sdo_errors) and `/sil/base/odom` (twist) > `base_state_adapter` > `/odom`, TF odom > base_footprint, `/base/joint_states` > `joint_state_publisher` > `/joint_states` |
| Emulator state | `/sil/drive/emulator_state`: published by the node every 50 ms of Gazebo time, yet absent from both recorded bags (cause not found; see Open points) |

### Lift (vcan_aux, CAN3 slot)

| Step | Value |
|---|---|
| Command producer | the mission (baseline1.py `stop_authorize`, `lift_to_target`): `/sil/lift/permit` (Bool) and `/sil/lift/command` (Int8: 0, 1, 2 move to stop L1000, L1175, L1350; -1 stop, -2 homing, -3 clear fault) |
| Into CAN | host stand-in `LiftHost`: NMT start 000h, core heartbeat 701h every 100 ms, RPDO1 210h `[seq, op, stop]` (op 0 STOP, 1 MOVE_TO_STOP, 2 CLEAR_FAULT, 3 START_HOMING) repeated as keep-alive every 100 ms, permit PDO 410h every 50 ms |
| Permit PDO | 410h, SIL test layout of sil.yaml (not a contract): flag u8 (1 permit, 0 none), boot id u16, sequence u16, reason u8, little endian. A refused command reports cmd_result 80h |
| Emulator | CTL-017 emulator node 10h: state machine INIT, UNHOMED, HOMING, IDLE, MOVING, STOPPING, FAULT; supervision of the keep-alive and the core heartbeat (300 ms), permit freshness 200 ms; faults F01 to F09 |
| Controller state | TPDO1 190h (on change, at least every 100 ms): state, flags (position valid, at stop, moving, lower limit, upper limit, fault, motor power, recovery), position u16 0.1 mm (FFFFh unknown), active stop, last seq, cmd_result, counter. TPDO2 290h: inputs (stop sensors L1000, L1175, L1350, second E-stop input, Hall supply), target stop, motor current mA, velocity i16 0.1 mm/s, faults F01 to F08, aux (F09, light, Getac). Heartbeat 710h. EMCY 090h (`<HBBH`: code FF00h + n, error register, fault n, fault mask; none in the recorded runs) |
| To the plant | emulator > `/sil/lift/gz_cmd_vel` (Float64, m/s) > ros_gz_bridge > `/sil/gz/lift_cmd_vel` > gz-sim JointController in force mode (PI on velocity, p 3000, i 80000, limit 800 N, unverified working values) |
| Plant | Gazebo joint `lift_joint` |
| Feedback producer | Gazebo `/sil/gz/joint_states` lift_joint position > emulator position and Hall model > TPDO1 and TPDO2 > host stand-in |
| ROS feedback | `/sil/lift/status` (JSON: the stand-in's decode of TPDO1 and TPDO2) > `base_state_adapter` > `/base/joint_states` lift_joint (0 m at the L1000 stop) > `/joint_states`; emulator state `/sil/lift/emulator_state` |

### Arms (vcan_arm_l and vcan_arm_r)

| Step | Value |
|---|---|
| Command producer | MoveIt `move_group` > `/{left,right}_arm_controller/follow_joint_trajectory` (JointTrajectoryController); `gravity_feedforward.py` > `/{left,right}_arm_gravity_controller/commands` (effort); `/{left,right}_gripper_controller/gripper_cmd` |
| Driver | openarm_ros2 `openarm_hardware`, unmodified, openarm_can, CAN FD with bit rate switch, 500 Hz on Gazebo time. Default gains kp 70, 70, 70, 60, 10, 10, 10, kd 2.75, 2.5, 2.0, 2.0, 0.7, 0.6, 0.5, gripper kp 5.0, kd 0.1 (openarm_simple_hardware.hpp; the recorded frames carry these) |
| Transport, every driver cycle | 7FFh refresh request per motor (`[id, 0, CCh, 0...]`), then MIT command on 001h to 008h, then two state replies per motor on 011h to 018h. Recorded: 8 x 7FFh, 8 commands and 16 replies per cycle |
| MIT command | 8 bytes, big-endian bit fields: q u16 over plus or minus 12.5 rad, dq u12, kp u12 over 0 to 500, kd u12 over 0 to 5, tau u12. dq and tau ranges by motor: DM8009 (ids 1, 2) 45 rad/s, 54 N m; DM4340 (3, 4) 10 rad/s, 28 N m; DM4310 (5 to 8) 30 rad/s, 10 N m |
| Other frames | FF..FF FBh to FEh: clear error, enable, disable, set zero. 7FFh with 33h, 55h, AAh: register read, write, save (replies on 011h to 018h) |
| Emulator | `OpenArmCanEmulator` (openamrobot_openarm_emulator): motor model per id, MIT control law at the physics step, torque and speed limits of the motor model |
| Controller state | state reply: D0 low nibble motor id, high nibble status (0 disabled, 1 enabled, 8h to Eh faults), q, dq, tau, MOS and rotor temperature (recorded: enabled, 25 C) |
| To the plant | the emulator's effort command interfaces of the simulator's controller manager |
| Plant | Gazebo joints `openarm_{left,right}_joint1` to `joint7` and the finger joints |
| ROS feedback | driver state interfaces > joint_state_broadcaster > `/arm/joint_states` > `joint_state_publisher` > `/joint_states`; controller state topics `/{side}_arm_controller/controller_state` |

### BMS (vcan_bms, CAN2 slot)

| Step | Value |
|---|---|
| Command producer | host stand-in `BmsHost`: polls Daly data IDs 90h, 93h, 98h every 200 ms of Gazebo time |
| Transport | vcan_bms, 29-bit IDs `18h << 24 | data id << 16 | destination << 8 | source`: requests 18xx0140h, replies 18xx4001h, 8 data bytes, big endian |
| Emulator | Daly BMS emulator: constant values from sil.yaml (25.6 V, 0 A, 80 %, 84 Ah), no battery model |
| Values | 90h: pack voltage 0.1 V, acquired voltage 0.1 V, current (raw - 30000) x 0.1 A, SOC 0.1 %. 93h: charge state, charge and discharge MOS, life counter, remaining mAh. 98h: failure bytes |
| Plant | none |
| ROS feedback | `/sil/bms/status` (JSON), `/sil/bms/battery_state` (sensor_msgs/BatteryState) from the stand-in; `/sil/bms/emulator_state` from the emulator |

### Auxiliary functions on vcan_aux

CTL-017 RPDO2 310h (head light, dim level, Getac enable) and SDO 610h and 590h are emulated, but nothing in the
stack sends them; neither recorded run has them. The ToF, ultrasonic, LiDAR and station camera are Gazebo sensors
bridged to ROS, not CAN devices.

## Decoded IDs in the control room

`/control_room/can/decoded` (vcan monitor node, 1 Hz) carries the latest decoded value of every signal and the last
raw frame of each ID. The offline report plots the drive wheel targets and actuals, lift position and velocity,
arm command and feedback positions and the BMS SOC, and lists the first raw frame of every bus and ID with its decode.

| Bus | IDs decoded |
|---|---|
| vcan_drive | 601h and 581h SDO (60FFh, 606Ch, 6041h, 603Fh, 6040h, 6060h, 2000h and the other ZLAC objects, aborts), 201h, 301h, 701h, 000h, EMCY |
| vcan_aux | 210h, 310h, 190h, 290h, 410h, 090h, 610h and 590h, 710h, 701h, 000h |
| vcan_bms | 18xx0140h requests and 18xx4001h replies of 90h to 94h and 98h |
| vcan_arm_l, vcan_arm_r | 001h to 008h (MIT and special commands), 101h to 308h (other modes, raw), 011h to 018h (state and register replies), 7FFh |

## Open points

- `/sil/drive/emulator_state` is not in either recorded bag although the node creates the publisher and a 50 ms
  timer; the control room end-to-end run checks it live (`ros2 topic info -v`, one `ros2 topic echo`).
- The Baseline 1 bag of run 44309817813 has none of the host stand-in's status topics (`/sil/drive/status`,
  `/sil/lift/status`, `/sil/bms/status`) although its recorder asked for them; the grocery bag has them.
- Action status topics are hidden topics: the recorders run without `--include-hidden-topics`, so no bag has them.
- Every layout above is the emulators' and the stand-in's. The CTL-017 dictionary is a proposal pending its owner,
  the Daly protocol is unconfirmed, and the ZLAC8015D sign and axis order are SIL test values.
