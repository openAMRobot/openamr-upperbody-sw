# Build 1 scene: what is modelled, assumed and not modelled

Package: [`ros2/src/openamrobot_build1_scene`](../ros2/src/openamrobot_build1_scene/README.md). Launch:
`build1.launch.py`. This page says what the scene stands for and what it does not.

Simulation only. Simulation, emulators and fake hardware are development evidence. Nothing here is hardware evidence,
physical acceptance, release readiness or safety evidence. No emulator and no node of this scene holds a safety
function. Functional telemetry, watchdogs, status displays and fixtures are never safety evidence.

## Inputs

Recorded with full hashes in `config/pins.yaml` of the package.

| Input | Pin | Use |
|---|---|---|
| openamr-upperbody-sw main | `b15bcb6` | branch point: combined model, A6, A7, arm emulator (#67) |
| openamr-upperbody-sw #66, open | `6fe5e9c` | merged into the branch by an explicit merge commit |
| openamr-upperbody-sw #65, open | `e3b5cb8` | read only |
| openamr-platform-sw main | `0741778` | description with wheel friction (#135), Nav2, perception, docking model |
| openamr-platform-sw #141, open | `8bc5d1a` | the package `openamrobot_sil` only, and its `sil-topology.md`. Contains #139 `4a44f7d` |
| openamr-platform-sw #105, open | `acc427d` | the form of the base RGB-D sensor block. Not built |
| audits `audit/2026-09-28` | `be17653` | Use Case 1 baseline, M-01, E-01, phase 1 baseline |

The reality-gap register exists in audits at `be17653`: `digital-twin/phase1/reality_gap_register.md` and `.csv`. It
is an internal document and is not copied here. The gaps this scene touches are named in the last section by subject.

## Modelled

### World

Generated from `config/scene.yaml`. Every number there has a status: sourced, derived or estimated.

| Thing | Value | Status |
|---|---|---|
| floor | level, 14 m by 10 m | level floor: sourced, Use Case 1 baseline item UC1-09. Size: estimated |
| floor friction | 0.8 | estimated. No measured coefficient exists |
| shelf levels | 0.82 m and 1.09 m | sourced: site heights of the Use Case 1 baseline. Inside its arm work span of 0.65 to 1.40 m |
| aisle | 1.50 m wide, 3.60 m long, three shelf units on each side | estimated. The baseline states no aisle |
| shelf unit | 1.20 m long, 0.40 m deep | estimated |
| work shelf | one unit in the open area, robot parked 0.30 m in front | parking gap: sourced, Use Case 1 baseline. Position: estimated |
| dock | the `apriltag_dock` model of openamr-platform-sw at a fixed pose, three 36h11 tags, 0.16 m | the pose is fixed for repeatability. Tag size: the simulation value of the platform |
| tote | 0.40 m by 0.30 m, 0.06 m wall, 0.6 kg | estimated |
| item | 35 mm by 35 mm by 140 mm, 0.15 kg | estimated |
| payload | 0.20 m by 0.15 m by 0.10 m, 1.0 kg | mass: sourced, Use Case 1 baseline item UC1-19. Size: estimated |
| obstacles | two boxes and one post | estimated |

The Use Case 1 baseline states work heights, the robot envelope and a parking distance. It states no aisle width, no
shelf depth, no tote size and no floor friction. The scene does not claim to be the Use Case 1 site.

Physics profiles: `fine`, 1/1500 s, the default, and `fast`, 1 ms. Both ask for a real-time factor of 1.0.

### Robot

The combined model `combined_robot.xacro`, unchanged, wrapped by `urdf/build1_robot.urdf.xacro`. The wrapper adds
Gazebo-only elements. No link, joint, limit, mass or inertia is added or changed. A test compares the kinematic part
of the model with and without the wrapper's edits.

| Part | State |
|---|---|
| base, two drive wheels | from openamr-platform-sw. Track 0.400 m is a placeholder there |
| castors | **fixed joints in the description**. Frictionless sliders. No swivel joint exists to use |
| lift, 350 mm | prismatic `lift_joint`, three calibrated stops |
| two arms, 7 joints each | upstream OpenArm 2.0 model, with the joint damping and friction of #66 |
| grippers | `finger_joint1` commanded, `finger_joint2` a mimic joint |
| LiDAR, IMU, two ToF, four ultrasonic, dock camera | sensors of the platform description |
| base camera, head camera, two wrist cameras | sensors added by the wrapper on frames the description has. All with optical frames |

### Buses and devices, profiles `firmware_sil` and `faults`

| Bus | Kind | Device emulator | Other side |
|---|---|---|---|
| `vcan_drive` | classic CAN | ZLAC8015D, `openamrobot_sil` | host test client, or with `stm32:=renode_readonly` the firmware image in Renode too |
| `vcan_bms` | classic CAN | Daly BMS, `openamrobot_sil` | host test client |
| `vcan_aux` | classic CAN | CTL-017 lift controller, `openamrobot_sil` | host test client |
| `vcan_arm_l`, `vcan_arm_r` | CAN FD, MTU 72 | eight motors each, `openamrobot_openarm_emulator` | the unmodified upstream driver `openarm_hardware/OpenArmHW` |

Gazebo owns the rigid-body dynamics of every joint. The arm emulators emulate the CAN motor devices only.

### Software on top

- Nav2 with the launch files and parameters of openamr-platform-sw, a map generated from the scene, and a footprint
  that grows with the floor projection of the arms. The lift moves the arms straight up, so it changes no footprint.
- MoveIt 2 for both arms and grippers, with the scene objects as collision objects.

## Assumed

| Subject | Assumption | Where it shows |
|---|---|---|
| ZLAC8015D | the emulator follows Manual 1.03 and CANopen Routine 1.08 as filed in platform-hw. The V4.2 package is still missing | drive, bus loss on `vcan_drive` |
| ZLAC8015D link loss | object 2000h, 250 ms, a proposal. The emulator ramps to zero and raises no fault | bus loss on `vcan_drive` |
| Daly BMS | the emulator follows a 2019 protocol copy. Not confirmed for the delivered unit | BMS values, bus loss on `vcan_bms`, replay test |
| CTL-017 | the emulator models a proposal. Permit COB-ID, period, timeout and refusal code are SIL test values | lift, bus loss on `vcan_aux` |
| OpenArm motors | assumed power-up state (disabled), assumed thresholds, assumed latency | every arm result, power cut |
| wheel axis order and sign | SIL test values | drive |
| contact | floor friction estimated. Wheel friction a simulation estimate of the platform. Item friction estimated | drive, turn, grasp |
| cameras | field of view, resolution, rate and range are working values. The head camera is one RGB-D sensor for a stereo unit | camera topics |
| finger follower | a position servo with working gains stands in for the mimic constraint on DART | grasp |
| clocks | Gazebo time for ROS nodes, host time inside emulators. Not synchronised. A slow Gazebo meets host-clock timers of the devices | every device scenario |
| driver clock | the real driver's controller manager runs on Gazebo time here, so its rate scales with the plant. `openamrobot_openarm_emulator` alone runs it on wall time | arm scenarios |

## Not modelled

- The hardwired safety chain: E-stops, contactors, brakes, charge inhibit. Nothing stops this robot by a safety path.
- Hub motor holding brakes. A stopped or unpowered base holds only by the velocity input of its wheels.
- Swivel castors. Suspension does not exist on the robot either.
- CAN physical layer: bit rate, arbitration, error frames, bus-off, termination, cable. Virtual CAN delivers at once.
- Battery behaviour. The BMS reports static values.
- Motor current, heat, torque ripple, gear backlash, cable drag.
- Sensor noise, lens distortion, rolling shutter, lighting. The AprilTag detection chain is not started.
- Docking. The dock is a target in the world. No docking node runs, because it would be a second writer of
  `/cmd_vel`. The 2.0 dock is positioning only.
- The host link between the STM32 and the computer. The host test client talks ROS directly.
- The 2.0 real bringup. None exists to compare the frame tree or the topics with.
- Localisation quality. AMCL starts at the spawn pose on a map made from the same scene.

## Engines

Measured on 15 Nov 2026 with the pinned models, profile `fast`, 6 s at 0.2 m/s and then 6 s at 0.5 rad/s.

| Engine | Straight, ground truth for odometry | Turn, ground truth for odometry | Grippers in `fast` and `fine` |
|---|---|---|---|
| DART | 1.200 m for 1.200 m | 168.3 degrees for 171.9 | do not move |
| Bullet Featherstone | 0.866 m for 1.200 m | 0.0 degrees for 171.7 | move |

Three experiments on the Bullet result changed nothing: isotropic wheel friction, a base body without friction, and
the castor spheres kept as separate bodies. Without the castor collisions the straight line improved to 1.063 m and
the turn stayed at 0.0 degrees. The cause is open. DART is the default. With DART `finger_joint2` is owned by
`finger_mimic_follower`.

## Measured on 15 Nov 2026

First runs. Simulation evidence only. The run identity of each number is in the scenario log of the CI run.

| What | Profile | Where | Result |
|---|---|---|---|
| drive 2 m through Nav2 | `firmware_sil` | CI runner | 2.015 m ground truth for 2.015 m odometry |
| turn 90 degrees through Nav2 | `firmware_sil` | CI runner | 81.8 degrees ground truth for 91.1 degrees odometry |
| drive and turn | `fast` | CI runner | 2.052 m for 2.052 m, 87.9 degrees for 97.4 degrees |
| every sensor topic stops | `faults` | CI runner | the base stops after 1.01 s and 0.20 m, and drives on when the sensors return |
| STM32 role silent on CAN1 | `faults` | CI runner | the drive reports link loss, the base stops after 0.13 s and 0.03 m, and drives on when the link returns |
| BMS silent on CAN2 | `faults` | CI runner | poll timeouts rise, the last voltage stays on the topic, the base still drives |
| arm power cut while holding | `faults` | CI runner | the arm falls, the driver's state does not change, the controllers stay active, nothing re-enables the motors |
| single-writer check | `firmware_sil`, `fast`, `fine` | CI runner | 0 problems, 27 topics, 5 actions |
| grasp of the tote item | `fast`, Bullet Featherstone | development machine | fingers stop on the item at 0.43 rad, item rises 62 mm with the hand and holds |
| cancel and resume | `fast` | development machine | base stops 0.47 s and 0.06 m after the cancel, arm holds, both resume |

The turn on the spot over-reads by about 9 degrees in every profile: the wheels slip at the turn rate of the Nav2 spin
behaviour with the wheel friction of the platform description. At 0.5 rad/s the over-read was 3.6 degrees.

Real-time factor and rates on the CI runner, 4 cores, no graphics card:

| Profile | Step | Real-time factor | Wall time per step | Controller rates measured, in Gazebo time |
|---|---|---|---|---|
| `fast` | 1 ms | 0.36 | 2.8 ms | 100.0 Hz of 100 Hz, 0 missed |
| `fine` | 1/1500 s | 0.25 | 2.7 ms | 749.8 Hz of 750 Hz, 2.1 % missed |
| `firmware_sil` | 1 ms | 0.27 | 3.8 ms | real driver 487 Hz of 500 Hz, 6.8 % missed. Plant joint states 616 Hz of 1000 Hz |
| `firmware_sil` | 1/1500 s | 0.18 | 3.7 ms | real driver 730 Hz of 750 Hz, 7.3 % missed. Plant joint states 750 Hz of 1500 Hz |

The plant joint states are a lower bound of the emulator rate: the broadcaster drops a message when its publisher is
busy. At the 1/1500 s step the simulator's controller manager runs every second step. No Renode ran, so there is no
Renode ratio.

## Time and slow machines

Rule: `docs/architecture/sil-topology.md` section 6 of openamr-platform-sw #141. The clocks are not synchronised.

- Every limit of a scenario is in Gazebo time. Wall limits are twenty times longer.
- The Gazebo real-time factor is reported in every log. A Renode ratio is reported apart, by `openamrobot_sil`.
- Timers inside the emulators run on the host clock. On a slow machine the plant is slow against them. Known case:
  the CTL-017 model faults below a real-time factor of about 0.5 while the lift moves. A scenario that meets it reports
  BLOCKED with the measured factor. BLOCKED is not PASS.
- Seen on the CI runner: the device stack ran at a real-time factor of 0.26 to 0.50, and every lift move was BLOCKED.
  The lift path over CAN3 has therefore no passing run yet. The refusal without permit was checked and passed.
- Seen on the CI runner: the drive thread of the host test client of `openamrobot_sil` ended on an unexpected SDO
  reply in four of nine runs. A late reply on a slow machine produces it. The rig reports such a run as BLOCKED.
- The link-loss time of the drive, 250 ms of host time, is shorter in Gazebo time on a slow machine. The stop times
  in the bus loss log are therefore not stopping-time measurements.

## Faults

The fault scenarios record what the stack does today. `config/expected_reactions.yaml` holds the simulation-only
reference next to each: what the emulated device or the functional software is documented to do. These are expected
behaviours. They are not certified safety behaviour and not safety requirements. This package implements none of them.

## Reality gaps this scene touches

By subject, as named in the reality-gap register in audits. No gap is closed by this scene.

- holding brake of the hub motors: not modelled
- drive reaction to loss of communication: emulated from a proposal
- wheel velocity unit and range: from the filed manual revision, not from the V4.2 package
- arm driver behaviours recorded with the emulator: move to zero on activation, healthy state on CAN loss and on a
  power cut, no re-enable after a power cut
- castors fixed in the model
- drive track placeholder
- floor and wheel friction not measured
- clocks not shared inside `firmware_sil`
