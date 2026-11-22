# openamrobot_openarm_emulator

Software in the loop for the OpenArm 2.0 arms. The real upstream driver (`openarm_hardware/OpenArmHW`, not modified) talks
over Linux virtual CAN to emulated Damiao motors. The motors' motion comes from Gazebo.

Simulation only. Nothing in this package is a safety function. A result from it is development evidence. It is never
hardware evidence, physical acceptance or safety evidence. The combined model is not changed.

Design input: the digital-twin audit of 28 Sep 2026 (`openAMRobot/audits`, branch `audit/2026-09-28`,
`digital-twin/openarm/openarm_interface.md` and `findings.md`).

## What runs

```
 root namespace: what would run on the robot          /sim namespace: the plant
 ----------------------------------------------       ---------------------------------------------
 arm and gripper controllers                          gz sim, combined model, hardware_profile gazebo
   | position commands                                  ^ joint effort        | joint position, velocity
 /controller_manager (ros2_control_node)              /sim/controller_manager (gz_ros2_control)
   OpenArmHW left      OpenArmHW right                  left_arm_emulator     right_arm_emulator
        |                   |                                |                      |
        +---- vcan_arm_l ---|--------------------------------+                      |
                            +---- vcan_arm_r ---------------------------------------+
```

- One emulator per arm, each on its own SocketCAN interface: `vcan_arm_l` and `vcan_arm_r`. Each plays eight motors:
  J1 to J7 and the gripper, ids 0x01 to 0x08, replies on 0x11 to 0x18.
- Joint motion and torque come from Gazebo **through ros2_control**. Each emulator is a ros2_control controller
  (`openamrobot_openarm_emulator/OpenArmCanEmulator`) in the controller manager that `gz_ros2_control` hosts inside the
  simulator. Every physics step it reads the eight joints' position and velocity state interfaces, runs the motors'
  control laws, and writes the eight effort command interfaces. There is no joint-state topic bridge in the loop.
- The emulator computes the MIT law, not Gazebo. The impedance stays on the motor side, as on the real arm. The Gazebo
  position servo of the normal `gazebo` profile is not used for the arms here.
- `/joint_states` is what the driver believes. `/sim/joint_states` is what the physics does. Most fault cases are about
  the difference between the two.

## Build

The driver and its CAN library are not released for Jazzy. Build them from source in the same workspace, at the commits
of `config/sil_pins.yaml`:

| Repository | Commit | Note |
|---|---|---|
| enactic/openarm_ros2 | `4e837e1d0dae692ff67b560b69d8d281d7a8d4ed` | the driver. Same pin as the model's `upstream_pins.yaml` |
| enactic/openarm_can | `bfd4e52194aefe38389fa6e670be3738136facb9` | tag 1.4.1. Upstream openarm_ros2 does not pin it |
| enactic/openarm_description | `14ff67b638ff1c738a1b9a6be8aaa5ce5ed2c831` | same pin as the model |
| enactic/openarm | `e15e09e056098348d6f437389bbf48179caa82b6` | CAN setup document, read only |

```bash
cd ws/src
git clone https://github.com/enactic/openarm_ros2 upstream/openarm_ros2 && git -C upstream/openarm_ros2 checkout 4e837e1d0dae692ff67b560b69d8d281d7a8d4ed
git clone https://github.com/enactic/openarm_can upstream/openarm_can && git -C upstream/openarm_can checkout bfd4e52194aefe38389fa6e670be3738136facb9
cd .. && rosdep install --from-paths $(colcon list --packages-up-to openamrobot_openarm_emulator --paths-only) --ignore-src -y
colcon build --packages-up-to openamrobot_openarm_emulator
```

## Run

```bash
ros2 launch openamrobot_openarm_emulator sil.launch.py                 # with the Gazebo window
ros2 launch openamrobot_openarm_emulator sil.launch.py headless:=true  # server only
```

The launch creates the interfaces, starts Gazebo with the combined model, both emulators, the real driver and an example
trajectory (`scripts/example_arm_trajectory.py`: both arms out to a bent pose and back). Arguments: see the launch file's
header. `example_trajectory:=false` leaves the trajectory out.

When `ros2_control_node` activates the hardware, the driver enables the motors and drives every arm joint to zero. That
is upstream behaviour (see "What the driver does today"). In the simulation the arms hang limp until then.

### Virtual CAN and CAN FD

`scripts/setup_vcan.sh` creates the interfaces (the launch runs it; `create_vcan:=false` skips it). It needs root or
passwordless sudo, and the `vcan` kernel module (Ubuntu: `linux-modules-extra-$(uname -r)`).

```bash
ros2 run openamrobot_openarm_emulator setup_vcan.sh             # vcan_arm_l and vcan_arm_r
ros2 run openamrobot_openarm_emulator setup_vcan.sh --delete
```

It sets MTU 72 so the interface carries CAN FD frames, and prints `CAN FD yes (mtu 72)` or `CAN FD NO (mtu 16)` per
interface. With `can_fd:=auto` (default) the launch gives the driver `can_fd true` only when both interfaces have MTU 72.
Otherwise it says so and runs the driver with classic CAN frames. The emulator answers in the frame type it was asked in.

vcan has no bit rate, no sample point, no acknowledgement and no bus-off. The real bus is 1 Mbit/s arbitration and
5 Mbit/s data. The emulator models frame time in software (below). It does not model error frames or bus-off.

## Frames

Layout source: enactic/openarm_can 1.4.1, `dm_motor_control.cpp`. The codec here is written independently and
`test_codec` compares it bit for bit with the upstream encoder and decoder.

| Host frame | CAN id | Emulated motor |
|---|---|---|
| Enable `FF FF FF FF FF FF FF FC` | send id | enabled, setpoints cleared. State reply |
| Disable `.. FD` | send id | disabled, no torque. State reply |
| Set zero `.. FE` | send id | position offset. State reply |
| Clear error `.. FB` | send id | clears a latched fault. State reply |
| MIT control | send id | stored when enabled and in MIT mode. State reply |
| Position-velocity | send id + 0x100 | stored when enabled and in that mode. State reply |
| Velocity, position-force | send id + 0x200, + 0x300 | as above |
| Refresh `idLo idHi CC` | 0x7FF | State reply |
| Read parameter `idLo idHi 33 RID` | 0x7FF | Parameter reply with the value |
| Write parameter `idLo idHi 55 RID value` | 0x7FF | stored in RAM, echoed. RID 10 changes the control mode |
| Save `idLo idHi AA` | 0x7FF | RAM to flash when disabled, reply `AA 01` |

State reply, on the reply id, 8 bytes: D0 low nibble motor id and high nibble status or error code; D1 D2 position
(16 bits, -12.5 to 12.5 rad); D3 and D4 high nibble velocity (12 bits); D4 low nibble and D5 torque (12 bits); D6 MOSFET
temperature; D7 rotor temperature. Codes: 0 disabled, 1 enabled, 8 over-voltage, 9 under-voltage, A over-current,
B MOS over-temperature, C coil over-temperature, D communication lost, E overload.

Position and velocity are the joint's, from Gazebo. Torque is what the emulated motor applied. No gear ratio and no sign,
as in the driver.

Parameters with a value: UV_Value (0), OT_Value (2), MST_ID (7), ESC_ID (8), TIMEOUT (9), CTRL_MODE (10), Gr (20),
PMAX (21), VMAX (22), TMAX (23), OV_Value (29). Any other register reads 0, KT_Value (1) and OC_Value (3) included: their
values as shipped are unknown. TIMEOUT acts: 1 count is 50 us, 0 is off (below, "Current and temperature").

## Timing

vcan delivers a frame at once. A wire does not. With `timing.model: true` (default) the emulator keeps a model of one
shared bus per arm: each frame, the host's and the motors', takes its length in bits, one frame at a time, lowest id
first. An 8-byte CAN FD frame with BRS at 1 and 5 Mbit/s is about 52 us in this model (10 % stuff bits assumed). The
audit estimates about 50 us. A reply is put on vcan when its modelled transmission ends.

Effect on the unmodified driver: `read()` sends eight refresh frames and waits up to 500 us for the first reply. The
first reply ends after two frame times, the last after sixteen (about 0.83 ms). So `read()` usually takes one fresh
state and the rest arrive in `write()` or in the next cycle. This is the model's prediction. It is not measured on
hardware. `timing.turnaround_us` (motor reply latency) is 0 until it is measured. `timing.model: false` answers at once.

The driver runs on wall time. The world (`worlds/sil_empty.sdf`) runs 1 ms physics at real time and has no sensor
rendering, so a machine without a GPU can keep real time. If the simulation runs slower than real time, the emulated arm
is slower than the driver expects.

## Damping and the physics step

A numerical treatment, on by default (`motor.implicit_damping`). It is not a model of the motor.

The MIT law's damping gain is stiff for the light links: kd times the 1 ms step, divided by the link inertia about its
joint, is up to 6. An explicit step is only stable below 2. Without a treatment the wrist and upper-arm joints chatter
at torque saturation in Gazebo (seen while this package was written: plus and minus 25 rad/s every step). The real motor does not have
this problem. It closes its loop faster, on a rotor with reflected inertia. Neither is modelled, and no source in scope
gives the rotor inertia (audit gap 3).

The emulator therefore low-passes its applied torque at `motor.damping_split_hz` (200 Hz). What the law asks beyond that
slow part is divided by `1 + kd * dt / (0.5 * damping_inertia)`, which is what an implicit step of the damping term
gives. `damping_inertia` is each joint's child link about the joint axis, from the URDF (`test_package_static`
recomputes it). Torque at rest and motion well below 200 Hz are the law's own. `test_motor_model` shows the chatter
without it, the stable response with it, and unchanged static torque.

**The simulator dynamics profile is switched off in this launch.** Since 15 Nov 2026 the `gazebo` profile of the combined model carries
joint damping and friction from upstream's MuJoCo model (0.6 to 1.0 N*m*s/rad). The physics steps that damping explicitly. With the
link inertias about the joints (1.1e-4 to 2.5e-3 kg*m^2) and the 1 ms step, damping times the step over the inertia is 0.35 to 6.6, above
the limit of 2 on four of the eight joints of an arm, the same problem as the MIT gain above. Upstream's values go with a reflected rotor inertia (0.002 to
0.030 kg*m^2) that would bring the ratio down to 0.03 to 0.42, and URDF cannot carry it. Seen in CI on the first merge of the two
changes: with the damping on, the right arm, which should hold at zero, read -1.33 rad. So `sil.launch.py` passes `sim_dynamics:=off`
and this package behaves as it did before the profile existed. The position-servo simulation (`openamrobot_upperbody_gazebo`) is not
affected: there the joint is driven by a velocity constraint, and its tests pass with the damping on. Using the profile here needs
the rotor inertia in the physics first.

## Supply voltage

`supply_voltage` (default 25.6 V) is the arm supply the motors see. The robot's range is 25.6 to 29.2 V, and the
emulator accepts any value. Two effects, both assumptions:

- No-load speed scales with supply over 24 V. The torque left for driving falls linearly to zero at that speed. Braking
  is not limited. `motor.voltage_speed_model: false` turns this off.
- Above `motor.over_voltage_v` (32 V) every motor trips 0x8. Below `motor.under_voltage_v` (15 V) it trips 0x9.

## Fault injection

Each fault is a parameter of `/sim/left_arm_emulator` or `/sim/right_arm_emulator`. `inject_fault.py` sets them.

```bash
ros2 run openamrobot_openarm_emulator inject_fault.py left estop on        # E-stop power cut
ros2 run openamrobot_openarm_emulator inject_fault.py left estop off       # reset
ros2 run openamrobot_openarm_emulator inject_fault.py left can_loss on
ros2 run openamrobot_openarm_emulator inject_fault.py left silent 4 on     # J4 stops answering
ros2 run openamrobot_openarm_emulator inject_fault.py both over_voltage on
ros2 run openamrobot_openarm_emulator inject_fault.py right over_temperature 2 on
ros2 run openamrobot_openarm_emulator inject_fault.py right coil_over_temperature 2 on
ros2 run openamrobot_openarm_emulator inject_fault.py left over_current 5 on    # J5 reports 0xA
ros2 run openamrobot_openarm_emulator inject_fault.py left comm_timeout 3 on    # J3 reports 0xD
ros2 run openamrobot_openarm_emulator inject_fault.py both timeout 0.1          # TIMEOUT register: 0.1 s
ros2 run openamrobot_openarm_emulator inject_fault.py left power_cut on         # same as estop
ros2 run openamrobot_openarm_emulator inject_fault.py both supply 29.2
ros2 run openamrobot_openarm_emulator inject_fault.py both clear
sudo ip link set vcan_arm_l down                                           # the interface itself
```

| Fault | Parameter | Emulated motors | What the unmodified driver shows today |
|---|---|---|---|
| CAN loss | `fault.can_loss` | nothing reaches them, nothing comes back. They hold their last setpoint | `read()` and `write()` return OK. State frozen at the last value. Stream resumes by itself |
| Interface down | `ip link set down` | as CAN loss | every write fails with ENETDOWN inside openarm_can. The driver still returns OK |
| Single-motor timeout | `fault.silent_motors` | one motor runs and does not answer | that joint's state is frozen while the joint moves. OK returned |
| Over-voltage | `supply_voltage` above 32 V, or `fault.error_codes` 8 | code 0x8, no torque, still answering | the joints go limp. The state shows it. The code is decoded and never checked. OK returned |
| Over-temperature | `fault.temperature_mos_c`, `fault.temperature_rotor_c`, `fault.temperature_rate_c_per_s` | temperature ramps in D6 or D7, then code 0xB or 0xC, no torque | as over-voltage. Temperature is exported nowhere |
| E-stop power cut | `fault.estop_power_cut: true` | bus up, every motor silent, no torque | the arm falls in the physics. The driver's state is frozen. Hardware component and controllers stay active |
| Reset | `fault.estop_power_cut: false` | motors answer again, disabled, positions kept | state is live again and shows the fallen arm. The driver never sends enable by itself. Only deactivate and activate re-enable, and that drives the arm to zero |
| Over-current | `fault.error_codes` 10 (`over_current`), or `over_current_a` set and exceeded | code 0xA, no torque | as over-voltage |
| Communication timeout | `motor.can_timeout_s` (TIMEOUT register) with `fault.can_loss`, or `fault.error_codes` 13 (`comm_timeout`) | each enabled motor trips 0xD once no frame has arrived for the TIMEOUT time, lets go, still answers | OK returned throughout. After the wire closes the state is live and shows the fall; the code stays, nothing re-enables |
| Any fault code | `fault.error_codes` (8 to 14 per motor) | that code, no torque | as over-voltage |

A fault stays until a clear-error frame or a power cycle. The driver sends neither.

`/sim/<side>_arm_emulator/status` (diagnostic_msgs/DiagnosticArray, 10 Hz) reports the bus and each motor: powered,
enabled, code, position, torque, temperatures, frame counts, and the first MIT frame after the last enable.

## Current and temperature

Electrical and thermal states of each emulated motor. Parameters with value, unit, source, confidence, uncertainty,
hardware revision, owner, test, date read and measurement item: `config/motor_electrical.yaml`. Nothing there is measured.

**Current from torque.** The motor applies output torque `tau` (the control law, clamped as above). Its phase current is

```
I = tau / (kt * N)          kt: N m/A at the rotor, N: gear ratio
P_cu = 1.5 * R * I^2        R: datasheet phase resistance, I: q-axis current amplitude
```

| Type | Joints | N | Rated | kt (N m/A) | kt * N | R (ohm) |
|---|---|---|---|---|---|---|
| DM-J8009P-2EC | J1, J2 | 9 | 20 N m, 20 A | 0.1111 | 1.000 | 0.09 |
| DM-J4340P-2EC | J3, J4 | 40 | 9 N m, 2.5 A | 0.0900 | 3.600 | 0.76 |
| DM-J4310-2EC V1.1 | J5 to J7, gripper | 10 | 3 N m, 2.5 A | 0.1200 | 1.200 | 0.65 |

Sources: the Damiao manuals that upstream publishes (`enactic/openarm` e15e09e,
`website/static/file/hardware/specification/motor/dm4310.pdf`, `dm4340.pdf`, `dm8009.pdf`) and the same table in
`website/docs/hardware/openarm-2.0/motor.mdx`. No manual gives a torque constant, so `kt` is the rated torque over gear
ratio and rated current (confidence: estimate). The peak point gives 6 to 22 % less. It includes the gear efficiency.
The 1.5 factor assumes amplitude-invariant currents and a phase-to-neutral R; the manuals state neither.

**Where the protocol reports it.** The Damiao state frame has no current field: D3 to D5 carry torque (12 bits), D6 the
MOSFET temperature and D7 the rotor temperature, both in whole degrees C (datasheet feedback frame; openarm_can
`dm_motor_control.cpp` L100-L114). The emulator reports `tau` in D3 to D5 and both temperatures in D6 and D7, as before.
Current and copper loss are in `/sim/<side>_arm_emulator/status` (`current`, `copper_loss`) only.

**Over-current (0xA).** With `over_current_a` set (8 values, A), a phase current above it trips 0xA: no torque, still
answering, latched as any fault. Unset by default: register 3 (OC_Value) as shipped is unknown, and the manuals' "at most
9.8 A" is advice for the user setting. The DM8009 manual repeats the DM4310 text, below its own 20 A rated current.

**Winding temperature.** Off by default (`motor.thermal_model: false`). No manual gives a thermal resistance or a time
constant. With the model on and both `thermal_resistance_k_per_w` and `thermal_time_constant_s` set (8 values each), the
rotor temperature is a lumped first-order state

```
T_rotor(t + dt) = T_ss + (T_rotor(t) - T_ss) * exp(-dt / tau_th),   T_ss = T_ambient + P_cu * R_th
```

and above `motor.over_temperature_c` (register 2) the motor trips 0xC. With the model on but a value missing, configure
fails. With the model on, injected rotor temperatures are ignored; MOSFET injection still works. The MOSFET temperature
has no model: injection only, trip 0xB above `motor.mos_over_temperature_c` (120 C, datasheet: fixed).

**TIMEOUT register (RID 9).** One count is 50 us (register table of the third-party python-damiao-driver; not a Damiao
document). 0 is off. `motor.can_timeout_s` sets it in RAM and flash of every motor, while running too, as the maker's
tool would. A host write to RID 9 acts at once and is lost at a power cycle unless saved. When an enabled motor has
received no control frame on its own id for that time it trips 0xD (whether a refresh on 0x7FF also
counts is unknown; here it does not). Upstream never writes the register; its value on a
delivered motor is unknown (MP-07).

**Not modelled.** Phase resistance rise with temperature, iron and gear losses, MOSFET heating, supply (bus) current and
regeneration (RG-DRV-002, RG-DRV-005), overload (0xE) as a torque-time limit, current-loop dynamics, saturation above the
rated point.

## Driver behaviours 1 to 6

The upstream driver is not changed. The emulator produces the motor-side conditions and the tests record what the driver
does with them (audit register rows RG-FLT-001 to 006, measurement item MP-08).

| # | Behaviour | Motor-side condition the emulator produces | Test |
|---|---|---|---|
| 1 | Move to zero on activate | enable answered; the first MIT frame and the position it found are recorded | `test_real_driver` KnownIssue_ActivateMovesEveryJointToZero, `test_sil_launch` |
| 2 | Gripper sent 0.044 rad on activate | motor 8 records its first MIT frame | KnownIssue_ActivateMovesEveryJointToZero |
| 3 | Healthy report on CAN loss and on a power cut | `fault.can_loss`, `ip link set down`, `fault.estop_power_cut` (motors silent), TIMEOUT trip | Fault_CanLoss_*, Fault_BusDown_*, Fault_EstopPowerCut_*, Fault_CommunicationTimeout_* |
| 4 | Fault codes and temperatures decoded, not acted on | codes 8 to E in D0, temperatures in D6 and D7, from thresholds, the thermal model or injection | Fault_OverVoltage_*, Fault_OverTemperature_*, Fault_CommunicationTimeout_* |
| 5 | No re-enable after a power cut | after the supply returns every motor answers disabled (code 0) and ignores MIT frames until an enable | Fault_EstopPowerCut_*, Fault_CommunicationTimeout_*; `test_motor_model` PowerCutSequenceOnTheWire |
| 6 | Gripper velocity and effort states 0 | motor 8 reports its real velocity and torque in the state frame | EnablesReadsAndCommandsOneJoint |

Register rows this package addresses or touches: RG-FLT-001 to 011, RG-DRV-001 to 007 (RG-DRV-002 current, RG-DRV-006
temperature), RG-MEC-034 to 041 (not changed here). Measurement items: MP-06 (kt from register 1 and a held-torque
current log, thermal resistance and time constant from a rated-torque hold), MP-07 (registers 0, 2, 3, 9, 29 as shipped),
MP-08 (fault and power behaviour of the delivered arm).

## What the driver does today

Recorded by the tests against upstream `OpenArmHW` at the pin. These are records, not approvals. This package does not
fix the driver.

1. **Move to zero on activate.** After enable, the first MIT frame to each arm joint is position 0 at full gains, sent
   wherever the joint is. Then a 2 s ramp to zero. Nothing plans the motion or checks it for collision. Activation
   blocks for it (about 2.2 s per arm).
2. **Gripper command on activate.** The gripper is sent 0.044 rad: the 1.0 gripper's open width in metres, unconverted.
3. **Healthy report on CAN loss.** `read()` and `write()` always return OK. A silent bus, a downed interface and a
   power cut all leave the last state in place with no sign that it is stale.
4. **Motor fault codes are not acted on.** The code is decoded and never read. Temperature is not exported.
5. **No re-enable after a power cut.** Enable is sent only in `on_activate`.
6. **Gripper velocity and effort states are 0**, hard-coded.

## Assumptions

No source in scope fixes these. Each is a parameter with a default. Measure on the bench, then set the value.

| Parameter | Default | Open question |
|---|---|---|
| `motor.answers_when_disabled` | true | does a disabled motor answer refresh? |
| `motor.power_up_enabled` | false | motor state after supply returns |
| `motor.enable_clears_fault` | false | does enable clear a fault, or only clear-error? |
| `motor.can_timeout_s` | 0 (off) | the TIMEOUT register as shipped. Unit 50 us from a third-party register table |
| `motor.over_voltage_v`, `motor.under_voltage_v` | 32, 15 | thresholds (registers 29 and 0) |
| `motor.over_temperature_c` | 120 | rotor threshold (register 2). The datasheets advise at most 100 |
| `over_current_a` | unset (off) | threshold (register 3) |
| `motor.thermal_model` | false | thermal resistance and time constant of the winding |
| `kt` (code default) | rated torque / (gear ratio * rated current) | the torque constant (register 1) |
| `motor.voltage_speed_model` | true | speed and torque at 25.6 to 29.2 V |
| `timing.turnaround_us` | 0 | motor reply latency and jitter |
| `reply_brs` | true | BRS flag on motor replies |
| position-velocity, velocity and position-force laws | simple cascade | the firmware's own loops and gains |
| joint friction, damping, rotor inertia | none: the launch expands the model with `sim_dynamics:=off` | audit gap 3. The simulator dynamics profile (`openamrobot_upperbody_description/config/sim_dynamics/openarm_v2.yaml`) has sourced damping and friction, but they cannot be used here yet, see below |

Also not emulated: error frames, bus-off, dropped or late frames, wrong ids, overload (0xE) detection, a MOSFET thermal
model, the gripper's motor sign per hand. Over-current detection and the winding thermal model exist and are off until
their values are known (below).

## Tests

```bash
sudo modprobe vcan && ros2 run openamrobot_openarm_emulator setup_vcan.sh
colcon test --packages-select openamrobot_openarm_emulator && colcon test-result --verbose
```

| Test | Needs | What it checks |
|---|---|---|
| `test_codec` | openarm_can | frame bytes, then encode and decode against the upstream library |
| `test_motor_model` | nothing | motor state machine, parameters, faults, every code in the frame, current, thermal model, TIMEOUT, power cut on the wire, supply voltage, damping, bus timing |
| `test_real_driver` | vcan, driver | the real driver enables, reads and commands a joint through the emulator. Each fault. Both arms |
| `test_package_static` | nothing | pins, configuration, `motor_electrical.yaml` fields and defaults, `inject_fault.py` usage, launch logic, `setup_vcan.sh` |
| `test_sil_launch` | vcan, driver, Gazebo | the whole launch headless: trajectory, E-stop power cut, reset |

A test that needs vcan or the driver and cannot have it reports **BLOCKED** (a skip with that word). It never passes.
`scripts/count_sil_tests.py` counts passed, failed, BLOCKED and skipped apart. The CI job
(`.github/workflows/openarm-emulator-tests.yml`) fails when a test is BLOCKED.
