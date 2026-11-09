# OpenArm 2.0: first power-up checklist

First power of an OpenArm on the robot. One arm, supported, lift at L1000, hand on the E-stop. Nothing here replaces the
hardwired E-stop (Category 0, PNOZ s3): software never stands in for it, and no bus carries a safety function.

Status: draft for the manipulation owner (Om) and Adnan (I1). Values marked `TODO(Om)` are not known yet and are
parameters, not decisions. Run `scripts/openarm_can_setup.sh --help` for the CAN commands used below.

## What the safe hardware interface does, and what it does not

The real profile can use `openamrobot_upperbody_hardware/OpenArmSafeHW` instead of upstream's `openarm_hardware/OpenArmHW`:

```
xacro combined_robot.xacro hardware_profile:=real real_arm_hardware:=safe arm_read_timeout_ms:=50
```

The default stays `real_arm_hardware:=upstream`. Use `safe` for every first power-up.

| | upstream `OpenArmHW` (4e837e1) | `OpenArmSafeHW` |
|---|---|---|
| Activation | enables the motors, then moves both arms to joint zero | refuses unless every motor answered; enables; measures again; commands the measured position (command = state) |
| `read()` | always OK | ERROR when a motor is silent longer than `read_timeout_ms`, never answered, the CAN link is down, or the bus went bus-off |
| `write()` | always OK | ERROR when a frame cannot be sent or a command is not finite |
| Gripper velocity and effort state | hard-coded 0 | NaN (not available) |
| After the first fault | n/a | stays in ERROR until the hardware is activated again |

What it does not do:

- **Deactivation or an ERROR disables the motors, and a disabled arm is back-drivable.** It falls. The same happens at an E-stop.
  This is why the arm is supported for the whole checklist.
- It does not limit how far a command is from the current position. A bad trajectory still moves the arm. The first
  moves are small, slow and by hand.
- It does not detect a motor that answers but is wrong. It detects silence.
- `read_timeout_ms` (default 50) is a software choice. `TODO(Om)` tune it on the bench with the real loop rate.
- The gripper position mapping is upstream's, and upstream marks it approximate. Treat the gripper last.

## Before power

1. **Arm supported.** Clamped to a stable surface or mounted and held by a person or a stand under the elbow. The arm drops
   when power is cut. Safety goggles on, E-stop within reach, the second person has their hand on it.
2. **One arm only.** Start with one (left or right). The other arm stays unpowered and its CAN cable unplugged.
3. **Lift at L1000** (calibration stop, the lowest shoulder height) and not moving. The lift is not under software control
   in the real profile. Check by hand that the arm has clearance at L1000 and at joint zero: the geometric clearance was
   checked at L1000 only, not at other heights and not against cables.
4. **Cables.** Nothing can be pinched or pulled by a joint moving through its full range.
5. **Power limits.** Bench supply or the robot's arm rail with a current limit set. `TODO(Om)`: the limit in amperes and the
   voltage from the motor datasheets and Enactic's setup guide. Not known here, so not written down. Start with the limit at
   the low end that still lets the motors enable, and raise it in steps.
6. **Motors.** IDs and bitrate are already set to CAN FD 1 Mbit/s nominal and 5 Mbit/s data (the openarm_can default). If the
   motors shipped in CAN 2.0 mode, change the baud rate first with Enactic's tool, with a Windows laptop and the Damiao USB
   CAN debugger at the bench.
7. **Software at the pins.** `config/upstream_pins.yaml`: openarm_description, openarm_ros2 and openarm_can are pinned. Build
   openarm_can from source at that commit, then the workspace, so that `openamrobot_upperbody_hardware` builds its plugin.
   With no openarm_can only the core and its tests are built, and a `WARNING` says so.

## CAN bus

With the arm power **off**:

1. Measure CANH to CANL: about 60 ohm means two terminations. About 120 ohm means one is missing, about 40 ohm means three.
   Confirm exactly two terminating resistors on the bus (carrier switch or adapter jumper, and the motor end).
2. Common ground between the computer and the arm supply. An idle bus level far from 2.5 V points to a bad ground.

Bring the interface up (as root):

```
sudo scripts/openarm_can_setup.sh --arm left --left can1      # or --arm right --right can0
scripts/openarm_can_setup.sh --check --arm left --left can1
```

The names are the xacro defaults (left `can1`, right `can0`). USB adapters enumerate in plug order, so check with
`ip -details link show` which interface is which arm before you connect motors. The script leaves `restart-ms 0`: a bus-off
stays visible. Switch to automatic restart only after the bus has been clean for a while, and then know that a restart hides a
fault from anything that polls state. `txqueuelen` defaults to 64 (`--txqueuelen`); `TODO(Om)` confirm it on the bench.

Watch the bus in a second terminal while you work:

```
candump -x can1            # error frames show up as ERRORFRAME
canbusload can1@1000000    # TODO(Om): expected load at the real loop rate
ip -details -statistics link show can1
```

## Power-up sequence

1. Power the arm. Nothing moves. The motors are disabled and back-drivable, so the arm is still held by hand or stand.
2. Start the stack with `real_arm_hardware:=safe`, one arm in the controller set.
3. Spawn the broadcaster first, then the arm controller.
   - Activation must log `activated, holding the measured position`. If it logs `activation refused`, read the reason:
     a motor that never answered, a link that is down, or enable frames that could not be sent. Fix that, do not retry blindly.
   - The arm must **not move** at activation. If it moves toward zero, you are on the upstream plugin: stop and press the E-stop.
4. `ros2 topic echo /joint_states --once`: seven arm joints plus the gripper. Gripper velocity and effort read `nan`. The
   positions match the arm's real pose. If they differ, stop.
5. Hold test. Leave the arm held for a minute. Watch for joint temperature and for drift. A small steady-state sag under
   gravity is expected with low gains. Growing drift or a joint that heats fast: stop.
6. Fault test (arm still supported, E-stop in reach):
   - Unplug the CAN cable of the arm. Within `read_timeout_ms` the controller manager must report the read error and the
     hardware must go to ERROR. Do not rely on it to hold the arm: it disables the motors, and the arm will drop onto your hands.
   - Plug it back and bring the interface up again. The hardware does not recover by itself. Activate it again.
7. First motion: one joint, a few degrees, slow, by `ros2 action send_goal` to the arm controller. Then each joint in turn.
   The gripper last.

## Stop at once if

- Any unexpected motion, noise, or a joint that does not follow.
- `ERRORFRAME` lines in `candump`, `bus-off` in `ip -details link show`, or a growing `error` count.
- A motor that stops answering, or a joint temperature that climbs.
- Anything you cannot explain. Press the E-stop first and read the log afterwards.

## Open parameters

| Parameter | Where | Status |
|---|---|---|
| Arm current limit and supply voltage | power setup | `TODO(Om)` from the motor datasheets and Enactic's guide |
| `read_timeout_ms` (default 50) | `arm_read_timeout_ms` xacro arg | `TODO(Om)` tune on the bench |
| `txqueuelen` (default 64) | `openarm_can_setup.sh --txqueuelen` | `TODO(Om)` confirm on the bench |
| Expected bus load | `canbusload` | `TODO(Om)` measure at the real loop rate |
| Persistent CAN names per arm | udev rule | not done: needs the adapter or carrier on the unit |
