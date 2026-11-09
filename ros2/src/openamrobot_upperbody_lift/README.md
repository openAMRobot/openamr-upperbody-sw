# openamrobot_upperbody_lift

Real `lift_joint` state on the host (issue #45). **Night-shift proposal, not adopted.**

```
CTL-017 lift node --CAN3--> base controller (STM32) --/base/aux_status--> lift_relay --/joint_states--> robot_state_publisher, move_group
                                                                                                        ^
                                                                          lift_guard (refuses plan / execute on a stale lift) ---+
```

The Jetson never talks to the lift node: no SocketCAN to node 0x10. The base controller relays the lift status on `/base/aux_status`
(5 Hz, stale after 0.6 s) and owns the lift interlock (I8 rev 3, decision D17). Nothing here is a safety function; the hardwired E-stop chain is.

## lift_relay

Subscribes `/base/aux_status` (reliable, transient-local, depth 1). Publishes `lift_joint` on `/joint_states`, in metres from the L1000 stop
(0 to 0.350, as the URDF), once per message, **only if all of this holds**:

| check | rule |
|---|---|
| link fresh | a progressing message arrived within 0.6 s of local receipt time (header stamp is diagnostic only) |
| progressing | `(boot_id, sequence)` moves forward; a repeat or a step back refreshes nothing; a transient-local replay cannot keep the state alive |
| resynchronised | 3 consecutive good frames after start-up, a restart (`boot_id` change), a sequence jump over 3, or any stale gap (decision D8) |
| measurement fresh | `lift_status_age_ms` at most 300 ms (a new message can carry an old lift measurement) |
| position valid | `lift_position_valid` true, `lift_recovery_required` false, position finite and inside 0 to 0.350 m by at most 2 mm (clamped to the URDF limit within that) |
| contract | `contract_version` is 1 |

Anything else: **no `lift_joint` is published** (never a held or default value) and `/lift_relay/status` plus `/diagnostics` carry the reason.

## lift_guard

Stopping the relay is not enough: MoveIt and TF keep the last value. `lift_guard` serves `/move_action`, `/execute_trajectory` and
`/plan_kinematic_path` and forwards to move_group (remapped to `*_unguarded`) only while a `lift_joint` sample younger than 0.6 s is on
`/joint_states`. Otherwise the goal is aborted at once with `INVALID_ROBOT_STATE` and the reason in `error_code.message` (source `lift_guard`);
a running goal is cancelled when the lift goes stale.

**Wiring (not done here).** There is no real-profile move_group launch yet (`move_group.launch.py` accepts `mock | gazebo` only, findings M-01), and
this change leaves the mock and Gazebo launches untouched. The guard is wired in by the future real launch: start move_group with
`remappings=GUARD_REMAPPINGS` (`openamrobot_upperbody_lift.guard_logic`: `move_action`, `execute_trajectory` and `plan_kinematic_path` renamed
to `*_unguarded`) and `lift_state.launch.py`. Until then `lift_guard` has nothing behind it. A client that talks to move_group's
topics or the controllers directly is not covered; the base controller's interlock is the authority.

## Lift commands

A documented stub (`command_stub.py`): every call raises. Pending the I8 lift contract (Evans). No motion code.

## Stand-in message

`openamrobot_upperbody_lift_msgs/BaseAuxStatusStandIn` carries the lift block with the I8 / CAN3 section 8.3 field names, because
`openamr_base_msgs` is not published yet (interfaces #30). One adapter, `relay_node.observation_from_msg`, knows the layout. TODO(Evans).

## Open parameters (TODO(Evans))

`position_offset_m` (calibration offset from the L1000 stop, 0.0), `max_lift_status_age_ms` (300, decision D15), `range_tolerance_m`
(2 mm, decision D3), `max_sequence_gap` (3), the accepted `contract_version` (1), the final field names.
