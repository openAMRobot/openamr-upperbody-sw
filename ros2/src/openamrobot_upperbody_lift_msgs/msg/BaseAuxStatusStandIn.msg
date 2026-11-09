# SPDX-License-Identifier: MIT
#
# LOCAL STAND-IN. Not the I8 contract.
#
# The real type is openamr_base_msgs/BaseAuxStatus on /base/aux_status (I8 rev 3, decision D17), which openamrobot-interfaces has not
# published yet (interfaces #30). This message carries only the lift part of the aux-device block, with the field names of the CAN3 lift
# proposal section 8.3 (lift_position_m, lift_position_valid, lift_status_age_ms, lift_active_stop_id), so that the host relay can be built
# and tested now. When openamr_base_msgs lands, change the one adapter (observation_from_msg in openamrobot_upperbody_lift/relay_node.py)
# and delete this package.
# TODO(Evans): final field names, types, enum values and the contract_version number; whether the lift block is nested or flat.
#
# Topic and QoS of the real message (I8 rev 3, frozen 6 Nov): /base/aux_status, reliable, transient-local depth 1, 5 Hz heartbeat,
# stale after 0.6 s measured from LOCAL receipt time. The header stamp is diagnostic only.

std_msgs/Header header

# Contract identity (I8 rev 3: uint16 starting at 1, boot identity and a uint32 sequence in every message).
uint16 contract_version
uint32 boot_id
uint32 sequence

# Lift block.
# lift_state: 0 INIT, 1 UNHOMED, 2 HOMING, 3 IDLE, 4 MOVING, 5 STOPPING, 6 FAULT (CAN3 proposal 5.3).
uint8 lift_state
# Metres on lift_joint, 0.0 at the L1000 stop up to 0.350 (P-03 item 17, interfaces #31 Q5). NaN when unknown.
# TODO(Evans): confirm the zero reference and the offset between the lower end and the L1000 stop (the relay has a position_offset_m parameter).
float32 lift_position_m
# Node-asserted. True only if the position can be trusted (CAN3 proposal 9.1, flag bit 0).
bool lift_position_valid
# Time since the base controller last received a valid LiftStatus frame from the lift node. Communication freshness only.
# 0xFFFFFFFF means no frame received since boot.
uint32 lift_status_age_ms
# 0 L1000, 1 L1175, 2 L1350, 0xFF none (not at a confirmed stop).
uint8 lift_active_stop_id
bool lift_moving
bool lift_fault_present
bool lift_recovery_required
