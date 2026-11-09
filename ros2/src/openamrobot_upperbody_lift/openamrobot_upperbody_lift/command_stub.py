# SPDX-License-Identifier: MIT
"""Lift command path: a documented interface STUB. No motion code.

Pending the I8 lift contract (Evans, due 11 Nov, openamrobot-interfaces #30). Until then the host has no way to command the lift on the
real profile, on purpose. When the contract exists, the command path is:

    MoveIt / teleop -> this interface -> I8 command on the base controller -> CTL-017 over CAN3 (the Jetson never opens CAN to node 0x10)

and the interlock (base stopped, arms in a safe pose, no motion on stale state) stays in firmware (I8 rev 3, interfaces #31 Q10, Q16). The CAN3
proposal's command set, for the later implementer, is LiftCommand {seq, op (STOP, MOVE_TO_STOP, CLEAR_FAULT, START_HOMING), stop_id (0 L1000,
1 L1175, 2 L1350)} with a keep-alive every 100 ms while a move is active; that is the CAN side and is NOT the I8 host interface. Whether the host
command is an action, a service or a topic is open (TODO(Evans)).
"""

PENDING = "lift commands wait for the I8 lift contract (TODO(Evans), 11 Nov); the host does not command the lift"
STOP_IDS = {"L1000": 0, "L1175": 1, "L1350": 2}     # CAN3 proposal D3; stop_id values, TODO(Evans) confirm in the I8 contract


class LiftCommandUnavailable(NotImplementedError):
    pass


class LiftCommandInterface:
    """What a later implementation has to provide. Every method refuses."""

    def stop(self):
        raise LiftCommandUnavailable(PENDING)

    def move_to_stop(self, stop):
        raise LiftCommandUnavailable(PENDING)

    def clear_fault(self):
        raise LiftCommandUnavailable(PENDING)

    def start_homing(self):
        raise LiftCommandUnavailable(PENDING)
