# SPDX-License-Identifier: MIT
"""lift_relay: /base/aux_status -> lift_joint on /joint_states, only while the lift state is fresh and valid.

Architecture (I8 rev 3, decision D17, issue #45): the Jetson never talks to the lift node (CTL-017, CAN3 node 0x10) itself. The base controller
relays the lift status on /base/aux_status and owns the lift interlock. This node opens no SocketCAN and sends no lift command.

Publishes
  /joint_states            sensor_msgs/JointState with ONE joint, lift_joint, in metres from the L1000 stop (0 to 0.350), once per FRESH, VALID
                           message. Nothing otherwise: never a held, default or extrapolated value.
  /lift_relay/status       diagnostic_msgs/DiagnosticStatus: OK, or the reason nothing is published (WARN while progression is re-established,
                           ERROR for stale, invalid, missing). Also sent on /diagnostics as a DiagnosticArray.
"""
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

from openamrobot_upperbody_lift import evaluator as ev

INPUT_TOPIC = "/base/aux_status"
OUTPUT_TOPIC = "/joint_states"
LIFT_JOINT = "lift_joint"
STATUS_NAME = "lift_relay"
HARDWARE_ID = "base_controller:aux_status"

# /base/aux_status: reliable, transient-local, depth 1 (I8 rev 3, frozen 6 Nov)
AUX_STATUS_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def observation_from_msg(msg):
    """The ONE place that knows the message layout. TODO(Evans): rewrite for openamr_base_msgs/BaseAuxStatus when it is published
    (interfaces #30), then delete openamrobot_upperbody_lift_msgs."""
    return ev.Observation(contract_version=int(msg.contract_version), boot_id=int(msg.boot_id), sequence=int(msg.sequence),
                          position_m=float(msg.lift_position_m), position_valid=bool(msg.lift_position_valid),
                          status_age_ms=int(msg.lift_status_age_ms), recovery_required=bool(msg.lift_recovery_required),
                          fault_present=bool(msg.lift_fault_present), lift_state=int(msg.lift_state))


def diagnostic_for(decision, now_s, last_message_age_s):
    """The DiagnosticStatus for a decision: pure, no node."""
    if decision.publish:
        level = DiagnosticStatus.OK
    elif decision.reason is ev.Reason.RESYNC:
        level = DiagnosticStatus.WARN
    else:
        level = DiagnosticStatus.ERROR
    status = DiagnosticStatus(level=level, name=STATUS_NAME, hardware_id=HARDWARE_ID, message=decision.text)
    status.values = [KeyValue(key="published", value=str(bool(decision.publish)).lower()),
                     KeyValue(key="reason", value=decision.reason.name),
                     KeyValue(key="seconds_since_last_message", value="none" if last_message_age_s is None else f"{last_message_age_s:.2f}")]
    if decision.publish:
        status.values.append(KeyValue(key="lift_position_m", value=f"{decision.position_m:.4f}"))
    return status


class LiftRelay(Node):
    def __init__(self, message_type, clock=time.monotonic):
        super().__init__("lift_relay")
        self.declare_parameter("stale_after_s", ev.STALE_AFTER_S)
        self.declare_parameter("max_lift_status_age_ms", ev.MAX_LIFT_STATUS_AGE_MS)
        self.declare_parameter("resync_frames", ev.RESYNC_FRAMES)
        self.declare_parameter("range_tolerance_m", ev.RANGE_TOLERANCE_M)
        self.declare_parameter("max_sequence_gap", ev.MAX_SEQUENCE_GAP)
        self.declare_parameter("position_offset_m", 0.0)         # TODO(Evans): calibration offset and fixture tolerance, 0.0 until confirmed
        self.declare_parameter("diagnostic_period_s", 0.5)
        get = lambda name: self.get_parameter(name).value        # noqa: E731
        self.evaluator = ev.LiftEvaluator(stale_after_s=get("stale_after_s"), max_lift_status_age_ms=get("max_lift_status_age_ms"),
                                          resync_frames=get("resync_frames"), range_tolerance_m=get("range_tolerance_m"),
                                          position_offset_m=get("position_offset_m"), max_sequence_gap=get("max_sequence_gap"))
        self.clock = clock                      # LOCAL monotonic receipt time: the header stamp of the message is diagnostic only
        self.last_message = None
        self.published = 0
        self.joint_pub = self.create_publisher(JointState, OUTPUT_TOPIC, 10)
        self.status_pub = self.create_publisher(DiagnosticStatus, "~/status", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_subscription(message_type, INPUT_TOPIC, self.on_status, AUX_STATUS_QOS)
        self.create_timer(float(get("diagnostic_period_s")), self.on_timer)
        self.get_logger().info(f"{INPUT_TOPIC} -> {LIFT_JOINT} on {OUTPUT_TOPIC}; stale after {get('stale_after_s'):g} s")

    def on_status(self, msg):
        now = self.clock()
        self.last_message = now
        decision = self.evaluator.on_status(observation_from_msg(msg), now)
        if decision.publish:
            out = JointState(name=[LIFT_JOINT], position=[decision.position_m])
            out.header.stamp = self.get_clock().now().to_msg()
            self.joint_pub.publish(out)
            self.published += 1
        self.publish_diagnostic(decision, now)

    def on_timer(self):
        now = self.clock()
        self.publish_diagnostic(self.evaluator.poll(now), now)

    def publish_diagnostic(self, decision, now):
        age = None if self.last_message is None else now - self.last_message
        status = diagnostic_for(decision, now, age)
        self.status_pub.publish(status)
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    from openamrobot_upperbody_lift_msgs.msg import BaseAuxStatusStandIn      # TODO(Evans): openamr_base_msgs.msg.BaseAuxStatus
    node = LiftRelay(BaseAuxStatusStandIn)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
