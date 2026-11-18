# SPDX-License-Identifier: MIT
"""The control room virtual CAN monitor node: receive-only frame counts of the Build 1 buses on /control_room/can/.

  /control_room/can/summary        std_msgs/String JSON, 1 Hz: per bus state, frames, rate, CANopen class counts, top IDs
  /control_room/can/<bus>/rate_hz  std_msgs/Float64, 1 Hz: frames per second of the last second

It opens each interface with python-can socketcan (can_monitor.open_socketcan) behind can_monitor.ReceiveOnlyBus,
which has no send method. An absent interface is reported as "absent" and retried; it is never created here
(setup_vcan_all.sh of openamrobot_build1_scene does that). Same read-only rules as monitor.py.
"""
import json

from openamrobot_control_room import can_monitor, readonly


class VcanMonitorNode:
    def __init__(self, node, opener=can_monitor.open_socketcan):
        from rclpy.clock import Clock, ClockType
        from std_msgs.msg import Float64, String
        self.node, self.String, self.Float64 = node, String, Float64
        buses = list(node.declare_parameter("buses", list(can_monitor.BUSES)).value)
        canopen = list(node.declare_parameter("canopen_buses", list(can_monitor.CANOPEN_BUSES)).value)
        self.monitor = can_monitor.Monitor(buses, canopen, opener=opener).start()
        self.summary_pub = node.create_publisher(String, readonly.guard_publisher_topic("/control_room/can/summary"), 10)
        self.rate_pub = {b: node.create_publisher(Float64, readonly.guard_publisher_topic(f"/control_room/can/{b}/rate_hz"), 10)
                         for b in buses}
        node.create_timer(1.0, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        summary = self.monitor.summary()
        self.summary_pub.publish(self.String(data=json.dumps(
            {"buses": summary, "note": "receive-only frame counts; no payload decoded, nothing sent"}, sort_keys=True)))
        for bus, record in summary.items():
            self.rate_pub[bus].publish(self.Float64(data=float(record["rate_hz"])))

    def stop(self):
        self.monitor.stop()


def main(args=None):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from openamrobot_control_room.monitor import make_node
    rclpy.init(args=args)
    node = make_node("control_room_vcan_monitor")
    monitor = VcanMonitorNode(node)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        monitor.stop()
        node.destroy_node()
        rclpy.try_shutdown()
