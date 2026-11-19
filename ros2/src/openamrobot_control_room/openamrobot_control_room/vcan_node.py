# SPDX-License-Identifier: MIT
"""The control room virtual CAN monitor node: receive-only counts and decoded values of the Build 1 buses.

  /control_room/can/summary          std_msgs/String JSON, 1 Hz: per bus state, frames, rate per wall second and per
                                     Gazebo second, CANopen class counts, top IDs
  /control_room/can/decoded          std_msgs/String JSON, 1 Hz: the latest engineering value of every signal seen on
                                     each bus (can_decode.py) and the last raw frame of each ID
  /control_room/can/<bus>/rate_hz    std_msgs/Float64, 1 Hz: frames per wall second of the last second

It opens each interface with python-can socketcan (can_monitor.open_socketcan) behind can_monitor.ReceiveOnlyBus,
which has no send method. An absent interface is reported as "absent" and retried; it is never created here
(setup_vcan_all.sh of openamrobot_build1_scene does that). The frame times are host time (as candump's); the
Gazebo time of each summary comes from /clock. Same read-only rules as monitor.py.
"""
import json
import threading

from openamrobot_control_room import can_decode, can_monitor, readonly


class VcanMonitorNode:
    def __init__(self, node, opener=can_monitor.open_socketcan):
        from rclpy.clock import Clock, ClockType
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock as ClockMsg
        from std_msgs.msg import Float64, String
        self.node, self.String, self.Float64 = node, String, Float64
        buses = list(node.declare_parameter("buses", list(can_monitor.BUSES)).value)
        canopen = list(node.declare_parameter("canopen_buses", list(can_monitor.CANOPEN_BUSES)).value)
        self.latest = can_decode.Latest()
        self.lock = threading.Lock()
        self.decode_errors = 0
        self.sim_time = None
        self.previous = None          # (sim time, {bus: frames}) of the last summary
        self.monitor = can_monitor.Monitor(buses, canopen, opener=opener, on_frame=self.on_frame).start()
        self.summary_pub = node.create_publisher(String, readonly.guard_publisher_topic("/control_room/can/summary"), 10)
        self.decoded_pub = node.create_publisher(String, readonly.guard_publisher_topic("/control_room/can/decoded"), 10)
        self.rate_pub = {b: node.create_publisher(Float64, readonly.guard_publisher_topic(f"/control_room/can/{b}/rate_hz"), 10)
                         for b in buses}
        node.create_subscription(ClockMsg, "/clock", self.on_clock, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_timer(1.0, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def on_clock(self, msg):
        self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_frame(self, bus, msg):
        """Called in the bus's receive thread: decode and keep the latest value. Never answers."""
        try:
            with self.lock:
                self.latest.add(bus, msg.arbitration_id, bytes(msg.data), bool(msg.is_extended_id), t=msg.timestamp)
        except Exception:  # noqa: BLE001 - a frame the decoder does not understand is counted, the monitor goes on
            self.decode_errors += 1

    def sim_rates(self, summary):
        """Frames per Gazebo second since the last summary, per bus (None until two summaries with /clock)."""
        now = self.sim_time
        frames = {bus: record["frames"] for bus, record in summary.items()}
        rates = {bus: None for bus in frames}
        if now is not None and self.previous is not None and now > self.previous[0]:
            dt = now - self.previous[0]
            rates = {bus: round((n - self.previous[1].get(bus, 0)) / dt, 1) for bus, n in frames.items()}
        if now is not None:
            self.previous = (now, frames)
        return rates

    def tick(self):
        summary = self.monitor.summary()
        for bus, rate in self.sim_rates(summary).items():
            summary[bus]["rate_per_sim_s"] = rate
        self.summary_pub.publish(self.String(data=json.dumps(
            {"buses": summary, "sim_time": self.sim_time,
             "note": "receive-only; rate_hz per wall second, rate_per_sim_s per Gazebo second; nothing sent"},
            sort_keys=True)))
        with self.lock:
            decoded = json.loads(json.dumps(self.latest.summary(), default=str))
        decoded.update(sim_time=self.sim_time, decode_errors=self.decode_errors,
                       note="decoded by openamrobot_control_room can_decode.py from emulated devices on vcan; "
                            "frame times t are host time")
        self.decoded_pub.publish(self.String(data=json.dumps(decoded, sort_keys=True)))
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
