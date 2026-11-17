#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The Gazebo real-time factor, measured from /clock against the host monotonic clock.

Publishes /build1/rtf (std_msgs/Float32, the factor of the last second) and /build1/rtf_stats (std_msgs/String, JSON:
last, mean, minimum, maximum, simulated seconds, wall seconds, physics steps expected and wall seconds per step).
Runs on wall time on purpose. It reports the Gazebo factor only. A Renode time ratio is a separate number:
openamrobot_sil prints it in /sil/fw/status.
"""
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32, String


class RtfMonitor(Node):
    def __init__(self):
        super().__init__("rtf_monitor")
        self.step_s = self.declare_parameter("step_s", 0.001).value
        # The identity of the run (profile, step, engine, model hash): results name it. Latched.
        self.info_pub = self.create_publisher(String, "/build1/run_info", QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.info_pub.publish(String(data=self.declare_parameter("run_info", "{}").value))
        self.sim = None
        self.first = None          # (wall, sim) of the first clock message
        self.window = None         # (wall, sim) at the last report
        self.samples = []
        self.create_subscription(Clock, "/clock", self.on_clock, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.rtf_pub = self.create_publisher(Float32, "/build1/rtf", 10)
        self.stats_pub = self.create_publisher(String, "/build1/rtf_stats", 10)
        self.create_timer(1.0, self.report)
        self.reports = 0

    def on_clock(self, msg):
        self.sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.first is None:
            self.first = self.window = (time.monotonic(), self.sim)

    def report(self):
        if self.window is None:
            return
        now = time.monotonic()
        wall, sim = now - self.window[0], self.sim - self.window[1]
        if wall <= 0.0:
            return
        factor = sim / wall
        self.window = (now, self.sim)
        self.samples.append(factor)
        total_wall, total_sim = now - self.first[0], self.sim - self.first[1]
        stats = {"rtf_last": round(factor, 4), "rtf_mean": round(total_sim / total_wall, 4) if total_wall > 0 else None,
                 "rtf_min": round(min(self.samples), 4), "rtf_max": round(max(self.samples), 4),
                 "sim_s": round(total_sim, 3), "wall_s": round(total_wall, 3), "step_s": self.step_s,
                 "steps": int(round(total_sim / self.step_s)),
                 "wall_ms_per_step": round(1000.0 * total_wall / max(1.0, total_sim / self.step_s), 4)}
        self.rtf_pub.publish(Float32(data=float(factor)))
        self.stats_pub.publish(String(data=json.dumps(stats, sort_keys=True)))
        self.reports += 1
        if self.reports % 10 == 0:
            self.get_logger().info(f"Gazebo real-time factor: last {factor:.2f}, mean {stats['rtf_mean']}, "
                                   f"min {stats['rtf_min']}, max {stats['rtf_max']} over {total_sim:.0f} s simulated")


def main():
    rclpy.init()
    node = RtfMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
