#!/usr/bin/env python3
"""Rest probe, P17 task 5: collect /arm/joint_states (position, velocity AND effort) for N messages after the controllers are up and the model has
settled, and print per-joint statistics. Nothing is commanded. `effort` is the joint_state_broadcaster's reading of the `effort` state interface
that gz_ros2_control fills from the simulator's joint force for the joint."""
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class Reader(Node):
    def __init__(self, topic):
        super().__init__("effort_reader", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.rows = []
        self.create_subscription(JointState, topic, self.cb, QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE))

    def cb(self, m):
        self.rows.append({n: (p, v, e) for n, p, v, e in zip(m.name, m.position,
                                                            m.velocity if len(m.velocity) == len(m.name) else [float("nan")] * len(m.name),
                                                            m.effort if len(m.effort) == len(m.name) else [float("nan")] * len(m.name))})


def main():
    n = int(sys.argv[1])
    topic = sys.argv[2] if len(sys.argv) > 2 else "/arm/joint_states"
    rclpy.init()
    node = Reader(topic)
    deadline = time.monotonic() + 170
    while time.monotonic() < deadline and len(node.rows) < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    settle_end = node.get_clock().now().nanoseconds * 1e-9 + 25.0       # 25 s of SIM time, as probe_reader.py does
    while time.monotonic() < deadline and node.get_clock().now().nanoseconds * 1e-9 < settle_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.rows.clear()
    while time.monotonic() < deadline and len(node.rows) < n:
        rclpy.spin_once(node, timeout_sec=0.2)
    out = {}
    for name in sorted({k for r in node.rows for k in r}):
        col = [r[name] for r in node.rows if name in r]
        stat = lambda i: {"mean": sum(c[i] for c in col) / len(col), "min": min(c[i] for c in col), "max": max(c[i] for c in col)}
        out[name] = {"n": len(col), "position": stat(0), "velocity": stat(1), "effort": stat(2)}
    print("EFFORT_JSON " + json.dumps({"messages": len(node.rows), "topic": topic, "joints": out}))
    node.destroy_node()
    rclpy.shutdown()


main()
