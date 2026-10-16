#!/usr/bin/env python3
"""Idle probe: collect /joint_states for N messages after the controllers are up; print per-joint stats of the arm joints near their limits."""
import json, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
import xml.etree.ElementTree as ET

class P(Node):
    def __init__(self, n_target, topic):
        super().__init__("probe_reader", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.n_target = n_target; self.msgs = []; self.t0 = None
        self.create_subscription(JointState, topic, self.cb, qos_profile_sensor_data)
    def cb(self, m):
        self.msgs.append((dict(zip(m.name, m.position)),))
def main():
    n = int(sys.argv[1]); topic = sys.argv[2] if len(sys.argv) > 2 else "/joint_states"
    rclpy.init(); node = P(n, topic)
    # wait for the sim clock to run for a while so the model has settled (spawn 0.15 m above the ground)
    deadline = time.monotonic() + 150
    while time.monotonic() < deadline and len(node.msgs) < 5:
        rclpy.spin_once(node, timeout_sec=0.2)
    settle_end = node.get_clock().now().nanoseconds * 1e-9 + 25.0     # 25 s of SIM time
    while time.monotonic() < deadline and node.get_clock().now().nanoseconds * 1e-9 < settle_end:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.msgs.clear()
    while time.monotonic() < deadline and len(node.msgs) < n:
        rclpy.spin_once(node, timeout_sec=0.2)
    names = sorted({k for (d,) in node.msgs for k in d})
    out = {}
    for k in names:
        v = [d[k] for (d,) in node.msgs if k in d]
        out[k] = {"n": len(v), "mean": sum(v) / len(v), "min": min(v), "max": max(v)}
    print("PROBE_JSON " + json.dumps({"messages": len(node.msgs), "joints": out}))
    node.destroy_node(); rclpy.shutdown()
main()
