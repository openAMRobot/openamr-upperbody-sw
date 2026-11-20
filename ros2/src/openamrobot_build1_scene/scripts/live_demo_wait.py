#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Wait until the simulated stack is up: simulation time advances and /tf, /scan and /odom are published.

    live_demo_wait.py [--timeout 600] [--min-sim 2.0] [--topics /tf /scan /odom]

run_grocery_demo_gui.sh runs it before it opens RViz and before it starts the mission. It subscribes only (best
effort QoS, which matches reliable and best effort publishers) and publishes nothing. Every 15 s it prints what is
still missing. Exit 0: ready. Exit 2: not ready within --timeout wall seconds (the missing topics are printed).
Exit 3: rclpy is not importable (ROS 2 not sourced). Simulation readiness only, not a health or safety check.
"""
import argparse
import sys
import time

DEFAULT_TOPICS = ("/tf", "/scan", "/odom")


class Readiness:
    """The bookkeeping, without ROS: seen topics and the simulation clock."""

    def __init__(self, topics, min_sim):
        self.topics = tuple(topics)
        self.min_sim = min_sim
        self.seen = set()
        self.first_clock = None
        self.last_clock = None

    def clock(self, seconds):
        if self.first_clock is None:
            self.first_clock = seconds
        self.last_clock = seconds

    def message(self, topic):
        self.seen.add(topic)

    def sim_advanced(self):
        return self.first_clock is not None and self.last_clock - self.first_clock >= self.min_sim

    def missing(self):
        missing = [topic for topic in self.topics if topic not in self.seen]
        if self.first_clock is None:
            missing.insert(0, "/clock")
        elif not self.sim_advanced():
            missing.insert(0, f"/clock advancing {self.min_sim:.1f} s (now {self.last_clock - self.first_clock:.1f} s)")
        return missing

    def ready(self):
        return not self.missing()


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=600.0, help="wall seconds")
    parser.add_argument("--min-sim", type=float, default=2.0, help="simulation seconds /clock must advance")
    parser.add_argument("--topics", nargs="+", default=list(DEFAULT_TOPICS))
    args, _ros = parser.parse_known_args(argv)
    try:
        import rclpy
        from nav_msgs.msg import Odometry
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan
        from tf2_msgs.msg import TFMessage
    except ImportError as error:
        print(f"live_demo_wait: {error}: source /opt/ros/jazzy/setup.bash and the workspace", flush=True)
        return 3
    kinds = {"/tf": TFMessage, "/tf_static": TFMessage, "/scan": LaserScan, "/scan_filtered": LaserScan, "/odom": Odometry}
    state = Readiness(args.topics, args.min_sim)
    rclpy.init()
    node = rclpy.create_node("live_demo_wait")
    qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    node.create_subscription(Clock, "/clock", lambda m: state.clock(m.clock.sec + m.clock.nanosec * 1e-9), qos)
    for topic in args.topics:
        node.create_subscription(kinds.get(topic, TFMessage), topic, lambda _m, t=topic: state.message(t), qos)
    start = time.monotonic()
    report = start
    try:
        while time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
            if state.ready():
                print(f"live_demo_wait: ready after {time.monotonic() - start:.0f} s wall: /clock advancing, "
                      + ", ".join(args.topics), flush=True)
                return 0
            if time.monotonic() - report >= 15.0:
                report = time.monotonic()
                print(f"live_demo_wait: {report - start:.0f} s, waiting for {', '.join(state.missing())}", flush=True)
        print(f"live_demo_wait: NOT READY after {args.timeout:.0f} s: missing {', '.join(state.missing())}", flush=True)
        return 2
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
