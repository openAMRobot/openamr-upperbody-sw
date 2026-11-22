#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Exit 0 at the first ROS /clock message; exit clock_gate.EXIT_NO_CLOCK when none arrives in time.

    wait_for_clock.py --launch-start EPOCH_S [--bound S] [--out DIR]

build1.launch.py starts the real arm driver only after this exits 0 (driver_clock:=sim), and ends the launch with the
named reason "no /clock" on EXIT_NO_CLOCK. Wall time on purpose: the clock it waits for is the one that is missing.
Writes DIR/clock_gate.json in both cases. Simulation only; not a safety function.
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import clock_gate  # noqa: E402


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-start", type=float, required=True)
    parser.add_argument("--bound", type=float, default=clock_gate.BOUND_S)
    parser.add_argument("--out", default=os.environ.get("BUILD1_LOG_DIR", "."))
    args, _ros = parser.parse_known_args(argv)
    rclpy.init()
    node = Node("build1_clock_gate")
    gate = clock_gate.Gate(args.launch_start, args.bound)
    node.create_subscription(Clock, "/clock", lambda m: gate.on_clock(m.clock.sec + m.clock.nanosec * 1e-9),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    status = None
    try:
        while status is None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            status = gate.poll()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        status = None
    record = gate.record()
    clock_gate.write_record(args.out, record)
    if status == 0:
        node.get_logger().info(f"first ROS /clock {record['first_clock_after_launch_s']} s after the launch start "
                               f"(sim {record['first_clock_sim_s']} s); starting the real arm driver")
    elif status == clock_gate.EXIT_NO_CLOCK:
        node.get_logger().error(record["reason"])
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if status is None else status


if __name__ == "__main__":
    sys.exit(main())
