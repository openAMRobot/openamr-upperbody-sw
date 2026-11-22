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


def dump_simulator_stacks(out):
    """Evidence only: thread stacks of the Gazebo server when no /clock arrives, written to OUT/clock_gate_gz_stacks.txt.
    The launch ends as soon as this script exits, so this is the last moment the stuck process can be read."""
    import subprocess
    path = os.path.join(out, "clock_gate_gz_stacks.txt")
    try:
        pids = subprocess.run(["pgrep", "-f", "gz sim"], capture_output=True, text=True, timeout=5).stdout.split()
        with open(path, "w") as handle:
            handle.write(f"pids of 'gz sim': {pids}\n")
            for pid in pids:
                handle.write(f"\n===== pid {pid}\n")
                handle.flush()
                subprocess.run(["sudo", "-n", "gdb", "-p", pid, "-batch", "-ex", "thread apply all bt 30"],
                               stdout=handle, stderr=subprocess.STDOUT, timeout=60)
    except Exception as error:  # noqa: BLE001 - evidence capture must never change the gate's result
        with open(path, "a") as handle:
            handle.write(f"\ncapture failed: {type(error).__name__}: {error}\n")


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
        dump_simulator_stacks(args.out)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 1 if status is None else status


if __name__ == "__main__":
    sys.exit(main())
