#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Diagnostic capture for the "no /clock for the whole leg" start-up failure (gate owner decision on #77, item 1).

    clock_capture.py --launch-start EPOCH_S [--out DIR]

Started by build1.launch.py at the launch start when BUILD1_CLOCK_CAPTURE=1. Into DIR/clock_capture/:
- netstat_su_after_15s.txt (and proc_net_snmp_after_15s.txt): UDP counters 15 s after the launch start; the workflow
  writes the "before" files;
- when no ROS /clock message has arrived 10 s after the launch start: gz_topic_echo_clock.txt
  (gz topic -e -t /clock -n 1, bounded), gz_topic_info_clock.txt (gz topic -i -t /clock) and
  ros2_topic_info_clock.txt (ros2 topic info /clock -v), each under this leg's GZ_PARTITION and ROS domain;
- capture.json: when the first /clock arrived and which probes ran.
Evidence only: it changes no gate, starts and stops nothing, and always exits 0. Simulation only.
"""
import argparse
import json
import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import clock_gate  # noqa: E402

PROBES = (("gz_topic_echo_clock.txt", ["timeout", "10", "gz", "topic", "-e", "-t", "/clock", "-n", "1"]),
          ("gz_topic_info_clock.txt", ["timeout", "10", "gz", "topic", "-i", "-t", "/clock"]),
          ("ros2_topic_info_clock.txt", ["timeout", "30", "ros2", "topic", "info", "/clock", "-v", "--no-daemon"]))


def run(path, command):
    started = time.time()
    try:
        done = subprocess.run(command, capture_output=True, text=True, timeout=60)
        text, rc = done.stdout + done.stderr, done.returncode
    except (OSError, subprocess.SubprocessError) as error:
        text, rc = f"{type(error).__name__}: {error}", None
    with open(path, "w", encoding="utf-8") as f:
        f.write(f"$ {' '.join(command)}\n# exit {rc}, {time.time() - started:.1f} s, GZ_PARTITION="
                f"{os.environ.get('GZ_PARTITION')}, ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}\n{text}")
    return rc


def udp_counters(out, suffix):
    run(os.path.join(out, f"netstat_su_{suffix}.txt"), ["netstat", "-su"])
    run(os.path.join(out, f"proc_net_snmp_{suffix}.txt"), ["cat", "/proc/net/snmp"])


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--launch-start", type=float, required=True)
    parser.add_argument("--out", default=os.environ.get("BUILD1_LOG_DIR", "."))
    args, _ros = parser.parse_known_args(argv)
    out = os.path.join(args.out, clock_gate.CAPTURE_DIR)
    os.makedirs(out, exist_ok=True)
    summary = {"launch_start": args.launch_start, "first_clock_after_launch_s": None, "probes_run": False,
               "probe_at_s": clock_gate.PROBE_AT_S, "netstat_at_s": clock_gate.NETSTAT_AT_S, "probe_exit": {}}
    rclpy.init()
    node = Node("build1_clock_capture")
    first = []
    node.create_subscription(Clock, "/clock", lambda _m: first or first.append(time.time()),
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    probed = netstat = False
    probes = None

    def probe_all():
        for name, command in PROBES:
            summary["probe_exit"][name] = run(os.path.join(out, name), command)
    try:
        # Until the probes and counters are done and either a clock arrived or the fail-fast bound passed.
        while rclpy.ok() and not (probed and netstat and (probes is None or not probes.is_alive())
                                  and (first or time.time() - args.launch_start >= clock_gate.BOUND_S)):
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = time.time() - args.launch_start
            if not probed and elapsed >= clock_gate.PROBE_AT_S:
                probed = True
                if not first:
                    summary["probes_run"] = True
                    node.get_logger().warning(f"no ROS /clock {elapsed:.1f} s after the launch start: capturing "
                                              f"gz topic and ros2 topic info into {out} (evidence only)")
                    # In a thread, so the 15 s UDP counters are taken on time.
                    probes = threading.Thread(target=probe_all, daemon=True)
                    probes.start()
            if not netstat and time.time() - args.launch_start >= clock_gate.NETSTAT_AT_S:
                netstat = True
                udp_counters(out, "after_15s")
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    summary["first_clock_after_launch_s"] = round(first[0] - args.launch_start, 3) if first else None
    with open(os.path.join(out, "capture.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, sort_keys=True)
    node.get_logger().info(f"clock capture: {json.dumps(summary, sort_keys=True)}")
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
