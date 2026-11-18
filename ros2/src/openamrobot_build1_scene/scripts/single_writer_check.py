#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Checks the running Build 1 system against config/command_map.yaml: one writer for /tf frames, /odom,
/joint_states and every command topic, one server per action, one claimed command interface per joint.

    single_writer_check.py --profile firmware_sil [--components nav2,moveit] [--settle 20]

Exit 0: no problem. Exit 1: problems, one per line. Simulation tooling, not a safety function.
"""
import argparse
import os
import sys

import yaml

from openamrobot_build1_scene import single_writer_live


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--profile", required=True, choices=["fast", "fine", "firmware_sil", "faults"])
    parser.add_argument("--components", default="nav2,moveit")
    parser.add_argument("--settle", type=float, default=20.0, help="wall seconds the ROS graph must stay unchanged")
    parser.add_argument("--map", default=None)
    args, _ros = parser.parse_known_args(argv)
    path = args.map
    if path is None:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "command_map.yaml")
    with open(path, encoding="utf-8") as f:
        command_map = yaml.safe_load(f)
    import rclpy
    rclpy.init()
    node = rclpy.create_node("build1_single_writer_check")
    try:
        report = single_writer_live.check(node, command_map, args.profile, {c for c in args.components.split(",") if c},
                                          settle_s=args.settle)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    for line in report["facts"]:
        print(line)
    for problem in report["problems"]:
        print(f"PROBLEM: {problem}")
    print(f"single-writer check: {len(report['problems'])} problems, {report['topics_checked']} topics, "
          f"{report['actions_checked']} actions, {report['frames_seen']} frames")
    return 1 if report["problems"] else 0


if __name__ == "__main__":
    sys.exit(main())
