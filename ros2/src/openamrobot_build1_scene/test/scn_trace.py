#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""SCRATCH (platform-sw #192, not for merge): 2 Hz trace of the base while a scene scenario runs.
Prints one JSON row per 0.5 s of wall time: commands on the Nav2 chain, AMCL and ground truth poses, Nav2 log lines."""
import json
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from std_msgs.msg import String

NAMES = ("controller_server", "bt_navigator", "planner_server", "collision_monitor", "behavior_server", "velocity_smoother",
         "bt_navigator_navigate_to_pose_rclcpp_node")
TYPES = {"geometry_msgs/msg/Twist": (Twist, False), "geometry_msgs/msg/TwistStamped": (TwistStamped, True)}


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Trace(Node):
    def __init__(self):
        super().__init__("scn_trace", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.cmd, self.pose, self.logs = {}, {}, []
        self.pending = {"/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel"}
        self.create_subscription(Log, "/rosout", self.on_log, 200)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", lambda m: self.store("amcl", m.pose.pose), 10)
        self.create_subscription(Odometry, "/build1/ground_truth/odom", lambda m: self.store("truth", m.pose.pose), 10)
        self.create_subscription(Odometry, "/odom", lambda m: self.store("odom", m.pose.pose), 10)
        # SIL drive path below /cmd_vel: host stand-in status, bus audit, emulator state, emulator output to Gazebo,
        # joint states back. Message counts per row and the latest text, so that the silent part can be named.
        self.sil_count, self.sil_text = {}, {}
        for topic in ("/sil/drive/status", "/sil/drive/emulator_state", "/sil/drive_bus/audit"):
            self.create_subscription(String, topic, self.on_sil_text(topic), 20)
        self.create_subscription(Twist, "/sil/drive/gz_cmd_vel", self.on_gz_cmd, 20)
        self.create_subscription(JointState, "/sil/gz/joint_states", self.on_joints, 20)
        self.create_subscription(Odometry, "/sil/base/odom", lambda m: self.store("sil_odom", m.pose.pose), 10)

    def store(self, key, pose):
        self.pose[key] = (round(pose.position.x, 4), round(pose.position.y, 4), round(yaw(pose.orientation), 4))

    def on_cmd(self, topic, stamped):
        def cb(msg):
            t = msg.twist if stamped else msg
            self.cmd[topic] = (round(t.linear.x, 4), round(t.angular.z, 4))
        return cb

    def on_sil_text(self, topic):
        def cb(msg):
            self.sil_count[topic] = self.sil_count.get(topic, 0) + 1
            self.sil_text[topic] = msg.data[:400]
        return cb

    def on_gz_cmd(self, msg):
        self.sil_count["gz_cmd"] = self.sil_count.get("gz_cmd", 0) + 1
        self.sil_text["gz_cmd"] = (round(msg.linear.x, 4), round(msg.angular.z, 4))

    def on_joints(self, msg):
        self.sil_count["joints"] = self.sil_count.get("joints", 0) + 1
        wheels = [(n, round(v, 3)) for n, v in zip(msg.name, msg.velocity) if "wheel" in n]
        self.sil_text["joints"] = wheels[:4]

    def on_log(self, msg):
        if msg.name in NAMES and msg.level >= 20:
            self.logs.append((msg.name, msg.msg[:140]))

    def subscribe_commands(self):
        known = dict(self.get_topic_names_and_types())
        for topic in sorted(self.pending):
            for name in known.get(topic, []):
                if name in TYPES:
                    cls, stamped = TYPES[name]
                    self.create_subscription(cls, topic, self.on_cmd(topic, stamped), 10)
                    self.pending.discard(topic)
                    break


def main():
    rclpy.init()
    node = Trace()
    last = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last >= 0.5:
                last = now
                node.subscribe_commands()
                row = {"w": round(now, 1), "sim": round(node.get_clock().now().nanoseconds * 1e-9, 2), "cmd": node.cmd,
                       **node.pose}
                if node.logs:
                    row["log"] = node.logs
                row["sil_n"] = node.sil_count
                rows = getattr(node, "rows", 0) + 1
                node.rows = rows
                if rows % 4 == 1:
                    row["sil"] = dict(node.sil_text)
                node.cmd, node.logs, node.sil_count = {}, [], {}
                print("TRACE2HZ " + json.dumps(row), flush=True)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    sys.exit(main())
