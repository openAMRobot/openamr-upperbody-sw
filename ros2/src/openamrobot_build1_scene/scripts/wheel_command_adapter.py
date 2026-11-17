#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Applies the drive emulator's per-wheel demand to the Gazebo wheels, one wheel at a time.

openamrobot_sil's ZLAC8015D emulator node folds its two ramped wheel speeds into one body twist on
/sil/drive/gz_cmd_vel, with the wheel radius and separation of its config/sil.yaml. This node unfolds that twist with
the same two numbers from the same file, which gives the two wheel speeds back exactly, and writes each to the velocity
input of its wheel joint. No second ramp, no second limit and no second set of geometry acts on the command
(conflict SW-03 of the phase 1 single-writer map). It adds no behaviour of its own: no limit, no timeout, no stop.
A message is passed on as it arrives. Simulation only. Not a safety function.
"""
import rclpy
import yaml
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64

from openamrobot_build1_scene.odometry import DiffDriveOdometry


class WheelCommandAdapter(Node):
    def __init__(self):
        super().__init__("wheel_command_adapter")
        with open(self.declare_parameter("config_file", "").value) as f:
            drive = yaml.safe_load(f)["drive"]
        self.model = DiffDriveOdometry(drive["wheel_radius"], drive["wheel_separation"])
        self.left = self.create_publisher(Float64, "/build1/drive/left_wheel_cmd_vel", 10)
        self.right = self.create_publisher(Float64, "/build1/drive/right_wheel_cmd_vel", 10)
        self.create_subscription(Twist, "/sil/drive/gz_cmd_vel", self.on_twist, 10)

    def on_twist(self, msg):
        left, right = self.model.wheel_speeds(msg.linear.x, msg.angular.z)
        self.left.publish(Float64(data=left))
        self.right.publish(Float64(data=right))


def main():
    rclpy.init()
    node = WheelCommandAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
