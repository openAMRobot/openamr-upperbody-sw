#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gravity feed-forward of both arms through the effort interface the real driver already reads. Simulation only.

    ros2 run openamrobot_build1_scene gravity_feedforward.py --ros-args -p robot_description:="$(xacro ...)"

Reads the driver's own joint states (/arm/joint_states, the joint_state_broadcaster of /controller_manager), computes
the static holding torque of the 14 arm joints from the URDF (openamrobot_build1_scene/gravity.py) and publishes it to
{side}_arm_gravity_controller (effort_controllers/JointGroupEffortController). OpenArmHW::write puts that value in the
tau field of each MIT frame, next to the trajectory controller's position. Without it tau is 0 and the arm settles
below every pose (docs/build1_arm_diagnosis.md).

It publishes only while the joint state is fresh (max_state_age_s of ROS time). A stale or incomplete state publishes
nothing: the effort controller keeps its last command, as it would with any silent writer. Not a safety function.
Status: Proposed, pending the software lead's adoption.
"""
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from openamrobot_build1_scene.gravity import ARM_JOINTS, GravityModel


class GravityFeedforward(Node):
    def __init__(self):
        super().__init__("gravity_feedforward")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("state_topic", "/arm/joint_states")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_state_age_s", 0.2)
        description = self.get_parameter("robot_description").value
        if not description:
            raise SystemExit("gravity_feedforward: the robot_description parameter is empty")
        self.model = GravityModel(description)
        self.max_age = float(self.get_parameter("max_state_age_s").value)
        self.publishers_ = {side: self.create_publisher(Float64MultiArray, f"/{side}_arm_gravity_controller/commands", 10)
                            for side in ARM_JOINTS}
        self.positions, self.stamp, self.sent = {}, None, 0
        self.create_subscription(JointState, self.get_parameter("state_topic").value, self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.on_timer)
        self.get_logger().info(f"gravity feed-forward of {sum(len(j) for j in ARM_JOINTS.values())} arm joints from "
                               f"{self.get_parameter('state_topic').value}")

    def on_state(self, msg):
        self.positions.update(zip(msg.name, msg.position))
        self.stamp = self.get_clock().now()

    def on_timer(self):
        if self.stamp is None or (self.get_clock().now() - self.stamp).nanoseconds * 1e-9 > self.max_age:
            return
        sides = [side for side, joints in ARM_JOINTS.items() if all(name in self.positions for name in joints)]
        if not sides:
            return
        names = [name for side in sides for name in ARM_JOINTS[side]]
        torques = dict(zip(names, self.model.torques(self.positions, names)))   # one forward pass for both arms
        for side in sides:
            self.publishers_[side].publish(Float64MultiArray(data=[torques[name] for name in ARM_JOINTS[side]]))
        self.sent += 1
        if self.sent == 1:
            self.get_logger().info("first gravity torques sent")


def main():
    rclpy.init(args=sys.argv)
    node = GravityFeedforward()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
