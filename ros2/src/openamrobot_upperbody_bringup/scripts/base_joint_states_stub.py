#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Publish the joints no ros2_control owns on /joint_states, for the MOCK stack only: the base's wheels at rest.

The base is not under ros2_control (ADR-003), so nothing in the mock bringup reports `left_wheel_joint` or `right_wheel_joint`, and
MoveIt, whose robot model includes them, warns every second that "the complete state of the robot is not yet known". The base does not
move during manipulation (AD-30), so reporting the wheels at zero is the true state of the thing being modelled.
`lift_joint` is no longer published here (issue #17): in the mock and gazebo profiles it has its own ros2_control interface and
joint_state_broadcaster reports it (its starting position is mock.launch.py's `lift_position`). In Gazebo the wheel states come from the
simulator and this node is not started.

    ros2 run openamrobot_upperbody_bringup base_joint_states_stub.py [--ros-args -p rate:=10.0]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ("left_wheel_joint", "right_wheel_joint")


class BaseJointStatesStub(Node):
    def __init__(self):
        super().__init__("base_joint_states_stub")
        self.declare_parameter("rate", 10.0)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

    def publish(self):
        names = list(WHEELS)
        msg = JointState(name=names, position=[0.0] * len(names), velocity=[0.0] * len(names))
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BaseJointStatesStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
