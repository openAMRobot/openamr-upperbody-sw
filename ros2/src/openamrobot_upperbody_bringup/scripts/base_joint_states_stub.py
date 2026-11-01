#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Publish the joints no ros2_control owns at their operating point on /joint_states, for the MOCK stack only: the base's wheels
at rest and the lift at `lift_position`.

Neither the base nor the lift is under ros2_control (ADR-003; the lift is a joint we have no authority to command, option B's
`effort="0" velocity="0"`), so nothing in the mock bringup reports `left_wheel_joint`, `right_wheel_joint` or `lift_joint`, and
MoveIt, whose robot model includes them, warns every second that "the complete state of the robot is not yet known", and
robot_state_publisher publishes nothing above the lift. The base does not move during manipulation (AD-30), so reporting the
wheels at zero is the true state of the thing being modelled. The lift is not at zero by nature: `lift_position` is the operating
point of a run (calibration stops L1000 / L1175 / L1350 = 0.0 / 0.175 / 0.350 m) and the default 0.0 is every joint at zero.
In Gazebo the wheel states come from the simulator and the lift from joint_state_publisher's default (merged by sim.launch.py),
and this node is not started.

    ros2 run openamrobot_upperbody_bringup base_joint_states_stub.py [--ros-args -p rate:=10.0 -p lift_position:=0.175]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

WHEELS = ("left_wheel_joint", "right_wheel_joint")
LIFT = "lift_joint"


class BaseJointStatesStub(Node):
    def __init__(self):
        super().__init__("base_joint_states_stub")
        self.declare_parameter("rate", 10.0)
        self.declare_parameter("lift_position", 0.0)
        self.lift_position = float(self.get_parameter("lift_position").value)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.publish)

    def publish(self):
        names = [*WHEELS, LIFT]
        msg = JointState(name=names, position=[0.0] * len(WHEELS) + [self.lift_position], velocity=[0.0] * len(names))
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
