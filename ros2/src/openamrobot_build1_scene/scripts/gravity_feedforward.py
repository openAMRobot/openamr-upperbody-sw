#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Gravity feed-forward of both arms through the effort interface the real driver already reads. Simulation only.

    ros2 run openamrobot_build1_scene gravity_feedforward.py --ros-args -p robot_description:="$(xacro ...)"

Reads the driver's own joint states (/arm/joint_states, the joint_state_broadcaster of /controller_manager), computes
the static holding torque of the 14 arm joints from the URDF (openamrobot_build1_scene/gravity.py) and publishes it to
{side}_arm_gravity_controller (effort_controllers/JointGroupEffortController). OpenArmHW::write puts that value in the
tau field of each MIT frame, next to the trajectory controller's position. Without it tau is 0 and the arm settles
below every pose (docs/build1_arm_diagnosis.md).

Every cycle each arm gets exactly one command. An arm whose seven joints all have a state younger than max_state_age_s
of ROS time gets the model torque; an arm with a stale, incomplete or never-received state gets zeros, with a throttled
warning. Zeros, not silence: the effort controller keeps its last command for ever, so a silent writer would leave the
last torque in the tau field (D4 refinement, docs/architecture/d4_mit_frame_field_ownership.md). This is a functional
command timeout of a simulation and driver-path controller, not a safety function.

A held object: payload_mass_kg (0 = off, the default), payload_com_xyz (m, in the frame of payload_link) and
payload_link (a URDF link, for example openarm_left_grasp_frame). They can be set at run time; scenario 4 sets them
only after the gripper reports the grasp and clears them after the release.
Status: Proposed, pending the software lead's adoption.
"""
import sys

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from openamrobot_build1_scene.gravity import ARM_JOINTS, FeedForward, GravityModel


class GravityFeedforward(Node):
    def __init__(self):
        super().__init__("gravity_feedforward")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("state_topic", "/arm/joint_states")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("max_state_age_s", 0.2)
        self.declare_parameter("payload_mass_kg", 0.0)
        self.declare_parameter("payload_com_xyz", [0.0, 0.0, 0.0])
        self.declare_parameter("payload_link", "")
        description = self.get_parameter("robot_description").value
        if not description:
            raise SystemExit("gravity_feedforward: the robot_description parameter is empty")
        self.model = GravityModel(description)
        self.logic = FeedForward(self.model, float(self.get_parameter("max_state_age_s").value))
        self.logic.payload = self.model.payload(self.get_parameter("payload_mass_kg").value,
                                                self.get_parameter("payload_com_xyz").value, self.get_parameter("payload_link").value)
        self.add_on_set_parameters_callback(self.on_parameters)
        self.publishers_ = {side: self.create_publisher(Float64MultiArray, f"/{side}_arm_gravity_controller/commands", 10)
                            for side in ARM_JOINTS}
        self.sent = 0
        self.create_subscription(JointState, self.get_parameter("state_topic").value, self.on_state, qos_profile_sensor_data)
        self.create_timer(1.0 / float(self.get_parameter("rate_hz").value), self.on_timer)
        self.get_logger().info(f"gravity feed-forward of {sum(len(j) for j in ARM_JOINTS.values())} arm joints from "
                               f"{self.get_parameter('state_topic').value}, zeros on a state older than {self.logic.max_age_s} s")

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_parameters(self, parameters):
        values = {name: self.get_parameter(name).value for name in ("payload_mass_kg", "payload_com_xyz", "payload_link")}
        values.update({p.name: p.value for p in parameters if p.name in values})
        if any(p.name == "max_state_age_s" for p in parameters):
            return SetParametersResult(successful=False, reason="max_state_age_s is read at start only")
        if not any(p.name in values for p in parameters):
            return SetParametersResult(successful=True)
        try:
            payload = self.model.payload(values["payload_mass_kg"], values["payload_com_xyz"], values["payload_link"])
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))
        self.logic.payload = payload
        self.get_logger().info("payload off" if payload is None else
                               f"payload {payload.mass} kg at {list(payload.com)} m in {payload.link}")
        return SetParametersResult(successful=True)

    def on_state(self, msg):
        self.logic.on_state(msg.name, msg.position, self.now_s())

    def on_timer(self):
        commands, stale = self.logic.command(self.now_s())
        for side, torques in commands.items():
            self.publishers_[side].publish(Float64MultiArray(data=torques))
        if stale:
            self.get_logger().warning(f"joint state of the {' and '.join(stale)} arm older than {self.logic.max_age_s} s "
                                      "or never received: tau 0", throttle_duration_sec=5.0)
        elif self.sent == 0:
            self.get_logger().info("first gravity torques sent")
        if not stale:
            self.sent += 1


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
