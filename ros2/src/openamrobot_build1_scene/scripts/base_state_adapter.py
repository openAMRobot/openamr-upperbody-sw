#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""The one writer of /odom, of the odom -> base_footprint transform and of the base and lift joint states.

openamrobot_sil (openamr-platform-sw #141) publishes what the STM32 slot knows on /sil topics and nothing on /odom,
/tf or /joint_states. This node turns that into the standard topics. It reads no Gazebo topic: everything it
publishes came over a virtual CAN bus first.

  wheel speeds   source:=host_client             twist of /sil/base/odom (the host test client read 606Ch over CAN1)
                 source:=firmware_wheel_speeds   /sil/fw/wheel_speeds (the firmware image in Renode read 606Ch)
  lift position  position_mm of /sil/lift/status (CTL-017 TPDO1 over CAN3). Joint 0 m is the L1000 stop

Time, per sil-topology.md section 6: the wheel speeds are integrated over Gazebo time at local receipt. The pose of
/sil/base/odom is not used, because the host test client integrates it on the host clock and that drifts from the plant
when Gazebo runs slower than real time. A sample older than `stale_s` of Gazebo time counts as zero speed.

Simulation only. Not a safety function.
"""
import json
import math

import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from tf2_ros import TransformBroadcaster

from openamrobot_build1_scene.odometry import DiffDriveOdometry, LIFT_ZERO_MM


class BaseStateAdapter(Node):
    def __init__(self):
        super().__init__("base_state_adapter")
        with open(self.declare_parameter("config_file", "").value) as f:
            drive = yaml.safe_load(f)["drive"]
        self.source = self.declare_parameter("source", "host_client").value
        self.stale_s = self.declare_parameter("stale_s", 0.5).value
        self.odometry = DiffDriveOdometry(drive["wheel_radius"], drive["wheel_separation"])
        self.wheels = (0.0, 0.0)
        self.last_sample = None
        self.lift = None
        self.stale_reported = False
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.joint_pub = self.create_publisher(JointState, "/base/joint_states", 10)
        self.broadcaster = TransformBroadcaster(self)
        if self.source == "host_client":
            self.create_subscription(Odometry, "/sil/base/odom", self.on_host_odom, 10)
        elif self.source == "firmware_wheel_speeds":
            self.create_subscription(Float64MultiArray, "/sil/fw/wheel_speeds", self.on_firmware, 10)
        else:
            raise ValueError(f"source {self.source!r}: host_client or firmware_wheel_speeds")
        self.create_subscription(String, "/sil/lift/status", self.on_lift, 10)
        self.last_tick = None
        self.create_timer(0.02, self.tick)      # 50 Hz of Gazebo time
        self.get_logger().info(f"wheel speeds from {self.source}, r {drive['wheel_radius']} m, separation {drive['wheel_separation']} m")

    def on_host_odom(self, msg):
        self.wheels = self.odometry.wheel_speeds(msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        self.last_sample = self.get_clock().now()

    def on_firmware(self, msg):
        if len(msg.data) == 2 and not any(math.isnan(v) for v in msg.data):
            self.wheels = (float(msg.data[0]), float(msg.data[1]))
            self.last_sample = self.get_clock().now()

    def on_lift(self, msg):
        try:
            position = json.loads(msg.data).get("position_mm")
        except ValueError:
            return
        if position is not None:
            self.lift = (float(position) - LIFT_ZERO_MM) / 1000.0

    def tick(self):
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return                          # no /clock yet
        if self.last_tick is None:
            self.last_tick = now
            return
        dt = (now - self.last_tick).nanoseconds * 1e-9
        self.last_tick = now
        if dt <= 0.0 or dt > 1.0:
            return
        stale = self.last_sample is None or (now - self.last_sample).nanoseconds * 1e-9 > self.stale_s
        if stale and not self.stale_reported and self.last_sample is not None:
            self.get_logger().warning(f"no wheel speed sample for {self.stale_s} s of Gazebo time: integrating zero speed")
        self.stale_reported = stale
        left, right = (0.0, 0.0) if stale else self.wheels
        state = self.odometry.step(left, right, dt)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp, odom.header.frame_id, odom.child_frame_id = stamp, "odom", "base_footprint"
        odom.pose.pose.position.x, odom.pose.pose.position.y = state["x"], state["y"]
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(state["yaw"] / 2.0), math.cos(state["yaw"] / 2.0)
        odom.twist.twist.linear.x, odom.twist.twist.angular.z = state["v"], state["w"]
        self.odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header.stamp, transform.header.frame_id, transform.child_frame_id = stamp, "odom", "base_footprint"
        transform.transform.translation.x, transform.transform.translation.y = state["x"], state["y"]
        transform.transform.rotation.z, transform.transform.rotation.w = odom.pose.pose.orientation.z, odom.pose.pose.orientation.w
        self.broadcaster.sendTransform(transform)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = ["left_wheel_joint", "right_wheel_joint"]
        joints.position = [state["left_angle"], state["right_angle"]]
        joints.velocity = [left, right]
        if self.lift is not None:
            joints.name.append("lift_joint")
            joints.position.append(self.lift)
            joints.velocity.append(0.0)
        self.joint_pub.publish(joints)


def main():
    rclpy.init()
    node = BaseStateAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
