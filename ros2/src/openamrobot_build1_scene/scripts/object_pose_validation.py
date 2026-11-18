#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Validation recorder of object_pose_source estimate: estimate minus Gazebo ground truth, per object, per estimate.

Subscribes each estimated object's pose (planning_scene.object_pose_topic("estimate", name), PoseWithCovarianceStamped
in base_footprint), Gazebo's pose of that object (object_truth_topic) and of the base (/build1/ground_truth/odom), and
publishes the running record (perception_validation.Record) as JSON on /build1/perception/validation (latched). The
scenario rig writes the last record into each scenario's JSON. Ground truth is used here only to measure; nothing this
node publishes is read by a grasp target, the planning scene or a controller. Simulation evidence only.
"""
import json

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from openamrobot_build1_scene import perception_validation as pv
from openamrobot_build1_scene.planning_scene import ESTIMATED_OBJECTS, ESTIMATE_SOURCE, object_pose_topic, object_truth_topic, pose_fields

TOPIC = "/build1/perception/validation"


class ObjectPoseValidation(Node):
    def __init__(self):
        super().__init__("object_pose_validation")
        self.base, self.truth, self.record = None, {}, pv.Record()
        self.publisher = self.create_publisher(String, TOPIC, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "/build1/ground_truth/odom", self.on_base, 10)
        for name in ESTIMATED_OBJECTS:
            self.create_subscription(PoseStamped, object_truth_topic(name), lambda m, n=name: self.on_truth(n, m), 10)
            self.create_subscription(PoseWithCovarianceStamped, object_pose_topic(ESTIMATE_SOURCE, name),
                                     lambda m, n=name: self.on_estimate(n, m), 10)
        self.get_logger().info(f"validation of object_pose_source estimate on {TOPIC}: estimate minus ground truth in "
                               "base_footprint; ground truth used only to measure")

    def on_base(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.base = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_truth(self, name, msg):
        _frame, position, quaternion = pose_fields(msg)
        self.truth[name] = (position, quaternion)

    def on_estimate(self, name, msg):
        if self.base is None or name not in self.truth:
            return
        _frame, position, quaternion = pose_fields(msg)
        truth_position, truth_yaw = pv.truth_in_base(*self.base, *self.truth[name])
        measured = pv.error(name, position, pv.yaw_of(quaternion), truth_position, truth_yaw, list(msg.pose.covariance))
        self.record.add(name, measured)
        self.publisher.publish(String(data=json.dumps(self.record.summary(), sort_keys=True)))
        count = self.record.objects[name]["count"]
        if count <= 3 or count % 20 == 0:
            self.get_logger().info(f"{name} estimate {count}: position error {measured['position']:.4f} m "
                                   f"(dx {measured['dx']:+.4f}, dy {measured['dy']:+.4f}, dz {measured['dz']:+.4f}), "
                                   f"yaw error {measured['yaw']:+.4f} rad")


def main():
    rclpy.init()
    node = ObjectPoseValidation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
