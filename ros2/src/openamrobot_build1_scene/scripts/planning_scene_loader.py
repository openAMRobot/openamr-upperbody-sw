#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Puts the scene's collision objects into the MoveIt planning scene, and keeps them where the robot believes they are.

MoveIt's model of this robot has a fixed world -> base_footprint joint: the planning frame moves with the base. So the
objects of config/scene.yaml are given in base_footprint, from the robot's own pose estimate (map -> base_footprint
when Nav2 localises, else the spawn pose with odom -> base_footprint). When the base has moved, the objects are sent
again. This node is the one writer of /planning_scene besides move_group itself.

The robot's pose for that comes from its own estimate by default (pose_source map, planning_scene.py); pose_source
ground_truth (a simulation oracle) takes /build1/ground_truth/odom instead and must be asked for explicitly. Topic /build1/planning_scene/exclude (std_msgs/String, comma separated names) removes objects, for example the item
before a grasp. The fixed boxes get the planning clearance fixture_padding_m (planning_scene.FIXTURE_PADDING_M) on every
side; the tote, the item and the payload get none. Simulation only.
"""
import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_build1_scene import scene as scene_model
from openamrobot_build1_scene.planning_scene import DEFAULT_POSE_SOURCE, FIXTURE_PADDING_M, choose_base_pose, declaration, padded


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__("planning_scene_loader")
        self.scene = scene_model.load(self.declare_parameter("scene_file", "").value)
        self.spawn = self.declare_parameter("spawn_pose", [0.0, 0.0, 0.0]).value
        padding = float(self.declare_parameter("fixture_padding_m", FIXTURE_PADDING_M).value)
        fixed = {name for name, _size, _centre, _yaw in scene_model.static_boxes(self.scene)}
        self.boxes = padded(scene_model.collision_boxes(self.scene), fixed, padding)
        self.get_logger().info(f"planning clearance {padding} m on every side of the {len(fixed)} fixed boxes; "
                               "tote, item and payload unpadded")
        self.excluded, self.sent_names = set(), set()
        self.pose_source = self.declare_parameter("pose_source", DEFAULT_POSE_SOURCE).value
        self.get_logger().info(declaration(self.pose_source))
        self.truth = None
        self.create_subscription(Odometry, "/build1/ground_truth/odom", self.on_truth, 10)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.names_pub = self.create_publisher(String, "/build1/planning_scene/objects",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/build1/planning_scene/exclude", self.on_exclude, 10)
        self.last_pose, self.last_sent = None, None
        self.startup_sends = 0
        self.create_timer(0.5, self.tick)

    def on_exclude(self, msg):
        self.excluded = {name.strip() for name in msg.data.split(",") if name.strip()}
        self.last_pose = None      # send again

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y, yaw_of(q))

    def base_pose(self):
        """(x, y, yaw) of base_footprint in the scene frame and its source: the robot's estimate (pose_source map, the
        default) or the ground truth (pose_source ground_truth, a simulation oracle), planning_scene.choose_base_pose."""
        return choose_base_pose(self.pose_source, self.truth, self.localization_pose())

    def localization_pose(self):
        """(x, y, yaw) of base_footprint in the scene frame, and the source of that belief."""
        try:
            t = self.buffer.lookup_transform("map", "base_footprint", Time()).transform
            return (t.translation.x, t.translation.y, yaw_of(t.rotation)), "map"
        except TransformException:
            pass
        try:
            t = self.buffer.lookup_transform("odom", "base_footprint", Time()).transform
        except TransformException:
            return None, None
        sx, sy, syaw = self.spawn
        c, s = math.cos(syaw), math.sin(syaw)
        return (sx + c * t.translation.x - s * t.translation.y, sy + s * t.translation.x + c * t.translation.y,
                syaw + yaw_of(t.rotation)), "spawn pose and odom"

    def tick(self):
        pose, source = self.base_pose()
        if pose is None:
            return
        now = self.get_clock().now()
        moved = self.last_pose is None or math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1]) > 0.01 \
            or abs(math.atan2(math.sin(pose[2] - self.last_pose[2]), math.cos(pose[2] - self.last_pose[2]))) > 0.01
        # Sent when the base has moved or the list changed. At start it is sent three times, 5 s apart, so that a
        # move_group that came up late has it. Not sent periodically after that: every update makes MoveIt check its
        # running plan again.
        if not moved and self.last_sent is not None:
            if self.startup_sends >= 3 or (now - self.last_sent).nanoseconds < 5e9:
                return
        self.startup_sends += 1
        message = PlanningScene(is_diff=True)
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        names = set()
        for name, size, centre, yaw in self.boxes:
            if name in self.excluded:
                continue
            dx, dy = centre[0] - pose[0], centre[1] - pose[1]
            box_pose = Pose()
            box_pose.position.x, box_pose.position.y, box_pose.position.z = c * dx - s * dy, s * dx + c * dy, centre[2]
            relative = yaw - pose[2]
            box_pose.orientation.z, box_pose.orientation.w = math.sin(relative / 2.0), math.cos(relative / 2.0)
            collision = CollisionObject(id=name, operation=CollisionObject.ADD)
            collision.header.frame_id = "base_footprint"
            collision.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size])]
            collision.primitive_poses = [box_pose]
            message.world.collision_objects.append(collision)
            names.add(name)
        for name in self.sent_names - names:
            removal = CollisionObject(id=name, operation=CollisionObject.REMOVE)
            removal.header.frame_id = "base_footprint"
            message.world.collision_objects.append(removal)
        self.publisher.publish(message)
        self.names_pub.publish(String(data=",".join(sorted(names))))
        if self.last_pose is None or names != self.sent_names:
            self.get_logger().info(f"{len(names)} collision objects in base_footprint, base at "
                                   f"({pose[0]:.2f}, {pose[1]:.2f}, {pose[2]:.2f}) from {source}")
        self.sent_names, self.last_pose, self.last_sent = names, pose, now


def main():
    rclpy.init()
    node = PlanningSceneLoader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
