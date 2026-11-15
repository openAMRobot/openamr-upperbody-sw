#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Puts the scene's collision objects into the MoveIt planning scene, and keeps them where the robot believes they are.

MoveIt's model of this robot has a fixed world -> base_footprint joint: the planning frame moves with the base. So the
objects of config/scene.yaml are given in base_footprint, from the robot's own pose estimate (map -> base_footprint
when Nav2 localises, else the spawn pose with odom -> base_footprint). When the base has moved, the objects are sent
again. This node is the one writer of /planning_scene besides move_group itself.

Topic /build1/planning_scene/exclude (std_msgs/String, comma separated names) removes objects, for example the item
before a grasp. Simulation only.
"""
import math

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_build1_scene import scene as scene_model


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PlanningSceneLoader(Node):
    def __init__(self):
        super().__init__("planning_scene_loader")
        self.scene = scene_model.load(self.declare_parameter("scene_file", "").value)
        self.spawn = self.declare_parameter("spawn_pose", [0.0, 0.0, 0.0]).value
        self.boxes = scene_model.collision_boxes(self.scene)
        self.excluded, self.sent_names = set(), set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.names_pub = self.create_publisher(String, "/build1/planning_scene/objects",
                                               QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(String, "/build1/planning_scene/exclude", self.on_exclude, 10)
        self.last_pose, self.last_sent = None, None
        self.create_timer(0.5, self.tick)

    def on_exclude(self, msg):
        self.excluded = {name.strip() for name in msg.data.split(",") if name.strip()}
        self.last_pose = None      # send again

    def base_pose(self):
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
        if not moved and self.last_sent is not None and (now - self.last_sent).nanoseconds < 5e9:
            return
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
