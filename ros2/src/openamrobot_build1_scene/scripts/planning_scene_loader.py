#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Puts the scene's collision objects into the MoveIt planning scene, and keeps them where the robot believes they are.

MoveIt's model of this robot has a fixed world -> base_footprint joint: the planning frame moves with the base. So the
objects of config/scene.yaml are given in base_footprint, from the robot's own pose estimate (map -> base_footprint
when Nav2 localises, else the spawn pose with odom -> base_footprint). When the base has moved, the objects are sent
again. This node is the one writer of /planning_scene besides move_group itself.

The robot's pose for that comes from its own estimate by default (pose_source map, planning_scene.py); pose_source
ground_truth (a simulation oracle) takes /build1/ground_truth/odom instead and must be asked for explicitly. The tote,
the item and the payload follow their observed poses from the declared object_pose_source in both cases. Topic /build1/planning_scene/exclude (std_msgs/String, comma separated names) removes objects, for example the item
before a grasp. The fixed boxes get the planning clearance fixture_padding_m (planning_scene.FIXTURE_PADDING_M) on every
side; the tote, the item and the payload get none. Simulation only.
"""
import math
import os

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import CollisionObject, PlanningScene
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_build1_scene import scene as scene_model
from openamrobot_build1_scene.planning_scene import (DEFAULT_POSE_SOURCE, FIXTURE_PADDING_M, MOVABLE_OBJECTS,
                                                     DEFAULT_OBJECT_POSE_SOURCE, choose_base_pose, declaration, object_box_in_base,
                                                     object_declaration, object_moved, object_pose_topic, padded)


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
        # Movable objects follow their observed poses (planning_scene.MOVABLE_OBJECTS); until one is seen, its start pose.
        movable = scene_model.movable_objects(self.scene)
        self.object_offset = {name: (movable[name]["size"][2] / 2.0 if movable[name]["kind"] == "tote" else 0.0)
                              for name in MOVABLE_OBJECTS if name in movable}
        self.object_pose, self.object_sent = {}, {}
        self.object_pose_source = self.declare_parameter("object_pose_source", DEFAULT_OBJECT_POSE_SOURCE).value
        self.get_logger().info(object_declaration(self.object_pose_source))
        for name in self.object_offset:
            self.create_subscription(PoseStamped, object_pose_topic(self.object_pose_source, name),
                                     lambda m, n=name: self.on_object(n, m), 10)
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

    def on_object(self, name, msg):
        p, q = msg.pose.position, msg.pose.orientation
        self.object_pose[name] = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))

    def on_truth(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.truth = (p.x, p.y, yaw_of(q))
        self.truth3 = ((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))     # scratch (diag/arm-geom-*): with the base's z and tilt

    def tilt_corrected(self, centre, quaternion):
        """Scratch: a box pose in the level base_footprint (z of the world) moved into the tilted base_footprint of the
        ground truth odom: the base's height and roll and pitch, which the level placement leaves out."""
        import numpy as np
        (_, _, bz), (x, y, z, w) = self.truth3
        R3 = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                       [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                       [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        yaw = yaw_of(type("Q", (), {"x": x, "y": y, "z": z, "w": w})())
        c, s_ = math.cos(yaw), math.sin(yaw)
        R2 = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        Rt = R2.T @ R3                                                   # roll and pitch in the base's yaw frame
        out = Rt.T @ (np.array(centre) - np.array((0.0, 0.0, bz)))
        qw = math.sqrt(max(0.0, 1.0 + np.trace(Rt))) / 2.0
        qt = ((Rt[2, 1] - Rt[1, 2]) / (4 * qw), (Rt[0, 2] - Rt[2, 0]) / (4 * qw), (Rt[1, 0] - Rt[0, 1]) / (4 * qw), qw)
        inv = (-qt[0], -qt[1], -qt[2], qt[3])
        ax, ay, az, aw = inv
        bx, by, bz2, bw = quaternion
        q = (aw * bx + ax * bw + ay * bz2 - az * by, aw * by - ax * bz2 + ay * bw + az * bx,
             aw * bz2 + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz2)
        return tuple(float(v) for v in out), q

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
        moved = moved or any(object_moved(self.object_sent.get(name), pose_now)
                             for name, pose_now in self.object_pose.items())
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
            if name in self.object_pose:
                position, quaternion = self.object_pose[name]
                centre_b, q_b = object_box_in_base(pose, position, quaternion, self.object_offset[name])
                box_pose.position.x, box_pose.position.y, box_pose.position.z = centre_b
                box_pose.orientation.x, box_pose.orientation.y, box_pose.orientation.z, box_pose.orientation.w = q_b
            else:
                box_pose.position.x, box_pose.position.y, box_pose.position.z = c * dx - s * dy, s * dx + c * dy, centre[2]
                relative = yaw - pose[2]
                box_pose.orientation.z, box_pose.orientation.w = math.sin(relative / 2.0), math.cos(relative / 2.0)
            if os.environ.get("BUILD1_DIAG_PLAN_TILT") and getattr(self, "truth3", None) is not None:      # scratch
                p_, o_ = box_pose.position, box_pose.orientation
                (p_.x, p_.y, p_.z), (o_.x, o_.y, o_.z, o_.w) = self.tilt_corrected((p_.x, p_.y, p_.z), (o_.x, o_.y, o_.z, o_.w))
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
        self.object_sent = dict(self.object_pose)


def main():
    rclpy.init()
    node = PlanningSceneLoader()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
