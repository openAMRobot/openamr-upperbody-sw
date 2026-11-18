#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Object pose estimator, milestone 1 (Proposed): the tote and the item from one depth camera, in base_footprint.

Subscribes the depth image and its camera info (default: the head camera of the Build 1 scene), takes the camera's
pose in `target_frame` from /tf (the robot's own joint states, no ground truth), and publishes
geometry_msgs/PoseWithCovarianceStamped on /perception/objects/<name>/pose (Proposed topic contract,
docs/architecture/object_perception.md), stamped with the image time. An object that is not found in an image is not
published for that image. A detection more than `max_jump_m` from the last published pose is held back until it repeats
`reacquire_count` times at one place (a hand near the item is not taken for the item). The method is in box_fit.py.

Reads no simulator ground truth and publishes no command. Simulation evidence only; not a safety function.
"""
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_object_perception import box_fit

TOPIC = "/perception/objects/{name}/pose"


def decode_depth(msg):
    """Depth in metres (rows x cols float32) of a 32FC1 or 16UC1 (millimetres) image."""
    if msg.encoding == "32FC1":
        dtype, scale = (">f4" if msg.is_bigendian else "<f4"), 1.0
    elif msg.encoding == "16UC1":
        dtype, scale = (">u2" if msg.is_bigendian else "<u2"), 0.001
    else:
        raise ValueError(f"depth encoding {msg.encoding} is not supported")
    item = np.dtype(dtype).itemsize
    raw = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.step // item)[:, :msg.width]
    depth = raw.astype(np.float32) * scale
    if msg.encoding == "16UC1":
        depth[raw == 0] = np.nan
    return depth


class Gate:
    """Holds back a detection that jumps from the last published pose until it repeats at one place."""

    def __init__(self, max_jump, reacquire_count, repeat_m=0.01):
        self.max_jump, self.reacquire_count, self.repeat_m = max_jump, reacquire_count, repeat_m
        self.last, self.pending, self.pending_count = None, None, 0

    def accept(self, position):
        if self.last is None or math.dist(position, self.last) <= self.max_jump:
            self.last, self.pending, self.pending_count = position, None, 0
            return True
        if self.pending is not None and math.dist(position, self.pending) <= self.repeat_m:
            self.pending_count += 1
        else:
            self.pending, self.pending_count = position, 1
        if self.pending_count >= self.reacquire_count:
            self.last, self.pending, self.pending_count = position, None, 0
            return True
        return False


class ObjectPoseEstimator(Node):
    def __init__(self):
        super().__init__("object_pose_estimator")
        p = self.declare_parameter
        self.target_frame = p("target_frame", "base_footprint").value
        self.camera_frame = p("camera_frame", "").value      # empty: the depth image's header frame
        self.period = 1.0 / float(p("rate_hz", 2.0).value)
        self.stride = int(p("stride", 1).value)
        self.max_range = float(p("max_range_m", 2.5).value)
        self.region = [float(v) for v in p("region", [0.45, 1.4, -0.9, 0.9]).value]
        self.support_z = float(p("support_z", 0.82).value)
        self.tote_size = [float(v) for v in p("tote_size", [0.40, 0.30, 0.06]).value]
        self.tote_wall = float(p("tote_wall", 0.005).value)
        self.item_size = [float(v) for v in p("item_size", [0.035, 0.035, 0.14]).value]
        self.objects = list(p("objects", ["tote", "item"]).value)
        jump, count = float(p("max_jump_m", 0.05).value), int(p("reacquire_count", 4).value)
        self.gates = {name: Gate(jump, count) for name in self.objects}
        self.publishers_ = {name: self.create_publisher(PoseWithCovarianceStamped, TOPIC.format(name=name), 10)
                            for name in self.objects}
        self.info = None
        self.last_run = None
        self.counts = {name: {"published": 0, "missed": 0, "held": 0, "reason": ""} for name in self.objects}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_subscription(CameraInfo, p("camera_info_topic", "/head_camera/rgbd/camera_info").value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, p("depth_topic", "/head_camera/rgbd/depth_image").value, self.on_depth,
                                 qos_profile_sensor_data)
        self.create_timer(10.0, self.report)
        self.get_logger().info(f"object pose estimator (Proposed): {self.objects} in {self.target_frame} on "
                               f"{TOPIC.format(name='<name>')}, support level {self.support_z} m. Simulation evidence only")

    def on_info(self, msg):
        self.info = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_depth(self, msg):
        if self.info is None:
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_run is not None and (stamp - self.last_run).nanoseconds * 1e-9 < self.period:
            return
        self.last_run = stamp
        camera_frame = self.camera_frame or msg.header.frame_id
        try:
            t = self.buffer.lookup_transform(self.target_frame, camera_frame, Time()).transform
        except TransformException as error:
            for name in self.objects:
                self.counts[name]["reason"] = f"no /tf {camera_frame} -> {self.target_frame}: {error}"
            return
        q, o = t.rotation, t.translation
        rotation = box_fit.quaternion_matrix((q.x, q.y, q.z, q.w))
        camera = (o.x, o.y, o.z)
        fx, fy, cx, cy = self.info
        points = box_fit.depth_points(decode_depth(msg), fx, fy, cx, cy, rotation, camera, self.stride, self.max_range)
        points = box_fit.in_region(points, self.region)
        for name in self.objects:
            if name == "tote":
                estimate, reason = box_fit.estimate_tote(points, self.tote_size, self.support_z, camera, fx)
            elif name == "item":
                rest = self.support_z + self.tote_wall
                estimate, reason = box_fit.estimate_item(points, self.item_size, rest,
                                                         self.support_z + self.tote_size[2] + 0.008, camera, fx)
            else:
                estimate, reason = None, "no method for this object"
            entry = self.counts[name]
            entry["reason"] = reason
            if estimate is None:
                entry["missed"] += 1
                continue
            if not self.gates[name].accept(estimate.position):
                entry["held"] += 1
                entry["reason"] = "held: jumped from the last published pose"
                continue
            self.publish(name, estimate, msg.header.stamp)
            entry["published"] += 1

    def publish(self, name, estimate, stamp):
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.target_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = estimate.position
        qx, qy, qz, qw = box_fit.yaw_quaternion(estimate.yaw)
        out.pose.pose.orientation.x, out.pose.pose.orientation.y = qx, qy
        out.pose.pose.orientation.z, out.pose.pose.orientation.w = qz, qw
        out.pose.covariance = estimate.covariance
        self.publishers_[name].publish(out)

    def report(self):
        self.get_logger().info("estimates: " + "; ".join(
            f"{name} published {c['published']}, missed {c['missed']}, held {c['held']} ({c['reason']})"
            for name, c in self.counts.items()))


def main():
    rclpy.init()
    node = ObjectPoseEstimator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
