#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Publishes the Nav2 footprint that matches the arm pose.

Reads the base polygon from the Nav2 parameter file. Looks up the arm links in base_link on /tf and publishes the
convex hull of base and padded arm points on /local_costmap/footprint and /global_costmap/footprint. Nav2's costmaps
subscribe to these topics, and the collision monitor follows /local_costmap/published_footprint.
The lift changes no floor projection, so the lift height is not used. Simulation only. Not a safety function.
"""
import rclpy
import yaml
from geometry_msgs.msg import Point32, Polygon
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

from openamrobot_build1_scene import footprint as fp

ARM_LINKS = ["openarm_{side}_link2", "openarm_{side}_link3", "openarm_{side}_link4", "openarm_{side}_link5",
             "openarm_{side}_link6", "openarm_{side}_ee_base_link", "openarm_{side}_ee_link1", "openarm_{side}_ee_link2",
             "{side}_wrist_camera_link"]


class FootprintPublisher(Node):
    def __init__(self):
        super().__init__("footprint_publisher")
        with open(self.declare_parameter("nav2_params_file", "").value) as f:
            params = yaml.safe_load(f)
        self.base = fp.parse_polygon(params["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"])
        self.padding = self.declare_parameter("padding", 0.08).value         # m. Estimated: about one link radius
        self.threshold = self.declare_parameter("change_threshold", 0.01).value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.links = [name.format(side=side) for side in ("left", "right") for name in ARM_LINKS]
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publishers_ = [self.create_publisher(Polygon, topic, 1)
                            for topic in ("/local_costmap/footprint", "/global_costmap/footprint")]
        self.last, self.last_time = None, None
        self.create_timer(0.5, self.tick)

    def tick(self):
        points = []
        for link in self.links:
            try:
                t = self.buffer.lookup_transform(self.base_frame, link, Time()).transform.translation
            except TransformException:
                return          # no complete arm state yet: publish nothing rather than a footprint without the arms
            points.append((t.x, t.y))
        polygon = fp.footprint(self.base, points, self.padding)
        now = self.get_clock().now()
        stale = self.last_time is None or (now - self.last_time).nanoseconds > 5e9
        if not stale and not fp.changed(self.last, polygon, self.threshold):
            return
        message = Polygon(points=[Point32(x=float(x), y=float(y), z=0.0) for x, y in polygon])
        for publisher in self.publishers_:
            publisher.publish(message)
        if fp.changed(self.last, polygon, self.threshold):
            xs, ys = [p[0] for p in polygon], [p[1] for p in polygon]
            self.get_logger().info(f"footprint: {len(polygon)} points, x {min(xs):.3f} to {max(xs):.3f} m, "
                                   f"y {min(ys):.3f} to {max(ys):.3f} m")
        self.last, self.last_time = polygon, now


def main():
    rclpy.init()
    node = FootprintPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
