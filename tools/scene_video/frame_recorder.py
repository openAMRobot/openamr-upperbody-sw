#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Writes the observer camera images of the scene video as numbered JPEG frames, with the Gazebo stamp of each in
frames.csv. Visual capture, not a test result.

    frame_recorder.py OUT_DIR [--topic /observer/image]
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image


class Recorder(Node):
    def __init__(self, out_dir, topic):
        super().__init__("scene_video_recorder", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.out_dir = out_dir
        os.makedirs(out_dir, exist_ok=True)
        self.index = 0
        self.csv = open(os.path.join(out_dir, "frames.csv"), "w", encoding="utf-8")
        self.csv.write("index,stamp_s\n")
        self.create_subscription(Image, topic, self.on_image, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))

    def on_image(self, msg):
        channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
        array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * channels]
        array = array.reshape(msg.height, msg.width, channels)
        if msg.encoding.startswith("rgb"):
            array = cv2.cvtColor(array, cv2.COLOR_RGBA2BGR if channels == 4 else cv2.COLOR_RGB2BGR)
        cv2.imwrite(os.path.join(self.out_dir, f"{self.index:06d}.jpg"), array, [cv2.IMWRITE_JPEG_QUALITY, 92])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv.write(f"{self.index},{stamp:.4f}\n")
        self.csv.flush()
        if self.index % 75 == 0:
            self.get_logger().info(f"frame {self.index} at {stamp:.2f} s of Gazebo time")
        self.index += 1


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("out_dir")
    parser.add_argument("--topic", default="/observer/image")
    args = parser.parse_args(argv)
    rclpy.init()
    node = Recorder(args.out_dir, args.topic)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.csv.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
