# SPDX-License-Identifier: MIT
"""Differential drive odometry from wheel speeds. No ROS import."""
import math

# CTL-017 reports the lift position from its lower end. The v1 object dictionary puts the L1000 stop 5 mm above it
# (0x2100:01 = 5000 um, as openamrobot_sil's ctl017_gazebo.py reads it). lift_joint 0 m is the L1000 stop.
LIFT_ZERO_MM = 5.0


class DiffDriveOdometry:
    def __init__(self, wheel_radius, wheel_separation):
        self.radius, self.separation = float(wheel_radius), float(wheel_separation)
        self.x = self.y = self.yaw = 0.0
        self.left_angle = self.right_angle = 0.0

    def wheel_speeds(self, v, w):
        """Wheel speeds (left, right) in rad/s of a body twist."""
        return ((v - w * self.separation / 2.0) / self.radius, (v + w * self.separation / 2.0) / self.radius)

    def twist(self, left, right):
        return self.radius * (left + right) / 2.0, self.radius * (right - left) / self.separation

    def step(self, left, right, dt):
        """Integrate wheel speeds (rad/s) over dt seconds, midpoint heading. -> the state as a dict."""
        v, w = self.twist(left, right)
        heading = self.yaw + w * dt / 2.0
        self.x += v * math.cos(heading) * dt
        self.y += v * math.sin(heading) * dt
        self.yaw = math.atan2(math.sin(self.yaw + w * dt), math.cos(self.yaw + w * dt))
        self.left_angle += left * dt
        self.right_angle += right * dt
        return {"x": self.x, "y": self.y, "yaw": self.yaw, "v": v, "w": w,
                "left_angle": self.left_angle, "right_angle": self.right_angle}
