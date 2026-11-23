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


# Wheel speed source of base_state_adapter.py. PROPOSED (23 Nov 2026, upperbody #137): on 22 and 23 Nov 2026 two scenario
# runs (scene 45075524313 scn18, scene 45093762359 scn01) had no wheel speed sample from the start. The adapter integrated
# zero speed and published an odometry of (0, 0) that looked valid; Nav2 drove the base 5.8 m and 3.6 m against a pose
# that never changed. Without a first sample there is no odometry to publish.
NO_SAMPLE, STALE, FRESH = "no sample", "stale", "fresh"
NO_SAMPLE_REPORT_S = 5.0        # Gazebo seconds without a first sample before the adapter reports it (once)


def wheel_sample_state(age_s, stale_s):
    """`age_s`: Gazebo seconds since the last wheel speed sample, None when none has arrived yet.
    NO_SAMPLE: publish nothing. STALE: integrate zero speed and say so. FRESH: integrate the sample."""
    if age_s is None:
        return NO_SAMPLE
    return STALE if age_s > stale_s else FRESH

