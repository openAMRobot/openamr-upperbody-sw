# SPDX-License-Identifier: MIT
"""Scenario: base yaw against wheel kinematics, fast profile, DART, castors present, no device.

Equal wheel speeds, then equal and opposite ones in both directions. Ground truth yaw against r * (wR - wL) / track
from the wheel joint positions and against /odom. The track is read from the model and never changed.
Headless, the whole launch without Nav2 and MoveIt, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn17_base_yaw_fast.json.
Simulation evidence only. BLOCKED is not a pass.
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build1_harness  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return build1_harness.description(profile="fast", nav2="false", moveit="false", sensors="false", extra_sensors="false")


class TestScn17BaseYawFast(unittest.TestCase):
    def test_scn17_base_yaw_fast(self):
        build1_harness.run(self, "scn17_base_yaw", "scn17_base_yaw_fast")
