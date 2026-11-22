# SPDX-License-Identifier: MIT
"""Scenario: drive 2 m and turn 90 degrees in the fast profile, no device.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn14_drive_fast*.json.
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
    return build1_harness.description(profile="fast", nav2="true", moveit="false", extra_sensors="false")


class TestScn14DriveFast(unittest.TestCase):
    def test_scn14_drive_fast(self):
        build1_harness.run(self, "scn01_drive", "scn14_drive_fast")
