# SPDX-License-Identifier: MIT
"""Scenario: drive 2 m and turn 90 degrees through Nav2, odometry against ground truth.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn01_drive*.json.
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
    return build1_harness.description(nav2="true", moveit="false", extra_sensors="false")


class TestScn01Drive(unittest.TestCase):
    def test_scn01_drive(self):
        build1_harness.run(self, "scn01_drive", "scn01_drive")
