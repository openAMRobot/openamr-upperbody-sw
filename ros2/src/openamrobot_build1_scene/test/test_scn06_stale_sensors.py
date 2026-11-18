# SPDX-License-Identifier: MIT
"""Scenario: every sensor topic goes stale while Nav2 drives.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn06_stale_sensors*.json.
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
    return build1_harness.description(profile="faults", nav2="true", moveit="false", extra_sensors="false")


class TestScn06StaleSensors(unittest.TestCase):
    def test_scn06_stale_sensors(self):
        build1_harness.run(self, "scn06_stale_sensors", "scn06_stale_sensors")
