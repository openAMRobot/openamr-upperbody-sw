# SPDX-License-Identifier: MIT
"""Scenario: grasp a tote item with the corrected open and closed states.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn04_grasp*.json.
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
    return build1_harness.description(station="work_shelf", sensors="false", nav2="false", moveit="true")


class TestScn04Grasp(unittest.TestCase):
    def test_scn04_grasp(self):
        build1_harness.run(self, "scn04_grasp", "scn04_grasp")
