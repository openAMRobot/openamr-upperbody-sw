# SPDX-License-Identifier: MIT
"""Scenario: grasp a tote item in the fast profile, no device, with Bullet Featherstone (the engine whose grippers move there).

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn15_grasp_fast*.json.
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
    return build1_harness.description(profile="fast", station="work_shelf", sensors="false", nav2="false", moveit="true",
                                      physics_engine="gz-physics-bullet-featherstone-plugin")


class TestScn15GraspFast(unittest.TestCase):
    def test_scn15_grasp_fast(self):
        build1_harness.run(self, "scn04_grasp", "scn15_grasp_fast")
