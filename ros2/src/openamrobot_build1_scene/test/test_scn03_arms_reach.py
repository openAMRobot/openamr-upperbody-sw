# SPDX-License-Identifier: MIT
"""Scenario: both arms reach a shelf pose at two lift heights.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn03_arms_reach*.json.
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
    return build1_harness.description(station="work_shelf", sensors="false", nav2="false", moveit="true", driver_rate="100")


class TestScn03ArmsReach(unittest.TestCase):
    def test_scn03_arms_reach(self):
        build1_harness.run(self, "scn03_arms_reach", "scn03_arms_reach")
