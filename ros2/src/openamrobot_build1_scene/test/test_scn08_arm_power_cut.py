# SPDX-License-Identifier: MIT
"""Scenario: arm motor power cut while holding a pose.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn08_arm_power_cut*.json.
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
    return build1_harness.description(profile="faults", sensors="false", nav2="false", moveit="false")


class TestScn08ArmPowerCut(unittest.TestCase):
    def test_scn08_arm_power_cut(self):
        build1_harness.run(self, "scn08_arm_power_cut", "scn08_arm_power_cut")
