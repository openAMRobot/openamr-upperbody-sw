# SPDX-License-Identifier: MIT
"""Scenario: the lift to every stop and back onto its lower limit through lift_controller, fast profile, DART, no device.

Each move is measured on /joint_states. The action must not report success when the joint is short of the stop.
Headless, the whole launch without Nav2 and MoveIt, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn16_lift_fast.json.
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


class TestScn16LiftFast(unittest.TestCase):
    def test_scn16_lift_fast(self):
        build1_harness.run(self, "scn16_lift_stops", "scn16_lift_fast")
