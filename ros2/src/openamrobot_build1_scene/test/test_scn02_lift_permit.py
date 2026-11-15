# SPDX-License-Identifier: MIT
"""Scenario: lift at three heights, with and without permit.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn02_lift_permit*.json.
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
    return build1_harness.description(sensors="false", nav2="false", moveit="false")


class TestScn02LiftPermit(unittest.TestCase):
    def test_scn02_lift_permit(self):
        build1_harness.run(self, "scn02_lift_permit", "scn02_lift_permit")
