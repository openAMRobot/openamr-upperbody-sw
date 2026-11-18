# SPDX-License-Identifier: MIT
"""Scenario: the fine profile: single writers, then rates and missed steps.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn13_profile_fine*.json.
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
    return build1_harness.description(profile="fine", nav2="true", moveit="true", extra_sensors="false")


class TestScn13ProfileFine(unittest.TestCase):
    def test_scn13_profile_fine_a_single_writer(self):
        build1_harness.run(self, "scn09_single_writer", "scn13_profile_fine_single_writer")

    def test_scn13_profile_fine_b_report(self):
        build1_harness.run(self, "scn10_profile_report", "scn13_profile_fine_report")
