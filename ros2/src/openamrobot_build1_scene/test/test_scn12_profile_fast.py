# SPDX-License-Identifier: MIT
"""Scenario: the fast profile: single writers, then rates and missed steps.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn12_profile_fast*.json.
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
    return build1_harness.description(profile="fast", nav2="true", moveit="true", extra_sensors="false")


class TestScn12ProfileFast(unittest.TestCase):
    def test_scn12_profile_fast_a_single_writer(self):
        build1_harness.run(self, "scn09_single_writer", "scn12_profile_fast_single_writer")

    def test_scn12_profile_fast_b_report(self):
        build1_harness.run(self, "scn10_profile_report", "scn12_profile_fast_report")
