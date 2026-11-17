# SPDX-License-Identifier: MIT
"""Scenario: real-time factor, controller rates and missed steps of firmware_sil at 1/1500 s.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn11_profile_sil_fine*.json.
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
    return build1_harness.description(step="fine", nav2="true", moveit="true", extra_sensors="false", sim_state_rate="step")


class TestScn11ProfileSilFine(unittest.TestCase):
    def test_scn11_profile_sil_fine(self):
        build1_harness.run(self, "scn10_profile_report", "scn11_profile_sil_fine")
