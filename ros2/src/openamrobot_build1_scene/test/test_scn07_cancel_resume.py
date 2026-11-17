# SPDX-License-Identifier: MIT
"""Scenario: command cancellation, stop then resume, base and arm.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn07_cancel_resume*.json.
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
    return build1_harness.description(nav2="true", moveit="false", extra_sensors="false")


class TestScn07CancelResume(unittest.TestCase):
    def test_scn07_cancel_resume(self):
        build1_harness.run(self, "scn07_cancel_resume", "scn07_cancel_resume")
