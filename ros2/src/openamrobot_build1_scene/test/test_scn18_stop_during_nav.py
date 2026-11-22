# SPDX-License-Identifier: MIT
"""Scenario: software Stop engaged during a Nav2 drive, held, released, drive finished.

Headless, the whole launch without MoveIt, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn18_stop_during_nav*.json.
Simulation evidence only: the software Stop is a functional stop, not the hardwired E-stop. BLOCKED is not a pass.
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


class TestScn18StopDuringNav(unittest.TestCase):
    def test_scn18_stop_during_nav(self):
        build1_harness.run(self, "scn18_stop_during_nav", "scn18_stop_during_nav")
