# SPDX-License-Identifier: MIT
"""Scenario: the Stop gate process ended (SIGKILL) during a Nav2 drive.

Headless, the whole launch without MoveIt, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn19_stop_gate_kill*.json.
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


class TestScn19StopGateKill(unittest.TestCase):
    def test_scn19_stop_gate_kill(self):
        build1_harness.run(self, "scn19_stop_gate_kill", "scn19_stop_gate_kill")
