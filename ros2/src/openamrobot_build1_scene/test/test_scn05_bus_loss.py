# SPDX-License-Identifier: MIT
"""Scenario: bus loss on each of the five buses.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn05_bus_loss*.json.
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


class TestScn05BusLoss(unittest.TestCase):
    def test_scn05_bus_loss(self):
        build1_harness.run(self, "scn05_bus_loss", "scn05_bus_loss")
