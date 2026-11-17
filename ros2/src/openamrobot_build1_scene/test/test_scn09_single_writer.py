# SPDX-License-Identifier: MIT
"""Scenario: one writer per state and command topic, and the frame kinds on the buses.

Headless, the whole launch, its own ROS domain. The numbers go to BUILD1_LOG_DIR/scn09_single_writer*.json.
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
    return build1_harness.description(nav2="true", moveit="true")


class TestScn09SingleWriter(unittest.TestCase):
    def test_scn09_single_writer(self):
        build1_harness.run(self, "scn09_single_writer", "scn09_single_writer")
