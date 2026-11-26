# SPDX-License-Identifier: MIT
"""Diagnostics: diag_reach_return, recorded with contact sensors on the arm links (BUILD1_CONTACT_SENSORS=1).

Headless, the whole launch, its own ROS domain. Numbers, trace and bag go to BUILD1_LOG_DIR/diag_reach_return*.
Run by .github/workflows/build1-arm-diagnostics.yml. Simulation evidence only. BLOCKED is not a pass.
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build1_harness  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return build1_harness.description(station="work_shelf", sensors="false", nav2="false", moveit="true")


class TestDiag(unittest.TestCase):
    def test_diag_reach_return(self):
        build1_harness.run(self, "diag_reach_return", "diag_reach_return")
