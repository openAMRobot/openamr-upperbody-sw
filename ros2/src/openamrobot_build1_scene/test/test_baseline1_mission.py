# SPDX-License-Identifier: MIT
"""Integrated Digital Twin Baseline 1: the whole-robot mission, firmware_sil, one launch, Nav2 and MoveIt 2.

spawn at origin, Nav2 to the work shelf, stop and authorize, lift, arm approach, grasp, lift the item, retreat
(openamrobot_build1_scene/baseline1.py). The phases, their checks and the trace go to
BUILD1_LOG_DIR/baseline1_mission.json; scripts/baseline1_report.py prints the phase table from it.
Python random (and with it the ROS domain of build1_harness) is seeded with BASELINE1_SEED.
Simulation evidence only. BLOCKED is not a pass.
"""
import os
import random
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build1_harness  # noqa: E402

random.seed(int(os.environ.get("BASELINE1_SEED", "20261117")))


@pytest.mark.launch_test
def generate_test_description():
    return build1_harness.description(profile="firmware_sil", station="origin", nav2="true", moveit="true")


class TestBaseline1Mission(unittest.TestCase):
    def test_baseline1_mission(self):
        from openamrobot_build1_scene import baseline1, scenarios
        scenarios.SCENARIOS.setdefault("baseline1_mission", (baseline1.mission, True, {"nav2": True, "moveit": True}))
        build1_harness.run(self, "baseline1_mission")
