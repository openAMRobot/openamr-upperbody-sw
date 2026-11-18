# SPDX-License-Identifier: MIT
"""Grocery dark store demonstrator: the whole-robot pick-and-place mission, firmware_sil, one launch, Nav2 and MoveIt 2,
in the grocery scene (scene:=grocery_scene.yaml).

park > aisle shelf > stop and authorize > lift > approach, grasp, lift the product > back up with it > packing bench >
place it in the order tote > retreat > back to parking (openamrobot_build1_scene/grocery_demo.py). The phases, their
checks and the trace go to BUILD1_LOG_DIR/grocery_demo_mission.json and grocery_demo_report.json;
scripts/grocery_demo_report.py prints the phase table. Profile sim-oracle (pose_source: ground_truth).
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
    # Profile sim-oracle, declared explicitly: the planning scene is placed from the simulator's ground truth.
    # Station alignment (openamr-platform-sw #167) stays off until it is adopted into Baseline 1; GROCERY_STATION_ALIGNMENT=1
    # switches it on for the work shelf (the launch argument station_alignment, as in Baseline 1).
    alignment = "true" if os.environ.get("GROCERY_STATION_ALIGNMENT", "0") == "1" else "false"
    return build1_harness.description(profile="firmware_sil", station="origin", nav2="true", moveit="true",
                                      station_alignment=alignment, scene="grocery_scene.yaml", planning_pose_source="ground_truth")


class TestGroceryDemoMission(unittest.TestCase):
    def test_grocery_demo_mission(self):
        from openamrobot_build1_scene import grocery_demo, scenarios
        scenarios.SCENARIOS.setdefault("grocery_demo_mission", (grocery_demo.mission, True, {"nav2": True, "moveit": True}))
        build1_harness.run(self, "grocery_demo_mission")
