# SPDX-License-Identifier: MIT
"""Scenario: scn04's grasp with object_pose_source estimate (Proposed): the tote and item poses of the planning scene and
the grasp target come from the head camera depth estimator (openamrobot_object_perception), not from Gazebo. Gazebo's
poses only measure the estimate error and the grasp outcome. Every scn04 check and limit is unchanged.

Headless, the whole launch with the rendered sensors, its own ROS domain. The numbers go to
BUILD1_LOG_DIR/scn04_grasp_estimate.json. Simulation evidence only. BLOCKED is not a pass.
"""
import os
import sys
import unittest

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import build1_harness  # noqa: E402


@pytest.mark.launch_test
def generate_test_description():
    return build1_harness.description(station="work_shelf", sensors="true", extra_sensors="true", nav2="false", moveit="true",
                                      object_pose_source="estimate")


class TestScn04GraspEstimate(unittest.TestCase):
    def test_scn04_grasp_estimate(self):
        build1_harness.run(self, "scn04_grasp", "scn04_grasp_estimate")
