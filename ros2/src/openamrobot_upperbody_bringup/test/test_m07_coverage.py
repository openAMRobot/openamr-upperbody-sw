# SPDX-License-Identifier: MIT
"""M07 cannot pass on incomplete joint-state coverage (PR #14 review, Mohamed). No simulator, no ROS graph.

Before: the message count was global, `rows` only had to be non-empty and `rest_rows()` iterated the joints that happened to publish, so 50
messages carrying one safely placed joint passed even when another expected joint (the right finger, the joint this gate exists to catch) never
appeared. Pinned here, against the real model and against synthetic samples:
  - the expected set comes from the model: the limited joints under ros2_control, minus the explicit exclusions (lift_joint), with the mimic
    finger_joint2s reported as not measured instead of silently absent;
  - an expected joint that is missing, or read fewer than the per-joint minimum, is a coverage problem (and M07 FAILs on it).
"""
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a7_support  # noqa: E402

LIFT = {"lift_joint": "not under ros2_control"}
MIN = 50


@pytest.fixture(scope="module")
def model():
    import xacro
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    return xacro.process_file(path, mappings={"hardware_profile": "gazebo"}).toxml()


def test_the_expected_set_is_the_models_limited_ros2_control_joints(model):
    expected, also = a7_support.expected_rest_joints(model, LIFT)
    assert len(expected) == 16 and "lift_joint" not in expected
    assert {"openarm_left_finger_joint1", "openarm_right_finger_joint1", "openarm_left_joint4", "openarm_right_joint4"} <= set(expected)
    assert sorted(also) == ["openarm_left_finger_joint2", "openarm_right_finger_joint2"]


def test_a_missing_finger_is_a_coverage_problem(model):
    """Mohamed's reproduction, on the real expected set: every joint read 50 times except the right finger, which never appears."""
    expected, _ = a7_support.expected_rest_joints(model, LIFT)
    readings = {j: [0.1] * MIN for j in expected if j != "openarm_right_finger_joint1"}
    problems = a7_support.coverage_problems(readings, expected, MIN)
    assert problems["missing"] == ["openarm_right_finger_joint1"]


def test_one_joint_with_fifty_samples_does_not_cover_two():
    problems = a7_support.coverage_problems({"a": [0.0] * MIN}, ["a", "b"], MIN)
    assert problems == {"missing": ["b"], "too_few_samples": {}}


def test_a_joint_read_once_does_not_cover_the_window():
    problems = a7_support.coverage_problems({"a": [0.0] * MIN, "b": [0.0]}, ["a", "b"], MIN)
    assert problems == {"missing": [], "too_few_samples": {"b": 1}}


def test_complete_coverage_has_no_problem(model):
    expected, _ = a7_support.expected_rest_joints(model, LIFT)
    assert a7_support.coverage_problems({j: [0.0] * MIN for j in expected}, expected, MIN) == {"missing": [], "too_few_samples": {}}
