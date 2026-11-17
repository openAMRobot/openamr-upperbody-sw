# SPDX-License-Identifier: MIT
"""The side grasp turned to the item's face (arm_frames.py) and its use in scn04's grasp. No ROS import."""
import math
import os

import numpy as np
import pytest

from openamrobot_build1_scene import arm_frames as af

HERE = os.path.dirname(os.path.abspath(__file__))


def matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_no_turn_is_the_old_side_grasp():
    assert np.allclose(af.side_grasp(0.0), af.SIDE_GRASP)
    assert np.allclose(af.side_grasp_offset((0.001, 0.0, -0.138)), (0.138, 0.0, 0.001))
    assert np.allclose(af.hand_for_tcp_at((0.75, 0.16, 0.925), (0.0, 0.0, -0.138)), (0.612, 0.16, 0.925))


@pytest.mark.parametrize("yaw", [-0.3, -0.073, 0.05, 0.2])
def test_the_turned_grasp_puts_the_tcp_on_the_point_and_the_fingers_along_the_yaw(yaw):
    q = af.side_grasp(yaw)
    assert math.isclose(sum(c * c for c in q), 1.0, abs_tol=1e-12)
    offset = np.array([-0.0014, 0.0, -0.138])
    point = np.array([0.75, 0.16, 0.925])
    hand = np.array(af.hand_for_tcp_at(point, offset, yaw))
    assert np.allclose(hand + matrix(*q) @ offset, point, atol=1e-12)
    fingers = matrix(*q) @ np.array([0.0, 0.0, -1.0])
    assert np.allclose(fingers, (math.cos(yaw), math.sin(yaw), 0.0), atol=1e-12)


def test_the_face_yaw_is_the_nearest_face_within_a_quarter_turn():
    station = -math.pi / 2.0
    assert math.isclose(af.face_yaw(station, 0.0), 0.0, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(station + 0.073, 0.0), -0.073, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(station, 0.1), 0.1, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(0.0, math.pi / 2.0 + 0.02), 0.02, abs_tol=1e-12)
    for item in np.linspace(-4, 4, 41):
        assert -math.pi / 4.0 <= af.face_yaw(0.3, item) < math.pi / 4.0


def test_scenario_4_turns_the_grasp_and_the_approach_to_the_items_face():
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "scenarios.py"), encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("def grasp(rig, log):"):text.index("def _held_payload(")]
    assert "quaternion = side_grasp(yaw)" in body and "quaternion = SIDE_GRASP" not in body
    assert "hand_for_tcp_at((item[0], item[1], grasp_z), found[0], yaw)" in body
    assert "rotate_z((0.10, 0.0, 0.0), yaw)" in body
