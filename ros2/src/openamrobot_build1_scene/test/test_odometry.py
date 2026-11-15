# SPDX-License-Identifier: MIT
"""Odometry from wheel speeds (SW-08) and the per-wheel command (SW-03)."""
import math

import pytest

from openamrobot_build1_scene.odometry import DiffDriveOdometry, LIFT_ZERO_MM

R, SEPARATION = 0.100, 0.400      # openamrobot_sil config/sil.yaml


def test_straight_line():
    odometry = DiffDriveOdometry(R, SEPARATION)
    for _ in range(500):
        state = odometry.step(2.0, 2.0, 0.02)
    assert state["x"] == pytest.approx(2.0) and state["y"] == pytest.approx(0.0) and state["yaw"] == 0.0
    assert state["v"] == pytest.approx(0.2) and state["left_angle"] == pytest.approx(20.0)


def test_turn_in_place_by_90_degrees():
    odometry = DiffDriveOdometry(R, SEPARATION)
    left, right = odometry.wheel_speeds(0.0, 0.5)
    for _ in range(int(round(math.pi / 2 / 0.5 / 0.01))):
        state = odometry.step(left, right, 0.01)
    assert state["yaw"] == pytest.approx(math.pi / 2, abs=0.006)
    assert math.hypot(state["x"], state["y"]) < 1e-9


def test_the_result_depends_on_gazebo_time_only():
    """The same wheel speeds over the same Gazebo time give the same pose, however the time is cut into samples.
    A slow machine delivers fewer, longer samples."""
    fine, coarse = DiffDriveOdometry(R, SEPARATION), DiffDriveOdometry(R, SEPARATION)
    for _ in range(1000):
        fine.step(1.9, 2.1, 0.005)
    for _ in range(50):
        coarse.step(1.9, 2.1, 0.1)
    assert coarse.x == pytest.approx(fine.x, abs=2e-3) and coarse.y == pytest.approx(fine.y, abs=2e-3)
    assert coarse.yaw == pytest.approx(fine.yaw, abs=1e-9)


def test_sw03_unfolding_the_twist_gives_the_wheel_speeds_back_exactly():
    """wheel_command_adapter.py inverts the twist the drive emulator node folds, with the same radius and separation."""
    model = DiffDriveOdometry(R, SEPARATION)
    for left, right in ((1.99, 1.99), (-1.0, 1.0), (0.0, 3.7), (2.5, -0.3), (0.0, 0.0)):
        v, w = model.twist(left, right)                        # what openamrobot_sil's wheel_speeds_to_twist does
        assert v == pytest.approx(R * (left + right) / 2.0) and w == pytest.approx(R * (right - left) / SEPARATION)
        back = model.wheel_speeds(v, w)
        assert back[0] == pytest.approx(left, abs=1e-12) and back[1] == pytest.approx(right, abs=1e-12)


def test_wheel_command_adapter_adds_no_limit_and_no_ramp():
    from conftest import package_path
    with open(package_path("scripts", "wheel_command_adapter.py"), encoding="utf-8") as f:
        text = f.read()
    code = text.split('"""', 2)[2]
    for word in ("max_", "min(", "max(", "clamp", "create_timer", "acceleration"):
        assert word not in code, f"the adapter must only unfold the twist, found {word!r}"
    assert "/sil/drive/gz_cmd_vel" in code and "left_wheel_cmd_vel" in code and "right_wheel_cmd_vel" in code


def test_lift_zero_is_the_l1000_stop():
    assert LIFT_ZERO_MM == 5.0
    assert (180.0 - LIFT_ZERO_MM) / 1000.0 == pytest.approx(0.175) and (355.0 - LIFT_ZERO_MM) / 1000.0 == pytest.approx(0.350)


def test_base_state_adapter_reads_no_gazebo_topic():
    from conftest import package_path
    with open(package_path("scripts", "base_state_adapter.py"), encoding="utf-8") as f:
        code = f.read().split('"""', 2)[2]
    assert "/sil/gz" not in code and "ground_truth" not in code, "state must come over a bus, not from the plant"
    assert '"/odom"' in code and '"/base/joint_states"' in code
