# SPDX-License-Identifier: MIT
"""Speed caps clamp down, keep the sign, record the clamp, and refuse non-finite or non-positive requests."""
import math

import pytest

import common


def test_base_twist_is_clamped_to_the_caps_with_sign_kept():
    log = common.CapLog()
    assert common.cap_base_twist(0.5, -2.0, log) == (0.2, -0.5)
    assert common.cap_base_twist(-0.3, 0.1, log) == (-0.2, 0.1)
    assert [e["what"] for e in log.events] == ["base linear.x m/s", "base angular.z rad/s", "base linear.x m/s"]


def test_requests_inside_the_caps_are_unchanged_and_not_logged():
    log = common.CapLog()
    assert common.cap_base_twist(0.2, 0.5, log) == (0.2, 0.5)
    assert common.cap_velocity_scaling(0.05, log) == 0.05
    assert common.cap_gripper_effort(5.0, log) == 5.0
    assert log.events == []


def test_scaling_and_effort_are_clamped():
    log = common.CapLog()
    assert common.cap_velocity_scaling(1.0, log) == 0.1
    assert common.cap_gripper_effort(40.0, log) == 5.0
    assert len(log.events) == 2


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_non_finite_requests_are_refused_not_clamped(bad):
    with pytest.raises(ValueError):
        common.cap_base_twist(bad, 0.0)
    with pytest.raises(ValueError):
        common.cap_gripper_effort(bad)


@pytest.mark.parametrize("bad", [0.0, -0.1])
def test_non_positive_scaling_and_effort_are_refused(bad):
    with pytest.raises(ValueError):
        common.cap_velocity_scaling(bad)
    with pytest.raises(ValueError):
        common.cap_gripper_effort(bad)


def test_caps_are_the_protocol_values():
    assert (common.BASE_MAX_LINEAR_MPS, common.BASE_MAX_ANGULAR_RADPS, common.ARM_MAX_VELOCITY_SCALING, common.GRIPPER_MAX_EFFORT) == (0.2, 0.5, 0.1, 5.0)


def test_segment_duration_uses_the_slowest_joint():
    assert common.segment_duration([0, 0], [1.5, 0.1], [3.0, 0.5], 0.1) == pytest.approx(5.0)


def test_robot_target_requires_the_robot_only_names_and_an_operator():
    p = common.base_parser("x", robot_required=[("--cmd-vel-topic", "t")])
    with pytest.raises(SystemExit):
        common.parse(p, ["--target", "robot"])
    with pytest.raises(SystemExit):
        common.parse(p, ["--target", "robot", "--cmd-vel-topic", "/x"])
    a = common.parse(p, ["--target", "robot", "--cmd-vel-topic", "/x", "--operator", "op"])
    assert a.cmd_vel_topic == "/x"
    assert common.parse(p, ["--target", "sim"]).cmd_vel_topic is None


def test_operator_must_type_yes_on_robot_only():
    class A:
        target, operator = "robot", "op"
    assert common.confirm(A, "move", ask=lambda _: "yes")
    with pytest.raises(SystemExit):
        common.confirm(A, "move", ask=lambda _: "y")
    A.target = "sim"
    assert common.confirm(A, "move", ask=lambda _: pytest.fail("sim must not ask"))


def test_floor_turn_angle():
    w = 0.58
    assert common.floor_turn_angle((0, 0), (0, -w), (0, 0), (w, 0)) == pytest.approx(math.pi / 2)
    assert abs(common.floor_turn_angle((0, 0), (0, -w), (1, 1), (1, 1 + w))) == pytest.approx(math.pi)


def test_gz_pose_parser_reads_pose_not_inertial_pose():
    text = ("  - Inertial Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [9 9 9]\n    [9 9 9]\n"
            "  - Pose [ XYZ (m) ] [ RPY (rad) ]:\n    [1.5 -0.25 0.1]\n    [0 0 1.2]\n")
    assert common.parse_gz_pose(text) == ([1.5, -0.25, 0.1], [0.0, 0.0, 1.2])


def test_gravity_torque_of_a_point_mass_on_a_horizontal_link():
    urdf = """<robot name="r"><link name="w"/><link name="a"><inertial><origin xyz="0.5 0 0"/><mass value="2"/>
      <inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial></link>
      <joint name="j" type="revolute"><parent link="w"/><child link="a"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>"""
    joints, links = common.load_urdf_joints(urdf)
    tau = common.gravity_torques(joints, links, {"j": 0.0}, "w", ["j"])["j"]
    # mass 2 kg at 0.5 m along x, axis +y: gravity's torque about +y is +2*9.80665*0.5, so the joint must supply minus that
    assert tau == pytest.approx(-2 * 9.80665 * 0.5)
