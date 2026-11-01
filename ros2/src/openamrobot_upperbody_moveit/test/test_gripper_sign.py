# SPDX-License-Identifier: MIT
"""M05 (adopted 7 Oct, VERIFICATION.md): the two grippers take opposite-signed GripperCommand positions, and no named pose may close both
hands from one number (ADR-009 point 5), and the check fails the single-number version in both directions (the two `fails` tests below).

The defect this guards is silent: left finger_joint1 ranges [0, 0.7854] and right [-0.7854, 0], so "close" written as +0.7854 for both
hands is a valid number for one hand and out of range for the other, and nothing fails where it is written. The check reads the SRDF's
gripper states and the URDF's own limits, per hand, and `test_m05_the_single_number_version_fails` proves it can fail."""
import copy

from conftest import srdf_text, urdf_text, write_sidecar
from openamrobot_upperbody_moveit.named_poses import (GRIPPER_GROUP, GRIPPER_JOINT, SIDES, closed_value, gripper_sign_problems,
                                                      read_group_states, urdf_joint_limits)


def real():
    return read_group_states(srdf_text()), urdf_joint_limits(urdf_text())


def test_m05_the_srdf_gripper_states_are_per_hand_and_opposite_signed():
    states, limits = real()
    problems = gripper_sign_problems(states, limits)
    closed = {side: states[(GRIPPER_GROUP[side], "closed")][GRIPPER_JOINT[side]] for side in SIDES}
    opened = {side: states[(GRIPPER_GROUP[side], "open")][GRIPPER_JOINT[side]] for side in SIDES}
    write_sidecar("M05", "gripper_sign", "per-hand gripper values: closed is each hand's own far limit, opposite signs (ADR-009 point 5)",
                  "FAIL" if problems else "PASS",
                  {"problems": problems, "closed": closed, "open": opened,
                   "urdf_ranges": {side: list(limits[GRIPPER_JOINT[side]]) for side in SIDES},
                   "single_number_version_fails": True})
    assert not problems, problems
    assert closed["left"] > 0 > closed["right"] and opened == {"left": 0.0, "right": 0.0}


def test_m05_the_helper_derives_closed_from_each_hands_own_limit():
    _, limits = real()
    assert closed_value("left", limits) == 0.7854 and closed_value("right", limits) == -0.7854


def test_m05_the_single_number_version_fails():
    """Both hands closed with the left hand's number: valid for left, outside right's range. The check must say so, by hand."""
    states, limits = real()
    broken = copy.deepcopy(states)
    broken[("right_gripper", "closed")] = {GRIPPER_JOINT["right"]: states[("left_gripper", "closed")][GRIPPER_JOINT["left"]]}
    problems = gripper_sign_problems(broken, limits)
    assert any("right_gripper/closed" in p and "outside right's own range" in p for p in problems), problems
    assert any("opposite signs" in p for p in problems), problems
    assert not any(p.startswith("left_gripper") for p in problems), "left is still right"


def test_m05_the_other_single_number_version_fails_too():
    """The mirror image: both hands closed with the right hand's (negative) number."""
    states, limits = real()
    broken = copy.deepcopy(states)
    broken[("left_gripper", "closed")] = {GRIPPER_JOINT["left"]: states[("right_gripper", "closed")][GRIPPER_JOINT["right"]]}
    problems = gripper_sign_problems(broken, limits)
    assert any("left_gripper/closed" in p and "outside left's own range" in p for p in problems), problems


def test_m05_a_value_in_range_but_not_the_far_limit_is_not_a_closed_pose():
    states, limits = real()
    short = copy.deepcopy(states)
    short[("left_gripper", "closed")] = {GRIPPER_JOINT["left"]: 0.4}
    assert any("not left's far limit" in p for p in gripper_sign_problems(short, limits))


def test_m05_a_missing_or_extra_state_is_reported():
    states, limits = real()
    missing = {k: v for k, v in states.items() if k != ("right_gripper", "open")}
    assert any("right_gripper: no group_state `open`" in p for p in gripper_sign_problems(missing, limits))
    extra = copy.deepcopy(states)
    extra[("left_gripper", "open")] = {GRIPPER_JOINT["left"]: 0.0, "openarm_left_finger_joint2": 0.0}
    assert any("expected only" in p for p in gripper_sign_problems(extra, limits))
