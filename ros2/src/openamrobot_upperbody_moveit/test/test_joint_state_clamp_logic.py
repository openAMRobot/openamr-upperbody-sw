# SPDX-License-Identifier: MIT
"""The simulated start state's float noise, clamped (joint_state_clamp.py): the logic, in unit tests with no ROS runtime. The clamp in the loop
with a real move_group is test_joint_state_clamp_loop.py."""
import json
import math

import pytest

from openamrobot_upperbody_moveit import joint_state_clamp as jsc

LOWER, UPPER = 0.0, 2.4


# --- the logic ------------------------------------------------------------------------------------------------------------------------------
def test_a_reading_inside_the_limits_or_exactly_on_one_is_left_alone():
    for value in (LOWER, UPPER, 1.0, 1e-300):
        assert jsc.clamp_position(value, LOWER, UPPER) == (value, 0.0, "inside")


def test_float_noise_past_a_limit_is_clamped_TO_the_limit_and_never_past_it():
    for value, limit in ((-2.5e-7, LOWER), (-9e-8, LOWER), (-jsc.EPSILON, LOWER), (UPPER + 3e-7, UPPER), (UPPER + 0.99 * jsc.EPSILON, UPPER)):
        clamped, overshoot, kind = jsc.clamp_position(value, LOWER, UPPER)
        assert kind == "clamped" and clamped == limit and LOWER <= clamped <= UPPER
        assert overshoot == pytest.approx(abs(value - limit))


def test_a_reading_beyond_the_epsilon_is_not_float_noise_and_is_passed_through_unchanged():
    for value in (-jsc.EPSILON * 1.01, -1e-3, UPPER + 0.5, math.nan, math.inf, -math.inf):
        clamped, overshoot, kind = jsc.clamp_position(value, LOWER, UPPER)
        assert kind == "over_epsilon" and (clamped == value or math.isnan(value)) and overshoot > jsc.EPSILON


def test_the_epsilon_is_named_and_is_a_few_orders_below_anything_a_plan_can_see():
    assert jsc.EPSILON == 1e-5
    # Observed on Gazebo here: 2.5e-7, 8.0e-7 and 9.9e-7 rad (README). The epsilon clears the worst by a factor of ten.
    assert jsc.EPSILON >= 10 * 9.9e-7
    assert jsc.EPSILON <= 1e-4


def test_the_counters_separate_activations_from_episodes_and_keep_the_largest_magnitude():
    clamp = jsc.Clamp({"j4": (0.0, 2.4), "g": (0.0, 0.8)})
    names = ["j4", "g", "wheel"]
    out, events = clamp.process(names, [-2.5e-7, 0.0, 12.0])
    assert out == [0.0, 0.0, 12.0] and [e["new_episode"] for e in events] == [True]
    out, events = clamp.process(names, [-9e-8, 0.0, 12.0])
    assert out[0] == 0.0 and not events[0]["new_episode"], "the same joint, the next message: the same episode"
    clamp.process(names, [0.3, 0.0, 12.0])                     # back inside: the episode ends
    clamp.process(names, [-4e-7, 0.0, 12.0])                   # a second episode
    s = clamp.summary()
    assert (s["messages"], s["activations"], s["episodes"], s["over_epsilon"]) == (4, 3, 2, 0)
    assert s["max_magnitude_rad"] == pytest.approx(4e-7) and s["max_magnitude_joint"] == "j4"
    assert s["joints"]["j4"] == {"activations": 3, "episodes": 2, "max_magnitude": pytest.approx(4e-7), "over_epsilon": 0}
    out, events = clamp.process(names, [-1e-3, 0.0, 12.0])
    assert out[0] == -1e-3 and events[0]["kind"] == "over_epsilon"
    s = clamp.summary()
    assert s["over_epsilon"] == 1 and s["max_over_epsilon_rad"] == pytest.approx(1e-3) and s["activations"] == 3, "not counted as a clamp"
    assert json.loads(clamp.summary_json()) == json.loads(json.dumps(s))


def test_a_joint_with_no_limits_and_a_short_position_list_pass_through():
    clamp = jsc.Clamp({"j": (0.0, 1.0)})
    assert clamp.process(["wheel", "j"], [5.0]) == ([5.0], [])
    assert clamp.process(["wheel"], [-7.0]) == ([-7.0], [])


def test_the_limits_are_the_urdfs_and_nothing_else():
    from conftest import urdf_root
    root = urdf_root()
    limits = jsc.limits_from_urdf(root)
    assert limits["openarm_left_joint4"][0] == 0.0, "home's joint 4 sits exactly on this limit"
    arm = [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)]
    assert set(arm) <= set(limits)
    assert not {"left_wheel_joint", "right_wheel_joint"} & set(limits), "continuous joints have no limits to clamp to"
    for joint in root.findall("joint"):
        if joint.get("name") in limits:
            limit = joint.find("limit")
            assert limits[joint.get("name")] == (float(limit.get("lower")), float(limit.get("upper")))


def test_home_and_the_other_named_poses_are_untouched_by_the_clamp():
    """The clamp is not a way to move a pose: every named pose value is inside the limits, so none of them is ever altered."""
    from conftest import srdf_text, urdf_root
    from openamrobot_upperbody_moveit.named_poses import read_group_states
    limits = jsc.limits_from_urdf(urdf_root())
    for (group, pose), joints in read_group_states(srdf_text()).items():
        for joint, value in joints.items():
            assert jsc.clamp_position(value, *limits[joint]) == (value, 0.0, "inside"), (group, pose, joint, value)
