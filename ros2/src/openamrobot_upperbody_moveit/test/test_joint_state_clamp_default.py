# SPDX-License-Identifier: MIT
"""Issue #18 (OA-65): the joint-state clamp's default follows the profile of the stack, not `true` for every profile.

The clamp exists for float noise of a SIMULATED joint resting on a limit (joint_state_clamp.py). On hardware a reading past a limit is a fact
about the robot and the clamp could hide a real limit violation, so `clamp_joint_states` defaults to `auto`: on for the simulated profiles
(mock, gazebo), off for anything else. An explicit `true` / `false` still wins. These tests fail on the old default (`true` for every profile):
the declared default, and what that default resolves to for a hardware profile. No ROS runtime: the launch file is read as source."""
import ast
import os

import pytest

from conftest import SHARE
from openamrobot_upperbody_moveit.joint_state_clamp import CLAMP_DEFAULT_ARGUMENT, CLAMP_SIMULATED_PROFILES, clamp_enabled

MOVE_GROUP_LAUNCH = os.path.join(SHARE, "launch", "move_group.launch.py")


def launch_tree():
    with open(MOVE_GROUP_LAUNCH) as f:
        return ast.parse(f.read())


def declared_default(name):
    """The default_value of DeclareLaunchArgument(name, ...) in move_group.launch.py, resolved through module-level string constants."""
    tree = launch_tree()
    constants = {t.id: n.value.value for n in tree.body if isinstance(n, ast.Assign) and isinstance(n.value, ast.Constant)
                 for t in n.targets if isinstance(t, ast.Name)}
    for call in ast.walk(tree):
        if isinstance(call, ast.Call) and getattr(call.func, "id", None) == "DeclareLaunchArgument" and call.args \
                and isinstance(call.args[0], ast.Constant) and call.args[0].value == name:
            value = next(k.value for k in call.keywords if k.arg == "default_value")
            return value.value if isinstance(value, ast.Constant) else constants[value.id]
    raise AssertionError(f"move_group.launch.py declares no {name} argument")


def test_the_declared_default_is_auto_not_true_for_every_profile():
    default = declared_default("clamp_joint_states")
    assert default != "true", "clamp_joint_states still defaults to true for every profile (issue #18)"
    assert default == CLAMP_DEFAULT_ARGUMENT == "auto"


@pytest.mark.parametrize("profile", ["real", "hardware", ""])
def test_the_default_is_off_on_hardware(profile):
    """With the old default (`true`, read as on whatever the profile) this is True for `real`: the clamp could hide a real limit violation."""
    assert clamp_enabled(declared_default("clamp_joint_states"), profile) is False


@pytest.mark.parametrize("profile", ["mock", "gazebo"])
def test_the_default_is_on_in_simulation(profile):
    assert profile in CLAMP_SIMULATED_PROFILES
    assert clamp_enabled(declared_default("clamp_joint_states"), profile) is True


@pytest.mark.parametrize("argument,profile,expected", [("true", "real", True), ("false", "gazebo", False), ("false", "mock", False),
                                                       ("true", "mock", True)])
def test_an_explicit_argument_wins(argument, profile, expected):
    assert clamp_enabled(argument, profile) is expected


def test_an_unknown_argument_is_refused_not_read_as_off():
    with pytest.raises(ValueError, match="clamp_joint_states:=yes"):
        clamp_enabled("yes", "gazebo")


def test_the_launch_resolves_the_argument_with_the_profile():
    """launch_setup reads the argument through clamp_enabled(argument, profile), not `== "true"`."""
    with open(MOVE_GROUP_LAUNCH) as f:
        text = f.read()
    assert "clamp_enabled(" in text and '.perform(context) == "true"' not in text.split("clamp =")[1].splitlines()[0]


def test_m07_still_reads_the_raw_arm_joint_states():
    """Issue #18's other condition: M07's evidence is the RAW /arm/joint_states, never the clamped topic move_group plans from."""
    m07 = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "openamrobot_upperbody_bringup", "test",
                       "test_m07_rest_position_margin.py")
    with open(m07) as f:
        text = f.read()
    assert 'create_subscription(JointState, "/arm/joint_states"' in text
    assert "joint_states_clamped" not in text
