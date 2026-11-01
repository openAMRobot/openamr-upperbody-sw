# SPDX-License-Identifier: MIT
"""T14: the `mock` profile drives the arms with mock_components/GenericSystem, and the base has no ros2_control.

The two arm blocks must also equal, whitespace aside, what upstream's own top-level v2.0 file expands to with
use_fake_hardware:=true: "exactly as upstream does" is asserted, not assumed.
"""
import pytest

from conftest import ARM_PACKAGES, BASE_PLATES, HARDWARE_PROFILES, expand
from ros2_control_utils import (BASE_JOINT_HINTS, arm_joints_needing_an_interface, blocks, controlled_joints, element_text,
                                normalised, plugins, upstream_blocks)

MOCK = "mock_components/GenericSystem"
FORBIDDEN = ("openarm_hardware/OpenArmHW", "gz_ros2_control")


def test_t14_mock_arms_use_generic_system():
    assert plugins(expand(hardware_profile="mock")) == [("openarm_left_hardware_interface", MOCK),
                                                         ("openarm_right_hardware_interface", MOCK)]


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_t14_mock_has_no_hardware_or_gazebo_plugin_for_any_base_plate(base_plate):
    text = element_text(expand(hardware_profile="mock", base_plate=base_plate))
    for forbidden in FORBIDDEN:
        assert forbidden not in text, f"{forbidden} appears in the mock expansion"


def test_t14_the_base_has_no_ros2_control():
    root = expand(hardware_profile="mock")
    joints = controlled_joints(root)
    assert joints, "no ros2_control joints at all"
    assert all(name.startswith("openarm_") for name in joints), sorted(joints)
    assert not [n for n in joints if any(h in n for h in BASE_JOINT_HINTS)]
    # the base's own joints exist in the model and none of them is controlled
    base = {j.get("name") for j in root.findall("joint") if any(h in j.get("name") for h in BASE_JOINT_HINTS)}
    assert base, "the base wheel and caster joints are missing from the expansion: this check would be vacuous"
    assert not base & set(joints)


def test_t14_every_movable_arm_joint_is_controlled_and_nothing_else():
    root = expand(hardware_profile="mock")
    assert sorted(controlled_joints(root)) == arm_joints_needing_an_interface(root)


def test_t14_joint_trajectory_controller_interfaces_are_exported():
    """The arm controllers command position and read position and velocity; the grippers command and read position."""
    for name, interfaces in controlled_joints(expand(hardware_profile="mock")).items():
        assert {"position", "velocity"} <= set(interfaces["state"]), name
        assert "position" in interfaces["command"], name


def test_t14_arm_blocks_equal_upstream_with_use_fake_hardware_true():
    assert normalised(expand(hardware_profile="mock")) == upstream_blocks(True)


@pytest.mark.parametrize("arm_package", [a for a in ARM_PACKAGES if a != "openarm"])
@pytest.mark.parametrize("hardware_profile", HARDWARE_PROFILES)
def test_t14_no_arm_package_no_ros2_control(arm_package, hardware_profile):
    assert not blocks(expand(arm_package=arm_package, hardware_profile=hardware_profile))
