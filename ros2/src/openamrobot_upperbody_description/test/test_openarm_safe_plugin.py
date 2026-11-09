# SPDX-License-Identifier: MIT
"""The real profile can use openamrobot_upperbody_hardware/OpenArmSafeHW instead of upstream's OpenArmHW. Expansion only.

real_arm_hardware:=upstream (the default) is what test_t16_ros2_control_real.py already checks. real_arm_hardware:=safe must
give upstream's blocks with exactly two differences: the plugin, and the added read_timeout_ms parameter.
"""
import re
import xml.etree.ElementTree as ET

import pytest
import xacro

from conftest import COMBINED, expand
from ros2_control_utils import arm_blocks, blocks, controlled_joints, hardware_params, lift_blocks, normalised, plugins, upstream_blocks

SAFE = "openamrobot_upperbody_hardware/OpenArmSafeHW"
UPSTREAM = "openarm_hardware/OpenArmHW"


def _safe(**kw):
    return expand(hardware_profile="real", real_arm_hardware="safe", **kw)


def test_default_real_profile_is_still_upstream():
    assert [p for _, p in plugins(expand(hardware_profile="real"))] == [UPSTREAM, UPSTREAM]


def test_safe_real_arms_use_the_safe_plugin():
    assert plugins(_safe()) == [("openarm_left_hardware_interface", SAFE), ("openarm_right_hardware_interface", SAFE)]


def test_safe_blocks_equal_upstream_except_plugin_and_read_timeout():
    def strip(root):
        out = []
        for text in normalised(root):
            text = text.replace(f"<plugin>{SAFE}</plugin>", f"<plugin>{UPSTREAM}</plugin>")
            text = re.sub(r'<paramname="read_timeout_ms">[^<]*</param>', "", text)
            # upstream's macro renders the Python bool as "True"; the plugin parses can_fd case-insensitively either way
            text = text.replace('name="can_fd">true<', 'name="can_fd">True<')
            out.append(text)
        return out
    assert strip(_safe()) == upstream_blocks(False)


def test_read_timeout_default_and_override():
    assert [hardware_params(b)["read_timeout_ms"] for b in arm_blocks(_safe())] == ["50", "50"]
    assert [hardware_params(b)["read_timeout_ms"] for b in arm_blocks(_safe(arm_read_timeout_ms=120))] == ["120", "120"]


def test_safe_controls_the_same_joints_as_upstream_real_and_no_lift():
    safe, upstream = _safe(), expand(hardware_profile="real")
    assert controlled_joints(safe) == controlled_joints(upstream)
    assert not lift_blocks(safe)


def test_safe_has_no_mock_or_gazebo_plugin():
    text = ET.tostring(_safe(), encoding="unicode")
    for forbidden in ("mock_components", "gz_ros2_control"):
        assert forbidden not in text


@pytest.mark.parametrize("profile", ["mock", "gazebo"])
def test_the_parameter_does_not_touch_simulated_profiles(profile):
    assert normalised(expand(hardware_profile=profile, real_arm_hardware="safe")) == normalised(expand(hardware_profile=profile))


def test_unknown_value_is_refused():
    with pytest.raises(xacro.XacroException):
        xacro.process_file(COMBINED, mappings={"hardware_profile": "real", "real_arm_hardware": "fast"})
