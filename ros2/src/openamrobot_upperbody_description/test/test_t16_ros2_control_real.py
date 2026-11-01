# SPDX-License-Identifier: MIT
"""T16: the `real` profile expands with openarm_hardware/OpenArmHW and its CAN parameters. Expansion only.

Nothing here loads the plugin, opens a CAN interface or launches anything: `openarm_hardware` cannot even be built in
this workspace (it needs OpenArmCAN). The values are checked against the upstream gains file the macro reads, and the
two blocks against upstream's own top-level file with use_fake_hardware:=false.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from conftest import expand
from ros2_control_utils import (arm_joints_needing_an_interface, blocks, controlled_joints, element_text, hardware_params,
                                normalised, plugins, upstream_blocks)

HW = "openarm_hardware/OpenArmHW"


def _gains():
    path = os.path.join(get_package_share_directory("openarm_description"), "assets", "robot", "openarm_v2.0", "config",
                        "arm", "control", "control_gains.yaml")
    with open(path) as f:
        return yaml.safe_load(f)


def test_t16_real_arms_use_openarm_hw():
    assert plugins(expand(hardware_profile="real")) == [("openarm_left_hardware_interface", HW),
                                                         ("openarm_right_hardware_interface", HW)]


def test_t16_real_has_no_mock_or_gazebo_plugin():
    text = element_text(expand(hardware_profile="real"))
    for forbidden in ("mock_components", "gz_ros2_control"):
        assert forbidden not in text, f"{forbidden} appears in the real expansion"


def test_t16_can_parameters_per_arm():
    left, right = (hardware_params(b) for b in blocks(expand(hardware_profile="real")))
    # upstream's defaults: its own top-level v2.0 file does not pass the interfaces
    assert (left["can_interface"], right["can_interface"]) == ("can1", "can0")
    assert (left["arm_prefix"], right["arm_prefix"]) == ("left_", "right_")
    for params in (left, right):
        assert params["arm_type"] == "v20" and params["ee_type"] == "pinch_gripper"
        assert params["can_fd"].lower() == "true"


def test_t16_gains_come_from_the_upstream_gains_file():
    gains = _gains()
    for block in blocks(expand(hardware_profile="real")):
        params = hardware_params(block)
        for n in range(1, 8):
            assert float(params[f"kp{n}"]) == float(gains[f"joint{n}"]["kp"])
            assert float(params[f"kd{n}"]) == float(gains[f"joint{n}"]["kd"])


def test_t16_real_controls_the_same_joints_as_mock():
    real, mock = expand(hardware_profile="real"), expand(hardware_profile="mock")
    assert controlled_joints(real) == controlled_joints(mock)
    assert sorted(controlled_joints(real)) == arm_joints_needing_an_interface(real)


def test_t16_arm_blocks_equal_upstream_with_use_fake_hardware_false():
    assert normalised(expand(hardware_profile="real")) == upstream_blocks(False)
