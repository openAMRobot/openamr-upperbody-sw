"""arm_controllers.yaml names only joints the arm <ros2_control> blocks declare, for every hardware profile.

Static check on the expansion (no controller_manager); the launch test L01 in openamrobot_upperbody_bringup is the
runtime counterpart.
"""
import os
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

CONTROL = get_package_share_directory("openamrobot_upperbody_control")
COMBINED = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
ARM_CONTROLLERS = ("left_arm_controller", "right_arm_controller")
GRIPPER_CONTROLLERS = ("left_gripper_controller", "right_gripper_controller")


def _config():
    with open(os.path.join(CONTROL, "config", "arm_controllers.yaml")) as f:
        return yaml.safe_load(f)


def _expansion(profile):
    return ET.fromstring(xacro.process_file(COMBINED, mappings={"hardware_profile": profile}).toxml())


def _declared(root):
    """{joint: ([command interfaces], [state interfaces])} over the ros2_control blocks."""
    return {j.get("name"): ([c.get("name") for c in j.findall("command_interface")],
                            [s.get("name") for s in j.findall("state_interface")])
            for b in root.findall("ros2_control") for j in b.findall("joint")}


def test_controller_manager_lists_all_five_controllers_at_100_hz():
    cm = _config()["controller_manager"]["ros__parameters"]
    assert cm["update_rate"] == 100
    assert sorted(k for k, v in cm.items() if isinstance(v, dict)) == sorted(
        ("joint_state_broadcaster",) + ARM_CONTROLLERS + GRIPPER_CONTROLLERS)
    for name in ARM_CONTROLLERS:
        assert cm[name]["type"] == "joint_trajectory_controller/JointTrajectoryController"
    for name in GRIPPER_CONTROLLERS:
        assert cm[name]["type"] == "position_controllers/GripperActionController"


def test_every_controller_has_a_parameter_block():
    config = _config()
    for name in ARM_CONTROLLERS + GRIPPER_CONTROLLERS:
        assert "ros__parameters" in config[name], name


@pytest.mark.parametrize("profile", ["mock", "gazebo", "real"])
def test_controller_joints_are_declared_with_the_interfaces_they_need(profile):
    config, declared = _config(), _declared(_expansion(profile))
    for name in ARM_CONTROLLERS:
        params = config[name]["ros__parameters"]
        assert len(params["joints"]) == 7
        for joint in params["joints"]:
            commands, states = declared[joint]
            assert set(params["command_interfaces"]) <= set(commands), (name, joint)
            assert set(params["state_interfaces"]) <= set(states), (name, joint)
    for name in GRIPPER_CONTROLLERS:
        params = config[name]["ros__parameters"]
        assert "joints" not in params, "GripperActionController reads `joint`, not `joints`"
        commands, states = declared[params["joint"]]
        assert "position" in commands and "position" in states


@pytest.mark.parametrize("profile", ["mock", "gazebo", "real"])
def test_every_declared_joint_has_exactly_one_controller_and_the_mimic_has_none(profile):
    config, root = _config(), _expansion(profile)
    controlled = [j for n in ARM_CONTROLLERS for j in config[n]["ros__parameters"]["joints"]]
    controlled += [config[n]["ros__parameters"]["joint"] for n in GRIPPER_CONTROLLERS]
    assert sorted(controlled) == sorted(_declared(root))
    assert len(controlled) == len(set(controlled))
    assert not [j for j in controlled if root.find(f"joint[@name='{j}']/mimic") is not None]
