# SPDX-License-Identifier: MIT
"""T15: the `gazebo` profile drives the arms with gz_ros2_control/GazeboSimSystem, same joints and interfaces as `mock`.

Expansion only: nothing launches Gazebo here (G01, in openamrobot_upperbody_gazebo, does). The block must not need a
hardware plugin of the other profiles, and the base still has no ros2_control (its DiffDrive plugin is platform-sw's,
ADR-003). The model also carries the Gazebo-side plugin that starts a controller manager next to it; its filename and class
are facts about the installed gz_ros2_control, and are checked against it here, not against documentation.
"""
import os
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from conftest import COMBINED, expand, expand_xml
from ros2_control_utils import arm_joints_needing_an_interface, blocks, controlled_joints, element_text, plugins

GAZEBO = "gz_ros2_control/GazeboSimSystem"
MODEL_PLUGIN_FILE = "gz_ros2_control-system"
MODEL_PLUGIN_CLASS = "gz_ros2_control::GazeboSimROS2ControlPlugin"


def test_t15_gazebo_arms_use_gazebo_sim_system():
    assert plugins(expand(hardware_profile="gazebo")) == [("openamr_arms_gazebo", GAZEBO)]
    assert blocks(expand(hardware_profile="gazebo"))[0].get("type") == "system"


def test_t15_gazebo_has_neither_mock_nor_hardware_plugin():
    text = element_text(expand(hardware_profile="gazebo"))
    for forbidden in ("mock_components", "openarm_hardware", "OpenArmHW"):
        assert forbidden not in text, f"{forbidden} appears in the gazebo expansion"


def test_t15_gazebo_controls_exactly_the_mock_joints_with_the_same_interfaces():
    gazebo, mock = expand(hardware_profile="gazebo"), expand(hardware_profile="mock")
    assert controlled_joints(gazebo) == controlled_joints(mock)
    assert sorted(controlled_joints(gazebo)) == arm_joints_needing_an_interface(gazebo)


def test_t15_the_base_has_no_ros2_control():
    assert all(n.startswith("openarm_") for n in controlled_joints(expand(hardware_profile="gazebo")))


def test_t15_finger_joint2_is_not_declared_and_stays_a_urdf_mimic():
    """Upstream leaves finger_joint2 out of ros2_control; the URDF <mimic> carries it. Decided P09: Gazebo's Bullet-Featherstone
    engine follows that mimic on its own (G01's gripper check, and the gazebo README), so nothing is declared for it."""
    root = expand(hardware_profile="gazebo")
    declared = controlled_joints(root)
    for side in ("left", "right"):
        assert f"openarm_{side}_finger_joint2" not in declared
        assert root.find(f"joint[@name='openarm_{side}_finger_joint2']/mimic") is not None


# --- the Gazebo-side plugin: loads a controller manager into the model ----------------------------------------------------

def _model_plugins(root):
    return [p for g in root.findall("gazebo") for p in g.findall("plugin")
            if p.get("filename") == MODEL_PLUGIN_FILE]


CONTROLLERS_FILE = "/opt/example/share/some_controllers_package/config/arm_controllers.yaml"      # a path the description never reads


def test_t15_gazebo_model_carries_the_gz_ros2_control_plugin_and_the_controllers_file_arg_is_its_parameters():
    """The description does not know where a controller config lives (ADR-003, P10 task 0): the path is the controllers_file
    xacro arg, and the <parameters> element is exactly that string."""
    plugin, = _model_plugins(expand(hardware_profile="gazebo", controllers_file=CONTROLLERS_FILE))
    assert plugin.get("name") == MODEL_PLUGIN_CLASS
    assert [p.text for p in plugin.findall("parameters")] == [CONTROLLERS_FILE]


def test_t15_without_the_arg_the_plugin_is_still_there_and_has_no_parameters_element():
    root = expand(hardware_profile="gazebo")
    plugin, = _model_plugins(root)
    assert plugin.get("name") == MODEL_PLUGIN_CLASS and plugin.find("parameters") is None
    assert plugin.findall("*") == [], "an empty controllers_file must leave the plugin with no children at all"
    assert [name for name, _ in plugins(root)] == ["openamr_arms_gazebo"], "the hardware contract is unchanged"


def test_t15_the_arg_is_the_only_difference_it_adds_and_removes_one_element():
    """With the arg the expansion equals the default one plus the <parameters> element, and nothing else moves."""
    default, given = expand_xml(hardware_profile="gazebo"), expand_xml(hardware_profile="gazebo", controllers_file=CONTROLLERS_FILE)
    assert default != given
    assert given.replace(f"<parameters>{CONTROLLERS_FILE}</parameters>", "") == default


@pytest.mark.parametrize("profile", ["mock", "real"])
def test_t15_the_other_profiles_ignore_the_controllers_file_arg(profile):
    assert expand_xml(hardware_profile=profile, controllers_file=CONTROLLERS_FILE) == expand_xml(hardware_profile=profile)


def test_t15_the_default_gazebo_expansion_is_valid_urdf_without_the_arg():
    """No controllers file is a legal description (it is a hardware contract, not a launch): check_urdf takes it."""
    import subprocess
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as f:
        f.write(expand_xml(hardware_profile="gazebo"))
        f.flush()
        result = subprocess.run(["check_urdf", f.name], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


def test_t15_the_description_does_not_resolve_the_control_package():
    """The colcon cycle was removed, not declared: no xacro, launch or config of this package may name the control package's
    path (a $(find) of it is an undeclared dependency, or a cycle because control test-depends on this package)."""
    urdf = os.path.join(os.path.dirname(COMBINED), "")
    offenders = []
    for name in sorted(os.listdir(urdf)):
        if not os.path.isfile(os.path.join(urdf, name)):      # a stale install-space link to a deleted source is I01's finding, not this check's
            continue
        with open(os.path.join(urdf, name)) as f:
            for number, line in enumerate(f, 1):
                if "$(find openamrobot_upperbody_control" in line:
                    offenders.append(f"{name}:{number}")
    assert not offenders, f"the description resolves the control package at {offenders}"


def test_t15_the_plugin_is_a_model_plugin_not_a_link_or_world_one():
    """A <gazebo> with no `reference` attribute becomes a plugin of the model; the library rejects anything that is not a model."""
    root = expand(hardware_profile="gazebo")
    holders = [g for g in root.findall("gazebo") if any(p.get("filename") == MODEL_PLUGIN_FILE for p in g.findall("plugin"))]
    assert len(holders) == 1 and holders[0].get("reference") is None


@pytest.mark.parametrize("profile", ["mock", "real"])
def test_t15_the_other_profiles_carry_no_gazebo_plugin_of_ours(profile):
    text = element_text(expand(hardware_profile=profile))
    assert "gz_ros2_control" not in text and MODEL_PLUGIN_FILE not in text


def test_t15_the_base_plugins_of_platform_sw_are_still_there_and_unchanged_by_us():
    """ADR-003: the base keeps its DiffDrive plugin. The combined model contains it exactly once, with platform-sw's topics. Its
    odom child is base_footprint since platform-sw 935efc7 (REP 105: so that base_link keeps one parent): their decision, followed
    here, and the one place this repository pins it so that a change on their side shows up."""
    root = expand(hardware_profile="gazebo")
    drive, = [p for g in root.findall("gazebo") for p in g.findall("plugin") if p.get("filename") == "gz-sim-diff-drive-system"]
    assert {t: drive.findtext(t) for t in ("topic", "odom_topic", "tf_topic", "frame_id", "child_frame_id")} == {
        "topic": "/cmd_vel", "odom_topic": "/odom", "tf_topic": "/tf", "frame_id": "odom", "child_frame_id": "base_footprint"}


def test_t15_plugin_names_are_the_ones_the_installed_gz_ros2_control_registers():
    """Read from the library and its pluginlib file, as P07 and P08 did for controller types: P06's review took a plugin name
    from a deprecation warning that misspelled it, and the lesson was to read the binary."""
    prefix = get_package_prefix("gz_ros2_control")
    library = os.path.join(prefix, "lib", f"lib{MODEL_PLUGIN_FILE}.so")
    assert os.path.isfile(library), f"{library}: the model plugin's filename does not match an installed library"
    namespace, cls = MODEL_PLUGIN_CLASS.split("::")
    mangled = f"N{len(namespace)}{namespace}{len(cls)}{cls}E".encode()       # the Itanium-mangled type name the library carries
    assert mangled in open(library, "rb").read(), f"{library} does not contain the class {MODEL_PLUGIN_CLASS}"
    xml = ET.parse(os.path.join(get_package_share_directory("gz_ros2_control"), "gz_hardware_plugins.xml")).getroot()
    assert GAZEBO in {c.get("name") for c in xml.iter("class")}, "GazeboSimSystem is not registered by gz_hardware_plugins.xml"
