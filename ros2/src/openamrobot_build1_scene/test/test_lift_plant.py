# SPDX-License-Identifier: MIT
"""The device lift plant drives lift_joint by force, and a CTL-017 fault is reported with its codes. No ROS import."""
import os
import xml.etree.ElementTree as ET

from openamrobot_build1_scene.faults import ctl017_faults

HERE = os.path.dirname(os.path.abspath(__file__))


def lift_plugin():
    with open(os.path.join(HERE, "..", "urdf", "build1_robot.urdf.xacro"), encoding="utf-8") as f:
        root = ET.fromstring(f.read())
    found = [p for p in root.iter("plugin")
             if p.get("name") == "gz::sim::systems::JointController" and p.findtext("joint_name") == "lift_joint"]
    assert len(found) == 1
    return found[0]


def test_the_device_lift_is_a_force_command_inside_the_effort_limit():
    plugin = lift_plugin()
    assert plugin.findtext("topic") == "/sil/gz/lift_cmd_vel"
    assert plugin.findtext("use_force_commands") == "true", "velocity mode is skipped by DART on the loaded lower limit"
    assert float(plugin.findtext("cmd_max")) <= 800.0 and float(plugin.findtext("cmd_min")) >= -800.0
    p, i = float(plugin.findtext("p_gain")), float(plugin.findtext("i_gain"))
    mass, weight = 19.8, 194.0
    assert weight / i <= 0.003, "holding error under the lift weight above the 3 mm stop window"
    assert p / (2.0 * (i * mass) ** 0.5) >= 1.0, "underdamped"
    assert p / mass * 0.001 < 2.0, "unstable at the 1 ms step"


def test_ctl017_fault_bits_are_named():
    assert ctl017_faults(0) == "none"
    assert ctl017_faults(4).startswith("F03")
    assert ctl017_faults(2 | 64) == "F02 wrong direction or limit reached while driving, F07 supervision lost"


def test_a_lift_fault_is_reported_and_never_blocked_on_the_real_time_factor():
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "rig.py"), encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("    def lift_go("):text.index("    def lift_move(")]
    assert "raise Blocked" not in body and "ctl017_faults(" in body
