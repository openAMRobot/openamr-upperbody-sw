# SPDX-License-Identifier: MIT
"""T15: the finger-mimic multiplier is read from upstream's joint_mimics.yaml, never written into ros2_control.xacro.

P06 disclosed one mutation its tests did not catch: replacing the multiplier with a hard-coded number. It was harmless
then, because upstream only uses the value inside a commented-out joint, so nothing in any expansion changed. It stops
being harmless the moment Gazebo computes with it (A7), so this test is what the mimic work stands on.

What makes the property checkable is that it lives at file scope in ros2_control.xacro: a probe file that includes it can
print its value, which a macro-local property never allowed. Two independent checks then protect it:

  - the *value*: the probe must equal what the yaml says, read here by a separate yaml.safe_load, so a wrong number
    (-1.0, 0.5) is a mismatch;
  - the *source*: the property's expression must call xacro.load_yaml on joint_mimics.yaml, so a literal that merely
    happens to equal today's value (1.0) is also caught. A value check cannot see that one, which is the whole point.

Both checks are demonstrated on mutated copies of the real file in tmp_path (the real file is never touched).
"""
import os
import re
import xml.etree.ElementTree as ET

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

from conftest import SHARE, expand

REAL_FILE = os.path.join(SHARE, "urdf", "ros2_control.xacro")
MIMICS_YAML = os.path.join(get_package_share_directory("openarm_description"),
                           "assets", "end_effector", "pinch_gripper", "config", "joint", "joint_mimics.yaml")
PROPERTY = "pinch_finger_mimic_multiplier"
DICT_PROPERTY = "pinch_mimics_dict"
FILE_PROPERTY = "pinch_mimics_file"


def upstream_multiplier():
    """What the yaml says, read independently of xacro."""
    with open(MIMICS_YAML) as f:
        doc = yaml.safe_load(f)
    assert "joint2" in doc, "upstream's joint_mimics.yaml has no joint2: the xacro expression would silently fall back to 1.0"
    return float(doc["joint2"]["multiplier"])


def probed_multiplier(xacro_path, tmp_path):
    """The value of the multiplier property as xacro evaluates it in `xacro_path` (via a probe file that includes it)."""
    probe = tmp_path / "probe.xacro"
    probe.write_text(f'<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="probe">'
                     f'<xacro:include filename="{xacro_path}"/>'
                     f'<probe multiplier="${{{PROPERTY}}}"/></robot>')
    root = ET.fromstring(xacro.process_file(str(probe)).toxml())
    return float(root.find("probe").get("multiplier"))


def property_value(xacro_path, name):
    """The value="..." text of a top-level <xacro:property name=name> (comments are not parsed, so cannot match)."""
    root = ET.parse(xacro_path).getroot()
    found = [e for e in root if e.tag.endswith("}property") and e.get("name") == name]
    assert len(found) == 1, f"{xacro_path} must define the file-scope property {name} exactly once, found {len(found)}"
    return found[0].get("value")


def multiplier_problems(xacro_path, tmp_path):
    """Why `xacro_path` does not read the multiplier from upstream's yaml; empty when it does."""
    problems = []
    try:
        expression = property_value(xacro_path, PROPERTY)
        source = property_value(xacro_path, DICT_PROPERTY)
        file_ = property_value(xacro_path, FILE_PROPERTY)
    except AssertionError as err:
        return [f"source: {err}"]
    if f"{DICT_PROPERTY}[" not in expression:
        problems.append(f"source: {PROPERTY} is {expression!r}, which does not index {DICT_PROPERTY}: written in, not read")
    if "xacro.load_yaml" not in source or FILE_PROPERTY not in source:
        problems.append(f"source: {DICT_PROPERTY} is {source!r}, which does not load {FILE_PROPERTY}")
    if "joint_mimics.yaml" not in file_ or "$(find openarm_description)" not in file_:
        problems.append(f"source: {FILE_PROPERTY} is {file_!r}, not upstream's joint_mimics.yaml")
    try:
        value = probed_multiplier(xacro_path, tmp_path)
    except xacro.XacroException as err:        # e.g. pointed at a yaml with no joint2.multiplier: an error, not a crash
        problems.append(f"value: {PROPERTY} cannot be evaluated: {str(err).splitlines()[0]}")
    else:
        if value != upstream_multiplier():
            problems.append(f"value: {PROPERTY} evaluates to {value}, the yaml says {upstream_multiplier()}")
    return problems


def mutated_copy(tmp_path, pattern, replacement):
    text = open(REAL_FILE).read()
    mutated, count = re.subn(pattern, replacement, text, flags=re.S)
    assert count == 1, f"the mutation {pattern!r} did not apply exactly once: the test's idea of the file is out of date"
    path = tmp_path / "ros2_control.xacro"
    path.write_text(mutated)
    return str(path)


def test_t15_the_multiplier_is_read_from_upstreams_yaml(tmp_path):
    assert multiplier_problems(REAL_FILE, tmp_path) == []


def test_t15_the_multiplier_matches_the_urdf_mimic_it_must_agree_with(tmp_path):
    """The number ros2_control would use for finger_joint2 is the one robot_state_publisher already applies."""
    root = expand(hardware_profile="mock")
    for side in ("left", "right"):
        mimic = root.find(f"joint[@name='openarm_{side}_finger_joint2']/mimic")
        assert mimic.get("joint") == f"openarm_{side}_finger_joint1"
        assert float(mimic.get("multiplier")) == probed_multiplier(REAL_FILE, tmp_path)


@pytest.mark.parametrize("literal", ["1.0", "-1.0", "0.5", "${1.0}"])
def test_t15_mutation_a_hard_coded_multiplier_is_caught(tmp_path, literal):
    """The P06 mutation, including the literal equal to today's value, which only the source check can see."""
    path = mutated_copy(tmp_path, rf'(<xacro:property name="{PROPERTY}" value=")[^"]*(")', rf"\g<1>{literal}\g<2>")
    problems = multiplier_problems(path, tmp_path)
    assert any(p.startswith("source:") and "written in, not read" in p for p in problems), problems
    if float(literal.strip("${}")) != upstream_multiplier():
        assert any(p.startswith("value:") for p in problems), problems          # a wrong number is caught twice


def test_t15_mutation_a_different_yaml_is_caught(tmp_path):
    path = mutated_copy(tmp_path, rf'(name="{FILE_PROPERTY}" value="[^"]*)joint_mimics\.yaml', r"\g<1>joint_limits.yaml")
    assert any("not upstream's joint_mimics.yaml" in p for p in multiplier_problems(path, tmp_path))


def test_t15_mutation_a_removed_property_is_caught_not_crashed_on(tmp_path):
    path = mutated_copy(tmp_path, rf'<xacro:property name="{PROPERTY}"[^>]*/>', "")
    problems = multiplier_problems(path, tmp_path)
    assert problems and problems[0].startswith("source:")


def test_t15_the_unmutated_copy_is_clean(tmp_path):
    """The copy machinery itself changes nothing, so the mutation tests above fail for the mutation and nothing else."""
    path = tmp_path / "ros2_control.xacro"
    path.write_text(open(REAL_FILE).read())
    assert multiplier_problems(str(path), tmp_path) == []
