# SPDX-License-Identifier: MIT
"""T11: the planar base joint is present in source, disabled by default, and correct when enabled."""
import os
import re

import pytest
import xacro

from conftest import ARM_PACKAGES, BASE_PLATES, COMBINED, expand, expand_xml, tree_facts

COMMENT = ("A6: planar base joint — present, DISABLED by default (AD-30: base and arm never move together in "
           "OpenAMRobot 2.0). Enable only for future whole-body planning experiments.")


def _source():
    with open(COMBINED, encoding="utf-8") as f:
        return f.read()


@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_t11_default_has_no_world_and_no_planar_joint(arm_package):
    links, joints, _ = tree_facts(expand(arm_package=arm_package))
    assert "world" not in links
    assert "base_planar_joint" not in joints


@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_t11_enabled_has_world_and_a_planar_joint_world_to_base_footprint(arm_package):
    root = expand(arm_package=arm_package, enable_planar_base="true")
    links, joints, child_of = tree_facts(root)
    assert "world" in links and "base_planar_joint" in joints
    joint = root.find("joint[@name='base_planar_joint']")
    assert joint.get("type") == "planar"
    assert joint.find("parent").get("link") == "world"
    assert joint.find("child").get("link") == "base_footprint"
    # a planar joint moves in the plane normal to its axis; the URDF default (x) would be a vertical plane
    assert joint.find("axis").get("xyz").split() == ["0", "0", "1"]
    # the root moves from base_footprint to world, and nothing else changes (base_link stays base_footprint's child)
    assert sorted(set(links) - set(child_of)) == ["world"]
    assert child_of["base_footprint"] == "world" and child_of["base_link"] == "base_footprint"
    default_links, default_joints, _ = tree_facts(expand(arm_package=arm_package))
    assert sorted(set(links) - set(default_links)) == ["world"]
    assert sorted(set(joints) - set(default_joints)) == ["base_planar_joint"]


def test_t11_planar_flag_leaves_the_lift_and_arms_untouched():
    for base_plate in BASE_PLATES:
        a = expand(base_plate=base_plate)
        b = expand(base_plate=base_plate, enable_planar_base="true")
        for name in ("lift_column_base_joint", "lift_joint", "openarm_left_base_link_mount_joint", "head_camera_mount_joint"):
            assert a.find(f"joint[@name='{name}']/origin").attrib == b.find(f"joint[@name='{name}']/origin").attrib


def test_t11_the_comment_block_precedes_the_planar_block_in_the_source():
    src = _source()
    comments = re.findall(r"<!--(.*?)-->", src, re.S)
    assert any(" ".join(c.split()) == COMMENT for c in comments), "the A6 planar comment is missing or has been edited"
    # ... and sits directly above the block it describes
    m = re.search(r"-->\s*<xacro:if value=\"\$\{enable_planar_base == 'true'\}\">\s*<link name=\"world\"/>", src)
    assert m and COMMENT.split(" — ")[0] in src[m.start() - len(COMMENT) - 20:m.start() + 5]


# --- the arg quoting in combined_robot.xacro is load-bearing (DESIGN_LOG 2026-10-05): do not "clean it up" ---------

@pytest.mark.parametrize("arg", ["column", "base_plate", "head_pitch", "base_tilt", "arm_package", "hardware_profile",
                                  "enable_planar_base", "datums_file"])
def test_t11_args_stay_quoted_and_eager(arg):
    src = _source()
    assert re.search(rf'<xacro:property name="{arg}" lazy_eval="false" value="\'\$\(arg {arg}\)\'"/>', src), (
        f"{arg}: the raw-string quoting of the arg has been changed")


def test_t11_planar_condition_compares_the_string():
    assert "<xacro:if value=\"${enable_planar_base == 'true'}\">" in _source()


@pytest.mark.parametrize("value", ["True", "TRUE", "1", "yes", ""])
def test_t11_only_the_lowercase_string_true_enables_it(value):
    """Without the quoting, xacro would turn True/true into a Python bool and the accepted-value check would pass."""
    with pytest.raises(xacro.XacroException):
        expand_xml(enable_planar_base=value)
