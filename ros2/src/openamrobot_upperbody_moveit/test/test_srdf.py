# SPDX-License-Identifier: MIT
"""The SRDF is consistent with the combined model at every base plate: names exist, poses are inside limits, the virtual joint is the
fixed one (W02), and `park` is only a placeholder copy of `home`."""
import re
import xml.etree.ElementTree as ET

import pytest

from conftest import BASE_PLATES, SRDF_FILE, load_yaml, read_text, srdf_root, srdf_text, urdf_root, urdf_text
from openamrobot_upperbody_moveit.named_poses import ARM_GROUP, ARM_POSES, GRIPPER_JOINT, read_group_states, urdf_joint_limits

ARM_JOINTS = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")}
REASONS = {"Adjacent", "ConstantTransform"}          # P12A: the only reasons a pair may be disabled for besides upstream's own (which are Adjacent)


def link_and_joint_names(root):
    return {l.get("name") for l in root.findall("link")}, {j.get("name") for j in root.findall("joint")}


def referenced_links(srdf):
    links = set()
    for d in srdf.findall("disable_collisions"):
        links |= {d.get("link1"), d.get("link2")}
    for e in srdf.findall("end_effector"):
        links.add(e.get("parent_link"))
    return links


def test_the_robot_name_is_the_urdfs():
    assert srdf_root().get("name") == urdf_root().get("name") == "openamrobot"


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_every_name_the_srdf_uses_exists_in_the_model_at_every_base_plate(base_plate):
    links, joints = link_and_joint_names(urdf_root(base_plate))
    srdf = srdf_root()
    assert referenced_links(srdf) <= links, sorted(referenced_links(srdf) - links)
    used = {j.get("name") for g in srdf.findall("group") for j in g.findall("joint")}
    used |= {j.get("name") for gs in srdf.findall("group_state") for j in gs.findall("joint")}
    used |= {p.get("name") for p in srdf.findall("passive_joint")}
    assert used <= joints, sorted(used - joints)
    # ADR-008: no OpenArm body. The names check above already fails on openarm_body_link0 (it is not a link of this model); this
    # says it directly, on attributes and not on the comments that explain why the pair was dropped.
    assert not any("openarm_body_link0" in value for el in srdf.iter() for value in el.attrib.values())


def test_the_four_groups_are_exactly_the_arms_and_the_grippers():
    groups = {g.get("name"): [j.get("name") for j in g.findall("joint")] for g in srdf_root().findall("group")}
    assert set(groups) == {"left_arm", "right_arm", "left_gripper", "right_gripper"}
    for side in ("left", "right"):
        assert groups[ARM_GROUP[side]] == ARM_JOINTS[side]
        assert groups[f"{side}_gripper"] == [GRIPPER_JOINT[side]]


def test_the_virtual_joint_is_the_fixed_world_to_base_footprint_one_not_the_planar_one():
    """AD-30: the base does not move during manipulation. W02: KDL has no planar joint, and robot_state_publisher has turned the planar
    joint into a fixed one, so a planar root would be planning against a joint the rest of the stack does not see."""
    joints = srdf_root().findall("virtual_joint")
    assert len(joints) == 1
    joint = joints[0]
    assert (joint.get("type"), joint.get("parent_frame"), joint.get("child_link")) == ("fixed", "world", "base_footprint")
    assert not any("planar" in value for el in srdf_root().iter() for value in el.attrib.values()), "no planar joint in the SRDF"


@pytest.mark.parametrize("base_plate", BASE_PLATES)
def test_named_pose_values_are_inside_the_urdf_limits(base_plate):
    limits = urdf_joint_limits(urdf_text(base_plate))
    states = read_group_states(srdf_text())
    for (group, name), values in states.items():
        for joint, value in values.items():
            lower, upper = limits[joint]
            assert lower <= value <= upper, f"{group}/{name}: {joint} = {value} outside [{lower}, {upper}]"


def test_every_arm_has_exactly_the_three_named_poses_each_setting_all_seven_joints():
    states = read_group_states(srdf_text())
    for side in ("left", "right"):
        for name in ARM_POSES:
            values = states[(ARM_GROUP[side], name)]
            assert sorted(values) == sorted(ARM_JOINTS[side]), f"{side}/{name} must set all seven arm joints"
    assert {name for (group, name) in states if group in ARM_GROUP.values()} == set(ARM_POSES)


def test_park_is_a_copy_of_home_and_says_it_is_a_placeholder():
    """A placeholder is not a licence to invent a shape: `park` is `home` and nothing else, and the file says so beside it."""
    states = read_group_states(srdf_text())
    for side in ("left", "right"):
        assert states[(ARM_GROUP[side], "park")] == states[(ARM_GROUP[side], "home")]
    text = srdf_text()
    comment = re.search(r"<!--[^>]*`park` is a PLACEHOLDER.*?-->", text, flags=re.S)
    assert comment and "Adnan" in comment.group(0) and "manipulation action contract" in comment.group(0)
    assert text.index(comment.group(0)) < text.index('<group_state name="park" group="left_arm">')


def test_passive_joints_are_the_urdf_mimic_finger_joints():
    root = urdf_root()
    passive = [p.get("name") for p in srdf_root().findall("passive_joint")]
    assert passive == ["openarm_left_finger_joint2", "openarm_right_finger_joint2"]
    for name in passive:
        assert root.find(f"joint[@name='{name}']/mimic") is not None


def test_each_disabled_pair_is_listed_once_with_a_known_reason_and_two_different_links():
    pairs = [(frozenset((d.get("link1"), d.get("link2"))), d.get("reason")) for d in srdf_root().findall("disable_collisions")]
    assert len(pairs) == load_yaml("acm/acm_generation.yaml")["counts"]["disabled_total"] > 20, "the derived matrix is missing (scripts/regenerate_acm.sh)"
    keys = [p for p, _ in pairs]
    duplicates = {tuple(sorted(k)) for k in keys if keys.count(k) > 1}
    assert not duplicates, f"pairs disabled more than once: {sorted(duplicates)[:5]}"
    assert all(len(k) == 2 for k in keys), "a link cannot be disabled against itself"
    assert {r for _, r in pairs} <= REASONS


def test_the_srdf_is_well_formed_and_the_generated_block_sits_between_its_markers():
    text = read_text(SRDF_FILE)
    ET.fromstring(text)
    assert text.count("<!-- BEGIN GENERATED ACM -->") == 1 and text.count("<!-- END GENERATED ACM -->") == 1
    block = text.split("<!-- BEGIN GENERATED ACM -->")[1].split("<!-- END GENERATED ACM -->")[0]
    counts = load_yaml("acm/acm_generation.yaml")["counts"]
    assert block.count("<disable_collisions") == counts["adjacent_derived"] + counts["constant_transform"] > 50
