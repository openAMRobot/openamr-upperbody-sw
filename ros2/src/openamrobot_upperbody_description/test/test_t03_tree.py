# SPDX-License-Identifier: MIT
"""T03: one root `base_footprint`, no duplicate link or joint names (and no `world`, no dangling joint ends).

The root is platform-sw's base_footprint since 935efc7 (REP 105): base_link is its child, 0.098 m up.
"""
from conftest import ARM_PACKAGES, BASE_PLATES, duplicates, expand, tree_facts


def test_t03_single_root_and_unique_names(combo):
    base_plate, arm_package = combo
    links, joints, child_of = tree_facts(expand(base_plate=base_plate, arm_package=arm_package))

    roots = sorted(set(links) - set(child_of))
    assert roots == ["base_footprint"], f"roots: {roots}"
    assert child_of["base_link"] == "base_footprint"
    assert not duplicates(links), f"duplicate links: {duplicates(links)}"
    assert not duplicates(joints), f"duplicate joints: {duplicates(joints)}"

    # every link has at most one parent joint, and every joint end names a declared link
    assert len(child_of) == len(joints), "a link is the child of more than one joint"
    declared = set(links)
    dangling = sorted((set(child_of) | set(child_of.values())) - declared)
    assert not dangling, f"joints reference undeclared links: {dangling}"

    # the upstream macro must not inject a world root (stop condition of the step)
    assert "world" not in declared


def test_t03_openarm_attaches_to_option_bs_carriage():
    """ADR-008 re-rooted (P15): the arm base links hang from lift_carriage_link through option B's mount joints, which
    the upstream macro declares at option B's pose; there is no intermediate plate link."""
    root = expand(arm_package="openarm")
    _, joints, child_of = tree_facts(root)
    assert child_of["openarm_left_base_link"] == "lift_carriage_link"
    assert child_of["openarm_right_base_link"] == "lift_carriage_link"
    assert "openarm_left_base_link_mount_joint" in joints and "openarm_right_base_link_mount_joint" in joints
    assert child_of["lift_carriage_link"] == "lift_column_base_link"
    assert child_of["lift_column_base_link"] == "base_footprint"
    assert not [n for n in tree_facts(root)[0] if "body" in n], "OpenArm body must not be instantiated (ADR-008)"


def test_t03_no_arm_links_without_openarm_but_the_mount_frames_stay():
    """Option B declares the mount frames whether or not an arm is mounted: with none or so101 they are its own."""
    for base_plate in BASE_PLATES:
        for arm_package in ARM_PACKAGES:
            links, joints, child_of = tree_facts(expand(base_plate=base_plate, arm_package=arm_package))
            has_arm = any(n.startswith("openarm_") and n not in ("openarm_left_base_link", "openarm_right_base_link") for n in links)
            assert has_arm == (arm_package == "openarm")
            for side in ("left", "right"):
                assert child_of[f"openarm_{side}_base_link"] == "lift_carriage_link"
                assert f"openarm_{side}_base_link_mount_joint" in joints
