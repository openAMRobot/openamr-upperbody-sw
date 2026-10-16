"""W01: the root link carries no inertial, so KDL consumers lose nothing. RETIRED AS A WARN in P15 (P12A's WARN, A6's since S3).

Until P15 the root was base_link, which carries platform-sw's 4.175 kg inertial, and KDL does not support a root link with an
inertia: robot_state_publisher logs "The root link base_link has an inertia specified in the URDF, but KDL does not support a root
link with an inertia" and MoveIt (KDL) dropped it. The published workaround is a dummy root above base_link, which ADR-010 s4
proposed as base_footprint and platform-sw 935efc7 made real (REP 105, FRAMES-REP105, Alex decision 2026-10-12): the root is now
base_footprint, which has no inertial, and base_link is its child, so its 4.175 kg is seen. The ADR said so in advance ("a frame
change fixes a warning we had decided to live with").

The check stays as a guard against that being undone: it FAILS if the root of an expansion has an inertial again, and records
that base_link's inertial is now a non-root link's. This is a moved number (W01 WARN -> PASS) with its reason, not a deleted check.
"""
import pytest

from conftest import ARM_PACKAGES, expand, tree_facts, write_sidecar


@pytest.mark.parametrize("planar", ["false", "true"])
@pytest.mark.parametrize("arm_package", ARM_PACKAGES)
def test_w01_the_root_has_no_inertial(arm_package, planar):
    root = expand(arm_package=arm_package, enable_planar_base=planar)
    links, _, child_of = tree_facts(root)
    roots = sorted(set(links) - set(child_of))
    assert roots == (["world"] if planar == "true" else ["base_footprint"])
    link = next(e for e in root.findall("link") if e.get("name") == roots[0])
    assert link.find("inertial") is None, f"the root {roots[0]} has an inertial again: KDL drops a root link's inertia (W01 is back)"


def test_w01_base_link_inertia_is_seen_by_kdl_consumers(record_property):
    root = expand(arm_package="openarm")
    links, _, child_of = tree_facts(root)
    assert child_of["base_link"] == "base_footprint"
    base_link = next(link for link in root.findall("link") if link.get("name") == "base_link")
    inertial = base_link.find("inertial")
    assert inertial is not None, "base_link has no inertial any more: W01's subject is gone, retire the check"
    mass = float(inertial.find("mass").get("value"))
    record_property("check_id", "W01")
    write_sidecar("W01", "kdl_root_inertia", "the root link has no inertial, so KDL consumers lose nothing", "PASS", {
        "root": "base_footprint (platform-sw 935efc7, REP 105)",
        "root_inertial": None,
        "base_link_mass_kg": mass,
        "base_link_is_a_non_root_link": True,
        "was": "WARN since S3: base_link was the root, so KDL consumers (robot_state_publisher, MoveIt) dropped its 4.175 kg",
        "why_it_moved": "the root is base_footprint now (a frame change by platform-sw, ADR-010 s4 predicted it); nothing here was "
                        "'fixed' by us",
        "guard": "FAILS if a root link carries an inertial again, in any arm package, planar or not",
    })
