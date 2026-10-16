"""T07 / T08: collision geometry.

T07 (FAIL): every link that has an <inertial> or a <visual> has a <collision>, or is marked `collision_free: true` in
frame_contract.yaml with a reason. Frame-only links (no inertial, no visual) need nothing.
T08 (WARN only): list the links whose collision mesh is the very same file as their visual mesh. A visual mesh is
detailed; planning against it is slow and an unnecessarily tight fit. It is a flag for the owner of the link, never a
failure.

Both are pure functions of (URDF root, contract) so that they are themselves tested on synthetic URDFs below.
"""
import os
import warnings
import xml.etree.ElementTree as ET

import pytest

from conftest import MATRIX, SHARE, expand, load_yaml, write_sidecar

CONTRACT = load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))
COLLISION_FREE = {e["name"]: e for e in CONTRACT["entries"] if e["kind"] == "link" and e.get("collision_free")}
OWNERS = {e["name"]: e["owner"] for e in CONTRACT["entries"] if e["kind"] == "link"}


def collision_problems(root, collision_free):
    """FAIL strings for T07. `collision_free` maps link name -> contract entry."""
    problems = []
    for link in root.findall("link"):
        name = link.get("name")
        physical = link.find("inertial") is not None or link.find("visual") is not None
        collisions = link.findall("collision")
        if physical and not collisions:
            entry = collision_free.get(name)
            if entry is None:
                problems.append(f"{name}: has an <inertial> or <visual> but no <collision>, and is not marked "
                                f"collision_free in frame_contract.yaml")
            elif not str(entry.get("collision_free_reason", "")).strip():
                problems.append(f"{name}: marked collision_free but with no reason")
        for c in collisions:
            geometry = c.find("geometry")
            if geometry is None or len(geometry) == 0:
                problems.append(f"{name}: <collision> without a <geometry>")
    return problems


def _mesh_files(elements):
    out = []
    for e in elements:
        geometry = e.find("geometry")
        mesh = None if geometry is None else geometry.find("mesh")
        if mesh is not None:
            out.append(mesh.get("filename"))
    return out


def shared_mesh_links(root):
    """{link: mesh file} for every link whose collision mesh is the same file as one of its visual meshes (T08)."""
    out = {}
    for link in root.findall("link"):
        visual = set(_mesh_files(link.findall("visual")))
        shared = sorted(visual & set(_mesh_files(link.findall("collision"))))
        if shared:
            out[link.get("name")] = shared[0]
    return out


def _expansions():
    """(roots of every base plate x arm package expansion, every link name in any of them)."""
    roots = [expand(base_plate=base_plate, arm_package=arm_package) for base_plate, arm_package in MATRIX]
    return roots, {link.get("name") for root in roots for link in root.findall("link")}


# --- the real model -------------------------------------------------------------------------------------------------

def test_t07_every_physical_link_has_collision_or_is_marked_collision_free():
    roots, _ = _expansions()
    problems = sorted({p for root in roots for p in collision_problems(root, COLLISION_FREE)})
    assert not problems, "\n" + "\n".join(problems)


def test_t07_records_which_links_are_collision_free_and_why():
    roots, all_links = _expansions()
    checked = with_collision = 0
    used = {}
    for root in roots:
        for link in root.findall("link"):
            if link.find("inertial") is None and link.find("visual") is None:
                continue
            checked += 1
            if link.find("collision") is not None:
                with_collision += 1
            else:
                used[link.get("name")] = COLLISION_FREE[link.get("name")]["collision_free_reason"]
    # every exemption in the contract is for a link that exists somewhere in the matrix
    assert set(COLLISION_FREE) <= all_links
    write_sidecar("T07", "collision", "every link with an inertial or visual has a collision or is collision_free", "PASS", {
        "expansions": len(roots), "link_instances_checked": checked, "with_collision": with_collision,
        "collision_free": {name: {"reason": reason, "owner": OWNERS[name]} for name, reason in sorted(used.items())},
    })


def test_t08_collision_mesh_same_file_as_visual_mesh_is_a_warning():
    roots, _ = _expansions()
    found = {}
    for root in roots:
        found.update(shared_mesh_links(root))
    by_owner = {}
    for name, filename in sorted(found.items()):
        by_owner.setdefault(OWNERS.get(name, "unlisted in frame_contract.yaml"), []).append(name)
    result = "WARN" if found else "PASS"
    write_sidecar("T08", "collision_mesh_reuse", "collision mesh is the same file as the visual mesh", result, {
        "links": sorted(found), "by_owner": by_owner,
        "files": {n: f.rsplit("/", 1)[-1] for n, f in sorted(found.items())},
        "meaning": "collision shape is as detailed as the visual one: slower planning and a looser-than-needed fit. "
                   "A flag for the link's owner; geometry is not changed here",
    })
    if found:
        warnings.warn(f"T08 WARN: collision mesh == visual mesh for {', '.join(sorted(found))}", UserWarning, stacklevel=1)


# --- the checks on synthetic URDFs: they must be able to fail -------------------------------------------------------

def _root(*links):
    return ET.fromstring("<robot>" + "".join(links) + "</robot>")


BOX = "<geometry><box size='1 1 1'/></geometry>"
INERTIAL = "<inertial><mass value='1'/><inertia ixx='1' iyy='1' izz='1' ixy='0' ixz='0' iyz='0'/></inertial>"
FREE = {"marker": {"collision_free": True, "collision_free_reason": "frame marker"}}


def test_t07_checker_passes_collision_or_a_listed_exemption_or_a_bare_frame():
    root = _root(f"<link name='a'>{INERTIAL}<collision>{BOX}</collision></link>",
                 f"<link name='marker'><visual>{BOX}</visual></link>",
                 "<link name='frame_only'/>")
    assert not collision_problems(root, FREE)


@pytest.mark.parametrize("link,why", [
    (f"<link name='a'>{INERTIAL}</link>", "inertial but no collision"),
    (f"<link name='a'><visual>{BOX}</visual></link>", "visual but no collision"),
])
def test_t07_checker_fails_a_physical_link_without_collision(link, why):
    problems = collision_problems(_root(link), FREE)
    assert any("a: has an <inertial> or <visual> but no <collision>" in p for p in problems), why


def test_t07_checker_fails_an_exemption_without_a_reason_and_a_collision_without_geometry():
    root = _root(f"<link name='marker'><visual>{BOX}</visual></link>")
    assert any("no reason" in p for p in collision_problems(root, {"marker": {"collision_free": True}}))
    assert any("no reason" in p for p in collision_problems(
        root, {"marker": {"collision_free": True, "collision_free_reason": "  "}}))
    empty = _root(f"<link name='a'>{INERTIAL}<collision><geometry/></collision></link>")
    assert any("without a <geometry>" in p for p in collision_problems(empty, {}))


def test_t08_detector_lists_a_shared_mesh_and_not_a_distinct_or_primitive_one():
    root = _root(
        "<link name='same'><visual><geometry><mesh filename='package://p/m/a.stl'/></geometry></visual>"
        "<collision><geometry><mesh filename='package://p/m/a.stl'/></geometry></collision></link>",
        "<link name='distinct'><visual><geometry><mesh filename='package://p/visual/a.dae'/></geometry></visual>"
        "<collision><geometry><mesh filename='package://p/collision/a.stl'/></geometry></collision></link>",
        f"<link name='primitive'><visual><geometry><mesh filename='package://p/m/a.stl'/></geometry></visual>"
        f"<collision>{BOX}</collision></link>",
        f"<link name='box_only'><visual>{BOX}</visual><collision>{BOX}</collision></link>")
    assert shared_mesh_links(root) == {"same": "package://p/m/a.stl"}
