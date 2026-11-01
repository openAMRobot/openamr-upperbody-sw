# SPDX-License-Identifier: MIT
"""P28: the lift column's geometry as Alex described it on 22 Oct 2026 (P-03 rev19.6 item 18) and as the DOLD Hexalift datasheet draws it.

What is pinned: the outer section is the datasheet's 240 x 60, the carriage is the INNER stage (a different, strictly smaller section, so it is
no longer a second copy of the outer box), the adapter plate is its own link of the stated size, flush with the column's rear face, and the arm
profile stands on its top face inside its footprint. What is NOT pinned and is not claimed: that the two stages do not overlap in volume (the
outer stage's collision is a solid box; a hollow one needs the housing wall, which the datasheet does not give), and the inner stage's length.
"""
import pytest

from conftest import expand, load_datums


def _box(root, link, kind="collision"):
    return tuple(float(v) for v in root.find(f"link[@name='{link}']/{kind}/geometry/box").get("size").split())


def _origin(root, joint):
    return tuple(float(v) for v in root.find(f"joint[@name='{joint}']/origin").get("xyz").split())


@pytest.fixture(scope="module")
def root():
    return expand(base_plate="bp000", arm_package="openarm")


def test_the_outer_section_is_the_datasheets_240_by_60(root):
    d = load_datums()["datums"]["lift_column_section"]
    assert d["value"] == [0.240, 0.060] and (d["source"], d["status"]) == ("supplier", "provisional")
    assert "dold-mechatronik.de" in d["ref"] and "option B drawing sheet 2 rev A" in d["ref"]
    assert _box(root, "lift_column_base_link")[:2] == pytest.approx((0.240, 0.060))


def test_the_carriage_is_the_inner_stage_not_a_second_copy_of_the_outer_box(root):
    d = load_datums()["datums"]["lift_inner_section"]
    assert d["value"] == [0.2095, 0.051] and (d["source"], d["status"]) == ("supplier", "provisional")
    outer, inner = _box(root, "lift_column_base_link"), _box(root, "lift_carriage_link")
    assert inner[:2] == pytest.approx((0.2095, 0.051))
    assert inner[0] < outer[0] and inner[1] < outer[1], "the inner stage must be smaller than the outer one on BOTH axes"
    for kind in ("visual", "collision"):
        assert _box(root, "lift_carriage_link", kind) == _box(root, "lift_carriage_link")
    # both boxes are centred on the column axis (no xy offset in their origins), so the inner section nests in the outer one with a margin all round
    for link, kind in (("lift_column_base_link", "collision"), ("lift_carriage_link", "collision")):
        origin = [float(v) for v in root.find(f"link[@name='{link}']/{kind}/origin").get("xyz").split()]
        assert origin[:2] == [0.0, 0.0]


def test_the_adapter_plate_is_its_own_link_of_the_stated_size(root):
    link = root.find("link[@name='adapter_plate_link']")
    assert link is not None and root.find("joint[@name='adapter_plate_joint']/parent").get("link") == "lift_carriage_link"
    assert _box(root, "adapter_plate_link") == pytest.approx((0.360, 0.100, 0.010))
    assert _box(root, "adapter_plate_link", "visual") == _box(root, "adapter_plate_link")
    d = load_datums()["datums"]["adapter_plate_section"]
    assert d["value"] == [0.360, 0.100] and "rev19.6" in d["ref"] and "item 18" in d["ref"]


def test_the_plate_is_240_of_column_plus_120_ahead_of_its_front_face(root):
    """Rear edge flush with the column's rear face (-0.120 from the axis), front edge 0.120 beyond the column's front face (+0.120): +0.240."""
    column_x = _box(root, "lift_column_base_link")[0]
    px, _, pz = _origin(root, "adapter_plate_joint")
    size_x, _, thickness = _box(root, "adapter_plate_link")
    assert px - size_x / 2 == pytest.approx(-column_x / 2), "the rear edge is not flush with the column's rear face"
    assert px + size_x / 2 == pytest.approx(column_x / 2 + 0.120), "the front edge is not 120 mm ahead of the column's front face"
    assert pz - thickness / 2 == pytest.approx(0.0), "the plate must rest on the carriage frame's plane (the column top at lift 0)"


def test_the_arm_profile_stands_on_the_plate_with_no_gap_and_inside_its_footprint(root):
    """Closes P25 finding 3: the profile was 10 mm above the carriage and wholly in front of the column. Now its bottom face is the plate's top face."""
    assert root.find("joint[@name='arm_profile_joint']/parent").get("link") == "adapter_plate_link", "touching siblings would be a permanent collision"
    ax, ay, az = _origin(root, "adapter_plate_joint")
    dx, dy, dz = _origin(root, "arm_profile_joint")                      # in the plate's frame
    px, py, pz = ax + dx, ay + dy, az + dz                                 # in the carriage's frame
    assert (px, py, pz) == pytest.approx((0.180, 0.0, 0.180)), "the profile's pose from the carriage is the one P-03 rev19.0 gave (centre 0.180 ahead, 0.010 + 0.170 up)"
    (ps_x, ps_y, ps_z), (as_x, as_y, as_z) = _box(root, "arm_profile_link"), _box(root, "adapter_plate_link")
    assert pz - ps_z / 2 == pytest.approx(az + as_z / 2), "a gap (or an overlap) between the plate's top face and the profile's bottom face"
    assert ax - as_x / 2 <= px - ps_x / 2 and px + ps_x / 2 <= ax + as_x / 2 + 1e-9, "the profile overhangs the plate fore-aft"
    assert ay - as_y / 2 <= py - ps_y / 2 and py + ps_y / 2 <= ay + as_y / 2 + 1e-9, "the profile overhangs the plate laterally"
    assert px + ps_x / 2 == pytest.approx(ax + as_x / 2), "the plate's front edge is the profile's front face (0.240 from the column axis)"


def test_the_plate_does_not_move_a_mount_or_a_stop():
    """Nothing but the plate and the two sections changed: the mounts, the camera and the stops are where P-03 rev19.0 put them."""
    r = expand(base_plate="bp000", arm_package="none")
    assert _origin(r, "openarm_left_base_link_mount_joint") == pytest.approx((0.180, 0.031, 0.166))
    assert _origin(r, "openarm_right_base_link_mount_joint") == pytest.approx((0.180, -0.031, 0.166))
    assert _origin(r, "lift_joint") == pytest.approx((0.0, 0.0, 0.530))
