# SPDX-License-Identifier: MIT
"""P28: the lift column's geometry as Alex described it on 22 Oct 2026 (P-03 rev19.6 item 18) and as the DOLD Hexalift datasheet draws it.

What is pinned: the outer section is the datasheet's 240 x 60, the carriage is the INNER stage (a different, strictly smaller section, so it is
no longer a second copy of the outer box), the adapter plate is its own link of the stated size, and the arm profile stands on its top face inside
its footprint. P-03 rev20.4 item 28 (owner decisions of 8 Nov 2026, DOLD supplier CAD Hexalift_EB530_Hub350_mit_Montagelaschen.STEP) changed three
of these pins: the fixed stage ends 25 mm below the retracted column top (static top 505 mm), the inner stage is 209.5 x 53.3 x 470 mm with its
top at the carriage frame (the datasheet's 51 mm and the stroke as its length are history), and the plate is 425 mm with a 65 mm rear extension
behind the column's rear face (360 mm, rear edge flush, is history). What is NOT pinned and is not claimed: that the two stages do not overlap in
volume (the outer stage's collision is a solid box; a hollow one needs the housing wall, which the STEP has but the model does not draw).
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


def test_the_fixed_stage_ends_at_the_static_top_of_the_dold_step(root):
    """P-03 rev20.4 item 28: static profile to 503 mm plus a 2 mm cover, top 505 mm above the column base, 25 mm below the retracted top."""
    d = load_datums()["datums"]["lift_static_profile_top_below_column_top"]
    assert d["value"] == 0.025 and (d["source"], d["status"]) == ("supplier", "provisional")
    assert "Hexalift_EB530_Hub350_mit_Montagelaschen.STEP" in d["ref"]
    for kind in ("visual", "collision"):
        size = _box(root, "lift_column_base_link", kind)
        origin = [float(v) for v in root.find(f"link[@name='lift_column_base_link']/{kind}/origin").get("xyz").split()]
        assert size[2] == pytest.approx(0.505) and origin[2] == pytest.approx(0.505 / 2), f"the fixed stage's {kind} is not 0 to 0.505 m"
    assert _origin(root, "lift_joint")[2] == pytest.approx(0.530), "the retracted column top (the carriage frame at lift 0) did not move"


def test_the_dold_foot_is_drawn_under_the_fixed_stage(root):
    """P-03 rev20.4 item 28 (DOLD STEP): foot with mounting tabs 280 x 60.4 x 10 mm, centred on the column axis, from the column base up; the
    roadmap model draws it (M.DIMS.COL.FOOT). Visual only: the link keeps one collision, the fixed stage's box from the column base."""
    d = load_datums()["datums"]["lift_foot_size"]
    assert d["value"] == [0.280, 0.0604, 0.010] and (d["source"], d["status"]) == ("supplier", "provisional")
    assert "Hexalift_EB530_Hub350_mit_Montagelaschen.STEP" in d["ref"] and "item 28" in d["ref"]
    visuals = root.findall("link[@name='lift_column_base_link']/visual")
    feet = [v for v in visuals if [float(x) for x in v.find("origin").get("xyz").split()] == pytest.approx([0.0, 0.0, 0.005])]
    assert len(visuals) == 2 and len(feet) == 1
    assert tuple(float(x) for x in feet[0].find("geometry/box").get("size").split()) == pytest.approx((0.280, 0.0604, 0.010))
    assert [c.get("name") for c in root.findall("link[@name='lift_column_base_link']/collision")] == ["placeholder_lift_column_fixed_stage"]


@pytest.mark.parametrize("base_plate", ["bp000", "bp050"])
def test_the_lift_base_plate_is_fixed_to_the_base_and_reaches_10_mm_past_the_foot(base_plate):
    """P-03 rev20.6 item 30 A3 (D-20): MEC-019 is 350 x 140 x 10 mm, 10 mm longer at each end than the foot, one plate for bp000 and bp050
    (the column moves on it, the plate does not); its top face is the column base (geometry.base_plate_top_z)."""
    from conftest import UPPERBODY
    from urdf_utils import fixed_fk
    r = expand(base_plate=base_plate, arm_package="openarm")
    d = load_datums()["datums"]["lift_base_plate_size"]
    assert d["value"] == [0.350, 0.140, 0.010] and "item 30 A3" in d["ref"]
    assert root_parent(r, "lift_base_plate_joint") == "base_footprint"
    assert _box(r, "lift_base_plate_link") == _box(r, "lift_base_plate_link", "visual") == pytest.approx((0.350, 0.140, 0.010))
    assert r.find("link[@name='lift_base_plate_link']/inertial") is None, "the mass ledger carries the plate on base_link: no second mass"
    plate = fixed_fk(r, "base_footprint", "lift_base_plate_link")
    column = fixed_fk(r, "base_footprint", "lift_column_base_link")
    assert plate[:3, 3] == pytest.approx((0.085, 0.0, 0.299), abs=1e-12), "the plate moved with the base plate configuration"
    assert plate[2, 3] + 0.005 == pytest.approx(column[2, 3]) == pytest.approx(UPPERBODY["geometry"]["base_plate_top_z"])
    foot = load_datums()["datums"]["lift_foot_size"]["value"]
    axes = [load_datums()["datums"]["column_axis_x_bp000"]["value"] + v["column_axis_offset_x"] for v in UPPERBODY["base_plate"].values()]
    assert plate[0, 3] - 0.175 == pytest.approx(min(axes) - foot[0] / 2 - 0.010), "not 10 mm past the foot's rear end at bp000"
    assert plate[0, 3] + 0.175 == pytest.approx(max(axes) + foot[0] / 2 + 0.010), "not 10 mm past the foot's front end at bp050"


def root_parent(root, joint):
    return root.find(f"joint[@name='{joint}']/parent").get("link")


def test_the_carriage_is_the_inner_stage_not_a_second_copy_of_the_outer_box(root):
    d = load_datums()["datums"]["lift_inner_section"]
    # P-03 rev20.4 item 28: the DOLD STEP's inner stage, 209.5 x 53.3 x 470 mm (was 209.5 x 51 from the datasheet, length the stroke)
    assert d["value"] == [0.2095, 0.0533, 0.470] and (d["source"], d["status"]) == ("supplier", "provisional")
    assert "Hexalift_EB530_Hub350_mit_Montagelaschen.STEP" in d["ref"]
    outer, inner = _box(root, "lift_column_base_link"), _box(root, "lift_carriage_link")
    assert inner == pytest.approx((0.2095, 0.0533, 0.470))
    # its top at the carriage frame, so its bottom is 60 mm above the column base when retracted (the STEP)
    origin = [float(v) for v in root.find("link[@name='lift_carriage_link']/collision/origin").get("xyz").split()]
    assert origin[2] == pytest.approx(-0.470 / 2)
    assert _origin(root, "lift_joint")[2] + origin[2] - inner[2] / 2 == pytest.approx(0.060)
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
    # P-03 rev20.4 item 28: one 6082-T6 part, 425 x 100 x 10 mm (was 360 x 100 x 10, P-03 rev19.6 item 18)
    assert _box(root, "adapter_plate_link") == pytest.approx((0.425, 0.100, 0.010))
    assert _box(root, "adapter_plate_link", "visual") == _box(root, "adapter_plate_link")
    d = load_datums()["datums"]["adapter_plate_section"]
    assert d["value"] == [0.425, 0.100] and "rev20.4" in d["ref"] and "item 28" in d["ref"]
    assert load_datums()["datums"]["adapter_plate_rear_extension"]["value"] == 0.065


def test_the_plate_reaches_65_mm_behind_the_column_and_120_mm_ahead_of_its_front_face(root):
    """P-03 rev20.4 item 28: rear edge 65 mm behind the column's rear face (-0.185 from the axis, the docs model's plate_rear_x), front edge
    0.120 beyond the column's front face (+0.240, the arm profile's front face). Was: rear edge flush with the column's rear face."""
    column_x = _box(root, "lift_column_base_link")[0]
    px, _, pz = _origin(root, "adapter_plate_joint")
    size_x, _, thickness = _box(root, "adapter_plate_link")
    assert px - size_x / 2 == pytest.approx(-column_x / 2 - 0.065) == pytest.approx(-0.185), "the rear edge is not 65 mm behind the column's rear face"
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
