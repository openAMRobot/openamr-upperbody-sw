# SPDX-License-Identifier: MIT
"""P-03 item 24 equipment (upper_body_equipment.xacro): placeholder geometry at the documentation model's positions.

What is pinned, on every base plate x arm package expansion:
  - the eight links exist, each on the parent the frame contract names, and on the right side of lift_joint (the tablet, the head light, the
    flag rod and the flag move with the lift; the second E-stop and the three stop sensors are static);
  - every pose equals the one computed HERE from the datums and option B's configuration (an independent restatement of the docs model's
    formulas, not a reading of the xacro): the chest in lift_carriage_link, the static parts in lift_column_base_link;
  - the three stop sensors are at 0.120 / 0.295 / 0.470 m above the column base, and at each calibration stop the flag is level with its sensor;
  - the mass report counts the tablet and the head light in the `equipment` group, the parts with no stated mass carry no inertial and are
    named in mass_provenance.yaml (source none), and the modelled total stays under gross_mass_limit.
Geometry only: no joint of the equipment moves and none is in ros2_control.
"""
import os

import numpy as np
import pytest

from conftest import MATRIX, SHARE, UPPERBODY, expand, load_datums, load_yaml
from urdf_utils import fixed_fk, zero_pose_poses

MOVING = {"getac_tablet_link": "arm_profile_link", "head_light_link": "arm_profile_link", "lift_flag_rod_link": "adapter_plate_link",
          "lift_flag_link": "lift_flag_rod_link", "head_camera_bracket_link": "head_camera_mount_link"}
STATIC = {"second_estop_link": "lift_column_base_link", "lift_stop_sensor_l1000_link": "lift_column_base_link",
          "lift_stop_sensor_l1175_link": "lift_column_base_link", "lift_stop_sensor_l1350_link": "lift_column_base_link"}
EQUIPMENT = {**MOVING, **STATIC}
SENSOR_Z = {"L1000": ("lift_stop_sensor_l1000_link", "lift_stop_sensor_z_l1000", 0.120),
            "L1175": ("lift_stop_sensor_l1175_link", "lift_stop_sensor_z_l1175", 0.295),
            "L1350": ("lift_stop_sensor_l1350_link", "lift_stop_sensor_z_l1350", 0.470)}
IDS = [f"{p}-{a}" for p, a in MATRIX]


def v(name):
    return load_datums()["datums"][name]["value"]


def path_to_root(root, link):
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    joints = []
    while link in by_child:
        joints.append(by_child[link])
        link = by_child[link].find("parent").get("link")
    return joints


def expected_poses():
    """{link: (frame, xyz, yaw)} from the datums and option B's configuration only."""
    geo = UPPERBODY["geometry"]
    profile = geo["arm_profile"]
    column = UPPERBODY["column"][UPPERBODY["baseline"]["column"]]
    sec_x, sec_y = v("lift_column_section")
    front = profile["center_x_from_column_axis"] + profile["size_x"] / 2 + v("chest_equipment_x_from_profile_front_face")
    tablet, light, flag = v("getac_tablet_size"), v("head_light_size"), v("lift_flag_size")
    tablet_z = v("getac_tablet_z_above_column_top")
    flag_back = v("lift_stop_sensor_face_x") - v("lift_flag_gap") - flag[0]
    out = {
        "getac_tablet_link": ("lift_carriage_link", (front + tablet[0] / 2, 0.0, tablet_z), 0.0),
        "head_light_link": ("lift_carriage_link", (front + light[0] / 2, 0.0,
                                                   tablet_z - tablet[2] / 2 - v("head_light_gap_below_tablet") - light[2] / 2), 0.0),
        "lift_flag_rod_link": ("lift_carriage_link", (flag_back - v("lift_flag_rod_width") / 2,
                                                      sec_y / 2 + v("lift_flag_rod_offset_from_face") + v("lift_flag_rod_width") / 2, 0.0), 0.0),
        "lift_flag_link": ("lift_carriage_link", (flag_back + flag[0] / 2, sec_y / 2 + v("lift_flag_offset_from_face") + flag[1] / 2,
                                                  -v("lift_flag_below_plate")), 0.0),
        "second_estop_link": ("lift_column_base_link", (-sec_x / 2 - v("second_estop_panel_behind_profile"), 0.0,
                                                        column["retracted_length"] - v("lift_static_profile_top_below_column_top")
                                                        - v("second_estop_below_profile_top")), np.pi),
    }
    lens_z = geo["arm_mount"]["z_above_column_top"] + geo["head_camera_lens"]["z_above_arm_mount"]
    lens_x = geo["arm_mount"]["x_from_column_axis"] + geo["head_camera_lens"]["x_ahead_of_arm_mount"]
    out["head_camera_bracket_link"] = ("lift_carriage_link", (lens_x - v("head_camera_bracket_x_behind_lens"), 0.0,
                                                              geo["adapter_plate_thickness"] + profile["length"]), 0.0)
    for link, datum, _ in SENSOR_Z.values():
        out[link] = ("lift_column_base_link", (v("lift_stop_sensor_face_x"), sec_y / 2 + v("lift_stop_sensor_axis_from_face"), v(datum)), 0.0)
    return out


def pose_in(root, frame, link):
    """Pose of `link` in `frame`; through fixed joints only (the carriage's children) or, for the static parts, from lift_column_base_link."""
    return fixed_fk(root, frame, link)


@pytest.mark.parametrize("base_plate,arm_package", MATRIX, ids=IDS)
def test_item24_links_present_on_their_parent_and_side_of_the_lift(base_plate, arm_package):
    root = expand(base_plate=base_plate, arm_package=arm_package)
    links = {l.get("name") for l in root.findall("link")}
    assert set(EQUIPMENT) <= links, f"missing: {sorted(set(EQUIPMENT) - links)}"
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    for link, parent in EQUIPMENT.items():
        joint = by_child[link]
        assert joint.find("parent").get("link") == parent, f"{link}: parent {joint.find('parent').get('link')}, expected {parent}"
        assert joint.get("type") == "fixed", f"{link}: joint {joint.get('name')} is {joint.get('type')}, geometry only means fixed"
        moving = any(j.get("name") == "lift_joint" for j in path_to_root(root, link))
        assert moving == (link in MOVING), f"{link}: {'below' if moving else 'not below'} lift_joint"
    contract = {e["name"]: e for e in load_yaml(os.path.join(SHARE, "config", "frame_contract.yaml"))["entries"]}
    for link, parent in EQUIPMENT.items():
        assert contract[link]["parent"] == parent and contract[link]["req"] == "R", link


@pytest.mark.parametrize("base_plate,arm_package", MATRIX, ids=IDS)
def test_item24_poses_equal_the_datums(base_plate, arm_package):
    root = expand(base_plate=base_plate, arm_package=arm_package)
    for link, (frame, xyz, yaw) in expected_poses().items():
        pose = pose_in(root, frame, link)
        assert pose[:3, 3] == pytest.approx(xyz, abs=1e-9), f"{link} in {frame}: {pose[:3, 3]} != {xyz}"
        assert pose[:3, :3] == pytest.approx(np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]), abs=1e-9), link


def test_item24_chest_boxes_sit_on_the_profile_front_face_and_the_estop_block_on_the_profile_rear_face():
    root = expand()
    profile = UPPERBODY["geometry"]["arm_profile"]
    front = profile["center_x_from_column_axis"] + profile["size_x"] / 2
    for link, size in (("getac_tablet_link", v("getac_tablet_size")), ("head_light_link", v("head_light_size"))):
        assert fixed_fk(root, "lift_carriage_link", link)[0, 3] - size[0] / 2 == pytest.approx(front), f"{link}: rear face off the profile face"
    estop = fixed_fk(root, "lift_column_base_link", "second_estop_link")
    block = root.find("link[@name='second_estop_link']/collision[@name='placeholder_second_estop_contact_block']")
    bx = float(block.find("origin").get("xyz").split()[0])
    size = float(block.find("geometry/box").get("size").split()[0])
    inner_face = (estop @ np.array([bx - size / 2, 0, 0, 1]))[0]          # the block's face toward the column, in the column frame
    assert inner_face == pytest.approx(-v("lift_column_section")[0] / 2), "the contact block does not stand on the profile rear face"
    # the head light's top edge is the gap below the tablet's bottom edge (gap_light_tablet of the docs model)
    t = fixed_fk(root, "lift_carriage_link", "getac_tablet_link")[2, 3] - v("getac_tablet_size")[2] / 2
    h = fixed_fk(root, "lift_carriage_link", "head_light_link")[2, 3] + v("head_light_size")[2] / 2
    assert t - h == pytest.approx(v("head_light_gap_below_tablet")) == pytest.approx(0.030)


@pytest.mark.parametrize("stop", sorted(SENSOR_Z))
def test_item24_the_three_stop_sensor_heights_and_the_flag_level_with_its_sensor_at_its_stop(stop):
    link, datum, height = SENSOR_Z[stop]
    root = expand()
    assert v(datum) == height
    above_base = fixed_fk(root, "lift_column_base_link", link)[2, 3]
    assert above_base == pytest.approx(height, abs=1e-12)
    plate_top = UPPERBODY["geometry"]["base_plate_top_z"]
    poses = zero_pose_poses(root)
    assert poses[link][2, 3] == pytest.approx(plate_top + height, abs=1e-12), f"{link}: not {plate_top + height} m above the floor"
    geo, column = UPPERBODY["geometry"], UPPERBODY["column"][UPPERBODY["baseline"]["column"]]
    q = UPPERBODY["lift_calibration_stops"][stop] - (geo["base_plate_top_z"] + column["retracted_length"] + geo["arm_mount"]["z_above_column_top"])
    at_stop = zero_pose_poses(root, {"lift_joint": q})
    assert at_stop["lift_flag_link"][2, 3] == pytest.approx(at_stop[link][2, 3], abs=1e-9), f"at {stop} the flag is not level with {link}"
    for other, _, _ in SENSOR_Z.values():
        if other != link:
            assert abs(at_stop["lift_flag_link"][2, 3] - at_stop[other][2, 3]) > 0.1, f"at {stop} the flag is also level with {other}"


def test_item24_the_mass_report_includes_the_new_parts():
    from test_t10_mass_report import load_provenance, mass_by_group
    root = expand()
    kg, links = mass_by_group(root)
    assert sorted(links["equipment"]) == ["getac_tablet_link", "head_light_link"]
    assert kg["equipment"] == pytest.approx(v("getac_tablet_mass") + v("head_light_mass")) == pytest.approx(0.700)
    assert not links["unclassified"]
    massless = [link for link in EQUIPMENT if link not in links["equipment"]]
    for link in massless:
        assert root.find(f"link[@name='{link}']/inertial") is None, f"{link}: an inertial with no stated mass"
    entry = load_provenance()["equipment_no_mass"]
    assert entry["source"] == "none" and entry["state"] == "placeholder"
    assert all(link in entry["note"] for link in massless), "every part with no stated mass is named in mass_provenance.yaml"


def test_item24_the_gross_mass_with_the_equipment_is_under_the_limit():
    from test_t10_mass_report import mass_by_group
    limit = v("gross_mass_limit")
    for base_plate, arm_package in MATRIX:
        kg, links = mass_by_group(expand(base_plate=base_plate, arm_package=arm_package))
        assert "getac_tablet_link" in links["equipment"], "the total must include the equipment"
        total = sum(kg.values())
        assert total < limit, f"{base_plate}/{arm_package}: {total:.3f} kg >= gross_mass_limit {limit} kg"


def test_item24_no_equipment_joint_is_in_ros2_control_or_movable():
    root = expand()
    controlled = {j.get("name") for rc in root.findall("ros2_control") for j in rc.findall("joint")}
    by_child = {j.find("child").get("link"): j for j in root.findall("joint")}
    for link in EQUIPMENT:
        joint = by_child[link]
        assert joint.get("type") == "fixed" and joint.get("name") not in controlled, joint.get("name")


# ---- head camera body (register HEAD-CAMERA-IDENTITY): the numbers below are the register's, written here on purpose, so a change of option B's
# configuration or of the datums that moved the camera away from them fails this test instead of following along.
HCI_LENS_AHEAD_OF_ARM_MOUNT = 0.042      # "lens 0.042 m ahead of and 0.223 m above the arm mount point, centred"
HCI_LENS_ABOVE_ARM_MOUNT = 0.223
HCI_BASELINE_PITCH_DEG = 30              # "pitch 15 to 35 degrees down in 5 degree steps about the lens line, baseline 30 degrees" (hp30)


def _rot_y(deg):
    a = np.radians(deg)
    return np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])


@pytest.mark.parametrize("base_plate,arm_package", MATRIX, ids=IDS)
def test_item24_the_head_camera_body_sits_at_the_head_camera_identity_pose(base_plate, arm_package):
    root = expand(base_plate=base_plate, arm_package=arm_package)
    assert UPPERBODY["baseline"]["head_pitch"] == "hp30"
    mount = fixed_fk(root, "lift_carriage_link", "openarm_left_base_link")
    lens = fixed_fk(root, "lift_carriage_link", "head_camera_link")
    assert lens[0, 3] - mount[0, 3] == pytest.approx(HCI_LENS_AHEAD_OF_ARM_MOUNT, abs=1e-12)
    assert lens[2, 3] - mount[2, 3] == pytest.approx(HCI_LENS_ABOVE_ARM_MOUNT, abs=1e-12)
    assert lens[1, 3] == pytest.approx(0.0, abs=1e-12), "the lens line is centred"
    assert lens[:3, :3] == pytest.approx(_rot_y(HCI_BASELINE_PITCH_DEG), abs=1e-12), "head_camera_link is not pitched 30 degrees down (hp30)"
    link = root.find("link[@name='head_camera_link']")
    size = tuple(v("head_camera_size"))
    visuals = [vis for vis in link.findall("visual") if vis.find("geometry/box") is not None]
    assert len(visuals) == 1, "head_camera_link has no ZED Mini body visual"
    collision = link.find("collision[@name='placeholder_head_camera_body']")
    assert collision is not None, "head_camera_link has no ZED Mini body collision"
    for element in (visuals[0], collision):
        assert tuple(float(x) for x in element.find("geometry/box").get("size").split()) == pytest.approx(size)
        assert [float(x) for x in element.find("origin").get("xyz").split()] == pytest.approx([v("head_camera_body_x_from_lens_line"), 0, 0])
        assert [float(x) for x in element.find("origin").get("rpy").split()] == pytest.approx([0, 0, 0]), "the body pitches with the frame"
    assert float(link.find("inertial/mass").get("value")) == pytest.approx(v("head_camera_mass")) == pytest.approx(0.0629)
    # the body's centre in the carriage frame: on the lens line's plane, 13 mm behind the lens along the pitched optical axis
    centre = lens @ np.array([v("head_camera_body_x_from_lens_line"), 0, 0, 1])
    expected = lens[:3, 3] + _rot_y(HCI_BASELINE_PITCH_DEG) @ np.array([v("head_camera_body_x_from_lens_line"), 0, 0])
    assert centre[:3] == pytest.approx(expected, abs=1e-12)


@pytest.mark.parametrize("head_pitch,deg", [("hp15", 15), ("hp35", 35)])
def test_item24_the_head_camera_body_pitches_with_head_pitch(head_pitch, deg):
    root = expand(head_pitch=head_pitch)
    assert fixed_fk(root, "lift_carriage_link", "head_camera_link")[:3, :3] == pytest.approx(_rot_y(deg), abs=1e-12)
    assert root.find("link[@name='head_camera_link']/collision[@name='placeholder_head_camera_body']") is not None


def test_item24_the_head_camera_mount_runs_from_the_profile_top_to_the_lens_line():
    root = expand()
    geo = UPPERBODY["geometry"]
    bracket = fixed_fk(root, "lift_carriage_link", "head_camera_bracket_link")
    lens = fixed_fk(root, "lift_carriage_link", "head_camera_link")
    profile_top = geo["adapter_plate_thickness"] + geo["arm_profile"]["length"]
    assert bracket[2, 3] == pytest.approx(profile_top, abs=1e-12), "the mount foot is not on the arm-profile top"
    post = root.find("link[@name='head_camera_bracket_link']/collision[@name='placeholder_head_camera_bracket_post']")
    height = float(post.find("geometry/box").get("size").split()[2])
    assert bracket[2, 3] + height == pytest.approx(lens[2, 3], abs=1e-12), "the mount post does not reach the lens line"
    assert lens[0, 3] - bracket[0, 3] == pytest.approx(v("head_camera_bracket_x_behind_lens"), abs=1e-12)
