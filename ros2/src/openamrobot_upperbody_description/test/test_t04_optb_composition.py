"""T-optB: the combined robot IS option B's upper body, plus what composition adds. Alex's model is consumed, not rewritten.

Three things are asserted, because a model can expand cleanly and be wrong (the rev19.0 lesson: a column that was not on the drive
axle, for six steps):
  1. every frame of his standalone model (urdf/openamrobot_upperbody.urdf.xacro, the file main ships) is in the combined
     expansion at the same pose from base_footprint, at every lift position, for every base plate and head pitch;
  2. the numbers he put on main are the ones the combined model uses: arm mount y +-0.031, x +0.180, z +0.166 above the column top;
     the column axis at x +0.060 (bp000) and +0.110 (bp050); shoulder 1.000 m at lift 0 and 1.350 m at 0.350; a 0.098 m base_link;
  3. his model is unchanged in what it is for: with the two opt-in parameters at their defaults it expands exactly as on main.
"""
import numpy as np
import pytest
import xacro

import evidence_common as ev
from conftest import AXES, BASELINE, LIFT_STROKE, SHARE, STOPS, UPPERBODY, expand, expand_xml, load_datums, read_text
from urdf_utils import zero_pose_poses

OPTION_B_FRAMES = ["base_footprint", "lift_column_base_link", "lift_carriage_link", "arm_profile_link", "openarm_left_base_link",
                   "openarm_right_base_link", "head_camera_mount_link", "head_camera_link", "head_camera_optical_frame"]
STANDALONE = f"{SHARE}/urdf/openamrobot_upperbody.urdf.xacro"
TOL = 1e-9


def standalone(**mappings):
    import xml.etree.ElementTree as ET
    return ET.fromstring(xacro.process_file(STANDALONE, mappings={k: str(v) for k, v in mappings.items()}).toxml())


CASES = [(plate, pitch) for plate in AXES["base_plate"] for pitch in AXES["head_pitch"]]


@pytest.mark.parametrize("arm_package", ev.ARM_PACKAGES)
@pytest.mark.parametrize("base_plate,head_pitch", CASES, ids=[f"{p}-{h}" for p, h in CASES])
def test_every_frame_of_option_b_is_in_the_combined_model_at_the_same_pose(base_plate, head_pitch, arm_package):
    mine = {"base_plate": base_plate, "head_pitch": head_pitch}
    for q in (0.0, LIFT_STROKE / 2, LIFT_STROKE):
        want = zero_pose_poses(standalone(**mine), {"lift_joint": q})
        got = zero_pose_poses(expand(**mine, arm_package=arm_package), {"lift_joint": q})
        for frame in OPTION_B_FRAMES:
            assert frame in got, f"{frame} is not in the combined model"
            assert np.allclose(want[frame], got[frame], atol=TOL), (
                f"{frame} at lift {q}: option B {want[frame][:3, 3]} vs combined {got[frame][:3, 3]}")


def test_the_numbers_option_b_put_on_main_are_the_ones_the_model_uses():
    geo = UPPERBODY["geometry"]
    assert (geo["arm_mount"]["x_from_column_axis"], geo["arm_mount"]["y"], geo["arm_mount"]["z_above_column_top"]) == (0.180, 0.031, 0.166)
    for plate, column_x in (("bp000", 0.060), ("bp050", 0.110)):
        poses = zero_pose_poses(expand(base_plate=plate), {"lift_joint": 0.0})
        assert poses["lift_column_base_link"][0, 3] == pytest.approx(column_x, abs=TOL), plate
        assert poses["lift_column_base_link"][2, 3] == pytest.approx(0.304, abs=TOL)
        for side, sign in (("left", 1), ("right", -1)):
            p = poses[f"openarm_{side}_base_link"][:3, 3]
            assert p == pytest.approx([column_x + 0.180, sign * 0.031, 1.000], abs=TOL), (plate, side, p)
        top = zero_pose_poses(expand(base_plate=plate), {"lift_joint": LIFT_STROKE})["openarm_left_base_link"][2, 3]
        assert top == pytest.approx(1.350, abs=TOL)


def test_the_column_axis_datum_is_the_default_of_option_bs_own_wrapper():
    import re
    wrapper = read_text(STANDALONE)
    default = float(re.search(r'<xacro:arg name="column_axis_x_bp000" default="([0-9.]+)"', wrapper).group(1))
    assert default == load_datums()["datums"]["column_axis_x_bp000"]["value"] == 0.060


def test_base_link_is_98_mm_above_the_floor_in_the_expansion_and_in_the_datum():
    """The datum is a decision record (provisional); the expansion is platform-sw's joint. T04: they must agree."""
    root = expand()
    joint = next(j for j in root.findall("joint") if j.get("name") == "base_footprint_joint")
    assert joint.find("parent").get("link") == "base_footprint" and joint.find("child").get("link") == "base_link"
    z = float(joint.find("origin").get("xyz").split()[2])
    assert z == pytest.approx(load_datums()["datums"]["base_link_height_above_floor"]["value"], abs=TOL) == pytest.approx(0.098, abs=TOL)


def test_the_arms_are_62_mm_apart_at_their_bases_and_that_is_option_bs_number_not_ours():
    poses = zero_pose_poses(expand(arm_package="openarm"))
    gap = poses["openarm_left_base_link"][1, 3] - poses["openarm_right_base_link"][1, 3]
    assert gap == pytest.approx(0.062, abs=TOL)


def test_the_lift_calibration_stops_are_the_stops_and_are_joint_positions():
    assert STOPS == {"L1000": 1.000, "L1175": 1.175, "L1350": 1.350}
    expected = {"L1000": 0.0, "L1175": 0.175, "L1350": 0.350}
    for stop, q in expected.items():
        assert ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop) == pytest.approx(q, abs=1e-9)
        z = zero_pose_poses(expand(), {"lift_joint": q})["openarm_left_base_link"][2, 3]
        assert z == pytest.approx(STOPS[stop], abs=TOL), stop


# --- option B on main is unchanged in what it is for ------------------------------------------------------------------------

def test_with_the_opt_in_parameters_at_their_defaults_the_macro_expands_as_it_does_on_main():
    """emit_arm_mounts defaults true and column_inertial to none: the standalone wrapper (which sets neither) has the mount
    frames and joints, and no inertial anywhere ('No masses, inertias or supplier outlines are modelled')."""
    root = standalone()
    names = {e.get("name") for e in root.findall("link")}
    assert {"openarm_left_base_link", "openarm_right_base_link"} <= names
    assert root.findall(".//inertial") == []
    assert {e.get("name") for e in root.findall("joint")} >= {"openarm_left_base_link_mount_joint", "openarm_right_base_link_mount_joint"}


def test_the_lift_joint_has_no_authority_to_be_commanded():
    """effort='0' velocity='0' (Alex's own choice, adopted): the model cannot be used to command the lift."""
    for arm_package in ev.ARM_PACKAGES:
        joint = next(j for j in expand(arm_package=arm_package).findall("joint") if j.get("name") == "lift_joint")
        limit = joint.find("limit")
        assert (float(limit.get("effort")), float(limit.get("velocity"))) == (0.0, 0.0)
        assert (float(limit.get("lower")), float(limit.get("upper"))) == (0.0, LIFT_STROKE)


def test_the_lift_is_not_in_ros2_control():
    """ros2_control.xacro is arms only (ADR-003): no joint of the lift, no command interface for it."""
    text = expand_xml(arm_package="openarm", hardware_profile="mock")
    assert 'joint name="lift_joint"' in text            # the URDF joint
    import xml.etree.ElementTree as ET
    root = ET.fromstring(text)
    controlled = {j.get("name") for rc in root.findall("ros2_control") for j in rc.findall("joint")}
    assert "lift_joint" not in controlled and controlled, controlled
