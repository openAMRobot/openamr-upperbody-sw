# SPDX-License-Identifier: MIT
"""T05: shoulder height per lift stop. The floor-relative height of openarm_{left,right}_base_link equals the stop's shoulder height.

The shoulder height is the position of lift_joint, not an argument (P15; it was `mast_id` in A6). Option B's three calibration  # decision-allow: CONFIG-IDS historical: the shoulder height was mast_id in A6
stops L1000 / L1175 / L1350 are joint positions (0, 0.175, 0.350 m at the DOLD V4 350's 0.350 m stroke), shoulder heights
1.000 / 1.175 / 1.350 m from the floor = base_footprint. The height is computed by forward kinematics through the lift joint,
independently of the xacro's own sums, and from the floor frame, so it does not depend on any datum value: the three-way
identity below survives the datums being replaced.

Calibration IDs are asserted NOT to be configuration IDs: no xacro arg and no input of the configuration id is a stop.
"""
import re

import numpy as np
import pytest

import evidence_common as ev
from conftest import BASELINE, COMBINED, LIFT_STROKE, STOPS, UPPERBODY, expand, read_text, xacro_files
from urdf_utils import zero_pose_poses

TOLERANCE = 0.001  # m
SIDES = ("left", "right")


def shoulder_heights(root, lift_position):
    """{side: height of openarm_<side>_base_link above the floor (base_footprint z 0)} with lift_joint at `lift_position`."""
    poses = zero_pose_poses(root, {"lift_joint": lift_position})
    return {side: float(poses[f"openarm_{side}_base_link"][2, 3]) for side in SIDES}


@pytest.mark.parametrize("stop", list(STOPS))
@pytest.mark.parametrize("arm_package", ev.ARM_PACKAGES)
def test_t05_shoulder_height_matches_the_lift_stop(stop, arm_package):
    position = ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop)
    heights = shoulder_heights(expand(arm_package=arm_package), position)
    for side, height in heights.items():
        assert abs(height - STOPS[stop]) <= TOLERANCE, f"{stop} {side}: {height:.4f} m, expected {STOPS[stop]:.4f} m"


@pytest.mark.parametrize("base_plate", UPPERBODY["configuration_axes"]["base_plate"])
def test_t05_the_shoulder_height_does_not_depend_on_the_base_plate(base_plate):
    """The two base-plate positions move the column fore-aft (x +0.060 / +0.110), never up."""
    for stop in STOPS:
        position = ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop)
        for height in shoulder_heights(expand(base_plate=base_plate), position).values():
            assert abs(height - STOPS[stop]) <= TOLERANCE


def test_t05_the_stops_are_inside_the_joint_range_in_order_and_span_it():
    positions = [ev.lift_joint_position(UPPERBODY, BASELINE["column"], s) for s in STOPS]
    assert positions == sorted(positions) and len(set(positions)) == len(STOPS)
    assert positions[0] == pytest.approx(0.0, abs=1e-9) and positions[-1] == pytest.approx(LIFT_STROKE, abs=1e-9)


def test_t05_stops_are_calibration_ids_not_configuration_ids():
    """ADR-010 had the lift stops as configuration IDs and was wrong; option B's file says calibration, explicitly."""
    assert "lift_calibration_stops" in UPPERBODY and set(STOPS) == {"L1000", "L1175", "L1350"}
    assert not set(STOPS) & {v for values in UPPERBODY["configuration_axes"].values() for v in values}
    text = read_text(COMBINED)
    assert not re.search(r'<xacro:arg name="(lift_stop|lift|stop|calibration)', text)
    assert ev.ROW_KEYS == ("column", "base_plate", "head_pitch", "base_tilt", "arm_package", "hardware_profile")


def test_t05_left_and_right_have_one_shoulder_height():
    """One lift, one shoulder height for both arms (option B declares both mounts at the same z)."""
    for stop in STOPS:
        left, right = shoulder_heights(expand(), ev.lift_joint_position(UPPERBODY, BASELINE["column"], stop)).values()
        assert left == pytest.approx(right, abs=1e-12)


# --- nothing here may read the OpenArm *body* reference points (ADR-008 does not install the body) ----------------

FORBIDDEN = [
    ("body/struct/reference_points", re.compile(r"body/struct/reference_points")),
    ("0.698 (the OpenArm body's arm-mount height)", re.compile(r"(?<![\d.])0?\.698")),
]


def forbidden_hits(text):
    return [label for label, pattern in FORBIDDEN if pattern.search(text)]


def test_t05_xacro_does_not_read_upstream_body_reference_points():
    hits = {}
    for path in xacro_files():
        with open(path) as f:
            found = forbidden_hits(f.read())     # raw text, comments included: the strictest reading
        if found:
            hits[path] = found
    assert not hits, f"upstream body reference point used: {hits}"


@pytest.mark.parametrize("text,expected", [
    ('<xacro:property name="z" value="0.698"/>', 1),
    ("value=.698", 1),
    ("$(find openarm_description)/config/body/struct/reference_points.yaml", 1),
    ("0.698 and body/struct/reference_points", 2),
    ("10.698 0.6 1.0865 0.0698", 0),
])
def test_t05_forbidden_pattern_matcher(text, expected):
    assert len(forbidden_hits(text)) == expected
