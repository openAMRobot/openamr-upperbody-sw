# SPDX-License-Identifier: MIT
"""T01: the expansion matrix expands (14 expansions, one factor at a time around option B's baseline), and what it does not expand.

The matrix is evidence_common.expansion_matrix: every base_plate x arm_package on `mock`, every other head_pitch and base_tilt on
`openarm`/`mock`, and `openarm` on `gazebo` and `real`. The full product of the four configuration IDs is option B's own test
(test_upperbody_configurations.py, on the standalone model); this check is about the combined robot.
"""
import re

import pytest
import xacro

import evidence_common as ev
from conftest import (ARG_DEFAULTS, AXES, BASELINE, COMBINED, HARDWARE_PROFILES, UPPERBODY, expand, expand_xml, read_text)

MATRIX_ROWS = ev.expansion_matrix(UPPERBODY)
LABELS = [ev.row_label(r) for r in MATRIX_ROWS]


def test_t01_matrix_is_fourteen_distinct_expansions():
    assert len(MATRIX_ROWS) == 14
    assert len(set(LABELS)) == 14
    assert {r["hardware_profile"] for r in MATRIX_ROWS} == set(HARDWARE_PROFILES)
    assert all(ev.column_is_buildable(UPPERBODY, r["column"]) for r in MATRIX_ROWS)


def test_t01_matrix_varies_one_factor_at_a_time_around_the_baseline():
    for row in MATRIX_ROWS:
        differing = [k for k in ev.CONFIG_KEYS if row[k] != BASELINE[k]]
        assert len(differing) <= 1, f"{ev.row_label(row)} differs from the baseline in {differing}"
    for key in ("base_plate", "head_pitch", "base_tilt"):
        assert {r[key] for r in MATRIX_ROWS} == set(AXES[key]), f"every {key} value must appear"


@pytest.mark.parametrize("row", MATRIX_ROWS, ids=LABELS)
def test_t01_expands(row):
    root = expand(**ev.row_mappings(row))
    assert root.tag == "robot" and root.get("name") == "openamrobot"
    names = [e.get("name") for e in root.findall("link")]
    for link in ("base_footprint", "base_link", "lift_column_base_link", "lift_carriage_link", "openarm_left_base_link",
                 "openarm_right_base_link", "head_camera_optical_frame"):
        assert link in names, f"{link} missing from {ev.row_label(row)}"


@pytest.mark.parametrize("row", MATRIX_ROWS, ids=LABELS)
def test_t01_there_is_no_mast_anywhere(row):
    """No link, joint or attribute value mentions a mast (the file's own header comment may: ElementTree drops comments)."""
    root = expand(**ev.row_mappings(row))
    hits = sorted({value for element in root.iter() for value in list(element.attrib.values()) + [element.text or ""]
                   if re.search(r"mast", value, re.I)})
    assert not hits, f"a mast is left in the expansion: {hits[:5]}"


def test_t01_hardware_profiles_expand():
    for profile in HARDWARE_PROFILES:
        assert expand(hardware_profile=profile).find("link[@name='base_link']") is not None


def test_t01_planar_flag_accepted_and_the_planar_joint_is_on_base_footprint():
    root = expand(enable_planar_base="true")
    joint = root.find("joint[@name='base_planar_joint']")
    assert joint is not None and joint.find("child").get("link") == "base_footprint"


# --- the defaults are option B's baseline, and a configuration without supplier data is refused ---------------------

def test_t01_xacro_arg_defaults_are_option_bs_baseline():
    text = read_text(COMBINED)
    for key in ev.CONFIG_KEYS:
        match = re.search(rf'<xacro:arg name="{key}" default="([^"]+)"', text)
        assert match, f"combined_robot.xacro declares no arg {key}"
        assert match.group(1) == BASELINE[key], f"arg {key} defaults to {match.group(1)}, option B's baseline is {BASELINE[key]}"
    assert ARG_DEFAULTS["column"] == "dold_v4_350"


def test_t01_the_default_expansion_is_the_baseline_expansion():
    assert expand_xml() == expand_xml(**ev.row_mappings(ev.baseline_row(UPPERBODY)))


def test_t01_a_column_with_null_supplier_data_does_not_expand():
    with pytest.raises(xacro.XacroException) as err:
        expand_xml(column="tl3_400")
    msg = str(err.value)
    assert "tl3_400" in msg and ("pending supplier data" in msg or "is null" in msg), msg
