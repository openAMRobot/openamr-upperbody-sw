# SPDX-License-Identifier: MIT
"""The base's wheel and caster friction reaches the converted SDF in the gazebo profile (base_contact_gazebo.xacro).

platform-sw's gazebo_control.xacro writes it as a raw <surface> under <gazebo reference>, which the URDF -> SDF conversion drops: the
drive wheels then had default friction and the four casters (lumped into base_footprint) dragged at mu 1, and in DART the base stood
still in ground truth while odom reported metres (night-shift 2026-11-09 smoke). Here: the values are platform-sw's own, they are
present only in the gazebo profile, and `gz sdf -p` (when installed) puts them on every wheel collision."""
import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import pytest

from conftest import expand

DRIVE = {"mu1": "1.5", "mu2": "0.5", "fdir1": "1 0 0"}      # platform-sw gazebo_control.xacro, left/right_wheel_link
CASTER = {"mu1": "0.0", "mu2": "0.0"}                       # platform-sw gazebo_control.xacro, caster_gazebo macro
CASTERS = ("fl", "fr", "bl", "br")


def gazebo_refs(root):
    return {g.get("reference"): {c.tag: (c.text or "").strip() for c in g} for g in root.findall("gazebo") if g.get("reference")}


def contact(refs, link):
    return {k: v for k, v in refs.get(link, {}).items() if k in ("mu1", "mu2", "fdir1")}


def test_gazebo_profile_restates_platform_sws_wheel_and_caster_friction():
    refs = gazebo_refs(expand(hardware_profile="gazebo"))
    for side in ("left", "right"):
        assert contact(refs, f"{side}_wheel_link") == DRIVE
    for prefix in CASTERS:
        assert contact(refs, f"{prefix}_wheel_link") == CASTER


def test_the_values_match_platform_sws_own_file():
    from ament_index_python.packages import get_package_share_directory
    text = open(os.path.join(get_package_share_directory("openamrobot_description"), "urdf", "gazebo_control.xacro")).read()
    for needle in ("<mu>1.5</mu>", "<mu2>0.5</mu2>", "<fdir1>1 0 0</fdir1>", "<mu>0.0</mu>", "<mu2>0.0</mu2>"):
        assert needle in text, f"platform-sw's gazebo_control.xacro no longer has {needle}: re-check base_contact_gazebo.xacro"


@pytest.mark.parametrize("profile", ["mock", "real"])
def test_other_profiles_carry_no_contact_override(profile):
    refs = gazebo_refs(expand(hardware_profile=profile))
    for link in ["left_wheel_link", "right_wheel_link"] + [f"{p}_wheel_link" for p in CASTERS]:
        assert contact(refs, link) == {}, (profile, link)


@pytest.mark.skipif(shutil.which("gz") is None, reason="gz (Gazebo Harmonic) not installed")
def test_the_converted_sdf_has_the_friction_on_every_wheel_collision():
    with tempfile.NamedTemporaryFile("w", suffix=".urdf", delete=False) as f:
        f.write(ET.tostring(expand(hardware_profile="gazebo"), encoding="unicode"))
    try:
        sdf = subprocess.run(["gz", "sdf", "-p", f.name], capture_output=True, text=True, timeout=120).stdout
    finally:
        os.unlink(f.name)
    model = ET.fromstring(sdf).find("model")
    seen = {}
    for link in model.findall("link"):
        for c in link.findall("collision"):
            name = c.get("name")
            for wheel in ["left_wheel_link", "right_wheel_link"] + [f"{p}_wheel_link" for p in CASTERS]:
                if wheel in name:
                    ode = c.find("surface/friction/ode")
                    seen[wheel] = None if ode is None else (float(ode.findtext("mu")), float(ode.findtext("mu2")))
    assert seen == {"left_wheel_link": (1.5, 0.5), "right_wheel_link": (1.5, 0.5),
                    **{f"{p}_wheel_link": (0.0, 0.0) for p in CASTERS}}
