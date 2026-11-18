# SPDX-License-Identifier: MIT
"""EXPERIMENT (diag/spin-test scratch branch): the variant edits and the arithmetic of the turn-slip experiment."""
import math

import pytest

from openamrobot_build1_scene import spin_test, spin_variants

MODEL = """<robot name="r">
  <link name="base_footprint"/>
  <joint name="base_footprint_joint" type="fixed"><parent link="base_footprint"/><child link="base_link"/>
    <origin xyz="0 0 0.098" rpy="0 0 0"/></joint>
  <link name="base_link"><inertial><mass value="40"/></inertial></link>
  <link name="left_wheel_link"><inertial><mass value="3.45"/></inertial>
    <collision><origin xyz="0 0 0" rpy="1.5707963 0 0"/><geometry><!-- a comment --><cylinder length="0.046" radius="0.098"/></geometry></collision></link>
  <joint name="left_wheel_joint" type="continuous"><origin xyz="0 0.200 0" rpy="0 0 0"/><parent link="base_link"/><child link="left_wheel_link"/></joint>
  <link name="right_wheel_link"><inertial><mass value="3.45"/></inertial>
    <collision><origin xyz="0 0 0" rpy="1.5707963 0 0"/><geometry><cylinder length="0.046" radius="0.098"/></geometry></collision></link>
  <joint name="right_wheel_joint" type="continuous"><origin xyz="0 -0.200 0" rpy="0 0 0"/><parent link="base_link"/><child link="right_wheel_link"/></joint>
""" + "".join(f"""
  <link name="{p}_caster_link"/>
  <joint name="{p}_caster_joint" type="fixed"><origin xyz="0.27 0.2 -0.043833" rpy="0 0 0"/><parent link="base_link"/><child link="{p}_caster_link"/></joint>
  <link name="{p}_wheel_link"><collision><geometry><sphere radius="0.026"/></geometry></collision></link>
  <joint name="{p}_wheel_joint" type="fixed"><origin xyz="-0.015 0 -0.027746" rpy="0 0 0"/><parent link="{p}_caster_link"/><child link="{p}_wheel_link"/></joint>
  <gazebo reference="{p}_wheel_link"><material>Gazebo/DarkGrey</material><mu1>0.0</mu1><mu2>0.0</mu2></gazebo>""" for p in ("fl", "fr", "bl", "br")) + """
  <gazebo reference="left_wheel_link">
    <material>Gazebo/DarkGrey</material>
    <mu1>1.5</mu1>
    <mu2>0.5</mu2>
    <fdir1>1 0 0</fdir1>
  </gazebo>
  <gazebo reference="right_wheel_link"><mu1>1.5</mu1><mu2>0.5</mu2><fdir1>1 0 0</fdir1></gazebo>
</robot>"""


def test_baseline_unchanged():
    out, notes = spin_variants.apply(MODEL, "baseline")
    assert out == MODEL and notes == ["no edit"]


def test_unknown_variant_refused():
    with pytest.raises(ValueError):
        spin_variants.apply(MODEL, "D_unknown")


def test_geometry():
    g = spin_variants.contact_geometry(MODEL)
    assert g["track"] == pytest.approx(0.4) and g["radius"] == 0.098
    assert g["left_wheel_link_bottom_z"] == pytest.approx(0.0)
    assert g["fl_wheel_link_bottom_z"] == pytest.approx(0.000421)


def test_a_iso():
    out, _ = spin_variants.apply(MODEL, "A_iso")
    assert "fdir1" not in out and out.count("<mu2>1.5</mu2>") == 2 and "<mu2>0.5</mu2>" not in out


def test_a_axial():
    out, _ = spin_variants.apply(MODEL, "A_axial")
    assert out.count("<fdir1>0 0 1</fdir1>") == 2 and out.count("<mu1>0.5</mu1>") == 2 and out.count("<mu2>1.5</mu2>") == 2


def test_b_sphere_and_thin():
    out, _ = spin_variants.apply(MODEL, "B_sphere")
    assert out.count('<sphere radius="0.098"/>') == 2 and 'length="0.046"' not in out
    out, _ = spin_variants.apply(MODEL, "B_thin")
    assert out.count('length="0.004"') == 2


def test_c_coplanar_and_friction():
    out, _ = spin_variants.apply(MODEL, "C_coplanar")
    g = spin_variants.contact_geometry(out)
    assert all(abs(g[f"{p}_wheel_link_bottom_z"]) < 1e-9 for p in ("fl", "fr", "bl", "br"))
    out, _ = spin_variants.apply(MODEL, "C_friction")
    assert out.count("<mu1>0.05</mu1>") == 4 and out.count("<mu2>0.05</mu2>") == 4


def test_all():
    out, notes = spin_variants.apply(MODEL, "ALL")
    g = spin_variants.contact_geometry(out)
    assert "fdir1" not in out and '<sphere radius="0.098"/>' in out and abs(g["fl_wheel_link_bottom_z"]) < 1e-9
    assert len(notes) == 8


def test_quantized_rates():
    # 0.25 rad/s -> 4.87 r/min per wheel -> 5 r/min -> 0.2566 rad/s, as the host test client rounds
    assert spin_test.quantized_rate(0.25, 0.098, 0.4) == pytest.approx(5 * 2 * math.pi / 60 * 0.098 / 0.2)
    assert spin_test.quantized_rate(-0.25, 0.098, 0.4) < 0
    assert spin_test.quantized_speed(0.2, 0.098) == pytest.approx(19 * 2 * math.pi / 60 * 0.098)


def test_contacts_and_split():
    pose = (1.0, 2.0, math.pi / 2)            # base x axis along world +y: base +y is world -x
    contacts = [("openamrobot::left_wheel_link::left_wheel_link_collision", [((0.8, 2.0, 0.0), (0, 0, 1), (0, 0, 300.0))]),
                ("openamrobot::right_wheel_link::right_wheel_link_collision", [((1.21, 2.0, 0.0), (0, 0, 1), (0, 0, 300.0))]),
                ("openamrobot::base_footprint::base_footprint_fixed_joint_lump__fl_wheel_link_collision_7",
                 [((0.8, 2.3, 0.0), (0, 0, 1), (0, 0, -100.0))])]
    load, lat, names = spin_test.contact_sample(contacts, pose)
    assert load["caster"] == pytest.approx(100.0) and load["left"] == pytest.approx(300.0)
    summary = spin_test.summarize_contacts([(load, lat)])
    assert summary["effective_track_m"] == pytest.approx(0.41)
    assert summary["caster_share"] == pytest.approx(100.0 / 700.0)
    s = spin_test.split("turn", math.pi, 3.20, 3.20, 3.15)
    assert s["residual"] == pytest.approx(0.05) and s["drive_chain"] == pytest.approx(0.0) and s["contact"] == pytest.approx(0.05)
