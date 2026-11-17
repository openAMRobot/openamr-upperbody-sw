# SPDX-License-Identifier: MIT
"""The arm diagnostics (diagnostics.py): contact sensors added to the model, the world and the bridge, and the
arithmetic on recorded traces. No ROS import."""
import math
import os
import xml.etree.ElementTree as ET

import pytest

from conftest import package_path
from openamrobot_build1_scene import diagnostics as dx

URDF = """<robot name="r">
  <link name="a"><collision name="a_collision"><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="b"><collision><geometry><box size="1 1 1"/></geometry></collision>
                 <collision name="pad"><geometry><box size="1 1 1"/></geometry></collision></link>
  <link name="c"><collision name="c_col"><geometry><box size="1 1 1"/></geometry></collision></link>
  <joint name="ab" type="revolute"><parent link="a"/><child link="b"/><limit lower="-1" upper="2" effort="1" velocity="1"/></joint>
  <joint name="bc" type="fixed"><parent link="b"/><child link="c"/></joint>
</robot>"""


def test_collision_names_follow_sdformat():
    assert dx.sdf_collision_names(URDF, "a") == ["a_collision_collision"]
    assert dx.sdf_collision_names(URDF, "b") == ["b_collision", "pad_collision_1"]
    with pytest.raises(ValueError, match="fixed joint"):
        dx.sdf_collision_names(URDF, "c")
    with pytest.raises(ValueError, match="no link"):
        dx.sdf_collision_names(URDF, "z")


def test_contact_sensors_observe_and_change_nothing_else():
    out = dx.with_contact_sensors(URDF, ["a", "b"])
    root, before = ET.fromstring(out), ET.fromstring(URDF)
    assert [ET.tostring(e) for e in root.findall("link")] == [ET.tostring(e) for e in before.findall("link")]
    assert [ET.tostring(e) for e in root.findall("joint")] == [ET.tostring(e) for e in before.findall("joint")]
    sensors = root.findall("gazebo/sensor")
    assert [s.find("contact/collision").text for s in sensors] == ["a_collision_collision", "b_collision", "pad_collision_1"]
    assert {s.get("type") for s in sensors} == {"contact"}
    assert [s.find("topic").text for s in sensors] == ["/build1/gz/contacts/a", "/build1/gz/contacts/b", "/build1/gz/contacts/b_1"]
    assert [g.get("reference") for g in root.findall("gazebo")] == ["a", "b", "b"]


def test_world_gets_the_contact_system_once_and_sensors_on_its_models():
    item = '<model name="item"><link name="link"><collision name="body_collision"/></link></model>'
    tote = '<model name="tote"><link name="link"><collision name="front_collision"/><collision name="back_collision"/></link></model>'
    world = f"<sdf><world>{dx.PHYSICS_SYSTEM}{item}{tote}</world></sdf>"
    once = dx.world_with_contact_system(world, models=("item", "tote"))
    assert once.count(dx.CONTACT_SYSTEM) == 1 and once.index(dx.CONTACT_SYSTEM) > once.index(dx.PHYSICS_SYSTEM)
    assert dx.world_with_contact_system(once, models=("item", "tote")) == once
    root = ET.fromstring(once)
    sensors = {s.get("name"): s for s in root.iter("sensor")}
    assert [c.text for c in sensors["build1_contact_item"].findall("contact/collision")] == ["body_collision"]
    assert [c.text for c in sensors["build1_contact_tote"].findall("contact/collision")] == ["front_collision", "back_collision"]
    assert sensors["build1_contact_tote"].findtext("contact/topic") == "/build1/gz/contacts/world_tote"
    with pytest.raises(ValueError):
        dx.world_with_contact_system("<sdf><world/></sdf>")
    with pytest.raises(ValueError, match="no model"):
        dx.world_with_contact_system(f"<sdf><world>{dx.PHYSICS_SYSTEM}</world></sdf>", models=("item",))


def test_the_generated_world_takes_the_three_world_sensors(scene):
    from openamrobot_build1_scene import scene as scene_model
    out = dx.world_with_contact_system(scene_model.world_sdf(scene, "fast", sensors=False))
    names = [s.get("name") for s in ET.fromstring(out[out.index("<sdf"):]).iter("sensor")]
    assert names == ["build1_contact_item", "build1_contact_tote", "build1_contact_fixtures"] or sorted(names) == sorted(
        ["build1_contact_item", "build1_contact_tote", "build1_contact_fixtures"])


def test_contact_bridge_and_the_switch():
    entries = dx.contact_bridge(["openarm_left_ee_link1"])
    assert entries == [{"ros_topic_name": "/build1/contacts/openarm_left_ee_link1", "gz_topic_name": "/build1/gz/contacts/openarm_left_ee_link1",
                        "ros_type_name": "ros_gz_interfaces/msg/Contacts", "gz_type_name": "gz.msgs.Contacts", "direction": "GZ_TO_ROS"}]
    assert not dx.enabled({}) and not dx.enabled({dx.ENV: "0"}) and dx.enabled({dx.ENV: "1"})
    assert len(dx.contact_links()) == 18 and "openarm_right_link2" in dx.contact_links()
    with open(package_path("launch", "build1.launch.py"), encoding="utf-8") as f:
        launch = f.read()
    assert "diagnostics.enabled(os.environ)" in launch and "diagnostics.with_contact_sensors(sim_urdf, contact_links)" in launch


def test_the_combined_model_takes_a_sensor_on_every_arm_link(devices_urdf):
    out = dx.with_contact_sensors(devices_urdf, dx.contact_links())
    sensors = ET.fromstring(out).findall("gazebo/sensor")
    assert len(sensors) == 18
    for sensor, link in zip(sensors, dx.contact_links()):
        assert sensor.find("contact/collision").text == dx.sdf_collision_names(devices_urdf, link)[0]


def test_normal_and_tangential_parts():
    assert dx.normal_tangential((3.0, 4.0, 0.0), (0.0, 2.0, 0.0)) == (4.0, 3.0)
    assert dx.normal_tangential((0.0, -5.0, 0.0), (0.0, 1.0, 0.0)) == (5.0, 0.0)
    assert all(math.isnan(v) for v in dx.normal_tangential((1.0, 0.0, 0.0), (0.0, 0.0, 0.0)))


def test_required_normal_force_of_a_two_finger_pinch():
    assert math.isclose(dx.required_normal_force(0.15, 1.0), 0.15 * 9.81 / 2.0)
    assert math.isclose(dx.required_normal_force(0.15, 1.0, accel=1.0, safety=2.0), 2.0 * 0.15 * 10.81 / 2.0)


def test_slip_distance_and_onset():
    samples = [(0.0, (0.0, 0.0, -0.03)), (1.0, (0.0, 0.0, -0.031)), (2.0, (0.0, 0.0, -0.036)), (3.0, (0.0, 0.0, -0.034))]
    result = dx.slip(samples)
    assert math.isclose(result["distance"], 0.004, abs_tol=1e-12) and math.isclose(result["max"], 0.006, abs_tol=1e-12)
    assert result["onset"] == 2.0
    assert dx.slip([(0.0, (0, 0, 0)), (1.0, (0.001, 0, 0))])["onset"] is None


def test_limits_and_paths():
    assert dx.on_limit(0.7850, 0.0, 0.7854) == "upper" and dx.on_limit(0.0, 0.0, 0.7854) == "lower"
    assert dx.on_limit(0.4, 0.0, 0.7854) == ""
    assert dx.joint_limits(URDF, ["ab", "bc"]) == {"ab": (-1.0, 2.0)}
    planned = [[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]]
    assert dx.path_deviation(planned, [[0.5, 0.0], [1.0, 0.5]]) == 0.0
    assert math.isclose(dx.path_deviation(planned, [[0.5, 0.2]]), 0.2)
    assert math.isclose(dx.planned_limit_margin(planned, [(-1.0, 1.2), (-0.5, 1.1)]), 0.1)


def test_peak_vertical_acceleration():
    samples = [(t * 0.1, 0.5 * 2.0 * (t * 0.1) ** 2) for t in range(10)]
    assert math.isclose(dx.peak_vertical_accel(samples), 2.0, rel_tol=1e-9)
    assert dx.peak_vertical_accel(samples[:2]) is None


def test_the_diagnostic_scenarios_are_registered_and_launched():
    with open(package_path("CMakeLists.txt"), encoding="utf-8") as f:
        cmake = f.read()
    assert "foreach(name diag_grasp diag_reach_return)" in cmake
    for name in ("diag_grasp", "diag_reach_return"):
        assert os.path.isfile(package_path("test", f"test_{name}.py"))


def test_the_table_reads_one_run(tmp_path):
    import json
    import subprocess
    import sys
    run = tmp_path / "run1"
    run.mkdir()
    log = {"result": "FAIL", "quantities": [{"name": "finger_position_on_item", "value": 0.16},
                                             {"name": "item_height_gain_after_lift", "value": 0.0386},
                                             {"name": "retreat_left_to_park_distance_from_park", "value": 0.78}],
           "diagnostics": {"grasp": {"finger1_normal_N_mean": 9.5, "tcp_rise_m": 0.0433, "item_rise_m": 0.0386},
                           "returns": {"retreat_left_to_park": {"joint6_max_abs_effort_Nm": 7.9, "joint6_limit_hit": "upper",
                                                                "contacts_max_force_N": {"ee_link1~tote:front_collision": 30.0}}}}}
    (run / "diag_grasp.json").write_text(json.dumps(log))
    out = subprocess.run([sys.executable, package_path("scripts", "diag_table.py"), str(run)], capture_output=True, text=True, check=True).stdout
    assert "| run1 | FAIL | 0.160 | 9.50 |" in out and "0.0433" in out and "0.0386 |" in out
    assert "| run1 | retreat_left_to_park | 0.780 | joint6 7.9 | joint6:upper |" in out and "ee_link1~tote:front_collision: 30.0" in out


def test_resampling_drops_repeated_states():
    stair = [(0.00, 0.0), (0.01, 0.0), (0.02, 0.001), (0.05, 0.001), (0.06, 0.002), (0.11, 0.004)]
    assert dx.resample(stair, 0.05) == [(0.00, 0.0), (0.05, 0.001), (0.11, 0.004)]
