# SPDX-License-Identifier: MIT
"""Shelf clearance of an in-place half turn at the work-shelf station (openamr-platform-sw #167 from 99ec17f).

A target behind the robot is corrected by turning to it first, up to a half turn in place next to the shelf. This
measures, on the expanded simulation model with both arms in `park` (the SRDF pose the mission drives in, a placeholder
carry pose) and the lift at L1000, the smallest distance from any collision shape of the robot to the work shelf and the
objects on it, over a turn of up to 180 degrees in either direction (turn_clearance.py). Four cases: the open shelf of
config/scene.yaml (boards and corner posts) and a closed shelf (one solid box over the unit), each at the station and
0.088 m past it towards the shelf (the largest station error of #164 without the marker correction, run 44409258260).
The values go to the log and to BUILD1_LOG_DIR/station_turn_clearance.json when that is set.

Geometry only (no controller error, no dynamics); simulation-model evidence, not a robot measurement. No ROS graph.
"""
import json
import math
import os
import xml.etree.ElementTree as ET

import pytest
from ament_index_python.packages import get_package_share_directory

from conftest import expand, package_path
from openamrobot_build1_scene import scene as scene_model
from openamrobot_build1_scene import turn_clearance as tc

OVERSHOOT_M = 0.088      # #164 alone, max station error over 5 trials, openamr-platform-sw run 44409258260


def resolve(uri):
    package, relative = uri[len("package://"):].split("/", 1)
    return os.path.join(get_package_share_directory(package), relative)


def named_state(group, name):
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_moveit"), "config", "openamrobot.srdf")
    for state in ET.parse(path).getroot().findall("group_state"):
        if state.get("group") == group and state.get("name") == name:
            return {joint.get("name"): float(joint.get("value")) for joint in state.findall("joint")}
    raise KeyError(f"{group}/{name}")


@pytest.fixture(scope="module")
def model():
    urdf = expand(devices="true", finger_follower="true")
    positions = dict(named_state("both_arms", "park"), **named_state("lift", "L1000"))
    points, names = tc.robot_points(urdf, positions, resolve)
    scene = scene_model.load(package_path("config", "scene.yaml"))
    return urdf, points, names, scene


def test_the_turn_axis_is_the_base_footprint_origin(model):
    """Both drive wheels sit on the base_link y axis, base_link straight above base_footprint: an in-place turn of a
    differential drive turns about the base_footprint origin, which the sweep assumes."""
    urdf = model[0]
    poses = tc.link_poses(urdf, {})
    for wheel in ("left_wheel_link", "right_wheel_link"):
        x, y, _ = poses[wheel][:3, 3]
        assert abs(x) < 1e-9 and abs(y) > 0.1, wheel


def test_the_sweep_finds_contact_when_the_robot_stands_too_close(model):
    _urdf, points, names, scene = model
    result = tc.sweep(points, names, tc.shelf_prisms(scene, solid=True), scene_model.station(scene, "work_shelf"),
                      overshoot=0.40, step_deg=2.0)
    assert result["min_clearance_m"] < 0.0, result


def test_a_half_turn_at_the_station_keeps_clear_of_the_shelf(model):
    _urdf, points, names, scene = model
    station = scene_model.station(scene, "work_shelf")
    results = {}
    for shelf in ("open", "solid"):
        prisms = tc.shelf_prisms(scene, solid=shelf == "solid")
        for overshoot in (0.0, OVERSHOOT_M):
            results[f"{shelf}_overshoot_{overshoot}"] = tc.sweep(points, names, prisms, station, turn=math.pi,
                                                                 step_deg=1.0, overshoot=overshoot)
    print(json.dumps(results, indent=2, sort_keys=True))
    if os.environ.get("BUILD1_LOG_DIR"):
        os.makedirs(os.environ["BUILD1_LOG_DIR"], exist_ok=True)
        with open(os.path.join(os.environ["BUILD1_LOG_DIR"], "station_turn_clearance.json"), "w") as f:
            json.dump(results, f, indent=2, sort_keys=True)
    for case, result in results.items():
        assert result["min_clearance_m"] > 0.0, (case, result)
