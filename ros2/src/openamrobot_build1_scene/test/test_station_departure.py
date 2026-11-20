# SPDX-License-Identifier: MIT
"""The Baseline 1 retreat options (openamrobot_build1_scene/station_departure.py, Proposed; Alex decides), without ROS.

Each option against two Nav2 configurations: the platform integration branch today (backup, drive_on_heading and
assisted_teleop registered, min_vel_x -0.05, smoother min_velocity[0] -0.20) and platform-sw #90's behaviour set
(spin and wait only, both clamps 0). Then the bounds of the station departure command, the forward option's
clearance rule, the reverse-speed measure, and that the default stays backup (nothing is enabled).
Simulation evidence only.
"""
import ast
import copy
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, PACKAGE)
from openamrobot_build1_scene import station_departure as sd  # noqa: E402

CONFIG = sd.load()
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE)))
STATION = {"plugin": "nav2_graceful_controller::GracefulController", "allow_backward": True, "initial_rotation": True}
FOLLOW = {"plugin": "nav2_rotation_shim_controller::RotationShimController",
          "primary_controller": "dwb_core::DWBLocalPlanner"}
TREE = '<root><BehaviorTree ID="T"><RoundRobin><Spin spin_dist="1.57"/><Wait wait_duration="5"/></RoundRobin>' \
       '</BehaviorTree></root>'


def integration_today():
    """The Nav2 values of openamr-platform-sw integration/baseline1 at fb25be1 that matter here."""
    return {
        "behavior_server": {"ros__parameters": {
            "behavior_plugins": ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"],
            "spin": {"plugin": "nav2_behaviors::Spin"}, "backup": {"plugin": "nav2_behaviors::BackUp"},
            "drive_on_heading": {"plugin": "nav2_behaviors::DriveOnHeading"},
            "assisted_teleop": {"plugin": "nav2_behaviors::AssistedTeleop"},
            "wait": {"plugin": "nav2_behaviors::Wait"}}},
        "controller_server": {"ros__parameters": {
            "controller_plugins": ["FollowPath", "StationApproach"], "FollowPath": dict(FOLLOW, min_vel_x=-0.05),
            "StationApproach": dict(STATION)}},
        "velocity_smoother": {"ros__parameters": {"min_velocity": [-0.20, 0.0, -0.5]}}}


def pr90_set():
    """The same with #90's behaviour set (nav/baseline1-no-reverse 0aa8e14)."""
    params = integration_today()
    b = params["behavior_server"]["ros__parameters"]
    b["behavior_plugins"] = ["spin", "wait"]
    for name in ("backup", "drive_on_heading", "assisted_teleop"):
        del b[name]
    params["controller_server"]["ros__parameters"]["FollowPath"]["min_vel_x"] = 0.0
    params["velocity_smoother"]["ros__parameters"]["min_velocity"] = [0.0, 0.0, -0.5]
    return params


TREES = {"navigate_to_pose.xml": TREE, "navigate_to_station.xml": TREE}


# ------------------------------------------------------------------ the option table
@pytest.mark.parametrize("name, today, pr90", [("backup", True, False), ("departure", False, True),
                                               ("forward", True, True)])
def test_each_option_against_today_and_the_pr90_behaviour_set(name, today, pr90):
    assert sd.allowed(name, CONFIG, integration_today(), TREES)[0] is today
    assert sd.allowed(name, CONFIG, pr90_set(), TREES)[0] is pr90


def test_the_departure_is_refused_while_any_reverse_path_is_enabled():
    ok, reasons = sd.allowed("departure", CONFIG, integration_today(), TREES)
    assert not ok and len(reasons) == 5      # three behaviours, min_vel_x, the smoother
    params = pr90_set()
    params["controller_server"]["ros__parameters"]["StationApproach"]["initial_rotation"] = False
    reason = "StationApproach allow_backward without initial_rotation"
    assert sd.allowed("departure", CONFIG, params, TREES) == (False, [reason])
    backup_tree = {"t.xml": TREE.replace("<Wait", '<BackUp backup_dist="0.3"/><Wait')}
    assert sd.allowed("departure", CONFIG, pr90_set(), backup_tree) == (False, ["t.xml calls BackUp"])


def test_only_the_forward_option_never_reverses():
    assert [n for n in sd.OPTIONS if not sd.TRADEOFFS[n]["reverses"]] == ["forward"]
    assert set(sd.TRADEOFFS) == set(sd.OPTIONS) == set(CONFIG["options"])


# ------------------------------------------------------------------ the departure command
def test_the_departure_backs_up_at_its_speed_and_stops_at_its_distance():
    assert sd.departure_command(CONFIG, "retreat", 0.0, 0.0) == pytest.approx(-0.10)
    assert sd.departure_command(CONFIG, "retreat", 0.29, 2.9) == pytest.approx(-0.10)
    assert sd.departure_command(CONFIG, "retreat", 0.30, 3.0) == 0.0
    assert sd.departure_command(CONFIG, "retreat", 0.10, CONFIG["departure"]["max_time_sim_s"]) == 0.0


def test_the_departure_moves_only_in_the_retreat_phase():
    for phase in ("spawn", "nav_to_station", "stop_authorize", "arm_approach", None):
        assert sd.departure_command(CONFIG, phase, 0.0, 0.0) == 0.0


def test_a_config_beyond_the_caps_commands_nothing():
    for key, value in (("distance_m", 0.31), ("speed_m_s", 0.15)):
        config = copy.deepcopy(CONFIG)
        config[key] = value
        assert sd.departure_plan_problems(config) and sd.departure_command(config, "retreat", 0.0, 0.0) == 0.0
        assert not sd.allowed("departure", config, pr90_set(), TREES)[0]
    config = copy.deepcopy(CONFIG)
    config["departure"]["simulation_only"] = False
    assert sd.departure_command(config, "retreat", 0.0, 0.0) == 0.0


# ------------------------------------------------------------------ the forward option
def test_the_forward_option_needs_the_measured_clearance_and_spin():
    assert CONFIG["forward"]["turn_clearance_measured_m"] == 0.141
    config = copy.deepcopy(CONFIG)
    config["forward"]["turn_clearance_measured_m"] = 0.04
    assert sd.allowed("forward", config, pr90_set(), TREES) == (False, ["turn clearance 0.04 m below 0.05 m"])
    params = pr90_set()
    params["behavior_server"]["ros__parameters"]["behavior_plugins"] = ["wait"]
    assert sd.allowed("forward", CONFIG, params, TREES) == (False, ["spin is not registered in behavior_server"])


# ------------------------------------------------------------------ measures
def test_the_reverse_speed_and_along_heading_measures():
    back = [(0.1 * i, -0.01 * i, 0.0, 0.0) for i in range(10)]           # 0.1 m/s backwards on heading 0
    assert sd.reverse_speed(back) == pytest.approx(0.1)
    ahead = [(0.1 * i, 0.0, 0.02 * i, 1.5707963) for i in range(10)]     # 0.2 m/s forwards on heading +y
    assert sd.reverse_speed(ahead) == pytest.approx(0.0, abs=1e-6)
    assert sd.along((0.0, 0.0, 1.5707963), (0.0, -0.3, 1.5707963)) == pytest.approx(-0.3)


# ------------------------------------------------------------------ nothing is enabled
def test_the_default_is_backup_and_the_workflow_does_not_choose_another():
    assert sd.option(CONFIG, environ={}) == "backup"
    assert sd.option(CONFIG, environ={"BASELINE1_RETREAT": "forward"}) == "forward"
    with pytest.raises(ValueError):
        sd.option(CONFIG, environ={"BASELINE1_RETREAT": "reverse"})
    workflow = open(os.path.join(REPO, ".github", "workflows", "baseline1-acceptance.yml"), encoding="utf-8").read()
    assert "BASELINE1_RETREAT" not in workflow


def test_the_mission_retreat_hands_over_only_for_another_option():
    tree = ast.parse(open(os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py"), encoding="utf-8").read())
    retreat = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "retreat")
    source = ast.unparse(retreat)
    assert "if chosen != 'backup':" in source and "station_departure.run(rig, log, chosen, retreat_config)" in source
    assert source.index("station_departure.run") < source.index('"/backup"' if '"/backup"' in source else "'/backup'")
