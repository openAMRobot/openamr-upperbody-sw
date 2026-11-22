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


def test_pose_jitter_between_close_samples_is_not_read_as_reverse():
    # A turn in place sampled every few ms of Gazebo time, with 1 mm of pose jitter: not a reverse.
    turn = [(0.004 * i, 0.001 * (i % 2), 0.0, 0.01 * i) for i in range(200)]
    assert sd.reverse_speed(turn) < 0.01
    # A real 0.1 m/s reverse sampled the same way is still found.
    back = [(0.004 * i, -0.0004 * i, 0.0, 0.0) for i in range(200)]
    assert sd.reverse_speed(back) == pytest.approx(0.1)


# ------------------------------------------------------------------ nothing is enabled
def test_the_default_is_forward_and_the_workflow_sets_and_declares_it():
    """Turn-and-forward retreat by default (Alex 20 Nov 2026); the acceptance workflow sets it and every leg declares it."""
    assert sd.option(CONFIG, environ={}) == "forward"
    assert sd.option(CONFIG, environ={"BASELINE1_RETREAT": "backup"}) == "backup"
    with pytest.raises(ValueError):
        sd.option(CONFIG, environ={"BASELINE1_RETREAT": "reverse"})
    workflow = open(os.path.join(REPO, ".github", "workflows", "baseline1-acceptance.yml"), encoding="utf-8").read()
    assert "\n  BASELINE1_RETREAT: forward\n" in workflow
    assert '"retreat": e["BASELINE1_RETREAT"]' in workflow
    from openamrobot_build1_scene import baseline1_gates
    assert "retreat" in baseline1_gates.load()["evidence"]["declared_settings"]


def test_the_default_works_on_the_no_reverse_configuration():
    ok, reasons = sd.allowed(sd.option(CONFIG, environ={}), CONFIG, pr90_set(), TREES)
    assert ok and not reasons
    assert sd.reverse_paths(pr90_set(), TREES) == []


def test_the_mission_retreat_hands_over_only_for_another_option():
    tree = ast.parse(open(os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py"), encoding="utf-8").read())
    retreat = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "retreat")
    source = ast.unparse(retreat)
    assert "if chosen != 'backup':" in source and "station_departure.run(rig, log, chosen, retreat_config, pose=lambda: map_pose(rig))" in source
    # backup sends no goal: the no-reverse behaviour set (openamr-platform-sw #178) has no /backup server.
    assert "send_goal" not in source and "BackUp" not in source
    assert "log.require(False, 'BASELINE1_RETREAT=backup:" in source


# ------------------------------------------------------------------ no consumer calls a removed Nav2 behaviour
REMOVED_ACTIONS = ("/backup", "/drive_on_heading", "/assisted_teleop")
REMOVED_TYPES = ("BackUp", "DriveOnHeading", "AssistedTeleop")
NON_PYTHON = __import__("re").compile(
    r"<(BackUp|DriveOnHeading|AssistedTeleop)\b|nav2_msgs/(action/)?(BackUp|DriveOnHeading|AssistedTeleop)\b"
    r"|nav2_behaviors::(BackUp|DriveOnHeading|AssistedTeleop)\b|(?<![\w/])/(backup|drive_on_heading|assisted_teleop)(?![\w/])")
SCANNED = (".py", ".xml", ".yaml", ".yml", ".json", ".sh", ".launch", ".repos", ".cfg", ".txt")


def removed_behaviour_calls(root):
    """Every place under `root` that would call a Nav2 behaviour openamr-platform-sw #178 removes (backup,
    drive_on_heading, assisted_teleop; spin and wait stay): Python imports of their action types and the action names
    as string constants, and in launch files, behaviour trees, configs and workflows the tree nodes, action types,
    plugin classes and action names. Documentation (.md) is history and is not scanned; this file's fixture trees and
    station_departure's detection tables are plain strings, neither an import nor an action name."""
    found = []
    for directory, dirs, files in os.walk(root):
        dirs[:] = [d for d in dirs if d not in (".git", "build", "install", "log", "__pycache__")]
        for name in files:
            path = os.path.join(directory, name)
            if not name.endswith(SCANNED) or os.path.samefile(path, __file__):
                continue
            text = open(path, encoding="utf-8", errors="replace").read()
            where = os.path.relpath(path, root)
            if name.endswith(".py"):
                try:
                    tree = ast.parse(text, path)
                except SyntaxError:
                    # Not valid Python 3 (a vendored script, a Python 2 remnant, a generated
                    # stub) - it cannot contain a real import/constant this AST walk is built
                    # to find, so it is not a removed-behaviour call by this test's own
                    # definition. Fall back to the same textual line-scan the non-.py branch
                    # below already uses, instead of letting the exception abort the whole
                    # scan over an unrelated file.
                    for number, line in enumerate(text.splitlines(), 1):
                        if NON_PYTHON.search(line):
                            found.append(f"{where}:{number} {line.strip()[:80]}")
                    continue
                for node in ast.walk(tree):
                    if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("nav2_msgs"):
                        found += [f"{where}:{node.lineno} imports {a.name}" for a in node.names if a.name in REMOVED_TYPES]
                    if isinstance(node, ast.Constant) and node.value in REMOVED_ACTIONS:
                        found.append(f"{where}:{node.lineno} names {node.value}")
            else:
                for number, line in enumerate(text.splitlines(), 1):
                    if NON_PYTHON.search(line):
                        found.append(f"{where}:{number} {line.strip()[:80]}")
    return found


def test_no_upperbody_code_calls_a_removed_nav2_behaviour():
    """openamr-platform-sw #178 keeps spin and wait only. scn01, scn06 and scn07 drive with a NavigateToPose goal
    straight ahead (scenarios._send_drive), the retreat turns and drives forward."""
    assert removed_behaviour_calls(REPO) == []


def test_the_removed_behaviour_scan_finds_a_caller(tmp_path):
    (tmp_path / "a.py").write_text('from nav2_msgs.action import DriveOnHeading\nrig.send_goal("/backup")\n')
    (tmp_path / "b.xml").write_text('<BackUp backup_dist="0.3"/>\n')
    (tmp_path / "c.yml").write_text("run: ros2 action send_goal /drive_on_heading nav2_msgs/action/DriveOnHeading\n")
    (tmp_path / "d.py").write_text('TOPICS = ["/backup/_action/status"]\n')
    assert len(removed_behaviour_calls(str(tmp_path))) == 4


# ------------------------------------------------------------------ the forward goal is built in the frame it is sent in
def test_the_forward_goal_comes_from_the_map_pose():
    """Diagnostic run 44800204846: the goal was built from ground truth and sent in map, 0.05 m off; the base cannot
    reverse to take that back and FollowPath aborted ("Failed to make progress")."""
    assert sd.forward_goal((1.0, 2.0, 1.5707963267948966), 0.3) == pytest.approx((1.0, 2.3, 1.5707963267948966))
    run = ast.unparse(next(n for n in ast.parse(open(sd.__file__, encoding="utf-8").read()).body
                           if isinstance(n, ast.FunctionDef) and n.name == "run"))
    assert "here = pose() if pose is not None else None" in run
    assert "gx, gy, gyaw = forward_goal(here or turned, f['forward_m'])" in run
    mission = open(os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py"), encoding="utf-8").read()
    assert "station_departure.run(rig, log, chosen, retreat_config, pose=lambda: map_pose(rig))" in mission


def test_the_forward_retreat_reverse_limit_is_the_rc_criterion():
    """RC C7 (Alex, 21 Nov 2026): peak reverse speed over 0.25 s windows at most 0.010 m/s, unchanged. The mission
    check must not accept more: diagnostic run 44800204846 read 0.01291 m/s and the old 0.02 limit reported it ok."""
    import json
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, "..", "config", "baseline1_mission_options.json")) as fh:
        text = fh.read()
    assert '"max_reverse_speed_m_s": 0.01,' in text
    assert 0.01291 > 0.01  # the measured value fails the unchanged limit
    json.loads(text)
