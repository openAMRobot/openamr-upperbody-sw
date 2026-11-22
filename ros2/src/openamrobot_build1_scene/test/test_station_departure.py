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
import math
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


def test_reverse_speed_by_phase_does_not_read_a_window_across_the_turn_and_the_drive():
    """Proposed measurement change (proposal/baseline1-measurement-retreat-phases, for the gate owner). The end of a
    half turn sampled at heading pi/2 and a drive along -pi/2 that starts there: in one series the window from the
    last turn sample to the drive is projected on the turn heading and reads as backwards; per phase nothing is."""
    import math as m
    from openamrobot_build1_scene import station_departure as sd
    turn = [(0.0, 0.0, 0.0, m.pi / 2), (0.3, 0.0, 0.0, m.pi / 2)]
    forward = [(0.3, 0.0, -0.005, -m.pi / 2), (0.6, 0.0, -0.035, -m.pi / 2)]
    assert sd.reverse_speed(turn + forward) > 0.01
    speeds, worst = sd.reverse_speed_by_phase({"turn": turn, "forward": forward})
    assert speeds == {"turn": 0.0, "forward": 0.0} and worst is None


def test_reverse_speed_by_phase_keeps_a_backward_move_inside_a_phase():
    import math as m
    from openamrobot_build1_scene import station_departure as sd
    forward = [(0.0, 0.0, 0.0, 0.0), (0.25, -0.004, 0.0, 0.0), (0.5, 0.02, 0.0, 0.0)]
    speeds, worst = sd.reverse_speed_by_phase({"turn": [], "forward": forward})
    assert speeds["forward"] == pytest.approx(0.016) and worst["phase"] == "forward" and worst["t"] == 0.0
    assert max(speeds.values()) > 0.01          # the unchanged 0.010 m/s limit still fails it


def test_rpp_allow_reversing_true_is_a_reverse_path():
    """PROPOSED (#117): the same rule as openamr-platform-sw test_baseline1_no_reverse_paths.py for StationRetreat."""
    params = pr90_set()
    c = params["controller_server"]["ros__parameters"]
    c["controller_plugins"].append("StationRetreat")
    c["StationRetreat"] = {"plugin": "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController",
                           "allow_reversing": False, "use_rotate_to_heading": True}
    assert sd.reverse_paths(params, TREES) == []
    c["StationRetreat"]["allow_reversing"] = True
    assert sd.reverse_paths(params, TREES) == ["StationRetreat allow_reversing true"]


def test_the_forward_retreat_drives_on_the_station_retreat_tree():
    """PROPOSED (#117): the forward goal uses openamr-platform-sw's navigate_to_station_retreat.xml."""
    assert CONFIG["forward"]["drive_tree"] == "navigate_to_station_retreat.xml"


def test_c7_option_1_gates_on_the_forward_drive_only():
    """#117 option 1 (PROPOSED): the turn's projected motion is observed, not gated; a reverse forward drive still fails."""
    from openamrobot_build1_scene import station_departure as sd
    assert sd.gate_reverse_speed({"turn": 0.01223, "forward": 0.0048}) == 0.0048
    assert sd.gate_reverse_speed({"turn": 0.0, "forward": 0.02}) == 0.02
    assert sd.gate_reverse_speed({"turn": 0.05}) is None  # a missing forward phase is not measured, and fails its check


# ------------------------------------------------------------------ observed only (#117 conditions 2 and 3)
SCENE_FILE = os.path.join(PACKAGE, "config", "scene.yaml")


class _Log:
    def __init__(self):
        self.observed_items, self.quantities = {}, []

    def observed(self, name, value, unit="", expected=None, reference="", note=""):
        self.observed_items[name] = value
        return value

    def quantity(self, *args, **kwargs):
        self.quantities.append(args)


def _scene():
    from openamrobot_build1_scene import scene as scene_model
    return scene_model.load(SCENE_FILE)


def test_the_goal_offset_inside_its_costmap_cell():
    assert sd.cell_offset(0.0, -1.785, 0.05, (-8.0, -6.0)) == pytest.approx((0.0, 0.015))
    assert sd.cell_offset(0.123, -6.0, 0.05, (-8.0, -6.0)) == pytest.approx((0.023, 0.0), abs=1e-9)
    assert sd.cell_offset(-7.99, -5.951, 0.05, (-8.0, -6.0)) == pytest.approx((0.01, 0.049))


def test_the_costmap_grid_is_the_map_the_mission_serves():
    """static_layer takes the map's resolution and origin: scene.write_map (launch/build1.launch.py) writes
    scene.occupancy_grid's. The nav2_params global_costmap resolution is only reported."""
    from openamrobot_build1_scene import scene as scene_model
    scene = _scene()
    params = {"global_costmap": {"global_costmap": {"ros__parameters": {"resolution": 0.05}}}}
    resolution, origin, source = sd.costmap_grid(params, scene)
    _grid, map_resolution, map_origin = scene_model.occupancy_grid(scene)
    assert (resolution, origin) == (map_resolution, map_origin) == (0.05, (-8.0, -6.0))
    assert "global_costmap.resolution 0.05" in source


def test_a_straight_forward_stage_has_not_rotated_in_place():
    straight = [(0.1 * i, 0.0, -2.085 + 0.01 * i, math.pi / 2) for i in range(31)]
    h = sd.forward_heading(straight)
    assert h["net_deg"] == pytest.approx(0.0) and h["tail_deg"] == pytest.approx(0.0)
    assert h["rotated_in_place"] is False
    assert sd.forward_heading([]) is None


def test_a_turn_in_place_at_the_end_of_the_forward_stage_is_found():
    drive = [(0.1 * i, 0.0, -2.085 + 0.01 * i, math.pi / 2) for i in range(31)]
    end = drive[-1]
    turn = [(end[0] + 0.1 * k, end[1] + 0.0005 * k, end[2], math.pi / 2 + math.radians(1.0 * k)) for k in range(1, 11)]
    h = sd.forward_heading(drive + turn)
    assert h["tail_deg"] == pytest.approx(10.0, abs=1e-6) and h["net_deg"] == pytest.approx(10.0, abs=1e-6)
    assert h["rotated_in_place"] is True
    # A turn out and back nets zero, but the sum of steps still shows it.
    back = [(turn[-1][0] + 0.1 * k, turn[-1][1], end[2], math.pi / 2 + math.radians(10.0 - 1.0 * k))
            for k in range(1, 11)]
    h = sd.forward_heading(drive + turn + back)
    assert h["tail_deg"] == pytest.approx(0.0, abs=1e-6) and h["tail_abs_deg"] == pytest.approx(20.0, abs=1e-6)
    assert h["rotated_in_place"] is True
    # Heading drift while still translating is not "in place".
    curving = [(0.1 * i, 0.0005 * i * i, -2.085 + 0.01 * i, math.pi / 2 + math.radians(0.3 * i)) for i in range(31)]
    h = sd.forward_heading(curving)
    assert h["net_deg"] == pytest.approx(9.0) and h["rotated_in_place"] is False


def test_the_end_clearance_against_the_work_shelf_front_face():
    from openamrobot_build1_scene import scene as scene_model
    scene = _scene()
    sx, sy, syaw = scene_model.station(scene, "work_shelf")      # 0.715 m in front of the face at y = -2.8
    assert (sx, sy) == pytest.approx((0.0, -2.085))
    assert sd.ROBOT_RADIUS_PARK_M == pytest.approx(0.486, abs=1e-3)
    # #93 cross-check: closed shelf, arms in park, at the station: 0.229 m.
    assert sd.end_clearance(scene, (sx, sy, syaw))["clearance_m"] == pytest.approx(0.229, abs=1e-3)
    c = sd.end_clearance(scene, (0.0, -1.785, math.pi / 2))     # 0.30 m out, after the half turn
    assert c["face_distance_m"] == pytest.approx(1.015) and c["clearance_m"] == pytest.approx(0.529, abs=1e-3)
    # Beside the shelf end the distance is to the face's end point, not to its line.
    c = sd.end_clearance(scene, (0.9, -2.4, 0.0))
    assert c["face_distance_m"] == pytest.approx(math.hypot(0.3, 0.4))


def test_the_new_records_are_observed_only_and_never_checks(monkeypatch):
    monkeypatch.setattr(sd, "_scene", _scene)
    log = _Log()
    params = {"global_costmap": {"global_costmap": {"ros__parameters": {"resolution": 0.05}}}}
    sd._observed_only(log, "goal", sd._observe_goal, log, params, 0.0, -1.785, "map")
    drive = [(0.1 * i, 0.0, -2.085 + 0.01 * i, math.pi / 2) for i in range(31)]
    sd._observed_only(log, "end", sd._observe_end, log, drive, (0.0, -1.785, math.pi / 2))
    assert log.quantities == []
    o = log.observed_items
    assert o["retreat_forward_goal_cell_offset_y"] == pytest.approx(0.015)
    assert o["retreat_forward_rotated_in_place"] is False
    assert o["retreat_end_clearance"] == pytest.approx(0.529, abs=1e-3)
    assert not any(k.endswith("_error") for k in o)
    # An observation that fails is recorded, not raised.
    monkeypatch.setattr(sd, "_scene", lambda: (_ for _ in ()).throw(FileNotFoundError("scene.yaml")))
    log = _Log()
    sd._observed_only(log, "end", sd._observe_end, log, drive, (0.0, -1.785, 0.0))
    assert "retreat_observation_end_error" in log.observed_items and log.quantities == []


def test_run_records_the_new_values_with_observed_only():
    source = open(sd.__file__, encoding="utf-8").read()
    run = ast.unparse(next(n for n in ast.parse(source).body if isinstance(n, ast.FunctionDef) and n.name == "run"))
    assert "_observed_only(log, 'goal', _observe_goal" in run and "_observed_only(log, 'end', _observe_end" in run
    for name in ("retreat_forward_goal", "retreat_forward_rotated_in_place", "retreat_end_clearance",
                 "retreat_forward_end_in_place"):
        assert f'log.quantity("{name}' not in source


# ------------------------------------------------------------------ rc.1: no departure below the Stop gate
def test_the_departure_cannot_be_selected_in_rc1():
    """BASELINE1_RETREAT=departure stops the mission with the reason: it would publish below platform #90's Stop gate."""
    with pytest.raises(ValueError, match="Stop gate"):
        sd.option(CONFIG, environ={"BASELINE1_RETREAT": "departure"})
    assert sd.option(CONFIG, environ={}) == "forward"


def test_run_refuses_the_departure_before_any_publisher_exists():
    """Second guard: run() refuses a disabled option before it creates the cmd_vel_smoothed publisher."""
    source = open(sd.__file__, encoding="utf-8").read()
    body = source[source.index("def run(rig, log, name"):]
    assert body.index("if name in RELEASE_DISABLED:") < body.index("create_publisher(")
    assert "departure" in sd.RELEASE_DISABLED and "forward" not in sd.RELEASE_DISABLED
