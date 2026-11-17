# SPDX-License-Identifier: MIT
"""Baseline 1 phase tagging, failure classification and reports, without ROS (baseline1_phases.py)."""
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))
from openamrobot_build1_scene import baseline1_phases as phases  # noqa: E402


class FakeLog:
    """The part of rig.Log the tracker wraps: quantity with low/high, require, failures, data."""

    def __init__(self):
        self.data, self.failures = {"quantities": []}, []

    def quantity(self, name, value, unit="", low=None, high=None, kind="measured", note=""):
        ok = None
        if low is not None or high is not None:
            ok = value is not None and (low is None or value >= low) and (high is None or value <= high)
            if not ok:
                self.failures.append(f"{name} = {value} {unit}, allowed {low} to {high}")
        self.data["quantities"].append({"name": name, "value": value, "low": low, "high": high, "ok": ok})
        return value

    def require(self, condition, text):
        if not condition:
            self.failures.append(text)
        return bool(condition)


class Blocked(Exception):
    pass


@pytest.mark.parametrize("text, expected", [
    ("grasp_approach_moveit_error_code = -1 MoveItErrorCodes, allowed 1 to 1", phases.PLANNING),
    ("grasp_reach_moveit_error_code = -4 MoveItErrorCodes, allowed 1 to 1", phases.CONTROL),
    ("grasp_lift_moveit_error_code = -6 MoveItErrorCodes, allowed 1 to 1", phases.TIMEOUT),
    ("grasp_lift_moveit_error_code = None MoveItErrorCodes, allowed 1 to 1", phases.TIMEOUT),
    ("nav_status = 6 GoalStatus, allowed 4 to 4", phases.PLANNING),
    ("nav_status = None GoalStatus, allowed 4 to 4", phases.TIMEOUT),
    ("retreat_status = 6 GoalStatus, allowed 4 to 4", phases.CONTROL),
    ("item_height_gain_after_lift = 0.001 m, allowed 0.04 to 0.12", phases.CONTACT),
    ("finger_position_on_item = 0.0 rad, allowed 0.02 to 0.7354", phases.CONTACT),
    ("grasp_reach_servo_torque_max = 1.2 N m, allowed 0.0 to 0.5", phases.CONTROL),
    ("lift did not arrive at stop 1", phases.CONTROL),
    ("not ready after 180.0 s of Gazebo time: lift idle on CAN3", phases.STARTUP),
    ("Nav2 navigate_to_pose did not accept the goal", phases.STARTUP),
    ("BLOCKED: no vcan interface", phases.ENV),
    ("nav_host_client_drive_thread_alive = False bool, allowed True to True", phases.ENV),
    ("something nobody expected", phases.UNCLASSIFIED),
])
def test_classify(text, expected):
    assert phases.classify(text) == expected


def run_mission(failing_reach=False, blocked_lift=False, stop_after_approach=False):
    log, clock = FakeLog(), iter(range(1000))
    tracker = phases.Tracker(log, clock=lambda: float(next(clock)))

    def lift():
        log.quantity("lift_stop_1_gazebo_position", 0.175, "m", low=0.169, high=0.181)
        if blocked_lift:
            raise Blocked("the CTL-017 model raised fault bits 4 while Gazebo ran at real-time factor 0.30")

    def grasp():  # the quantity names of scenarios.grasp
        log.quantity("finger_open_position", 0.78, "rad", low=0.70, high=0.80)
        log.quantity("grasp_reach_moveit_error_code", -1 if failing_reach else 1, "MoveItErrorCodes", low=1, high=1)
        if stop_after_approach:
            return
        log.quantity("finger_position_on_item", 0.3, "rad", low=0.02, high=0.73)
        log.quantity("grasp_lift_moveit_error_code", 1, "MoveItErrorCodes", low=1, high=1)
        log.quantity("item_height_gain_after_lift", 0.07, "m", low=0.04, high=0.12)

    tracker.run("spawn", lambda: log.quantity("spawn_position_error", 0.0, "m", low=0.0, high=0.02))
    tracker.run("nav_to_station", lambda: log.quantity("nav_status", 4, "", low=4, high=4))
    tracker.run("stop_authorize", lambda: log.quantity("stop_base_speed", 0.0, "m/s", low=0.0, high=0.01))
    tracker.run("lift_to_target", lift)
    tracker.run("arm_approach", grasp, then=("grasp", "lift_object"))
    tracker.run("retreat", lambda: log.quantity("retreat_status", 4, "", low=4, high=4))
    return log, tracker


def test_every_phase_passes_and_the_split_of_grasp_tags_each_check():
    log, tracker = run_mission()
    table = phases.rows({"phases": tracker.result()})
    assert [row[3] for row in table] == ["PASS"] * len(phases.PHASES)
    assert phases.complete(table) and phases.partial_passes(table) == []
    tags = {q["name"]: q["phase"] for q in log.data["quantities"]}
    assert tags["finger_open_position"] == "arm_approach" and tags["finger_position_on_item"] == "grasp"
    assert tags["item_height_gain_after_lift"] == "lift_object"
    assert "Baseline 1: COMPLETE" in phases.render(table)


def test_a_failed_reach_is_planning_and_later_passes_are_partial():
    _log, tracker = run_mission(failing_reach=True)
    table = phases.rows({"phases": tracker.result()})
    row = {r[0]: r for r in table}
    assert row["arm_approach"][3] == "FAIL" and row["arm_approach"][4] == [phases.PLANNING]
    assert not phases.complete(table)
    assert "retreat (after an earlier phase did not pass)" in phases.partial_passes(table)
    assert "NOT COMPLETE" in phases.render(table)


def test_a_blocked_lift_is_env_infra_and_the_mission_goes_on():
    _log, tracker = run_mission(blocked_lift=True)
    table = {r[0]: r for r in phases.rows({"phases": tracker.result()})}
    assert table["lift_to_target"][3] == "BLOCKED" and table["lift_to_target"][4] == [phases.ENV]
    assert table["retreat"][3] == "PASS" and tracker.blocked()


def test_phases_grasp_returned_before_are_not_run():
    _log, tracker = run_mission(stop_after_approach=True)
    table = {r[0]: r for r in phases.rows({"phases": tracker.result()})}
    assert table["grasp"][3] == "NOT RUN" and table["lift_object"][3] == "NOT RUN"
    assert "not reached" in table["grasp"][6]


def test_any_other_exception_stops_the_mission_and_is_raised():
    log = FakeLog()
    tracker = phases.Tracker(log)
    with pytest.raises(RuntimeError):
        tracker.run("spawn", lambda: (_ for _ in ()).throw(RuntimeError("not ready after 180 s: odometry")))
    table = {r[0]: r for r in phases.rows({"phases": tracker.result()})}
    assert table["spawn"][3] == "FAIL" and table["spawn"][4] == [phases.STARTUP]
    assert table["retreat"][3] == "NOT RUN"


def test_a_phase_without_a_check_does_not_pass():
    log = FakeLog()
    tracker = phases.Tracker(log)
    tracker.run("spawn", lambda: None)
    assert phases.verdict(tracker.records["spawn"]) == "FAIL"


def test_without_a_log_the_stage_classifies_every_phase():
    assert {tuple(r[4]) for r in phases.rows(None, build_ok=False)} == {(phases.BUILD,)}
    assert {tuple(r[4]) for r in phases.rows(None, env_ok=False)} == {(phases.ENV,)}
    table = phases.rows(None)
    assert table[0][3] == "FAIL" and table[0][4] == [phases.STARTUP] and table[1][3] == "NOT RUN"
    started = phases.rows({"result": "FAIL", "reason": "not ready after 180.0 s of Gazebo time: drive configured over CAN1"})
    assert started[0][4] == [phases.STARTUP]


def test_trace_summary_is_on_one_time_base():
    trace = [{"sim": t * 0.5, "wall": 100.0 + t, "phase": "nav_to_station", "x": 0.0, "y": -0.1 * t, "v": 0.2,
              "lift": 0.0, "item_z": 0.9, "odom_err": 0.01, "rtf": 0.5} for t in range(5)]
    lines = phases.trace_summary(trace, [{"phase": "spawn"}, {"phase": "nav_to_station"}])
    assert lines[1].startswith("spawn | 0") and lines[2].startswith("nav_to_station | 5 | 2.0 | 4.0 | 0.4 | 0.2")
    assert len(phases.trace_table(trace, every_sim_s=1.0)) == 1 + 3


def test_report_script_exit_status_and_table(tmp_path, capsys):
    import baseline1_report
    _log, tracker = run_mission()
    (tmp_path / "baseline1_mission.json").write_text(json.dumps({"result": "PASS", "phases": tracker.result(),
                                                                 "failures": [], "trace": []}))
    assert baseline1_report.main([str(tmp_path)]) == 0
    _log, tracker = run_mission(failing_reach=True)
    (tmp_path / "baseline1_mission.json").write_text(json.dumps({"result": "FAIL", "phases": tracker.result(),
                                                                 "failures": ["drive_emulator_plant_switches = 2"],
                                                                 "trace": []}))
    assert baseline1_report.main([str(tmp_path)]) == 1
    out = capsys.readouterr().out
    assert "| arm_approach |" in out and "PLANNING" in out and "drive_emulator_plant_switches = 2" in out
