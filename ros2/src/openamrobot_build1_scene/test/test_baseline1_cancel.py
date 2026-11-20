# SPDX-License-Identifier: MIT
"""The Baseline 1 cancellation check (openamrobot_build1_scene/baseline1_cancel.py, Proposed), without ROS.

1. verdict(): a clean cancel passes every check; each kind of wrong outcome fails its own check (status other than
   CANCELED, base still moving in the hold window, a nonzero /cmd_vel, nothing measured).
2. step() against a fake stack (fake rig, fake message modules): a stack that honours the cancel passes; one that
   reports ABORTED records 6 and fails cancel_goal_status; one that ignores the cancel fails the stop checks.
3. The checks step() writes satisfy the strict gate baseline1_gates.stop_cancel; without the option the gate fails.
4. The option is declared: off by default, on in the acceptance workflow, recorded in each leg's declaration, and the
   mission calls step() at the start of nav_to_station.
Simulation evidence only.
"""
import ast
import math
import os
import sys
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, PACKAGE)
from openamrobot_build1_scene import baseline1_cancel as cancel  # noqa: E402
from openamrobot_build1_scene import baseline1_gates as gates  # noqa: E402

CONFIG = cancel.load()
REPO = os.path.dirname(os.path.dirname(os.path.dirname(PACKAGE)))
WORKFLOW = os.path.join(REPO, ".github", "workflows", "baseline1-acceptance.yml")


def good_record():
    hold = [(10.0 + 0.1 * i, 0.20, 0.0, 0.0) for i in range(31)]
    return {"accepted": True, "progress_m": 0.15, "speed_m_s": 0.18, "t_cancel": 9.0, "return_code": 0,
            "goals_canceling": 1, "status": 5, "time_to_stop_s": 0.6, "distance_after_cancel_m": 0.02,
            "window": (10.0, 13.0), "hold_truth": hold, "hold_odom": hold,
            "cmd_vel": [(8.9, 0.18, 0.0, 0.0), (9.05, 0.05, 0.0, 0.0), (9.1, 0.0, 0.0, 0.0)]}


def failed(checks):
    return sorted(c["name"] for c in checks if not ok(c))


def ok(check):
    v, lo, hi = check["value"], check["low"], check["high"]
    return v is not None and (lo is None or v >= lo) and (hi is None or v <= hi)


# ------------------------------------------------------------------ 1. verdict
def test_a_clean_cancel_passes_every_check_and_every_name_is_a_cancellation_check():
    checks = cancel.verdict(good_record(), CONFIG)
    assert failed(checks) == []
    assert len(checks) == 13 and all(c["name"].startswith("cancel_") for c in checks)


def test_aborted_instead_of_canceled_fails_only_the_status_check():
    record = dict(good_record(), status=6)
    assert failed(cancel.verdict(record, CONFIG)) == ["cancel_goal_status"]


def test_motion_in_the_hold_window_fails_ground_truth_and_odometry():
    record = good_record()
    record["hold_truth"] = record["hold_truth"][:-1] + [(13.0, 0.23, 0.0, 0.0)]
    record["hold_odom"] = record["hold_odom"][:-1] + [(13.0, 0.20, 0.0, 0.05)]
    assert failed(cancel.verdict(record, CONFIG)) == ["cancel_hold_ground_truth_displacement",
                                                      "cancel_hold_odometry_yaw_change"]


def test_a_nonzero_cmd_vel_in_or_just_before_the_window_fails():
    record = dict(good_record(), cmd_vel=[(8.9, 0.18, 0.0, 0.0), (11.0, 0.0, 0.0, 0.03)])
    assert "cancel_hold_cmd_vel_peak" in failed(cancel.verdict(record, CONFIG))
    # The last message before the window is still the command the base holds.
    record = dict(good_record(), cmd_vel=[(9.5, 0.05, 0.0, 0.0)])
    assert failed(cancel.verdict(record, CONFIG)) == ["cancel_hold_cmd_vel_peak"]


def test_commands_that_go_on_after_the_cancel_are_measured():
    record = dict(good_record(), cmd_vel=[(9.0 + 0.1 * i, 0.1, 0.0, 0.0) for i in range(60)] + [(15.0, 0.0, 0.0, 0.0)])
    by_name = {c["name"]: c for c in cancel.verdict(record, CONFIG)}
    assert by_name["cancel_cmd_vel_nonzero_after_cancel"]["value"] == pytest.approx(5.9)
    assert not ok(by_name["cancel_cmd_vel_nonzero_after_cancel"])


def test_nothing_measured_is_never_a_pass():
    assert failed(cancel.verdict({"accepted": False}, CONFIG)) == ["cancel_goal_accepted"]
    record = {"accepted": True}
    names = failed(cancel.verdict(record, CONFIG))
    assert "cancel_goal_status" in names and "cancel_hold_ground_truth_displacement" in names
    assert "cancel_hold_cmd_vel_peak" in names and "cancel_request_accepted" in names


def test_a_goal_that_was_cancelled_too_late_is_not_mid_way():
    record = dict(good_record(), progress_m=0.45)
    assert failed(cancel.verdict(record, CONFIG)) == ["cancel_progress_at_cancel"]


# ------------------------------------------------------------------ 2. step() against a fake stack
class Msg(types.SimpleNamespace):
    pass


def _fake_modules(monkeypatch):
    def twist(vx=0.0, wz=0.0):
        return Msg(linear=Msg(x=vx, y=0.0, z=0.0), angular=Msg(x=0.0, y=0.0, z=wz))

    def nav_goal():
        pose = Msg(position=Msg(x=0.0, y=0.0), orientation=Msg(z=0.0, w=1.0))
        return Msg(pose=Msg(header=Msg(frame_id=""), pose=pose), behavior_tree=None)
    geometry = types.ModuleType("geometry_msgs.msg")
    geometry.Twist = twist
    nav2 = types.ModuleType("nav2_msgs.action")
    nav2.NavigateToPose = Msg(Goal=nav_goal)
    for name, module in (("geometry_msgs", types.ModuleType("geometry_msgs")), ("geometry_msgs.msg", geometry),
                         ("nav2_msgs", types.ModuleType("nav2_msgs")), ("nav2_msgs.action", nav2)):
        monkeypatch.setitem(sys.modules, name, module)
    return twist


class Future:
    def __init__(self, value):
        self.value = value

    def done(self):
        return self.value is not None

    def result(self):
        return self.value


class FakeStack:
    """A base that drives at 0.2 m/s towards the goal while it runs. `behaviour`: honours the cancel and reports
    CANCELED ("canceled"), honours it and reports ABORTED ("aborted"), or ignores it ("ignored")."""
    DT = 0.05

    def __init__(self, twist, behaviour="canceled"):
        self.twist, self.behaviour = twist, behaviour
        self.t, self.x, self.v = 0.0, 0.0, 0.0
        self.goal_x, self.status, self.cancelled = None, None, False
        self.subscribers = []
        self.node = Msg(create_subscription=self._subscribe, destroy_subscription=lambda s: None)

    def _subscribe(self, _kind, _topic, callback, _qos):
        self.subscribers.append(callback)
        return callback

    def _advance(self):
        self.t += self.DT
        honoured = not self.cancelled or self.behaviour == "ignored"
        driving = self.goal_x is not None and self.status is None and honoured
        self.v = 0.2 if driving else 0.0
        self.x += self.v * self.DT
        if driving and self.x >= self.goal_x:
            self.status, self.v = 4, 0.0
        if self.cancelled and self.behaviour != "ignored" and self.status is None:
            self.status = 5 if self.behaviour == "canceled" else 6
        if self.goal_x is not None:
            for callback in self.subscribers:
                callback(self.twist(self.v))

    # the rig interface baseline1_cancel.step uses
    def sim_time(self):
        return self.t

    def truth(self):
        return self.x, 0.0, 0.0

    def odom(self):
        return self.x, 0.0, 0.0

    def truth_speed(self):
        return self.v, 0.0

    def wait(self, condition, sim_s, _what="", step=None):
        end = self.t + sim_s
        while self.t < end:
            if condition():
                return True
            self._advance()
        return bool(condition())

    def sleep_sim(self, seconds):
        self.wait(lambda: False, seconds)

    def wait_stopped(self, sim_s=10.0, hold_s=0.5):
        start, since = self.t, [None]

        def held():
            if self.v < 0.01:
                since[0] = self.t if since[0] is None else since[0]
                return self.t - since[0] >= hold_s
            since[0] = None
            return False
        return since[0] - start if self.wait(held, sim_s) else None

    def send_goal(self, _name, _module, _kind, goal):
        self.goal_x = goal.pose.pose.position.x
        stack = self

        class Handle:
            def cancel_goal_async(self):
                stack.cancelled = True
                return Future(Msg(return_code=0, goals_canceling=[1]))
        return Handle()

    def wait_result(self, _handle, sim_s, _what=""):
        self.wait(lambda: self.status is not None, sim_s)
        return self.status, None


class FakeLog:
    def __init__(self):
        self.data, self.checks, self.events = {}, {}, []

    def quantity(self, name, value, unit="", low=None, high=None, note=""):
        self.checks[name] = {"name": name, "value": value, "low": low, "high": high}

    def event(self, text):
        self.events.append(text)


def run_step(monkeypatch, behaviour):
    stack = FakeStack(_fake_modules(monkeypatch), behaviour)
    log = FakeLog()
    record = cancel.step(stack, log, config=CONFIG)
    return stack, log, record


def test_a_stack_that_honours_the_cancel_passes_and_stops_short_of_the_goal(monkeypatch):
    stack, log, record = run_step(monkeypatch, "canceled")
    assert failed(log.checks.values()) == []
    assert log.checks["cancel_goal_status"]["value"] == cancel.CANCELED
    assert stack.x < CONFIG["goal_distance_m"] - 0.2       # the goal was not reached: the cancel ended it
    assert record["cmd_vel"] and log.data["baseline1"]["cancel_check"]["record"]["status"] == 5


def test_a_stack_that_reports_aborted_is_recorded_as_aborted(monkeypatch):
    _stack, log, _record = run_step(monkeypatch, "aborted")
    assert log.checks["cancel_goal_status"]["value"] == 6
    assert failed(log.checks.values()) == ["cancel_goal_status"]


def test_a_stack_that_ignores_the_cancel_fails_the_stop_checks(monkeypatch):
    _stack, log, _record = run_step(monkeypatch, "ignored")
    names = failed(log.checks.values())
    assert "cancel_goal_status" in names and "cancel_distance_after_cancel" in names


# ------------------------------------------------------------------ 3. the strict gate
def _mission_with(checks):
    quantities = [{"name": c["name"], "value": c["value"], "ok": ok(c)} for c in checks]
    record = {"phase": "nav_to_station", "quantities": quantities}
    stop = {"phase": "stop_authorize", "quantities": [
        {"name": n, "value": True, "ok": True}
        for n in gates.load()["stop_cancel"]["required_checks"]["stop_authorize"]]}
    return {"phases": [record, stop]}


def test_the_checks_of_the_step_pass_the_strict_cancellation_gate(monkeypatch):
    _stack, log, _record = run_step(monkeypatch, "canceled")
    assert gates.stop_cancel(_mission_with(log.checks.values()), gates.load()) == []


def test_without_the_step_or_with_an_aborted_cancel_the_gate_fails(monkeypatch):
    problems = gates.stop_cancel(_mission_with([]), gates.load())
    assert any("0 cancellation checks ran" in p for p in problems)
    _stack, log, _record = run_step(monkeypatch, "aborted")
    problems = gates.stop_cancel(_mission_with(log.checks.values()), gates.load())
    assert problems == ["cancellation: check cancel_goal_status failed (6)"]


# ------------------------------------------------------------------ 4. the option is declared
def test_the_option_is_off_by_default_and_on_only_with_1():
    assert not cancel.enabled(CONFIG, environ={})
    assert not cancel.enabled(CONFIG, environ={"BASELINE1_CANCEL_CHECK": "0"})
    assert cancel.enabled(CONFIG, environ={"BASELINE1_CANCEL_CHECK": "1"})


def test_the_acceptance_workflow_enables_and_declares_the_option():
    text = open(WORKFLOW, encoding="utf-8").read()
    assert "BASELINE1_CANCEL_CHECK: '1'" in text
    assert '"cancel_check": e["BASELINE1_CANCEL_CHECK"]' in text
    assert "cancel_check" in gates.load()["evidence"]["declared_settings"]


def test_the_mission_runs_the_step_at_the_start_of_nav_to_station():
    path = os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "nav_to_station")
    calls = [n for n in ast.walk(function) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
    step_calls = [c for c in calls if c.func.attr == "step" and getattr(c.func.value, "id", "") == "baseline1_cancel"]
    goal_calls = [c for c in calls if c.func.attr == "send_goal"]
    assert step_calls and goal_calls and step_calls[0].lineno < goal_calls[0].lineno


def test_the_short_goal_is_straight_ahead_on_the_spawn_heading():
    assert cancel.goal_pose((1.0, 2.0, math.pi / 2), 0.5) == pytest.approx((1.0, 2.5, math.pi / 2))
