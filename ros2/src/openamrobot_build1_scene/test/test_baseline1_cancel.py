# SPDX-License-Identifier: MIT
"""The Baseline 1 cancellation check (openamrobot_build1_scene/baseline1_cancel.py, Proposed), without ROS.

1. verdict(): a clean cancel passes every check; each kind of wrong outcome fails its own check (status other than
   CANCELED, base still moving in the hold window, a nonzero /cmd_vel, nothing measured).
2. step() against a fake stack (fake rig, fake message modules): a stack that honours the cancel passes; one that
   reports ABORTED records 6 and fails cancel_goal_status; one that ignores the cancel fails the stop checks.
3. The checks step() writes satisfy the strict gate baseline1_gates.stop_cancel; without the option the gate fails.
4. The option is declared: off by default, on in the acceptance workflow, recorded in each leg's declaration, and the
   mission calls step() at the start of nav_to_station.
7. The hold window is judged by publication time (rclpy message info source_timestamp): a command published before
   the window but delivered late is not counted in it (run 44974639950), a command published in it still fails, and
   without source_timestamp the callback sim time is used and the fallback logged (PROPOSED, gate owner decides).
Simulation evidence only.
"""
import ast
import inspect
import math
import os
import sys
import time
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


def deliver(callback, msg, source_timestamp):
    """As rclpy jazzy calls a subscription callback (rclpy/subscription.py callback setter, executors.py
    _take_subscription): with the message alone if the callback binds one argument, else with the message and the
    message info dict that subscription.cpp take_message builds (source_timestamp, received_timestamp, ns)."""
    try:
        inspect.signature(callback).bind(object())
        return callback(msg)
    except TypeError:
        pass
    inspect.signature(callback).bind(object(), object())
    return callback(msg, {"source_timestamp": source_timestamp, "received_timestamp": time.time_ns(),
                          "publication_sequence_number": None, "reception_sequence_number": None})


class FakeStack:
    """A base that drives at 0.2 m/s towards the goal while it runs. `behaviour`: honours the cancel and reports
    CANCELED ("canceled"), honours it and reports ABORTED ("aborted"), or ignores it ("ignored")."""
    DT = 0.05

    def __init__(self, twist, behaviour="canceled", source=time.time_ns):
        self.twist, self.behaviour, self.source = twist, behaviour, source
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
                deliver(callback, self.twist(self.v), self.source())

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


def _functions():
    path = os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py")
    tree = ast.parse(open(path, encoding="utf-8").read())
    return {n.name: n for n in tree.body if isinstance(n, ast.FunctionDef)}


def _step_calls(function):
    return [n for n in ast.walk(function) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == "step" and getattr(n.func.value, "id", "") == "baseline1_cancel"]


def test_the_step_runs_at_the_end_of_the_mission_after_the_retreat():
    """Position end_of_mission (Alex 20 Nov 2026): no cancel step in nav_to_station, so the odometry guard at the
    station runs before it and no turn follows the cancel; the retreat phase leaves the station first, then cancels."""
    functions = _functions()
    assert CONFIG["position"] == "end_of_mission" and CONFIG["position"] in CONFIG["positions"]
    assert not _step_calls(functions["nav_to_station"])
    assert "nav_odometry_minus_ground_truth" in ast.unparse(functions["nav_to_station"])
    assert _step_calls(functions["end_of_mission_cancel"])
    body = [ast.unparse(s) for s in functions["retreat_then_cancel"].body
            if isinstance(s, ast.Expr) and isinstance(s.value, ast.Call)]
    assert body == ["retreat(rig, log)", "end_of_mission_cancel(rig, log)"]
    mission = ast.unparse(functions["mission"])
    assert "tracker.run('retreat', lambda: retreat_then_cancel(rig, log))" in mission
    order = [mission.index(f"tracker.run('{p}'") for p in ("nav_to_station", "retreat")]
    assert order == sorted(order)


def test_canceled_is_the_only_pass():
    assert CONFIG["expected_status"] == cancel.CANCELED
    for status in (4, 6, None):
        record = dict(accepted=True, status=status)
        checks = {c["name"]: c for c in cancel.verdict(record, CONFIG)}
        status_check = checks["cancel_goal_status"]
        assert status_check["low"] == status_check["high"] == 5 and status_check["value"] != 5


def test_the_short_goal_is_straight_ahead_on_the_spawn_heading():
    assert cancel.goal_pose((1.0, 2.0, math.pi / 2), 0.5) == pytest.approx((1.0, 2.5, math.pi / 2))


# ------------------------------------------------------------------ 5. the goal checker cannot win the race (run 44800204846)
def test_the_goal_checker_cannot_accept_the_goal_before_the_cancel():
    """bt_navigator's default checker (0.35 m) accepts a 0.5 m goal after 0.15 m, the cancel distance: run 44800204846
    ended SUCCEEDED and the cancel was refused (return code 3). The declared goal uses the station tree (0.025 m)."""
    assert not cancel.goal_reached_before_cancel(CONFIG)
    assert CONFIG["behavior_tree"] == "navigate_to_station.xml" and CONFIG["goal_xy_tolerance_m"] == 0.025
    assert cancel.goal_reached_before_cancel(dict(CONFIG, behavior_tree="", goal_xy_tolerance_m=0.35))
    assert (CONFIG["goal_distance_m"], CONFIG["cancel_at_m"], CONFIG["expected_status"]) == (0.5, 0.15, 5)


def test_a_bare_tree_name_is_resolved_and_the_default_stays_empty():
    assert cancel.tree_path("") == "" and cancel.tree_path("/x/y.xml") == "/x/y.xml"
    step = ast.unparse(next(n for n in ast.parse(open(cancel.__file__, encoding="utf-8").read()).body
                            if isinstance(n, ast.FunctionDef) and n.name == "step"))
    assert "goal.behavior_tree = tree_path(c.get('behavior_tree', ''))" in step


# ------------------------------------------------------------------ 6. the rig's executor survives the step (run 44848867373)
class SpinningNode:
    """rclpy while a MultiThreadedExecutor spins the node in another thread (rig.Rig): destroying a subscription there
    raises InvalidHandle in the executor thread, which then ends (run 44848867373, job 136675191420: "Exception in
    thread Thread-1 (spin) ... cannot use Destroyable because destruction was requested"). Here the destroy raises."""

    def __init__(self, subscribe):
        self.create_subscription, self.destroyed = subscribe, []

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)
        raise RuntimeError("InvalidHandle: cannot use Destroyable because destruction was requested")


def test_the_step_does_not_destroy_its_subscription_under_the_spinning_executor(monkeypatch):
    """After the executor thread died the rig received no /sil/drive/status, so host_client_drive_thread_alive read
    False at the end of a run whose host client was alive. The step keeps the subscription and stops recording."""
    stack = FakeStack(_fake_modules(monkeypatch), "canceled")
    stack.node = SpinningNode(stack._subscribe)
    log = FakeLog()
    record = cancel.step(stack, log, config=CONFIG)
    assert stack.node.destroyed == []
    assert failed(log.checks.values()) == []
    recorded = len(record["cmd_vel"])
    callback = stack.subscribers[-1]
    deliver(callback, stack.twist(0.3), time.time_ns())  # a message after the step: not recorded, nothing raises
    assert callback.__self__.messages[-1][1] == 0.0 and len(callback.__self__.messages) == recorded


def test_the_recorder_stops_on_stop():
    clock = iter([1.0, 2.0, 3.0])
    recorder = cancel.CmdVelRecorder(lambda: next(clock))
    twist = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.1, y=0.0), angular=types.SimpleNamespace(z=0.2))
    recorder.on_message(twist)
    assert recorder.stop() == [(1.0, 0.1, 0.0, 0.2)]
    recorder.on_message(twist)
    assert recorder.messages == [(1.0, 0.1, 0.0, 0.2)]


# ---------------------------------------------------------------- 7. hold window by publication time (run 44974639950)
WALL0 = 1_795_000_000_000_000_000     # ns since the epoch at sim time 0, for the fake records below


def wall_ns(sim_s):
    return WALL0 + round(sim_s * 1e9)


def late_delivery_record(published_in_window=()):
    """Run 44974639950 (job 137061674788): the base stopped 0.059 s after the cancel (ground truth, hold displacement
    0.0), yet /cmd_vel messages stamped in the callback read 0.18966 until +0.414 s, and the last of them, just before
    the window, made cancel_hold_cmd_vel_peak 0.18966. Here: t_cancel 9.0 s, the controller publishes 0.18966 until
    9.05 s and zero from 9.06 s at 20 Hz; the busy executor delivers the zero of 9.06 s first and the backlog of
    nonzero messages up to 9.414 s (callback sim time), then the zeros. `published_in_window`: (sim s, vx, wz)
    messages genuinely published inside the window [9.6, 12.6] s."""
    nonzero_pub = [8.80 + 0.05 * i for i in range(6)]                                 # 8.80 .. 9.05
    zero_pub = [round(9.06 + 0.05 * i, 3) for i in range(70)]                         # 9.06 .. 12.51
    delivered = [(9.07, zero_pub[0], 0.0, 0.0)]                                        # (callback t, published t, v, w)
    delivered += [(round(9.10 + 0.0628 * i, 3), p, 0.18966, 0.0) for i, p in enumerate(nonzero_pub)]   # to 9.414
    delivered += [(round(p + 0.55, 3), p, 0.0, 0.0) for p in zero_pub[1:]]           # late, but zero
    delivered += [(round(p + 0.55, 3), p, vx, wz) for p, vx, wz in published_in_window]
    record = dict(good_record(), t_cancel=9.0, time_to_stop_s=0.059, window=(9.6, 12.6))
    record["cmd_vel"] = [(t, vx, 0.0, wz) for t, _p, vx, wz in delivered]
    record["cmd_vel_published"] = [(wall_ns(p), vx, 0.0, wz) for _t, p, vx, wz in delivered]
    record["window_wall_ns"] = (wall_ns(9.6), wall_ns(12.6))
    return record


def peak_check(record):
    return next(c for c in cancel.verdict(record, CONFIG) if c["name"] == "cancel_hold_cmd_vel_peak")


def test_a_late_delivered_command_published_before_the_window_is_not_counted_in_it():
    """(a) The failure pattern: by callback time the last message before the window is a late 0.18966; by publication
    time it is the zero of 9.06 s, and every message published in the window is zero. Fails on 6c70f3c (0.18966)."""
    record = late_delivery_record()
    assert cancel.cmd_vel_in_window(record["cmd_vel"], 9.6, 12.6)[0] == pytest.approx(0.18966)   # the artefact
    check = peak_check(record)
    assert check["value"] == 0.0 and ok(check) and "publication time" in check["note"]
    assert failed(cancel.verdict(record, CONFIG)) == []
    # the sim-time checks are unchanged: the late nonzero is still measured on callback time
    by_name = {c["name"]: c for c in cancel.verdict(record, CONFIG)}
    assert by_name["cancel_cmd_vel_nonzero_after_cancel"]["value"] == pytest.approx(0.414)


def test_a_late_backlog_delivered_inside_the_sim_window_is_not_counted_either():
    """(a) The same pattern with the backlog delivered after the window opened (callback time 9.7 s >= 9.6 s)."""
    record = late_delivery_record()
    record["cmd_vel"] = [(9.7, 0.0, 0.0, 0.0) if m[0] == 9.07 else m for m in record["cmd_vel"]]
    record["cmd_vel"].append((9.75, 0.18966, 0.0, 0.0))
    record["cmd_vel_published"].append((wall_ns(9.05), 0.18966, 0.0, 0.0))
    assert cancel.cmd_vel_in_window(record["cmd_vel"], 9.6, 12.6)[0] == pytest.approx(0.18966)
    assert peak_check(record)["value"] == 0.0


@pytest.mark.parametrize("vx, wz", [(0.0, 0.03), (0.002, 0.0)])
def test_a_nonzero_command_published_inside_the_window_still_fails(vx, wz):
    """(b) A command genuinely published in the window fails, by publication time as by callback time."""
    record = late_delivery_record(published_in_window=[(11.0, vx, wz)])
    check = peak_check(record)
    assert check["value"] == pytest.approx(max(vx, wz)) and not ok(check)
    assert failed(cancel.verdict(record, CONFIG)) == ["cancel_hold_cmd_vel_peak"]


def test_a_nonzero_command_published_last_before_the_window_still_fails():
    """(b) The base keeps the last command it was given: a nonzero one published last before the window counts."""
    record = late_delivery_record()
    record["cmd_vel_published"] = [m for m in record["cmd_vel_published"] if m[0] < wall_ns(9.5)]
    record["cmd_vel_published"].append((wall_ns(9.55), 0.05, 0.0, 0.0))
    check = peak_check(record)
    assert check["value"] == pytest.approx(0.05) and not ok(check)


def test_publication_order_not_delivery_order_picks_the_last_message_before_the_window():
    """Delivered last but published first: the zero published after it is the command the base holds."""
    record = dict(good_record(), window_wall_ns=(wall_ns(10.0), wall_ns(13.0)),
                  cmd_vel_published=[(wall_ns(9.1), 0.0, 0.0, 0.0), (wall_ns(9.0), 0.2, 0.0, 0.0)])
    assert peak_check(record)["value"] == 0.0


@pytest.mark.parametrize("missing", [0, None, "x"])
def test_without_a_source_timestamp_the_callback_time_is_used_as_before(missing):
    """Fallback: one message without source_timestamp (0, missing) and the check is the 6c70f3c one, which fails the
    late-delivery record, and its note says so."""
    record = late_delivery_record()
    stamp = cancel.source_ns({"source_timestamp": missing})
    record["cmd_vel_published"][3] = (stamp,) + record["cmd_vel_published"][3][1:]
    check = peak_check(record)
    assert check["value"] == pytest.approx(0.18966) and "fallback" in check["note"]
    assert cancel.hold_messages({"cmd_vel": [(1.0, 0, 0, 0)], "window": (0.0, 2.0)})[2] == "fallback"


def test_source_ns_reads_the_rclpy_message_info():
    assert cancel.source_ns({"source_timestamp": 1_795_000_000_123_456_789, "received_timestamp": 0}) \
        == 1_795_000_000_123_456_789
    assert cancel.source_ns(types.SimpleNamespace(source_timestamp=5)) == 5
    assert cancel.source_ns({"source_timestamp": 0}) is None and cancel.source_ns(None) is None
    assert cancel.source_ns({}) is None


def test_the_subscription_callback_receives_the_message_info():
    """rclpy jazzy passes the message info only to a callback that does not bind one argument: on_message_info has
    two required parameters (a default on info would make rclpy pass the message alone)."""
    recorder = cancel.CmdVelRecorder(lambda: 1.0)
    with pytest.raises(TypeError):
        inspect.signature(recorder.on_message_info).bind(object())
    inspect.signature(recorder.on_message_info).bind(object(), object())
    twist = types.SimpleNamespace(linear=types.SimpleNamespace(x=0.1, y=0.0), angular=types.SimpleNamespace(z=0.0))
    deliver(recorder.on_message_info, twist, 42)
    assert recorder.published == [(42, 0.1, 0.0, 0.0)] and recorder.messages == [(1.0, 0.1, 0.0, 0.0)]


def test_the_step_judges_by_publication_time_and_records_the_basis(monkeypatch):
    stack, log, record = run_step(monkeypatch, "canceled")
    assert record["cmd_vel_time_basis"] == "publication"
    assert all(m[0] is not None for m in record["cmd_vel_published"])
    assert record["wall_cancel_ns"] <= record["wall_stopped_ns"] <= record["window_wall_ns"][0] \
        < record["window_wall_ns"][1]
    assert failed(log.checks.values()) == []
    assert not any("fallback" in e for e in log.events)
    assert log.data["baseline1"]["cancel_check"]["record"]["cmd_vel_time_basis"] == "publication"


def test_the_step_falls_back_and_logs_it_without_source_timestamps(monkeypatch):
    stack = FakeStack(_fake_modules(monkeypatch), "canceled", source=lambda: 0)
    log = FakeLog()
    record = cancel.step(stack, log, config=CONFIG)
    assert record["cmd_vel_time_basis"] == "fallback"
    assert any("without source_timestamp" in e and "fallback" in e for e in log.events)
    assert failed(log.checks.values()) == []
