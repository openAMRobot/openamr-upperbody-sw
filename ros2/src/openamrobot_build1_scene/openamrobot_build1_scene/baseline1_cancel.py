# SPDX-License-Identifier: MIT
"""Baseline 1 cancellation check (Proposed for Alex and Mohamed, 20 Nov 2026), in the test harness, not a stop gate.

On only when the declared mission option BASELINE1_CANCEL_CHECK=1 is set (the acceptance workflow sets it and declares
it). At the start of nav_to_station, before the station goal: one short NavigateToPose goal straight ahead of the spawn
pose on open floor, which the mission does not need; once the base has moved cancel_at_m, the goal is cancelled through
the public action interface (the action's cancel service, as any client would). Then:
- the terminal status must be CANCELED (5). Whatever the stack returns is recorded as cancel_goal_status;
- the base stops within max_time_to_stop_sim_s and max_distance_after_cancel_m;
- over a declared hold window after the stop, ground truth and odometry move less than max_hold_displacement_m and
  max_hold_yaw_change_rad, and every /cmd_vel message is zero (the last one before the window counts too, since the
  base keeps the last command it was given).
The mission then goes on with the station goal from where the base stopped.

verdict() and the helpers have no ROS import: test/test_baseline1_cancel.py runs them on recorded samples. step() runs
inside the mission. Every check name starts with cancel_, which baseline1_gates.stop_cancel requires.
Simulation evidence only: a functional check of the action interface, never hardware, physical or safety evidence.
"""
import json
import math
import os

CANCELED = 5                     # action_msgs/GoalStatus
CANCEL_ERROR_NONE = 0            # action_msgs/CancelGoal.Response
SOURCE_FILE = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config",
                           "baseline1_mission_options.json")


def load(path=None):
    """The cancel_check section of config/baseline1_mission_options.json: the source tree, else the installed share."""
    candidates = [path] if path else [SOURCE_FILE]
    if not path:
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config",
                                           "baseline1_mission_options.json"))
        except Exception:  # noqa: BLE001 - no ament index outside a sourced workspace
            pass
    for candidate in candidates:
        if candidate and os.path.isfile(candidate):
            with open(candidate, encoding="utf-8") as f:
                return json.load(f)["cancel_check"]
    raise FileNotFoundError(f"baseline1_mission_options.json not found in {candidates}")


def enabled(config=None, environ=None):
    environ = os.environ if environ is None else environ
    name = (config or {}).get("env", "BASELINE1_CANCEL_CHECK")
    return environ.get(name, "0") == "1"


def goal_pose(start, distance):
    """(x, y, yaw) `distance` straight ahead of `start`, same heading."""
    x, y, yaw = start
    return x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw


def _angle(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def displacement(samples):
    """samples [(t, x, y, yaw)] -> (largest distance from the first sample, largest |yaw change|), or (None, None)."""
    points = [s for s in samples if s is not None and None not in s]
    if len(points) < 2:
        return None, None
    _t0, x0, y0, yaw0 = points[0]
    return (max(math.hypot(x - x0, y - y0) for _t, x, y, _yaw in points),
            max(abs(_angle(yaw, yaw0)) for _t, _x, _y, yaw in points))


def _magnitude(message):
    _t, vx, vy, wz = message
    return max(abs(vx), abs(vy), abs(wz))


def cmd_vel_in_window(messages, t0, t1):
    """messages [(t, vx, vy, wz)] -> (peak |component| in [t0, t1] including the last message before t0, number of
    messages in the window). The last message before the window counts: the base keeps the last command it was
    given. None when no message was ever received before t1 (nothing to judge)."""
    before = [m for m in messages if m[0] < t0]
    inside = [m for m in messages if t0 <= m[0] <= t1]
    judged = inside + before[-1:]
    if not judged:
        return None, 0
    return max(_magnitude(m) for m in judged), len(inside)


def last_nonzero_after(messages, t_cancel, threshold):
    """Seconds from the cancel to the last /cmd_vel message above `threshold` (0.0 when none after the cancel)."""
    late = [m[0] for m in messages if m[0] >= t_cancel and _magnitude(m) > threshold]
    return (max(late) - t_cancel) if late else 0.0


def verdict(record, config):
    """The checks of one cancellation step, as dicts for rig.Log.quantity: name, value, unit, low, high, note.
    A value that was not measured is None and fails its check: a missing record is never a pass."""
    c = config
    lo, hi = c["mid_way_m"]
    checks = [dict(name="cancel_goal_accepted", value=bool(record.get("accepted")), unit="bool", low=True, high=True,
                   note=f"short NavigateToPose goal on {c['action']}, {c['goal_distance_m']} m straight ahead")]
    if not record.get("accepted"):
        return checks
    checks += [
        dict(name="cancel_progress_at_cancel", value=record.get("progress_m"), unit="m", low=lo, high=hi,
             note=f"ground truth from the start at the cancel: mid-way of the {c['goal_distance_m']} m goal"),
        dict(name="cancel_base_speed_at_cancel", value=record.get("speed_m_s"), unit="m/s",
             low=c["min_speed_at_cancel_m_s"], high=None, note="the base was moving when the cancel was sent"),
        dict(name="cancel_request_accepted", value=record.get("return_code") == CANCEL_ERROR_NONE
             and (record.get("goals_canceling") or 0) >= 1, unit="bool", low=True, high=True,
             note=f"CancelGoal return_code {record.get('return_code')!r}, goals canceling "
                  f"{record.get('goals_canceling')!r} (0 is ERROR_NONE)"),
        dict(name="cancel_goal_status", value=record.get("status"), unit="GoalStatus", low=c["expected_status"],
             high=c["expected_status"], note="5 CANCELED, 4 SUCCEEDED, 6 ABORTED, None: no result in time"),
        dict(name="cancel_time_to_stop", value=record.get("time_to_stop_s"), unit="s of Gazebo time", low=0.0,
             high=c["max_time_to_stop_sim_s"], note="from the cancel until below 0.01 m/s and 0.02 rad/s for 0.5 s"
                                                    " (rig.wait_stopped); limit of scn07"),
        dict(name="cancel_distance_after_cancel", value=record.get("distance_after_cancel_m"), unit="m", low=0.0,
             high=c["max_distance_after_cancel_m"], note="ground truth from the cancel to the stop"),
    ]
    t0, t1 = record.get("window") or (None, None)
    truth_d, truth_yaw = displacement(record.get("hold_truth") or [])
    odom_d, odom_yaw = displacement(record.get("hold_odom") or [])
    window = f"over {c['hold_window_sim_s']} s of Gazebo time after the stop"
    checks += [
        dict(name="cancel_hold_ground_truth_displacement", value=truth_d, unit="m", low=0.0,
             high=c["max_hold_displacement_m"], note=window),
        dict(name="cancel_hold_ground_truth_yaw_change", value=truth_yaw, unit="rad", low=0.0,
             high=c["max_hold_yaw_change_rad"], note=window),
        dict(name="cancel_hold_odometry_displacement", value=odom_d, unit="m", low=0.0,
             high=c["max_hold_displacement_m"], note=window + ", /odom"),
        dict(name="cancel_hold_odometry_yaw_change", value=odom_yaw, unit="rad", low=0.0,
             high=c["max_hold_yaw_change_rad"], note=window + ", /odom"),
    ]
    messages = record.get("cmd_vel") or []
    peak, count = cmd_vel_in_window(messages, t0, t1) if t0 is not None else (None, 0)
    checks += [
        dict(name="cancel_hold_cmd_vel_peak", value=peak, unit="m/s or rad/s", low=0.0, high=c["max_cmd_vel_abs"],
             note=f"largest |component| of {c['cmd_vel_topic']} in the hold window ({count} messages) and of the last "
                  "message before it"),
    ]
    if record.get("t_cancel") is not None:
        checks.append(dict(name="cancel_cmd_vel_nonzero_after_cancel", value=last_nonzero_after(
            messages, record["t_cancel"], c["max_cmd_vel_abs"]), unit="s of Gazebo time", low=0.0,
            high=c["max_time_to_stop_sim_s"], note=f"last nonzero {c['cmd_vel_topic']} message after the cancel"))
    return checks


def _cancel(handle, wall_s=30.0):
    """Send the cancel request of one goal handle. -> (return_code, number of goals canceling), (None, None) on no
    answer."""
    import time
    future = handle.cancel_goal_async()
    end = time.monotonic() + wall_s
    while not future.done() and time.monotonic() < end:
        time.sleep(0.01)
    if not future.done() or future.result() is None:
        return None, None
    response = future.result()
    return int(response.return_code), len(response.goals_canceling)


def step(rig, log, pose=None, config=None):
    """Run the cancellation step against the running stack and log its checks. `pose()` gives the map pose of the base
    (baseline1.map_pose); without it ground truth is used, as the map starts at the world frame."""
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    c = config or load()
    log.data.setdefault("baseline1", {})["cancel_check"] = {"enabled": True, "config": c}
    messages = []
    subscription = rig.node.create_subscription(
        Twist, c["cmd_vel_topic"],
        lambda m: messages.append((rig.sim_time(), m.linear.x, m.linear.y, m.angular.z)), 50)
    rig.wait_stopped(10.0)
    start = rig.truth()
    here = pose() if pose is not None else None
    gx, gy, gyaw = goal_pose(here or start, c["goal_distance_m"])
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(gx), float(gy)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2.0), math.cos(gyaw / 2.0)
    goal.behavior_tree = c.get("behavior_tree", "")
    log.event(f"cancel check: goal x {gx:.3f} y {gy:.3f} yaw {gyaw:.3f} (map) on {c['action']}, cancel after "
              f"{c['cancel_at_m']} m")
    handle = rig.send_goal(c["action"], "nav2_msgs.action", "NavigateToPose", goal)
    record = {"accepted": handle is not None}
    if handle is not None:
        def moved():
            now = rig.truth()
            return math.hypot(now[0] - start[0], now[1] - start[1])
        rig.wait(lambda: moved() >= c["cancel_at_m"], c["move_timeout_sim_s"], "cancel check: base under way")
        at = rig.truth()
        record.update(progress_m=moved(), speed_m_s=rig.truth_speed()[0], t_cancel=rig.sim_time())
        record["return_code"], record["goals_canceling"] = _cancel(handle)
        t_call = rig.sim_time()
        settle = rig.wait_stopped(10.0)
        record["time_to_stop_s"] = None if settle is None else t_call + settle - record["t_cancel"]
        record["distance_after_cancel_m"] = math.hypot(rig.truth()[0] - at[0], rig.truth()[1] - at[1])
        record["status"], _ = rig.wait_result(handle, c["result_timeout_sim_s"], "cancel check: terminal status")
        t0 = rig.sim_time()
        truth, odom = [], []
        while rig.sim_time() - t0 < c["hold_window_sim_s"]:
            t = rig.sim_time()
            truth.append((t, *rig.truth()))
            odom.append((t, *rig.odom()))
            rig.sleep_sim(c["hold_sample_sim_s"])
        t1 = rig.sim_time()
        truth.append((t1, *rig.truth()))
        odom.append((t1, *rig.odom()))
        record.update(window=(t0, t1), hold_truth=truth, hold_odom=odom, cmd_vel=list(messages))
    rig.node.destroy_subscription(subscription)
    for check in verdict(record, c):
        log.quantity(check["name"], check["value"], check["unit"], low=check["low"], high=check["high"],
                     note=check["note"])
    summary = {k: v for k, v in record.items() if k not in ("hold_truth", "hold_odom", "cmd_vel")}
    summary["cmd_vel_messages"] = len(record.get("cmd_vel") or [])
    log.data["baseline1"]["cancel_check"]["record"] = summary
    return record
