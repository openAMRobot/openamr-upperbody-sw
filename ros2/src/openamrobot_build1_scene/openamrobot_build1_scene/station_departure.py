# SPDX-License-Identifier: MIT
"""How the Baseline 1 retreat phase leaves the work-shelf station: three options (Proposed, 20 Nov 2026; Alex decides).

BASELINE1_RETREAT selects one; the default is "backup" and nothing here changes it.

backup     today: Nav2 behavior_server's /backup action, 0.30 m at 0.10 m/s (baseline1.retreat). Needs the backup
           behaviour registered and a velocity_smoother that lets a negative linear x through. With platform-sw #90's
           behaviour set (spin and wait only, min_velocity[0] 0) there is no /backup server: the goal is not
           accepted and the retreat phase fails (STARTUP, "did not accept the goal").
departure  a declared, simulation-only station departure: one straight move back of at most max_distance_m at most
           max_speed_m_s for at most max_time_sim_s, only in the retreat phase, published by the test harness on
           cmd_vel_smoothed (collision_monitor's input; below velocity_smoother, whose clamp would zero it). Allowed
           only when the Nav2 configuration has no reverse path (reverse_paths() is empty), so it is the one reverse of
           the run and no recovery tree or Nav2 behaviour can reverse. With #90's command chain (twist_mux, stop gate,
           smoother, collision_monitor) a source below the smoother also bypasses the stop gate: on #90 this option
           needs a contract change request (a mux input and a departure-only smoother exception), so it is
           integration-branch only as written.
forward    no reverse at all: turn in place by turn_rad with Nav2's spin behaviour (kept in #90's set), then a forward
           NavigateToPose goal forward_m ahead. Allowed when the measured turn clearance (#93: 0.141 m, closed shelf,
           base 0.088 m past the station, arms in park) is at least min_clearance_m and spin is registered. The turn
           is next to the shelf, and the clearance is geometry only (no tracking error, no carry pose with an item).

The decision logic and the commands have no ROS import (test/test_station_departure.py); run() drives the stack.
Simulation evidence only: never hardware, physical or safety evidence. No stop, E-stop or interlock logic here.
"""
import math
import os
import xml.etree.ElementTree as ET

OPTIONS = ("backup", "departure", "forward")
REVERSE_BEHAVIOURS = ("nav2_behaviors::BackUp", "nav2_behaviors::DriveOnHeading", "nav2_behaviors::AssistedTeleop")
REVERSE_TREE_NODES = ("BackUp", "DriveOnHeading", "AssistedTeleop")
SUCCEEDED = 4

# What each option needs and gives up, for the decision (docs/integration/baseline1.md carries the same table).
TRADEOFFS = {
    "backup": {"reverses": True, "needs": "behavior_server backup and min_velocity[0] < 0",
               "on_no_reverse_config": "no /backup server: the retreat fails",
               "cost": "keeps a general, directly callable reverse action registered in Nav2"},
    "departure": {"reverses": True, "needs": "a Nav2 configuration with no reverse path; a harness publisher",
                  "on_no_reverse_config": "the only reverse of the run, bounded and in the retreat phase only",
                  "cost": "below the smoother; on #90 also below the stop gate: needs a contract change there"},
    "forward": {"reverses": False, "needs": "spin registered; measured turn clearance above the margin",
                "on_no_reverse_config": "works through the normal Nav2 chain",
                "cost": "a half turn next to the shelf; clearance is geometry only, carry pose not measured"},
}


def load(path=None):
    """The retreat section of config/baseline1_mission_options.json: the source tree, else the installed share."""
    import json
    from openamrobot_build1_scene import baseline1_cancel
    for candidate in ([path] if path else [baseline1_cancel.SOURCE_FILE, _share_file()]):
        if candidate and os.path.isfile(candidate):
            with open(candidate, encoding="utf-8") as f:
                return json.load(f)["retreat"]
    raise FileNotFoundError("baseline1_mission_options.json not found")


def _share_file():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config",
                            "baseline1_mission_options.json")
    except Exception:  # noqa: BLE001 - no ament index outside a sourced workspace
        return None


def option(config, environ=None):
    environ = os.environ if environ is None else environ
    value = environ.get(config["env"], config["default"])
    if value not in config["options"]:
        raise ValueError(f"{config['env']}={value!r}: not one of {config['options']}")
    return value


# ------------------------------------------------------------------ Nav2 configuration
def reverse_paths(params, trees):
    """Every reverse path of a Nav2 configuration (the same rules as openamr-platform-sw
    test_baseline1_no_reverse_paths.py on nav/baseline1-no-reverse): reverse-capable behaviours by plugin class, a
    negative DWB min_vel_x, a negative velocity_smoother min_velocity[0], a graceful controller with allow_backward and
    without initial_rotation, a Regulated Pure Pursuit controller with allow_reversing true (PROPOSED, #117), a tree
    that calls BackUp, DriveOnHeading or AssistedTeleop."""
    found = []
    behaviors = params.get("behavior_server", {}).get("ros__parameters", {})
    for name in behaviors.get("behavior_plugins", []):
        plugin = (behaviors.get(name) or {}).get("plugin")
        if plugin in REVERSE_BEHAVIOURS or plugin is None:
            found.append(f"behavior_server registers {name} ({plugin})")
    controller = params.get("controller_server", {}).get("ros__parameters", {})
    for name in controller.get("controller_plugins", []):
        values = controller.get(name, {})
        plugin, primary = values.get("plugin", ""), values.get("primary_controller", "")
        if ("dwb" in plugin.lower() or "dwb" in primary.lower()) and values.get("min_vel_x", 0.0) < 0.0:
            found.append(f"{name} min_vel_x {values['min_vel_x']}")
        if "graceful" in plugin.lower() and values.get("allow_backward") and not values.get("initial_rotation", True):
            found.append(f"{name} allow_backward without initial_rotation")
        # PROPOSED (#117, openamr-platform-sw StationRetreat): Regulated Pure Pursuit with allow_reversing true. RPP
        # only rejects a later dynamic change to true while use_rotate_to_heading is on, so any true is listed.
        if "regulated_pure_pursuit" in plugin.lower() and values.get("allow_reversing"):
            found.append(f"{name} allow_reversing true")
    smoother = params.get("velocity_smoother", {}).get("ros__parameters", {})
    if smoother.get("min_velocity", [0.0])[0] < 0.0:
        found.append(f"velocity_smoother min_velocity {smoother['min_velocity']}")
    for name, text in sorted(trees.items()):
        root = ET.fromstring(text)
        found += [f"{name} calls {tag}" for tag in REVERSE_TREE_NODES if any(True for _ in root.iter(tag))]
    return found


def behaviours(params):
    return list(params.get("behavior_server", {}).get("ros__parameters", {}).get("behavior_plugins", []))


def installed_nav2():
    """(params, trees) of the installed openamrobot_nav2 (the platform checkout the run uses)."""
    import glob
    import yaml
    from ament_index_python.packages import get_package_share_directory
    share = get_package_share_directory("openamrobot_nav2")
    with open(os.path.join(share, "config", "nav2_params.yaml"), encoding="utf-8") as f:
        params = yaml.safe_load(f)
    trees = {}
    for path in sorted(glob.glob(os.path.join(share, "behavior_trees", "*.xml"))):
        with open(path, encoding="utf-8") as f:
            trees[os.path.basename(path)] = f.read()
    return params, trees


# ------------------------------------------------------------------ preconditions
def allowed(name, config, params, trees):
    """-> (allowed, reasons) of one option against a Nav2 configuration."""
    found = reverse_paths(params, trees)
    if name == "backup":
        ok = "backup" in behaviours(params) and params["velocity_smoother"]["ros__parameters"]["min_velocity"][0] < 0
        return ok, [] if ok else ["no backup behaviour or the smoother clamps linear x at 0 (platform-sw #90 set)"]
    if name == "departure":
        plan_problems = departure_plan_problems(config)
        return (not found and not plan_problems), found + plan_problems
    if name == "forward":
        f = config["forward"]
        reasons = []
        if f["turn_clearance_measured_m"] < f["min_clearance_m"]:
            reasons.append(f"turn clearance {f['turn_clearance_measured_m']} m below {f['min_clearance_m']} m")
        if "spin" not in behaviours(params):
            reasons.append("spin is not registered in behavior_server")
        return not reasons, reasons
    raise ValueError(name)


def departure_plan_problems(config):
    d = config["departure"]
    problems = []
    if config["distance_m"] > d["max_distance_m"]:
        problems.append(f"distance {config['distance_m']} m over the cap {d['max_distance_m']} m")
    if config["speed_m_s"] > d["max_speed_m_s"]:
        problems.append(f"speed {config['speed_m_s']} m/s over the cap {d['max_speed_m_s']} m/s")
    if config["speed_m_s"] <= 0 or config["distance_m"] / config["speed_m_s"] > d["max_time_sim_s"]:
        problems.append("the distance cannot be covered at that speed within max_time_sim_s")
    if d["phase"] != "retreat" or not d["simulation_only"]:
        problems.append("the departure is declared for the retreat phase and simulation only")
    return problems


def departure_command(config, phase, travelled_m, elapsed_s):
    """Linear x of the station departure at one step: -speed until the distance is covered, 0 afterwards, after the
    time cap, and in every phase but retreat. Never beyond the caps, whatever the config asks."""
    d = config["departure"]
    if phase != d["phase"] or departure_plan_problems(config):
        return 0.0
    if travelled_m >= min(config["distance_m"], d["max_distance_m"]) or elapsed_s >= d["max_time_sim_s"]:
        return 0.0
    return -min(config["speed_m_s"], d["max_speed_m_s"])


def along(start, pose):
    """Signed displacement of `pose` from `start` along the start heading (negative: backwards)."""
    return (pose[0] - start[0]) * math.cos(start[2]) + (pose[1] - start[1]) * math.sin(start[2])


def reverse_speed(samples, min_dt=0.25):
    """samples [(t, x, y, yaw)] -> the largest backward speed along the heading (0.0 when none), each over at least
    min_dt seconds of Gazebo time. Samples a few milliseconds of Gazebo time apart divide pose jitter by almost
    nothing (diagnostic run 44736351073 read 0.236 m/s in a turn whose largest base speed was 0.082 m/s)."""
    worst = 0.0
    for i, (t1, x1, y1, yaw1) in enumerate(samples):
        later = next((s for s in samples[i + 1:] if s[0] - t1 >= min_dt), None)
        if later is None:
            break
        t2, x2, y2, _yaw2 = later
        v = ((x2 - x1) * math.cos(yaw1) + (y2 - y1) * math.sin(yaw1)) / (t2 - t1)
        worst = max(worst, -v)
    return worst


def reverse_speed_by_phase(phases, min_dt=0.25):
    """phases {name: samples [(t, x, y, yaw)]} -> ({name: reverse_speed of that phase alone}, worst). Proposed
    measurement change (gate owner decides): no 0.25 s window spans two phases, so a window from the end of the turn to
    the start of the forward drive is not read as one move; the gate value is the largest of the phases. worst is
    {"phase", "t", "v"} of the largest window, None when nothing is backward."""
    speeds, worst = {}, None
    for name, samples in phases.items():
        speeds[name] = reverse_speed(samples, min_dt)
        for i, (t1, x1, y1, yaw1) in enumerate(samples):
            later = next((s for s in samples[i + 1:] if s[0] - t1 >= min_dt), None)
            if later is None:
                break
            v = -((later[1] - x1) * math.cos(yaw1) + (later[2] - y1) * math.sin(yaw1)) / (later[0] - t1)
            if v > 0.0 and (worst is None or v > worst["v"]):
                worst = {"phase": name, "t": t1, "v": v}
    return speeds, worst


def gate_reverse_speed(speeds):
    """The C7 gate value under upperbody issue #117 option 1 (PROPOSED, gate owner decides): the reverse speed of the
    forward drive only. In the in-place turn the ground-truth reference point moves off the rotation axis (drift
    0.0136 to 0.0170 m per half turn) and its projection on the heading reads as reverse with no reverse commanded
    (turn 0.01223 m/s against forward 0.0048 m/s, run 44979089718); that motion stays guarded by
    retreat_turn_position_drift (0.05 m) and is reported as retreat_turn_peak_reverse_speed. The limit is unchanged."""
    return speeds.get("forward")


# ------------------------------------------------------------------ ROS
def _sampled_wait(rig, condition, sim_s, what, samples):
    def check():
        samples.append((rig.sim_time(), *rig.truth()))
        return condition()
    return rig.wait(check, sim_s, what, step=0.05)


def forward_goal(pose, forward_m):
    """(x, y, yaw) of the forward retreat goal: `forward_m` straight ahead of `pose`, same heading. `pose` must be in
    the frame the goal is sent in (map): a goal built from ground truth and sent in map is off by the localization
    error, which a forward-only base cannot take back (diagnostic run 44800204846: 0.05 m, FollowPath "Failed to make
    progress", ABORTED)."""
    x, y, yaw = pose
    return x + forward_m * math.cos(yaw), y + forward_m * math.sin(yaw), yaw


def run(rig, log, name, config=None, pose=None):
    """Leave the station with option `name` ('departure' or 'forward'; 'backup' stays in baseline1.retreat).
    `pose()` gives the map pose of the base (baseline1.map_pose) for the forward goal; ground truth without it."""
    import time
    c = config or load()
    params, trees = installed_nav2()
    ok, reasons = allowed(name, c, params, trees)
    log.data.setdefault("baseline1", {})["retreat_option"] = {"option": name, "allowed": ok, "reasons": reasons}
    log.quantity(f"retreat_{name}_allowed", ok, "bool", low=True, high=True,
                 note="; ".join(reasons) or "preconditions of station_departure.allowed hold")
    if not ok:
        return
    start = rig.truth()
    samples = []
    if name == "departure":
        from geometry_msgs.msg import Twist
        d = c["departure"]
        publisher = rig.node.create_publisher(Twist, d["topic"], 10)
        odom0, t0 = rig.odom(), rig.sim_time()
        peak = 0.0
        while True:
            travelled = -along(odom0, rig.odom())
            command = departure_command(c, "retreat", travelled, rig.sim_time() - t0)
            message = Twist()
            message.linear.x = command
            publisher.publish(message)
            peak = max(peak, abs(command))
            samples.append((rig.sim_time(), *rig.truth()))
            if command == 0.0:
                break
            time.sleep(1.0 / d["rate_hz"])
        for _ in range(5):
            publisher.publish(Twist())
            time.sleep(0.05)
        # The publisher stays until Rig.close(): destroying it while the rig's executor spins in another thread kills
        # that thread (its QoS event handler is in the wait set; rclpy InvalidHandle, run 44848867373).
        log.quantity("retreat_departure_peak_command", peak, "m/s", low=0.0, high=d["max_speed_m_s"],
                     note=f"published on {d['topic']}, harness only")
        log.quantity("retreat_departure_time", rig.sim_time() - t0, "s of Gazebo time", low=0.0,
                     high=d["max_time_sim_s"])
        log.quantity("retreat_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
        end = rig.truth()
        log.quantity("retreat_along_heading", along(start, end), "m", low=-(c["distance_m"] + 0.05),
                     high=-(c["distance_m"] - 0.05), note="negative: away from the shelf. Guard value, estimated")
        log.quantity("retreat_heading_change", math.degrees(math.atan2(math.sin(end[2] - start[2]),
                                                                       math.cos(end[2] - start[2]))),
                     "deg", low=-5.0, high=5.0, note="as in scn01")
        log.quantity("retreat_peak_reverse_speed", reverse_speed(samples), "m/s", low=0.0,
                     high=d["max_speed_m_s"] + 0.02, note="ground truth along the heading")
        return
    from nav2_msgs.action import NavigateToPose, Spin
    from openamrobot_build1_scene import scenarios
    f = c["forward"]
    goal = Spin.Goal()
    goal.target_yaw, goal.time_allowance = float(f["turn_rad"]), scenarios._duration(60.0)
    handle = rig.send_goal(f["spin_action"], "nav2_msgs.action", "Spin", goal)
    if not log.require(handle is not None, "Nav2 spin did not accept the goal"):
        return
    future = handle.get_result_async()
    _sampled_wait(rig, future.done, 120.0, "retreat turn", samples)
    turn_samples = list(samples)
    status = future.result().status if future.done() else None
    log.quantity("retreat_turn_status", status, "GoalStatus", low=SUCCEEDED, high=SUCCEEDED)
    turned = rig.truth()
    change = abs(math.degrees(math.atan2(math.sin(turned[2] - start[2]), math.cos(turned[2] - start[2]))))
    log.quantity("retreat_turn_heading_change", change, "deg", low=170.0, high=180.0, note="a half turn in place")
    log.quantity("retreat_turn_position_drift", math.hypot(turned[0] - start[0], turned[1] - start[1]), "m",
                 low=0.0, high=0.05, note="turn in place about base_footprint (#93's axis). Guard value, estimated")
    rig.wait_stopped(10.0)
    here = pose() if pose is not None else None
    log.data["baseline1"]["retreat_option"]["goal_from"] = "map pose" if here is not None else "ground truth"
    gx, gy, gyaw = forward_goal(here or turned, f["forward_m"])
    nav = NavigateToPose.Goal()
    nav.pose.header.frame_id = "map"
    nav.pose.pose.position.x, nav.pose.pose.position.y = float(gx), float(gy)
    nav.pose.pose.orientation.z, nav.pose.pose.orientation.w = math.sin(gyaw / 2.0), math.cos(gyaw / 2.0)
    # A station-checker tree (#164's checker, 0.025 m) ends the goal at the point; the default checker's 0.35 m would
    # count a 0.30 m goal as reached before the base moves. drive_tree names the tree (PROPOSED #117:
    # navigate_to_station_retreat.xml, the StationRetreat controller of openamr-platform-sw).
    from ament_index_python.packages import get_package_share_directory
    nav.behavior_tree = os.path.join(get_package_share_directory("openamrobot_nav2"), "behavior_trees", f["drive_tree"])
    handle = rig.send_goal(f["drive_action"], "nav2_msgs.action", "NavigateToPose", nav)
    if not log.require(handle is not None, "Nav2 navigate_to_pose did not accept the retreat goal"):
        return
    future = handle.get_result_async()
    _sampled_wait(rig, future.done, 120.0, "retreat forward", samples)
    log.quantity("retreat_forward_status", future.result().status if future.done() else None, "GoalStatus",
                 low=SUCCEEDED, high=SUCCEEDED)
    log.quantity("retreat_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    end = rig.truth()
    log.quantity("retreat_distance_from_station", math.hypot(end[0] - start[0], end[1] - start[1]), "m",
                 low=f["forward_m"] - 0.05, high=f["forward_m"] + 0.05, note="guard value, estimated")
    speeds, worst = reverse_speed_by_phase({"turn": turn_samples, "forward": samples[len(turn_samples):]})
    for phase, value in speeds.items():
        log.observed(f"retreat_{phase}_peak_reverse_speed", value, "m/s", note="this phase alone, 0.25 s windows")
    log.data["baseline1"]["retreat_option"]["worst_reverse_window"] = worst
    log.quantity("retreat_peak_reverse_speed", gate_reverse_speed(speeds), "m/s", low=0.0,
                 high=f["max_reverse_speed_m_s"], note="translational reverse of the forward drive (#117 option 1, "
                 f"PROPOSED); the turn is observed above and guarded by retreat_turn_position_drift; worst {worst}")
