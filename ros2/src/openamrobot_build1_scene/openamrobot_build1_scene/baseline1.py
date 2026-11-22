# SPDX-License-Identifier: MIT
"""Integrated Digital Twin Baseline 1: one whole-robot mission in one launch, phase by phase.

spawn > nav_to_station > stop_authorize > lift_to_target > arm_approach > grasp > lift_object > retreat (with the
declared cancellation check as its last step, baseline1_cancel.py).
The checks are the existing ones of scenarios.py and rig.py wherever they exist (lift refusal of scn02, park and grasp
of scn04, lift_move, wait_stopped); this module adds the navigation, authorization, lift and retreat steps around them,
tags every check with its phase (baseline1_phases.Tracker) and records a synchronized trace of base, lift, hand and
item on the Gazebo time base. Simulation evidence only: never hardware, physical acceptance or safety evidence.
"""
import math
import os
import threading
import time

from openamrobot_build1_scene import baseline1_phases as phases
from openamrobot_build1_scene.rig import LIFT_STOP_M, Blocked, angle_diff

SEED = int(os.environ.get("BASELINE1_SEED", "20261117"))
SPAWN_STATION, WORK_STATION = "origin", "work_shelf"
LIFT_SEQUENCE = (1, 0)          # L1175, then back to L1000: the grasp height of shelf level 0 (scn03 uses stop 0 there)
NAV_SIM_S = 300.0
# Nav2 goal checker of openamr-platform-sw nav2_params.yaml: xy_goal_tolerance 0.35 m, yaw_goal_tolerance 0.35 rad.
NAV_XY_TOLERANCE_M, NAV_YAW_TOLERANCE_RAD = 0.35, 0.35
# The arm-reach budget of the work-shelf grasp (docs/integration/station_approach.md): every base pose error up to
# 0.04 m and 0.08 rad keeps the scn04 approach, grasp and lift poses reachable. Derived, simulation only.
REACH_XY_M, REACH_YAW_RAD = 0.04, 0.08
# openamr-platform-sw #164: the station tree (default navigation, then StationPlanner, StationApproach and
# station_goal_checker). BASELINE1_STATION_APPROACH=0 sends the goal with the default tree (the revert run).
STATION_TREE = "navigate_to_station.xml"
# openamr-platform-sw #167 (Proposed): the marker alignment action. It sends the goal to Nav2 with STATION_TREE first,
# then corrects it from the shelf marker. BASELINE1_STATION_ALIGNMENT=0 sends the goal to Nav2 directly (the revert run).
ALIGNMENT_ACTION = "/station_alignment/navigate_to_station"


def alignment_on():
    return os.environ.get("BASELINE1_STATION_ALIGNMENT", "1") != "0" and os.environ.get("BASELINE1_STATION_APPROACH", "1") != "0"
SUCCEEDED = 4                   # action_msgs/GoalStatus


class Trace:
    """Samples every channel of the run on one time base, every TRACE_SIM_S Gazebo seconds, in its own thread."""
    TRACE_SIM_S = 0.5
    LIMIT = 6000

    def __init__(self, rig, tracker):
        self.rig, self.tracker, self.samples = rig, tracker, []
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _read(self, function):
        try:
            return function()
        except Exception:  # noqa: BLE001 - a channel without data is recorded as None
            return None

    def sample(self):
        rig = self.rig
        truth, odom, speed = self._read(rig.truth), self._read(rig.odom), self._read(rig.truth_speed)
        amcl = self._read(lambda: map_pose(rig))
        item = self._read(lambda: rig.object_position("item"))
        return {"sim": rig.sim_time(), "wall": time.time(), "phase": self.tracker.current,
                "x": truth[0] if truth else None, "y": truth[1] if truth else None, "yaw": truth[2] if truth else None,
                "v": speed[0] if speed else None, "w": speed[1] if speed else None,
                "odom_err": math.hypot(odom[0] - truth[0], odom[1] - truth[1]) if truth and odom else None,
                "odom_yaw_err": abs(angle_diff(odom[2], truth[2])) if truth and odom else None,
                "map_err": math.hypot(amcl[0] - truth[0], amcl[1] - truth[1]) if truth and amcl else None,
                "lift": self._read(rig.lift_truth), "lift_js": self._read(lambda: rig.joint("lift_joint")),
                "finger": self._read(lambda: rig.physics_joint("openarm_left_finger_joint1")),
                "item_z": item[2] if item else None, "rtf": self._read(rig.rtf), **self.counters()}

    def counters(self):
        """The work counter of each component, which only that component's own work advances (baseline1_gates.liveness
        requires each to advance through the whole run): the drive host's 606Ch samples, the BMS host's answers, the
        CTL-017 emulator's received permit frames, and the number of /joint_states messages."""
        rig = self.rig
        return {"live_drive_samples": self._read(lambda: (rig.get("/sil/drive/status") or {}).get("samples")),
                "live_bms_answers": self._read(lambda: (rig.get("/sil/bms/status") or {}).get("answers")),
                "live_lift_permit_rx": self._read(lambda: (rig.get("/sil/lift/emulator_state") or {}).get("permit_rx_count")),
                "live_joint_states": self._read(lambda: rig.count("/joint_states") or None)}

    def _loop(self):
        last = None
        while self.running and len(self.samples) < self.LIMIT:
            now = self.rig.sim_time()
            if last is None or now - last >= self.TRACE_SIM_S:
                last = now
                self.samples.append(self.sample())
            time.sleep(0.05)

    def stop(self):
        self.running = False
        self.thread.join(timeout=2.0)


def _station(name):
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_build1_scene import scene as scene_model
    scene = scene_model.load(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml"))
    return scene_model.station(scene, name)


def map_pose(rig):
    """(x, y, yaw) of base_footprint in map from /tf: AMCL's map > odom correction plus odometry. None without /tf."""
    found = rig.transform("map", "base_footprint")
    if found is None:
        return None
    (x, y, _z), q = found
    return x, y, math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def station_tree():
    if os.environ.get("BASELINE1_STATION_APPROACH", "1") == "0":
        return ""
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory("openamrobot_nav2"), "behavior_trees", STATION_TREE)


def spawn(rig, log):
    sx, sy, syaw = _station(SPAWN_STATION)
    x, y, yaw = rig.truth()
    log.quantity("spawn_position_error", math.hypot(x - sx, y - sy), "m", low=0.0, high=0.02,
                 note="ground truth against config/scene.yaml stations.origin. Guard value, estimated")
    log.quantity("spawn_yaw_error", abs(angle_diff(yaw, syaw)), "rad", low=0.0, high=0.02, note="guard value, estimated")
    log.quantity("spawn_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0,
                 note="until below 0.01 m/s and 0.02 rad/s for 0.5 s (rig.base_stopped)")
    ox, oy, _ = rig.odom()
    log.quantity("spawn_odometry_minus_ground_truth", math.hypot(ox - x, oy - y), "m", low=0.0, high=0.02,
                 note="odom starts at the spawn pose, which is the world origin here. Guard value, estimated")
    if rig.devices:
        log.quantity("spawn_drive_configured", bool(rig.get("/sil/drive/status")["configured"]), "bool", low=True, high=True)


class AmclListener:
    """The latest /amcl_pose (map pose and its covariance) and how many AMCL published, for the localization record."""

    def __init__(self, rig):
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from rclpy.qos import DurabilityPolicy, QoSProfile
        self.rig, self.last, self.count = rig, None, 0
        qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        rig.node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on, qos)

    def _on(self, msg):
        self.last, self.count = msg, self.count + 1


def localization_record(rig, log, amcl, truth, count_at_start):
    """AMCL against ground truth at the station: 1-sigma of its covariance, updates during the navigation, LiDAR beams
    with a return. Recorded, no limits: docs/integration/station_approach.md turns them into run statistics."""
    log.quantity("loc_amcl_updates_during_nav", amcl.count - count_at_start, "messages", note="/amcl_pose during nav_to_station")
    msg = amcl.last
    if msg is not None:
        c = msg.pose.covariance
        log.quantity("loc_amcl_sigma_x", math.sqrt(max(c[0], 0.0)), "m", note="sqrt of covariance[0], map frame")
        log.quantity("loc_amcl_sigma_y", math.sqrt(max(c[7], 0.0)), "m", note="sqrt of covariance[7]")
        log.quantity("loc_amcl_sigma_yaw", math.sqrt(max(c[35], 0.0)), "rad", note="sqrt of covariance[35]")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        log.quantity("loc_amcl_last_update_age", rig.sim_time() - stamp, "s of Gazebo time", note="at the station check")
        p = msg.pose.pose
        from openamrobot_build1_scene.rig import yaw_of
        log.quantity("loc_amcl_pose_minus_ground_truth", math.hypot(p.position.x - truth[0], p.position.y - truth[1]), "m",
                     note="the last /amcl_pose, not the /tf map pose")
        log.quantity("loc_amcl_pose_yaw_minus_ground_truth", abs(angle_diff(yaw_of(p.orientation), truth[2])), "rad")
    scan = rig.get("/scan")
    if scan is not None:
        log.quantity("loc_scan_beams_with_return", sum(1 for v in scan.ranges if math.isfinite(v)), "beams",
                     note=f"of {len(scan.ranges)}, at the station")


def scan_returns(rig, log):
    """How the simulated LiDAR reports beams without a return: +inf (REP 117) or range_max. A no-return beam at
    range_max is a false obstacle for the costmaps and a false wall for AMCL (openamr-platform-sw #165)."""
    for topic in ("/scan", "/scan_filtered"):
        scan = rig.get(topic)
        if scan is None:
            log.event(f"{topic}: no message, no-return beams not recorded")
            continue
        ranges = list(scan.ranges)
        key = topic.strip("/").replace("/", "_")
        log.quantity(f"{key}_beams", len(ranges), "beams")
        log.quantity(f"{key}_no_return_inf", sum(1 for v in ranges if math.isinf(v)), "beams")
        log.quantity(f"{key}_nan", sum(1 for v in ranges if math.isnan(v)), "beams")
        log.quantity(f"{key}_at_range_max", sum(1 for v in ranges if math.isfinite(v) and v >= scan.range_max - 1e-3),
                     "beams", note=f"finite readings at range_max {scan.range_max:.2f} m: a no-return reported as a range")
        log.quantity(f"{key}_below_range_min", sum(1 for v in ranges if math.isfinite(v) and v < scan.range_min), "beams")


def nav_to_station(rig, log):
    from nav2_msgs.action import NavigateToPose
    # The cancellation check runs at the end of the mission (end_of_mission), not here: at the start of nav_to_station
    # it was followed by the station turn, whose slip failed the odometry guard below (runs 44735544898, 44736351073).
    scan_returns(rig, log)
    amcl_topic = AmclListener(rig)
    rig.sleep_sim(0.5)
    amcl_count0 = amcl_topic.count
    gx, gy, gyaw = _station(WORK_STATION)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"     # AMCL starts at the spawn pose in the world frame (launch_support.nav2_params)
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(gx), float(gy)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2.0), math.cos(gyaw / 2.0)
    goal.behavior_tree = station_tree()
    log.data["baseline1"]["behavior_tree"] = goal.behavior_tree or "default (bt_navigator)"
    log.event(f"nav goal {WORK_STATION} x {gx:.3f} y {gy:.3f} yaw {gyaw:.3f} (map), "
              f"behavior tree {goal.behavior_tree or 'default'}")
    start = rig.sim_time()
    server = ALIGNMENT_ACTION if alignment_on() else "/navigate_to_pose"
    log.data["baseline1"]["navigate_action"] = server
    reports = []
    if alignment_on():
        from std_msgs.msg import String
        rig.node.create_subscription(String, "/station_alignment/report", lambda m: reports.append(m.data), 10)
    handle = rig.send_goal(server, "nav2_msgs.action", "NavigateToPose", goal)
    if not log.require(handle is not None, "Nav2 navigate_to_pose did not accept the goal"):
        return
    status, _ = rig.wait_result(handle, NAV_SIM_S, f"navigate to {WORK_STATION} through {server}")
    for report in reports:
        log.event(f"station alignment report: {report}")
    log.data["baseline1"]["alignment_reports"] = list(reports)
    log.quantity("nav_status", status, "GoalStatus", low=SUCCEEDED, high=SUCCEEDED, note="4 SUCCEEDED, 6 ABORTED, None: no result in time")
    log.quantity("nav_time", rig.sim_time() - start, "s of Gazebo time")
    log.quantity("nav_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    x, y, yaw = rig.truth()
    ox, oy, _ = rig.odom()
    log.quantity("nav_ground_truth_position_error", math.hypot(x - gx, y - gy), "m", low=0.0, high=NAV_XY_TOLERANCE_M,
                 note="against the goal. Nav2 xy_goal_tolerance of openamr-platform-sw nav2_params.yaml")
    log.quantity("nav_ground_truth_yaw_error", abs(angle_diff(yaw, gyaw)), "rad", low=0.0, high=NAV_YAW_TOLERANCE_RAD,
                 note="Nav2 yaw_goal_tolerance of openamr-platform-sw nav2_params.yaml")
    log.quantity("nav_odometry_minus_ground_truth", math.hypot(ox - x, oy - y), "m", low=0.0, high=0.10,
                 note="the guard value of scn01 for 2 m, estimated")
    _, _, oyaw = rig.odom()
    log.quantity("nav_odometry_yaw_minus_ground_truth", abs(angle_diff(oyaw, yaw)), "rad",
                 note="recorded, no limit: the firmware odometry heading against Gazebo")
    amcl = map_pose(rig)
    if amcl is not None:
        log.quantity("nav_localization_minus_ground_truth", math.hypot(amcl[0] - x, amcl[1] - y), "m",
                     note="map > base_footprint (AMCL) against Gazebo. Recorded; the station approach leaves 0.015 m to it")
        log.quantity("nav_localization_yaw_minus_ground_truth", abs(angle_diff(amcl[2], yaw)), "rad",
                     note="recorded; the station approach leaves 0.03 rad to it")
        log.quantity("nav_localization_position_error", math.hypot(amcl[0] - gx, amcl[1] - gy), "m",
                     note="where Nav2 believes the base is, against the goal")
    else:
        log.event("no map > base_footprint transform: localization not recorded")
    # Along the station heading (+ towards the shelf) and to its left: the reach is asymmetric (docs/integration).
    along = (x - gx) * math.cos(gyaw) + (y - gy) * math.sin(gyaw)
    left = -(x - gx) * math.sin(gyaw) + (y - gy) * math.cos(gyaw)
    log.quantity("nav_station_error_along_heading", along, "m", note="+ is closer to the shelf; reachable -0.06 to +0.12 alone")
    log.quantity("nav_station_error_left", left, "m", note="reachable +-0.15 alone")
    localization_record(rig, log, amcl_topic, (x, y, yaw), amcl_count0)
    log.quantity("station_reach_position_error", math.hypot(x - gx, y - gy), "m", low=0.0, high=REACH_XY_M,
                 note="ground truth against the station: the arm-reach budget (docs/integration/station_approach.md), derived")
    log.quantity("station_reach_yaw_error", abs(angle_diff(yaw, gyaw)), "rad", low=0.0, high=REACH_YAW_RAD,
                 note="the arm-reach budget, derived")
    if rig.devices:
        log.quantity("nav_host_client_drive_thread_alive", rig.drive_host_alive(), "bool", low=True, high=True,
                     note="false: the drive thread of openamrobot_sil's host test client ended (an unexpected SDO reply, "
                          "seen on slow runners); the base could not be commanded after that")


def stop_authorize(rig, log):
    from openamrobot_build1_scene import scenarios
    log.quantity("stop_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    v, w = rig.truth_speed()
    log.quantity("stop_base_speed", v, "m/s", low=0.0, high=0.01, note="the provisional base-stopped rule of 13 Nov 2026")
    log.quantity("stop_base_yaw_rate", w, "rad/s", low=0.0, high=0.02)
    if not rig.devices:
        log.event("no device emulators in this profile: no lift permit to authorize")
        return
    emulator = "/sil/lift/emulator_state"
    log.require(rig.get(emulator)["permit_gate"], "the permit gate of the CTL-017 emulator is off")
    rig.lift_permit(False)
    rig.wait_permit_state(False, 3.0, "permit withdrawn")
    failures = len(log.failures)
    scenarios._lift_refused(rig, log, LIFT_SEQUENCE[0], "authorize")     # scn02's refusal check, unchanged
    # The refusal as a named check, so the strict verdict can require that it ran (baseline1_gates.stop_cancel).
    refused = not any("not refused" in text for text in log.failures[failures:])
    log.quantity("authorize_command_refused", refused, "bool", low=True, high=True,
                 note="the lift command without permit was refused (scn02's check above)")
    rig.lift_permit(True)
    fresh = bool(rig.wait_permit_fresh_after_grant(3.0))
    log.require(fresh, "authorize: the permit did not become fresh")
    log.quantity("authorize_permit_fresh", fresh, "bool", low=True, high=True, note="the CTL-017 emulator holds a fresh permit")
    rig.lift_permit(False)                                               # lift_move grants it again per move


def lift_to_target(rig, log):
    for stop in LIFT_SEQUENCE:
        before = rig.lift_truth()
        log.quantity(f"lift_stop_{stop}_expected_travel", abs(LIFT_STOP_M[stop] - before), "m", kind="expected")
        arrived = rig.lift_move(stop, 60.0)
        log.require(arrived, f"lift did not arrive at stop {stop}")
        rig.sleep_sim(0.5)
        log.quantity(f"lift_stop_{stop}_gazebo_position", rig.lift_truth(), "m",
                     low=LIFT_STOP_M[stop] - 0.006, high=LIFT_STOP_M[stop] + 0.006, note="the window of scn02")
        log.quantity(f"lift_stop_{stop}_joint_state_position", rig.joint("lift_joint"), "m",
                     low=LIFT_STOP_M[stop] - 0.008, high=LIFT_STOP_M[stop] + 0.008, note="/joint_states, the window of scn02")
        if rig.devices:
            log.quantity(f"lift_stop_{stop}_fault_bits", rig.get("/sil/lift/emulator_state")["faults"], "bits", low=0, high=0)


def retreat(rig, log):
    from openamrobot_build1_scene import scenarios
    for side in ("left", "right"):
        # scn03's return move: a MoveIt joint goal to park, so the arm does not sweep through the shelf.
        park = {f"openarm_{side}_joint{i}": value for i, value in enumerate(scenarios.PARK, 1)}
        scenarios._move_group(rig, log, scenarios._joint_goal(f"{side}_arm", park, 0.3), f"retreat_{side}_to_park", attempts=3)
        rig.sleep_sim(1.0)
        away = max(abs(rig.physics_joint(f"openarm_{side}_joint{i}") - value) for i, value in enumerate(scenarios.PARK, 1))
        log.quantity(f"retreat_{side}_distance_from_park", away, "rad", low=0.0, high=0.1, note="as in scn03")
    # How the base leaves the station (station_departure.py): BASELINE1_RETREAT forward (default, turn and forward, no
    # reverse; Alex 20 Nov 2026) or departure (not enabled). backup sends no goal: Nav2's behaviour set has no /backup
    # server (openamr-platform-sw #178, spin and wait only), so the option fails here instead of waiting on it.
    from openamrobot_build1_scene import station_departure
    retreat_config = station_departure.load()
    chosen = station_departure.option(retreat_config)
    log.data.setdefault("baseline1", {})["retreat_option"] = {"option": chosen}
    if chosen != "backup":
        station_departure.run(rig, log, chosen, retreat_config, pose=lambda: map_pose(rig))
        return
    log.require(False, "BASELINE1_RETREAT=backup: no Nav2 backup behaviour in the no-reverse behaviour set; use forward")


def end_of_mission_cancel(rig, log):
    """The declared cancellation check (BASELINE1_CANCEL_CHECK=1, baseline1_cancel.py) as the last step of the mission:
    after the retreat, on open floor ahead of the base, so the odometry guard at the station is already evaluated and
    no turn follows the cancel. Recorded as off when the option is not set."""
    from openamrobot_build1_scene import baseline1_cancel
    options = baseline1_cancel.load()
    enabled = baseline1_cancel.enabled(options)
    log.data.setdefault("baseline1", {})["cancel_check"] = {"enabled": enabled, "position": options.get("position")}
    if not enabled:
        return None
    log.event(f"cancel check at position {options.get('position')}: after the retreat, before the evidence close")
    return baseline1_cancel.step(rig, log, pose=lambda: map_pose(rig), config=options)


def retreat_then_cancel(rig, log):
    """The retreat phase: leave the station, then the end-of-mission cancellation check."""
    retreat(rig, log)
    end_of_mission_cancel(rig, log)


def mission(rig, log):
    from openamrobot_build1_scene import scenarios
    log.event(f"Baseline 1 mission, seed {SEED} (BASELINE1_SEED): Python random and the ROS domain of the test. "
              "Gazebo is started without a seed argument; the launch has none")
    log.data["baseline1"] = {"seed": SEED, "spawn": SPAWN_STATION, "station": WORK_STATION, "lift_sequence": list(LIFT_SEQUENCE)}
    from openamrobot_build1_scene.baseline1_record import Recorders
    tracker = phases.Tracker(log, clock=rig.sim_time)
    recorders = Recorders(log)
    try:
        recorders.start(rig)
        recorders.subscribe_frames(rig)
    except Exception as error:  # noqa: BLE001 - a recording is never a check
        log.event(f"recording: {error}")
    from openamrobot_build1_scene.baseline1_record import ContactLog
    contacts = None
    try:
        contacts = ContactLog(rig, log)
    except Exception as error:  # noqa: BLE001 - recorded; the strict verdict fails a run without a contact record
        log.event(f"contacts: {error}")
    # Time marks of scenarios._mark (Gazebo time), for the contact windows of config/baseline1_gates.json.
    rig.marks = []
    trace = Trace(rig, tracker).start()
    try:
        tracker.run("spawn", lambda: spawn(rig, log))
        tracker.run("nav_to_station", lambda: nav_to_station(rig, log))
        tracker.run("stop_authorize", lambda: stop_authorize(rig, log))
        tracker.run("lift_to_target", lambda: lift_to_target(rig, log))
        # scn04's grasp, unchanged: park, open, approach, reach, close, lift 0.08 m, hold, release.
        tracker.run("arm_approach", lambda: scenarios.grasp(rig, log), then=("grasp", "lift_object"))
        tracker.run("retreat", lambda: retreat_then_cancel(rig, log))
    finally:
        trace.stop()
        recorders.stop()
        log.data["phases"] = tracker.result()
        log.data["trace"] = trace.samples
        log.data["marks"] = [{"label": label, "t": t} for label, t in list(getattr(rig, "marks", None) or [])]
        if contacts is not None:
            log.data["contacts"] = contacts.record(tracker.result(), getattr(rig, "marks", None) or [])
        for line in phases.trace_summary(trace.samples, tracker.result()):
            print(f"BASELINE1 TRACE {line}", flush=True)
        for line in phases.trace_table(trace.samples):
            print(f"BASELINE1 SAMPLE {line}", flush=True)
        print(phases.render(phases.rows(log.data)), flush=True)
    blocked = tracker.blocked()
    if blocked and not log.failures:
        raise Blocked("; ".join(f"{r['phase']}: {r['exception']['text']}" for r in blocked))
