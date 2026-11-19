# SPDX-License-Identifier: MIT
"""What the control room monitor computes from what it reads. No ROS import: the node (monitor.py) feeds it plain
values and the unit tests do the same.

Everything here is arithmetic on topics other nodes publish. Nothing is a check, a verdict or safety evidence: the
mission (openamrobot_build1_scene baseline1.py) owns the checks. Simulation evidence only.
"""
import math

# action_msgs/msg/GoalStatus
STATUS_NAMES = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING", 4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}
ACTIVE = (1, 2, 3)

# The action servers of the Build 1 stack whose status topics are read, and what an active goal on each means.
# The mission phase is the mission's own; this is only what the stack is observed doing.
ACTIONS = {
    "/navigate_to_pose": "navigating (Nav2)",
    "/station_alignment/navigate_to_station": "station alignment",
    "/backup": "backing up (Nav2)",
    "/move_action": "arm planning (MoveIt)",
    "/execute_trajectory": "arm executing (MoveIt)",
    "/left_arm_controller/follow_joint_trajectory": "left arm trajectory",
    "/right_arm_controller/follow_joint_trajectory": "right arm trajectory",
    "/left_gripper_controller/gripper_cmd": "left gripper",
    "/right_gripper_controller/gripper_cmd": "right gripper",
    "/lift_controller/follow_joint_trajectory": "lift trajectory (fast and fine profiles)",
}

# Proposed (not in the mission yet): a latched std_msgs/String JSON topic the Baseline 1 mission would publish from its
# phase Tracker: {"phase": name or null, "state": "running|passed|failed|blocked", "sim": seconds,
# "phases": [{"phase": name, "verdict": "PASS|FAIL|BLOCKED|NOT RUN|RUNNING"}]}. The monitor shows it when it appears.
MISSION_STATUS_TOPIC = "/baseline1/mission_status"


def yaw_of(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose_error(estimate, truth):
    """(position error m, absolute yaw error rad) of an (x, y, yaw) estimate against (x, y, yaw) ground truth."""
    if estimate is None or truth is None:
        return None, None
    return math.hypot(estimate[0] - truth[0], estimate[1] - truth[1]), abs(angle_diff(estimate[2], truth[2]))


def station_error(truth, station):
    """Ground truth against a station pose: distance, along the station heading (+ towards the shelf), to its left and
    the yaw error. The same decomposition as baseline1.nav_to_station."""
    if truth is None or station is None:
        return None
    dx, dy = truth[0] - station[0], truth[1] - station[1]
    c, s = math.cos(station[2]), math.sin(station[2])
    return {"distance_m": math.hypot(dx, dy), "along_m": dx * c + dy * s, "left_m": -dx * s + dy * c,
            "yaw_rad": abs(angle_diff(truth[2], station[2]))}


class ActionWatch:
    """The last status of every goal of every watched action server."""

    def __init__(self):
        self.goals = {}            # action -> {goal_id: status}
        self.last_terminal = {}    # action -> status name

    def update(self, action, statuses):
        """`statuses`: [(goal_id, status code)] of one GoalStatusArray."""
        goals = {gid: code for gid, code in statuses}
        self.goals[action] = goals
        terminal = [code for code in goals.values() if code not in ACTIVE]
        if terminal:
            self.last_terminal[action] = STATUS_NAMES.get(terminal[-1], str(terminal[-1]))

    def active(self):
        return [ACTIONS.get(a, a) for a, goals in sorted(self.goals.items()) if any(c in ACTIVE for c in goals.values())]

    def summary(self):
        return {a: {"active": sum(1 for c in g.values() if c in ACTIVE), "last_result": self.last_terminal.get(a)}
                for a, g in sorted(self.goals.items())}


class Motion:
    """Displacement and speed of a tracked point (the grasp item) from its ground truth pose."""

    def __init__(self):
        self.first = self.last = None
        self.speed = None          # unknown until two poses at different times: never shown as 0.0

    def update(self, t, xyz):
        if self.last is not None and t > self.last[0]:
            self.speed = math.dist(xyz, self.last[1]) / (t - self.last[0])
        if self.first is None:
            self.first = (t, xyz)
        self.last = (t, xyz)

    def summary(self):
        if self.last is None:
            return None
        return {"position": [round(v, 4) for v in self.last[1]], "displacement_m": round(math.dist(self.last[1], self.first[1]), 4),
                "speed_mps": None if self.speed is None else round(self.speed, 4)}


def contact_force(contacts):
    """Largest total force magnitude over the contacts of one ros_gz_interfaces/Contacts message, given as a list of
    lists of (fx, fy, fz) wrench forces. 0.0 for no contact."""
    best = 0.0
    for wrenches in contacts:
        total = [sum(w[i] for w in wrenches) for i in range(3)] if wrenches else [0.0, 0.0, 0.0]
        best = max(best, math.sqrt(sum(v * v for v in total)))
    return best


class Contacts:
    """Contact events per contact topic: a new contact after a quiet message starts an event."""

    def __init__(self):
        self.topics = {}           # topic -> {"in_contact": bool, "events": int, "max_force_n": float, "last_force_n": float}

    def update(self, topic, count, force):
        record = self.topics.setdefault(topic, {"in_contact": False, "events": 0, "max_force_n": 0.0, "last_force_n": 0.0})
        if count and not record["in_contact"]:
            record["events"] += 1
        record["in_contact"] = bool(count)
        record["last_force_n"] = round(force, 3)
        record["max_force_n"] = round(max(record["max_force_n"], force), 3)

    def summary(self):
        """Without a contact sensor topic (BUILD1_CONTACT_SENSORS unset) nothing is known: events, in_contact and
        max_force_n are None, never 0 (no contact measured is not the same as no contact)."""
        if not self.topics:
            return {"topics": 0, "events": None, "in_contact": None, "max_force_n": None,
                    "unknown": "no contact sensor topic (/build1/contacts/*): contact sensors run only with BUILD1_CONTACT_SENSORS=1"}
        return {"topics": len(self.topics), "events": sum(r["events"] for r in self.topics.values()),
                "in_contact": sorted(t.rsplit("/", 1)[-1] for t, r in self.topics.items() if r["in_contact"]),
                "max_force_n": max(r["max_force_n"] for r in self.topics.values())}


def scalars(value, limit=24):
    """The scalar fields of a JSON status dict (the SIL status topics), at most `limit`, for the status line."""
    if not isinstance(value, dict):
        return value
    out = {}
    for key in sorted(value):
        item = value[key]
        if isinstance(item, (bool, int, float, str)) or item is None:
            out[key] = item
        if len(out) >= limit:
            break
    return out


def mission_view(status, grocery_phase, grocery_events, observed):
    """Where the mission phase comes from, in this order: the Proposed latched /baseline1/mission_status (#82), the
    grocery mission's volatile /grocery_demo/phase, or only the activity inferred from action status topics."""
    if status:
        source = MISSION_STATUS_TOPIC + " (Proposed, openamr-upperbody-sw#82)"
    elif grocery_phase:
        source = "/grocery_demo/phase (grocery mission, volatile: phases before the monitor started are not seen)"
    else:
        source = ("none: no mission status topic seen; observed activity below is inferred from action status topics")
    return {"source": source, "status": status, "grocery_phase": grocery_phase, "grocery_events": grocery_events or None,
            "observed_activity": observed}


# What each component of a firmware_sil run is, from the code that launches it (openamrobot_build1_scene
# build1.launch.py and openamrobot_sil sil.launch.py can_nodes at openamr-platform-sw 931e631). The run's own
# /build1/run_info picks the entries; nothing is labelled before it arrives.
BASE_CONTROLLER = {
    "host": "host test stand-in (openamrobot_sil stm32_host_client), not firmware",
    "renode": "platform-fw telemetry image in Renode on vcan_drive, reads only; nobody commands the drive",
    "renode_readonly": "platform-fw telemetry image in Renode plus the host stand-in sending drive receive PDOs",
}
DEVICE_COMPONENTS = {
    "drive": "ZLAC8015D emulator (openamrobot_sil zlac8015d_emulator, host process) on vcan_drive; wheels: Gazebo physics",
    "lift": "CTL-017 emulator (openamrobot_sil ctl017_emulator, host process, CTL-017 v1 proposal) on vcan_aux; "
            "lift joint: Gazebo physics",
    "bms": "Daly BMS emulator (openamrobot_sil daly_bms_emulator, host process, protocol unconfirmed) on vcan_bms",
    "arms": "unmodified openarm_ros2 driver (real ROS 2 driver, openarm_can) on vcan_arm_l and vcan_arm_r, to emulated "
            "Damiao motors (openamrobot_openarm_emulator, a ros2_control controller in the simulator); joints: Gazebo physics",
    "can": "Linux virtual CAN (vcan): no bitrate, no arbitration delay of a physical bus, no electrical layer",
}


def provenance(run_info, writers, json_topics):
    """Labels of the run's components from /build1/run_info, and the command topic writers read from the graph."""
    out = {"hardware": "none: simulation only"}
    if not isinstance(run_info, dict):
        out["components"] = "unknown until /build1/run_info arrives"
    elif run_info.get("devices"):
        stm32 = run_info.get("stm32")
        out["components"] = dict(DEVICE_COMPONENTS, base_controller=BASE_CONTROLLER.get(stm32, f"unknown stm32 value {stm32!r}"))
        out["clocks"] = {"gazebo": "/clock (simulation time)", "arm_driver": run_info.get("driver_clock"),
                         "arm_driver_rate_hz": run_info.get("driver_rate_hz"), "emulators": "Gazebo time (use_sim_time)",
                         "candump_stamps": "host wall clock"}
    else:
        out["components"] = {"devices": "none: Gazebo ros2_control directly, no CAN devices (profile %s)" % run_info.get("profile")}
    if isinstance(run_info, dict):
        out["run"] = {k: run_info.get(k) for k in ("profile", "stm32", "physics_engine", "planning_pose_source", "scene",
                                                    "station_alignment") if k in run_info}
    cmd = writers.get("/cmd_vel")
    out["command_arbiter"] = "none launched by build1.launch.py"
    out["writers"] = dict(writers)
    if cmd is not None and cmd != ["/collision_monitor"]:
        out["warning"] = f"/cmd_vel has writers {cmd}, not only /collision_monitor"
    return out


def scene_warning(scene_file, run_info):
    """A text when the station pose comes from another scene file than the one the run uses, else None."""
    run_scene = (run_info or {}).get("scene") if isinstance(run_info, dict) else None
    used = (scene_file or "").rsplit("/", 1)[-1] or "scene.yaml"
    if run_scene and run_scene.rsplit("/", 1)[-1] != used:
        return f"the station error uses {used}, the run uses {run_scene}: restart the control room with scene:={run_scene}"
    return None


# Every topic the monitor node reads, with its producer in a firmware_sil run (build1.launch.py, openamrobot_sil at
# openamr-platform-sw 931e631). test/test_topic_audit.py checks this against the topic lists of two recorded runs and
# test_monitor_graph.py checks that the live node subscribes to exactly these (plus the action status topics).
MONITOR_TOPICS = {
    "/clock": "Gazebo (ros_gz_bridge): simulation time",
    "/tf": "robot_state_publisher, base_state_adapter (odom), AMCL (map)",
    "/tf_static": "robot_state_publisher",
    "/build1/ground_truth/odom": "Gazebo pose of the base (ros_gz_bridge): simulator ground truth",
    "/amcl_pose": "Nav2 AMCL",
    "/plan": "Nav2 planner",
    "/joint_states": "joint_state_publisher of the real-side states: base and lift from base_state_adapter (CAN-derived), "
                     "arms from the openarm_ros2 driver",
    "/sil/gz/joint_states": "Gazebo joint states (ros_gz_bridge): the physics plant",
    "/build1/ground_truth/finger_joint_states": "Gazebo finger joints: simulator ground truth",
    "/build1/ground_truth/item/pose": "Gazebo pose of the item: simulator ground truth",
    "/sil/lift/permit": "the mission (Baseline 1 stop_authorize) to the host stand-in",
    "/sil/drive/status": "host stand-in stm32_host_client: its decode of the ZLAC8015D SDO replies",
    "/sil/drive/emulator_state": "ZLAC8015D emulator node",
    "/sil/lift/status": "host stand-in stm32_host_client: its decode of CTL-017 TPDO1 and TPDO2",
    "/sil/lift/emulator_state": "CTL-017 emulator node",
    "/sil/bms/status": "host stand-in stm32_host_client: its decode of the Daly replies",
    "/sil/bms/emulator_state": "Daly BMS emulator node",
    "/sil/fw/status": "Renode firmware node (stm32:=renode or renode_readonly only)",
    "/build1/rtf_stats": "rtf_monitor.py",
    "/build1/run_info": "rtf_monitor.py (latched): the launch's own description of the run",
    MISSION_STATUS_TOPIC: "the Baseline 1 mission (Proposed, #82)",
    "/grocery_demo/phase": "the grocery mission (demo/grocery-dark-store)",
    "/grocery_demo/events": "the grocery mission (demo/grocery-dark-store)",
    "/scan_filtered": "openamrobot_perception scan_body_filter",
}
# Topics of MONITOR_TOPICS that neither recorded run has, and why. Labelled as absent, never shown as data.
NOT_IN_RECORDED_RUNS = {
    "/sil/fw/status": "both runs use stm32: host; only the Renode firmware node publishes it",
    "/sil/drive/emulator_state": "not in either bag although the ZLAC8015D node ran and the recorder asked for it; "
                                 "cause not found in the code (checked live by the control-room end-to-end run)",
    MISSION_STATUS_TOPIC: "Proposed (#82); the mission publishes it from this branch on, after both runs",
}
# Hidden topics: the recorders run without --include-hidden-topics, so no bag has them; they exist live.
ACTION_STATUS_TOPICS = tuple(f"{a}/_action/status" for a in ACTIONS)
