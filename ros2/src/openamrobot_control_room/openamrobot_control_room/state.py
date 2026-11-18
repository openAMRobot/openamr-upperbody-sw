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
        self.speed = 0.0

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
                "speed_mps": round(self.speed, 4)}


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
        return {"topics": len(self.topics), "events": sum(r["events"] for r in self.topics.values()),
                "in_contact": sorted(t.rsplit("/", 1)[-1] for t, r in self.topics.items() if r["in_contact"]),
                "max_force_n": max((r["max_force_n"] for r in self.topics.values()), default=0.0)}


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
