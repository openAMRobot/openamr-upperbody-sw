# SPDX-License-Identifier: MIT
"""Objects taken out of the MoveIt planning scene for one step of a scenario, and put back after it on every path out of
that step. No ROS import: the caller passes its publish and sleep functions and the std_msgs/String type.
Simulation only.
"""
import math
from contextlib import contextmanager

SETTLE_S = 2.0      # Gazebo seconds for planning_scene_loader.py and move_group to take the change


@contextmanager
def excluded_from_planning(publish, sleep, names, message):
    """Inside the block the comma separated `names` are out of the planning scene (/build1/planning_scene/exclude).
    On leaving it, also by a return or an exception, the exclusion is emptied and every object is back."""
    publish("exclude", message(data=names))
    sleep(SETTLE_S)
    try:
        yield
    finally:
        publish("exclude", message(data=""))
        sleep(SETTLE_S)


# MoveIt's planning clearance to the fixed boxes of the scene (shelf boards, posts, floor boxes), added to every side of
# each box in the planning scene only; Gazebo's boxes are unchanged. MoveIt plans with zero padding. With none, a finger
# hit work_shelf_board_0 in most scn03 reaches and returns and Baseline 1 retreats (9.8 to 370 N, diagnostics runs
# 44137806328 and 44138101782); the arm wedged on the board edge and MoveIt refused the next start state (-10).
# The executed hand base leaves its planned path by at most 0.009 m where it touches nothing, and a wrist error of
# 0.03 rad moves the fingertips 0.138 m out by 0.004 m more (diagnostics runs of bbd1346 and d139f76): 0.013 m. 0.04 m was tried and
# closed the 0.25 m between the two work shelf boards: the lift and the level 0 reaches became infeasible (Cartesian
# fractions 0.52 to 0.88, MoveIt 99999). The tote, the item and the payload get no padding: the fingers close on the item.
FIXTURE_PADDING_M = 0.015


def padded(boxes, fixed_names, margin):
    """`boxes` [(name, size, centre, yaw)] with `margin` added on every side but the underside of each box named in
    `fixed_names`. The undersides stay: under the work shelf board above, the lifted hand has 0.004 m to the padded
    face with the hand level, and a held orientation tilted 1 to 7 degrees cut the 0.08 m lift short (Cartesian fractions
    0.9375 in Baseline 1 run 44155397328, 0.941 in diagnostics run 44168251650); no finger touched an underside."""
    out = []
    for name, size, centre, yaw in boxes:
        if name in fixed_names:
            size = (float(size[0]) + 2.0 * margin, float(size[1]) + 2.0 * margin, float(size[2]) + margin)
            centre = (centre[0], centre[1], centre[2] + margin / 2.0)
        out.append((name, tuple(size), centre, yaw))
    return out


# Where the planning scene puts the objects, relative to the robot (launch argument planning_pose_source, Proposed).
# map (default): the robot's own estimate, map -> base_footprint from AMCL, or the spawn pose and odom before a map is up.
# ground_truth: Gazebo's pose of the base, a simulation oracle. It must be asked for, and every run that uses it says so
# in /build1/run_info, in each scenario's JSON (run.planning_pose_source) and in its log.
# Why the oracle exists: the scenarios place their grasp targets from the ground truth base pose (a stand-in for
# perceiving the item from the robot). With map the obstacles were 0.023 to 0.064 m and up to 0.024 rad away from that
# frame in the Baseline 1 missions of 17 Nov, and a straight reach was cut short (Cartesian fraction 0.185, 44155471421).
POSE_SOURCES = ("map", "ground_truth")
DEFAULT_POSE_SOURCE = "map"
ORACLE_POSE_SOURCE = "ground_truth"


def choose_base_pose(source, ground_truth, localization):
    """(pose, label) for the planning scene: (x, y, yaw) of base_footprint in the scene frame, or (None, None).
    `localization` is the (pose, label) of the robot's own estimate; there is no fall-back between the two sources."""
    if source not in POSE_SOURCES:
        raise ValueError(f"pose_source must be one of {POSE_SOURCES}, got {source!r}")
    if source == ORACLE_POSE_SOURCE:
        return (ground_truth, "ground truth (simulation oracle)") if ground_truth is not None else (None, None)
    return localization


def declaration(source):
    """The line every run prints about where its planning scene comes from."""
    oracle = " (SIMULATION ORACLE: obstacles from Gazebo's ground truth, not from the robot's estimate)" if source == ORACLE_POSE_SOURCE else ""
    return f"planning_pose_source: {source}{oracle}"


# The movable objects (tote, item, payload) are placed from their observed poses, the same input the scenarios take
# their grasp targets from. Where those poses come from is its own declared source, object_pose_source (launch argument,
# Proposed), independent of planning_pose_source. Its only implementation today is ground_truth, Gazebo's pose of each
# object (/build1/ground_truth/<name>/pose), a stand-in for perception. It is recorded in /build1/run_info, every
# scenario's JSON (run.object_pose_source) and the logs; a reader without the declaration is refused. With
# object_pose_source ground_truth no run counts as the perception profile until an object detector replaces it. Before 18 Nov they stayed at their scene.yaml start poses: when the scn03 reach knocked the item over (it lay
# at z 0.8425, 0.09 m from its start), MoveIt still had it standing under the hand and refused every return move as a
# start state in collision (item - openarm_left_ee_base_link, job 134653118730), and the arm stayed off park.
MOVABLE_OBJECTS = ("tote", "item", "payload")
OBJECT_MOVED_M = 0.01
OBJECT_TURNED_RAD = 0.05
OBJECT_POSE_SOURCES = ("ground_truth",)
DEFAULT_OBJECT_POSE_SOURCE = "ground_truth"


def object_pose_topic(source, name):
    """The topic of object `name`'s pose under the declared `source`. Refuses an undeclared or unknown source, so no
    reader takes Gazebo's object poses without the run saying so."""
    if source not in OBJECT_POSE_SOURCES:
        raise ValueError(f"object_pose_source must be declared as one of {OBJECT_POSE_SOURCES}, got {source!r}")
    return f"/build1/ground_truth/{name}/pose"


def object_declaration(source):
    """The line every run prints about where its object poses and grasp targets come from."""
    if source == "ground_truth":
        return ("object_pose_source: ground_truth (PERCEPTION STAND-IN: tote, item and payload poses and the grasp targets "
                "come from Gazebo's ground truth; no run with it counts as the perception profile until an object detector "
                "replaces it)")
    return f"object_pose_source: {source}"


def _quat_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def _rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # v' = v + 2 w (q x v) + 2 q x (q x v)
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def object_box_in_base(base_pose, position, quaternion, centre_offset_z=0.0):
    """Centre and orientation (x, y, z, w) in base_footprint of an object's collision box. `base_pose` (x, y, yaw) of
    base_footprint in the scene frame; `position`, `quaternion` the object's pose in the scene frame; the box centre is
    `centre_offset_z` along the object's own z from that pose (the tote's pose is its bottom)."""
    bx, by, byaw = base_pose
    offset = _rotate(quaternion, (0.0, 0.0, centre_offset_z))
    wx, wy, wz = position[0] + offset[0] - bx, position[1] + offset[1] - by, position[2] + offset[2]
    c, s = math.cos(-byaw), math.sin(-byaw)
    turn = (0.0, 0.0, math.sin(-byaw / 2.0), math.cos(-byaw / 2.0))
    return (c * wx - s * wy, s * wx + c * wy, wz), _quat_multiply(turn, quaternion)


def object_moved(last, now, moved_m=OBJECT_MOVED_M, turned_rad=OBJECT_TURNED_RAD):
    """True when an object's pose (position, quaternion) changed enough since the last send to send the scene again."""
    if last is None:
        return now is not None
    (p0, q0), (p1, q1) = last, now
    dot = abs(sum(a * b for a, b in zip(q0, q1)))
    return math.dist(p0, p1) > moved_m or 2.0 * math.acos(min(1.0, dot)) > turned_rad
