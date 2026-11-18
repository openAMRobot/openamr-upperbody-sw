# SPDX-License-Identifier: MIT
"""Objects taken out of the MoveIt planning scene for one step of a scenario, and put back after it on every path out of
that step. No ROS import: the caller passes its publish and sleep functions and the std_msgs/String type.
Simulation only.
"""
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
