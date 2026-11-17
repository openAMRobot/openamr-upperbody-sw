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
    """`boxes` [(name, size, centre, yaw)] with `margin` added on every side of each box named in `fixed_names`."""
    out = []
    for name, size, centre, yaw in boxes:
        if name in fixed_names:
            size = tuple(float(v) + 2.0 * margin for v in size)
        out.append((name, tuple(size), centre, yaw))
    return out


# Where the planning scene puts the objects, relative to the robot. The scenarios place the grasp and the reach targets
# from the ground truth base pose (a stand-in for perceiving them from the robot), so the obstacles next to those
# targets must come from the same pose. From the localization (AMCL) they were 0.023 to 0.064 m and up to 0.024 rad out
# of place in the Baseline 1 missions of 17 Nov (runs 44155397328 to 44155485209): the straight lift and reach were
# then cut short against a board or tote that was not there (Cartesian fractions 0.9375 and 0.185).
POSE_SOURCES = ("ground_truth", "localization")
DEFAULT_POSE_SOURCE = "ground_truth"


def choose_base_pose(source, ground_truth, localization):
    """(pose, label) for the planning scene: (x, y, yaw) of base_footprint in the scene frame, or (None, None)."""
    if source not in POSE_SOURCES:
        raise ValueError(f"pose_source must be one of {POSE_SOURCES}, got {source!r}")
    if source == "ground_truth":
        return (ground_truth, "ground truth") if ground_truth is not None else (None, None)
    return localization
