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
# each box in the planning scene only; Gazebo's boxes are unchanged. MoveIt plans with zero padding, and the executed arm
# leaves its planned joint path by up to 0.064 rad where it touches nothing (diagnostics run 44137806328, scn03 returns
# and reaches), about 0.038 m at the hand 0.6 m from the shoulder. In 6 of 10 recorded reaches and returns a finger then
# hit work_shelf_board_0 (9.8 to 89 N): the arm wedged on the board edge, MoveIt refused the next start state (-10) and the
# arm stayed off park. The tote, the item and the payload get no padding: the fingers must close on the item.
FIXTURE_PADDING_M = 0.04


def padded(boxes, fixed_names, margin):
    """`boxes` [(name, size, centre, yaw)] with `margin` added on every side of each box named in `fixed_names`."""
    out = []
    for name, size, centre, yaw in boxes:
        if name in fixed_names:
            size = tuple(float(v) + 2.0 * margin for v in size)
        out.append((name, tuple(size), centre, yaw))
    return out
