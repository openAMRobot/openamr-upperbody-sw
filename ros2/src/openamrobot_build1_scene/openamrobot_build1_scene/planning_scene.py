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
