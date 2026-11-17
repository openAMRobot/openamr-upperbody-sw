# SPDX-License-Identifier: MIT
"""Hand orientation and TCP offset of the arm scenarios (D9 step 2). No ROS import: the static tests check it against the model.

The fingers point along -z of openarm_{side}_ee_base_link. For a side grasp they point along +x of base_footprint and
open and close in a level plane: a rotation of -90 degrees about y. The model's TCP is openarm_{side}_grasp_frame, a
fixed child of the hand base (0.138 m along the fingers at the pins); the scenarios read it from /tf and do not estimate it.
"""
import math

SIDE_GRASP = (0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5))   # x, y, z, w in base_footprint
TCP_FRAME = {side: f"openarm_{side}_grasp_frame" for side in ("left", "right")}
HAND_FRAME = {side: f"openarm_{side}_ee_base_link" for side in ("left", "right")}


def side_grasp_offset(offset):
    """A vector of the hand base frame, expressed in base_footprint when the hand has the SIDE_GRASP orientation."""
    x, y, z = offset
    return (-z, y, x)


def hand_for_tcp_at(point, offset):
    """Where the hand base must go so that the TCP, `offset` from it in the hand frame, is at `point` (SIDE_GRASP)."""
    dx, dy, dz = side_grasp_offset(offset)
    return (point[0] - dx, point[1] - dy, point[2] - dz)
