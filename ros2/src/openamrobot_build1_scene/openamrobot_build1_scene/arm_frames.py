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


def rotate_z(vector, angle):
    """`vector` (x, y, z) turned by `angle` about z."""
    c, s = math.cos(angle), math.sin(angle)
    return (c * vector[0] - s * vector[1], s * vector[0] + c * vector[1], vector[2])


def side_grasp(yaw=0.0):
    """SIDE_GRASP turned by `yaw` about z of base_footprint (x, y, z, w): the fingers point along `yaw` in the floor plane."""
    qz, qw = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    x, y, z, w = SIDE_GRASP
    return (qw * x - qz * y, qw * y + qz * x, qw * z + qz * w, qw * w - qz * z)


def side_grasp_offset(offset, yaw=0.0):
    """A vector of the hand base frame, expressed in base_footprint when the hand has the side_grasp(yaw) orientation."""
    x, y, z = offset
    return rotate_z((-z, y, x), yaw)


def hand_for_tcp_at(point, offset, yaw=0.0):
    """Where the hand base must go so that the TCP, `offset` from it in the hand frame, is at `point` (side_grasp(yaw))."""
    dx, dy, dz = side_grasp_offset(offset, yaw)
    return (point[0] - dx, point[1] - dy, point[2] - dz)


def face_yaw(base_yaw, item_yaw):
    """The heading in base_footprint that points the fingers square at a face of a box standing at `item_yaw` (world),
    seen from a base at `base_yaw` (world): the face nearest to the base heading, within +-45 degrees. A box has four
    faces, so the result wraps by quarter turns. 0 when the base is square to the item."""
    relative = item_yaw - base_yaw
    return (relative + math.pi / 4.0) % (math.pi / 2.0) - math.pi / 4.0
