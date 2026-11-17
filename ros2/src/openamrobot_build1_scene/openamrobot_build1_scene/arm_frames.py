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


# The reach and the lift of scn04 are straight lines of the hand with its orientation held, computed by MoveIt's Cartesian
# path service. A pose goal with 0.3 rad per axis let the hand turn by 8 to 31 degrees between the approach, the grasp and
# the lift (diagnostics run 44131608383 and the Baseline 1 runs of 17 Nov): the fingers swept the item 24 to 55 mm before
# they closed, and the TCP, 0.138 m from the hand base, rose 21 to 49 mm when the hand base rose 64 to 91 mm.
LIFT_RISE_M = 0.08
REACH_STANDOFF_M = 0.10
# The approach goal's orientation tolerance per axis. The reach keeps the orientation the approach ended with, and with
# 0.3 rad the hand could be tilted by up to 0.52 rad, enough to bring the front of the hand body below the tote wall top.
# With 0.12 rad (up to 0.21 rad combined) and GRASP_ABOVE_ITEM_CENTRE_M the lowest point clears the wall (test_arm_frames.py).
APPROACH_ANGLE_TOLERANCE_RAD = 0.12
# How far above the item's centre the TCP closes. The collision meshes of the pinch gripper (openarm_description at the
# pins) reach 0.0305 m below the TCP with the hand body (ee_base_link, 0.0319 m below the hand axis, which is 0.0014 m
# above the TCP) and 0.0275 m with the fingers, and the hand body reaches 0.11 m along the fingers, over the tote's
# front wall when the TCP is on the item (the item stands 0.035 m behind that wall). At 0.03 m the hand body cleared
# the wall top by 0.0145 m level; a tilt inside the approach tolerance and the 0.009 m path deviation closed that, and
# a finger caught the wall (diagnostics run of d139f76: 8.8 N, item pushed 45 mm). The fingers still cover the item
# from 0.0225 m above its centre to its top (0.07 m above it).
GRASP_ABOVE_ITEM_CENTRE_M = 0.05
HAND_BODY_BELOW_TCP_M = 0.0305
FINGER_HALF_HEIGHT_M = 0.0275
HAND_BODY_REACH_ALONG_FINGERS_M = 0.11
CARTESIAN_STEP_M = 0.005
CARTESIAN_MIN_FRACTION = 0.999


def quaternion_matrix(q):
    """Rotation matrix (rows) of a unit quaternion (x, y, z, w)."""
    x, y, z, w = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)))


def _apply(R, v):
    return tuple(sum(R[i][k] * v[k] for k in range(3)) for i in range(3))


def grasp_line(tcp_point, tcp_offset, hand_quaternion, standoff=REACH_STANDOFF_M):
    """The straight reach of the hand with orientation `hand_quaternion` (the one the approach ended with): the hand base
    position that puts the TCP (`tcp_offset` from the hand base, in the hand frame) on `tcp_point`, and the position
    `standoff` back along the fingers (-z of the hand) from it. -> (pre_grasp, grasp) hand base positions."""
    R = quaternion_matrix(hand_quaternion)
    offset = _apply(R, tcp_offset)
    grasp = tuple(tcp_point[i] - offset[i] for i in range(3))
    fingers = _apply(R, (0.0, 0.0, -1.0))
    return tuple(grasp[i] - standoff * fingers[i] for i in range(3)), grasp


def straight_lift(hand_xyz, hand_quaternion, rise):
    """The hand base goal that raises the TCP and the held item by `rise` straight up: the hand translated along z of
    base_footprint with the orientation it has on the item. A translation moves every point of the hand by the same vector."""
    return (hand_xyz[0], hand_xyz[1], hand_xyz[2] + rise), tuple(hand_quaternion)


def worst_tcp_rise(rise, position_tolerance, angle_tolerance, tcp_distance):
    """The least the TCP can rise when MoveIt ends anywhere in the goal region of a lift goal on the hand base: the hand
    base `position_tolerance` low, and the hand turned by up to sqrt(3) * angle_tolerance (an absolute tolerance on each of
    three axes), which moves a point `tcp_distance` from the hand base by up to 2 * tcp_distance * sin(angle / 2)."""
    angle = math.sqrt(3.0) * angle_tolerance
    return rise - position_tolerance - 2.0 * tcp_distance * math.sin(angle / 2.0)
