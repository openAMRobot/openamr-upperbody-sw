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
# With 0.12 rad (up to 0.21 rad combined) and REACH_CLEARANCE_M the lowest point clears the wall (test_arm_frames.py).
APPROACH_ANGLE_TOLERANCE_RAD = 0.12
# About the vertical the approach is tighter: the reach keeps the orientation the approach ended with, and a hand turned
# about the vertical against the item's face closes on the item's edges. In the Baseline 1 missions of 17 Nov the base
# stood up to 0.067 m and 0.08 rad off the station, the approach ended 0.025 to 0.075 rad off the face, and the fingers
# stopped at 0.069 to 0.107 rad: the grip torque, kp times that angle, fell to 0.18 to 0.33 N m and the normal force to
# 0.58 to 0.74 N per finger, at or below the 0.736 N the item needs; 4 of 7 lifted nothing. In the fixed-pose runs the
# approach ended 0.012 to 0.025 rad off, the fingers stopped at 0.137 to 0.158 rad and held with 1.95 to 2.35 N.
APPROACH_YAW_TOLERANCE_RAD = 0.02


def approach_tolerances(quaternion, tolerance=APPROACH_ANGLE_TOLERANCE_RAD, yaw_tolerance=APPROACH_YAW_TOLERANCE_RAD):
    """Per-axis orientation tolerances (x, y, z of the hand frame at `quaternion`, as MoveIt's OrientationConstraint reads
    them): `yaw_tolerance` about the hand axis nearest to the vertical of base_footprint, `tolerance` about the others."""
    R = quaternion_matrix(quaternion)
    vertical = max(range(3), key=lambda i: abs(R[2][i]))
    return tuple(yaw_tolerance if i == vertical else tolerance for i in range(3))
# How the TCP gets onto the item. The collision meshes of the pinch gripper (openarm_description at the pins) reach
# 0.0305 m below the TCP with the hand body (ee_base_link, 0.0319 m below the hand axis, which is 0.0014 m above the
# TCP), 0.0464 m above it, and 0.11 m along the fingers, so the hand body passes over the tote's front wall (the item
# stands 0.035 m behind it). Closing 0.03 m above the item centre, the level hand body clears the wall top by 0.0145 m;
# a tilt inside the approach tolerance and the 0.009 m path deviation closed that once in five (diagnostics run of
# d139f76: a finger on the wall at 8.8 N, item pushed 45 mm). Closing higher does not fit: at 0.05 m the hand body,
# lifted 0.08 m, reaches the work shelf board above (the lift's Cartesian fraction fell to 0.71 to 0.82 on 696458d).
# So the reach runs REACH_CLEARANCE_M higher along the fingers and the hand comes down onto the grasp height last,
# with the open fingers already on both sides of the item.
GRASP_ABOVE_ITEM_CENTRE_M = 0.03
REACH_CLEARANCE_M = 0.02
HAND_BODY_BELOW_TCP_M = 0.0305
HAND_BODY_ABOVE_TCP_M = 0.0464
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


# How deep the item sits in the fingers. The fingers are a position servo to closed (kp_hand 5, MIT kp 4.884 on the
# emulator), so the grip torque is kp times the angle at which they meet the item, and that angle depends on where along
# the fingers the item is. With the item centre on the TCP (0.138 m, near the fingertips) they met it at 0.065 to 0.12 rad
# in the Baseline 1 missions: 0.18 to 0.41 N m, 0.58 to 2.6 N per finger, and 3 of 6 grasps lifted nothing or less than
# 40 mm. With it 0.008 to 0.013 m deeper (fixed-pose runs, by tracking) they met it at 0.137 to 0.158 rad and 10 of 10
# lifted 0.076 to 0.080 m. The palm (ee_base_link, 0.1106 m along the fingers) leaves the item's front face room for
# 0.138 - 0.1106 - 0.0175 = 0.0099 m.
ITEM_DEPTH_BEHIND_TCP_M = 0.009
PALM_ALONG_FINGERS_M = 0.1106
TCP_ALONG_FINGERS_M = 0.138


def tcp_for_item(item_point, hand_quaternion, depth=ITEM_DEPTH_BEHIND_TCP_M):
    """Where the TCP goes so that the item centre `item_point` sits `depth` behind it along the fingers (toward the palm)."""
    fingers = _apply(quaternion_matrix(hand_quaternion), (0.0, 0.0, -1.0))
    return tuple(item_point[i] + depth * fingers[i] for i in range(3))
