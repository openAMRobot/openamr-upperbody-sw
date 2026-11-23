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

# PROPOSED (23 Nov 2026, upperbody #134), awaiting the code owner. The approach pose has many joint solutions on a
# 7-joint arm, and a pose goal lets MoveIt end in any of them. On 22 and 23 Nov 2026 the outliers failed the next step:
# joint 5 on its limit (reach fraction 0.083), a near-straight arm (0.175), the palm on the item (1.35 N, cap 1.0 N).
# So the approach pose is solved once by inverse kinematics from this seed and sent as a joint goal. The seed is the
# commanded end point of the approach of strict run 45069865483 leg 2 (PASS, no palm contact). A solution is used only
# when IK succeeded and no joint is further than APPROACH_SEED_MAX_TRAVEL_RAD from the seed; otherwise the pose goal
# of before is sent and the log says so. No tolerance, limit or contact cap changes.
APPROACH_SEED = {"left": (-0.5428, -0.32574, 0.30464, 1.68953, -0.54288, -0.44611, 0.36682)}
APPROACH_SEED_MAX_TRAVEL_RAD = 0.6
IK_SUCCESS = 1      # moveit_msgs/MoveItErrorCodes.SUCCESS


def seeded_approach(side, names, positions, error_code, seeds=None, max_travel=APPROACH_SEED_MAX_TRAVEL_RAD):
    """-> ({joint: position} for openarm_<side>_joint1..7, "") when the IK answer may replace the approach pose goal,
    or (None, reason). `names` and `positions` are the joint state of the IK answer, `error_code` its MoveItErrorCodes."""
    seed = (APPROACH_SEED if seeds is None else seeds).get(side)
    if seed is None:
        return None, f"no approach seed for the {side} arm"
    if error_code != IK_SUCCESS:
        return None, f"inverse kinematics from the seed ended with error code {error_code}"
    found = dict(zip(names, positions))
    joints = {}
    for index, value in enumerate(seed):
        name = f"openarm_{side}_joint{index + 1}"
        if name not in found:
            return None, f"the IK answer has no {name}"
        if abs(found[name] - value) > max_travel:
            return None, f"{name} of the IK answer is {abs(found[name] - value):.3f} rad from the seed, above {max_travel}"
        joints[name] = float(found[name])
    return joints, ""


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


# The descent onto the grasp height sweeps the palm (ee_base_link) down past the top edge of the item's face on the robot
# side, and the item is out of the planning scene for the reach (scenarios.grasp). With the item ITEM_DEPTH_BEHIND_TCP_M
# into the fingers the palm's collision mesh is planned 1.5 mm (run 44476577635) and 4.4 mm (run 44470870769, leg 1) from
# that face at the end point; the executed hand leaves its plan by up to REACH_PATH_DEVIATION_M (diagnostics run of d139f76).
# In run 44476577635 and its grasp-only reproduction from the logged approach state the palm caught the item's top edge on
# the way down (contact ee_base_link with the item, about 1.5 N, 0.35 mm deep) and held joint 1 0.016 rad short of its end
# point for about 1 s with zero velocity: grasp_reach_servo_torque_max 1.018 N m at the 1 s reading. So the hand comes down
# DESCENT_STANDOFF_M farther back along the fingers and the reach ends with a straight move of that length along the fingers
# onto the unchanged end point: the item sits as deep in the fingers as before, and the palm closes on the face head-on.
# Coming down that far back puts the hand body over the tote's front wall at the grasp height, where MoveIt sees the tote
# as one unpadded box (planning_scene.py). At the end point the hand is planned 5.7 to 16.8 mm over the wall (15
# diagnostics runs of 323a2ce, 87d0f7a and e7a251c), and no run of a head that ended the descent there touched the wall
# (0 of 25). Stopping the descent at the grasp height 9 mm back left 7.1 mm (run 5 of 44609269748) and the hand touched the
# wall in 2 of 5 runs (ee_base_link 7.63 N, ee_link1 4.94 N), at the bottom of the descent (scratch run 44645357800). So the
# descent stops DESCENT_RISE_M above the grasp height: the end state, which MoveIt has checked against the tote box, raised
# by the path deviation. The last move is the straight line from there onto the end point, DESCENT_STANDOFF_M along the
# fingers and DESCENT_RISE_M down (test_arm_frames.py).
REACH_PATH_DEVIATION_M = 0.009
DESCENT_STANDOFF_M = REACH_PATH_DEVIATION_M
DESCENT_RISE_M = REACH_PATH_DEVIATION_M


def reach_waypoints(tcp_point, tcp_offset, hand_quaternion, standoff=None, rise=None):
    """The hand base waypoints of scn04's reach with the orientation `hand_quaternion` held: REACH_STANDOFF_M back along
    the fingers and REACH_CLEARANCE_M up, in along the fingers to `standoff` short of the end, down to `rise` above the
    grasp height, and on a straight line onto the end point (the TCP on `tcp_point`). A standoff of 0 is the reach without
    the last move: [pre_grasp, above, grasp]. A rise of 0 comes down onto the grasp height and moves in along the fingers."""
    standoff = DESCENT_STANDOFF_M if standoff is None else standoff
    rise = DESCENT_RISE_M if rise is None else rise
    raised = (tcp_point[0], tcp_point[1], tcp_point[2] + REACH_CLEARANCE_M)
    pre_grasp, above = grasp_line(raised, tcp_offset, hand_quaternion)
    _, grasp = grasp_line(tcp_point, tcp_offset, hand_quaternion)
    if standoff <= 0.0:
        return [pre_grasp, above, grasp]
    fingers = _apply(quaternion_matrix(hand_quaternion), (0.0, 0.0, -1.0))
    back = tuple(standoff * f for f in fingers)
    down = (grasp[0] - back[0], grasp[1] - back[1], grasp[2] - back[2] + rise)
    return [pre_grasp, tuple(above[i] - back[i] for i in range(3)), down, grasp]


LAST_MOVE_TOLERANCE_M = 0.001


def in_last_move(hand_xyz, waypoints, tolerance=LAST_MOVE_TOLERANCE_M):
    """True once the hand base is on the last move of the reach: no farther from the end point than the start of the
    last move is (waypoints[-2], 'down' of reach_waypoints), plus `tolerance`. Every earlier point of the reach is
    farther from the end point (the descent comes down onto 'down' from REACH_CLEARANCE_M above), so the first time
    this holds marks the start of the last move. The executed hand leaves its plan by up to REACH_PATH_DEVIATION_M, so
    the mark is that close and no closer. Used for the strict contact window grasp_reach_last_move
    (config/baseline1_gates.json)."""
    if len(waypoints) < 2:
        return False
    end, start = waypoints[-1], waypoints[-2]
    return math.dist(hand_xyz, end) <= math.dist(start, end) + tolerance


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
ITEM_DEPTH_BEHIND_TCP_M = 0.006      # PROPOSED (#134 B): 3.9 mm to the palm instead of 0.9 mm; 26 legs, 0 palm contacts, 0 slips
PALM_ALONG_FINGERS_M = 0.1106
TCP_ALONG_FINGERS_M = 0.138


def tcp_for_item(item_point, hand_quaternion, depth=ITEM_DEPTH_BEHIND_TCP_M):
    """Where the TCP goes so that the item centre `item_point` sits `depth` behind it along the fingers (toward the palm)."""
    fingers = _apply(quaternion_matrix(hand_quaternion), (0.0, 0.0, -1.0))
    return tuple(item_point[i] + depth * fingers[i] for i in range(3))


# The return to park starts with the fingers out of the shelf. From a reach over a shelf level the fingertips are about
# 0.11 m past the front edge, and a joint-space plan to park lowers the hand before it has left the shelf: in arm
# diagnostics run 5 of #99 (job 136508288405) MoveIt refused all three plans of the left return (-2, INVALID_MOTION_PLAN),
# each with a finger (ee_link1, ee_link1, ee_link2) through work_shelf_board_0 at a state in the middle of the path. So the
# hand first moves straight out along the shelf's outward normal, orientation held, until the fingertips are
# RETRACT_CLEARANCE_M in front of the front plane; then the park plan starts from a state with nothing of the hand over a
# board. FINGERTIP_DEPTH_M is the farthest point of the closed fingers along -z of the hand base at the openarm_description
# pin 14ff67b: finger joint origin z -0.068 plus the finger collision mesh (finger_inner.stl, finger_outer.stl) to z -0.102.
# REACH_STANDOFF_M (scn04's withdraw, 0.10 m) leaves the tips about 9 mm over the board from the level 0 reach.
FINGERTIP_DEPTH_M = 0.170
RETRACT_CLEARANCE_M = 0.03      # working value (Proposed): covers a hand turned by the 0.3 rad goal tolerance (0.0275 m half width)


def retract_out_of_shelf(hand_xyz, hand_quaternion, front_point, outward, depth=FINGERTIP_DEPTH_M, clearance=RETRACT_CLEARANCE_M):
    """The hand base goal that puts the fingertips (`depth` along -z of the hand) `clearance` in front of the shelf front
    plane (through `front_point`, normal `outward` toward the robot, both in the floor plane of the hand's frame), reached
    by a translation along `outward` with the orientation held. None when the fingertips are already that far out."""
    norm = math.hypot(outward[0], outward[1])
    n = (outward[0] / norm, outward[1] / norm)
    fingers = _apply(quaternion_matrix(hand_quaternion), (0.0, 0.0, -1.0))
    tip = tuple(hand_xyz[i] + depth * fingers[i] for i in range(3))
    distance = clearance - ((tip[0] - front_point[0]) * n[0] + (tip[1] - front_point[1]) * n[1])
    if distance <= 0.0:
        return None
    return (hand_xyz[0] + distance * n[0], hand_xyz[1] + distance * n[1], hand_xyz[2])


# A joint command counts as executed only when the joints were MEASURED to move, never on the controller's status alone:
# joint_trajectory_controller without goal tolerances (arm_controllers.yaml has none) reports success when the trajectory time has
# passed, whether the joint moved or not (docs/build1_lift_and_base_diagnosis.md, the lift case). Half the commanded travel separates
# "did not move" from "moved and is still settling"; how close the joint came is each scenario's own check.
MIN_TRAVEL_FRACTION = 0.5
MIN_COMMANDED_TRAVEL_RAD = 0.01     # rad. Joints commanded to stay (reading noise at the activation pose is about 1e-3) are not judged


def unmoved_joints(start, end, target, min_fraction=MIN_TRAVEL_FRACTION, min_commanded=MIN_COMMANDED_TRAVEL_RAD):
    """{joint: (commanded travel, measured travel toward the target)} for every joint commanded to move by more than min_commanded
    whose measured travel toward the target is below min_fraction of the commanded travel. Empty when every commanded joint moved."""
    short = {}
    for name, goal in target.items():
        commanded = goal - start[name]
        if abs(commanded) <= min_commanded:
            continue
        toward = (end[name] - start[name]) * (1.0 if commanded > 0 else -1.0)
        if toward < min_fraction * abs(commanded):
            short[name] = (abs(commanded), toward)
    return short
