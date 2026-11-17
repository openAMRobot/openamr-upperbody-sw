# SPDX-License-Identifier: MIT
"""The Build 1 scenarios. Each function takes the rig and its log and drives the running stack.

Normal cases check numbers against limits. Fault cases record what the stack does today next to the simulation-only
reference of config/expected_reactions.yaml: expected behaviour, not certified safety behaviour.
Emulator limits that a result depends on are written into the log of that scenario.

Simulation only. Nothing here is hardware, physical acceptance or safety evidence.
"""
import math
import os
import threading
import time

import yaml
from geometry_msgs.msg import Twist

from openamrobot_build1_scene import faults
from openamrobot_build1_scene.arm_frames import (SIDE_GRASP, TCP_FRAME, face_yaw, hand_for_tcp_at, rotate_z, side_grasp,
                                                  side_grasp_offset)
from openamrobot_build1_scene.rig import (Blocked, LIFT_MIN_RTF, LIFT_REFUSED, LIFT_STOP_M, ScenarioFailed, angle_diff)

LIMIT_ZLAC = ("The ZLAC8015D emulator follows Manual 1.03 and CANopen Routine 1.08 as filed in platform-hw. "
              "The V4.2 package is still missing.")
LIMIT_BMS = "The Daly BMS emulator follows a 2019 protocol copy. The protocol is not confirmed for the delivered unit."
LIMIT_ARM = "The OpenArm motor emulator has an assumed power-up state, assumed thresholds and assumed latency."
DRIVE_SPEED = 0.2            # m/s. The Nav2 commissioning limit of openamr-platform-sw nav2_params.yaml


def limits(rig, log, *texts):
    """Write the emulator limits a result depends on into its log. Without devices no emulator is in the loop."""
    if rig.devices:
        for text in texts:
            log.event(text)
    else:
        log.event("No device emulator in this run profile.")


def reference():
    from ament_index_python.packages import get_package_share_directory
    path = os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "expected_reactions.yaml")
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)["reactions"]


def _duration(seconds):
    from builtin_interfaces.msg import Duration
    return Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))


def _drive_goal(distance, speed=DRIVE_SPEED, allowance=60.0):
    from nav2_msgs.action import DriveOnHeading
    goal = DriveOnHeading.Goal()
    goal.target.x = float(distance)
    goal.speed = float(speed)
    goal.time_allowance = _duration(allowance)
    return goal


def _send_drive(rig, distance, allowance=60.0):
    handle = rig.send_goal("/drive_on_heading", "nav2_msgs.action", "DriveOnHeading", _drive_goal(distance, allowance=allowance))
    if handle is None:
        raise ScenarioFailed("Nav2 drive_on_heading did not accept the goal")
    return handle


def _distance(a, b):
    return math.hypot(b[0] - a[0], b[1] - a[1])


class CmdVel:
    """A plain /cmd_vel source for scenarios that run without Nav2. With Nav2 off nothing else writes /cmd_vel."""

    def __init__(self, rig):
        self.publisher = rig.node.create_publisher(Twist, "/cmd_vel", 10)
        self.command = (0.0, 0.0)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def _loop(self):
        while self.running:
            message = Twist()
            message.linear.x, message.angular.z = self.command
            self.publisher.publish(message)
            time.sleep(0.05)

    def stop(self):
        self.command = (0.0, 0.0)
        time.sleep(0.3)
        self.running = False


# ---------------------------------------------------------------- 1. drive 2 m and turn 90 degrees
def drive(rig, log):
    limits(rig, log, LIMIT_ZLAC)
    truth0, odom0 = rig.truth(), rig.odom()
    status, _ = rig.wait_result(_send_drive(rig, 2.0), 90.0, "drive 2 m")
    log.require(rig.succeeded(status), f"drive_on_heading 2 m ended with status {status}")
    log.quantity("drive_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", note="until below 0.01 m/s and 0.02 rad/s for 0.5 s")
    truth1, odom1 = rig.truth(), rig.odom()
    truth_distance, odom_distance = _distance(truth0, truth1), _distance(odom0, odom1)
    log.quantity("drive_ground_truth_distance", truth_distance, "m", low=1.85, high=2.15)
    log.quantity("drive_odometry_distance", odom_distance, "m")
    log.quantity("drive_odometry_minus_ground_truth", odom_distance - truth_distance, "m", low=-0.10, high=0.10)
    log.quantity("drive_ground_truth_heading_change", math.degrees(angle_diff(truth1[2], truth0[2])), "deg", low=-5.0, high=5.0)

    from nav2_msgs.action import Spin
    goal = Spin.Goal()
    goal.target_yaw = math.pi / 2.0
    goal.time_allowance = _duration(60.0)
    handle = rig.send_goal("/spin", "nav2_msgs.action", "Spin", goal)
    if handle is None:
        raise ScenarioFailed("Nav2 spin did not accept the goal")
    status, _ = rig.wait_result(handle, 90.0, "turn 90 degrees")
    log.require(rig.succeeded(status), f"spin 90 degrees ended with status {status}")
    log.quantity("turn_settle_time", rig.wait_stopped(10.0), "s of Gazebo time")
    truth2, odom2 = rig.truth(), rig.odom()
    truth_turn = math.degrees(angle_diff(truth2[2], truth1[2]))
    odom_turn = math.degrees(angle_diff(odom2[2], odom1[2]))
    log.quantity("turn_ground_truth_angle", truth_turn, "deg", low=78.0, high=102.0)
    log.quantity("turn_odometry_angle", odom_turn, "deg")
    log.quantity("turn_odometry_minus_ground_truth", odom_turn - truth_turn, "deg", low=-12.0, high=12.0,
                 note="guard value, estimated. Measured 9.4 deg in the fast profile: the wheels slip in a fast turn on the spot")
    log.quantity("turn_ground_truth_position_shift", _distance(truth1, truth2), "m", low=0.0, high=0.10)


# ---------------------------------------------------------------- 2. lift at three heights, with and without permit
def _lift_refused(rig, log, stop, label):
    before = rig.lift_truth()
    refusals = rig.get("/sil/lift/emulator_state")["permit_refusals"]
    rig.lift_command(stop)
    refused = rig.wait(lambda: rig.get("/sil/lift/status")["cmd_result"] == LIFT_REFUSED
                       and rig.get("/sil/lift/emulator_state")["permit_refusals"] > refusals, 3.0, "refusal", step=0.02)
    rig.sleep_sim(2.0)
    log.require(refused, f"{label}: the command without permit was not refused")
    log.quantity(f"{label}_motion_without_permit", abs(rig.lift_truth() - before), "m", low=0.0, high=0.003)


def lift_permit(rig, log):
    log.event("Permit COB-ID, period, timeout and refusal code are SIL test values of openamrobot_sil, not decisions.")
    emulator = "/sil/lift/emulator_state"
    log.require(rig.get(emulator)["permit_gate"], "the permit gate of the CTL-017 emulator is off")
    for stop in (1, 2, 0):
        label = f"lift_stop_{stop}"
        rig.lift_permit(False)
        rig.wait(lambda: not rig.get(emulator)["permit_fresh"], 3.0, "permit withdrawn")
        _lift_refused(rig, log, stop, label)
        rig.lift_permit(True)
        log.require(rig.wait(lambda: rig.get(emulator)["permit_fresh"], 3.0, "permit fresh"), f"{label}: the permit did not become fresh")
        rig.sleep_sim(1.0)
        log.quantity(f"{label}_motion_after_permit_without_new_command", rig.get(emulator)["drive_on"], "bool", low=False, high=False,
                     note="a refused command is not replayed")
        start = rig.sim_time()
        arrived = rig.lift_go(stop, 45.0)
        log.require(arrived, f"{label}: the lift did not reach the stop with a permit")
        log.quantity(f"{label}_travel_time", rig.sim_time() - start, "s of Gazebo time")
        rig.sleep_sim(0.5)
        log.quantity(f"{label}_gazebo_position", rig.lift_truth(), "m", low=LIFT_STOP_M[stop] - 0.006, high=LIFT_STOP_M[stop] + 0.006)
        log.quantity(f"{label}_joint_state_position", rig.joint("lift_joint"), "m",
                     low=LIFT_STOP_M[stop] - 0.008, high=LIFT_STOP_M[stop] + 0.008, note="/joint_states, from CTL-017 over CAN3")
        log.quantity(f"{label}_fault_bits", rig.get(emulator)["faults"], "bits", low=0, high=0)
    rig.lift_permit(False)


# ---------------------------------------------------------------- 3. both arms reach a shelf pose at two lift heights
def _position_goal(group, link, xyz, tolerance, velocity_scale, plan_only=False):
    from geometry_msgs.msg import Pose
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import BoundingVolume, Constraints, PositionConstraint
    from shape_msgs.msg import SolidPrimitive
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = group
    request.num_planning_attempts = 5
    request.allowed_planning_time = 15.0
    request.max_velocity_scaling_factor = velocity_scale
    request.max_acceleration_scaling_factor = velocity_scale
    constraint = PositionConstraint(link_name=link, weight=1.0)
    constraint.header.frame_id = "base_footprint"
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = xyz
    pose.orientation.w = 1.0
    constraint.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[tolerance])],
                                                  primitive_poses=[pose])
    request.goal_constraints = [Constraints(position_constraints=[constraint])]
    goal.planning_options.plan_only = plan_only
    goal.planning_options.replan = True
    goal.planning_options.replan_attempts = 3
    return goal


def _joint_goal(group, joints, velocity_scale):
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint
    goal = MoveGroup.Goal()
    request = goal.request
    request.group_name = group
    request.num_planning_attempts = 5
    request.allowed_planning_time = 15.0
    request.max_velocity_scaling_factor = velocity_scale
    request.max_acceleration_scaling_factor = velocity_scale
    request.goal_constraints = [Constraints(joint_constraints=[
        JointConstraint(joint_name=name, position=float(value), tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
        for name, value in joints.items()])]
    return goal


def _in_base(rig, world_point):
    """A scene point in base_footprint, from the ground truth base pose (the scenario's reference for the target)."""
    x, y, yaw = rig.truth()
    dx, dy = world_point[0] - x, world_point[1] - y
    c, s = math.cos(-yaw), math.sin(-yaw)
    return c * dx - s * dy, s * dx + c * dy, world_point[2]


def _move_group(rig, log, goal, label, sim_s=120.0, required=True, attempts=1):
    """Plan and execute one MoveIt goal. `required` False records the outcome without a limit (a return move)."""
    code, result = None, None
    for _ in range(attempts):
        handle = rig.send_goal("/move_action", "moveit_msgs.action", "MoveGroup", goal, server_wall_s=300.0)
        if handle is None:
            continue
        _status, result = rig.wait_result(handle, sim_s, label)
        code = result.error_code.val if result is not None else None
        if code == 1:
            break
    plans = getattr(rig, "plans", None)       # set by the diagnostics recorder only: the plan MoveIt executed
    if plans is not None and code == 1 and result is not None:
        trajectory = result.planned_trajectory.joint_trajectory
        plans.append({"label": label, "joint_names": list(trajectory.joint_names), "t_result": rig.sim_time(),
                      "points": [list(point.positions) for point in trajectory.points],
                      "times": [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 for point in trajectory.points]})
    if required:
        log.quantity(f"{label}_moveit_error_code", code, "MoveItErrorCodes", low=1, high=1, note="1 is SUCCESS")
    else:
        log.quantity(f"{label}_moveit_error_code", code, "MoveItErrorCodes", note="1 is SUCCESS. Recorded, no limit: a return move")
    if result is not None:
        log.quantity(f"{label}_planning_time", result.planning_time, "s")
    if code == 1 and result is not None:
        _tracking(rig, log, label, result.planned_trajectory.joint_trajectory, required)
    return code == 1


# Commanded end point against the driver's reading, 1 s after the controller reports success, weighed by the driver's
# position gain: kp * |error| is the torque the joint's own servo is still pushing with (openarm_ros2 4e837e1 kp,
# the same as upstream control_gains.yaml). 0.5 N m is 7 % of the smallest motor peak (DM4310, 7 N m) and 5 % of the
# joint 1 gravity load at the grasp pose. Without the feed-forward joint 1 alone gives 70 * 0.145 = 10 N m.
DRIVER_KP = (70.0, 70.0, 70.0, 60.0, 10.0, 10.0, 10.0)
TRACKING_LIMIT_NM = 0.5


def _tracking(rig, log, label, trajectory, required):
    """Commanded against actual, per joint (D9 step 3): the last point of the trajectory MoveIt sent to the controller,
    against /joint_states (the driver's reading over CAN with devices) after 1 s of settling."""
    if not trajectory.points:
        return
    rig.sleep_sim(1.0)
    errors = {name: rig.joint(name) - target for name, target in zip(trajectory.joint_names, trajectory.points[-1].positions)}
    worst = max(errors, key=lambda name: abs(errors[name]))
    servo = {name: DRIVER_KP[int(name[-1]) - 1] * abs(error) for name, error in errors.items() if name[-1].isdigit()}
    worst_servo = max(servo, key=servo.get)
    log.event(f"{label} commanded end point {dict(zip(trajectory.joint_names, [round(v, 5) for v in trajectory.points[-1].positions]))}")
    log.event(f"{label} reading minus command {dict((name, round(error, 5)) for name, error in errors.items())}")
    log.quantity(f"{label}_tracking_error_max", abs(errors[worst]), "rad", note=f"largest |/joint_states - last trajectory point|, joint {worst}")
    note = (f"largest driver kp * |reading - command|, joint {worst_servo}. Without the gravity feed-forward 10 N m on "
            "joint 1 (docs/build1_arm_diagnosis.md)")
    if required:
        log.quantity(f"{label}_servo_torque_max", servo[worst_servo], "N m", low=0.0, high=TRACKING_LIMIT_NM, note=note)
    else:
        log.quantity(f"{label}_servo_torque_max", servo[worst_servo], "N m", note=note + ". Recorded, no limit: a return move")


def _belief_vs_physics(rig, joints):
    return max(abs(rig.joint(name, "/joint_states") - rig.physics_joint(name)) for name in joints)


def _require_devices(rig):
    if not rig.devices:
        raise ScenarioFailed("this scenario needs the device emulators: profile firmware_sil or faults")


PARK = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]      # the SRDF pose park: joint 4 off its lower limit


def _mark(rig, label):
    """A time mark for the diagnostics recorder, when one runs. Nothing otherwise."""
    recorder = getattr(rig, "recorder", None)
    if recorder is not None:
        recorder.mark(label)


def park_arms(rig, log):
    """Move both arms from the activation pose to the park pose with the arm controllers, without MoveIt.
    At the activation pose joint 4 sits on its lower limit of 0. With the real driver the joint then reads a little
    below 0 (seen: -0.00019 rad), and MoveIt refuses to plan from a start state outside the limits. Recorded as
    what the stack does today: an operator has to leave the activation pose by a joint command first."""
    for side in ("left", "right"):
        before = rig.joint(f"openarm_{side}_joint4")
        handle = rig.follow_joint_trajectory(side, PARK, 3.0)
        status = rig.wait_result(handle, 30.0, f"{side} arm to park")[0] if handle is not None else None
        log.quantity(f"{side}_joint4_reading_at_activation_pose", before, "rad",
                     note="/joint_states. Below 0 is outside the joint limit: MoveIt cannot plan from there")
        log.require(rig.succeeded(status), f"the {side} arm did not reach the park pose by a joint command, status {status}")
    rig.sleep_sim(1.0)


def arms_reach(rig, log):
    from openamrobot_build1_scene import scene as scene_model
    from ament_index_python.packages import get_package_share_directory
    limits(rig, log, LIMIT_ARM)
    scene = scene_model.load(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml"))
    targets = scene_model.shelf_reach_targets(scene)
    rig.wait(lambda: rig.get("/build1/planning_scene/objects"), 30.0, "collision objects")
    objects = (rig.get("/build1/planning_scene/objects") or "").split(",")
    log.quantity("planning_scene_collision_objects", len([o for o in objects if o]), "objects", low=10, high=None)
    scale = 0.3
    park_arms(rig, log)
    for stop, level in ((0, 0), (2, 1)):
        if abs(rig.lift_truth() - LIFT_STOP_M[stop]) > 0.003 and not rig.lift_move(stop):
            raise ScenarioFailed(f"the lift did not reach stop {stop}")
        rig.sleep_sim(1.0)
        log.quantity(f"L{stop}_lift_gazebo_position", rig.lift_truth(), "m", low=LIFT_STOP_M[stop] - 0.006, high=LIFT_STOP_M[stop] + 0.006)
        for side in ("left", "right"):
            _reach(rig, log, stop, level, side, targets, scale)
        for side in ("left", "right"):
            _back_to_park(rig, log, stop, side, scale)


def _reach(rig, log, stop, level, side, targets, scale):
    """scn03's reach of one arm to the shelf target of `level`, with the tracking and position checks."""
    label = f"L{stop}_{side}"
    link = f"openarm_{side}_ee_base_link"
    target = _in_base(rig, targets[level][side])
    log.quantity(f"{label}_target_height", target[2], "m", note=f"work shelf level {level}, 0.16 m above the board")
    # A pose goal, not a position-only one: with any orientation allowed the planner returned the right arm with
    # joint 1 on its limit and joint 2 at 3.0 rad, over the top (CI, 16 Nov, docs/build1_arm_diagnosis.md).
    ok = _move_group(rig, log, _pose_goal(f"{side}_arm", link, target, SIDE_GRASP, scale, position_tolerance=0.02), label)
    rig.sleep_sim(1.5)
    found = rig.transform("base_footprint", link)
    if found is not None:
        log.event(f"{label} hand base {[round(v, 4) for v in found[0]]} for the target {[round(v, 4) for v in target]} "
                  f"(base_footprint, from the driver's joint states)")
    for name in ("tote", "item", "payload"):
        try:
            log.event(f"{label} {name} at {[round(v, 4) for v in rig.object_position(name)]} (world, ground truth)")
        except Exception as error:  # noqa: BLE001  a trace line only: a missing pose is reported, not raised
            log.event(f"{label} {name}: no ground truth pose ({error})")
    if ok and found is not None:
        error = math.dist(found[0], target)
        log.quantity(f"{label}_position_error_from_tf", error, "m", low=0.0, high=0.08,
                     note="hand base link against the target, from the driver's joint states. Guard value, "
                          "estimated. The goal region is 0.02 m. Measured 0.029 and 0.067 m with the real driver: "
                          "the arm settles short of the goal under gravity with the driver's gains")
    joints = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    if rig.devices:
        log.quantity(f"{label}_driver_state_minus_physics", _belief_vs_physics(rig, joints), "rad", low=0.0, high=0.05,
                     note="largest joint difference between /joint_states and /sim/joint_states")


def _back_to_park(rig, log, stop, side, scale):
    """scn03's return move: a MoveIt joint goal to park, then the distance of the physical arm from park."""
    park = {f"openarm_{side}_joint{i}": (0.1 if i == 4 else 0.0) for i in range(1, 8)}
    label = f"L{stop}_{side}_back_to_park"
    _mark(rig, f"{label}_start")
    _move_group(rig, log, _joint_goal(f"{side}_arm", park, scale), label, required=False, attempts=3)
    rig.sleep_sim(1.0)
    _mark(rig, f"{label}_end")
    away = max(abs(rig.physics_joint(name) - value) for name, value in park.items())
    log.quantity(f"L{stop}_{side}_distance_from_park", away, "rad", low=0.0, high=0.1,
                 note="the arm must be back at the park pose before the lift moves: an arm under a shelf board blocks it")
    if away > 0.1:
        raise ScenarioFailed(f"the {side} arm did not return to the park pose. The lift is not moved with an arm over the shelf")


# ---------------------------------------------------------------- 4. grasp a tote item
OPEN, CLOSED = 0.7854, 0.0   # left finger joint: open is the far limit, closed is 0 (openamr-upperbody-sw #66)


def _pose_goal(group, link, xyz, quaternion, velocity_scale, position_tolerance=0.01, angle_tolerance=0.3):
    """A pose goal. The angle tolerance is wide on purpose: the exact side-grasp orientation puts wrist joint 6 on its
    limit, and a joint that rests on a limit reads back past it and makes the next plan start from an invalid state."""
    from moveit_msgs.msg import OrientationConstraint
    goal = _position_goal(group, link, xyz, position_tolerance, velocity_scale)
    orientation = OrientationConstraint(link_name=link, weight=1.0)
    orientation.header.frame_id = "base_footprint"
    orientation.orientation.x, orientation.orientation.y, orientation.orientation.z, orientation.orientation.w = quaternion
    orientation.absolute_x_axis_tolerance = orientation.absolute_y_axis_tolerance = orientation.absolute_z_axis_tolerance = angle_tolerance
    goal.request.goal_constraints[0].orientation_constraints = [orientation]
    return goal


def grasp(rig, log):
    limits(rig, log, LIMIT_ARM)
    log.event("Gripper states as corrected in openamr-upperbody-sw #66: open is the far joint limit, closed is 0.")
    side, link = "left", "openarm_left_ee_base_link"
    finger = "openarm_left_finger_joint1"
    item0 = rig.object_position("item")
    log.quantity("item_start_height", item0[2], "m")
    # The fingers point square at the item's face, not along the base heading: a base that stops a few degrees off
    # (the station approach allows 0.05 rad) otherwise sweeps the fingers into the item and the tote wall (Baseline 1
    # run 44100120091: base 0.073 rad off, item pushed 51 mm, not lifted). 0 when the base stands square to the item.
    from openamrobot_build1_scene.rig import yaw_of
    yaw = face_yaw(rig.truth()[2], yaw_of(rig.get("/build1/ground_truth/item/pose").pose.orientation))
    log.quantity("grasp_face_yaw", yaw, "rad", note="the side grasp turned about z of base_footprint to the item's face")
    quaternion = side_grasp(yaw)
    item = _in_base(rig, item0)
    grasp_z = item[2] + 0.03                      # above the tote wall
    # D9 step 2: the item goes to the model's TCP, openarm_left_grasp_frame (0.138 m along the fingers, from the URDF
    # through /tf), not to an estimated depth. The estimate of 0.105 m put the item against the finger roots: it was
    # pushed 52 mm (fast) and 90 mm (firmware_sil) before the fingers closed.
    rig.wait(lambda: rig.transform(link, TCP_FRAME[side]) is not None, 30.0, "the hand base to TCP transform")
    found = rig.transform(link, TCP_FRAME[side])
    if not log.require(found is not None, f"no /tf from {link} to {TCP_FRAME[side]}"):
        return
    log.quantity("tcp_depth_along_fingers", side_grasp_offset(found[0])[0], "m", note=f"{TCP_FRAME[side]} in {link}, from /tf")
    hand = hand_for_tcp_at((item[0], item[1], grasp_z), found[0], yaw)
    back = rotate_z((0.10, 0.0, 0.0), yaw)       # 0.10 m back along the fingers
    approach = (hand[0] - back[0], hand[1] - back[1], hand[2])

    park_arms(rig, log)
    opened = rig.gripper(side, OPEN)
    log.require(opened is not None, "the gripper open command got no result")
    rig.sleep_sim(1.0)
    log.quantity("finger_open_position", rig.physics_joint(finger), "rad", low=OPEN - 0.08, high=OPEN + 0.02)
    log.quantity("finger_mimic_error_open", rig.finger_mimic_error(), "rad", low=0.0, high=0.03,
                 note="finger_joint2 against finger_joint1 in the physics (SW-09)")

    if not _move_group(rig, log, _pose_goal(f"{side}_arm", link, approach, quaternion, 0.3), "grasp_approach"):
        return
    # The item and the tote leave the planning scene for the last 0.10 m: the fingers must pass next to the item.
    from std_msgs.msg import String
    rig.publish("exclude", String(data="item,tote"))
    rig.sleep_sim(2.0)
    if not _move_group(rig, log, _pose_goal(f"{side}_arm", link, hand, quaternion, 0.1), "grasp_reach"):
        return
    rig.sleep_sim(1.0)
    log.quantity("item_pushed_before_closing", math.dist(rig.object_position("item")[:2], item0[:2]), "m", low=0.0, high=0.05)

    closed = rig.gripper(side, CLOSED, sim_s=30.0)
    rig.sleep_sim(1.5)
    _mark(rig, "closed")
    finger_closed = rig.physics_joint(finger)
    log.quantity("finger_position_on_item", finger_closed, "rad", low=0.02, high=OPEN - 0.05,
                 note="above 0 means the fingers stopped on the item and did not close fully")
    log.quantity("finger_driver_reading_on_item", rig.joint(finger), "rad",
                 note="D9 step 4: commanded 0, this is what the driver reads back over CAN; finger_position_on_item is the physics")
    if closed is not None:
        log.quantity("gripper_reported_stalled", bool(closed[1]), "bool")
        log.quantity("gripper_reported_reached_goal", bool(closed[0]), "bool")
    # The held item enters the gravity feed-forward only once the grasp is reported: the gripper action returned and the
    # fingers stopped on the item. Its mass is the scene's, at the model's TCP. Cleared again after the release.
    held = _held_payload(rig, log, side) if closed is not None and finger_closed > 0.02 else False

    lifted = (hand[0], hand[1], hand[2] + 0.08)
    _mark(rig, "lift_start")
    if not _move_group(rig, log, _pose_goal(f"{side}_arm", link, lifted, quaternion, 0.1, position_tolerance=0.015), "grasp_lift"):
        return
    _mark(rig, "lift_end")
    rig.sleep_sim(3.0)
    item1 = rig.object_position("item")
    log.quantity("item_height_gain_after_lift", item1[2] - item0[2], "m", low=0.04, high=0.12)
    rig.sleep_sim(3.0)
    item2 = rig.object_position("item")
    _mark(rig, "hold_end")
    log.quantity("item_height_drift_while_held", item2[2] - item1[2], "m", low=-0.01, high=0.01, note="over 3 s of Gazebo time")
    rig.gripper(side, OPEN)
    _mark(rig, "released")
    if held:
        rig.set_parameters("/gravity_feedforward", {"payload_mass_kg": 0.0})
        log.event("gravity feed-forward payload cleared after the release")
    rig.sleep_sim(2.0)
    log.quantity("item_height_after_release", rig.object_position("item")[2] - item0[2], "m", note="recorded, no limit: it falls back")


def _held_payload(rig, log, side):
    """Give the gravity feed-forward the held item: the scene's mass at the TCP of `side`. -> True when it was set.
    The link and the CoM go first: the node checks every parameter on its own, and a mass without a link is refused."""
    if not (rig.get("/build1/run_info") or {}).get("gravity_feedforward"):
        log.event("no gravity feed-forward in this run: the held item is not added")
        return False
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_build1_scene import scene as scene_model
    scene = scene_model.load(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml"))
    mass = scene_model.value(scene["item"]["mass"])
    rig.set_parameters("/gravity_feedforward", {"payload_link": TCP_FRAME[side], "payload_com_xyz": [0.0, 0.0, 0.0],
                                                "payload_mass_kg": mass})
    log.quantity("gravity_payload_mass", mass, "kg", note=f"set after the grasp was reported, at {TCP_FRAME[side]}. Estimated")
    return True


# ---------------------------------------------------------------- 5. bus loss on each bus
def _arm_snapshot(rig, side):
    joints = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
    return ([rig.joint(name, "/arm/joint_states") for name in joints], [rig.physics_joint(name) for name in joints])


def _controller_states(rig, manager="/controller_manager"):
    from controller_manager_msgs.srv import ListControllers
    client = rig.node.create_client(ListControllers, f"{manager}/list_controllers")
    if not client.wait_for_service(timeout_sec=20.0):
        return {}
    future = client.call_async(ListControllers.Request())
    end = time.monotonic() + 20.0
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    return {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}


def bus_loss(rig, log):
    from std_msgs.msg import Bool, String
    ref = reference()
    limits(rig, log, LIMIT_ZLAC, LIMIT_BMS, LIMIT_ARM)
    log.event("Bus loss is injected with the hooks of the emulators and of the host test client. The interfaces stay up.")

    # vcan_drive: the STM32 role goes silent while the base drives.
    expected = ref["drive_bus_loss"]["expected"]
    cmd = CmdVel(rig)
    try:
        cmd.command = (DRIVE_SPEED, 0.0)
        log.require(rig.wait(lambda: rig.truth_speed()[0] > 0.15, 15.0, "base at speed"), "vcan_drive: the base did not reach speed")
        events = rig.get("/sil/drive/emulator_state")["link_loss_events"]
        position = rig.truth()
        rig.publish("can1_silent", Bool(data=True), repeat=1)
        lost = rig.wait(lambda: rig.get("/sil/drive/emulator_state")["link_lost"], 5.0, "link lost", step=0.005)
        log.observed("vcan_drive_emulator_reports_link_lost", lost, "bool", expected["drive_reports_link_lost"], ref["drive_bus_loss"]["source"])
        stop_time = rig.wait_stopped(10.0)
        log.observed("vcan_drive_base_stops", stop_time is not None, "bool", expected["base_stops"], ref["drive_bus_loss"]["source"])
        log.observed("vcan_drive_time_to_stop", stop_time, "s of Gazebo time",
                     note="from the silence to below 0.01 m/s. The 250 ms link-loss time runs on the host clock")
        log.observed("vcan_drive_distance_after_silence", _distance(position, rig.truth()), "m")
        log.observed("vcan_drive_link_loss_events", rig.get("/sil/drive/emulator_state")["link_loss_events"] - events, "events")
        log.observed("vcan_drive_cmd_vel_still_asks_for", DRIVE_SPEED, "m/s", note="nothing upstream learns of the loss")
        rig.publish("can1_silent", Bool(data=False), repeat=1)
        recovered = rig.wait(lambda: rig.get("/sil/drive/status")["configured"] and rig.truth_speed()[0] > 0.15, 15.0, "drive again")
        log.observed("vcan_drive_drives_again_when_the_link_returns", recovered, "bool",
                     note="/cmd_vel never stopped, so the base moves again without a new command")
    finally:
        cmd.stop()
        rig.publish("can1_silent", Bool(data=False), repeat=1)
    rig.wait_stopped(10.0)

    # vcan_bms: the BMS stops answering.
    expected = ref["bms_bus_loss"]["expected"]
    before = rig.get("/sil/bms/status")
    rig.publish("bms_inject", String(data="offline"), repeat=1)
    rig.sleep_sim(3.0)
    after = rig.get("/sil/bms/status")
    log.observed("vcan_bms_poll_timeouts_rise", after["timeouts"] > before["timeouts"], "bool", expected["poll_timeouts_rise"],
                 ref["bms_bus_loss"]["source"])
    log.observed("vcan_bms_answer_age", after["age_s"], "s of host time")
    log.observed("vcan_bms_reported_voltage_while_offline", after.get("pack_voltage_v"), "V",
                 note="the host test client keeps publishing the last value on /sil/bms/battery_state")
    cmd = CmdVel(rig)
    try:
        cmd.command = (DRIVE_SPEED, 0.0)
        still_drives = rig.wait(lambda: rig.truth_speed()[0] > 0.15, 10.0, "base drives with the BMS offline")
        log.observed("vcan_bms_base_still_drives", still_drives, "bool", note="no reaction to a silent BMS exists in the stack today")
    finally:
        cmd.stop()
    rig.publish("bms_inject", String(data="online"), repeat=1)
    log.observed("vcan_bms_answers_return", rig.wait(lambda: rig.get("/sil/bms/status")["age_s"] < 0.5, 5.0, "BMS answers"), "bool")
    rig.wait_stopped(10.0)

    # vcan_aux: the permit frames stop while the lift moves.
    expected = ref["aux_bus_loss"]["expected"]
    emulator = "/sil/lift/emulator_state"
    rig.lift_permit(True)
    rig.wait(lambda: rig.get(emulator)["permit_fresh"], 3.0, "permit fresh")
    stops = rig.get(emulator)["permit_stops"]
    sim0, wall0 = rig.sim_time(), time.monotonic()
    rig.lift_command(2)
    moving = rig.wait(lambda: rig.lift_truth() > 0.05, 20.0, "lift under way", step=0.02)
    factor = (rig.sim_time() - sim0) / max(1e-6, time.monotonic() - wall0)
    if not moving and rig.get(emulator)["faults"] and factor < LIFT_MIN_RTF:
        raise Blocked(f"vcan_aux: the CTL-017 model faulted at real-time factor {factor:.2f}, below its Hall rate rule")
    log.require(moving, "vcan_aux: the lift did not start")
    rig.publish("permit_silent", Bool(data=True), repeat=1)
    halted = rig.wait(lambda: not rig.get(emulator)["drive_on"], 5.0, "lift halted", step=0.005)
    rig.sleep_sim(1.5)
    position = rig.lift_truth()
    log.observed("vcan_aux_lift_halts", halted, "bool", expected["lift_halts"], ref["aux_bus_loss"]["source"])
    log.observed("vcan_aux_halt_position", position, "m")
    log.observed("vcan_aux_target_dropped", rig.get(emulator)["target_stop"] == 0xFF, "bool", expected["target_dropped"], ref["aux_bus_loss"]["source"])
    log.observed("vcan_aux_permit_stops", rig.get(emulator)["permit_stops"] - stops, "events")
    rig.publish("permit_silent", Bool(data=False), repeat=1)
    rig.wait(lambda: rig.get(emulator)["permit_fresh"], 3.0, "permit fresh again")
    rig.sleep_sim(2.0)
    log.observed("vcan_aux_moves_again_without_new_command", abs(rig.lift_truth() - position) > 0.003, "bool",
                 expected["moves_again_without_new_command"], ref["aux_bus_loss"]["source"])
    rig.lift_go(0, 45.0)
    rig.lift_permit(False)

    # vcan_arm_l and vcan_arm_r: the emulated motors stop answering.
    expected = ref["arm_bus_loss"]["expected"]
    for side, bus in (("left", "vcan_arm_l"), ("right", "vcan_arm_r")):
        handle = rig.follow_joint_trajectory(side, [0.0, 0.0, 0.0, 0.6, 0.0, 0.0, 0.0], 4.0)
        if handle is not None:
            rig.wait_result(handle, 30.0, f"{side} arm to the test pose")
        rig.sleep_sim(1.0)
        belief0, physics0 = _arm_snapshot(rig, side)
        rig.set_emulator_parameter(side, "fault.can_loss", True)
        rig.sleep_sim(1.0)
        handle = rig.follow_joint_trajectory(side, [0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0], 3.0)
        status = rig.wait_result(handle, 30.0, f"{side} arm command during the loss")[0] if handle is not None else None
        rig.sleep_sim(1.0)
        belief1, physics1 = _arm_snapshot(rig, side)
        states = _controller_states(rig)
        log.observed(f"{bus}_driver_state_change_during_loss", max(abs(a - b) for a, b in zip(belief0, belief1)), "rad",
                     note="/arm/joint_states of that arm")
        log.observed(f"{bus}_driver_state_frozen", max(abs(a - b) for a, b in zip(belief0, belief1)) < 0.01, "bool",
                     expected["driver_state_frozen"], ref["arm_bus_loss"]["source"])
        log.observed(f"{bus}_physics_motion_during_loss", max(abs(a - b) for a, b in zip(physics0, physics1)), "rad",
                     note="the emulator keeps applying its last command: the arm holds or follows nothing new")
        log.observed(f"{bus}_trajectory_goal_status_during_loss", status, "GoalStatus", note="4 is SUCCEEDED, 6 is ABORTED")
        log.observed(f"{bus}_controllers_stay_active", all(states.get(f"{side}_{kind}_controller") == "active" for kind in ("arm", "gripper")),
                     "bool", expected["controllers_stay_active"], ref["arm_bus_loss"]["source"])
        rig.set_emulator_parameter(side, "fault.can_loss", False)
        rig.sleep_sim(3.0)
        belief2, physics2 = _arm_snapshot(rig, side)
        log.observed(f"{bus}_driver_minus_physics_after_return", max(abs(a - b) for a, b in zip(belief2, physics2)), "rad")
        log.observed(f"{bus}_joint4_after_return", physics2[3], "rad", note="1.2 was commanded during the loss")


# ---------------------------------------------------------------- 6. stale sensors
def stale_sensors(rig, log):
    ref = reference()["stale_sensors"]
    limits(rig, log, LIMIT_ZLAC)
    log.require(rig.wait(lambda: rig.count("/scan_filtered") > 3, 30.0, "scan"), "no filtered scan before the fault")
    start_pose = rig.truth()
    handle = _send_drive(rig, 2.5, allowance=120.0)
    try:
        log.require(rig.wait(lambda: rig.truth_speed()[0] > 0.15, 20.0, "base at speed"), "the base did not reach speed")
        scans = rig.count("/scan")
        position, start = rig.truth(), rig.sim_time()
        pids = faults.stale_sensors(True)
        log.event(f"sensor bridge stopped, pids {pids}")
        log.require(bool(pids), "the sensor bridge process was not found")
        stop_time = rig.wait_stopped(15.0)
        log.observed("base_stops", stop_time is not None, "bool", ref["expected"]["base_stops"], ref["source"])
        log.observed("time_to_stop", stop_time, "s of Gazebo time", note="Nav2 collision monitor source_timeout is 1.0 s")
        log.observed("distance_after_sensors_went_stale", _distance(position, rig.truth()), "m")
        log.observed("scan_messages_during_the_fault", rig.count("/scan") - scans, "messages")
        rig.sleep_sim(2.0)
        log.observed("base_speed_2_s_later", rig.truth_speed()[0], "m/s")
        log.observed("fault_window", rig.sim_time() - start, "s of Gazebo time")
    finally:
        faults.stale_sensors(False)
        log.event("sensor bridge continued")
    resumed = rig.wait(lambda: rig.truth_speed()[0] > 0.15, 15.0, "base drives again")
    log.observed("drives_again_when_sensors_return", resumed, "bool", note="the drive goal was still active")
    status, _ = rig.wait_result(handle, 120.0, "drive goal after the fault")
    log.observed("drive_goal_status", status, "GoalStatus", note="4 is SUCCEEDED, 6 is ABORTED")
    rig.wait_stopped(10.0)
    log.observed("total_ground_truth_distance", _distance(start_pose, rig.truth()), "m", note="2.5 m was commanded")


# ---------------------------------------------------------------- 7. command cancellation, stop then resume
def cancel_resume(rig, log):
    limits(rig, log, LIMIT_ZLAC, LIMIT_ARM)
    start = rig.truth()
    handle = _send_drive(rig, 2.0)
    log.require(rig.wait(lambda: _distance(start, rig.truth()) > 0.6, 30.0, "0.6 m driven"), "the base did not drive 0.6 m")
    at_cancel = rig.truth()
    rig.cancel(handle)
    stop_time = rig.wait_stopped(10.0)
    log.quantity("base_time_to_stop_after_cancel", stop_time, "s of Gazebo time", low=0.0, high=4.0)
    log.quantity("base_distance_after_cancel", _distance(at_cancel, rig.truth()), "m", low=0.0, high=0.25)
    status, _ = rig.wait_result(handle, 10.0, "cancelled drive goal")
    log.quantity("base_cancelled_goal_status", status, "GoalStatus", low=5, high=5, note="5 is CANCELED")
    held = rig.truth()
    rig.sleep_sim(2.0)
    log.quantity("base_motion_while_stopped", _distance(rig.truth(), held), "m", low=0.0, high=0.01, note="over 2 s of Gazebo time")
    driven = _distance(start, rig.truth())
    status, _ = rig.wait_result(_send_drive(rig, 2.0 - driven), 90.0, "resumed drive")
    log.require(rig.succeeded(status), f"the resumed drive ended with status {status}")
    rig.wait_stopped(10.0)
    log.quantity("base_total_ground_truth_distance", _distance(start, rig.truth()), "m", low=1.85, high=2.15)

    joint = "openarm_left_joint4"
    handle = rig.follow_joint_trajectory("left", [0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0], 8.0)
    if handle is None:
        raise ScenarioFailed("the left arm controller did not accept the goal")
    log.require(rig.wait(lambda: rig.physics_joint(joint) > 0.4, 30.0, "arm under way"), "the arm did not move")
    rig.cancel(handle)
    rig.sleep_sim(1.5)
    stopped_at = rig.physics_joint(joint)
    log.quantity("arm_joint4_at_stop", stopped_at, "rad", low=0.3, high=1.1)
    log.quantity("arm_joint4_speed_after_cancel", abs(rig.physics_joint(joint, "velocity")), "rad/s", low=0.0, high=0.05)
    rig.sleep_sim(2.0)
    log.quantity("arm_joint4_drift_while_stopped", abs(rig.physics_joint(joint) - stopped_at), "rad", low=0.0, high=0.03)
    status, _ = rig.wait_result(rig.follow_joint_trajectory("left", [0.0, 0.0, 0.0, 1.2, 0.0, 0.0, 0.0], 5.0), 40.0, "resumed arm goal")
    rig.sleep_sim(1.0)
    log.require(rig.succeeded(status), f"the resumed arm goal ended with status {status}")
    log.quantity("arm_joint4_after_resume", rig.physics_joint(joint), "rad", low=1.1, high=1.3)


# ---------------------------------------------------------------- 8. arm motor power cut while holding
def arm_power_cut(rig, log):
    ref = reference()["arm_power_cut"]
    expected = ref["expected"]
    limits(rig, log, LIMIT_ARM)
    side = "left"
    pose = [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]          # the SRDF pose hands_up: the forearm is held against gravity
    status, _ = rig.wait_result(rig.follow_joint_trajectory(side, pose, 5.0), 40.0, "arm to the holding pose")
    log.require(rig.succeeded(status), f"the arm did not reach the holding pose, status {status}")
    rig.sleep_sim(2.0)
    belief0, physics0 = _arm_snapshot(rig, side)
    other0 = _arm_snapshot(rig, "right")[1]
    log.quantity("holding_joint4_physics", physics0[3], "rad", low=1.4, high=1.6)
    log.quantity("holding_drift_before_the_cut", max(abs(a - b) for a, b in zip(physics0, _arm_snapshot(rig, side)[1])), "rad", low=0.0, high=0.02)
    start = rig.sim_time()
    rig.set_emulator_parameter(side, "fault.estop_power_cut", True)
    fell = rig.wait(lambda: abs(rig.physics_joint(f"openarm_{side}_joint4") - physics0[3]) > 0.3, 10.0, "arm falls", step=0.01)
    fall_time = rig.sim_time() - start
    rig.sleep_sim(3.0)
    belief1, physics1 = _arm_snapshot(rig, side)
    states = _controller_states(rig)
    log.observed("arm_falls_in_physics", fell, "bool", expected["arm_falls_in_physics"], ref["source"])
    log.observed("time_until_joint4_moved_0_3_rad", fall_time if fell else None, "s of Gazebo time")
    log.observed("joint4_physics_after_the_cut", physics1[3], "rad", note=f"was {physics0[3]:.3f} rad")
    log.observed("largest_physics_joint_change", max(abs(a - b) for a, b in zip(physics0, physics1)), "rad")
    log.observed("driver_state_change", max(abs(a - b) for a, b in zip(belief0, belief1)), "rad")
    log.observed("driver_state_frozen", max(abs(a - b) for a, b in zip(belief0, belief1)) < 0.01, "bool", expected["driver_state_frozen"], ref["source"])
    log.observed("controllers_stay_active", all(states.get(f"{side}_{kind}_controller") == "active" for kind in ("arm", "gripper")), "bool",
                 expected["controllers_stay_active"], ref["source"])
    log.observed("other_arm_largest_joint_change", max(abs(a - b) for a, b in zip(other0, _arm_snapshot(rig, "right")[1])), "rad")
    rig.set_emulator_parameter(side, "fault.estop_power_cut", False)
    rig.sleep_sim(3.0)
    belief2, physics2 = _arm_snapshot(rig, side)
    back = abs(physics2[3] - physics0[3]) < 0.1
    log.observed("motors_enabled_after_power_returns", back, "bool", expected["motors_enabled_after_power_returns"], ref["source"],
                 note="judged by joint 4 returning to the held pose. Nothing re-enables the motors today")
    log.observed("driver_minus_physics_after_power_returns", max(abs(a - b) for a, b in zip(belief2, physics2)), "rad")


# ---------------------------------------------------------------- 10. physics profile report
class StampCounter:
    """Counts the messages of a joint state topic and the update periods its header stamps skip."""

    def __init__(self, rig, topic, period):
        from rclpy.qos import QoSProfile
        from sensor_msgs.msg import JointState
        self.period, self.count, self.missed, self.last, self.first = period, 0, 0, None, None
        self.subscription = rig.node.create_subscription(JointState, topic, self.on_message, QoSProfile(depth=5000))

    def on_message(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.first is None:
            self.first = stamp
        elif stamp > self.last:
            self.missed += max(0, int(round((stamp - self.last) / self.period)) - 1)
        self.last = stamp
        self.count += 1

    def reset(self):
        self.count, self.missed, self.last, self.first = 0, 0, None, None

    def rate(self):
        span = (self.last - self.first) if self.count > 1 else 0.0
        return (self.count - 1) / span if span > 0 else 0.0


def profile_report(rig, log, window_s=15.0):
    """Real-time factor, controller rates and missed steps of the running profile. A report: the limits only guard
    against a stack that does not run at all, never against a slow machine."""
    info = rig.get("/build1/run_info")
    step_s = info["step_s"]
    log.quantity("physics_step", step_s, "s", kind="configured")
    counters = {}
    if rig.devices:
        log.quantity("emulator_update_rate_configured", info["emulator_rate_hz"], "Hz", kind="configured")
        log.quantity("driver_update_rate_configured", info["driver_rate_hz"], "Hz", kind="configured",
                     note=f"on {info['driver_clock']} time")
        if info.get("sim_state_rate") != "step":
            raise ScenarioFailed("the profile report needs sim_state_rate:=step: /sim/joint_states is its probe of the emulator rate")
        counters["emulator"] = (StampCounter(rig, "/sim/joint_states", 1.0 / info["emulator_rate_hz"]), info["emulator_rate_hz"], 0.4,
                                "/sim/joint_states: the simulator's controller manager, where the arm motor emulators run. "
                                "The broadcaster drops a message when its publisher is busy, so this is a lower bound of the "
                                "update rate and the missed count an upper bound")
        counters["driver"] = (StampCounter(rig, "/arm/joint_states", 1.0 / info["driver_rate_hz"]), info["driver_rate_hz"], 0.3,
                              "/arm/joint_states: the real driver's controller manager. It waits for CAN replies on the host "
                              "clock, so a slow Gazebo shows up here")
    else:
        log.quantity("controller_rate_requested", info["controller_rate_requested_hz"], "Hz", kind="configured")
        log.quantity("controller_rate_expected", info["controller_rate_hz"], "Hz", kind="derived",
                     note="gz_ros2_control updates on the first physics step at or after each period")
        counters["controller"] = (StampCounter(rig, "/arm/joint_states", 1.0 / info["controller_rate_hz"]), info["controller_rate_hz"], 0.9,
                                  "/arm/joint_states: the controller manager inside Gazebo")
    rig.sleep_sim(2.0)
    for counter, _rate, _low, _note in counters.values():
        counter.reset()
    wall0, sim0 = time.monotonic(), rig.sim_time()
    rig.sleep_sim(window_s)
    wall, simulated = time.monotonic() - wall0, rig.sim_time() - sim0
    stats = rig.get("/build1/rtf_stats")
    log.quantity("window", simulated, "s of Gazebo time")
    log.quantity("gazebo_rtf_window", simulated / wall, "1", low=0.01, high=1.2)
    log.quantity("physics_steps_in_window", int(round(simulated / step_s)), "steps")
    log.quantity("wall_time_per_physics_step", 1000.0 * wall / (simulated / step_s), "ms")
    for name, (counter, rate, low, note) in counters.items():
        log.quantity(f"{name}_rate_measured", counter.rate(), "Hz of Gazebo time", low=low * rate, high=1.05 * rate, note=note)
        log.quantity(f"{name}_missed_updates", counter.missed, "updates", note="update periods skipped between header stamps")
        log.quantity(f"{name}_missed_update_share", counter.missed / max(1, counter.missed + counter.count), "1")
    if stats:
        log.quantity("gazebo_rtf_min_since_start", stats["rtf_min"], "1")
        log.quantity("gazebo_rtf_mean_since_start", stats["rtf_mean"], "1")
    firmware = rig.get("/sil/fw/status")
    if firmware:
        log.quantity("renode_status", str({k: firmware[k] for k in sorted(firmware) if "ratio" in k or "time" in k}), "",
                     note="Renode virtual time is a separate clock. openamrobot_sil reports it")
    else:
        log.event("No Renode in this run: there is no Renode time ratio to report.")


# ---------------------------------------------------------------- 9. single writer and bus audit
def single_writer(rig, log):
    """The running system against config/command_map.yaml, and the frame kinds on the five buses."""
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_build1_scene import single_writer_live
    path = os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "command_map.yaml")
    with open(path, encoding="utf-8") as f:
        command_map = yaml.safe_load(f)
    info = rig.get("/build1/run_info")
    components = {name for name in ("nav2", "moveit") if info.get(name)}
    if "follower" in info.get("finger_joint2_owner", ""):
        components.add("follower")
    log.event(f"finger_joint2 owner: {info.get('finger_joint2_owner')}")
    report = single_writer_live.check(rig.node, command_map, info["profile"], components, spinning=True)
    for fact in report["facts"]:
        log.event(fact)
    for problem in report["problems"]:
        log.require(False, problem)
    log.quantity("single_writer_problems", len(report["problems"]), "problems", low=0, high=0)
    log.quantity("single_writer_topics_checked", report["topics_checked"], "topics")
    log.quantity("single_writer_actions_checked", report["actions_checked"], "actions")
    log.quantity("tf_frames_seen", report["frames_seen"], "frames", low=20, high=None)
    log.quantity("finger_mimic_error", rig.finger_mimic_error(), "rad", low=0.0, high=0.03,
                 note="finger_joint2 against finger_joint1 in the physics (SW-09)")
    if rig.devices:
        from openamrobot_build1_scene import bus_audit
        for bus, counts in bus_audit.listen_all(2.0).items():
            fd_bus = bus.startswith("vcan_arm")
            log.quantity(f"{bus}_frames_in_2_s", counts["classic"] + counts["fd"], "frames", low=1, high=None)
            log.quantity(f"{bus}_can_fd_frames", counts["fd"], "frames", low=1 if fd_bus else 0, high=None if fd_bus else 0,
                         note="read as 72-byte canfd_frame" if fd_bus else "the base buses carry classic CAN only")
            log.quantity(f"{bus}_classic_frames", counts["classic"], "frames", low=0 if fd_bus else 1, high=0 if fd_bus else None)
            if fd_bus:
                log.quantity(f"{bus}_frames_with_bit_rate_switch", counts["brs"], "frames",
                             note="the BRS flag is carried by vcan. No bit rate exists on a virtual bus")


# ---------------------------------------------------------------- diagnostics: grasp, lift and the return to park, recorded
# Run by .github/workflows/build1-arm-diagnostics.yml with BUILD1_CONTACT_SENSORS=1. The checks inside are those of
# scn03 and scn04, unchanged; the recorder adds contact forces, finger states, item against TCP and joint torques.
def _diag_start(rig, log):
    from openamrobot_build1_scene import diagnostics as dx
    from openamrobot_build1_scene.diagnostics_live import Recorder
    info = rig.get("/build1/run_info") or {}
    log.quantity("contact_sensor_links", len(info.get("contact_sensors") or []), "links", low=1, high=None,
                 note="gz contact sensors in the model (BUILD1_CONTACT_SENSORS=1)")
    recorder = Recorder(rig, dx.contact_links()).start()
    rig.wait(lambda: recorder.model is not None, 30.0, "/robot_description for the TCP kinematics")
    log.quantity("diag_model_received", recorder.model is not None, "bool", low=True, high=True)
    return recorder


def _diag_return(rig, log, side, label, required=False):
    park = {f"openarm_{side}_joint{i}": value for i, value in enumerate(PARK, 1)}
    _mark(rig, f"{label}_start")
    _move_group(rig, log, _joint_goal(f"{side}_arm", park, 0.3), label, required=required, attempts=3)
    rig.sleep_sim(1.0)
    _mark(rig, f"{label}_end")
    away = max(abs(rig.physics_joint(name) - value) for name, value in park.items())
    log.quantity(f"{label}_distance_from_park", away, "rad", note="recorded, no limit here")


def _diag_report(rig, log, recorder, name, grasped, returns):
    from ament_index_python.packages import get_package_share_directory
    from openamrobot_build1_scene import diagnostics as dx
    from openamrobot_build1_scene import diagnostics_live as live
    from openamrobot_build1_scene import scene as scene_model
    directory = os.environ.get("BUILD1_LOG_DIR", os.path.join(os.getcwd(), "build1_logs"))
    scene = scene_model.load(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml"))
    mass, mu = scene_model.value(scene["item"]["mass"]), scene_model.value(scene["item"]["friction"])
    report = {"contact_messages": dict(sorted(recorder.contact_counts.items())), "samples": len(recorder.samples),
              "marks": recorder.marks}
    if grasped:
        report["grasp"] = live.grasp_numbers(recorder, "left", mass, mu)
    joints = [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)]
    limits = dx.joint_limits(recorder.urdf, joints) if recorder.urdf else {}
    report["returns"] = {label: live.return_numbers(recorder, side, label, limits) for side, label in returns}
    log.data["diagnostics"] = report
    for key, value in sorted(report.get("grasp", {}).items()):
        log.quantity(f"diag_{key}", value, note="diagnostic, recorded, no limit")
    for label, numbers in report["returns"].items():
        for key, value in sorted(numbers.items()):
            log.quantity(f"diag_{label}_{key}", value, note="diagnostic, recorded, no limit")
    log.event(f"trace {recorder.write(directory, name)}")
    log.event(f"bag {recorder.write_bag(directory, name)}")


def diag_grasp(rig, log):
    """scn04's grasp and lift, then both arms to park as the Baseline 1 retreat does, with the recorder on."""
    recorder = _diag_start(rig, log)
    try:
        try:
            grasp(rig, log)
        finally:
            for side in ("left", "right"):
                _diag_return(rig, log, side, f"retreat_{side}_to_park")
    finally:
        recorder.stop()
        _diag_report(rig, log, recorder, "diag_grasp", True, [(s, f"retreat_{s}_to_park") for s in ("left", "right")])


def diag_reach_return(rig, log):
    """scn03 at lift stop 0: both arms reach shelf level 0 and return to park, with the recorder on. No lift move."""
    from openamrobot_build1_scene import scene as scene_model
    from ament_index_python.packages import get_package_share_directory
    recorder = _diag_start(rig, log)
    labels = []
    try:
        scene = scene_model.load(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml"))
        targets = scene_model.shelf_reach_targets(scene)
        rig.wait(lambda: rig.get("/build1/planning_scene/objects"), 30.0, "collision objects")
        park_arms(rig, log)
        log.quantity("L0_lift_gazebo_position", rig.lift_truth(), "m", note="no lift move here: the lift starts at stop 0")
        for side in ("left", "right"):
            _reach(rig, log, 0, 0, side, targets, 0.3)
        for side in ("left", "right"):
            labels.append((side, f"L0_{side}_back_to_park"))
            try:
                _back_to_park(rig, log, 0, side, 0.3)
            except ScenarioFailed as failed:
                log.require(False, str(failed))
    finally:
        recorder.stop()
        _diag_report(rig, log, recorder, "diag_reach_return", False, labels)


BASE_SCENARIOS = ("scn01_drive", "scn05_bus_loss", "scn06_stale_sensors", "scn07_cancel_resume")     # they command the base

SCENARIOS = {
    # name: (function, needs devices, ready arguments)
    "scn01_drive": (drive, False, {"nav2": True, "arms": False}),
    "scn02_lift_permit": (lift_permit, True, {"arms": False}),
    "scn03_arms_reach": (arms_reach, False, {"moveit": True}),
    "scn04_grasp": (grasp, False, {"moveit": True}),
    "scn05_bus_loss": (bus_loss, True, {}),
    "scn06_stale_sensors": (stale_sensors, False, {"nav2": True, "arms": False}),
    "scn07_cancel_resume": (cancel_resume, False, {"nav2": True}),
    "scn08_arm_power_cut": (arm_power_cut, True, {}),
    "scn09_single_writer": (single_writer, False, {"nav2": True, "moveit": True}),
    "scn10_profile_report": (profile_report, False, {}),
    "diag_grasp": (diag_grasp, False, {"moveit": True}),
    "diag_reach_return": (diag_reach_return, False, {"moveit": True}),
}
