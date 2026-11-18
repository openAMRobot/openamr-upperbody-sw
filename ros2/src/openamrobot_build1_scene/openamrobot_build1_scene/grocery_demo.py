# SPDX-License-Identifier: MIT
"""Grocery dark store demonstrator: one whole-robot pick-and-place mission in one launch, phase by phase.

spawn (parking bay) > nav_to_shelf > stop_authorize > lift_to_target > arm_approach > grasp > lift_object >
retreat_with_item > nav_to_packing > place > retreat > return_to_parking.

Reused unchanged from Baseline 1 (baseline1.py, scenarios.py, rig.py): the phase tracker, the failure classes, the
station tree goal, stop_authorize, lift_to_target, the retreat, the scn04 grasp steps (park, open, approach, straight
reach, close, straight lift 0.08 m, hold) and their checks with their limits. This module adds the carry (the product is
not released after the hold), the navigation to the packing bench and back to parking, and the place (lower, open,
release checks, withdraw). Profile sim-oracle: the planning scene and the grasp targets come from the simulator's
ground truth, declared as `pose_source: ground_truth` (read from planning_scene_loader at run time, never assumed).

Simulation evidence only: never hardware, physical acceptance, perception or safety evidence. No velocity publisher
and no joint writer here: the base moves through Nav2 actions, the arms through MoveIt and the arm controllers' actions,
the lift through CTL-017 with a permit, as in Baseline 1.
"""
import math
import os
import threading
import time

from openamrobot_build1_scene import baseline1 as b1
from openamrobot_build1_scene import grocery_phases as phases
from openamrobot_build1_scene import grocery_record as record
from openamrobot_build1_scene import scene as scene_model
from openamrobot_build1_scene.rig import angle_diff

SEED = b1.SEED
SPAWN_STATION, PICK_STATION, PACK_STATION = "origin", "work_shelf", "packing"
NAV_SIM_S = 400.0
SIDE, LINK, FINGER = "left", "openarm_left_ee_base_link", "openarm_left_finger_joint1"
ITEM_RISE_M = 0.04                       # the item rise check of scn04 and Baseline 1 (item_height_gain_after_lift)
ITEM_TCP_GUARD_M = 0.05                  # product centre to TCP while carried. Guard value, estimated
# Odometry against ground truth: scn01's guard is 0.10 m for a 2 m drive; this mission drives about 14 m. Derived: the
# same 0.05 m per metre driven since the spawn (odometry is never reset). Recorded with the distance in the note.
ODOMETRY_GUARD_PER_M = 0.05
PLACE_CLEARANCE_M = 0.02                 # product bottom above the tote floor when the gripper opens. Estimated
PLACE_LOWER_MAX_M = 0.10                 # the place never lowers the hand more than this. Estimated
SETTLED_SPEED = 0.01                     # m/s of the product centre over 1 s of Gazebo time after release. Guard, estimated
# With the product held the arm reaches 0.79 m ahead of the base centre, and the Nav2 footprint follows it. Turning in
# place at the packing station sweeps it through the bench legs (run 44227129411: RotationShimController "collision
# ahead", Nav2 ABORTED 1.56 rad off). So the robot turns APPROACH_M back from the station, facing the bench, and drives
# the last APPROACH_M straight in. Estimated: the legs are 1.27 m from that turning point, the sweep 0.86 m.
APPROACH_M = 0.60


def scene_of(rig):
    """The scene file the launch used (/build1/run_info `scene`), loaded."""
    from ament_index_python.packages import get_package_share_directory
    name = (rig.get("/build1/run_info") or {}).get("scene") or "scene.yaml"
    path = name if os.path.isabs(name) else os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", name)
    return scene_model.load(path), name


def station(scene, name):
    from openamrobot_build1_scene import grocery_scene
    return grocery_scene.station(scene, name)


class Events:
    """Mission events: to the log and on /grocery_demo/events (in the bag); the phase on /grocery_demo/phase."""

    def __init__(self, rig, log):
        from std_msgs.msg import String
        self.rig, self.log, self.String = rig, log, String
        self.events = rig.node.create_publisher(String, record.EVENTS_TOPIC, 10)
        self.phase = rig.node.create_publisher(String, record.PHASE_TOPIC, 10)

    def __call__(self, text):
        self.log.event(text)
        self.events.publish(self.String(data=f"{self.rig.sim_time():.3f} {text}"))

    def phase_started(self, name):
        self.phase.publish(self.String(data=name))


def get_parameter(rig, node, name):
    """One parameter of another node, or None."""
    from rcl_interfaces.srv import GetParameters
    client = rig.node.create_client(GetParameters, f"{node}/get_parameters")
    if not client.wait_for_service(timeout_sec=30.0):
        return None
    future = client.call_async(GetParameters.Request(names=[name]))
    end = time.monotonic() + 30.0
    while not future.done() and time.monotonic() < end:
        time.sleep(0.05)
    if not future.done() or not future.result().values:
        return None
    v = future.result().values[0]
    return v.string_value if v.type == 4 else None


# ---------------------------------------------------------------- phases

def spawn(rig, log, scene, demo):
    sx, sy, syaw = station(scene, SPAWN_STATION)
    x, y, yaw = rig.truth()
    log.quantity("spawn_position_error", math.hypot(x - sx, y - sy), "m", low=0.0, high=0.02,
                 note="ground truth against stations.origin, the parking bay. Guard value of Baseline 1, estimated")
    log.quantity("spawn_yaw_error", abs(angle_diff(yaw, syaw)), "rad", low=0.0, high=0.02, note="guard value of Baseline 1")
    log.quantity("spawn_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    ox, oy, _ = rig.odom()
    log.quantity("spawn_odometry_minus_ground_truth", math.hypot(ox - x, oy - y), "m", low=0.0, high=0.02,
                 note="odom starts at the spawn pose, the world origin. Guard value of Baseline 1")
    if rig.devices:
        log.quantity("spawn_drive_configured", bool(rig.get("/sil/drive/status")["configured"]), "bool", low=True, high=True)
    # The profile: sim-oracle needs the planning scene placed from the ground truth. Read, not assumed.
    source = get_parameter(rig, "/planning_scene_loader", "pose_source")
    demo["pose_source"] = source
    demo["launch_planning_pose_source"] = (rig.get("/build1/run_info") or {}).get("planning_pose_source")
    log.event(f"profile {phases.PROFILE}; planning_scene_loader pose_source: {source}")
    log.quantity("profile_pose_source_ground_truth", source == phases.POSE_SOURCE, "bool", low=True, high=True,
                 note="the sim-oracle profile declares pose_source: ground_truth (docs/integration/baseline1.md)")


def approach_pose(scene, name, distance=APPROACH_M):
    """The pose `distance` back from station `name` along its heading, facing the same way."""
    x, y, yaw = station(scene, name)
    return x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw


def navigate(rig, log, scene, name, demo, events, reach_budget=False, pose=None, checks=True):
    """Nav2 navigate_to_pose to station `name` (or to `pose`) with the station tree of openamr-platform-sw #164, and the
    checks of Baseline 1's nav_to_station. The arm-reach budget (0.04 m, 0.08 rad) applies where the arm must reach a
    target from the stop: the work shelf. Elsewhere the station is held to Nav2's own goal tolerance. checks=False
    records the result of an intermediate goal and checks only that Nav2 succeeded."""
    from nav2_msgs.action import NavigateToPose
    amcl = b1.AmclListener(rig)
    rig.sleep_sim(0.5)
    count0 = amcl.count
    gx, gy, gyaw = pose if pose is not None else station(scene, name)
    goal = NavigateToPose.Goal()
    goal.pose.header.frame_id = "map"
    goal.pose.pose.position.x, goal.pose.pose.position.y = float(gx), float(gy)
    goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(gyaw / 2.0), math.cos(gyaw / 2.0)
    goal.behavior_tree = b1.station_tree()
    demo["behavior_tree"] = goal.behavior_tree or "default (bt_navigator)"
    events(f"nav goal {name} x {gx:.3f} y {gy:.3f} yaw {gyaw:.3f} (map), behavior tree {goal.behavior_tree or 'default'}")
    start, travelled0 = rig.sim_time(), demo["travelled"]
    handle = rig.send_goal("/navigate_to_pose", "nav2_msgs.action", "NavigateToPose", goal)
    if not log.require(handle is not None, "Nav2 navigate_to_pose did not accept the goal"):
        return None
    status = wait_nav(rig, log, handle, name, demo, events)
    if demo.get("odom_frozen"):
        return status
    events(f"nav result {name}: status {status}")
    if not checks:
        log.quantity(f"nav_status_{name}", status, "GoalStatus", low=b1.SUCCEEDED, high=b1.SUCCEEDED,
                     note="an intermediate goal: 4 SUCCEEDED, 6 ABORTED, None: no result in time")
        x, y, yaw = rig.truth()
        log.quantity(f"nav_{name}_position_error", math.hypot(x - gx, y - gy), "m", note="recorded, no limit")
        log.quantity(f"nav_{name}_yaw_error", abs(angle_diff(yaw, gyaw)), "rad", note="recorded, no limit")
        return status
    log.quantity("nav_status", status, "GoalStatus", low=b1.SUCCEEDED, high=b1.SUCCEEDED, note="4 SUCCEEDED, 6 ABORTED, None: no result in time")
    log.quantity("nav_time", rig.sim_time() - start, "s of Gazebo time")
    log.quantity("nav_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    x, y, yaw = rig.truth()
    ox, oy, oyaw = rig.odom()
    log.quantity("nav_ground_truth_position_error", math.hypot(x - gx, y - gy), "m", low=0.0, high=b1.NAV_XY_TOLERANCE_M,
                 note="against the goal. Nav2 xy_goal_tolerance of openamr-platform-sw nav2_params.yaml")
    log.quantity("nav_ground_truth_yaw_error", abs(angle_diff(yaw, gyaw)), "rad", low=0.0, high=b1.NAV_YAW_TOLERANCE_RAD,
                 note="Nav2 yaw_goal_tolerance of openamr-platform-sw nav2_params.yaml")
    driven = demo["travelled"]
    log.quantity("nav_path_driven", driven - travelled0, "m", note="ground truth, this phase")
    log.quantity("nav_odometry_minus_ground_truth", math.hypot(ox - x, oy - y), "m", low=0.0,
                 high=round(max(0.10, ODOMETRY_GUARD_PER_M * driven), 3),
                 note=f"odometry is never reset: {driven:.2f} m driven since the spawn. Derived from scn01's 0.10 m per 2 m")
    log.quantity("nav_odometry_yaw_minus_ground_truth", abs(angle_diff(oyaw, yaw)), "rad", note="recorded, no limit")
    found = b1.map_pose(rig)
    if found is not None:
        log.quantity("nav_localization_minus_ground_truth", math.hypot(found[0] - x, found[1] - y), "m", note="AMCL against Gazebo")
        log.quantity("nav_localization_yaw_minus_ground_truth", abs(angle_diff(found[2], yaw)), "rad")
    b1.localization_record(rig, log, amcl, (x, y, yaw), count0)
    along = (x - gx) * math.cos(gyaw) + (y - gy) * math.sin(gyaw)
    left = -(x - gx) * math.sin(gyaw) + (y - gy) * math.cos(gyaw)
    log.quantity("nav_station_error_along_heading", along, "m", note="+ is closer to the fixture")
    log.quantity("nav_station_error_left", left, "m")
    if reach_budget:
        log.quantity("station_reach_position_error", math.hypot(x - gx, y - gy), "m", low=0.0, high=b1.REACH_XY_M,
                     note="the arm-reach budget of Baseline 1 (docs/integration/station_approach.md), derived")
        log.quantity("station_reach_yaw_error", abs(angle_diff(yaw, gyaw)), "rad", low=0.0, high=b1.REACH_YAW_RAD,
                     note="the arm-reach budget of Baseline 1, derived")
    if rig.devices:
        log.quantity("nav_host_client_drive_thread_alive", rig.drive_host_alive(), "bool", low=True, high=True)
    return status


class OdomProbe:
    """Diagnostics of the odometry chain (runs 44242589231, both attempts: /odom stood still while the base drove):
    /sil/base/odom of openamrobot_sil's host client (the base_state_adapter's input), counted with its last twist."""

    def __init__(self, rig):
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import JointState
        self.rig, self.count, self.last, self.stamp = rig, 0, None, None
        self.base_joints = 0                 # /base/joint_states: written by base_state_adapter.py on every tick
        rig.node.create_subscription(Odometry, "/sil/base/odom", self._on, 10)
        rig.node.create_subscription(JointState, "/base/joint_states", self._on_joints, 10)
        self.lines = 0

    def _on_joints(self, _msg):
        self.base_joints += 1

    def _on(self, msg):
        self.count += 1
        self.last = (msg.twist.twist.linear.x, msg.twist.twist.angular.z, msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.stamp = time.monotonic()

    def line(self):
        rig = self.rig

        def read(function):
            try:
                return function()
            except Exception:  # noqa: BLE001 - a diagnostic line never fails the mission
                return None
        truth, odom = read(rig.truth), read(rig.odom)
        drive = rig.get("/sil/drive/status") or {}
        emulator = rig.get("/sil/drive/emulator_state") or {}
        wheels = read(lambda: [round(rig.joint(n, "/sil/gz/joint_states", "velocity"), 3) for n in ("left_wheel_joint", "right_wheel_joint")])
        fmt = (lambda v: None if v is None else [round(x, 3) for x in v])  # noqa: E731
        return (f"sim {rig.sim_time():.1f} truth {fmt(truth)} odom {fmt(odom)} /odom count {rig.count('/odom')} age "
                f"{None if rig.age('/odom') is None else round(rig.age('/odom'), 2)} s | /sil/base/odom count {self.count} "
                f"v,w,x,y {fmt(self.last)} age {None if self.stamp is None else round(time.monotonic() - self.stamp, 2)} s | "
                f"drive status samples {drive.get('samples')} v {drive.get('v')} w {drive.get('w')} | emulator "
                f"{str(emulator)[:400]} | "
                f"gz wheel velocity {wheels} | /base/joint_states count {self.base_joints} | rtf {rig.rtf()}" + self._process())

    def _process(self):
        """Every sixth line: the base_state_adapter.py process as ps sees it (alive, state, CPU time)."""
        self.lines += 1
        if self.lines % 6 != 1:
            return ""
        import subprocess
        done = subprocess.run(["ps", "-eo", "pid,stat,etime,time,args"], capture_output=True, text=True, check=False)
        rows = [r.strip()[:120] for r in done.stdout.splitlines() if "base_state_adapter" in r or "stm32_host_client" in r]
        return " | ps: " + " ; ".join(rows)


FROZEN_TRUTH_M, FROZEN_ODOM_M = 0.50, 0.05


def wait_nav(rig, log, handle, name, demo, events):
    """rig.wait_result for a Nav2 goal, in 5 s steps of Gazebo time, each with an odometry diagnostic line. When the base
    has driven FROZEN_TRUTH_M (ground truth) while /odom moved less than FROZEN_ODOM_M, the goal is cancelled: Nav2 then
    drives on a frozen odom frame (runs 44242589231), which only ends against a fixture."""
    probe = demo.setdefault("_probe", OdomProbe(rig))
    future = handle.get_result_async()
    start = rig.sim_time()
    truth0, odom0 = rig.truth(), rig.odom()
    while not future.done() and rig.sim_time() - start < NAV_SIM_S:
        rig.wait(future.done, 5.0, f"navigate to {name}")
        print(f"GROCERY ODOM {name} {probe.line()}", flush=True)
        truth, odom = rig.truth(), rig.odom()
        moved, odom_moved = math.dist(truth[:2], truth0[:2]), math.dist(odom[:2], odom0[:2])
        if moved > FROZEN_TRUTH_M and odom_moved < FROZEN_ODOM_M:
            events(f"odometry frozen: the base drove {moved:.2f} m (ground truth), /odom moved {odom_moved:.3f} m; goal cancelled")
            log.require(False, f"nav_odometry_minus_ground_truth: /odom frozen, {odom_moved:.3f} m while the base drove {moved:.2f} m")
            demo["odom_frozen"] = True
            rig.cancel(handle)
            rig.wait_stopped(10.0)
            return None
    if not future.done():
        return None
    return future.result().status


def _held_payload(rig, log, scene):
    """scenarios._held_payload with this scene's product mass: the gravity feed-forward carries the held product."""
    from openamrobot_build1_scene.arm_frames import TCP_FRAME
    if not (rig.get("/build1/run_info") or {}).get("gravity_feedforward"):
        log.event("no gravity feed-forward in this run: the held product is not added")
        return False
    mass = scene_model.value(scene["item"]["mass"])
    rig.set_parameters("/gravity_feedforward", {"payload_link": TCP_FRAME[SIDE], "payload_com_xyz": [0.0, 0.0, 0.0],
                                                "payload_mass_kg": mass})
    log.quantity("gravity_payload_mass", mass, "kg", note=f"set after the grasp was reported, at {TCP_FRAME[SIDE]}. Estimated")
    return True


def pick(rig, log, scene, demo, events):
    """scn04's grasp (scenarios.grasp) up to the hold, with its checks and limits, and without the release: the product
    stays in the hand for the carry. The product leaves the MoveIt planning scene before the reach and stays out of it
    (its scene pose is where it stood, not where it is carried). On a failure the gripper opens and the hand withdraws."""
    from std_msgs.msg import String
    from openamrobot_build1_scene import scenarios as s
    from openamrobot_build1_scene.arm_frames import (GRASP_ABOVE_ITEM_CENTRE_M, LIFT_RISE_M, REACH_CLEARANCE_M, TCP_FRAME,
                                                      approach_tolerances, face_yaw, grasp_line, hand_for_tcp_at, rotate_z,
                                                      side_grasp, side_grasp_offset, straight_lift, tcp_for_item)
    from openamrobot_build1_scene.planning_scene import SETTLE_S
    from openamrobot_build1_scene.rig import yaw_of
    s.limits(rig, log, s.LIMIT_ARM)
    item0 = rig.object_position("item")
    demo["item_start"] = list(item0)
    log.quantity("item_start_height", item0[2], "m")
    yaw = face_yaw(rig.truth()[2], yaw_of(rig.get("/build1/ground_truth/item/pose").pose.orientation))
    log.quantity("grasp_face_yaw", yaw, "rad", note="the side grasp turned about z of base_footprint to the product's face")
    quaternion = side_grasp(yaw)
    item = s._in_base(rig, item0)
    grasp_z = item[2] + GRASP_ABOVE_ITEM_CENTRE_M
    rig.wait(lambda: rig.transform(LINK, TCP_FRAME[SIDE]) is not None, 30.0, "the hand base to TCP transform")
    found = rig.transform(LINK, TCP_FRAME[SIDE])
    if not log.require(found is not None, f"no /tf from {LINK} to {TCP_FRAME[SIDE]}"):
        return False
    log.quantity("tcp_depth_along_fingers", side_grasp_offset(found[0])[0], "m", note=f"{TCP_FRAME[SIDE]} in {LINK}, from /tf")
    hand = hand_for_tcp_at((item[0], item[1], grasp_z), found[0], yaw)
    back = rotate_z((0.10, 0.0, 0.0), yaw)
    approach = (hand[0] - back[0], hand[1] - back[1], hand[2])

    s.park_arms(rig, log)
    opened = rig.gripper(SIDE, s.OPEN)
    log.require(opened is not None, "the gripper open command got no result")
    rig.sleep_sim(1.0)
    log.quantity("finger_open_position", rig.physics_joint(FINGER), "rad", low=s.OPEN - 0.08, high=s.OPEN + 0.02)
    log.quantity("finger_mimic_error_open", rig.finger_mimic_error(), "rad", low=0.0, high=0.03,
                 note="finger_joint2 against finger_joint1 in the physics (SW-09)")
    events("pick: approach")
    if not s._move_group(rig, log, s._pose_goal(f"{SIDE}_arm", LINK, approach, quaternion, 0.3,
                                                 angle_tolerance=approach_tolerances(quaternion)), "grasp_approach"):
        return False
    rig.publish("exclude", String(data="item"))
    rig.sleep_sim(SETTLE_S)
    held, closing = False, False
    try:
        approached = rig.transform("base_footprint", LINK)
        if not log.require(approached is not None, "no /tf of the hand after the approach"):
            return False
        held_orientation = (approached[1].x, approached[1].y, approached[1].z, approached[1].w)
        tcp_x, tcp_y, _ = tcp_for_item((item[0], item[1], grasp_z), held_orientation)
        pre_grasp, above = grasp_line((tcp_x, tcp_y, grasp_z + REACH_CLEARANCE_M), found[0], held_orientation)
        _, hand = grasp_line((tcp_x, tcp_y, grasp_z), found[0], held_orientation)
        events("pick: reach")
        reached = s._cartesian(rig, log, SIDE, LINK, [pre_grasp, above, hand], held_orientation, "grasp_reach", 0.1)
        rig.sleep_sim(1.0)
        if not reached:
            return False
        log.quantity("item_pushed_before_closing", math.dist(rig.object_position("item")[:2], item0[:2]), "m", low=0.0, high=0.05)
        closing = True
        events("pick: close")
        closed = rig.gripper(SIDE, s.CLOSED, sim_s=30.0)
        rig.sleep_sim(1.5)
        finger_closed = rig.physics_joint(FINGER)
        log.quantity("finger_position_on_item", finger_closed, "rad", low=0.02, high=s.OPEN - 0.05,
                     note="above 0 means the fingers stopped on the product and did not close fully")
        log.quantity("finger_driver_reading_on_item", rig.joint(FINGER), "rad", note="what the driver reads back over CAN")
        if closed is not None:
            log.quantity("gripper_reported_stalled", bool(closed[1]), "bool")
            log.quantity("gripper_reported_reached_goal", bool(closed[0]), "bool")
        payload = _held_payload(rig, log, scene) if closed is not None and finger_closed > 0.02 else False
        demo["payload_set"] = payload
        at_grasp = rig.transform("base_footprint", LINK)
        tcp_at_grasp = rig.transform("base_footprint", TCP_FRAME[SIDE])
        if not log.require(at_grasp is not None and tcp_at_grasp is not None, "no /tf of the hand and the TCP after the close"):
            return False
        item_closed = rig.object_position("item")
        rotation = at_grasp[1]
        lifted, lift_orientation = straight_lift(at_grasp[0], (rotation.x, rotation.y, rotation.z, rotation.w), LIFT_RISE_M)
        events("pick: lift")
        if not s._cartesian(rig, log, SIDE, LINK, [lifted], lift_orientation, "grasp_lift", 0.1):
            return False
        rig.sleep_sim(3.0)
        item1 = rig.object_position("item")
        log.quantity("item_height_gain_after_lift", item1[2] - item0[2], "m", low=ITEM_RISE_M, high=0.12)
        tcp_lifted = rig.transform("base_footprint", TCP_FRAME[SIDE])
        if tcp_lifted is not None:
            tcp_rise = tcp_lifted[0][2] - tcp_at_grasp[0][2]
            log.quantity("grasp_lift_tcp_rise", tcp_rise, "m", note="TCP from /tf. Commanded 0.08")
            log.quantity("grasp_lift_item_rise_minus_tcp_rise", (item1[2] - item_closed[2]) - tcp_rise, "m",
                         note="recorded, no limit: below 0 the product slid down in the fingers")
        rig.sleep_sim(3.0)
        item2 = rig.object_position("item")
        log.quantity("item_height_drift_while_held", item2[2] - item1[2], "m", low=-0.01, high=0.01, note="over 3 s of Gazebo time")
        held = item2[2] - item0[2] >= ITEM_RISE_M
        return held
    finally:
        if not held:
            if closing:
                rig.gripper(SIDE, s.OPEN)
                events("pick left early: the gripper opened where the hand is")
            s._withdraw(rig, log, SIDE, LINK)
            if demo.get("payload_set"):
                rig.set_parameters("/gravity_feedforward", {"payload_mass_kg": 0.0})
                demo["payload_set"] = False
                log.event("gravity feed-forward payload cleared")


def held_checks(rig, log, demo, when):
    """The product is still in the hand: its rise above the start (scn04's 0.04 m) and its distance to the TCP."""
    from openamrobot_build1_scene.arm_frames import TCP_FRAME
    item = rig.object_position("item")
    log.quantity(f"item_height_gain_{when}", item[2] - demo["item_start"][2], "m", low=ITEM_RISE_M, high=0.12,
                 note="the item rise requirement of scn04 and Baseline 1")
    tcp = rig.transform("base_footprint", TCP_FRAME[SIDE])
    if tcp is not None:
        x, y, yaw = rig.truth()
        c, s_ = math.cos(yaw), math.sin(yaw)
        world = (x + c * tcp[0][0] - s_ * tcp[0][1], y + s_ * tcp[0][0] + c * tcp[0][1], tcp[0][2])
        log.quantity(f"item_distance_to_tcp_{when}", math.dist(item, world), "m", low=0.0, high=ITEM_TCP_GUARD_M,
                     note="product centre (ground truth) to the TCP (/tf on the ground truth base pose). Guard value, estimated")
    else:
        log.require(False, f"item_distance_to_tcp_{when}: no /tf of the TCP")


def retreat_with_item(rig, log, demo, events):
    """Baseline 1's Nav2 backup 0.30 m, with the arm left where it holds the product."""
    from nav2_msgs.action import BackUp
    from openamrobot_build1_scene import scenarios as s
    x0, y0, yaw0 = rig.truth()
    goal = BackUp.Goal()
    goal.target.x, goal.speed, goal.time_allowance = b1.RETREAT_M, b1.RETREAT_SPEED, s._duration(60.0)
    events("backup 0.30 m with the product held")
    handle = rig.send_goal("/backup", "nav2_msgs.action", "BackUp", goal)
    if not log.require(handle is not None, "Nav2 backup did not accept the goal"):
        return
    status, _ = rig.wait_result(handle, 120.0, "retreat with the product")
    log.quantity("retreat_status", status, "GoalStatus", low=b1.SUCCEEDED, high=b1.SUCCEEDED)
    log.quantity("retreat_settle_time", rig.wait_stopped(10.0), "s of Gazebo time", low=0.0, high=10.0)
    x1, y1, yaw1 = rig.truth()
    along = (x1 - x0) * math.cos(yaw0) + (y1 - y0) * math.sin(yaw0)
    log.quantity("retreat_along_heading", along, "m", low=-(b1.RETREAT_M + 0.05), high=-(b1.RETREAT_M - 0.05),
                 note="negative: away from the shelf. Guard value of Baseline 1")
    log.quantity("retreat_heading_change", math.degrees(angle_diff(yaw1, yaw0)), "deg", low=-5.0, high=5.0, note="as in scn01")
    held_checks(rig, log, demo, "after_backup")


def nav_to_packing(rig, log, scene, demo, events):
    """To the approach pose in front of the packing station, then straight in to the station; the product still held."""
    events(f"packing approach: turn {APPROACH_M} m before the station, then straight in")
    first = navigate(rig, log, scene, "packing_approach", demo, events, pose=approach_pose(scene, PACK_STATION), checks=False)
    if first != b1.SUCCEEDED:
        return
    navigate(rig, log, scene, PACK_STATION, demo, events)
    held_checks(rig, log, demo, "at_packing")


def _lower_and_open(rig, log, scene, tote, events):
    """Lower the held product to PLACE_CLEARANCE_M above the order tote floor and open. -> True once the gripper opened."""
    from openamrobot_build1_scene import grocery_scene
    from openamrobot_build1_scene import scenarios as s
    from openamrobot_build1_scene.arm_frames import straight_lift
    half = scene_model.value(scene["item"]["size"])[2] / 2.0
    side_half = scene_model.value(scene["item"]["size"])[0] / 2.0
    item = rig.object_position("item")
    along, deep = grocery_scene.in_tote(scene, item)
    log.quantity("place_item_offset_along_tote", along, "m", note="product centre in the order tote frame before lowering")
    log.quantity("place_item_offset_deep_tote", deep, "m")
    over = abs(along) <= tote["inner_half"][0] - side_half and abs(deep) <= tote["inner_half"][1] - side_half
    if not log.require(over, f"item_not_over_tote: the product is not over the order tote interior (along {along:.3f}, deep {deep:.3f})"):
        return False
    bottom = item[2] - half
    lower = min(max(bottom - (tote["floor_top"] + PLACE_CLEARANCE_M), 0.0), PLACE_LOWER_MAX_M)
    log.quantity("place_lowering_distance", lower, "m", note=f"product bottom {bottom:.3f} m to {PLACE_CLEARANCE_M} m above the tote floor")
    hand = rig.transform("base_footprint", LINK)
    if not log.require(hand is not None, "no /tf of the hand before the place"):
        return False
    target, orientation = straight_lift(hand[0], (hand[1].x, hand[1].y, hand[1].z, hand[1].w), -lower)
    events(f"place: lower {lower:.3f} m")
    if not s._cartesian(rig, log, SIDE, LINK, [target], orientation, "place_lower", 0.1):
        return False
    rig.sleep_sim(1.0)
    events("place: open")
    opened = rig.gripper(SIDE, s.OPEN)
    log.require(opened is not None, "the gripper open command got no result (place)")
    return True


def place(rig, log, scene, demo, events):
    """Lower the product into the order tote, open, check that it rests in the tote, withdraw the hand. On any path out
    of the place the gripper is open, the payload is out of the gravity feed-forward and the hand has withdrawn, so the
    retreat that follows never drags a held product."""
    from openamrobot_build1_scene import grocery_scene
    from openamrobot_build1_scene import scenarios as s
    tote = grocery_scene.order_tote(scene)
    half = scene_model.value(scene["item"]["size"])[2] / 2.0
    opened = False
    try:
        opened = _lower_and_open(rig, log, scene, tote, events)
    finally:
        if not opened:
            rig.gripper(SIDE, s.OPEN)
            events("place left early: the gripper opened where the hand is")
        rig.sleep_sim(1.5)
        if demo.get("payload_set"):
            rig.set_parameters("/gravity_feedforward", {"payload_mass_kg": 0.0})
            demo["payload_set"] = False
            log.event("gravity feed-forward payload cleared")
        if opened:
            log.quantity("finger_open_after_place", rig.physics_joint(FINGER), "rad", low=s.OPEN - 0.08, high=s.OPEN + 0.02,
                         note="the open window of scn04's finger_open_position")
        events("place: withdraw")
        s._withdraw(rig, log, SIDE, LINK)
    if not opened:
        return
    rig.sleep_sim(2.0)
    p0, t0 = rig.object_position("item"), rig.sim_time()
    rig.sleep_sim(1.0)
    p1, t1 = rig.object_position("item"), rig.sim_time()
    log.quantity("item_speed_after_release", math.dist(p0, p1) / max(t1 - t0, 1e-6), "m/s", low=0.0, high=SETTLED_SPEED,
                 note="product centre over 1 s of Gazebo time, 2 s after the withdraw. Guard value, estimated")
    along, deep = grocery_scene.in_tote(scene, p1)
    log.quantity("item_in_tote_along", abs(along), "m", low=0.0, high=tote["inner_half"][0],
                 note="|product centre| from the tote centre, inside the inner wall")
    log.quantity("item_in_tote_deep", abs(deep), "m", low=0.0, high=tote["inner_half"][1])
    log.quantity("item_height_above_tote_floor", p1[2] - tote["floor_top"], "m", low=0.0, high=half + 0.01,
                 note="product centre above the tote floor: standing (half its height) or lying (less)")
    demo["item_end"] = list(p1)


# ---------------------------------------------------------------- the mission

def _snapshot(rig, phase):
    """Controller and firmware states at the end of a phase."""
    from openamrobot_build1_scene import scenarios as s
    out = {"phase": phase, "sim": rig.sim_time()}
    try:
        out["controllers"] = s._controller_states(rig, "/controller_manager")
        if rig.devices:
            out["sim_controllers"] = s._controller_states(rig, "/sim/controller_manager")
    except Exception as error:  # noqa: BLE001 - a snapshot is a record, never a check
        out["controllers_error"] = str(error)
    for topic in ("/sil/drive/status", "/sil/lift/status", "/sil/lift/emulator_state", "/sil/bms/status", "/sil/fw/status"):
        out[topic] = rig.get(topic)
    return out


def mission(rig, log):
    scene, scene_name = scene_of(rig)
    demo = {"profile": phases.PROFILE, "seed": SEED, "scene": scene_name, "spawn": SPAWN_STATION, "pick": PICK_STATION,
            "pack": PACK_STATION, "travelled": 0.0, "pose_source": None}
    log.data["grocery"] = demo
    events = Events(rig, log)
    events(f"grocery dark store demonstrator, profile {phases.PROFILE}, seed {SEED}, scene {scene_name}")
    from openamrobot_build1_scene.diagnostics import enabled
    tracker = phases.Tracker(log, clock=rig.sim_time)
    recorders = record.Recorders(log)
    try:
        recorders.start(rig)
        recorders.subscribe_frames(rig)
    except Exception as error:  # noqa: BLE001 - a recording is never a check
        log.event(f"recording: {error}")
    camera = record.CameraPath(rig, tracker, log).start() if recorders.frames else None
    contacts = record.ContactEvents(rig, tracker) if enabled(os.environ) else None
    trace = b1.Trace(rig, tracker).start()
    states = []
    sim0, wall0 = rig.sim_time(), time.time()

    def odometer():
        last = None
        while trace.running:
            try:
                x, y, _ = rig.truth()
                if last is not None:
                    demo["travelled"] += math.hypot(x - last[0], y - last[1])
                last = (x, y)
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.1)
    threading.Thread(target=odometer, daemon=True).start()

    def run(phase, function, then=()):
        events.phase_started(phase)
        events(f"phase {phase} start")
        tracker.run(phase, function, then=then)
        states.append(_snapshot(rig, phase))
        record_ = tracker.records[phase]
        return phases.verdict(record_) == "PASS"

    def skip(names, reason):
        for name in names:
            tracker.skip(name, reason)
            events(f"phase {name} not run: {reason}")

    status = {}
    try:
        run("spawn", lambda: spawn(rig, log, scene, demo))
        status["nav"] = None
        run("nav_to_shelf", lambda: status.__setitem__("nav", navigate(rig, log, scene, PICK_STATION, demo, events, reach_budget=True)))
        if status["nav"] == b1.SUCCEEDED:
            run("stop_authorize", lambda: b1.stop_authorize(rig, log))
            run("lift_to_target", lambda: b1.lift_to_target(rig, log))
            status["held"] = False
            run("arm_approach", lambda: status.__setitem__("held", pick(rig, log, scene, demo, events)), then=("grasp", "lift_object"))
            if status["held"]:
                run("retreat_with_item", lambda: retreat_with_item(rig, log, demo, events))
                run("nav_to_packing", lambda: nav_to_packing(rig, log, scene, demo, events))
                run("place", lambda: place(rig, log, scene, demo, events))
            else:
                skip(("retreat_with_item", "nav_to_packing", "place"), "the product is not held after lift_object")
            run("retreat", lambda: b1.retreat(rig, log))
        else:
            skip(("stop_authorize", "lift_to_target", "arm_approach", "grasp", "lift_object", "retreat_with_item",
                  "nav_to_packing", "place", "retreat"), "Nav2 did not reach the work shelf")
        if demo.get("odom_frozen"):
            skip(("return_to_parking",), "odometry frozen (see GROCERY ODOM lines)")
        else:
            run("return_to_parking", lambda: navigate(rig, log, scene, SPAWN_STATION, demo, events))
    finally:
        if camera is not None:
            camera.finished = True
            rig.sleep_sim(2.0)
            camera.stop()
        trace.stop()
        recorders.stop()
        demo["sim_seconds"] = round(rig.sim_time() - sim0, 2)
        demo["wall_seconds"] = round(time.time() - wall0, 1)
        log.data["phases"] = tracker.result()
        log.data["trace"] = trace.samples
        directory = recorders.directory
        record.write_json(directory, "grocery_states.json", states)
        if contacts is not None:
            record.write_json(directory, "grocery_contacts.json", {"available": contacts.available, "rows": contacts.rows()})
        table = phases.rows(log.data)
        demo.pop("_probe", None)
        report = {"title": phases.TITLE, "profile": phases.PROFILE, "pose_source": demo.get("pose_source"),
                  "evidence_class": "simulation only: not hardware, physical acceptance, perception or safety evidence",
                  "scene": scene_name, "seed": SEED, "run": log.data.get("run", {}), "grocery": demo,
                  "phases": [{"phase": r[0], "verdict": r[3], "classes": r[4], "gazebo_s": r[5], "detail": r[6],
                              "sim_start": rec["sim_start"], "sim_end": rec["sim_end"]}
                             for r, rec in zip(table, tracker.result())],
                  "complete": phases.complete(table)}
        record.write_json(directory, "grocery_demo_report.json", report)
        for line in b1.phases.trace_summary(trace.samples, tracker.result()):
            print(f"GROCERY TRACE {line}", flush=True)
        print(phases.render(table), flush=True)
    blocked = tracker.blocked()
    if blocked and not log.failures:
        raise b1.Blocked("; ".join(f"{r['phase']}: {r['exception']['text']}" for r in blocked))
