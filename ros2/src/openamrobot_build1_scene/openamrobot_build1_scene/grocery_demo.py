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
    log.event(f"profile {phases.PROFILE}; planning_scene_loader pose_source: {source}")
    log.quantity("profile_pose_source_ground_truth", source == phases.POSE_SOURCE, "bool", low=True, high=True,
                 note="the sim-oracle profile declares pose_source: ground_truth (docs/integration/baseline1.md)")


def navigate(rig, log, scene, name, demo, events, reach_budget=False):
    """Nav2 navigate_to_pose to station `name` with the station tree of openamr-platform-sw #164, and the checks of
    Baseline 1's nav_to_station. The arm-reach budget (0.04 m, 0.08 rad) applies where the arm must reach a target from
    the stop: the work shelf. Elsewhere the station is held to Nav2's own goal tolerance."""
    from nav2_msgs.action import NavigateToPose
    amcl = b1.AmclListener(rig)
    rig.sleep_sim(0.5)
    count0 = amcl.count
    gx, gy, gyaw = station(scene, name)
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
    status, _ = rig.wait_result(handle, NAV_SIM_S, f"navigate to {name}")
    events(f"nav result {name}: status {status}")
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


def place(rig, log, scene, demo, events):
    """Lower the product into the order tote, open, check that it rests in the tote, withdraw the hand."""
    from openamrobot_build1_scene import grocery_scene
    from openamrobot_build1_scene import scenarios as s
    from openamrobot_build1_scene.arm_frames import straight_lift
    tote = grocery_scene.order_tote(scene)
    half = scene_model.value(scene["item"]["size"])[2] / 2.0
    item = rig.object_position("item")
    along, deep = grocery_scene.in_tote(scene, item)
    log.quantity("place_item_offset_along_tote", along, "m", note="product centre in the order tote frame before lowering")
    log.quantity("place_item_offset_deep_tote", deep, "m")
    side_half = scene_model.value(scene["item"]["size"])[0] / 2.0
    over = abs(along) <= tote["inner_half"][0] - side_half and abs(deep) <= tote["inner_half"][1] - side_half
    if not log.require(over, f"item_not_over_tote: the product is not over the order tote interior (along {along:.3f}, deep {deep:.3f})"):
        return
    bottom = item[2] - half
    lower = min(max(bottom - (tote["floor_top"] + PLACE_CLEARANCE_M), 0.0), PLACE_LOWER_MAX_M)
    log.quantity("place_lowering_distance", lower, "m", note=f"product bottom {bottom:.3f} m to {PLACE_CLEARANCE_M} m above the tote floor")
    hand = rig.transform("base_footprint", LINK)
    if not log.require(hand is not None, "no /tf of the hand before the place"):
        return
    target, orientation = straight_lift(hand[0], (hand[1].x, hand[1].y, hand[1].z, hand[1].w), -lower)
    events(f"place: lower {lower:.3f} m")
    if not s._cartesian(rig, log, SIDE, LINK, [target], orientation, "place_lower", 0.1):
        return
    rig.sleep_sim(1.0)
    events("place: open")
    opened = rig.gripper(SIDE, s.OPEN)
    log.require(opened is not None, "the gripper open command got no result (place)")
    rig.sleep_sim(1.5)
    log.quantity("finger_open_after_place", rig.physics_joint(FINGER), "rad", low=s.OPEN - 0.08, high=s.OPEN + 0.02,
                 note="the open window of scn04's finger_open_position")
    if demo.get("payload_set"):
        rig.set_parameters("/gravity_feedforward", {"payload_mass_kg": 0.0})
        demo["payload_set"] = False
        log.event("gravity feed-forward payload cleared")
    events("place: withdraw")
    s._withdraw(rig, log, SIDE, LINK)
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
                run("nav_to_packing", lambda: (navigate(rig, log, scene, PACK_STATION, demo, events), held_checks(rig, log, demo, "at_packing")))
                run("place", lambda: place(rig, log, scene, demo, events))
            else:
                skip(("retreat_with_item", "nav_to_packing", "place"), "the product is not held after lift_object")
            run("retreat", lambda: b1.retreat(rig, log))
        else:
            skip(("stop_authorize", "lift_to_target", "arm_approach", "grasp", "lift_object", "retreat_with_item",
                  "nav_to_packing", "place", "retreat"), "Nav2 did not reach the work shelf")
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
