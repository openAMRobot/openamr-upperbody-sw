# SPDX-License-Identifier: MIT
"""The side grasp turned to the item's face (arm_frames.py) and its use in scn04's grasp. No ROS import."""
import math
import os

import numpy as np
import pytest

from openamrobot_build1_scene import arm_frames as af

HERE = os.path.dirname(os.path.abspath(__file__))


def matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def test_no_turn_is_the_old_side_grasp():
    assert np.allclose(af.side_grasp(0.0), af.SIDE_GRASP)
    assert np.allclose(af.side_grasp_offset((0.001, 0.0, -0.138)), (0.138, 0.0, 0.001))
    assert np.allclose(af.hand_for_tcp_at((0.75, 0.16, 0.925), (0.0, 0.0, -0.138)), (0.612, 0.16, 0.925))


@pytest.mark.parametrize("yaw", [-0.3, -0.073, 0.05, 0.2])
def test_the_turned_grasp_puts_the_tcp_on_the_point_and_the_fingers_along_the_yaw(yaw):
    q = af.side_grasp(yaw)
    assert math.isclose(sum(c * c for c in q), 1.0, abs_tol=1e-12)
    offset = np.array([-0.0014, 0.0, -0.138])
    point = np.array([0.75, 0.16, 0.925])
    hand = np.array(af.hand_for_tcp_at(point, offset, yaw))
    assert np.allclose(hand + matrix(*q) @ offset, point, atol=1e-12)
    fingers = matrix(*q) @ np.array([0.0, 0.0, -1.0])
    assert np.allclose(fingers, (math.cos(yaw), math.sin(yaw), 0.0), atol=1e-12)


def test_the_face_yaw_is_the_nearest_face_within_a_quarter_turn():
    station = -math.pi / 2.0
    assert math.isclose(af.face_yaw(station, 0.0), 0.0, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(station + 0.073, 0.0), -0.073, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(station, 0.1), 0.1, abs_tol=1e-12)
    assert math.isclose(af.face_yaw(0.0, math.pi / 2.0 + 0.02), 0.02, abs_tol=1e-12)
    for item in np.linspace(-4, 4, 41):
        assert -math.pi / 4.0 <= af.face_yaw(0.3, item) < math.pi / 4.0


def test_scenario_4_turns_the_grasp_and_the_approach_to_the_items_face():
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "scenarios.py"), encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("def grasp(rig, log):"):text.index("def _held_payload(")]
    assert "quaternion = side_grasp(yaw)" in body and "quaternion = SIDE_GRASP" not in body
    assert "hand_for_tcp_at((item[0], item[1], grasp_z), found[0], yaw)" in body
    assert "rotate_z((0.10, 0.0, 0.0), yaw)" in body


def _grasp_body():
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "scenarios.py"), encoding="utf-8") as f:
        text = f.read()
    return text[text.index("def grasp(rig, log):"):text.index("def _held_payload(")]


def test_the_straight_lift_translates_the_hand_and_keeps_its_orientation():
    q = (0.01, -0.70, 0.02, 0.71)
    xyz, kept = af.straight_lift((0.6, 0.17, 0.93), q, 0.08)
    assert np.allclose(xyz, (0.6, 0.17, 1.01)) and kept == q
    offset = np.array([-0.0014, 0.0, -0.138])
    tcp_before = np.array((0.6, 0.17, 0.93)) + matrix(*q) @ offset
    tcp_after = np.array(xyz) + matrix(*kept) @ offset
    assert np.allclose(tcp_after - tcp_before, (0.0, 0.0, 0.08), atol=1e-12)


def test_the_old_lift_goal_region_allowed_a_tcp_below_the_40_mm_requirement():
    # The old goal: the hand base within 0.015 m, each axis within 0.3 rad. The TCP is 0.138 m from the hand base.
    assert af.worst_tcp_rise(0.08, 0.015, 0.3, 0.138) < 0.0
    # Measured with that goal (FK of the driver's joint readings, Baseline 1 runs of 17 Nov): hand turned up to 28.9 degrees,
    # which alone moves the TCP by up to 2 * 0.138 * sin(28.9 deg / 2) = 0.069 m.
    assert 2.0 * 0.138 * math.sin(math.radians(28.9) / 2.0) > 0.06


@pytest.mark.parametrize("yaw", [0.0, 0.2])
@pytest.mark.parametrize("tilt", [0.0, 0.15])
def test_the_grasp_line_runs_along_the_fingers_and_ends_with_the_tcp_on_the_point(yaw, tilt):
    x, y, z, w = af.side_grasp(yaw)
    tx, ty, tz, tw = (math.sin(tilt / 2.0), 0.0, 0.0, math.cos(tilt / 2.0))   # the approach ended `tilt` off about x
    q = (w * tx + x * tw + y * tz - z * ty, w * ty - x * tz + y * tw + z * tx,
         w * tz + x * ty - y * tx + z * tw, w * tw - x * tx - y * ty - z * tz)
    offset = (-0.0014, 0.0, -0.138)
    point = (0.75, 0.16, 0.925)
    pre, grasp = af.grasp_line(point, offset, q)
    R = matrix(*q)
    assert np.allclose(np.array(grasp) + R @ np.array(offset), point, atol=1e-12)
    fingers = R @ np.array((0.0, 0.0, -1.0))
    assert np.allclose(np.array(grasp) - np.array(pre), af.REACH_STANDOFF_M * fingers, atol=1e-12)
    if tilt == 0.0:
        assert np.allclose(np.array(grasp) - np.array(pre), (0.10 * math.cos(yaw), 0.10 * math.sin(yaw), 0.0), atol=1e-12)


def test_scenario_4_reaches_and_lifts_on_straight_lines_with_the_orientation_held():
    body = _grasp_body()
    assert '_pose_goal(f"{side}_arm", link, hand' not in body and '_pose_goal(f"{side}_arm", link, lifted' not in body
    assert "waypoints = reach_waypoints((tcp_x, tcp_y, grasp_z), found[0], held_orientation)" in body
    assert "straight_lift(at_grasp[0], (rotation.x, rotation.y, rotation.z, rotation.w), LIFT_RISE_M)" in body
    assert '_cartesian(rig, log, side, link, [lifted], lift_orientation, "grasp_lift", 0.1)' in body
    assert af.LIFT_RISE_M == 0.08 and af.CARTESIAN_MIN_FRACTION >= 0.99 and af.CARTESIAN_STEP_M <= 0.01
    assert 'log.quantity("item_height_gain_after_lift", item1[2] - item0[2], "m", low=0.04, high=0.12)' in body


def test_the_exclusion_ends_on_every_path_out_of_the_block():
    from openamrobot_build1_scene.planning_scene import excluded_from_planning

    class Message:
        def __init__(self, data):
            self.data = data

    sent, slept = [], []
    with excluded_from_planning(lambda key, m: sent.append((key, m.data)), slept.append, "item,tote", Message):
        assert sent == [("exclude", "item,tote")]
    assert sent == [("exclude", "item,tote"), ("exclude", "")] and slept == [2.0, 2.0]
    sent.clear()
    with pytest.raises(RuntimeError):
        with excluded_from_planning(lambda key, m: sent.append((key, m.data)), slept.append, "item", Message):
            raise RuntimeError("a failed move")
    assert sent == [("exclude", "item"), ("exclude", "")]


def test_scenario_4_puts_the_item_and_tote_back_after_the_release_and_clears_the_payload():
    body = _grasp_body()
    assert 'rig.publish("exclude"' not in body, "the exclusion goes through excluded_from_planning only"
    lines = body.splitlines()
    block = next(i for i, line in enumerate(lines) if "with excluded_from_planning(rig.publish, rig.sleep_sim, \"item\", String):" in line)
    indent = len(lines[block]) - len(lines[block].lstrip())
    inner = [i for i, line in enumerate(lines) if i > block and line.strip() and len(line) - len(line.lstrip()) <= indent][0]
    inside = "\n".join(lines[block:inner])
    for step in ('"grasp_reach"', "rig.gripper(side, CLOSED", '"grasp_lift"', "rig.gripper(side, OPEN)"):
        assert step in inside, step
    cleanup = inside.split("finally:")[-1]
    assert "finally:" in inside and '{"payload_mass_kg": 0.0}' in cleanup
    # Released and withdrawn before the item comes back into the planning scene, on every path out of the grasp.
    assert "if closing and not released:" in cleanup and cleanup.index("rig.gripper(side, OPEN)") < cleanup.index("_withdraw(rig, log, side, link)")
    assert "tote" not in lines[block], "the tote stays in the planning scene"


def test_the_reach_clears_the_tote_wall_and_the_lift_clears_the_board_above(scene):
    """The straight part of the reach runs REACH_CLEARANCE_M above the grasp height. A tilt of the hand inside the approach
    tolerance (up to sqrt(3) times the per-axis value) lowers the front of the hand body, 0.11 m along the fingers, by
    0.11 sin(tilt); the executed hand leaves its plan by up to 0.009 m. The lifted hand body stays below the work shelf
    board above, padded in the planning scene. Mesh extents from openarm_description at the pins (arm_frames.py)."""
    from openamrobot_build1_scene import scene as scene_model
    from openamrobot_build1_scene.planning_scene import FIXTURE_PADDING_M
    boxes = {name: (size, centre) for name, size, centre, _ in scene_model.collision_boxes(scene)}
    tote, item, board1 = boxes["tote"], boxes["item"], boxes["work_shelf_board_1"]
    wall_top = tote[1][2] + tote[0][2] / 2.0
    item_top = item[1][2] + item[0][2] / 2.0
    board1_bottom = board1[1][2] - board1[0][2] / 2.0

    def lowest_on_the_way_in(above_centre, clearance, tolerance):
        tilt = math.sqrt(3.0) * tolerance
        return (item[1][2] + above_centre + clearance - af.HAND_BODY_BELOW_TCP_M
                - af.HAND_BODY_REACH_ALONG_FINGERS_M * math.sin(tilt) - 0.009)

    def lifted_top(above_centre):
        return item[1][2] + above_centre + af.LIFT_RISE_M + af.HAND_BODY_ABOVE_TCP_M

    grasp = af.GRASP_ABOVE_ITEM_CENTRE_M
    assert lowest_on_the_way_in(grasp, af.REACH_CLEARANCE_M, af.APPROACH_ANGLE_TOLERANCE_RAD) > wall_top
    assert lowest_on_the_way_in(0.03, 0.0, 0.3) < wall_top, "the old straight reach at the grasp height reached the wall"
    # The planning scene does not pad undersides (planning_scene.padded). Held orientations tilted up to 7 degrees
    # (FK of the grasps of 17 Nov) raise the hand body's far corner, 0.11 m along the fingers, by 0.11 sin(7 deg).
    tilt = math.radians(7.0)
    assert lifted_top(grasp) + af.HAND_BODY_REACH_ALONG_FINGERS_M * math.sin(tilt) < board1_bottom
    assert lifted_top(grasp) + af.HAND_BODY_REACH_ALONG_FINGERS_M * math.sin(tilt) > board1_bottom - FIXTURE_PADDING_M, \
        "with the underside padded the tilted lift reaches the padding (fractions 0.9375 and 0.941)"
    assert lifted_top(0.05) > board1_bottom, "closing at 0.05 m left no room for the lift (696458d)"
    # The fingers close on the item: their lower edge is well below the item's top.
    assert item[1][2] + grasp - af.FINGER_HALF_HEIGHT_M < item_top - 0.06
    body = _grasp_body()
    assert "angle_tolerance=approach_tolerances(quaternion)" in body and "grasp_z = item[2] + GRASP_ABOVE_ITEM_CENTRE_M" in body
    assert "waypoints = reach_waypoints((tcp_x, tcp_y, grasp_z), found[0], held_orientation)" in body
    assert '_cartesian(rig, log, side, link, waypoints, held_orientation, "grasp_reach", 0.1)' in body


def test_the_fixed_boxes_get_the_planning_clearance_and_the_objects_do_not(scene):
    from openamrobot_build1_scene import scene as scene_model
    from openamrobot_build1_scene.planning_scene import FIXTURE_PADDING_M, padded
    boxes = scene_model.collision_boxes(scene)
    fixed = {name for name, _s, _c, _y in scene_model.static_boxes(scene)}
    out = {name: (size, centre) for name, size, centre, _yaw in padded(boxes, fixed, FIXTURE_PADDING_M)}
    original = {name: (size, centre) for name, size, centre, _yaw in boxes}
    assert "work_shelf_board_0" in fixed and {"tote", "item", "payload"} <= set(out) and not {"tote", "item", "payload"} & fixed
    for name in fixed:
        size, centre = np.array(original[name][0]), np.array(original[name][1])
        assert np.allclose(out[name][0], size + np.array([2, 2, 1]) * FIXTURE_PADDING_M)
        assert np.isclose(out[name][1][2] - out[name][0][2] / 2.0, centre[2] - size[2] / 2.0), "underside unpadded"
        assert np.isclose(out[name][1][2] + out[name][0][2] / 2.0, centre[2] + size[2] / 2.0 + FIXTURE_PADDING_M)
    for name in ("tote", "item", "payload"):
        assert out[name] == (tuple(original[name][0]), original[name][1])
    # Larger than the measured contact-free deviation: 0.009 m at the hand base, 0.004 m more at the fingertips.
    assert FIXTURE_PADDING_M >= 0.009 + 0.03 * 0.138
    # The grasp and the lift stay possible: the padded board 0 top stays below the fingers at the grasp height (item
    # centre + 0.03 m, fingers about 0.015 m below the TCP), and the padded board 1 bottom above the lifted hand (hand
    # body about 0.04 m above the TCP, estimated). 0.04 m failed this: the lift was refused in CI.
    board0, board1, item = original["work_shelf_board_0"], original["work_shelf_board_1"], original["item"]
    assert board0[1][2] + board0[0][2] / 2.0 + FIXTURE_PADDING_M < item[1][2] + 0.03 - 0.015
    assert board1[1][2] - board1[0][2] / 2.0 > item[1][2] + 0.03 + 0.08 + 0.04
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert 'declare_parameter("fixture_padding_m", FIXTURE_PADDING_M)' in loader
    assert "self.boxes = padded(scene_model.collision_boxes(self.scene), fixed, padding)" in loader


def test_the_planning_scene_defaults_to_the_robots_own_estimate_and_the_oracle_must_be_asked_for():
    """Production-oriented runs must not plan against Gazebo's ground truth: the default is map, ground_truth is a
    simulation oracle that is set explicitly. No silent fall-back between the two."""
    from openamrobot_build1_scene import planning_scene as ps
    truth, belief = (0.01, -2.10, -1.60), ((0.03, -2.12, -1.61), "map")
    assert ps.DEFAULT_POSE_SOURCE == "map" and ps.DEFAULT_POSE_SOURCE != ps.ORACLE_POSE_SOURCE
    assert ps.choose_base_pose(ps.DEFAULT_POSE_SOURCE, truth, belief) == belief
    assert ps.choose_base_pose("ground_truth", truth, belief) == (truth, "ground truth (simulation oracle)")
    assert ps.choose_base_pose("ground_truth", None, belief) == (None, None)
    with pytest.raises(ValueError):
        ps.choose_base_pose("localization", truth, belief)
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert 'declare_parameter("pose_source", DEFAULT_POSE_SOURCE)' in loader
    assert "return choose_base_pose(self.pose_source, self.truth, self.localization_pose())" in loader
    with open(os.path.join(HERE, "..", "launch", "build1.launch.py"), encoding="utf-8") as f:
        launch = f.read()
    assert 'DeclareLaunchArgument("planning_pose_source", default_value=planning_scene.DEFAULT_POSE_SOURCE' in launch
    assert '"pose_source": pose_source' in launch


def test_every_run_declares_its_planning_pose_source():
    """The launch puts the source into /build1/run_info and logs it; the rig copies run_info into every scenario JSON
    (run.planning_pose_source) and logs the declaration; the oracle is spelled out as one."""
    from openamrobot_build1_scene import planning_scene as ps
    assert ps.declaration("map") == "planning_pose_source: map"
    assert ps.declaration("ground_truth").startswith("planning_pose_source: ground_truth (SIMULATION ORACLE")
    with open(os.path.join(HERE, "..", "launch", "build1.launch.py"), encoding="utf-8") as f:
        launch = f.read()
    assert 'info["planning_pose_source"] = pose_source' in launch and "LogInfo(msg=planning_scene.declaration(pose_source))" in launch
    assert launch.index('info["planning_pose_source"] = pose_source') < launch.index('"run_info": json.dumps(info, sort_keys=True)')
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "rig.py"), encoding="utf-8") as f:
        rig = f.read()
    assert 'self.log.data["run"] = dict(info, baseline_lock=BASELINE_LOCK)' in rig
    assert 'self.log.event(declaration(info.get("planning_pose_source", "not declared by the launch")))' in rig
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        assert "self.get_logger().info(declaration(self.pose_source))" in f.read()
    with open(os.path.join(HERE, "build1_harness.py"), encoding="utf-8") as f:
        assert 'arguments.setdefault("planning_pose_source", os.environ["BUILD1_PLANNING_POSE_SOURCE"])' in f.read()


@pytest.mark.parametrize("yaw", [0.0, -0.08, 0.05])
def test_the_approach_holds_the_fingers_square_to_the_items_face(yaw):
    """Measured: an approach that ended 0.025 to 0.075 rad off the face closed the fingers at 0.069 to 0.107 rad, with
    0.18 to 0.33 N m of grip torque, and 4 of 7 such grasps lifted nothing; 0.012 to 0.025 rad off held 5 of 5 at
    0.137 to 0.158 rad. The tolerance about the vertical stays inside the range that held."""
    tolerances = af.approach_tolerances(af.side_grasp(yaw))
    R = matrix(*af.side_grasp(yaw))
    vertical = int(np.argmax(np.abs(R[2, :])))
    assert vertical == 0, "the side grasp's hand x axis is the vertical"
    assert tolerances[vertical] == af.APPROACH_YAW_TOLERANCE_RAD <= 0.025
    assert all(t == af.APPROACH_ANGLE_TOLERANCE_RAD for i, t in enumerate(tolerances) if i != vertical)


def test_the_item_sits_in_the_fingers_short_of_the_palm():
    """The grip torque is kp times the angle where the fingers meet the item: 0.065 to 0.12 rad with the item on the TCP
    (missions, 3 of 6 no lift), 0.137 to 0.158 rad with it 0.008 to 0.013 m deeper (10 of 10 lifted)."""
    q = af.side_grasp(0.04)
    item = (0.75, 0.16, 0.925)
    tcp = np.array(af.tcp_for_item(item, q))
    fingers = matrix(*q) @ np.array((0.0, 0.0, -1.0))
    assert np.allclose(tcp - np.array(item), af.ITEM_DEPTH_BEHIND_TCP_M * fingers, atol=1e-12)
    assert 0.008 <= af.ITEM_DEPTH_BEHIND_TCP_M <= af.TCP_ALONG_FINGERS_M - af.PALM_ALONG_FINGERS_M - 0.0175
    body = _grasp_body()
    assert "tcp_x, tcp_y, _ = tcp_for_item((item[0], item[1], grasp_z), held_orientation)" in body


def _yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def test_a_movable_object_is_placed_where_it_was_observed_in_the_base_frame(scene):
    """At its start pose the observed placement equals the old static one; a yawed base and a tipped box are placed
    with their full orientation; the tote's box centre sits half its height up its own z axis."""
    from openamrobot_build1_scene import scene as scene_model
    from openamrobot_build1_scene.planning_scene import object_box_in_base
    base = (0.0, -2.08, -1.57)
    c, s = math.cos(-base[2]), math.sin(-base[2])
    movable = scene_model.movable_objects(scene)
    for name, size, centre, yaw in scene_model.collision_boxes(scene):
        if name not in movable:
            continue
        body = movable[name]
        offset = body["size"][2] / 2.0 if body["kind"] == "tote" else 0.0
        position, q = object_box_in_base(base, body["pose"][:3], _yaw_quaternion(body["pose"][3]), offset)
        dx, dy = centre[0] - base[0], centre[1] - base[1]
        assert np.allclose(position, (c * dx - s * dy, s * dx + c * dy, centre[2]), atol=1e-9), name
        assert np.allclose(matrix(*q), matrix(*_yaw_quaternion(yaw - base[2])), atol=1e-9), name
    assert np.allclose(object_box_in_base((1.0, 2.0, math.pi / 2), (1.0, 3.0, 0.5), _yaw_quaternion(math.pi / 2))[0],
                       (1.0, 0.0, 0.5), atol=1e-9)
    tipped = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))   # 90 deg about x: its z axis along -y
    position, q = object_box_in_base((0.0, 0.0, 0.0), (0.0, 0.0, 0.8), tipped, 0.1)
    assert np.allclose(position, (0.0, -0.1, 0.8), atol=1e-9)
    assert np.allclose(matrix(*q) @ np.array([0.0, 0.0, 1.0]), (0.0, -1.0, 0.0), atol=1e-9)


def test_the_knocked_over_item_of_scn03_leaves_its_start_box(scene):
    """CI job 134653118730: the L0_left reach knocked the item over (ground truth [0.0694, -2.8438, 0.8425], 0.14 m
    tall box lying down) and MoveIt, which still had it standing at its start pose, refused the return to park as a
    start state in collision. The placed box now follows the item."""
    from openamrobot_build1_scene import scene as scene_model
    from openamrobot_build1_scene.planning_scene import object_box_in_base
    base = (0.0, -2.08, -1.57)
    start = scene_model.movable_objects(scene)["item"]["pose"]
    lying = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    stale, _ = object_box_in_base(base, start[:3], _yaw_quaternion(start[3]))
    observed, q = object_box_in_base(base, (0.0694, -2.8438, 0.8425), lying)
    assert math.dist(stale, observed) > 0.08
    assert abs((matrix(*q) @ np.array([0.0, 0.0, 1.0]))[2]) < 1e-9, "the box lies down as the item does"


def test_the_scene_is_sent_again_when_an_object_moves_or_turns():
    from openamrobot_build1_scene.planning_scene import OBJECT_MOVED_M, OBJECT_TURNED_RAD, object_moved
    here = ((0.1, -2.8, 0.87), (0.0, 0.0, 0.0, 1.0))
    assert object_moved(None, here)
    assert not object_moved(here, here)
    assert not object_moved(here, ((0.1 + OBJECT_MOVED_M / 2, -2.8, 0.87), here[1]))
    assert object_moved(here, ((0.1 + 2 * OBJECT_MOVED_M, -2.8, 0.87), here[1]))
    turn = 2 * OBJECT_TURNED_RAD
    assert object_moved(here, (here[0], (math.sin(turn / 2), 0.0, 0.0, math.cos(turn / 2))))
    half = OBJECT_TURNED_RAD / 2
    assert not object_moved(here, (here[0], (0.0, 0.0, -math.sin(half / 2), -math.cos(half / 2))))  # q and -q agree


def test_the_loader_places_the_movable_objects_from_their_observed_poses_in_both_pose_sources():
    from openamrobot_build1_scene.planning_scene import MOVABLE_OBJECTS
    assert set(MOVABLE_OBJECTS) == {"tote", "item", "payload"}
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert "object_pose_topic(self.object_pose_source, name)" in loader
    assert "object_box_in_base(pose, position, quaternion, self.object_offset[name])" in loader
    assert "object_moved(self.object_sent.get(name), pose_now)" in loader
    assert "self.object_sent = dict(self.object_pose)" in loader
    # The same base pose for objects and fixtures: the declared source, never a separate truth path for the objects.
    assert loader.count("object_box_in_base(") == 1


def test_object_poses_come_from_ground_truth_only_under_a_declared_object_pose_source():
    """Alex, 18 Nov: ground truth only through an explicit, declared simulation-oracle profile, never silent. The object
    poses and grasp targets are ground truth today (a perception stand-in): every reader goes through
    object_pose_topic, which refuses an undeclared source, and no reader names a ground-truth object topic itself."""
    from openamrobot_build1_scene import planning_scene as ps
    assert ps.OBJECT_POSE_SOURCES == ("ground_truth",) and ps.DEFAULT_OBJECT_POSE_SOURCE == "ground_truth"
    assert ps.object_pose_topic("ground_truth", "item") == "/build1/ground_truth/item/pose"
    for undeclared in (None, "", "map", "detector"):
        with pytest.raises(ValueError):
            ps.object_pose_topic(undeclared, "item")
    line = ps.object_declaration("ground_truth")
    assert line.startswith("object_pose_source: ground_truth (PERCEPTION STAND-IN")
    assert "no run with it counts as the perception profile until an object detector replaces it" in line
    for path in (("scripts", "planning_scene_loader.py"), ("openamrobot_build1_scene", "rig.py"),
                 ("openamrobot_build1_scene", "scenarios.py")):
        with open(os.path.join(HERE, "..", *path), encoding="utf-8") as f:
            text = f.read()
        for literal in ("ground_truth/{name}/pose", "ground_truth/item/pose", "ground_truth/tote/pose", "ground_truth/payload/pose"):
            assert literal not in text, f"{path[-1]} reads {literal} without the declared object_pose_source"
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "rig.py"), encoding="utf-8") as f:
        rig = f.read()
    assert "return self.get(object_pose_topic(self.object_pose_source, name))" in rig
    assert "p = self.object_pose(name).pose.position" in rig
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert 'declare_parameter("object_pose_source", DEFAULT_OBJECT_POSE_SOURCE)' in loader
    assert "self.get_logger().info(object_declaration(self.object_pose_source))" in loader


def test_every_run_declares_its_object_pose_source_and_a_run_without_it_is_refused():
    """In /build1/run_info (and so in every scenario JSON, run.object_pose_source), in the launch log, in each scenario
    log, and as the loader's parameter. The rig refuses to start a scenario when run_info lacks it."""
    with open(os.path.join(HERE, "..", "launch", "build1.launch.py"), encoding="utf-8") as f:
        launch = f.read()
    assert 'DeclareLaunchArgument("object_pose_source", default_value=planning_scene.DEFAULT_OBJECT_POSE_SOURCE' in launch
    assert 'info["object_pose_source"] = object_source' in launch
    assert "LogInfo(msg=planning_scene.object_declaration(object_source))" in launch
    assert launch.index('info["object_pose_source"] = object_source') < launch.index('"run_info": json.dumps(info, sort_keys=True)')
    assert '"object_pose_source": object_source' in launch
    with open(os.path.join(HERE, "..", "openamrobot_build1_scene", "rig.py"), encoding="utf-8") as f:
        rig = f.read()
    refuse = 'if "object_pose_source" not in info:\n            raise ScenarioFailed('
    assert refuse in rig
    assert rig.index('self.log.data["run"] = dict(info, baseline_lock=BASELINE_LOCK)') < rig.index(refuse)
    assert 'self.object_pose_source = info["object_pose_source"]' in rig
    assert "self.log.event(object_declaration(self.object_pose_source))" in rig
    assert "self.object_pose_source = None" in rig
    with open(os.path.join(HERE, "build1_harness.py"), encoding="utf-8") as f:
        assert 'arguments.setdefault("object_pose_source", os.environ["BUILD1_OBJECT_POSE_SOURCE"])' in f.read()


@pytest.mark.parametrize("yaw", [0.0, 0.2])
@pytest.mark.parametrize("tilt", [0.0, 0.15])
def test_the_reach_comes_down_short_of_the_end_and_above_the_grasp_height_and_closes_on_a_straight_line(yaw, tilt):
    x, y, z, w = af.side_grasp(yaw)
    tx, tw = math.sin(tilt / 2.0), math.cos(tilt / 2.0)                        # the approach ended `tilt` off about x
    q = (w * tx + x * tw, y * tw + z * tx, z * tw - y * tx, w * tw - x * tx)
    offset, point = (-0.0014, 0.0, -0.138), (0.75, 0.16, 0.925)
    pre, above, down, end = (np.array(v) for v in af.reach_waypoints(point, offset, q))
    fingers, up = matrix(*q) @ np.array((0.0, 0.0, -1.0)), np.array((0.0, 0.0, 1.0))
    assert np.allclose(end, af.grasp_line(point, offset, q)[1], atol=1e-12)          # the end point is unchanged
    # last: the straight line onto the end point, DESCENT_STANDOFF_M along the fingers and DESCENT_RISE_M down
    assert np.allclose(end - down, af.DESCENT_STANDOFF_M * fingers - af.DESCENT_RISE_M * up, atol=1e-12)
    assert np.allclose(down - above, -(af.REACH_CLEARANCE_M - af.DESCENT_RISE_M) * up, atol=1e-12)   # down, short of it
    assert np.allclose(above - pre, (af.REACH_STANDOFF_M - af.DESCENT_STANDOFF_M) * fingers, atol=1e-12)
    assert np.allclose(pre, af.grasp_line((point[0], point[1], point[2] + af.REACH_CLEARANCE_M), offset, q)[0], atol=1e-12)
    old = af.reach_waypoints(point, offset, q, standoff=0.0)
    assert len(old) == 3 and np.allclose(old[2], end, atol=1e-12) and np.allclose(old[0], pre, atol=1e-12)
    level = af.reach_waypoints(point, offset, q, rise=0.0)                             # e7a251c: down onto the grasp height
    assert np.allclose(np.array(level[3]) - np.array(level[2]), af.DESCENT_STANDOFF_M * fingers, atol=1e-12)
    assert af.DESCENT_STANDOFF_M >= af.REACH_PATH_DEVIATION_M and af.DESCENT_RISE_M >= af.REACH_PATH_DEVIATION_M
    assert af.DESCENT_RISE_M < af.REACH_CLEARANCE_M


# The palm against the item along scn04's reach, from two recorded Baseline 1 missions: the ground truth base pose at
# arm_approach, the left arm controller's state at the end of the approach (the orientation the reach holds), and the
# grasp_reach commanded end point the mission logged. Item at its ground truth pose (0.160, -2.835, 0.895), yaw 0.
REACH_RUNS = {
    # run 44476577635 (control room end to end, ee36e87): grasp_reach_servo_torque_max 1.018 N m, the palm on the item
    "44476577635": ((-0.02065, -2.09111, -1.55065), 0.00081,
                    [-0.45529, -0.02155, -0.08030, 1.74125, 0.10548, -0.63191, -0.14591],
                    [-0.90234, -0.00542, -0.05834, 0.97391, 0.11682, -0.31423, -0.07811], 0.00155),
    # run 44470870769, leg 1 (acceptance, eb920f0): 0.158 N m, no contact
    "44470870769-leg1": ((-0.00872, -2.07965, -1.56224), 0.00078,
                         [-0.50183, -0.01202, -0.01698, 1.64130, -0.07763, -0.49954, 0.01125],
                         [-0.93651, -0.02215, -0.00937, 0.85183, -0.07229, -0.14584, -0.01797], 0.00441),
}
# The hand on the tote's front wall, from the build1-arm-diagnostics run 44609269748 of e7a251c (diag_grasp, spawned at the
# work_shelf station, lift at stop 0): the grasp_approach and grasp_reach commanded end points the runs logged. Run 4 put
# ee_base_link on tote:front_collision (7.63 N), run 5 ee_base_link (0.63 N) and ee_link1 (4.94 N), both in grasp_reach.
WALL_RUNS = {
    "44609269748-run4": ((0.0, -2.085, -math.pi / 2.0), 0.0,
                         [-0.44675, 0.17453, -0.21264, 1.73937, 0.14212, -0.51658, -0.19447],
                         [-0.90495, 0.09325, -0.2177, 0.92346, 0.10279, -0.16049, -0.08078], 0.00489),
    "44609269748-run5": ((0.0, -2.085, -math.pi / 2.0), 0.0,
                         [-0.46728, -0.20396, 0.1277, 1.74437, -0.13721, -0.51415, 0.10058],
                         [-0.89829, -0.11626, 0.18752, 0.90798, -0.09276, -0.10421, 0.03881], 0.00379),
}
ITEM_WORLD = (0.160, -2.835, 0.895)
HAND_LINKS = ("openarm_left_ee_base_link", "openarm_left_ee_link1", "openarm_left_ee_link2")
FINGERS_OPEN_RAD = 0.765       # finger_open_position in the runs above (0.7854 commanded, the far limit)


def _stl_triangles(path):
    """Triangles (n, 3, 3) of a binary STL file."""
    with open(path, "rb") as f:
        data = f.read()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    records = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=count, offset=84)
    return records["v"].astype(float)


def _palm_points(urdf, link="openarm_left_ee_base_link"):
    """Points on the collision mesh of `link` in its own frame: vertices and a barycentric grid on every triangle."""
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    collision = ET.fromstring(urdf).find(f"link[@name='{link}']/collision")
    origin, mesh = collision.find("origin"), collision.find("geometry/mesh")
    package, relative = mesh.get("filename")[len("package://"):].split("/", 1)
    triangles = _stl_triangles(os.path.join(get_package_share_directory(package), relative))
    triangles = triangles * np.array([float(v) for v in (mesh.get("scale") or "1 1 1").split()])
    assert [float(v) for v in (origin.get("rpy") or "0 0 0").split()] == [0.0, 0.0, 0.0]
    weights = np.array([(i, j, 8 - i - j) for i in range(9) for j in range(9 - i)], dtype=float) / 8.0
    points = np.einsum("wk,tkd->twd", weights, triangles).reshape(-1, 3)
    return points + np.array([float(v) for v in origin.get("xyz").split()])


def _box_clearance(points, half):
    """Smallest distance of `points` (box frame) to an axis-aligned box of half sizes `half`; minus the depth if inside."""
    outside = np.linalg.norm(np.maximum(np.abs(points) - half, 0.0), axis=1)
    inside = np.all(np.abs(points) < half, axis=1)
    if inside.any():
        return -float(np.min((half - np.abs(points[inside])).min(axis=1)))
    return float(outside.min())


def _recorded_reach(devices_urdf, run, rise=None):
    """The reach of a recorded run in base_footprint: (waypoints, held rotation, item centre, item axes, frames(q))."""
    from openamrobot_build1_scene.gravity import ARM_JOINTS, GravityModel
    (bx, by, byaw), lift, q_approach, q_end, _ = {**REACH_RUNS, **WALL_RUNS}[run]
    model, joints = GravityModel(devices_urdf), ARM_JOINTS["left"]
    hand, tcp_frame = af.HAND_FRAME["left"], af.TCP_FRAME["left"]
    fingers = {"openarm_left_finger_joint1": FINGERS_OPEN_RAD, "openarm_left_finger_joint2": FINGERS_OPEN_RAD}

    def frames(q):
        links, _ = model._frames({**dict(zip(joints, q)), "lift_joint": lift, **fingers})
        R0, p0 = links.get("base_footprint", (np.eye(3), np.zeros(3)))
        return {name: (R0.T @ R, R0.T @ (p - p0)) for name, (R, p) in links.items()}

    links = frames(q_approach)
    R_held = links[hand][0]
    tcp_offset = tuple(R_held.T @ (links[tcp_frame][1] - links[hand][1]))
    w = math.sqrt(max(0.0, 1.0 + np.trace(R_held))) / 2.0
    held = ((R_held[2, 1] - R_held[1, 2]) / (4 * w), (R_held[0, 2] - R_held[2, 0]) / (4 * w),
            (R_held[1, 0] - R_held[0, 1]) / (4 * w), w)
    c, s = math.cos(byaw), math.sin(byaw)
    R_world = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])          # world axes in base_footprint

    def in_base(point):
        return R_world @ (np.array(point[:2] + (point[2],)) - np.array((bx, by, 0.0)))

    item = in_base(ITEM_WORLD)
    tcp = af.tcp_for_item((item[0], item[1], item[2] + af.GRASP_ABOVE_ITEM_CENTRE_M), held)
    waypoints = [np.array(v) for v in af.reach_waypoints(tcp, tcp_offset, held, rise=rise)]
    # The inputs reproduce the run: its logged end point puts the hand where the last waypoint is.
    assert np.linalg.norm(frames(q_end)[hand][1] - waypoints[-1]) < 0.003
    return waypoints, R_held, item, R_world, in_base, frames(q_end)


def _sweeps(waypoints, clearance, step=0.0005):
    """The smallest `clearance(position)` of the hand base on each straight move between `waypoints`."""
    out = []
    for start, end in zip(waypoints[:-1], waypoints[1:]):
        move = end - start
        steps = max(1, int(math.ceil(np.linalg.norm(move) / step)))
        out.append(min(clearance(start + move * k / steps) for k in range(steps + 1)))
    return out


def _assert_the_last_move_is_the_short_push(waypoints, R_held):
    """The last move is the straight line onto the end point: DESCENT_STANDOFF_M along the fingers, DESCENT_RISE_M down."""
    last = waypoints[-1] - waypoints[-2]
    fingers = R_held @ np.array((0.0, 0.0, -1.0))
    assert np.allclose(last + af.DESCENT_RISE_M * np.array((0.0, 0.0, 1.0)), af.DESCENT_STANDOFF_M * fingers, atol=1e-9), \
        "the last move closes on the item along the fingers, besides DESCENT_RISE_M down"


@pytest.mark.parametrize("run", sorted(REACH_RUNS))
def test_the_palm_does_not_sweep_past_the_items_face_within_the_path_deviation(devices_urdf, scene, run):
    """The item is out of the planning scene for the reach, so MoveIt does not see the palm against it. Every move of the
    reach but the last keeps the palm's collision mesh REACH_PATH_DEVIATION_M or more from the item; the last one is the
    short straight push onto the end point. Without the standoff (reach_waypoints standoff 0) the descent of run
    44476577635 is the last move, 0.02 m long, and sweeps the palm 1.5 mm from the item's top edge, where the run and its
    reproduction recorded the palm on the item."""
    from openamrobot_build1_scene import scene as scene_model
    waypoints, R_held, item, R_item, _, _ = _recorded_reach(devices_urdf, run)
    half = np.array({name: size for name, size, _, _ in scene_model.collision_boxes(scene)}["item"]) / 2.0
    palm = _palm_points(devices_urdf)

    def clearance(position):
        return _box_clearance(((R_held @ palm.T).T + position - item) @ R_item, half)

    assert clearance(waypoints[-1]) == pytest.approx(REACH_RUNS[run][4], abs=0.0005)
    _assert_the_last_move_is_the_short_push(waypoints, R_held)
    sweeps = _sweeps(waypoints[:-1], clearance)
    assert len(sweeps) >= 2, "the reach has a descent before the last move"
    assert min(sweeps) >= af.REACH_PATH_DEVIATION_M, f"the palm sweeps {min(sweeps) * 1000:.2f} mm from the item"


@pytest.mark.parametrize("run", sorted(REACH_RUNS) + sorted(WALL_RUNS))
def test_the_hand_stays_the_path_deviation_off_the_tote_front_wall_until_the_last_move(devices_urdf, scene, run):
    """The tote is in the planning scene for the reach as one unpadded box of its outer size, so MoveIt accepts a hand that
    passes a millimetre over its front wall. The palm (ee_base_link) and the open fingers (ee_link1, ee_link2), by their
    collision meshes, stay REACH_PATH_DEVIATION_M or more from the front wall as Gazebo builds it (scene._movable_sdf) on
    every move of the reach but the last, the short push onto the end point that MoveIt has checked against the tote.
    With the descent onto the grasp height 9 mm back (e7a251c, rise 0) run 5 of 44609269748 comes down with ee_link1
    7.1 mm over the wall; the run recorded ee_link1 on tote:front_collision at 4.94 N."""
    from openamrobot_build1_scene import scene as scene_model
    waypoints, R_held, _, R_world, in_base, at_end = _recorded_reach(devices_urdf, run)
    tote = scene_model.movable_objects(scene)["tote"]
    (sx, sy, sz), wall = tote["size"], tote["wall"]
    tx, ty, tz, tyaw = tote["pose"]
    assert tyaw == 0.0
    centre = in_base((tx, ty + sy / 2.0 - wall / 2.0, tz + sz / 2.0))     # the "front" part, the face towards the robot
    half = np.array((sx, wall, sz)) / 2.0
    hand = af.HAND_FRAME["left"]
    meshes = {}
    for link in HAND_LINKS:
        R_link, p_link = at_end[link]
        R_hand, p_hand = at_end[hand]
        points = _palm_points(devices_urdf, link)
        meshes[link] = (R_hand.T @ ((R_link @ points.T).T + p_link - p_hand).T).T      # in the hand frame, fingers open

    def clearance(position):
        return min(_box_clearance(((R_held @ points.T).T + position - centre) @ R_world, half) for points in meshes.values())

    assert clearance(waypoints[-1]) > 0.0, "the end point MoveIt checked against the tote box is off the wall"
    _assert_the_last_move_is_the_short_push(waypoints, R_held)
    sweeps = _sweeps(waypoints[:-1], clearance)
    assert min(sweeps) >= af.REACH_PATH_DEVIATION_M, f"the hand passes {min(sweeps) * 1000:.2f} mm from the tote front wall"


# ------------------------------------------------------------------ the last-move mark (strict contact window, #98 review)
@pytest.mark.parametrize("yaw", [0.0, 0.2])
@pytest.mark.parametrize("tilt", [0.0, 0.15])
def test_the_last_move_mark_fires_where_the_last_move_starts_and_not_before(yaw, tilt):
    """Walk the planned reach in 0.5 mm steps: in_last_move is false on every earlier move and becomes true within
    LAST_MOVE_TOLERANCE_M of the start of the last move ('down'), and stays true on it."""
    x, y, z, w = af.side_grasp(yaw)
    tx, tw = math.sin(tilt / 2.0), math.cos(tilt / 2.0)
    q = (w * tx + x * tw, y * tw + z * tx, z * tw - y * tx, w * tw - x * tx)
    waypoints = af.reach_waypoints((0.75, 0.16, 0.925), (-0.0014, 0.0, -0.138), q)
    points = [np.array(p) for p in waypoints]
    path = []
    for start, end in zip(points[:-1], points[1:]):
        steps = max(1, int(math.ceil(np.linalg.norm(end - start) / 0.0005)))
        path += [start + (end - start) * k / steps for k in range(steps)]
    path.append(points[-1])
    flags = [af.in_last_move(tuple(p), waypoints) for p in path]
    first = flags.index(True)
    assert np.linalg.norm(path[first] - points[-2]) <= af.LAST_MOVE_TOLERANCE_M + 0.0005
    assert all(flags[first:]), "the mark must hold through the last move"
    assert not af.in_last_move(tuple(points[0]), waypoints) and not af.in_last_move(tuple(points[1]), waypoints)
    assert not af.in_last_move((0.0, 0.0, 0.0), [points[-1]])


def test_the_grasp_marks_the_last_move_with_the_label_of_the_strict_contact_window():
    body = _grasp_body()
    assert "watcher = LastMoveMark(rig, link, waypoints).start()" in body
    assert body.index("LastMoveMark(rig, link, waypoints)") < body.index('"grasp_reach", 0.1)') < body.index("watcher.stop()")
    from openamrobot_build1_scene import baseline1_gates, scenarios
    window = baseline1_gates.load()["contacts"]["windows"]["grasp_reach_last_move"]
    assert window == [scenarios.LastMoveMark.LABEL, "grasp_reach_end"]
# ------------------------------------------------------------- the return to park leaves the shelf first
# Diagnostics run 5 of #99 (8878aaf, job 136508288405): the left hand base where the level 0 reach left it, in base_footprint
# (from the driver's joint states), the robot at the work_shelf station. MoveIt then refused 3 of 3 park plans (-2), a finger
# through work_shelf_board_0 in the middle of each path.
RUN5_LEFT_HAND = (0.6542, 0.2769, 0.9705)
WORK_SHELF_STATION = (0.0, -2.085, -math.pi / 2.0)


def _shelf_front_in_base(scene):
    from openamrobot_build1_scene import scene as scene_model
    (fx, fy), (nx, ny) = scene_model.work_shelf_front(scene)
    x, y, yaw = WORK_SHELF_STATION
    front = af.rotate_z((fx - x, fy - y, 0.0), -yaw)
    return front, af.rotate_z((nx, ny, 0.0), -yaw)


def _tip_out(hand, quaternion, front, outward):
    """How far the fingertips are in front of the shelf front plane (negative: over a board)."""
    fingers = matrix(*quaternion) @ np.array((0.0, 0.0, -1.0))
    tip = np.array(hand) + af.FINGERTIP_DEPTH_M * fingers
    return float(np.dot(tip[:2] - np.array(front[:2]), np.array(outward[:2])))


@pytest.mark.parametrize("yaw", [0.0, -0.3, 0.3])
def test_the_retract_takes_the_fingertips_out_of_the_shelf_before_the_park_plan(scene, yaw):
    front, outward = _shelf_front_in_base(scene)
    assert np.allclose(front[:2], (0.715, 0.0), atol=1e-6) and np.allclose(outward[:2], (-1.0, 0.0), atol=1e-12)
    q = af.side_grasp(yaw)
    assert _tip_out(RUN5_LEFT_HAND, q, front, outward) < -0.09, "at the reach end the fingertips are over board 0"
    goal = af.retract_out_of_shelf(RUN5_LEFT_HAND, q, front, outward)
    assert goal is not None and goal[2] == RUN5_LEFT_HAND[2], "a straight move out, no height change"
    assert math.isclose(_tip_out(goal, q, front, outward), af.RETRACT_CLEARANCE_M, abs_tol=1e-9)
    assert 0.12 < math.dist(goal, RUN5_LEFT_HAND) < 0.15
    # scn04's withdraw distance would leave the fingertips over the board
    withdrawn, _ = af.grasp_line(RUN5_LEFT_HAND, (0.0, 0.0, 0.0), af.SIDE_GRASP)
    assert _tip_out(withdrawn, af.SIDE_GRASP, front, outward) < 0.0


def test_no_retract_when_the_fingertips_are_already_out(scene):
    front, outward = _shelf_front_in_base(scene)
    assert af.retract_out_of_shelf((0.45, 0.27, 0.97), af.SIDE_GRASP, front, outward) is None


def test_the_return_to_park_retracts_before_it_plans():
    """Static: _back_to_park (scn03 and diag_reach_return) runs the retract after its start mark and before the park plan."""
    text = open(os.path.join(HERE, "..", "openamrobot_build1_scene", "scenarios.py"), encoding="utf-8").read()
    body = text[text.index("def _back_to_park("):text.index("# ---------------------------------------------------------------- 4.")]
    retract, plan = body.index("_retract_from_shelf(rig, log, side, label, scale)"), body.index("_move_group(")
    assert body.index('_mark(rig, f"{label}_start")') < retract < plan
    helper = text[text.index("def _retract_from_shelf("):text.index("def _back_to_park(")]
    assert "retract_out_of_shelf(" in helper and "_cartesian(" in helper and "required=False" in helper
