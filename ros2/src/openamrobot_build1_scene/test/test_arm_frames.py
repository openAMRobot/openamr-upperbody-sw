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
    assert "_, hand = grasp_line((tcp_x, tcp_y, grasp_z), found[0], held_orientation)" in body
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
    assert "grasp_line((tcp_x, tcp_y, grasp_z + REACH_CLEARANCE_M), found[0], held_orientation)" in body
    assert '_cartesian(rig, log, side, link, [pre_grasp, above, hand], held_orientation, "grasp_reach", 0.1)' in body


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
    from openamrobot_build1_scene.planning_scene import MOVABLE_OBJECTS, OBJECTS_DECLARATION
    assert set(MOVABLE_OBJECTS) == {"tote", "item", "payload"} and "observed" in OBJECTS_DECLARATION
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert 'f"/build1/ground_truth/{name}/pose"' in loader
    assert "object_box_in_base(pose, position, quaternion, self.object_offset[name])" in loader
    assert "object_moved(self.object_sent.get(name), pose_now)" in loader
    assert "self.object_sent = dict(self.object_pose)" in loader
    assert "self.get_logger().info(OBJECTS_DECLARATION)" in loader
    # The same base pose for objects and fixtures: the declared source, never a separate truth path for the objects.
    assert loader.count("object_box_in_base(") == 1
