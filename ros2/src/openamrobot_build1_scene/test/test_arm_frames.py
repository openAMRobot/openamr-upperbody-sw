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
    assert "pre_grasp, hand = grasp_line((item[0], item[1], grasp_z), found[0], held_orientation)" in body
    assert '_cartesian(rig, log, side, link, [pre_grasp, hand], held_orientation, "grasp_reach", 0.1)' in body
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


def test_the_grasp_height_and_the_approach_tolerance_keep_the_hand_above_the_tote_wall(scene):
    """The reach keeps the orientation the approach ended with. A tilt of the hand inside the approach tolerance (up to
    sqrt(3) times the per-axis value) lowers the front of the hand body, 0.11 m along the fingers, by 0.11 sin(tilt); the
    executed hand leaves its plan by up to 0.007 m. Mesh extents from openarm_description at the pins (arm_frames.py)."""
    from openamrobot_build1_scene import scene as scene_model
    boxes = {name: (size, centre) for name, size, centre, _ in scene_model.collision_boxes(scene)}
    tote, item = boxes["tote"], boxes["item"]
    wall_top = tote[1][2] + tote[0][2] / 2.0
    item_top = item[1][2] + item[0][2] / 2.0

    def lowest_point(above_centre, tolerance):
        tilt = math.sqrt(3.0) * tolerance
        return item[1][2] + above_centre - af.HAND_BODY_BELOW_TCP_M - af.HAND_BODY_REACH_ALONG_FINGERS_M * math.sin(tilt) - 0.007

    assert lowest_point(af.GRASP_ABOVE_ITEM_CENTRE_M, af.APPROACH_ANGLE_TOLERANCE_RAD) > wall_top
    assert lowest_point(0.03, 0.3) < wall_top, "the old 0.03 m and 0.3 rad let the hand reach the wall"
    assert lowest_point(0.03, af.APPROACH_ANGLE_TOLERANCE_RAD) < wall_top, "0.03 m alone was not enough"
    # The fingers still close on the item: their lower edge is below the item's top.
    assert item[1][2] + af.GRASP_ABOVE_ITEM_CENTRE_M - af.FINGER_HALF_HEIGHT_M < item_top - 0.04
    body = _grasp_body()
    assert "angle_tolerance=APPROACH_ANGLE_TOLERANCE_RAD" in body
    assert "grasp_z = item[2] + GRASP_ABOVE_ITEM_CENTRE_M" in body


def test_the_fixed_boxes_get_the_planning_clearance_and_the_objects_do_not(scene):
    from openamrobot_build1_scene import scene as scene_model
    from openamrobot_build1_scene.planning_scene import FIXTURE_PADDING_M, padded
    boxes = scene_model.collision_boxes(scene)
    fixed = {name for name, _s, _c, _y in scene_model.static_boxes(scene)}
    out = {name: (size, centre) for name, size, centre, _yaw in padded(boxes, fixed, FIXTURE_PADDING_M)}
    original = {name: (size, centre) for name, size, centre, _yaw in boxes}
    assert "work_shelf_board_0" in fixed and {"tote", "item", "payload"} <= set(out) and not {"tote", "item", "payload"} & fixed
    for name in fixed:
        assert np.allclose(out[name][0], np.array(original[name][0]) + 2 * FIXTURE_PADDING_M) and out[name][1] == original[name][1]
    for name in ("tote", "item", "payload"):
        assert out[name] == (tuple(original[name][0]), original[name][1])
    # Larger than the measured contact-free deviation: 0.007 m at the hand base, 0.004 m more at the fingertips.
    assert FIXTURE_PADDING_M >= 0.007 + 0.03 * 0.138
    # The grasp and the lift stay possible: the padded board 0 top stays below the fingers at the grasp height (item
    # centre + 0.03 m, fingers about 0.015 m below the TCP), and the padded board 1 bottom above the lifted hand (hand
    # body about 0.04 m above the TCP, estimated). 0.04 m failed this: the lift was refused in CI.
    board0, board1, item = original["work_shelf_board_0"], original["work_shelf_board_1"], original["item"]
    assert board0[1][2] + board0[0][2] / 2.0 + FIXTURE_PADDING_M < item[1][2] + 0.03 - 0.015
    assert board1[1][2] - board1[0][2] / 2.0 - FIXTURE_PADDING_M > item[1][2] + 0.03 + 0.08 + 0.04
    assert not board1[1][2] - board1[0][2] / 2.0 - 0.04 > item[1][2] + 0.03 + 0.08 + 0.04
    with open(os.path.join(HERE, "..", "scripts", "planning_scene_loader.py"), encoding="utf-8") as f:
        loader = f.read()
    assert 'declare_parameter("fixture_padding_m", FIXTURE_PADDING_M)' in loader
    assert "self.boxes = padded(scene_model.collision_boxes(self.scene), fixed, padding)" in loader
