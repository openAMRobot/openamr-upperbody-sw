# SPDX-License-Identifier: MIT
"""box_fit.py against ideal depth images of a shelf, a tote and an item (synthetic.py). No ROS.

The geometry is that of the Build 1 work shelf seen from the parked robot (base_footprint): shelf front 0.715 m ahead,
level 0 at 0.82 m, the head camera 0.30 m ahead and 1.223 m up, pitched down 30 degrees. It is a fixture, not the scene.
"""
import math
import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from openamrobot_object_perception import box_fit as bf  # noqa: E402
from openamrobot_object_perception import synthetic as syn  # noqa: E402

FRONT, LEVEL, WALL = 0.715, 0.82, 0.005
TOTE, ITEM = (0.40, 0.30, 0.06), (0.035, 0.035, 0.14)
REGION = (0.45, 1.4, -0.9, 0.9)


def turned(xy, angle, shift):
    c, s = math.cos(angle), math.sin(angle)
    return (c * xy[0] - s * xy[1] + shift[0], s * xy[0] + c * xy[1] + shift[1])


def scene(item_xy=(FRONT + 0.035, 0.16), item_yaw=0.0, item_lift=0.0, base_turn=0.0, base_shift=(0.0, 0.0), extra=()):
    """Boxes in base_footprint for a base turned by `base_turn` and shifted by `base_shift` from square parking, and the
    true tote and item poses ((x, y, z), yaw)."""
    def place(centre, size, yaw):
        x, y = turned(centre[:2], base_turn, base_shift)
        return ((x, y, centre[2]), size, yaw + base_turn)
    boxes = [place((FRONT + 0.2, 0.0, LEVEL - 0.01), (0.40, 1.2, 0.02), 0.0),
             place((FRONT + 0.2, 0.0, 1.08), (0.40, 1.2, 0.02), 0.0)]
    for sy in (-1, 1):
        for x in (FRONT + 0.02, FRONT + 0.38):
            boxes.append(place((x, sy * 0.58, 0.535), (0.04, 0.04, 1.07), 0.0))
    tote_xy = turned((FRONT + TOTE[1] / 2.0, 0.0), base_turn, base_shift)
    tote = ((tote_xy[0], tote_xy[1], LEVEL), math.pi / 2 + base_turn)
    boxes += syn.tote_boxes(tote[0], TOTE, WALL, tote[1])
    ixy = turned(item_xy, base_turn, base_shift)
    item = ((ixy[0], ixy[1], LEVEL + WALL + ITEM[2] / 2.0 + item_lift), math.pi / 2 + item_yaw + base_turn)
    boxes.append((item[0], ITEM, item[1]))
    boxes += list(extra)
    return boxes, tote, item


def look(boxes, camera_z=1.223):
    rotation, camera = syn.optical_rotation(0.0, math.radians(30.0)), (0.30, 0.0, camera_z)
    fx, fy, cx, cy = syn.intrinsics(640, 360, 1.57)
    depth = syn.render(boxes, rotation, camera, 640, 360, fx, fy, cx, cy)
    return bf.in_region(bf.depth_points(depth, fx, fy, cx, cy, rotation, camera), REGION), camera, fx


def fit(boxes, camera_z=1.223):
    points, camera, fx = look(boxes, camera_z)
    item, why_item = bf.estimate_item(points, ITEM, LEVEL + WALL, LEVEL + TOTE[2] + 0.008, camera, fx)
    tote, why_tote = bf.estimate_tote(points, TOTE, LEVEL, camera, fx)
    return item, why_item, tote, why_tote


def errors(estimate, truth, size):
    position = math.dist(estimate.position, truth[0])
    yaw = abs(bf.wrap(estimate.yaw - truth[1], bf.symmetry(size)))
    return position, yaw


@pytest.mark.parametrize("item_yaw, base_turn, base_shift, camera_z", [
    (0.0, 0.0, (0.0, 0.0), 1.223),
    (0.03, 0.0, (0.0, 0.0), 1.223),
    (0.0, 0.06, (0.02, -0.03), 1.223),     # the base stopped 0.06 rad and a few cm off the station
    (0.3, -0.05, (0.0, 0.0), 1.223),        # an item turned in the tote
    (0.0, 0.0, (0.0, 0.0), 1.398),          # lift at its middle stop
])
def test_tote_and_item_poses_within_a_millimetre_on_ideal_depth(item_yaw, base_turn, base_shift, camera_z):
    boxes, tote_truth, item_truth = scene(item_yaw=item_yaw, base_turn=base_turn, base_shift=base_shift)
    item, why_item, tote, why_tote = fit(boxes, camera_z)
    assert item is not None, why_item
    assert tote is not None, why_tote
    item_position, item_yaw_error = errors(item, item_truth, ITEM)
    tote_position, tote_yaw_error = errors(tote, tote_truth, TOTE)
    assert item_position < 0.0015 and item_yaw_error < 0.01, (item.position, item_truth, item.yaw, item.detail)
    assert tote_position < 0.0015 and tote_yaw_error < 0.005, (tote.position, tote_truth, tote.yaw, tote.detail)


def test_the_item_centre_is_half_a_side_behind_its_face_not_on_it():
    """The visible face is the front of the box: the centre is depth / 2 behind it, away from the camera."""
    boxes, _tote, item_truth = scene()
    item = fit(boxes)[0]
    assert abs(item.position[0] - item_truth[0][0]) < 0.001
    assert item.position[0] > FRONT + 0.0175 + 0.01      # the face is at FRONT + 0.0175


def test_a_lifted_or_missing_item_is_not_reported():
    boxes, _tote, _item = scene(item_lift=0.08)
    item, why, tote, _ = fit(boxes)
    assert item is None and "expected" in why
    assert tote is not None
    boxes, _tote, _item = scene()
    item, why, _tote_estimate, _ = fit(boxes[:-1])
    assert item is None


def test_a_finger_near_the_item_gives_the_item_or_nothing_never_the_finger():
    """A thin box at the grasp height (a finger) 0.012 m beside the item merges with it into one group too wide for the
    item: nothing is reported. 0.06 m away it is its own group of another height, and the item is found."""
    for gap, found in ((0.012, False), (0.06, True)):
        finger = ((FRONT + 0.035, 0.16 + 0.0175 + gap + 0.01, 0.93), (0.012, 0.02, 0.05), 0.0)
        boxes, _tote, item_truth = scene(extra=[finger])
        item, why, _tote_estimate, _ = fit(boxes)
        assert (item is not None) == found, (gap, why)
        if item is not None:
            assert errors(item, item_truth, ITEM)[0] < 0.0015


def test_covariance_is_symmetric_with_unestimated_roll_and_pitch():
    boxes, _tote, _item = scene()
    item = fit(boxes)[0]
    cov = np.array(item.covariance).reshape(6, 6)
    assert np.allclose(cov, cov.T)
    assert cov[3, 3] == cov[4, 4] == bf.UNESTIMATED_VARIANCE
    assert 0.0 < cov[0, 0] < 1e-5 and 0.0 < cov[5, 5] < 1e-3


def test_ransac_line_and_clusters():
    rng = np.random.default_rng(3)
    t = rng.uniform(-0.2, 0.2, 300)
    line = np.stack([0.7 + 0.0 * t, t], axis=1)
    noise = rng.uniform(-1, 1, (100, 2))
    normal, c, inliers = bf.ransac_line(np.concatenate([line, noise]), 0.001)
    assert abs(abs(normal[0]) - 1.0) < 1e-6 and abs(abs(c) - 0.7) < 1e-6 and inliers[:300].all()
    labels = bf.grid_clusters(np.array([[0.0, 0.0], [0.005, 0.0], [0.5, 0.5]]), 0.01)
    assert labels[0] == labels[1] != labels[2]


def test_depth_points_puts_the_optical_axis_where_the_camera_looks():
    rotation = syn.optical_rotation(0.0, math.radians(30.0))
    depth = np.full((3, 3), 1.0)
    points = bf.depth_points(depth, 1.0, 1.0, 1.0, 1.0, rotation, (0.3, 0.0, 1.2))
    centre = points[4]
    assert np.allclose(centre, (0.3 + math.cos(math.radians(30.0)), 0.0, 1.2 - math.sin(math.radians(30.0))))


# --- the item is looked for inside the tote (review of 25 Nov 2026) -------------------------------------------------------
#
# On ideal depth, estimate_item without a container returned a shelf post (0.04 m square at y = +0.56 / -0.58 m) as the
# 0.035 m item in 9 of 315 parking poses, 0.42 to 0.74 m from the truth. These are four of those poses and four ordinary ones.

POST_POSES = [(0.03, 0.04, (0.02, -0.03)), (0.0, 0.04, (0.02, -0.03)), (0.3, -0.06, (0.0, 0.03)), (0.3, -0.02, (0.02, 0.0))]
ORDINARY_POSES = [(0.0, 0.0, (0.0, 0.0)), (0.03, 0.0, (0.0, 0.0)), (0.0, 0.06, (0.02, -0.03)), (0.3, -0.05, (0.0, 0.0))]


def fit_inside_tote(boxes):
    points, camera, fx = look(boxes)
    tote, _ = bf.estimate_tote(points, TOTE, LEVEL, camera, fx)
    if tote is None:
        return None, "no tote estimate"
    return bf.estimate_item(points, ITEM, LEVEL + WALL, LEVEL + TOTE[2] + 0.008, camera, fx,
                            container=(tote.position[:2], TOTE[:2], tote.yaw))


@pytest.mark.parametrize("item_yaw, base_turn, base_shift", POST_POSES + ORDINARY_POSES)
def test_an_item_estimate_is_never_a_shelf_post(item_yaw, base_turn, base_shift):
    boxes, _tote, item = scene(item_yaw=item_yaw, base_turn=base_turn, base_shift=base_shift)
    estimate, why = fit_inside_tote(boxes)
    if estimate is None:
        assert "outside the tote" in why or "no group fits" in why, why      # nothing is better than a wrong pose
        return
    position, _yaw = errors(estimate, item, ITEM)
    assert position < 0.005, (position, estimate.position, item[0], why)


@pytest.mark.parametrize("item_yaw, base_turn, base_shift", ORDINARY_POSES)
def test_the_container_does_not_hide_an_item_that_is_found_without_it(item_yaw, base_turn, base_shift):
    boxes, _tote, item = scene(item_yaw=item_yaw, base_turn=base_turn, base_shift=base_shift)
    estimate, why = fit_inside_tote(boxes)
    assert estimate is not None, why
    assert errors(estimate, item, ITEM)[0] < 0.0015


def test_inside_footprint_follows_the_yaw():
    assert bf.inside_footprint((1.0, 0.19), (1.0, 0.0), (0.30, 0.40), 0.0)
    assert not bf.inside_footprint((1.0, 0.19), (1.0, 0.0), (0.30, 0.40), math.pi / 2)
    assert bf.inside_footprint((1.0, 0.19), (1.0, 0.0), (0.30, 0.40), math.pi / 2, margin=0.05)
