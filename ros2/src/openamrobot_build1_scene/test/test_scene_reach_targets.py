# SPDX-License-Identifier: MIT
"""scn03's level 0 reach targets against scn04's item in the tote (scene.shelf_reach_targets). No ROS import."""
import math
import os

import numpy as np
import pytest

from openamrobot_build1_scene import scene as scene_model

# The left hand where scn03's level 0 reach left it, from the arm diagnostics (diag_reach_return, spawned at the work_shelf
# station, lift at stop 0): the physics joint state at the end of L0_left. The target then: (0.16, -2.75, 0.98) in the world.
STATION = (0.0, -2.085, -math.pi / 2.0)
L0_LEFT_RUNS = {
    # run 3 of 44609269748 (e7a251c): palm on the item at 33.6 N, hand base z 1.0026, return refused (-10, work_shelf_board_1)
    "44609269748-run3": [-1.4595, -0.0791, 0.492, 0.2686, -0.2061, 0.0704, 0.0813],
    # run 5 of 44609269748: palm on the item at 8.81 N
    "44609269748-run5": [-1.3989, -0.1124, 0.4434, 0.3088, -0.5593, 0.1331, 0.0549],
    # run 5 of 44480277987 (323a2ce): palm on the item at 32.3 N
    "44480277987-run5": [-1.1908, -0.0483, 0.127, 0.6934, -0.1058, -0.1357, 0.1207],
}
OLD_LEFT_TARGET_WORLD = (0.16, -2.75, 0.98)
HAND_LINKS = ("openarm_left_ee_base_link", "openarm_left_ee_link1", "openarm_left_ee_link2")
POSITION_TOLERANCE_M = 0.02       # scn03's pose goal (scenarios._reach)
PATH_DEVIATION_M = 0.009          # the executed hand off its plan (arm_frames.REACH_PATH_DEVIATION_M on the scn04 branch)


def _stl_triangles(path):
    with open(path, "rb") as f:
        data = f.read()
    count = int(np.frombuffer(data, dtype="<u4", count=1, offset=80)[0])
    records = np.frombuffer(data, dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]), count=count, offset=84)
    return records["v"].astype(float)


def _mesh_points(urdf, link):
    """Points on the collision mesh of `link` in its own frame: vertices and a barycentric grid on every triangle."""
    import xml.etree.ElementTree as ET
    from ament_index_python.packages import get_package_share_directory
    collision = ET.fromstring(urdf).find(f"link[@name='{link}']/collision")
    origin, mesh = collision.find("origin"), collision.find("geometry/mesh")
    package, relative = mesh.get("filename")[len("package://"):].split("/", 1)
    triangles = _stl_triangles(os.path.join(get_package_share_directory(package), relative))
    triangles = triangles * np.array([float(v) for v in (mesh.get("scale") or "1 1 1").split()])
    assert [float(v) for v in (origin.get("rpy") or "0 0 0").split()] == [0.0, 0.0, 0.0]
    weights = np.array([(i, j, 6 - i - j) for i in range(7) for j in range(7 - i)], dtype=float) / 6.0
    points = np.einsum("wk,tkd->twd", weights, triangles).reshape(-1, 3)
    return points + np.array([float(v) for v in origin.get("xyz").split()])


def _box_clearance(points, centre, half):
    """Smallest distance of `points` to an axis-aligned box; minus the depth if a point is inside."""
    d = np.abs(points - centre)
    inside = np.all(d < half, axis=1)
    if inside.any():
        return -float(np.min((half - d[inside]).min(axis=1)))
    return float(np.linalg.norm(np.maximum(d - half, 0.0), axis=1).min())


def _in_base(point):
    x, y, yaw = STATION
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = point[0] - x, point[1] - y
    return np.array((c * dx - s * dy, s * dx + c * dy, point[2]))


def test_the_level_0_targets_go_to_the_side_and_level_1_keeps_its_targets(scene):
    targets = scene_model.shelf_reach_targets(scene)
    left, right = (_in_base(targets[0][side]) for side in ("left", "right"))
    assert np.allclose(left, (0.665, scene_model.LEVEL0_REACH_SIDE_M, 0.98), atol=1e-6)
    assert np.allclose(right, (0.665, -scene_model.LEVEL0_REACH_SIDE_M, 0.98), atol=1e-6)
    assert np.allclose(_in_base(targets[1]["left"]), (0.665, 0.16, 1.25), atol=1e-6)
    assert np.allclose(_in_base(targets[1]["right"]), (0.665, -0.16, 1.25), atol=1e-6)
    tote = scene_model.movable_objects(scene)["tote"]
    assert scene_model.LEVEL0_REACH_SIDE_M > abs(tote["pose"][0]) + tote["size"][0] / 2.0, "beside the tote, not over it"


@pytest.mark.parametrize("run", sorted(L0_LEFT_RUNS))
def test_the_left_hand_at_the_level_0_target_stays_off_the_tote_item(devices_urdf, scene, run):
    """The recorded end states of the left level 0 reach, moved with the target, keep the palm and the fingers (as the
    scenario leaves them, not commanded: 0 rad) POSITION_TOLERANCE_M + PATH_DEVIATION_M or more from scn04's item.
    At the old target, 0.16 m to the side, these states put the hand 0.9 to 10.6 mm from it; at 0.27 m, 48.6 to 63.4 mm."""
    from openamrobot_build1_scene.gravity import ARM_JOINTS, GravityModel
    model = GravityModel(devices_urdf)
    links, _ = model._frames({**dict(zip(ARM_JOINTS["left"], L0_LEFT_RUNS[run])), "lift_joint": 0.0})
    R0, p0 = links.get("base_footprint", (np.eye(3), np.zeros(3)))
    points = np.vstack([(R0.T @ links[link][0] @ _mesh_points(devices_urdf, link).T).T + R0.T @ (links[link][1] - p0)
                        for link in HAND_LINKS])
    item = scene_model.movable_objects(scene)["item"]
    assert item["pose"][3] == 0.0 and STATION[2] == -math.pi / 2.0
    centre = _in_base(item["pose"][:3])
    half = np.array((item["size"][1], item["size"][0], item["size"][2])) / 2.0      # world x, y swap in base_footprint
    target = _in_base(scene_model.shelf_reach_targets(scene)[0]["left"])
    moved = points + (target - _in_base(OLD_LEFT_TARGET_WORLD))
    before = _box_clearance(points, centre, half)
    after = _box_clearance(moved, centre, half)
    assert before < POSITION_TOLERANCE_M + PATH_DEVIATION_M, "the recorded state was close to the item"
    assert after >= POSITION_TOLERANCE_M + PATH_DEVIATION_M, f"the hand is {after * 1000:.1f} mm from the item"
