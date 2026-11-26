# SPDX-License-Identifier: MIT
"""The generated Gazebo world and Nav2 map."""
import os
import xml.etree.ElementTree as ET

import pytest
import yaml

from openamrobot_build1_scene import scene as scene_model


@pytest.mark.parametrize("profile, step", [("fine", "0.00066667"), ("fast", "0.001")])
def test_physics_profiles(scene, profile, step):
    world = ET.fromstring(scene_model.world_sdf(scene, profile)).find("world")
    physics = world.find("physics")
    assert physics.findtext("max_step_size") == step
    assert float(physics.findtext("real_time_factor")) == 1.0
    assert abs(1.0 / float(step) - {"fine": 1500.0, "fast": 1000.0}[profile]) < 0.02


def test_fine_is_the_default_physics_step(scene):
    assert scene["physics_profiles"]["fine"].get("default") is True


def test_world_has_floor_shelves_dock_tote_item_payload_and_obstacles(scene):
    world = ET.fromstring(scene_model.world_sdf(scene, "fine")).find("world")
    models = {model.get("name") for model in world.findall("model")}
    assert {"floor", "fixtures", "tote", "item", "payload"} <= models
    include = world.find("include")
    assert include.findtext("uri") == "model://apriltag_dock" and include.findtext("name") == "apriltag_dock"
    assert include.findtext("pose").split()[:3] == ["6.5", "0.0", "0.3"]            # the dock pose is fixed: repeatable
    fixtures = [c.get("name") for c in world.find("model[@name='fixtures']").iter("collision")]
    assert sum("shelf_" in name for name in fixtures) == 7 * 6                      # seven units, two boards and four posts
    assert sum("obstacle_" in name for name in fixtures) == 3
    floor_mu = world.find("model[@name='floor']/link/collision/surface/friction/ode/mu").text
    assert float(floor_mu) == scene["floor"]["friction"]["mu"]["value"]


def test_free_bodies_report_their_ground_truth_pose(scene):
    world = ET.fromstring(scene_model.world_sdf(scene, "fast")).find("world")
    for name in ("tote", "item", "payload"):
        model = world.find(f"model[@name='{name}']")
        assert model.find("static") is None
        assert model.find("plugin").get("filename") == "gz-sim-pose-publisher-system"
        assert float(model.findtext("link/inertial/mass")) > 0.0


def test_sensor_system_is_optional(scene):
    assert "gz-sim-sensors-system" in scene_model.world_sdf(scene, "fast", sensors=True)
    assert "gz-sim-sensors-system" not in scene_model.world_sdf(scene, "fast", sensors=False)


def test_map_marks_what_the_lidar_plane_cuts(scene, tmp_path):
    path = scene_model.write_map(scene, str(tmp_path))
    with open(path) as f:
        meta = yaml.safe_load(f)
    grid, resolution, origin = scene_model.occupancy_grid(scene)
    assert meta["resolution"] == resolution == 0.05 and meta["origin"][:2] == list(origin)
    assert os.path.getsize(os.path.join(str(tmp_path), meta["image"])) > len(grid) * len(grid[0])

    def cell(x, y):
        row = len(grid) - 1 - int((y - origin[1]) / resolution)
        return grid[row][int((x - origin[0]) / resolution)]
    for name in ("origin", "work_shelf", "aisle_entry"):
        x, y, _ = scene_model.station(scene, name)
        assert cell(x, y) == 0, f"station {name} is not free in the map"
    posts = [box for box in scene_model.static_boxes(scene) if "_post_" in box[0]]
    assert len(posts) == 28 and all(cell(c[0], c[1]) == 100 for _n, _s, c, _y in posts)
    boards = [box for box in scene_model.static_boxes(scene) if box[0] == "work_shelf_board_0"]
    assert cell(boards[0][2][0], boards[0][2][1]) == 0, "a board at 0.82 m is above the scan plane and must not be in the map"
    assert cell(1.5, 1.2) == 100 and cell(2.5, -1.5) == 100                      # the post and box a
    aisle_x, aisle_y, _ = scene["aisle"]["centre"]
    assert all(cell(aisle_x + d, aisle_y) == 0 for d in (0.2, 1.0, 2.0, 3.0)), "the aisle centre line must be free"


def test_collision_boxes_cover_fixtures_and_objects(scene):
    names = {name for name, _s, _c, _y in scene_model.collision_boxes(scene)}
    assert {"tote", "item", "payload", "work_shelf_board_0", "obstacle_box_a"} <= names
    tote = next(box for box in scene_model.collision_boxes(scene) if box[0] == "tote")
    assert abs(tote[2][2] - (0.82 + 0.03)) < 1e-9          # the tote box is centred on its height
