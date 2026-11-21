# SPDX-License-Identifier: MIT
"""config/scene.yaml: every number says where it comes from, and the layout is what the scenarios assume."""
import math

from openamrobot_build1_scene import scene as scene_model


def test_every_quantity_has_a_status_and_a_source(scene):
    quantities = scene_model.quantities(scene)
    assert len(quantities) >= 20
    for path, entry in quantities:
        assert entry["status"] in scene_model.STATUSES, path
        if "source" in entry:
            assert entry["source"].strip(), path
        if entry["status"] == "sourced":
            assert entry.get("source", "").strip(), f"{path} is sourced and names no source"


def test_floor_friction_is_marked_estimated(scene):
    assert scene["floor"]["friction"]["mu"]["status"] == "estimated"


def test_the_baseline_gives_no_aisle_or_shelf_size_so_they_are_estimated(scene):
    for entry in (scene["aisle"]["width"], scene["aisle"]["length"], scene["shelf"]["depth"], scene["shelf"]["unit_length"],
                  scene["tote"]["size"], scene["item"]["size"]):
        assert entry["status"] == "estimated"


def test_shelf_levels_are_sourced_and_inside_the_work_span(scene):
    low, high = scene["shelf"]["work_span"]["value"]
    for level in scene["shelf"]["levels"]:
        assert level["status"] == "sourced" and "UC1" in level["source"]
        assert low <= level["value"] <= high


def test_the_aisle_is_as_wide_as_the_file_says(scene):
    poses = scene_model.shelf_poses(scene)
    depth = scene["shelf"]["depth"]["value"]
    gap = abs(poses["shelf_a0"][1] - poses["shelf_b0"][1]) - depth
    assert gap == scene["aisle"]["width"]["value"]
    assert gap > scene["robot_envelope"]["width"]["value"]
    assert abs(angle := poses["shelf_a0"][2] - poses["shelf_b0"][2]) == math.pi, angle    # the two rows face each other


def test_work_shelf_station_parks_the_footprint_at_the_sourced_gap(scene):
    x, y, yaw = scene_model.station(scene, "work_shelf")
    (fx, fy), (nx, ny) = scene_model.work_shelf_front(scene)
    front_x, front_y = x + scene_model.FOOTPRINT_FRONT_X * math.cos(yaw), y + scene_model.FOOTPRINT_FRONT_X * math.sin(yaw)
    gap = (front_x - fx) * nx + (front_y - fy) * ny
    assert abs(gap - scene["work_shelf"]["parking_gap"]["value"]) < 1e-9
    assert scene["work_shelf"]["parking_gap"]["status"] == "sourced"
    assert abs(math.cos(yaw) * nx + math.sin(yaw) * ny + 1.0) < 1e-9     # the robot faces the shelf


def test_origin_station_has_three_metres_of_free_floor_ahead(scene):
    x, y, yaw = scene_model.station(scene, "origin")
    half_width = scene["robot_envelope"]["width"]["value"] / 2.0 + 0.25
    for name, size, centre, _yaw in scene_model.static_boxes(scene):
        ahead = (centre[0] - x) * math.cos(yaw) + (centre[1] - y) * math.sin(yaw)
        side = abs(-(centre[0] - x) * math.sin(yaw) + (centre[1] - y) * math.cos(yaw))
        reach = max(size[0], size[1]) / 2.0
        assert not (-0.5 < ahead < 3.5 + reach and side < half_width + reach), f"{name} is in the drive lane"


def test_tote_and_item_stand_on_the_work_shelf(scene):
    objects = scene_model.movable_objects(scene)
    level = scene["shelf"]["levels"][scene["tote"]["level"]]["value"]
    tote, item = objects["tote"], objects["item"]
    assert tote["pose"][2] == level
    wall = scene["tote"]["wall"]["value"]
    assert abs(item["pose"][2] - (level + wall + item["size"][2] / 2.0)) < 1e-9
    assert abs(item["pose"][0] - tote["pose"][0]) < tote["size"][0] / 2.0 - item["size"][0]
    assert abs(item["pose"][1] - tote["pose"][1]) < tote["size"][1] / 2.0 - item["size"][1] / 2.0
    assert item["size"][2] > scene["tote"]["size"]["value"][2] + 0.05, "the item must stand above the tote wall for a side grasp"
    assert item["mass"] <= 1.0 and objects["payload"]["mass"] <= 1.0      # UC1-19


def test_reach_targets_are_above_the_boards_on_the_robot_side(scene):
    targets = scene_model.shelf_reach_targets(scene)
    (fx, fy), (nx, ny) = scene_model.work_shelf_front(scene)
    for index, level in enumerate(scene["shelf"]["levels"]):
        for side in ("left", "right"):
            x, y, z = targets[index][side]
            assert z > level["value"] + 0.1
            assert (x - fx) * nx + (y - fy) * ny > 0.0, "the target must be in front of the shelf, not inside it"
    assert targets[0]["left"][0] > targets[0]["right"][0], "the robot faces -y here, so its left is +x"
