# SPDX-License-Identifier: MIT
"""The grocery dark store demonstrator: scene, map, stations, the pick geometry kept from Baseline 1, the place geometry,
the camera path and the phase reports. No ROS, no simulator."""
import json
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import PACKAGE_DIR, package_path
from openamrobot_build1_scene import grocery_phases, grocery_scene
from openamrobot_build1_scene import scene as scene_model


@pytest.fixture(scope="module")
def grocery():
    return scene_model.load(package_path("config", "grocery_scene.yaml"))


@pytest.fixture(scope="module")
def build1():
    return scene_model.load(package_path("config", "scene.yaml"))


def test_every_number_says_where_it_comes_from(grocery):
    found = scene_model.quantities(grocery)
    assert len(found) > 30
    for path, entry in found:
        assert entry["status"] in scene_model.STATUSES, path
        assert str(entry.get("source", "")).strip(), path


def test_the_build1_scene_is_unchanged_by_the_extension(build1):
    """scene.yaml has no grocery section, so scene.extension adds nothing to its world, map or planning scene."""
    assert scene_model.extension(build1) is None
    assert not any(name.startswith(("row_", "packing_", "order_tote", "wall_", "work_shelf_deco"))
                   for name, _s, _c, _y in scene_model.static_boxes(build1))
    assert "store_decor" not in scene_model.world_sdf(build1, "fast")


def test_pick_geometry_is_baseline1_relative_to_the_station(grocery, build1):
    """The work shelf, the tray, the target product and the payload stand where Baseline 1 has them, seen from the
    work-shelf station: the scn04 grasp and its limits apply unchanged."""
    for key in ("shelf", "tote", "item", "payload", "robot_envelope"):
        if key == "shelf":
            for sub in ("unit_length", "depth", "board_thickness", "post", "levels", "work_span"):
                assert grocery[key][sub] == build1[key][sub], sub
        else:
            assert grocery[key] == build1[key], key
    assert grocery["work_shelf"]["parking_gap"] == build1["work_shelf"]["parking_gap"]

    def relative(scene):
        sx, sy, syaw = scene_model.station(scene, "work_shelf")
        out = {}
        for name, body in scene_model.movable_objects(scene).items():
            x, y, z, yaw = body["pose"]
            c, s = math.cos(-syaw), math.sin(-syaw)
            out[name] = (c * (x - sx) - s * (y - sy), s * (x - sx) + c * (y - sy), z, yaw - syaw)
        return out
    a, b = relative(grocery), relative(build1)
    for name in a:
        assert all(abs(u - v) < 1e-9 for u, v in zip(a[name], b[name])), name


def test_stations_are_free_and_the_aisles_are_open(grocery):
    grid, resolution, origin = scene_model.occupancy_grid(grocery)

    def cell(x, y):
        return grid[len(grid) - 1 - int((y - origin[1]) / resolution)][int((x - origin[0]) / resolution)]
    for name in ("origin", "work_shelf", "aisle_entry", "packing"):
        x, y, _ = grocery_scene.station(grocery, name)
        assert cell(x, y) == 0, name
    assert all(cell(x, -1.3) == 0 for x in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)), "aisle 1 centre line"
    assert all(cell(x, 1.7) == 0 for x in (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)), "aisle 2 centre line"
    # The plinths close the row faces in the LiDAR plane; the work shelf stays open below level 0 as in Baseline 1.
    assert cell(2.2, -2.5) == 100 and cell(4.6, -2.5) == 100
    assert cell(3.4, -2.6) == 0
    assert cell(-6.93, 0.0) == 100 and cell(6.93, 0.0) == 100 and cell(0.0, 4.93) == 100, "walls"


def test_committed_map_equals_a_fresh_generation(grocery, tmp_path):
    scene_model.write_map(grocery, str(tmp_path), name="grocery_dark_store")
    for name in ("grocery_dark_store.pgm", "grocery_dark_store.yaml"):
        with open(os.path.join(str(tmp_path), name), "rb") as fresh, open(package_path("config", "maps", name), "rb") as kept:
            assert fresh.read() == kept.read(), f"{name}: run scripts/grocery_map.py"


def test_world_parses_and_keeps_the_ground_truth_objects(grocery):
    world = ET.fromstring(scene_model.world_sdf(grocery, "fast")).find("world")
    assert world.get("name") == "build1"
    models = {model.get("name") for model in world.findall("model")}
    assert {"floor", "fixtures", "tote", "item", "payload", "store_decor"} <= models
    decor = world.find("model[@name='store_decor']")
    assert decor.find("static").text == "true" and decor.find("link/collision") is None, "products are visual only"
    assert len(decor.findall("link/visual")) == len(grocery_scene.products(grocery)) + len(grocery_scene.markings(grocery))
    assert 200 < len(grocery_scene.products(grocery)) < 600
    fixtures = {c.get("name") for c in world.find("model[@name='fixtures']").iter("collision")}
    assert {"packing_bench_top_collision", "order_tote_floor_collision", "row_a_0_plinth_collision"} <= fixtures


def test_no_product_where_the_arm_works_or_on_the_payload(grocery):
    shelf = tuple(grocery["work_shelf"]["pose"])
    level0 = scene_model.value(grocery["shelf"]["levels"][0])
    px, py, pz, _ = scene_model.movable_objects(grocery)["payload"]["pose"]
    for name, size, centre, _yaw, _c in grocery_scene.products(grocery):
        if not name.startswith("product_work_shelf_"):
            continue
        assert abs(centre[2] - size[2] / 2.0 - level0) > 1e-6, name
        overlap_x = abs(centre[0] - px) < (size[0] + 0.20) / 2.0
        overlap_z = abs(centre[2] - pz) < (size[2] + 0.10) / 2.0
        assert not (overlap_x and overlap_z), name
    assert shelf


def test_order_tote_is_under_the_carried_product(grocery):
    """The product arrives over the middle of the order tote, with room for the station error of Baseline 1's runs
    (up to 0.08 m), and the tote stands on the bench."""
    tote = grocery_scene.order_tote(grocery)
    item = scene_model.movable_objects(grocery)["item"]
    along, deep = grocery_scene.in_tote(grocery, grocery_scene.drop_point(grocery))
    assert abs(along) < 1e-9 and abs(deep) < 1e-9
    half = scene_model.value(grocery["item"]["size"])[0] / 2.0
    assert tote["inner_half"][0] - half > 0.08 and tote["inner_half"][1] - half > 0.08
    bench = grocery["grocery"]["packing_bench"]
    length, depth = scene_model.value(bench["size"])
    bx, by, _ = grocery_scene.bench_pose(grocery)
    cx, cy = tote["centre"]
    assert abs(cx - bx) + tote["size"][0] / 2.0 <= length / 2.0 and abs(cy - by) + tote["size"][1] / 2.0 <= depth / 2.0
    # The held product (lifted 0.08 m at the work shelf) clears the order tote wall on the way in; the tote wall is
    # lower than the tray wall the hand cleared at the pick, and the place lowers about what the lift raised.
    tray_wall_top = scene_model.value(grocery["shelf"]["levels"][0]) + scene_model.value(grocery["tote"]["size"])[2]
    carried_bottom = item["pose"][2] - scene_model.value(grocery["item"]["size"])[2] / 2.0 + 0.08
    assert carried_bottom - tote["wall_top"] > 0.04
    assert tote["wall_top"] < tray_wall_top
    lower = carried_bottom - (tote["floor_top"] + 0.02)
    assert 0.06 < lower < 0.10


def test_stations_face_their_fixtures_and_the_lift_stays_at_l1000(grocery):
    pick = grocery_scene.station(grocery, "work_shelf")
    pack = grocery_scene.station(grocery, "packing")
    assert abs(pick[2] - pack[2]) < 1e-9, "both stations face their fixture the same way"
    (fx, fy), _n = grocery_scene.bench_front(grocery)
    gap = math.hypot(pack[0] - fx, pack[1] - fy) - scene_model.FOOTPRINT_FRONT_X
    assert abs(gap - scene_model.value(grocery["grocery"]["packing_bench"]["packing_gap"])) < 1e-9
    rig = open(package_path("openamrobot_build1_scene", "rig.py"), encoding="utf-8").read()
    assert "LIFT_STOP_M = (0.0, 0.175, 0.350)" in rig, "L1000, L1175, L1350"
    level0 = scene_model.value(grocery["shelf"]["levels"][0])
    bench_top = scene_model.value(grocery["grocery"]["packing_bench"]["height"])
    assert 0.0 < level0 - bench_top <= 0.05, "the place happens at L1000, the stop of shelf level 0"


def test_packing_approach_turns_clear_of_the_bench_legs(grocery):
    """With the product held the Nav2 footprint reaches 0.79 m ahead (run 44227129411); the robot turns 0.60 m back from
    the packing station (grocery_demo.APPROACH_M), where every bench leg is beyond that sweep, and drives straight in."""
    text = open(package_path("openamrobot_build1_scene", "grocery_demo.py"), encoding="utf-8").read()
    assert "APPROACH_M = 0.60" in text
    x, y, yaw = grocery_scene.station(grocery, "packing")
    ax, ay = x - 0.60 * math.cos(yaw), y - 0.60 * math.sin(yaw)
    legs = [c for name, _s, c, _y in scene_model.static_boxes(grocery) if name.startswith("packing_bench_leg_")]
    sweep = math.hypot(0.794, 0.333) + 0.02
    assert len(legs) == 4
    assert min(math.hypot(c[0] - ax, c[1] - ay) for c in legs) > sweep + 0.3
    assert min(math.hypot(c[0] - x, c[1] - y) for c in legs) < sweep, "a turn at the station itself would sweep a leg"
    grid, resolution, origin = scene_model.occupancy_grid(grocery)
    assert grid[len(grid) - 1 - int((ay - origin[1]) / resolution)][int((ax - origin[0]) / resolution)] == 0


def test_bollards_mark_the_row_ends_in_the_map(grocery):
    grid, resolution, origin = scene_model.occupancy_grid(grocery)
    bollards = [box for box in scene_model.static_boxes(grocery) if box[0].startswith("bollard_")]
    assert len(bollards) == 6
    for _name, _size, (x, y, _z), _yaw in bollards:
        assert grid[len(grid) - 1 - int((y - origin[1]) / resolution)][int((x - origin[0]) / resolution)] == 100


def test_camera_path_looks_at_its_target():
    from openamrobot_build1_scene import grocery_record as rec
    for eye, target in (rec.OVERVIEW, ((0.0, 0.0, 3.0), (2.0, 0.0, 0.5)), ((1.0, 1.0, 2.0), (0.0, 2.0, 0.9))):
        x, y, z, qx, qy, qz, qw = rec.look_pose(eye, target)
        assert abs(qx * qx + qy * qy + qz * qz + qw * qw - 1.0) < 1e-9
        # +x of the camera frame, rotated by the quaternion, points at the target.
        fx = 1 - 2 * (qy * qy + qz * qz)
        fy = 2 * (qx * qy + qw * qz)
        fz = 2 * (qx * qz - qw * qy)
        d = [target[i] - eye[i] for i in range(3)]
        n = math.sqrt(sum(v * v for v in d))
        assert abs(fx * d[0] / n + fy * d[1] / n + fz * d[2] / n - 1.0) < 1e-9
    assert rec.camera_target("spawn", (0.0, 0.0, 0.0)) == rec.OVERVIEW
    eye, look = rec.camera_target("place", (0.0, 0.0, -math.pi / 2))
    assert eye[2] > look[2] and look[1] < 0.0, "the close view looks down at the hand in front of the robot"
    eye, look = rec.camera_target("nav_to_packing", (0.0, 0.0, 0.0))
    assert eye[0] < 0.0 < look[0], "the chase view is behind the robot"


def test_phase_report_covers_twelve_phases_and_the_profile():
    table = grocery_phases.rows(None)
    assert [row[0] for row in table] == list(grocery_phases.PHASE_NAMES) and len(table) == 12
    assert not grocery_phases.complete(table)
    text = grocery_phases.render(table)
    assert "of 12." in text and "Demonstrator: NOT COMPLETE" in text and "Baseline 1: " not in text
    assert grocery_phases.PROFILE == "sim-oracle" and grocery_phases.POSE_SOURCE == "ground_truth"
    # The Baseline 1 phase list is untouched by the report.
    from openamrobot_build1_scene import baseline1_phases
    assert len(baseline1_phases.PHASES) == 8


def test_tracker_tags_the_split_grasp_phases():
    class Log:
        def __init__(self):
            self.data, self.failures = {"quantities": []}, []

        def quantity(self, name, value, unit="", low=None, high=None, **_kw):
            ok = None if low is None and high is None else (low is None or value >= low) and (high is None or value <= high)
            if ok is False:
                self.failures.append(f"{name} = {value} {unit}, allowed {low} to {high}")
            self.data["quantities"].append({"name": name, "value": value, "unit": unit, "low": low, "high": high, "ok": ok})

        def require(self, condition, text):
            if not condition:
                self.failures.append(text)
            return bool(condition)
    log = Log()
    tracker = grocery_phases.Tracker(log, clock=lambda: 1.0)

    def pick():
        log.quantity("grasp_approach_moveit_error_code", 1, low=1, high=1)
        log.quantity("finger_position_on_item", 0.14, low=0.02, high=0.7)
        log.quantity("grasp_lift_moveit_error_code", 1, low=1, high=1)
        log.quantity("item_height_gain_after_lift", 0.02, low=0.04, high=0.12)
    tracker.run("arm_approach", pick, then=("grasp", "lift_object"))
    tracker.skip("place", "the product is not held after lift_object")
    data = {"phases": tracker.result()}
    verdicts = {row[0]: row[3] for row in grocery_phases.rows(data)}
    assert verdicts["arm_approach"] == "PASS" and verdicts["grasp"] == "PASS"
    assert verdicts["lift_object"] == "FAIL" and verdicts["place"] == "NOT RUN"
    classes = {row[0]: row[4] for row in grocery_phases.rows(data)}
    assert classes["lift_object"] == ["CONTACT/PHYSICS"]


def test_report_script_runs_without_a_log(tmp_path):
    script = os.path.join(PACKAGE_DIR, "scripts", "grocery_demo_report.py")
    done = subprocess.run([sys.executable, script, str(tmp_path)], capture_output=True, text=True, check=False)
    assert done.returncode == 1 and "NOT COMPLETE" in done.stdout and "Evidence summary" in done.stdout


def test_report_script_reads_a_written_log(tmp_path):
    log = {"result": "FAIL", "reason": "x", "run": {"scene": "grocery_scene.yaml"}, "grocery": {"profile": "sim-oracle",
           "pose_source": "ground_truth"}, "quantities": [], "failures": [],
           "phases": [{"phase": name, "ran": True, "failures": [], "exception": None, "sim_start": 0.0, "sim_end": 1.0,
                       "note": "", "quantities": [{"name": "c", "value": 1, "unit": "", "low": 1, "high": 1, "ok": True}]}
                      for name in grocery_phases.PHASE_NAMES]}
    with open(os.path.join(str(tmp_path), "grocery_demo_mission.json"), "w", encoding="utf-8") as f:
        json.dump(log, f)
    script = os.path.join(PACKAGE_DIR, "scripts", "grocery_demo_report.py")
    done = subprocess.run([sys.executable, script, str(tmp_path)], capture_output=True, text=True, check=False)
    assert done.returncode == 0, done.stdout
    assert "ALL PHASES PASSED" in done.stdout and "pose_source: ground_truth" in done.stdout


def test_launch_offers_the_scene_argument_with_the_build1_default():
    text = open(package_path("launch", "build1.launch.py"), encoding="utf-8").read()
    assert 'DeclareLaunchArgument("scene", default_value="scene.yaml"' in text
    assert '"scene_file": scene_file' in text
    with open(package_path("config", "grocery_scene.yaml"), encoding="utf-8") as f:
        assert yaml.safe_load(f)["world_name"] == "build1"
