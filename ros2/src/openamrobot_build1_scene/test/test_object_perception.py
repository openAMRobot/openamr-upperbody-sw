# SPDX-License-Identifier: MIT
"""object_pose_source estimate (Proposed, docs/architecture/object_perception.md): the estimator on the Build 1 work
shelf geometry, the validation arithmetic, and the wiring of the estimate through the launch, the loader, the rig and
scn04. No ROS, no simulator: the depth image is an ideal ray-cast fixture (openamrobot_object_perception.synthetic).
"""
import math
import os
import re
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.dirname(HERE)
sys.path.insert(0, PACKAGE)
sys.path.insert(0, os.path.join(PACKAGE, "..", "openamrobot_object_perception"))

from openamrobot_build1_scene import perception_validation as pv  # noqa: E402
from openamrobot_build1_scene import planning_scene as ps  # noqa: E402
from openamrobot_build1_scene import scene as scene_model  # noqa: E402
from openamrobot_object_perception import box_fit as bf  # noqa: E402
from openamrobot_object_perception import synthetic as syn  # noqa: E402

# The head camera lens in base_footprint at lift 0 for the fixture: column axis 0.060 m ahead of base_link
# (upperbody_configuration.yaml, base_plate bp000), arm mount 0.180 m and lens 0.060 m further; height deck 0.304 +
# column 0.530 + arm mount 0.166 + lens 0.223; pitched down 30 degrees (hp30). The running estimator takes it from /tf.
HEAD_CAMERA = (0.30, 0.0, 1.223)
HEAD_PITCH_DOWN = math.radians(30.0)
REGION = (0.45, 1.4, -0.9, 0.9)


def read(*parts):
    with open(os.path.join(PACKAGE, *parts), encoding="utf-8") as f:
        return f.read()


def in_base(station, xyz, yaw):
    x, y, base_yaw = station
    dx, dy = xyz[0] - x, xyz[1] - y
    c, s = math.cos(-base_yaw), math.sin(-base_yaw)
    return (c * dx - s * dy, s * dx + c * dy, xyz[2]), yaw - base_yaw


def build1_view(station_error=(0.0, 0.0, 0.0)):
    """Boxes of the Build 1 scene in base_footprint of a robot parked at the work-shelf station (plus `station_error`
    in x, y, yaw of the world), and the true tote and item poses in base_footprint."""
    scene = scene_model.load(os.path.join(PACKAGE, "config", "scene.yaml"))
    sx, sy, syaw = scene_model.station(scene, "work_shelf")
    station = (sx + station_error[0], sy + station_error[1], syaw + station_error[2])
    boxes = [(in_base(station, centre, yaw)[0], size, in_base(station, centre, yaw)[1])
             for _name, size, centre, yaw in scene_model.static_boxes(scene)]
    bodies = scene_model.movable_objects(scene)
    truth = {}
    for name, body in bodies.items():
        x, y, z, yaw = body["pose"]
        position, relative = in_base(station, (x, y, z), yaw)
        truth[name] = (position, relative)
        if body["kind"] == "tote":
            boxes += syn.tote_boxes(position, body["size"], body["wall"], relative)
        else:
            boxes.append((position, body["size"], relative))
    return boxes, truth, bodies


def estimate(boxes, bodies):
    rotation = syn.optical_rotation(0.0, HEAD_PITCH_DOWN)
    fx, fy, cx, cy = syn.intrinsics(640, 360, 1.57)            # the head camera sensor of build1_robot.urdf.xacro
    depth = syn.render(boxes, rotation, HEAD_CAMERA, 640, 360, fx, fy, cx, cy, near=0.1, far=10.0)
    points = bf.in_region(bf.depth_points(depth, fx, fy, cx, cy, rotation, HEAD_CAMERA), REGION)
    tote, item = bodies["tote"], bodies["item"]
    support = tote["pose"][2]                                     # what the launch passes as support_z
    found_item = bf.estimate_item(points, item["size"], support + tote["wall"], support + tote["size"][2] + 0.008, HEAD_CAMERA, fx)
    found_tote = bf.estimate_tote(points, tote["size"], support, HEAD_CAMERA, fx)
    return found_item, found_tote


@pytest.mark.parametrize("station_error", [(0.0, 0.0, 0.0), (0.03, -0.02, 0.05), (-0.04, 0.03, -0.07)])
def test_the_estimator_finds_tote_and_item_on_the_build1_work_shelf(station_error):
    """The Build 1 work shelf (scene.yaml: boards, posts, tote, item, payload) from the head camera at the station, and
    with the base off the station as the Baseline 1 missions stopped (up to 0.067 m and 0.08 rad). Ideal depth: the
    estimate is within 1.5 mm and 0.01 rad of the true pose in base_footprint."""
    boxes, truth, bodies = build1_view(station_error)
    (item, why_item), (tote, why_tote) = estimate(boxes, bodies)
    assert item is not None, why_item
    assert tote is not None, why_tote
    for name, found in (("item", item), ("tote", tote)):
        measured = pv.error(name, found.position, found.yaw, *truth[name], found.covariance)
        assert measured["position"] < 0.0015 and abs(measured["yaw"]) < 0.01, (name, measured, found.detail)


def test_the_item_estimate_meets_the_grasp_depth_budget_along_the_fingers():
    """The side grasp puts the item centre 9 mm behind the TCP; with 9.9 mm the palm touches its face
    (arm_frames.py). On ideal depth the error along the base x axis (along the fingers at face yaw 0) is under 1 mm."""
    from openamrobot_build1_scene import arm_frames
    budget = arm_frames.TCP_ALONG_FINGERS_M - arm_frames.PALM_ALONG_FINGERS_M - 0.035 / 2.0 - arm_frames.ITEM_DEPTH_BEHIND_TCP_M
    assert 0.0 < budget < 0.002
    boxes, truth, bodies = build1_view()
    (item, _why), _tote = estimate(boxes, bodies)
    assert abs(item.position[0] - truth["item"][0][0]) < 0.001


def test_validation_measures_in_base_footprint_and_folds_the_yaw_by_shape():
    base = ((1.0, 2.0, 0.0), (0.0, 0.0, math.sin(-math.pi / 4), math.cos(-math.pi / 4)))   # yaw -pi/2
    position, yaw = pv.truth_in_base(*base, (1.5, 2.0, 0.9), (0.0, 0.0, 0.0, 1.0))
    assert np.allclose(position, (0.0, 0.5, 0.9)) and abs(yaw - math.pi / 2) < 1e-12
    cov = [0.0] * 36
    cov[0] = cov[7] = 1e-6
    measured = pv.error("item", (0.001, 0.5, 0.9), 0.02, position, yaw, cov)
    assert abs(measured["dx"] - 0.001) < 1e-12 and abs(measured["yaw"] - 0.02 + 0.0) < 1e-9   # pi/2 apart is no error
    assert abs(measured["nees_xy"] - 1.0) < 1e-9
    assert abs(pv.error("tote", position, yaw + math.pi, position, yaw)["yaw"]) < 1e-9
    record = pv.Record()
    record.add("item", measured)
    record.add("item", pv.error("item", (0.003, 0.5, 0.9), yaw, position, yaw))
    summary = record.summary()["item"]
    assert summary["count"] == 2 and abs(summary["max_position_m"] - 0.003) < 1e-9


def test_estimate_is_a_declared_proposed_source_with_its_own_topic_type_and_frame():
    assert ps.OBJECT_POSE_SOURCES == ("ground_truth", "estimate") and ps.DEFAULT_OBJECT_POSE_SOURCE == "ground_truth"
    assert ps.object_pose_topic("estimate", "item") == "/perception/objects/item/pose"
    assert ps.object_pose_type("estimate") == "PoseWithCovarianceStamped"
    assert ps.object_pose_type("ground_truth") == "PoseStamped"
    assert ps.object_truth_topic("tote") == ps.object_pose_topic("ground_truth", "tote")
    assert ps.ESTIMATE_FRAME == "base_footprint" and ps.ESTIMATED_OBJECTS == ("tote", "item")
    line = ps.object_declaration("estimate")
    assert line.startswith("object_pose_source: estimate (Proposed") and "ground truth is read only to measure" in line
    for bad in (None, "", "detector"):
        with pytest.raises(ValueError):
            ps.object_pose_type(bad)


def test_pose_fields_reads_both_message_shapes():
    pose = SimpleNamespace(position=SimpleNamespace(x=1.0, y=2.0, z=3.0), orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))
    stamped = SimpleNamespace(header=SimpleNamespace(frame_id="world"), pose=pose)
    with_cov = SimpleNamespace(header=SimpleNamespace(frame_id="base_footprint"), pose=SimpleNamespace(pose=pose, covariance=[0.0] * 36))
    assert ps.pose_fields(stamped) == ("world", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))
    assert ps.pose_fields(with_cov) == ("base_footprint", (1.0, 2.0, 3.0), (0.0, 0.0, 0.0, 1.0))


def test_the_loader_places_estimates_in_base_footprint_without_the_base_pose_and_waits_for_the_first():
    loader = read("scripts", "planning_scene_loader.py")
    assert "object_pose_type(self.object_pose_source)" in loader
    assert "base = (0.0, 0.0, 0.0) if self.object_frame.get(name) == ESTIMATE_FRAME else pose" in loader
    assert "name in self.unseen_left_out and name not in self.object_pose" in loader
    centre, _q = ps.object_box_in_base((0.0, 0.0, 0.0), (0.85, 0.1, 0.82), (0.0, 0.0, 0.0, 1.0), 0.03)
    assert np.allclose(centre, (0.85, 0.1, 0.85))


def test_launch_starts_the_estimator_and_the_validation_only_for_estimate_and_refuses_it_without_the_camera():
    launch = read("launch", "build1.launch.py")
    assert "if object_source == planning_scene.ESTIMATE_SOURCE:" in launch
    assert 'package="openamrobot_object_perception", executable="object_pose_estimator.py"' in launch
    assert 'executable="object_pose_validation.py"' in launch
    assert '"depth_topic": "/head_camera/rgbd/depth_image"' in launch
    assert 'Shutdown(reason="estimate without the head camera")' in launch
    assert 'info["object_pose_estimator"]' in launch
    assert "<exec_depend>openamrobot_object_perception</exec_depend>" in read("package.xml")
    assert "scripts/object_pose_validation.py" in read("CMakeLists.txt")


def test_scn04_takes_its_target_from_the_declared_source_and_measures_on_ground_truth():
    scenarios = read("openamrobot_build1_scene", "scenarios.py")
    grasp = scenarios[scenarios.index("def grasp(rig, log):"):scenarios.index("def _withdraw(")]
    assert 'item, item_yaw = rig.object_in_base("item")' in grasp
    assert 'rig.record_estimate_error(log, ("item", "tote"), "when the grasp target is read")' in grasp
    assert "yaw = face_yaw(0.0, item_yaw)" in grasp
    assert "object_position(" not in scenarios
    # every outcome read (start height, push, lift, hold, release) is Gazebo's pose
    assert grasp.count('rig.object_truth_position("item")') == 6
    rig = read("openamrobot_build1_scene", "rig.py")
    assert "return pose_fields(self.get(object_truth_topic(name)))[1]" in rig
    assert 'log.data["perception_validation"] = rig.get(VALIDATION)' in rig
    assert "ground_truth/" not in read("openamrobot_build1_scene", "planning_scene.py").split("def object_truth_topic")[1].split("def pose_fields")[0]


def test_the_estimate_grasp_scenario_is_registered_and_selected_by_the_grasp_job():
    assert "scn04_grasp_estimate:2400" in read("CMakeLists.txt")
    test = read("test", "test_scn04_grasp_estimate.py")
    assert 'object_pose_source="estimate"' in test and 'sensors="true"' in test
    assert 'build1_harness.run(self, "scn04_grasp", "scn04_grasp_estimate")' in test
    with open(os.path.join(PACKAGE, "..", "..", "..", ".github", "workflows", "build1-scene-tests.yml"), encoding="utf-8") as f:
        workflow = f.read()
    grasp_job = re.search(r'name: firmware_sil arms, grasp, single writer\n\s+tests: "([^"]+)"', workflow).group(1)
    assert re.match(grasp_job, "test_scn04_grasp_estimate") and re.match(grasp_job, "test_scn04_grasp")
    static_job = re.search(r'name: static and vcan\n\s+tests: "([^"]+)"', workflow).group(1)
    assert re.match(static_job, "test_object_perception")
