# SPDX-License-Identifier: MIT
"""The station marker plate and the simulation-only station camera of openamr-platform-sw #167 in the Build 1 scene.
No ROS import."""
import math
import os
import re
import xml.etree.ElementTree as ET

from openamrobot_build1_scene import launch_support as support
from openamrobot_build1_scene import scene as scene_model

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.join(HERE, "..")
# openamr-platform-sw #167 at 353e3d6, config/station_alignment.yaml station_marker_detector.standoff and the test
# sensor of urdf/station_alignment_test_robot.urdf.xacro.
DETECTOR_STANDOFF_M = 0.713
TEST_CAMERA_POSE = "0.387 0 0.046 0 -0.1745 0"


def scene():
    return scene_model.load(os.path.join(PACKAGE, "config", "scene.yaml"))


def test_the_plate_faces_the_station_at_the_detectors_standoff():
    s = scene()
    x, y, z, yaw = scene_model.station_marker_pose(s)
    sx, sy, syaw = scene_model.station(s, "work_shelf")
    assert math.isclose(scene_model.station_marker_standoff(s), DETECTOR_STANDOFF_M, abs_tol=1e-9)
    face = scene_model.value(s["station_marker"]["face_offset"])
    face_x, face_y = x + math.cos(yaw) * face / 2.0, y + math.sin(yaw) * face / 2.0
    assert math.isclose(math.hypot(sx - face_x, sy - face_y), DETECTOR_STANDOFF_M, abs_tol=1e-6)
    assert math.isclose(math.cos(yaw - syaw), -1.0, abs_tol=1e-9), "the tags face the robot at the station"
    assert z - 0.0625 > 0.222, "the plate stays above the LiDAR plane"


def test_the_plate_is_only_in_the_opt_in_world():
    """#167 is an extension, not a Baseline 1 dependency (Alex, 18 Nov 2026): the default world has no marker plate."""
    assert "model://station_marker" not in scene_model.world_sdf(scene(), "fast")
    assert scene_model.world_sdf(scene(), "fast", station_marker=True).count("<uri>model://station_marker</uri>") == 1


def test_the_acceptance_workflow_runs_with_the_alignment_by_default():
    """Strict acceptance runs #167's alignment, declared in every leg (19 Nov 2026); the scene still has no hard
    dependency on the extension package, and station_alignment 0 stays available as the revert run."""
    with open(os.path.join(PACKAGE, "..", "..", "..", ".github", "workflows", "baseline1-acceptance.yml"), encoding="utf-8") as f:
        wf = f.read()
    assert "BASELINE1_STATION_ALIGNMENT: ${{ github.event.inputs.station_alignment || '1' }}" in wf
    assert "--require-alignment 1" in wf
    with open(os.path.join(PACKAGE, "package.xml"), encoding="utf-8") as f:
        assert "openamrobot_station_alignment" not in f.read(), "no hard dependency on the extension package"
    # but the opt-in run must still build it
    assert wf.count('[ "${BASELINE1_STATION_ALIGNMENT}" = "1" ] && extra="openamrobot_station_alignment"') == 2


def test_the_station_camera_is_bridged_only_with_the_alignment():
    on = support.bridge_topics(support.sensor_bridge(True, station_camera=True), "GZ_TO_ROS")
    off = support.bridge_topics(support.sensor_bridge(True), "GZ_TO_ROS")
    assert {"/base_camera/color/image_raw", "/base_camera/color/camera_info"} <= set(on)
    assert not {"/base_camera/color/image_raw", "/base_camera/color/camera_info"} & set(off)


def test_the_station_camera_is_the_test_sensor_of_pr_167_behind_its_own_argument():
    with open(os.path.join(PACKAGE, "urdf", "build1_robot.urdf.xacro"), encoding="utf-8") as f:
        root = ET.fromstring(f.read())
    blocks = [b for b in root.iter("{http://www.ros.org/wiki/xacro}if") if b.get("value") == "$(arg station_camera)"]
    assert len(blocks) == 1
    sensors = list(blocks[0].iter("sensor"))
    assert [s.get("name") for s in sensors] == ["station_alignment_base_camera"]
    assert sensors[0].findtext("pose") == TEST_CAMERA_POSE
    assert sensors[0].findtext("topic") == "/base_camera/color/image_raw"


def test_the_mission_sends_the_station_goal_through_the_alignment_action():
    with open(os.path.join(PACKAGE, "openamrobot_build1_scene", "baseline1.py"), encoding="utf-8") as f:
        text = f.read()
    assert 'ALIGNMENT_ACTION = "/station_alignment/navigate_to_station"' in text
    assert re.search(r"server = ALIGNMENT_ACTION if alignment_on\(\) else \"/navigate_to_pose\"", text)
    assert "station_reach_position_error" in text and "high=REACH_XY_M" in text, "the 0.04 m check stays"
