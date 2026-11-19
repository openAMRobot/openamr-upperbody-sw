# SPDX-License-Identifier: MIT
"""The Baseline 1 recorders: the bag holds what the phase table is computed from, candump covers every virtual CAN
bus of setup_vcan_all.sh, and the trace carries localization next to odometry. No ROS import."""
import os
import re

from openamrobot_build1_scene import baseline1_phases as phases
from openamrobot_build1_scene import baseline1_record as record

HERE = os.path.dirname(os.path.abspath(__file__))


def test_the_bag_records_ground_truth_odometry_localization_commands_and_device_states():
    for topic in ("/build1/ground_truth/odom", "/odom", "/tf", "/amcl_pose", "/cmd_vel", "/cmd_vel_nav",
                  "/sil/drive/emulator_state", "/sil/lift/emulator_state", "/sil/fw/status", "/build1/rtf_stats",
                  "/build1/ground_truth/item/pose", "/arm/joint_states"):
        assert topic in record.BAG_TOPICS, topic
    assert re.fullmatch(record.BAG_REGEX, "/left_arm_controller/controller_state")


def test_candump_covers_every_virtual_can_bus_the_setup_script_creates():
    with open(os.path.join(HERE, "..", "scripts", "setup_vcan_all.sh"), encoding="utf-8") as f:
        created = set(re.findall(r"vcan_[a-z_]+", f.read())) - {"vcan_all"}
    assert created == set(record.VCAN)


def test_the_trace_table_has_the_localization_and_heading_columns():
    trace = [{"sim": float(t), "wall": float(t), "phase": "nav_to_station", "x": 0.0, "y": 0.0, "yaw": 0.0, "v": 0.0,
              "w": 0.0, "odom_err": 0.01, "odom_yaw_err": 0.002, "map_err": 0.02, "lift": 0.0, "item_z": 0.9,
              "rtf": 0.5} for t in range(3)]
    header = phases.trace_table(trace, every_sim_s=1.0)[0].split()
    assert "odom_yaw_err" in header and "map_err" in header
    summary = phases.trace_summary(trace, [{"phase": "nav_to_station"}])
    assert summary[0].count("|") == summary[1].count("|")
    assert summary[1].split(" | ")[-2] == "0.02"


# AI capture (Proposed, opt-in BASELINE1_AI_CAPTURE=1). Off by default: the Baseline 1 bag is recorded as before.
def test_ai_capture_is_off_by_default_and_the_default_recording_is_unchanged(tmp_path):
    assert not record.ai_capture_enabled({})
    command = record.bag_command("/tmp/bag", record.ai_capture_enabled({}))
    assert command == ["ros2", "bag", "record", "--use-sim-time", "-o", "/tmp/bag", "--topics", *record.BAG_TOPICS,
                       "--regex", record.BAG_REGEX]
    env = record.recorder_environment({"ROS_DOMAIN_ID": "35"}, str(tmp_path))
    assert env == {"ROS_DOMAIN_ID": "35"} and not os.listdir(tmp_path)


def test_ai_capture_records_the_four_robot_cameras_their_info_and_the_gripper_command():
    from openamrobot_build1_scene import launch_support
    command = record.bag_command("/tmp/bag", record.ai_capture_enabled({record.AI_CAPTURE_ENV: "1"}))
    pattern = re.compile(command[command.index("--regex") + 1])
    bridged = set(launch_support.bridge_topics(launch_support.sensor_bridge(True), "GZ_TO_ROS"))
    for camera in record.CAMERA_TOPICS:
        assert pattern.search(camera) and camera in bridged, camera    # a topic the sensor bridge really publishes
    assert len([t for t in record.CAMERA_TOPICS if not t.endswith("camera_info")]) == 4
    for topic in (record.GRIPPER_COMMAND_TOPIC, "/odom", "/build1/run_info", *record.EMULATOR_STATUS_TOPICS,
                  "/left_arm_controller/controller_state", "/left_gripper_controller/gripper_cmd/_action/status"):
        assert pattern.search(topic), topic
    for topic in ("/scan", "/odom_extra", "/x/odom", "/head_camera/rgbd/depth_image"):
        assert not pattern.search(topic), topic
    assert "--include-hidden-topics" in command


def test_ai_capture_names_no_topics_list_so_rosbag2_never_stops_discovery_early():
    # rosbag2 Jazzy stops discovery once its subscriptions equal the --topics count (regex matches included):
    # run 44448676604 named 48, subscribed 48 and never subscribed /build1/run_info or /build1/rtf_stats.
    command = record.bag_command("/tmp/bag", True)
    assert "--topics" not in command and command.count("--regex") == 1


def test_the_camera_topics_are_the_sensors_of_the_scene_robot():
    with open(os.path.join(HERE, "..", "urdf", "build1_robot.urdf.xacro"), encoding="utf-8") as f:
        sensors = set(re.findall(r"<topic>([^<]+)</topic>", f.read()))
    assert {"/head_camera/rgbd", "/base_camera/rgbd", "/${side}_wrist_camera/image"} <= sensors
    for topic in record.CAMERA_TOPICS:
        assert topic.startswith(("/head_camera/rgbd/", "/base_camera/rgbd/", "/left_wrist_camera/", "/right_wrist_camera/"))


def test_ai_capture_gives_only_the_recorder_discovery_peers_for_120_participant_indices(tmp_path):
    env = record.recorder_environment({record.AI_CAPTURE_ENV: "1", "ROS_DOMAIN_ID": "35"}, str(tmp_path))
    path = env["FASTRTPS_DEFAULT_PROFILES_FILE"]
    assert env["FASTDDS_DEFAULT_PROFILES_FILE"] == path and os.path.dirname(path) == str(tmp_path)
    ports = [int(p) for p in re.findall(r"<port>(\d+)</port>", open(path, encoding="utf-8").read())]
    assert len(ports) == 120 and ports[0] == 7400 + 250 * 35 + 10 and ports[-1] == ports[0] + 2 * 119
    other = tmp_path / "cyclone"
    other.mkdir()
    cyclone = record.recorder_environment({record.AI_CAPTURE_ENV: "1", "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp"}, str(other))
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" not in cyclone and not os.listdir(other)


def test_gripper_hold_publishes_nothing_until_both_sides_have_a_value():
    hold = record.GripperCommandHold()
    assert hold.sample(1_000) is None
    hold.goal("left", 0.6)
    assert hold.sample(2_000) is None
    hold.start({"openarm_right_finger_joint1": -0.001, "openarm_left_finger_joint1": 0.0})
    assert hold.source == {"left": "goal", "right": "measured_at_capture_start"}
    assert hold.sample(3_000) == (["openarm_left_finger_joint1", "openarm_right_finger_joint1"], [0.6, -0.001])


def test_gripper_hold_keeps_the_latest_goal_and_strictly_increasing_stamps():
    hold = record.GripperCommandHold()
    hold.start({"openarm_left_finger_joint1": 0.0, "openarm_right_finger_joint1": 0.0})
    assert hold.sample(10) is not None
    hold.goal("left", 0.785)
    hold.start({"openarm_left_finger_joint1": 0.3, "openarm_right_finger_joint1": 0.2})   # start never overrides
    assert hold.sample(10) is None and hold.sample(9) is None           # a stamp that does not advance is dropped
    assert hold.sample(20)[1] == [0.785, 0.0]
