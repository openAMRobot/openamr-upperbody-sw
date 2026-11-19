# SPDX-License-Identifier: MIT
"""tools/baseline1_capture_binding.py on a small rosbag2 MCAP bag written here (synthetic fixture, not a recording)."""
import json
import pathlib
import sys

import pytest

rosbags = pytest.importorskip("rosbags", reason="rosbags==0.11.9 is installed by bag-to-lerobot.yml; tracking issue openAMRobot/openamr-upperbody-sw#37")  # noqa: E501

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
import baseline1_capture_binding as tool  # noqa: E402

JTC_STATE = """
std_msgs/Header header
string[] joint_names
trajectory_msgs/JointTrajectoryPoint reference
"""
ARM = [f"openarm_left_joint{i}" for i in range(1, 8)]
PROFILE = {"channels": [
    {"index": 0, "name": "openarm_left_joint1", "joint": "openarm_left_joint1", "component": "left_arm", "unit": "rad",
     "range": [-3.4907, 1.3963]},
    {"index": 1, "name": "openarm_left_finger_joint1", "joint": "openarm_left_finger_joint1", "component": "left_gripper",
     "unit": "rad", "range": [0.0, 0.7854]},
    {"index": 2, "name": "lift_joint", "joint": "lift_joint", "component": "lift", "unit": "m", "range": [0.0, 0.35],
     "state_source": "/arm/joint_states sensor_msgs/msg/JointState.position (sim)"},
    {"index": 3, "name": "base_linear_x", "joint": "base_link/twist/linear.x", "component": "base", "unit": "m/s",
     "range": None},   # interfaces #39 gives base channels a pseudo-joint name
]}
RUN_INFO = {"profile": "firmware_sil", "planning_pose_source": "ground_truth", "object_pose_source": "ground_truth"}


CAMERAS = {"/left_wrist_camera/image": ("left_wrist_camera_optical_frame", 5), "/right_wrist_camera/image":
           ("right_wrist_camera_optical_frame", 5), "/head_camera/rgbd/image": ("head_camera_optical_frame", 6),
           "/base_camera/rgbd/image": ("base_camera_color_optical_frame", 6)}


def write_bag(path, finger_peak=0.6, lift_peak=0.175, capture=False, duplicate_cmd_vel=False, run_info=True):
    from rosbags.rosbag2 import StoragePlugin, Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    ts = get_typestore(Stores.ROS2_JAZZY)
    ts.register(get_types_from_msg(JTC_STATE, "control_msgs/msg/JointTrajectoryControllerState"))
    T = ts.types
    stamp = lambda ns: T["builtin_interfaces/msg/Time"](sec=ns // 10**9, nanosec=ns % 10**9)  # noqa: E731
    header = lambda ns, f="": T["std_msgs/msg/Header"](stamp=stamp(ns), frame_id=f)  # noqa: E731
    with Writer(path, version=9, storage_plugin=StoragePlugin.MCAP) as w:
        c = {t: w.add_connection(t, m, typestore=ts) for t, m in (
            ("/clock", "rosgraph_msgs/msg/Clock"), ("/arm/joint_states", "sensor_msgs/msg/JointState"),
            ("/left_arm_controller/controller_state", "control_msgs/msg/JointTrajectoryControllerState"),
            ("/cmd_vel", "geometry_msgs/msg/Twist"), ("/sil/lift/command", "std_msgs/msg/Int8"),
            ("/build1/run_info", "std_msgs/msg/String"), ("/joint_states", "sensor_msgs/msg/JointState"))}
        if capture:
            for topic in CAMERAS:
                c[topic] = w.add_connection(topic, "sensor_msgs/msg/Image", typestore=ts)
                info = topic.rsplit("/", 1)[0] + "/camera_info"
                c[info] = w.add_connection(info, "sensor_msgs/msg/CameraInfo", typestore=ts)
            c["/build1/capture/gripper_command"] = w.add_connection("/build1/capture/gripper_command",
                                                                    "sensor_msgs/msg/JointState", typestore=ts)
            c["/sim/left_arm_emulator/status"] = w.add_connection("/sim/left_arm_emulator/status",
                                                                  "diagnostic_msgs/msg/DiagnosticArray", typestore=ts)
        import numpy as np
        if run_info:
            w.write(c["/build1/run_info"], 7_000_000_000, ts.serialize_cdr(T["std_msgs/msg/String"](data=json.dumps(RUN_INFO)), "std_msgs/msg/String"))
        w.write(c["/sil/lift/command"], 7_500_000_000, ts.serialize_cdr(T["std_msgs/msg/Int8"](data=1), "std_msgs/msg/Int8"))
        for k in range(21):
            ns = 8_000_000_000 + k * 100_000_000
            f = finger_peak * k / 20
            w.write(c["/clock"], ns, ts.serialize_cdr(T["rosgraph_msgs/msg/Clock"](clock=stamp(ns)), "rosgraph_msgs/msg/Clock"))
            js = T["sensor_msgs/msg/JointState"](header=header(ns), name=ARM + ["openarm_left_finger_joint1", "lift_joint"],
                                                 position=np.array([0.1] * 7 + [f, lift_peak * k / 20]), velocity=np.array([]),
                                                 effort=np.array([]))
            w.write(c["/arm/joint_states"], ns, ts.serialize_cdr(js, "sensor_msgs/msg/JointState"))
            point = T["trajectory_msgs/msg/JointTrajectoryPoint"](
                positions=np.array([0.2] * 7), velocities=np.array([]), accelerations=np.array([]), effort=np.array([]),
                time_from_start=T["builtin_interfaces/msg/Duration"](sec=0, nanosec=0))
            jtc = T["control_msgs/msg/JointTrajectoryControllerState"](header=header(ns), joint_names=ARM, reference=point)
            w.write(c["/left_arm_controller/controller_state"], ns, ts.serialize_cdr(jtc, "control_msgs/msg/JointTrajectoryControllerState"))
            tw = T["geometry_msgs/msg/Twist"](linear=T["geometry_msgs/msg/Vector3"](x=0.3, y=0.0, z=0.0),
                                              angular=T["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.1))
            w.write(c["/cmd_vel"], ns, ts.serialize_cdr(tw, "geometry_msgs/msg/Twist"))
            if duplicate_cmd_vel and k % 5 == 0:   # two commands within one tick of the simulation clock
                w.write(c["/cmd_vel"], ns, ts.serialize_cdr(tw, "geometry_msgs/msg/Twist"))
            lift = T["sensor_msgs/msg/JointState"](header=header(ns), name=["lift_joint"], position=np.array([lift_peak * k / 20]),
                                                   velocity=np.array([]), effort=np.array([]))
            w.write(c["/joint_states"], ns, ts.serialize_cdr(lift, "sensor_msgs/msg/JointState"))
            if not capture:
                continue
            if k % 2 == 0:   # 5 Hz
                for topic, (frame, _rate) in CAMERAS.items():
                    img = T["sensor_msgs/msg/Image"](header=header(ns, frame), height=2, width=2, encoding="rgb8",
                                                     is_bigendian=0, step=6, data=np.zeros(12, dtype=np.uint8))
                    w.write(c[topic], ns, ts.serialize_cdr(img, "sensor_msgs/msg/Image"))
                    roi = T["sensor_msgs/msg/RegionOfInterest"](x_offset=0, y_offset=0, height=0, width=0, do_rectify=False)
                    info = T["sensor_msgs/msg/CameraInfo"](
                        header=header(ns, frame), height=2, width=2, distortion_model="plumb_bob", d=np.zeros(5),
                        k=np.array([277.0, 0, 1, 0, 277.0, 1, 0, 0, 1]), r=np.eye(3).flatten(), p=np.zeros(12),
                        binning_x=0, binning_y=0, roi=roi)
                    info_topic = topic.rsplit("/", 1)[0] + "/camera_info"
                    w.write(c[info_topic], ns, ts.serialize_cdr(info, "sensor_msgs/msg/CameraInfo"))
            held = 0.0 if k < 10 else 0.785
            cmd = T["sensor_msgs/msg/JointState"](header=header(ns), name=["openarm_left_finger_joint1", "openarm_right_finger_joint1"],
                                                  position=np.array([held, 0.0]), velocity=np.array([]), effort=np.array([]))
            w.write(c["/build1/capture/gripper_command"], ns, ts.serialize_cdr(cmd, "sensor_msgs/msg/JointState"))
            kv = T["diagnostic_msgs/msg/KeyValue"]
            motor = T["diagnostic_msgs/msg/DiagnosticStatus"](level=0, name="openarm_emulator/vcan_arm_l/openarm_left_finger_joint1",
                                                              message="enabled", hardware_id="x",
                                                              values=[kv(key="mit_q", value=str(held * 2.0))])
            diag = T["diagnostic_msgs/msg/DiagnosticArray"](header=header(ns), status=[motor])
            w.write(c["/sim/left_arm_emulator/status"], ns, ts.serialize_cdr(diag, "diagnostic_msgs/msg/DiagnosticArray"))


def run(tmp_path, extra=(), **kw):
    bag = tmp_path / "bag"
    write_bag(bag, **kw)
    prof = tmp_path / "profile.json"
    prof.write_text(json.dumps(PROFILE))
    out = tmp_path / "out"
    assert tool.main(["--bag", str(bag), "--component-profile", str(prof), "--out", str(out), "--run-id", "1",
                      "--run-sha", "a" * 40, *extra]) == 0
    return json.loads((out / "binding.json").read_text()), json.loads((out / "capture_profile_candidate.json").read_text())


def channel(report, name):
    return next(r for r in report["channels"] if r["name"] == name)


def test_state_and_commands_are_bound_with_rates_and_ranges(tmp_path):
    report, _ = run(tmp_path)
    j1 = channel(report, "openarm_left_joint1")
    assert j1["observation"]["topic"] == "/arm/joint_states" and j1["observation"]["samples"] == 21
    assert j1["observation"]["rate_hz"] == pytest.approx(10.0) and j1["observation"]["within_profile_range"] is True
    assert j1["action"]["topic"] == "/left_arm_controller/controller_state" and j1["action"]["max"] == pytest.approx(0.2)
    assert report["bag"]["simulation_time"] is True


def test_gripper_command_and_cameras_are_reported_unbound_without_the_capture(tmp_path):
    report, _ = run(tmp_path)
    assert "GripperCommand" in channel(report, "openarm_left_finger_joint1")["action"]["unbound"]
    assert report["totals"]["c9_cameras_bound"] == 0 and report["c9_v1_complete"] is False


def test_gripper_values_above_the_parallel_link_maximum_are_radians(tmp_path):
    report, _ = run(tmp_path, finger_peak=0.6)
    assert report["gripper_unit"]["openarm_left_finger_joint1"]["verdict"].startswith("radians")


def test_small_gripper_values_are_not_claimed_as_radians(tmp_path):
    report, _ = run(tmp_path, finger_peak=0.03)
    assert report["gripper_unit"]["openarm_left_finger_joint1"]["verdict"].startswith("not distinguishable")


def test_out_of_range_state_is_flagged(tmp_path):
    report, _ = run(tmp_path, lift_peak=0.5)
    assert channel(report, "lift_joint")["observation"]["within_profile_range"] is False


def test_lift_stop_index_and_base_velocity_semantics(tmp_path):
    report, cand = run(tmp_path)
    lift = channel(report, "lift_joint")["action"]
    assert lift["stop_indices"] == [1] and lift["targets_m"] == [0.175]
    base = channel(report, "base_linear_x")
    assert "unbound" in base["observation"] and base["action"]["topic"] == "/cmd_vel"
    assert [c["semantics"] for c in cand["action_channels"] if c["component"] == "base"] == ["requested_velocity"]


def test_candidate_profile_carries_the_run_declarations_and_stays_proposed(tmp_path):
    _, cand = run(tmp_path)
    assert cand["status"] == "proposed" and cand["origin"] == "simulation"
    assert cand["provenance"]["object_pose_source"] == "ground_truth"
    assert cand["provenance"]["run_id"] == "1" and cand["source_commits"] == {"openamr-upperbody-sw": "a" * 40}
    assert cand["state_names"] == [c["name"] for c in cand["state_channels"]]


def test_unreadable_bag_exits_2(tmp_path):
    prof = tmp_path / "p.json"
    prof.write_text(json.dumps(PROFILE))
    assert tool.main(["--bag", str(tmp_path / "missing"), "--component-profile", str(prof), "--out", str(tmp_path / "o")]) == 2


# Opt-in AI capture recordings (baseline1_record.py, Proposed): cameras, held gripper command, emulator status.
def test_capture_binds_the_four_c9_cameras_with_camera_info_and_a_camera_limited_grid(tmp_path):
    report, cand = run(tmp_path, capture=True)
    assert report["totals"]["c9_cameras_bound"] == 4
    head = report["cameras"]["bound"]["observation.images.head"]
    assert head["topic"] == "/head_camera/rgbd/image" and head["camera_info_topic"] == "/head_camera/rgbd/camera_info"
    assert head["rate_hz"] == pytest.approx(5.0)
    keys = {s["camera_key"]: s for s in cand["streams"] if s.get("camera_key")}
    assert set(keys) == set(tool.C9_CAMERAS) and all(s["calibration_id"].startswith("gazebo-camera-info:") for s in keys.values())
    assert keys["observation.images.base"]["frame_id"] == "base_camera_color_optical_frame"
    assert cand["fps"] == 5 and cand["max_skew_ns"] == 100_000_000


def test_capture_binds_the_held_gripper_command_and_reports_the_motor_side_trace(tmp_path):
    report, cand = run(tmp_path, capture=True)
    act = channel(report, "openarm_left_finger_joint1")["action"]
    assert act["topic"] == "/build1/capture/gripper_command" and act["max"] == pytest.approx(0.785)
    assert act["motor_side_trace"]["topic"] == "/sim/left_arm_emulator/status"
    assert act["motor_side_trace"]["max"] == pytest.approx(1.57)
    assert [c["semantics"] for c in cand["action_channels"] if c["component"] == "left_gripper"] == ["held_gripper_goal"]


def test_lift_state_on_another_topic_than_the_component_profile_names_is_flagged(tmp_path):
    report, _ = run(tmp_path)
    obs = channel(report, "lift_joint")["observation"]
    assert obs["topic"] == "/arm/joint_states" and "state_source_differs_from_profile" not in obs
    profile = json.loads(json.dumps(PROFILE))
    data = {"state": {("/joint_states", "lift_joint"): tool.Series()}, "command": {}, "other": {}, "motor_command": {},
            "topics": {}}
    data["state"][("/joint_states", "lift_joint")].add(1, 0.1)
    row = next(r for r in tool.bind(data, profile) if r["name"] == "lift_joint")
    assert row["observation"]["state_source_differs_from_profile"] is True
    assert row["observation"]["component_profile_state_source"] == "/arm/joint_states"


def test_duplicate_receipt_stamps_of_a_headerless_topic_are_counted(tmp_path):
    report, _ = run(tmp_path, duplicate_cmd_vel=True)
    st = report["timestamps"]["/cmd_vel"]
    assert st["has_header"] is False and st["receipt_nonincreasing"] == 5 and st["strictly_increasing_as_read"] is False
    assert st["receipt_equal"] == 5 and st["receipt_decreasing"] == 0
    assert report["timestamps"]["/arm/joint_states"]["strictly_increasing_as_read"] is True


def test_bag_size_is_reported_per_minute_of_simulation_time(tmp_path):
    report, _ = run(tmp_path, capture=True)
    size = report["bag"]["size"]
    assert size["files_mb"] > 0 and size["mb_per_sim_minute"] > 0 and size["camera_image_mb"] >= 0


def test_acceptance_profile_is_read_from_a_timestamped_coloured_job_log(tmp_path):
    log = tmp_path / "run.log"
    log.write_text(
        '2026-11-18T14:53:33.2807058Z \x1b[36;1mecho "acceptance profile: ${BASELINE1_PROFILE}, pose_source: x"\x1b[0m\n'
        "2026-11-18T14:53:33.5745371Z acceptance profile: sim-oracle, pose_source: ground_truth (SIMULATION ORACLE)\n")
    assert tool.acceptance_from_log(log.read_text()) == ("sim-oracle", "ground_truth")
    report, cand = run(tmp_path, extra=("--run-log", str(log)))
    assert report["acceptance"]["profile"] == "sim-oracle" and report["acceptance"]["pose_source_from_log"] == "ground_truth"
    assert cand["provenance"]["acceptance_profile"] == "sim-oracle"
    assert cand["provenance"]["declarations_from"] == "/build1/run_info"   # the fixture bag has /build1/run_info


def test_without_run_info_in_the_bag_the_declarations_come_from_the_run_log(tmp_path):
    log = tmp_path / "run.log"
    log.write_text("2026-11-19T03:20:00.1Z acceptance profile: sim-oracle, pose_source: ground_truth (SIMULATION ORACLE)\n"
                   "2026-11-19T03:20:01.1Z 35: [INFO] [launch.user]: object_pose_source: ground_truth (PERCEPTION STAND-IN)\n")
    assert tool.object_pose_source_from_log(log.read_text()) == "ground_truth"
    bag = tmp_path / "bag"
    write_bag(bag, run_info=False)
    prof = tmp_path / "profile.json"
    prof.write_text(json.dumps(PROFILE))
    assert tool.main(["--bag", str(bag), "--component-profile", str(prof), "--out", str(tmp_path / "o"), "--run-log", str(log)]) == 0
    prov = json.loads((tmp_path / "o" / "capture_profile_candidate.json").read_text())["provenance"]
    assert prov["declarations_from"].startswith("run log") and prov["object_pose_source"] == "ground_truth"
    assert prov["pose_source"] == "ground_truth"


def test_a_log_without_the_line_is_not_turned_into_a_profile(tmp_path):
    assert tool.acceptance_from_log("nothing here\n") == (None, None)
    report, _ = run(tmp_path)
    assert report["acceptance"]["profile"] == "not declared in the run log"
