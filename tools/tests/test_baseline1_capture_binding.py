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
    {"index": 2, "name": "lift_joint", "joint": "lift_joint", "component": "lift", "unit": "m", "range": [0.0, 0.35]},
    {"index": 3, "name": "base_linear_x", "joint": "base_link/twist/linear.x", "component": "base", "unit": "m/s",
     "range": None},   # interfaces #39 gives base channels a pseudo-joint name
]}
RUN_INFO = {"profile": "firmware_sil", "planning_pose_source": "ground_truth", "object_pose_source": "ground_truth"}


def write_bag(path, finger_peak=0.6, lift_peak=0.175):
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
            ("/build1/run_info", "std_msgs/msg/String"))}
        import numpy as np
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


def run(tmp_path, **kw):
    bag = tmp_path / "bag"
    write_bag(bag, **kw)
    prof = tmp_path / "profile.json"
    prof.write_text(json.dumps(PROFILE))
    out = tmp_path / "out"
    assert tool.main(["--bag", str(bag), "--component-profile", str(prof), "--out", str(out), "--run-id", "1",
                      "--run-sha", "a" * 40]) == 0
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


def test_gripper_command_and_cameras_are_reported_unbound(tmp_path):
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
