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
