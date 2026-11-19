# SPDX-License-Identifier: MIT
"""Control room REPLAY: a recorded run on its own simulation time, labelled REPLAY. Needs no running simulation.

    ros2 launch openamrobot_control_room replay.launch.py bag:=BUILD1_LOG_DIR/bag[,CONTROL_ROOM_BAG]
        mission:=BUILD1_LOG_DIR/baseline1_mission.json [rate:=1.0] [start:=SIM_S] [end:=SIM_S] [rviz:=true]

Starts the replay node (replay.py: every recorded non-command topic on /control_room/replay/<topic>, the recorded
clock on /control_room/replay/clock, the mission phase at that time on /control_room/replay/status) and RViz2 with
rviz/control_room_replay.rviz, /tf, /tf_static and /clock remapped to /control_room/replay/ and use_sim_time, so RViz
draws the recording and nothing live. Lichtblick can open the bag file itself (config/lichtblick_layout.json).
A replay is a record of a simulation run, never a live, hardware or safety result.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "openamrobot_control_room"
REMAPS = [("/tf", "/control_room/replay/tf"), ("/tf_static", "/control_room/replay/tf_static"),
          ("/clock", "/control_room/replay/clock")]


def nodes(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    arguments = []
    for bag in filter(None, value("bag").split(",")):
        arguments += ["--bag", bag]
    if value("mission"):
        arguments += ["--mission", value("mission")]
    arguments += ["--rate", value("rate"), "--hold", value("hold")]
    for name in ("start", "end"):
        if value(name):
            arguments += [f"--{name}", value(name)]
    share = get_package_share_directory(PACKAGE)
    return [
        Node(package=PACKAGE, executable="control_room_replay.py", name="control_room_replay", output="screen",
             arguments=arguments),
        Node(package="rviz2", executable="rviz2", name="control_room_replay_rviz", output="screen",
             condition=IfCondition(LaunchConfiguration("rviz")),
             arguments=["-d", os.path.join(share, "rviz", "control_room_replay.rviz")],
             parameters=[{"use_sim_time": True}], remappings=REMAPS),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("bag", description="rosbag2 directories of the run, comma separated"),
        DeclareLaunchArgument("mission", default_value="", description="mission JSON of the same run"),
        DeclareLaunchArgument("rate", default_value="1.0", description="recorded simulation seconds per host second"),
        DeclareLaunchArgument("start", default_value=""),
        DeclareLaunchArgument("end", default_value=""),
        DeclareLaunchArgument("hold", default_value="5.0", description="host seconds the status stays up after the end"),
        DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=nodes),
    ])
