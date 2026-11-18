# SPDX-License-Identifier: MIT
"""Digital Twin Control Room: monitoring components only, next to a running Build 1, Baseline 1 or grocery launch.

    ros2 launch openamrobot_control_room control_room.launch.py [rviz:=true] [foxglove:=false] [vcan:=true]
        [monitor:=true] [use_sim_time:=true] [station:=work_shelf] [foxglove_port:=8765]

Starts, each optional:
  control_room_monitor  read-only summaries on /control_room/ (openamrobot_control_room/monitor.py)
  vcan_monitor          receive-only frame counts of the five virtual CAN buses on /control_room/can/
  rviz2                 rviz/control_room.rviz: view and measure tools only, no goal, pose or point tool
  foxglove_bridge       config/foxglove_bridge.yaml: no client publish, services, parameters or assets, so a
                        browser viewer (Lichtblick, config/lichtblick_layout.json) can only subscribe

Nothing here publishes a command, sends an action goal, calls a service that moves the robot or writes a parameter.
The Gazebo GUI is not started: its world controls and transform tools write to the simulation.
Simulation monitoring only; nothing shown is hardware, physical acceptance or safety evidence.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "openamrobot_control_room"


def station_pose(name):
    """(x, y, yaw) of a Build 1 station from openamrobot_build1_scene's scene.yaml, or NaNs without that package."""
    try:
        from openamrobot_build1_scene import scene as scene_model
        path = os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml")
        return [float(v) for v in scene_model.station(scene_model.load(path), name)]
    except Exception:  # noqa: BLE001 - the monitor then reports no station error
        return [float("nan")] * 3


def nodes(context):
    share = get_package_share_directory(PACKAGE)
    sim = LaunchConfiguration("use_sim_time").perform(context) == "true"
    name = LaunchConfiguration("station").perform(context)
    pose = station_pose(name)
    actions = [LogInfo(msg=f"control room: station {name} at {pose} (NaN: no station error)")]
    actions.append(Node(package=PACKAGE, executable="control_room_monitor.py", name="control_room_monitor", output="screen",
                        condition=IfCondition(LaunchConfiguration("monitor")),
                        parameters=[{"use_sim_time": sim, "station": pose, "station_name": name}]))
    actions.append(Node(package=PACKAGE, executable="vcan_monitor.py", name="control_room_vcan_monitor", output="screen",
                        condition=IfCondition(LaunchConfiguration("vcan")), parameters=[{"use_sim_time": sim}]))
    actions.append(Node(package="rviz2", executable="rviz2", name="control_room_rviz", output="screen",
                        condition=IfCondition(LaunchConfiguration("rviz")),
                        arguments=["-d", os.path.join(share, "rviz", "control_room.rviz")],
                        parameters=[{"use_sim_time": sim}]))
    actions.append(Node(package="foxglove_bridge", executable="foxglove_bridge", name="control_room_foxglove_bridge",
                        output="screen", condition=IfCondition(LaunchConfiguration("foxglove")),
                        parameters=[os.path.join(share, "config", "foxglove_bridge.yaml"),
                                    {"use_sim_time": sim, "port": int(LaunchConfiguration("foxglove_port").perform(context)),
                                     "address": LaunchConfiguration("foxglove_address").perform(context)}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("rviz", default_value="true", description="RViz2 with rviz/control_room.rviz"),
        DeclareLaunchArgument("foxglove", default_value="false", description="foxglove_bridge, read only, for Lichtblick"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("foxglove_address", default_value="127.0.0.1",
                              description="127.0.0.1 keeps the bridge on this machine; 0.0.0.0 opens it to the network"),
        DeclareLaunchArgument("vcan", default_value="true", description="receive-only virtual CAN monitor"),
        DeclareLaunchArgument("monitor", default_value="true", description="read-only control room monitor node"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("station", default_value="work_shelf",
                              description="Build 1 station for the station error (scene.yaml stations)"),
        OpaqueFunction(function=nodes),
    ])
