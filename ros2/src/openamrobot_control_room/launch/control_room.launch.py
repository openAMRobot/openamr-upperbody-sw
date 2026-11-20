# SPDX-License-Identifier: MIT
"""Digital Twin Control Room: monitoring components only, next to a running Build 1, Baseline 1 or grocery launch.

    ros2 launch openamrobot_control_room control_room.launch.py [rviz:=true] [foxglove:=false] [vcan:=true]
        [monitor:=true] [use_sim_time:=true] [station:=work_shelf] [scene:=scene.yaml] [foxglove_port:=8765]

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


def station_pose(name, scene="scene.yaml"):
    """(x, y, yaw) of a station from a scene file of openamrobot_build1_scene (scene.yaml for Baseline 1,
    grocery_scene.yaml for the grocery dark store, whose `packing` station only grocery_scene.station knows), or NaNs
    without that package or station."""
    try:
        from openamrobot_build1_scene import scene as scene_model
        path = scene if os.path.isabs(scene) else os.path.join(
            get_package_share_directory("openamrobot_build1_scene"), "config", scene)
        loaded = scene_model.load(path)
        try:
            from openamrobot_build1_scene import grocery_scene
            lookup = grocery_scene.station if "grocery" in loaded else scene_model.station
        except ImportError:
            lookup = scene_model.station
        return [float(v) for v in lookup(loaded, name)]
    except Exception:  # noqa: BLE001 - the monitor then reports no station error
        return [float("nan")] * 3


def nodes(context):
    share = get_package_share_directory(PACKAGE)
    sim = LaunchConfiguration("use_sim_time").perform(context) == "true"
    name = LaunchConfiguration("station").perform(context)
    scene = LaunchConfiguration("scene").perform(context)
    pose = station_pose(name, scene)
    actions = [LogInfo(msg=f"control room: station {name} of {scene} at {pose} (NaN: no station error)")]
    actions.append(Node(package=PACKAGE, executable="control_room_monitor.py", name="control_room_monitor", output="screen",
                        condition=IfCondition(LaunchConfiguration("monitor")),
                        parameters=[{"use_sim_time": sim, "station": pose, "station_name": name, "scene": scene,
                                     "status_log": LaunchConfiguration("status_log").perform(context)}]))
    actions.append(Node(package=PACKAGE, executable="vcan_monitor.py", name="control_room_vcan_monitor", output="screen",
                        condition=IfCondition(LaunchConfiguration("vcan")),
                        parameters=[{"use_sim_time": sim, "log": LaunchConfiguration("can_log").perform(context)}]))
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
                              description="station for the station error (stations of the scene file)"),
        DeclareLaunchArgument("scene", default_value="scene.yaml",
                              description="scene file of openamrobot_build1_scene: scene.yaml (Baseline 1) or "
                                          "grocery_scene.yaml (grocery dark store)"),
        DeclareLaunchArgument("status_log", default_value="", description="file: every status as one JSON line"),
        DeclareLaunchArgument("can_log", default_value="", description="file: CAN summary and decoded values, one JSON line a second"),
        OpaqueFunction(function=nodes),
    ])
