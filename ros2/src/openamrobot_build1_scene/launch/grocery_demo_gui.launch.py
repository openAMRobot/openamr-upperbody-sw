# SPDX-License-Identifier: MIT
"""Watch the grocery dark store demonstrator live: the Gazebo window, RViz and the mission, on a local machine.

    ros2 launch openamrobot_build1_scene grocery_demo_gui.launch.py [stack:=true] [gazebo_gui:=true] [follow:=true]
        [rviz:=true] [mission:=true] [mission_delay:=20.0] [create_vcan:=true] [log_dir:=...]

run_grocery_demo_gui.sh uses it in three parts: stack:=true with rviz, follow and mission false; then, once
live_demo_wait.py has seen simulation time, /tf, /scan and /odom, stack:=false with the views; then stack:=false
mission:=true mission_delay:=0. Launched alone with the defaults it starts everything at once as before.

What it starts
  build1.launch.py   (stack:=true) the integrated stack exactly as the grocery-demo workflow runs it (profile
                     firmware_sil, scene grocery_scene.yaml, planning_pose_source and object_pose_source ground_truth:
                     the sim-oracle profile, station alignment off), but with headless:=false, so the Gazebo window
                     opens (gazebo_gui:=false: server only)
  gz_follow.py       (follow:=true) asks the Gazebo GUI's camera tracking to follow the robot model `openamrobot` and
                     waits until the GUI confirms it
  rviz2              (rviz:=true) rviz/grocery_demo.rviz, read only, with the MoveIt description for the trajectory display
  the mission        (mission:=true) scripts/grocery_demo_run.py after mission_delay seconds of wall time; it waits for
                     the stack itself. Logs, ROS bag and candump go to log_dir

Nothing here changes the headless CI path: .github/workflows/grocery-demo.yml runs test_grocery_demo_mission, not this.
Simulation only. Not hardware, physical acceptance, perception or safety evidence.
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "openamrobot_build1_scene"
# The arguments build1.launch.py gets from this launch. test_grocery_scene.py checks that each one exists there.
# headless follows gazebo_gui (false unless gazebo_gui:=false).
STACK_ARGUMENTS = {"profile": "firmware_sil", "scene": "grocery_scene.yaml", "station": "origin", "headless": "false",
                   "rviz": "false", "nav2": "true", "moveit": "true", "station_alignment": "false",
                   "planning_pose_source": "ground_truth", "object_pose_source": "ground_truth"}


def setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    share = get_package_share_directory(PACKAGE)
    log_dir = value("log_dir") or os.path.join(os.path.expanduser("~"), "grocery_demo_logs", time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(log_dir, exist_ok=True)
    actions = [SetEnvironmentVariable("BUILD1_LOG_DIR", log_dir),
               SetEnvironmentVariable("BASELINE1_STATION_APPROACH", "1"),
               SetEnvironmentVariable("BASELINE1_STATION_ALIGNMENT", "0"),
               SetEnvironmentVariable("GROCERY_VIDEO", value("video")),
               LogInfo(msg=f"grocery demonstrator, live: logs, ROS bag and candump in {log_dir}")]
    if value("stack") == "true":
        headless = "false" if value("gazebo_gui") == "true" else "true"
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "build1.launch.py")),
            launch_arguments={**STACK_ARGUMENTS, "headless": headless, "create_vcan": value("create_vcan")}.items()))
    if value("follow") == "true":
        actions.append(Node(package=PACKAGE, executable="gz_follow.py", name="gz_follow", output="screen",
                            arguments=["--entity", "openamrobot", "--offset", value("follow_offset")]))
    if value("rviz") == "true":
        from openamrobot_upperbody_moveit.moveit_config import moveit_config
        config = moveit_config(hardware_profile="gazebo")
        actions.append(Node(package="rviz2", executable="rviz2", name="grocery_demo_rviz", output="log",
                            arguments=["-d", os.path.join(share, "rviz", "grocery_demo.rviz")],
                            parameters=[{"use_sim_time": True}, config.robot_description, config.robot_description_semantic,
                                        config.robot_description_kinematics, config.planning_pipelines, config.joint_limits]))
    if value("mission") == "true":
        actions.append(TimerAction(period=float(value("mission_delay")), actions=[
            ExecuteProcess(cmd=["ros2", "run", PACKAGE, "grocery_demo_run.py", "--log-dir", log_dir], output="screen",
                           name="grocery_demo_mission")]))
    return actions


def generate_launch_description():
    boolean = ["true", "false"]
    return LaunchDescription([
        DeclareLaunchArgument("stack", default_value="true", choices=boolean,
                              description="Start the integrated stack (build1.launch.py); false: only the views or the mission"),
        DeclareLaunchArgument("gazebo_gui", default_value="true", choices=boolean,
                              description="Gazebo window; false: server only (headless smoke runs)"),
        DeclareLaunchArgument("follow", default_value="true", choices=boolean,
                              description="Gazebo GUI camera follows the robot (camera tracking of the GUI)"),
        DeclareLaunchArgument("follow_offset", default_value="-2.5,0.0,2.0",
                              description="x,y,z of the follow camera from the robot, m, in the robot frame"),
        DeclareLaunchArgument("rviz", default_value="true", choices=boolean, description="Open RViz with rviz/grocery_demo.rviz"),
        DeclareLaunchArgument("mission", default_value="true", choices=boolean, description="Run the grocery mission"),
        DeclareLaunchArgument("mission_delay", default_value="20.0", description="Wall seconds before the mission starts"),
        DeclareLaunchArgument("video", default_value="0", choices=["0", "1"],
                              description="1: also record the store camera frames as in CI (costs real-time factor)"),
        DeclareLaunchArgument("create_vcan", default_value="true", choices=boolean,
                              description="Create the five virtual CAN buses first (needs sudo); false if they exist"),
        DeclareLaunchArgument("log_dir", default_value="", description="Logs, bag and candump (default ~/grocery_demo_logs/<time>)"),
        OpaqueFunction(function=setup),
    ])
