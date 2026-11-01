# SPDX-License-Identifier: MIT
"""The mock MoveIt stack (bringup moveit.launch.py, sim:=mock, rviz:=false) plus an RViz2 with evidence/renders/rviz/capture.rviz and the same
parameters move_group.launch.py gives its own RViz. Only for taking the screenshots; `ros2 launch openamrobot_upperbody_bringup moveit.launch.py
rviz:=true ...` is the command a person uses.

    ros2 launch capture.launch.py lift_position:=0.175 base_plate:=bp050
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def setup(context, *args, **kwargs):
    from openamrobot_upperbody_moveit.moveit_config import moveit_config
    base_plate = LaunchConfiguration("base_plate").perform(context)
    config = moveit_config(hardware_profile="mock", base_plate=base_plate)
    bringup = get_package_share_directory("openamrobot_upperbody_bringup")
    stack = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "moveit.launch.py")),
                                     launch_arguments={"sim": "mock", "rviz": "false", "base_plate": base_plate,
                                                       "lift_position": LaunchConfiguration("lift_position").perform(context)}.items())
    rviz = Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", os.path.join(HERE, "capture.rviz")],
                parameters=[config.robot_description, config.robot_description_semantic, config.robot_description_kinematics,
                            config.planning_pipelines, config.joint_limits, {"use_sim_time": False}])
    return [stack, rviz]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("base_plate", default_value="bp000"),
                              DeclareLaunchArgument("lift_position", default_value="0.0"),
                              OpaqueFunction(function=setup)])
