# SPDX-License-Identifier: MIT
"""The real-profile lift state path: lift_relay (/base/aux_status -> lift_joint on /joint_states) and, with `guard:=true` (default), lift_guard
(MoveIt refuses to plan or execute on a missing or stale lift state).

    ros2 launch openamrobot_upperbody_lift lift_state.launch.py [guard:=true] [position_offset_m:=0.0]

move_group must be started with GUARD_REMAPPINGS (openamrobot_upperbody_lift.guard_logic: move_action, execute_trajectory and
plan_kinematic_path renamed to *_unguarded). That belongs to the future real-profile launch; mock and sim launches do not use this file.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("guard", default_value="true", description="true | false: also start lift_guard"),
        DeclareLaunchArgument("position_offset_m", default_value="0.0", description="TODO(Evans): calibration offset from the L1000 stop, m"),
        Node(package="openamrobot_upperbody_lift", executable="lift_relay.py", name="lift_relay", output="screen",
             parameters=[{"position_offset_m": LaunchConfiguration("position_offset_m")}]),
        Node(package="openamrobot_upperbody_lift", executable="lift_guard.py", name="lift_guard", output="screen",
             condition=IfCondition(LaunchConfiguration("guard"))),
    ])
