# SPDX-License-Identifier: MIT
"""The whole robot in one simulation: ONE launch for base, lift, both arms and sensors, headless on a CI runner.

    ros2 launch openamrobot_upperbody_bringup robot_sim.launch.py [headless:=true] [nav2:=false] [base_plate:=bp000]
                                                                   [physics_engine:=...] [world:=/path/x.sdf]

It includes openamrobot_upperbody_gazebo/sim.launch.py, which is the only place that starts the simulator, so there is one gz server, one
/clock bridge, one robot_state_publisher (the combined description), one /joint_states publisher (the merger), the diff-drive base and the
arm, gripper and lift controllers. On top of it this launch adds
  * the base IMU on /imu/data_raw, the name the real robot uses (D-10); platform-sw's bridge only has /imu, and
  * Nav2, only with nav2:=true (platform-sw's navigation_launch.py with use_sim_time; needs the navigation2 packages installed).
It never starts a second simulator, robot_state_publisher or /clock source: do not include platform-sw's bringup beside it.
"""
import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# argument -> default, forwarded to sim.launch.py (same defaults as there; the other arguments of sim.launch.py are not forwarded)
FORWARDED = {"headless": "true", "base_plate": "bp000", "physics_engine": "gz-physics-dartsim-plugin"}


def generate_launch_description():
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    here = get_package_share_directory("openamrobot_upperbody_bringup")
    args = [DeclareLaunchArgument(name, default_value=default) for name, default in FORWARDED.items()]
    args.append(DeclareLaunchArgument("nav2", default_value="false", description="true | false: platform-sw's Nav2"))
    include = IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                       launch_arguments={name: LaunchConfiguration(name) for name in FORWARDED}.items())

    imu = Node(package="ros_gz_bridge", executable="parameter_bridge", name="imu_data_raw_bridge", output="screen",
               parameters=[{"config_file": os.path.join(here, "config", "gz_bridge_imu_data_raw.yaml")}])

    actions = [*args, include, imu]
    try:
        nav2 = os.path.join(get_package_share_directory("openamrobot_nav2"), "launch", "navigation_launch.py")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(nav2), launch_arguments={"use_sim_time": "true"}.items(),
                                                condition=IfCondition(LaunchConfiguration("nav2"))))
    except PackageNotFoundError:
        pass
    return LaunchDescription(actions)
