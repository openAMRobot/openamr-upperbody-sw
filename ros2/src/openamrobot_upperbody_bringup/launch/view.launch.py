# SPDX-License-Identifier: MIT
"""View the combined OpenAMRobot description in RViz.

Starts robot_state_publisher, joint_state_publisher_gui and RViz2 for combined_robot.xacro.

    ros2 launch openamrobot_upperbody_bringup view.launch.py base_plate:=bp000 arm_package:=openarm

The description (xacro, RViz config) stays in openamrobot_upperbody_description; this launch file moved here with the
rest of the bringup. Nothing is controlled here: joint_state_publisher_gui drives the joints. For controllers see
mock.launch.py.

Overlay: the OpenArm meshes are referenced as package://openarm_description/..., and the base meshes as
package://openamrobot_description/.... They only resolve once the workspace overlay that contains our build is
sourced (`source ~/openamr_ws/install/setup.bash`). Without it RViz loads the model but logs mesh errors and draws
the arms and base as missing geometry.

Args
  column       dold_v4_350 | tl3_400   (default dold_v4_350; tl3_400 has no supplier data and does not expand)
  base_plate   bp000 | bp050             (default bp000: option B's baseline; column axis x +0.060 / +0.110 from base_link)
  head_pitch   hp15 | hp20 | hp25 | hp30 | hp35   (default hp25)
  base_tilt    bt05 | bt10 | bt15        (default bt10)
  arm_package  openarm | so101 | none                           (default openarm)
  enable_planar_base  true | false                              (default false; AD-30: base and arm never move together.
               With true a `world` link and base_planar_joint sit above base_footprint, which is then no longer the root.
               robot_state_publisher / joint_state_publisher_gui may not drive a planar joint: it is for experiments.)
An unknown value stops xacro with the list of accepted values.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

PKG = "openamrobot_upperbody_description"


def generate_launch_description():
    arm_package = LaunchConfiguration("arm_package")
    enable_planar_base = LaunchConfiguration("enable_planar_base")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(PKG), "urdf", "combined_robot.xacro"]),
            " column:=", LaunchConfiguration("column"),
            " base_plate:=", LaunchConfiguration("base_plate"),
            " head_pitch:=", LaunchConfiguration("head_pitch"),
            " base_tilt:=", LaunchConfiguration("base_tilt"),
            " arm_package:=", arm_package,
            " enable_planar_base:=", enable_planar_base,
        ]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 | tl3_400 (no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp25", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("arm_package", default_value="openarm",
                              description="openarm | so101 | none"),
        DeclareLaunchArgument("enable_planar_base", default_value="false",
                              description="true | false. Emits world + base_planar_joint above base_footprint (experiments only)"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}], output="screen"),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui", output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", PathJoinSubstitution([FindPackageShare(PKG), "config", "view.rviz"])]),
    ])
