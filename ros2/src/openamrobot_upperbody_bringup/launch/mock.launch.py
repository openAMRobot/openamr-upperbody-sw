# SPDX-License-Identifier: MIT
"""Mock bringup: the arms on mock_components/GenericSystem, all five controllers active, nothing else.

    ros2 launch openamrobot_upperbody_bringup mock.launch.py [base_plate:=bp000] [head_pitch:=hp25] [rviz:=true]

Starts robot_state_publisher on the combined description with hardware_profile:=mock, ros2_control_node with
openamrobot_upperbody_control/config/arm_controllers.yaml, and spawners for joint_state_broadcaster,
left_arm_controller, right_arm_controller, left_gripper_controller and right_gripper_controller. The mobile base is not
under ros2_control (ADR-003) and is not started here.

Args
  column       dold_v4_350 | tl3_400   (default dold_v4_350; tl3_400 has no supplier data and does not expand)
  base_plate   bp000 | bp050             (default bp000: option B's baseline; column axis x +0.060 / +0.110 from base_link)
  head_pitch   hp15 | hp20 | hp25 | hp30 | hp35   (default hp25)
  base_tilt    bt05 | bt10 | bt15        (default bt10)
  rviz         true | false                      (default false: headless)
The lift is not an argument: lift_joint is a joint (calibration stops L1000 / L1175 / L1350 are positions of it), and with no
ros2_control interface for it nothing in this launch reports it; moveit.launch.py's stub does.

Overlay: the OpenArm and base meshes resolve through the workspace overlay; source install/setup.bash first (RViz only).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
ARM_AND_GRIPPER_CONTROLLERS = ("left_arm_controller", "right_arm_controller",
                               "left_gripper_controller", "right_gripper_controller")


def spawner(controller):
    return Node(package="controller_manager", executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"], output="screen")


def generate_launch_description():
    rviz = LaunchConfiguration("rviz")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([FindPackageShare(DESCRIPTION), "urdf", "combined_robot.xacro"]),
            " column:=", LaunchConfiguration("column"),
            " base_plate:=", LaunchConfiguration("base_plate"),
            " head_pitch:=", LaunchConfiguration("head_pitch"),
            " base_tilt:=", LaunchConfiguration("base_tilt"),
            " hardware_profile:=mock",
        ]),
        value_type=str,
    )
    controllers = PathJoinSubstitution([FindPackageShare(CONTROL), "config", "arm_controllers.yaml"])

    robot_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher",
                                 parameters=[{"robot_description": robot_description}], output="screen")
    ros2_control_node = Node(package="controller_manager", executable="ros2_control_node",
                             parameters=[{"robot_description": robot_description}, controllers], output="screen")
    rviz_node = Node(package="rviz2", executable="rviz2", output="screen", condition=IfCondition(rviz),
                     arguments=["-d", PathJoinSubstitution([FindPackageShare(DESCRIPTION), "config", "view.rviz"])])

    # joint_state_broadcaster first: the other four are spawned when it has exited (its spawner exits once it is active).
    joint_state_broadcaster = spawner("joint_state_broadcaster")
    then_the_rest = RegisterEventHandler(OnProcessExit(
        target_action=joint_state_broadcaster,
        on_exit=[spawner(name) for name in ARM_AND_GRIPPER_CONTROLLERS]))

    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 | tl3_400 (no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp25", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. Start RViz2 (default headless)"),
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        then_the_rest,
        rviz_node,
    ])
