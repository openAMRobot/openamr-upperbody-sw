# SPDX-License-Identifier: MIT
"""move_group (and, optionally, RViz's MotionPlanning display) for the combined OpenAMRobot model. A building block: it assumes the robot
is already running (robot_state_publisher, the controller manager, joint states) and starts nothing of that. The user-facing launch is
openamrobot_upperbody_bringup's moveit.launch.py, which starts the stack too.

    ros2 launch openamrobot_upperbody_moveit move_group.launch.py [base_plate:=bp000] [hardware_profile:=mock|gazebo] [rviz:=false]
                                                                  [clamp_joint_states:=true]

Args
  column, base_plate, head_pitch, base_tilt   option B's configuration IDs (defaults: its baseline dold_v4_350 / bp000 / hp25 / bt10)
  hardware_profile  mock | gazebo    which stack is running; `gazebo` also sets use_sim_time (the simulator's /clock)
  rviz              true | false     RViz2 with the MotionPlanning display (config/moveit.rviz)
  clamp_joint_states  true | false   start joint_state_clamp.py and plan from /joint_states_clamped instead of /joint_states: the simulated
                                     joint state with float noise past a limit (at most 1e-5 rad) taken off, counted and logged. Simulation only:
                                     see joint_state_clamp.py for why it exists and what it is not. Nothing else reads the clamped topic

The configuration comes from MoveItConfigsBuilder over the combined xacro and this package's own files (moveit_config.py). Everything is
simulation: nothing here claims a physical limit (ADR-009).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Execution monitoring in MoveIt's TrajectoryExecutionManager: the duration allowed for a trajectory is its own duration scaled and padded.
# Simulated time runs at a real-time factor below 1 here, so with use_sim_time everything is on the same (simulated) clock.
TRAJECTORY_EXECUTION = {
    "trajectory_execution.allowed_execution_duration_scaling": 1.2,
    "trajectory_execution.allowed_goal_duration_margin": 0.5,
    "trajectory_execution.allowed_start_tolerance": 0.01,
}


CLAMPED_TOPIC = "/joint_states_clamped"       # what joint_state_clamp.py publishes


def launch_setup(context, *args, **kwargs):
    from openamrobot_upperbody_moveit.moveit_config import moveit_config

    ids = {key: LaunchConfiguration(key).perform(context) for key in ("column", "base_plate", "head_pitch", "base_tilt")}
    profile = LaunchConfiguration("hardware_profile").perform(context)
    use_sim_time = profile == "gazebo"
    config = moveit_config(hardware_profile=profile, **ids)
    parameters = [config.to_dict(), TRAJECTORY_EXECUTION, {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True}]

    clamp = LaunchConfiguration("clamp_joint_states").perform(context) == "true"
    # move_group plans from the clamped copy of /joint_states (a relative-name remap of the topic its planning scene monitor subscribes to);
    # robot_state_publisher, RViz and everything else keep the raw one.
    nodes = [Node(package="moveit_ros_move_group", executable="move_group", output="screen", parameters=parameters,
                  remappings=[("joint_states", CLAMPED_TOPIC)] if clamp else [])]
    if clamp:
        nodes.append(Node(package="openamrobot_upperbody_moveit", executable="joint_state_clamp.py", name="joint_state_clamp", output="screen",
                          parameters=[{"robot_description": config.robot_description["robot_description"], "use_sim_time": use_sim_time}]))
    if LaunchConfiguration("rviz").perform(context) == "true":
        rviz_config = get_package_share_directory("openamrobot_upperbody_moveit") + "/config/moveit.rviz"
        nodes.append(Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", rviz_config],
                          parameters=[config.robot_description, config.robot_description_semantic,
                                      config.robot_description_kinematics, config.planning_pipelines, config.joint_limits,
                                      {"use_sim_time": use_sim_time}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 (tl3_400 has no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp25", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("hardware_profile", default_value="mock", description="mock | gazebo: which stack is running"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. RViz2 with the MotionPlanning display"),
        DeclareLaunchArgument("clamp_joint_states", default_value="true",
                              description="true | false. Plan from /joint_states_clamped (float noise past a limit clamped, counted, logged)"),
        OpaqueFunction(function=launch_setup),
    ])
