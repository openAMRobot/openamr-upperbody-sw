"""MoveIt on the combined OpenAMRobot model: the stack (mock or Gazebo), then move_group, optionally RViz's MotionPlanning display.

    ros2 launch openamrobot_upperbody_bringup moveit.launch.py [base_plate:=bp000] [sim:=mock|gazebo] [rviz:=false]
                                                              [lift_position:=0.0] [headless:=false] [physics_engine:=<gz-physics plugin>]

  sim=mock     mock.launch.py (arms on mock_components/GenericSystem, five controllers active), a stub that reports the wheel joints at
               rest (the base is not under ros2_control) and the lift at `lift_position` (no ros2_control interface: we have no
               authority to command it), then move_group. Nothing else of the base runs.
  sim=gazebo   openamrobot_upperbody_gazebo's sim.launch.py (Gazebo, the base on its DiffDrive plugin, the arms on gz_ros2_control, the
               five controllers), then move_group on the simulator's clock.

Args
  column       dold_v4_350 | tl3_400   (default dold_v4_350; tl3_400 has no supplier data and does not expand)
  base_plate   bp000 | bp050             (default bp000: option B's baseline; column axis x +0.060 / +0.110 from base_link)
  head_pitch   hp15 | hp20 | hp25 | hp30 | hp35   (default hp25)
  base_tilt    bt05 | bt10 | bt15        (default bt10)
  lift_position   sim:=mock only. m, the position of lift_joint the stub reports: 0.0 / 0.175 / 0.350 are the calibration stops
                  L1000 / L1175 / L1350 (default 0.0: every joint at zero). With sim:=gazebo the merger's default is the same 0.0
  sim             mock | gazebo
  rviz            true | false  RViz2 with the MotionPlanning display (config/moveit.rviz of the moveit package). With sim:=gazebo the
                  simulator's own RViz is not started: this one replaces it.
  headless        sim:=gazebo only. true runs `gz sim -s` (no Gazebo window), as the checks do
  physics_engine  sim:=gazebo only. The gz-physics plugin, passed to sim.launch.py. EMPTY by default, which means sim.launch.py's own
                  default: DART, the engine platform-sw tuned the base on. DART does not close the grippers (its fingers do not follow
                  position commands at the default gain, and it has no mimic constraint), so the arm named poses work there and the
                  gripper ones may not. Pass gz-physics-bullet-featherstone-plugin for gripper poses to close: M02 does, for that reason.

Everything here is simulation: it permits no statement about a physical robot or a limit of one (ADR-009).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def include(package, launch_file, **launch_arguments):
    path = get_package_share_directory(package) + "/launch/" + launch_file
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path), launch_arguments=launch_arguments.items())


def launch_setup(context, *args, **kwargs):
    config = {key: LaunchConfiguration(key).perform(context) for key in ("column", "base_plate", "head_pitch", "base_tilt")}
    sim = LaunchConfiguration("sim").perform(context)
    rviz = LaunchConfiguration("rviz").perform(context)
    if sim not in ("mock", "gazebo"):
        raise RuntimeError(f"sim:={sim}: accepted values are mock, gazebo")
    if sim == "mock":
        # The base is not under ros2_control, so on mock nothing reports its wheel joints and MoveIt warns every second that the
        # robot's complete state is unknown. The stub reports them at rest (the base does not move during manipulation, AD-30).
        stack = [include("openamrobot_upperbody_bringup", "mock.launch.py", rviz="false", **config),
                 Node(package="openamrobot_upperbody_bringup", executable="base_joint_states_stub.py", output="log",
                      parameters=[{"lift_position": float(LaunchConfiguration("lift_position").perform(context))}])]
    else:
        sim_args = {**config, "headless": LaunchConfiguration("headless").perform(context), "rviz": "false"}
        engine = LaunchConfiguration("physics_engine").perform(context)
        if engine:
            sim_args["physics_engine"] = engine
        stack = [include("openamrobot_upperbody_gazebo", "sim.launch.py", **sim_args)]
    move_group = include("openamrobot_upperbody_moveit", "move_group.launch.py", hardware_profile=sim, rviz=rviz, **config)
    return [*stack, move_group]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("column", default_value="dold_v4_350", description="dold_v4_350 | tl3_400 (no supplier data: refused)"),
        DeclareLaunchArgument("base_plate", default_value="bp000", description="bp000 | bp050"),
        DeclareLaunchArgument("head_pitch", default_value="hp25", description="hp15 | hp20 | hp25 | hp30 | hp35"),
        DeclareLaunchArgument("base_tilt", default_value="bt10", description="bt05 | bt10 | bt15"),
        DeclareLaunchArgument("lift_position", default_value="0.0",
                              description="sim:=mock only: lift_joint position (m) the stub reports; 0.0 / 0.175 / 0.350 = L1000 / L1175 / L1350"),
        DeclareLaunchArgument("sim", default_value="mock", description="mock | gazebo"),
        DeclareLaunchArgument("rviz", default_value="false", description="true | false. RViz2 with the MotionPlanning display"),
        DeclareLaunchArgument("headless", default_value="false", description="sim:=gazebo only: true runs gz sim -s (no window)"),
        DeclareLaunchArgument("physics_engine", default_value="",
                              description="sim:=gazebo only: gz-physics plugin; empty = sim.launch.py's default (DART)"),
        OpaqueFunction(function=launch_setup),
    ])
