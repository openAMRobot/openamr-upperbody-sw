# SPDX-License-Identifier: MIT
"""Build 1 scene: the whole OpenAMRobot 2.0 combined model in one Gazebo world.

    ros2 launch openamrobot_build1_scene build1.launch.py [profile:=firmware_sil] [headless:=true] [rviz:=true]

Run profiles, named as in the phase 1 baseline (audits, digital-twin/phase1/run_profiles.md)
  firmware_sil  (default) every device on virtual CAN: vcan_drive, vcan_bms, vcan_aux (classic CAN) and vcan_arm_l,
                vcan_arm_r (CAN FD, MTU 72). The emulators, the host test client or the firmware image in Renode,
                and the unmodified upstream arm driver are in the loop. 1 ms steps, or 1/1500 s with step:=fine
  faults        firmware_sil, for runs that inject faults. Same launch, the name goes into every result
  fast          no device. gz_ros2_control position controllers for arms, grippers and lift, the platform's
                DiffDrive plugin for the base. 1 ms steps
  fine          as fast with 1/1500 s steps and the 750 Hz overlay of openamr-upperbody-sw #66
  geometry      has no physics and is refused here. openamrobot_upperbody_bringup runs it

What it starts
  gz sim           the world generated from config/scene.yaml. Gazebo owns all rigid-body dynamics in every profile
  firmware_sil and faults
    plant, in /sim   the combined model. /sim/controller_manager runs joint_state_broadcaster and the two arm motor
                     emulators. The base DiffDrive plugin is out of the model: each wheel has one velocity input
    base buses       openamrobot_sil (openamr-platform-sw #141): ZLAC8015D, Daly BMS and CTL-017 emulators, the host
                     test client in the STM32 slot, and with stm32:=renode_readonly the firmware image in Renode
    arm buses        openamrobot_openarm_emulator and the unmodified upstream driver openarm_hardware/OpenArmHW in
                     /controller_manager, with both arm and both gripper controllers
    state            base_state_adapter.py (/odom, odom -> base_footprint, base and lift joint states) and
                     wheel_command_adapter.py (the drive emulator's demand, per wheel)
  fast and fine
    plant and control  the combined model with gz_ros2_control in /controller_manager: both arm, both gripper and the
                       lift controller. /cmd_vel, /odom and odom -> base_footprint through the DiffDrive plugin
  every profile
    joint_state_publisher (the one writer of /joint_states), robot_state_publisher (/tf)
    Nav2             openamr-platform-sw's localization and navigation launch files, its scan body filter, this
                     package's map of the scene, and footprint_publisher.py. The docking nodes are not started:
                     dock_trigger would be a second writer of /cmd_vel (conflict SW-01)
    MoveIt 2         move_group for both arms and planning_scene_loader.py with the scene's collision objects
    rtf_monitor.py   the Gazebo real-time factor from /clock on /build1/rtf, and the run identity on /build1/run_info

Args
  profile          firmware_sil | faults | fast | fine (default firmware_sil)
  step             auto | fast | fine (default auto: the profile's own step). firmware_sil and faults only
  physics_engine   gz-physics plugin (default gz-physics-dartsim-plugin: the base drives correctly with it).
                   finger_joint2 then follows finger_joint1 through finger_mimic_follower.py. With
                   gz-physics-bullet-featherstone-plugin the engine applies the mimic constraint, but the base does
                   not turn. The launch prints the known limit of the chosen engine
  stm32            host | renode_readonly (default host), as openamrobot_sil's argument. firmware_sil and faults only
  create_vcan      true | false (default true). false: the five interfaces must exist already
  station          origin | work_shelf | aisle_entry (default origin): where the robot is spawned
  sensors          true | false (default true). false: no rendered sensor, and no Nav2
  extra_sensors    true | false (default true). The RGB-D and wrist cameras this package adds
  nav2, moveit     true | false (default true)
  rviz             true | false (default false)
  headless         true | false (default false)
  driver_rate      auto | Hz (default auto: every second physics step, 500 Hz at 1 ms, 750 Hz at 1/1500 s)
  driver_clock     sim | wall (default sim): the clock of the real driver's controller manager
  sim_state_rate   Hz | step (default 100): rate of /sim/joint_states, the plant's own joint states. step publishes at
                   every physics step, which costs real-time factor. The emulators always run at every step
  replay_drive, replay_bms, replay_aux, replay_arm_l, replay_arm_r
                   a candump -L file. It is played into that bus in place of the bus's emulator
  replay_loop      true | false (default true)

Time follows docs/architecture/sil-topology.md section 6 of openamr-platform-sw #141: Gazebo time on /clock for every
ROS node here, host monotonic time inside the device emulators, Renode virtual time inside the firmware. The clocks are
not synchronised. docs/build1_scene.md says what that means for a slow machine.

Simulation only. No part of this launch is a safety function, and a result from it is not hardware evidence.
"""
import hashlib
import importlib.util
import json
import os
import subprocess
import sys
import tempfile

import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, Shutdown)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from openamrobot_build1_scene import diagnostics
from openamrobot_build1_scene import launch_support as support
from openamrobot_build1_scene import scene as scene_model

PACKAGE = "openamrobot_build1_scene"
EMULATOR = "openamrobot_openarm_emulator"
DESCRIPTION = "openamrobot_upperbody_description"
CONTROL = "openamrobot_upperbody_control"
GAZEBO = "openamrobot_upperbody_gazebo"
MESH_PACKAGES = ("openamrobot_description", "openarm_description", DESCRIPTION)
SIM_CONTROLLER_MANAGER = f"/{support.SIM_NAMESPACE}/controller_manager"
REAL_CONTROLLER_MANAGER = "/controller_manager"
REPLAY_ARGS = {"replay_drive": "vcan_drive", "replay_bms": "vcan_bms", "replay_aux": "vcan_aux",
               "replay_arm_l": "vcan_arm_l", "replay_arm_r": "vcan_arm_r"}
SPAWN_Z = "0.002"


def interface_mtu(interface):
    """MTU of a network interface, or None when it does not exist. 72 is CAN FD, 16 is classic CAN."""
    try:
        with open(os.path.join(os.environ.get("BUILD1_SYS_NET", "/sys/class/net"), interface, "mtu")) as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def bus_problem(mtus):
    """None when the five interfaces fit, else the reason. The arm buses must carry CAN FD."""
    missing = [bus for bus in support.ALL_BUSES if mtus.get(bus) is None]
    if missing:
        return f"virtual CAN interfaces missing: {', '.join(missing)}"
    classic = [bus for bus in support.ARM_BUSES.values() if mtus[bus] != support.CAN_FD_MTU]
    if classic:
        return (f"{', '.join(classic)} do not carry CAN FD (MTU {[mtus[b] for b in classic]}, {support.CAN_FD_MTU} needed). "
                "The arm buses are CAN FD. This launch does not fall back to classic CAN")
    wide = [bus for bus in support.BASE_BUSES if mtus[bus] != support.CLASSIC_CAN_MTU]
    if wide:
        return (f"{', '.join(wide)} are not classic CAN (MTU {[mtus[b] for b in wide]}, {support.CLASSIC_CAN_MTU} needed). "
                "A new vcan interface takes the kernel default, which is 2060 on recent kernels")
    return None


def platform_sil_launch():
    """openamrobot_sil's launch file as a module: its can_nodes() is reused unchanged."""
    path = os.path.join(get_package_share_directory("openamrobot_sil"), "launch", "sil.launch.py")
    spec = importlib.util.spec_from_file_location("openamrobot_sil_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def xacro(path, **mappings):
    command = ["xacro", path, *[f"{key}:={value}" for key, value in mappings.items()]]
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def temp_yaml(prefix, data):
    handle, path = tempfile.mkstemp(prefix=prefix, suffix=".yaml")
    with os.fdopen(handle, "w") as f:
        yaml.safe_dump(data, f)
    return path


def spawner(controllers, controller_manager, extra=()):
    return Node(package="controller_manager", executable="spawner", output="screen",
                arguments=[*controllers, "--controller-manager", controller_manager, "--controller-manager-timeout", "180",
                           "--switch-timeout", "60", "--service-call-timeout", "60", *extra],
                parameters=[{"use_sim_time": True}])


def then(target, next_actions, what):
    """After `target` exits 0, start `next_actions`. Any other exit code stops the launch and says why."""
    def on_exit(event, context):
        if event.returncode == 0:
            return next_actions
        return [LogInfo(msg=f"{what} exited {event.returncode}: the Build 1 stack cannot start"), Shutdown(reason=f"{what} failed")]
    return RegisterEventHandler(OnProcessExit(target_action=target, on_exit=on_exit))


def launch_setup(context):
    value = lambda name: LaunchConfiguration(name).perform(context)  # noqa: E731
    flag = lambda name: value(name) == "true"                        # noqa: E731
    share = get_package_share_directory(PACKAGE)
    scene = scene_model.load(os.path.join(share, "config", "scene.yaml"))
    try:
        run = support.profile(value("profile"), value("step"))
    except ValueError as error:
        return [LogInfo(msg=f"refused: {error}"), Shutdown(reason="profile refused")]
    devices, step, stm32 = run["devices"], run["step"], value("stm32")
    gravity = devices and flag("gravity_feedforward")
    engine = value("physics_engine")
    follower = support.finger_owner(engine) == "follower"
    sensors, nav2 = flag("sensors"), flag("nav2")
    if nav2 and not sensors:
        return [LogInfo(msg="nav2:=true needs sensors:=true: Nav2 has no scan without the rendered LiDAR"),
                Shutdown(reason="nav2 without sensors")]
    replays = support.parse_replays({bus: value(arg) for arg, bus in REPLAY_ARGS.items()}) if devices else {}
    actions, temp_files = [], []

    # 1. The five buses.
    if devices:
        if flag("create_vcan"):
            done = subprocess.run([os.path.join(share, "..", "..", "lib", PACKAGE, "setup_vcan_all.sh")], capture_output=True, text=True)
            actions.append(LogInfo(msg=f"setup_vcan_all.sh (exit {done.returncode}):\n{(done.stdout + done.stderr).strip()}"))
        problem = bus_problem({bus: interface_mtu(bus) for bus in support.ALL_BUSES})
        if problem:
            return [*actions, LogInfo(msg=f"BLOCKED: {problem}. Create them with: ros2 run {PACKAGE} setup_vcan_all.sh"),
                    Shutdown(reason="BLOCKED: virtual CAN unavailable")]
        if stm32 == "renode_readonly" and not os.environ.get("SIL_FW_ELF"):
            return [*actions, LogInfo(msg="BLOCKED: stm32:=renode_readonly needs the firmware image. Run openamrobot_sil's "
                                          "scripts/sil_renode_firmware.sh and export SIL_RENODE, SIL_FW_ELF and SIL_FW_DIR"),
                    Shutdown(reason="BLOCKED: no firmware image")]

    # 2. World, map, descriptions and controller files.
    spawn = scene_model.station(scene, value("station"))
    work_dir = tempfile.mkdtemp(prefix="build1_scene_")
    world_file = os.path.join(work_dir, f"build1_{step}.sdf")
    # Diagnostics only (BUILD1_CONTACT_SENSORS=1): contact sensors on the arm links and gz-sim's Contact system.
    contact_links = diagnostics.contact_links() if diagnostics.enabled(os.environ) else []
    world_text = scene_model.world_sdf(scene, step, sensors=sensors)
    with open(world_file, "w", encoding="utf-8") as f:
        f.write(diagnostics.world_with_contact_system(world_text) if contact_links else world_text)
    map_file = scene_model.write_map(scene, work_dir)
    objects = sorted(scene_model.movable_objects(scene))
    control_share, emulator_share = get_package_share_directory(CONTROL), get_package_share_directory(EMULATOR)
    extra_sensors = "true" if sensors and flag("extra_sensors") else "false"
    wrapper = os.path.join(share, "urdf", "build1_robot.urdf.xacro")
    with open(os.path.join(control_share, "config", "arm_controllers.yaml")) as f:
        arm_controllers = yaml.safe_load(f)
    info = {"profile": run["name"], "physics_step": step, "step_s": support.step_seconds(scene, step), "physics_engine": engine,
            "devices": devices, "station": value("station"), "sensors": sensors, "nav2": nav2, "moveit": flag("moveit"),
            "seed": "none set: a repeated run is not guaranteed to be identical"}

    if devices:
        with open(os.path.join(emulator_share, "config", "sim_controllers.yaml")) as f:
            sim_controllers = support.sim_controllers(yaml.safe_load(f), scene, step, replayed_buses=replays,
                                                      state_rate=0 if value("sim_state_rate") == "step" else int(value("sim_state_rate")))
        sim_controllers_file = temp_yaml("build1_sim_controllers_", sim_controllers)
        driver_rate = support.driver_update_rate(scene, step, value("driver_rate"))
        driver_sim_time = value("driver_clock") == "sim"
        driver_controllers_file = temp_yaml("build1_driver_controllers_", support.driver_controllers(arm_controllers, driver_rate, driver_sim_time,
                                                                                                    gravity_feedforward=gravity))
        temp_files += [sim_controllers_file, driver_controllers_file]
        sim_urdf = support.sim_description(xacro(wrapper, hardware_profile="gazebo", controllers_file=sim_controllers_file,
                                                 devices="true", extra_sensors=extra_sensors,
                                                 finger_follower="true" if follower else "false"))
        # What robot_state_publisher and the real driver read: the real profile with the two bus names.
        root_urdf = xacro(os.path.join(emulator_share, "urdf", "openarm_sil_real.urdf.xacro"), hardware_profile="real",
                          left_can_interface=support.ARM_BUSES["left"], right_can_interface=support.ARM_BUSES["right"], can_fd="true")
        info.update(stm32=stm32, emulator_rate_hz=support.sim_update_rate(scene, step), driver_rate_hz=driver_rate, gravity_feedforward=gravity,
                    driver_clock="gazebo" if driver_sim_time else "wall", replayed_buses=sorted(replays),
                    sim_state_rate=value("sim_state_rate"))
    else:
        overlay = os.path.join(control_share, "config", "sim_overlay.yaml") if support.direct_overlay_needed(step) else ""
        sim_urdf = xacro(wrapper, hardware_profile="gazebo", controllers_file=os.path.join(control_share, "config", "arm_controllers.yaml"),
                         controllers_overlay_file=overlay, devices="false", extra_sensors=extra_sensors,
                         finger_follower="true" if follower else "false")
        root_urdf = sim_urdf
        requested = 750 if overlay else arm_controllers["controller_manager"]["ros__parameters"]["update_rate"]
        info.update(controller_rate_requested_hz=requested, controller_rate_hz=support.effective_rate(support.step_seconds(scene, step), requested))
    if contact_links:
        with_sensors = diagnostics.with_contact_sensors(sim_urdf, contact_links)
        root_urdf = with_sensors if root_urdf is sim_urdf else root_urdf
        sim_urdf = with_sensors
        info["contact_sensors"] = contact_links
        model_copy = os.path.join(os.environ.get("BUILD1_LOG_DIR", work_dir), "diag_plant_model.urdf")
        os.makedirs(os.path.dirname(model_copy), exist_ok=True)
        with open(model_copy, "w", encoding="utf-8") as f:
            f.write(sim_urdf)
        actions.append(LogInfo(msg=f"diagnostics: plant model with contact sensors written to {model_copy}"))
    info["model_sha256"] = hashlib.sha256(sim_urdf.encode()).hexdigest()
    info["finger_joint2_owner"] = "finger_mimic_follower.py" if follower else "mimic constraint of the engine"
    actions.append(LogInfo(msg="Build 1 scene: " + json.dumps(info, sort_keys=True)))
    for note in support.engine_notes(engine, devices):
        actions.append(LogInfo(msg=f"KNOWN LIMIT: {note}"))

    # 3. The simulator and the two bridges.
    for package in MESH_PACKAGES:
        actions.append(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.dirname(get_package_share_directory(package))))
    actions.append(AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                                             os.path.join(get_package_share_directory("openamrobot_docking"), "models")))
    headless = flag("headless")
    gz_args = f"-r {'-s --headless-rendering ' if headless else ''}-v 2 --physics-engine {engine} {world_file}"
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": gz_args, "on_exit_shutdown": "true"}.items()))
    control_bridge_file = temp_yaml("build1_control_bridge_", support.control_bridge(
        objects, devices=devices, replay_drive="vcan_drive" in replays, replay_aux="vcan_aux" in replays, follower=follower))
    temp_files.append(control_bridge_file)
    actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="build1_control_bridge", output="screen",
                        parameters=[{"config_file": control_bridge_file}]))
    if contact_links:
        contact_bridge_file = temp_yaml("build1_contact_bridge_", diagnostics.contact_bridge(contact_links))
        temp_files.append(contact_bridge_file)
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="build1_contact_bridge", output="screen",
                            parameters=[{"config_file": contact_bridge_file}]))
    if sensors:
        sensor_bridge_file = temp_yaml("build1_sensor_bridge_", support.sensor_bridge(flag("extra_sensors")))
        temp_files.append(sensor_bridge_file)
        actions.append(Node(package="ros_gz_bridge", executable="parameter_bridge", name="build1_sensor_bridge", output="screen",
                            parameters=[{"config_file": sensor_bridge_file}]))

    # 4. The plant. Gazebo owns the dynamics of every joint. With devices its controller manager lives in /sim.
    spawn_arguments = ["-name", "openamrobot", "-x", str(spawn[0]), "-y", str(spawn[1]), "-z", SPAWN_Z, "-Y", str(spawn[2])]
    root_state_publisher = Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
                                output="screen", parameters=[{"use_sim_time": True, "robot_description": ParameterValue(root_urdf, value_type=str)}])
    actions.append(root_state_publisher)
    actions.append(Node(package="joint_state_publisher", executable="joint_state_publisher", name="joint_state_publisher", output="screen",
                        parameters=[{"use_sim_time": True, "rate": 50, "source_list": ["/base/joint_states", "/arm/joint_states"],
                                     "robot_description": ParameterValue(root_urdf, value_type=str)}]))
    arm_states = ["--controller-ros-args", "-r joint_states:=/arm/joint_states"]
    if devices:
        actions.append(Node(package="robot_state_publisher", executable="robot_state_publisher", name="robot_state_publisher",
                            namespace=support.SIM_NAMESPACE, output="screen",
                            parameters=[{"use_sim_time": True, "robot_description": ParameterValue(sim_urdf, value_type=str)}],
                            remappings=[("/tf", f"/{support.SIM_NAMESPACE}/tf"), ("/tf_static", f"/{support.SIM_NAMESPACE}/tf_static")]))
        actions.append(Node(package="ros_gz_sim", executable="create", output="screen",
                            arguments=[*spawn_arguments, "-topic", f"/{support.SIM_NAMESPACE}/robot_description"]))
        sim_broadcaster = spawner(["joint_state_broadcaster"], SIM_CONTROLLER_MANAGER)
        actions.append(sim_broadcaster)

        # 5. What would run on the robot, in the root namespace.
        emulators = support.emulators_to_spawn(replays)
        real_control = Node(package="controller_manager", executable="ros2_control_node", output="screen",
                            parameters=[driver_controllers_file, {"use_sim_time": driver_sim_time}])
        real_broadcaster = spawner(["joint_state_broadcaster"], REAL_CONTROLLER_MANAGER, extra=arm_states)
        real_rest = spawner(support.driver_controllers_to_spawn(gravity), REAL_CONTROLLER_MANAGER)
        if gravity:
            actions.append(Node(package=PACKAGE, executable="gravity_feedforward.py", name="gravity_feedforward", output="screen",
                                parameters=[{"use_sim_time": True, "robot_description": ParameterValue(root_urdf, value_type=str)}]))
        if emulators:
            sim_rest = spawner(emulators, SIM_CONTROLLER_MANAGER)
            actions.append(then(sim_broadcaster, [sim_rest], "the simulator's joint_state_broadcaster spawner"))
            actions.append(then(sim_rest, [real_control, real_broadcaster], "the arm emulator spawner"))
        else:
            actions.append(then(sim_broadcaster, [real_control, real_broadcaster], "the simulator's joint_state_broadcaster spawner"))
        actions.append(then(real_broadcaster, [real_rest], "the real driver's joint_state_broadcaster spawner"))

        # The base buses: openamrobot_sil's own node list, minus the emulator of a replayed bus.
        sil = platform_sil_launch()
        sil_config = os.path.join(get_package_share_directory("openamrobot_sil"), "config", "sil.yaml")
        skipped = {support.BASE_EMULATOR_OF_BUS[bus] for bus in replays if bus in support.BASE_EMULATOR_OF_BUS}
        for node in sil.can_nodes(sil_config, stm32, True, sil.default_renode_args()):
            if getattr(node, "node_executable", None) in skipped:
                continue
            actions.append(node)
        for bus, path in sorted(replays.items()):
            actions.append(Node(package=PACKAGE, executable="can_replay.py", name=f"can_replay_{bus}", output="screen",
                                arguments=["--interface", bus, "--file", path, *(["--loop"] if flag("replay_loop") else []),
                                           *(["--fd"] if bus in support.ARM_BUSES.values() else [])]))
        actions.append(Node(package=PACKAGE, executable="base_state_adapter.py", name="base_state_adapter", output="screen",
                            parameters=[{"use_sim_time": True, "config_file": sil_config,
                                         "source": "firmware_wheel_speeds" if stm32 == "renode_readonly" else "host_client"}]))
        actions.append(Node(package=PACKAGE, executable="wheel_command_adapter.py", name="wheel_command_adapter", output="screen",
                            parameters=[{"use_sim_time": True, "config_file": sil_config}]))
    else:
        actions.append(Node(package="ros_gz_sim", executable="create", output="screen", arguments=[*spawn_arguments, "-topic", "/robot_description"]))
        broadcaster = spawner(["joint_state_broadcaster"], REAL_CONTROLLER_MANAGER, extra=arm_states)
        actions.append(broadcaster)
        actions.append(then(broadcaster, [spawner([*support.DRIVER_CONTROLLERS, "lift_controller"], REAL_CONTROLLER_MANAGER)],
                            "the joint_state_broadcaster spawner"))
    if follower:
        actions.append(Node(package=PACKAGE, executable="finger_mimic_follower.py", name="finger_mimic_follower", output="screen",
                            parameters=[{"use_sim_time": True}]))
    actions.append(Node(package=PACKAGE, executable="rtf_monitor.py", name="rtf_monitor", output="screen",
                        parameters=[{"step_s": support.step_seconds(scene, step), "run_info": json.dumps(info, sort_keys=True)}]))

    # 6. Nav2: the platform's launch files and parameters, this scene's map, a footprint that follows the arms.
    if nav2:
        nav2_share = get_package_share_directory("openamrobot_nav2")
        with open(os.path.join(nav2_share, "config", "nav2_params.yaml")) as f:
            platform_params = yaml.safe_load(f)
        with open(os.path.join(share, "config", "nav2_overrides.yaml")) as f:
            overrides = yaml.safe_load(f) or {}
        nav2_file = temp_yaml("build1_nav2_", support.nav2_params(platform_params, overrides, spawn))
        temp_files.append(nav2_file)
        actions.append(Node(package="openamrobot_perception", executable="scan_body_filter", name="scan_body_filter", output="screen",
                            parameters=[os.path.join(nav2_share, "config", "scan_body_filter_sim.yaml"), {"use_sim_time": True}]))
        for launch_file, arguments in (("localization_launch.py", {"map": map_file}), ("navigation_launch.py", {})):
            actions.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", launch_file)),
                launch_arguments={"use_sim_time": "true", "params_file": nav2_file, **arguments}.items()))
        actions.append(Node(package=PACKAGE, executable="footprint_publisher.py", name="footprint_publisher", output="screen",
                            parameters=[{"use_sim_time": True, "nav2_params_file": nav2_file}]))

    # 7. MoveIt 2. With devices the lift is not a MoveIt controller: it moves through CTL-017 only (SW-06).
    rviz_parameters = [{"use_sim_time": True}]
    if flag("moveit"):
        from openamrobot_upperbody_moveit.moveit_config import moveit_config
        config = moveit_config(hardware_profile="gazebo")
        move_group_parameters = support.moveit_controllers_without_lift(config.to_dict()) if devices else config.to_dict()
        actions.append(Node(package="moveit_ros_move_group", executable="move_group", output="screen",
                            parameters=[move_group_parameters,
                                        {"trajectory_execution.allowed_execution_duration_scaling": 3.0,
                                         "trajectory_execution.allowed_goal_duration_margin": 5.0,
                                         "trajectory_execution.allowed_start_tolerance": 0.05,
                                         "use_sim_time": True, "publish_robot_description_semantic": True}],
                            remappings=[("joint_states", "/joint_states_clamped")]))
        actions.append(Node(package="openamrobot_upperbody_moveit", executable="joint_state_clamp.py", name="joint_state_clamp",
                            output="screen", parameters=[{"robot_description": config.robot_description["robot_description"],
                                                          "use_sim_time": True,
                                                          # with devices the joint states cross the Damiao position field
                                                          **({"epsilon": support.DAMIAO_POSITION_RESOLUTION} if devices else {})}]))
        actions.append(Node(package=PACKAGE, executable="planning_scene_loader.py", name="planning_scene_loader", output="screen",
                            parameters=[{"use_sim_time": True, "scene_file": os.path.join(share, "config", "scene.yaml"),
                                         "spawn_pose": [float(v) for v in spawn]}]))
        rviz_parameters += [config.robot_description, config.robot_description_semantic, config.robot_description_kinematics,
                            config.planning_pipelines, config.joint_limits]
    if flag("rviz"):
        actions.append(Node(package="rviz2", executable="rviz2", output="log", arguments=["-d", os.path.join(share, "rviz", "build1.rviz")],
                            parameters=rviz_parameters))

    # 8. Stopping the simulator: openamrobot_upperbody_gazebo's helper, once per launch.
    gazebo_shell = {"pid": None, "stop_requested": False}

    def remember_the_shell(event, context):
        if event.process_name.startswith("gazebo") and gazebo_shell["pid"] is None:
            gazebo_shell["pid"] = event.pid

    def stop_the_server_once(event, context):
        if gazebo_shell["stop_requested"]:
            return None
        gazebo_shell["stop_requested"] = True
        for path in temp_files:
            try:
                os.unlink(path)
            except OSError:
                pass
        try:
            helper = os.path.join(get_package_share_directory(GAZEBO), "launch", "sim.launch.py")
        except PackageNotFoundError:
            return None
        subprocess.Popen([sys.executable, helper, "--stop-gz-server", str(gazebo_shell["pid"] or 0)],
                         stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        return None

    actions.append(RegisterEventHandler(OnProcessStart(on_start=remember_the_shell)))
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=stop_the_server_once)))
    return actions


def generate_launch_description():
    boolean = ["true", "false"]
    arguments = [
        DeclareLaunchArgument("profile", default_value="firmware_sil", choices=["firmware_sil", "faults", "fast", "fine", "geometry"],
                              description="Run profile of the phase 1 baseline. firmware_sil and faults: devices on virtual CAN. "
                                          "fast and fine: no device. geometry is refused: it has no physics"),
        DeclareLaunchArgument("step", default_value="auto", choices=["auto", "fast", "fine"],
                              description="Physics step of firmware_sil and faults. auto and fast: 1 ms. fine: 1/1500 s"),
        DeclareLaunchArgument("physics_engine", default_value=support.DEFAULT_ENGINE,
                              description="gz-physics plugin. DART drives the base correctly. Bullet Featherstone applies the "
                                          "finger mimic constraint, but the base does not turn with it"),
        DeclareLaunchArgument("stm32", default_value="host", choices=["host", "renode_readonly"],
                              description="host: the host test client plays the STM32 CAN role. renode_readonly: the firmware "
                                          "image in Renode reads the drive, the host test client commands it"),
        DeclareLaunchArgument("create_vcan", default_value="true", choices=boolean,
                              description="Run setup_vcan_all.sh first (needs root or passwordless sudo)"),
        DeclareLaunchArgument("station", default_value="origin", choices=["origin", "work_shelf", "aisle_entry"],
                              description="Where the robot is spawned (config/scene.yaml, stations)"),
        DeclareLaunchArgument("sensors", default_value="true", choices=boolean, description="Render the sensors"),
        DeclareLaunchArgument("extra_sensors", default_value="true", choices=boolean,
                              description="The RGB-D and wrist cameras this package adds to the model"),
        DeclareLaunchArgument("nav2", default_value="true", choices=boolean, description="Start Nav2"),
        DeclareLaunchArgument("moveit", default_value="true", choices=boolean, description="Start MoveIt 2"),
        DeclareLaunchArgument("rviz", default_value="false", choices=boolean, description="Start RViz with rviz/build1.rviz"),
        DeclareLaunchArgument("headless", default_value="false", choices=boolean, description="gz sim -s: server only, no window"),
        DeclareLaunchArgument("driver_rate", default_value="auto", description="auto | Hz of the real driver's controller manager"),
        DeclareLaunchArgument("gravity_feedforward", default_value="true", choices=boolean,
                              description="With devices: send the arms' gravity torque through the driver's effort interface"),
        DeclareLaunchArgument("sim_state_rate", default_value="100",
                              description="Hz of /sim/joint_states, or step for every physics step. Device profiles only"),
        DeclareLaunchArgument("driver_clock", default_value="sim", choices=["sim", "wall"],
                              description="Clock of the real driver's controller manager"),
        DeclareLaunchArgument("replay_loop", default_value="true", choices=boolean, description="Repeat a replayed trace"),
    ]
    for name, bus in REPLAY_ARGS.items():
        arguments.append(DeclareLaunchArgument(name, default_value="",
                                               description=f"candump -L file played into {bus} in place of its emulator"))
    return LaunchDescription([*arguments, OpaqueFunction(function=launch_setup)])
