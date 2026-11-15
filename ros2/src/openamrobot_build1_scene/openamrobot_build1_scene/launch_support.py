# SPDX-License-Identifier: MIT
"""What build1.launch.py computes before it starts anything. No ROS import: the tests call these functions directly.

Simulation only. Nothing here is a safety function.
"""
import copy
import re

BASE_BUSES = ("vcan_drive", "vcan_bms", "vcan_aux")
ARM_BUSES = {"left": "vcan_arm_l", "right": "vcan_arm_r"}
ALL_BUSES = BASE_BUSES + tuple(ARM_BUSES.values())
CAN_FD_MTU = 72
CLASSIC_CAN_MTU = 16

# Run profiles, named as in the phase 1 baseline (audits, digital-twin/phase1/run_profiles.md).
# geometry has no physics, so it is not a profile of this Gazebo scene: openamrobot_upperbody_bringup runs it.
#   devices  the emulators on virtual CAN and the real driver are in the loop
#   step     the physics profile of config/scene.yaml the run profile uses unless the launch argument step says otherwise
RUN_PROFILES = {
    "fast": {"devices": False, "step": "fast"},
    "fine": {"devices": False, "step": "fine"},
    "firmware_sil": {"devices": True, "step": "fast"},
    "faults": {"devices": True, "step": "fast"},
}
REFUSED_PROFILES = {"geometry": "geometry has no physics. Run it with: ros2 launch openamrobot_upperbody_bringup moveit.launch.py sim:=mock"}
# finger_joint2 is a mimic joint that no controller owns (conflict SW-09). Its owner depends on the engine:
# Bullet Featherstone applies the mimic constraint itself. Any other engine gets finger_mimic_follower.py.
MIMIC_ENGINES = ("gz-physics-bullet-featherstone-plugin",)
DEFAULT_ENGINE = "gz-physics-dartsim-plugin"

SIM_NAMESPACE = "sim"
DIFF_DRIVE = re.compile(r'\s*<plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">.*?</plugin>', re.S)
GZ_PLUGIN_OPEN = '<plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">'
LIFT_SYSTEM = re.compile(r'\s*<ros2_control name="openamr_lift_gazebo" type="system">.*?</ros2_control>', re.S)

# Emulator nodes of openamrobot_sil by the bus they sit on. A replayed bus starts none of them.
BASE_EMULATOR_OF_BUS = {"vcan_drive": "zlac8015d_emulator", "vcan_bms": "daly_bms_emulator", "vcan_aux": "ctl017_emulator"}
ARM_EMULATOR_OF_BUS = {"vcan_arm_l": "left_arm_emulator", "vcan_arm_r": "right_arm_emulator"}

DRIVER_CONTROLLERS = ("left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller")


def profile(name, step="auto"):
    """-> {"name", "devices", "step"} of a run profile. `step` auto takes the profile's own physics step."""
    if name in REFUSED_PROFILES:
        raise ValueError(REFUSED_PROFILES[name])
    if name not in RUN_PROFILES:
        raise ValueError(f"profile {name!r}: one of {', '.join(RUN_PROFILES)}")
    if step not in ("auto", "fast", "fine"):
        raise ValueError(f"step {step!r}: auto, fast or fine")
    entry = dict(RUN_PROFILES[name], name=name)
    if step != "auto":
        if not entry["devices"] and step != entry["step"]:
            raise ValueError(f"profile {name} has its step in its name. Use profile:={step}")
        entry["step"] = step
    return entry


def finger_owner(engine):
    """Who owns openarm_*_finger_joint2 with this physics engine: "engine" (mimic constraint) or "follower"."""
    return "engine" if engine in MIMIC_ENGINES else "follower"


def engine_notes(engine, devices):
    """What this engine cannot do with the pinned models, measured on 15 Nov 2026. -> list of texts for the log."""
    notes = []
    if engine in MIMIC_ENGINES:
        notes.append("With Bullet Featherstone the base does not turn: 0 degrees of ground truth yaw for 172 degrees of odometry, "
                     "and 0.87 m of travel for 1.20 m of odometry. Do not use this engine for a run that drives. "
                     "docs/build1_scene.md has the measurement")
    elif not devices:
        notes.append("With DART the grippers do not move in the fast and fine profiles: the position servo of gz_ros2_control "
                     "stalls on the finger joints (openamrobot_upperbody_gazebo README). For gripper work without devices pass "
                     f"physics_engine:={MIMIC_ENGINES[0]} and do not drive")
    return notes


def sim_description(urdf):
    """The plant URDF of the device profiles (firmware_sil, faults), from the expanded gazebo-profile model.

    Three edits, all outside links and joints:
    1. the simulator's controller manager moves to /sim, as openamrobot_openarm_emulator does;
    2. the lift leaves ros2_control. gz_ros2_control holds every joint it owns and no controller claims
       (hold_joints), so a lift left there would have two writers: that hold and the CTL-017 path (SW-06);
    3. the DiffDrive plugin of the base is taken out. The wheels get one velocity input each
       (urdf/build1_robot.urdf.xacro), so the drive emulator alone shapes the wheel command (SW-03).
    """
    if urdf.count(GZ_PLUGIN_OPEN) != 1:
        raise RuntimeError("the gz_ros2_control plugin element was not found exactly once in the model")
    urdf = urdf.replace(GZ_PLUGIN_OPEN, f"{GZ_PLUGIN_OPEN}<ros><namespace>/{SIM_NAMESPACE}</namespace></ros>")
    urdf, removed = LIFT_SYSTEM.subn("", urdf)
    if removed != 1:
        raise RuntimeError("the lift ros2_control system openamr_lift_gazebo was not found exactly once in the model")
    urdf, removed = DIFF_DRIVE.subn("", urdf)
    if removed != 1:
        raise RuntimeError("the DiffDrive plugin of the base was not found exactly once in the model")
    return urdf


def step_seconds(scene, profile):
    return float(scene["physics_profiles"][profile]["max_step_size"])


def sim_update_rate(scene, profile):
    """The emulators run once per physics step."""
    return int(round(1.0 / step_seconds(scene, profile)))


def effective_rate(step_s, requested_hz):
    """The rate gz_ros2_control really runs a controller manager at: it updates on the first physics step at or
    after each period. 750 Hz asked on a 1 ms step gives 500 Hz. Same formula as openamr-upperbody-sw #66."""
    step_ns = int(round(step_s * 1e9))
    period_ns = int(1e9 / requested_hz)
    return 1e9 / (step_ns * max(1, -(-period_ns // step_ns)))


def driver_update_rate(scene, profile, requested="auto"):
    """Rate of the real driver's controller manager. auto: every second physics step.
    fine gives 750 Hz, the rate upstream runs the arm at. fast gives 500 Hz."""
    if requested != "auto":
        return int(requested)
    return sim_update_rate(scene, profile) // 2


def direct_overlay_needed(step):
    """The 750 Hz overlay of openamr-upperbody-sw #66 belongs to the fine step."""
    return step == "fine"


def sim_controllers(source, scene, profile, replayed_buses=(), supply_voltage=25.6, timing_model=True):
    """The simulator-side controller file: openamrobot_openarm_emulator's, without the lift controller,
    with the rate of the physics profile, and without the emulator of a replayed arm bus."""
    config = copy.deepcopy(source)
    manager = config["/**/controller_manager"]["ros__parameters"]
    manager["update_rate"] = sim_update_rate(scene, profile)
    manager.pop("lift_controller", None)
    config.pop("/**/lift_controller", None)
    for side, interface in ARM_BUSES.items():
        name = f"{side}_arm_emulator"
        if interface in replayed_buses:
            manager.pop(name, None)
            config.pop(f"/**/{name}", None)
            continue
        parameters = config[f"/**/{name}"]["ros__parameters"]
        parameters["can_interface"] = interface
        parameters["supply_voltage"] = float(supply_voltage)
        parameters["timing"]["model"] = bool(timing_model)
    return config


def driver_controllers(source, rate, use_sim_time):
    """The real driver's controller file: arm_controllers.yaml without the lift controller. The lift has no
    ros2_control system in the real profile. Its only command path is CAN3."""
    config = copy.deepcopy(source)
    manager = config["controller_manager"]["ros__parameters"]
    manager["update_rate"] = int(rate)
    manager["use_sim_time"] = bool(use_sim_time)
    manager.pop("lift_controller", None)
    config.pop("lift_controller", None)
    return config


def emulators_to_spawn(replayed_buses=()):
    return [name for bus, name in ARM_EMULATOR_OF_BUS.items() if bus not in replayed_buses]


def parse_replays(values):
    """{bus: candump file} from the replay_* launch arguments. Empty strings mean no replay."""
    replays = {}
    for bus, path in values.items():
        if bus not in ALL_BUSES:
            raise ValueError(f"{bus} is not a Build 1 bus: {', '.join(ALL_BUSES)}")
        if path:
            replays[bus] = path
    return replays


def deep_merge(base, override):
    out = copy.deepcopy(base)
    for key, item in override.items():
        if isinstance(item, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], item)
        else:
            out[key] = copy.deepcopy(item)
    return out


def nav2_params(platform_params, overrides, spawn_pose):
    """openamr-platform-sw's nav2_params.yaml with this package's overrides and the spawn pose for AMCL."""
    params = deep_merge(platform_params, overrides or {})
    amcl = params.setdefault("amcl", {}).setdefault("ros__parameters", {})
    amcl["set_initial_pose"] = True
    amcl["initial_pose"] = {"x": float(spawn_pose[0]), "y": float(spawn_pose[1]), "z": 0.0, "yaw": float(spawn_pose[2])}
    return params


def moveit_controllers_without_lift(parameters):
    """MoveIt's controller list without lift_controller: in Build 1 the lift moves through CTL-017 only."""
    out = copy.deepcopy(parameters)
    manager = out.get("moveit_simple_controller_manager", {})
    names = [name for name in manager.get("controller_names", []) if name != "lift_controller"]
    manager["controller_names"] = names
    manager.pop("lift_controller", None)
    return out


def _entry(ros, gz, ros_type, gz_type, direction):
    return {"ros_topic_name": ros, "gz_topic_name": gz, "ros_type_name": ros_type, "gz_type_name": gz_type, "direction": direction}


GROUND_TRUTH_FINGERS = "/build1/ground_truth/finger_joint_states"


def _ground_truth(objects):
    entries = [_entry("/build1/ground_truth/odom", "/build1/gz/ground_truth_odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
               _entry(GROUND_TRUTH_FINGERS, "/build1/gz/finger_joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS")]
    for name in objects:
        entries.append(_entry(f"/build1/ground_truth/{name}/pose", f"/model/{name}/pose",
                              "geometry_msgs/msg/PoseStamped", "gz.msgs.Pose", "GZ_TO_ROS"))
    return entries


def control_bridge(objects, devices=True, replay_drive=False, replay_aux=False, follower=False):
    """Bridge entries of the plant interface: clock, the actuator inputs, feedback and ground truth.

    With devices (firmware_sil, faults): /cmd_vel is not bridged. The only path to the wheels is CAN1 and the drive
    emulator, whose per-wheel demand arrives on the two wheel inputs. Nothing of Gazebo's own odometry is bridged.
    Without devices (fast, fine): the base is the platform's DiffDrive plugin, bridged as openamr-platform-sw does:
    /cmd_vel in, /odom and the odom -> base_footprint transform out.
    The real-time factor is not bridged: rtf_monitor.py derives it from /clock."""
    entries = [_entry("/clock", "/clock", "rosgraph_msgs/msg/Clock", "gz.msgs.Clock", "GZ_TO_ROS")]
    if devices:
        entries.append(_entry("/sil/gz/joint_states", "/sil/gz/joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS"))
        if not replay_drive:
            for side in ("left", "right"):
                entries.append(_entry(f"/build1/drive/{side}_wheel_cmd_vel", f"/build1/gz/{side}_wheel_cmd_vel",
                                      "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
        if not replay_aux:
            entries.append(_entry("/sil/lift/gz_cmd_vel", "/sil/gz/lift_cmd_vel", "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
    else:
        entries += [_entry("/cmd_vel", "/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "ROS_TO_GZ"),
                    _entry("/odom", "/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS"),
                    _entry("/tf", "/tf", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS"),
                    _entry("/base/joint_states", "/joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS")]
    if follower:
        for side in ("left", "right"):
            entries.append(_entry(f"/build1/fingers/{side}_finger2_cmd_pos", f"/build1/gz/{side}_finger2_cmd_pos",
                                  "std_msgs/msg/Float64", "gz.msgs.Double", "ROS_TO_GZ"))
    return entries + _ground_truth(objects)


def sensor_bridge(extra_sensors=True):
    """Bridge entries of every sensor. One process carries them all, so that the stale sensor fault can stop it."""
    scan = ("sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan")
    image = ("sensor_msgs/msg/Image", "gz.msgs.Image")
    info = ("sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo")
    entries = [_entry("/scan", "/scan", *scan, "GZ_TO_ROS"),
               _entry("/imu", "imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS"),
               _entry("/rgb_image", "/rgb_image", *image, "GZ_TO_ROS"),
               _entry("/camera_info", "/camera_info", *info, "GZ_TO_ROS")]
    for topic in ("/tof/front", "/tof/rear", "/ultrasonic/front_left", "/ultrasonic/front_right",
                  "/ultrasonic/rear_left", "/ultrasonic/rear_right"):
        entries.append(_entry(topic, topic, *scan, "GZ_TO_ROS"))
    if extra_sensors:
        for camera in ("base_camera", "head_camera"):
            entries += [_entry(f"/{camera}/rgbd/image", f"/{camera}/rgbd/image", *image, "GZ_TO_ROS"),
                        _entry(f"/{camera}/rgbd/depth_image", f"/{camera}/rgbd/depth_image", *image, "GZ_TO_ROS"),
                        _entry(f"/{camera}/rgbd/camera_info", f"/{camera}/rgbd/camera_info", *info, "GZ_TO_ROS")]
        for side in ("left", "right"):
            entries += [_entry(f"/{side}_wrist_camera/image", f"/{side}_wrist_camera/image", *image, "GZ_TO_ROS"),
                        _entry(f"/{side}_wrist_camera/camera_info", f"/{side}_wrist_camera/camera_info", *info, "GZ_TO_ROS")]
    return entries


def bridge_topics(entries, direction):
    return [entry["ros_topic_name"] for entry in entries if entry["direction"] == direction]
