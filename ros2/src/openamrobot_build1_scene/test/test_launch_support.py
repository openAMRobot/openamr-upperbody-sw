# SPDX-License-Identifier: MIT
"""What build1.launch.py computes before it starts anything: profiles, the plant model, controller files, bridges."""
import re

import pytest

from conftest import load_yaml, package_path
from openamrobot_build1_scene import launch_support as support


def test_run_profiles_are_those_of_the_baseline():
    assert set(support.RUN_PROFILES) | set(support.REFUSED_PROFILES) == {"geometry", "fast", "fine", "firmware_sil", "faults"}
    assert support.profile("fast") == {"name": "fast", "devices": False, "step": "fast"}
    assert support.profile("fine") == {"name": "fine", "devices": False, "step": "fine"}
    assert support.profile("firmware_sil")["devices"] and support.profile("firmware_sil")["step"] == "fast"
    assert support.profile("faults") == dict(support.profile("firmware_sil"), name="faults")
    assert support.profile("firmware_sil", "fine")["step"] == "fine"


def test_geometry_and_unknown_profiles_are_refused():
    with pytest.raises(ValueError, match="no physics"):
        support.profile("geometry")
    with pytest.raises(ValueError):
        support.profile("host")
    with pytest.raises(ValueError, match="step in its name"):
        support.profile("fast", "fine")


def test_launch_file_declares_the_profile_names():
    with open(package_path("launch", "build1.launch.py"), encoding="utf-8") as f:
        text = f.read()
    declared = re.search(r'DeclareLaunchArgument\("profile", default_value="(\w+)", choices=\[([^\]]+)\]', text)
    assert declared.group(1) == "firmware_sil"
    assert set(re.findall(r'"(\w+)"', declared.group(2))) == {"firmware_sil", "faults", "fast", "fine", "geometry"}
    assert "dock_trigger" in text and "openamrobot_docking.launch.py" not in text, "the docking nodes must not be started (SW-01)"


def test_device_plant_moves_the_controller_manager_to_sim(devices_urdf):
    urdf = support.sim_description(devices_urdf)
    assert "<ros><namespace>/sim</namespace></ros>" in urdf


def test_sw06_lift_has_no_ros2_control_interface_with_devices(devices_urdf, direct_urdf):
    assert "openamr_lift_gazebo" in devices_urdf, "the model changed: the lift system is expected before the edit"
    urdf = support.sim_description(devices_urdf)
    assert "openamr_lift_gazebo" not in urdf
    assert not re.search(r'<ros2_control.*?<joint name="lift_joint">', urdf, re.S)
    assert urdf.count("<joint_name>lift_joint</joint_name>") == 2          # the velocity input and the state output
    assert "openamr_lift_gazebo" in direct_urdf and "/sil/gz/lift_cmd_vel" not in direct_urdf


def test_sw03_wheels_have_one_input_each_and_no_diff_drive_with_devices(devices_urdf, direct_urdf):
    urdf = support.sim_description(devices_urdf)
    assert "gz-sim-diff-drive-system" not in urdf
    for side in ("left", "right"):
        assert urdf.count(f"<topic>/build1/gz/{side}_wheel_cmd_vel</topic>") == 1
    assert direct_urdf.count("gz-sim-diff-drive-system") == 1 and "wheel_cmd_vel" not in direct_urdf


def test_plant_edits_touch_no_link_joint_limit_or_mass(devices_urdf, direct_urdf):
    def kinematics(urdf):
        body = re.sub(r"<gazebo.*?</gazebo>", "", urdf, flags=re.S)
        body = re.sub(r"<ros2_control.*?</ros2_control>", "", body, flags=re.S)
        return re.sub(r"\s+", " ", re.sub(r"<!--.*?-->", "", body, flags=re.S))
    assert kinematics(support.sim_description(devices_urdf)) == kinematics(direct_urdf)


def test_missing_plugin_elements_stop_the_launch():
    with pytest.raises(RuntimeError):
        support.sim_description("<robot/>")


def test_sw09_finger_joint2_always_has_exactly_one_owner(devices_urdf):
    assert support.finger_owner("gz-physics-bullet-featherstone-plugin") == "engine"
    assert support.finger_owner("gz-physics-dartsim-plugin") == "follower"
    assert support.finger_owner(support.DEFAULT_ENGINE) == "follower"
    for side in ("left", "right"):
        assert devices_urdf.count(f"<joint_name>openarm_{side}_finger_joint2</joint_name>") == 2      # controller and ground truth
        assert devices_urdf.count(f"/build1/gz/{side}_finger2_cmd_pos") == 1
    from conftest import expand
    assert "finger2_cmd_pos" not in expand(devices="true", finger_follower="false"), "with the engine's mimic the follower must be out"


def test_each_engine_states_its_known_limit():
    bullet = support.engine_notes("gz-physics-bullet-featherstone-plugin", True)
    assert len(bullet) == 1 and "does not turn" in bullet[0]
    assert support.engine_notes("gz-physics-dartsim-plugin", True) == []
    assert "grippers do not move" in support.engine_notes("gz-physics-dartsim-plugin", False)[0]


def test_rates_follow_the_physics_step(scene):
    assert support.sim_update_rate(scene, "fast") == 1000 and support.sim_update_rate(scene, "fine") == 1500
    assert support.driver_update_rate(scene, "fast") == 500 and support.driver_update_rate(scene, "fine") == 750
    assert support.driver_update_rate(scene, "fine", "100") == 100
    assert support.effective_rate(0.001, 750) == pytest.approx(500.0)
    assert support.effective_rate(0.00066667, 750) == pytest.approx(750.0, abs=0.01)
    assert support.effective_rate(0.001, 100) == pytest.approx(100.0)
    assert support.direct_overlay_needed("fine") and not support.direct_overlay_needed("fast")


EMULATOR_CONTROLLERS = {
    "/**/controller_manager": {"ros__parameters": {"update_rate": 1000, "joint_state_broadcaster": {}, "lift_controller": {},
                                                   "left_arm_emulator": {}, "right_arm_emulator": {}}},
    "/**/lift_controller": {"ros__parameters": {"joints": ["lift_joint"]}},
    "/**/left_arm_emulator": {"ros__parameters": {"can_interface": "x", "supply_voltage": 0.0, "timing": {"model": False}}},
    "/**/right_arm_emulator": {"ros__parameters": {"can_interface": "x", "supply_voltage": 0.0, "timing": {"model": False}}},
}


def test_simulator_controllers_have_no_lift_controller_and_the_step_rate(scene):
    config = support.sim_controllers(EMULATOR_CONTROLLERS, scene, "fine")
    manager = config["/**/controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 1500 and "lift_controller" not in manager and "/**/lift_controller" not in config
    assert config["/**/left_arm_emulator"]["ros__parameters"]["can_interface"] == "vcan_arm_l"
    assert config["/**/right_arm_emulator"]["ros__parameters"]["can_interface"] == "vcan_arm_r"
    assert config["/**/left_arm_emulator"]["ros__parameters"]["timing"]["model"] is True
    assert EMULATOR_CONTROLLERS["/**/controller_manager"]["ros__parameters"]["update_rate"] == 1000, "the source must not be edited"


def test_plant_joint_state_rate_can_be_lowered_without_touching_the_emulators(scene):
    config = support.sim_controllers(EMULATOR_CONTROLLERS, scene, "fast", state_rate=100)
    assert config["/**/joint_state_broadcaster"] == {"ros__parameters": {"update_rate": 100}}
    assert config["/**/controller_manager"]["ros__parameters"]["update_rate"] == 1000
    assert "/**/joint_state_broadcaster" not in support.sim_controllers(EMULATOR_CONTROLLERS, scene, "fast")


def test_a_replayed_arm_bus_has_no_emulator(scene):
    config = support.sim_controllers(EMULATOR_CONTROLLERS, scene, "fast", replayed_buses={"vcan_arm_l": "trace.log"})
    assert "left_arm_emulator" not in config["/**/controller_manager"]["ros__parameters"] and "/**/left_arm_emulator" not in config
    assert "/**/right_arm_emulator" in config
    assert support.emulators_to_spawn({"vcan_arm_l": "x"}) == ["right_arm_emulator"]
    assert support.emulators_to_spawn() == ["left_arm_emulator", "right_arm_emulator"]


def test_replay_arguments():
    assert support.parse_replays({"vcan_bms": "/a.log", "vcan_drive": ""}) == {"vcan_bms": "/a.log"}
    with pytest.raises(ValueError):
        support.parse_replays({"can0": "/a.log"})
    assert set(support.BASE_EMULATOR_OF_BUS) == set(support.BASE_BUSES)


def test_driver_controllers_have_no_lift_controller():
    source = {"controller_manager": {"ros__parameters": {"update_rate": 100, "lift_controller": {}, "left_arm_controller": {}}},
              "lift_controller": {"ros__parameters": {}}, "left_arm_controller": {"ros__parameters": {}}}
    config = support.driver_controllers(source, 750, True)
    manager = config["controller_manager"]["ros__parameters"]
    assert manager["update_rate"] == 750 and manager["use_sim_time"] is True
    assert "lift_controller" not in manager and "lift_controller" not in config and "left_arm_controller" in config


def test_moveit_has_no_lift_controller_with_devices():
    parameters = {"moveit_simple_controller_manager": {"controller_names": ["left_arm_controller", "lift_controller"],
                                                       "lift_controller": {}, "left_arm_controller": {}}}
    out = support.moveit_controllers_without_lift(parameters)["moveit_simple_controller_manager"]
    assert out["controller_names"] == ["left_arm_controller"] and "lift_controller" not in out
    assert "lift_controller" in parameters["moveit_simple_controller_manager"]


def test_device_bridge_has_no_cmd_vel_and_no_gazebo_odometry():
    entries = support.control_bridge(["item"], devices=True, follower=True)
    to_gz = {entry["gz_topic_name"] for entry in entries if entry["direction"] == "ROS_TO_GZ"}
    assert to_gz == {"/build1/gz/left_wheel_cmd_vel", "/build1/gz/right_wheel_cmd_vel", "/sil/gz/lift_cmd_vel",
                     "/build1/gz/left_finger2_cmd_pos", "/build1/gz/right_finger2_cmd_pos"}
    ros = set(support.bridge_topics(entries, "GZ_TO_ROS"))
    assert "/odom" not in ros and "/tf" not in ros and "/joint_states" not in ros          # SW-08: one owner, not Gazebo
    assert {"/clock", "/sil/gz/joint_states", "/build1/ground_truth/odom", "/build1/ground_truth/item/pose"} <= ros


def test_replayed_base_buses_lose_their_plant_input():
    entries = support.control_bridge([], devices=True, replay_drive=True, replay_aux=True)
    assert [entry for entry in entries if entry["direction"] == "ROS_TO_GZ"] == []


def test_direct_bridge_is_the_platform_base_interface():
    entries = support.control_bridge([], devices=False)
    assert {entry["gz_topic_name"] for entry in entries if entry["direction"] == "ROS_TO_GZ"} == {"/cmd_vel"}
    assert {"/odom", "/tf", "/base/joint_states"} <= set(support.bridge_topics(entries, "GZ_TO_ROS"))


def test_every_sensor_goes_through_the_one_sensor_bridge():
    topics = set(support.bridge_topics(support.sensor_bridge(True), "GZ_TO_ROS"))
    assert {"/scan", "/imu", "/rgb_image", "/tof/front", "/ultrasonic/rear_right", "/base_camera/rgbd/depth_image",
            "/head_camera/rgbd/image", "/left_wrist_camera/image", "/right_wrist_camera/camera_info"} <= topics
    assert not any("wrist" in topic or "rgbd" in topic for topic in support.bridge_topics(support.sensor_bridge(False), "GZ_TO_ROS"))
    control = set(support.bridge_topics(support.control_bridge([], devices=True), "GZ_TO_ROS"))
    assert not (topics & control), "a sensor on the control bridge would survive the stale sensor fault"


def test_model_sensors_have_optical_frames(devices_urdf):
    frames = dict(re.findall(r'<sensor name="(\w+)".*?<gz_frame_id>(\w+)</gz_frame_id>', devices_urdf, re.S))
    assert frames["base_camera_rgbd_sensor"] == "base_camera_color_optical_frame"
    assert frames["head_camera_rgbd_sensor"] == "head_camera_optical_frame"
    assert frames["left_wrist_camera_sensor"] == "left_wrist_camera_optical_frame"
    assert frames["right_wrist_camera_sensor"] == "right_wrist_camera_optical_frame"
    assert frames["camera_sensor"] == "camera_optical_frame"
    for frame in frames.values():
        assert f'<link name="{frame}"' in devices_urdf, f"{frame} is not a link of the model"
    from conftest import expand
    assert "rgbd_camera" not in expand(extra_sensors="false")


def test_castors_are_fixed_joints_in_the_description(devices_urdf):
    """Reported in the README: the description has no swivel joint for the castors."""
    for corner in ("fl", "fr", "bl", "br"):
        for joint in (f"{corner}_caster_joint", f"{corner}_wheel_joint"):
            assert re.search(rf'<joint name="{joint}" type="fixed">', devices_urdf), joint


def test_nav2_parameters_keep_the_platform_values_and_get_the_spawn_pose():
    platform = {"amcl": {"ros__parameters": {"alpha1": 0.2}}, "controller_server": {"ros__parameters": {"x": 1}}}
    params = support.nav2_params(platform, {}, (1.0, -2.0, 0.5))
    assert params["amcl"]["ros__parameters"]["initial_pose"] == {"x": 1.0, "y": -2.0, "z": 0.0, "yaw": 0.5}
    assert params["amcl"]["ros__parameters"]["alpha1"] == 0.2 and params["controller_server"] == platform["controller_server"]
    assert "initial_pose" not in platform["amcl"]["ros__parameters"]
    assert support.deep_merge({"a": {"b": 1, "c": 2}}, {"a": {"b": 3}}) == {"a": {"b": 3, "c": 2}}
    assert (load_yaml("config", "nav2_overrides.yaml") or {}) == {}, "an override must be listed in the README first"
