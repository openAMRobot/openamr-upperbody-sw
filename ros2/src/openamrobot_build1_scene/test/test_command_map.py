# SPDX-License-Identifier: MIT
"""config/command_map.yaml against the model, and the single-writer rules against graphs that break them.

Each conflict of the phase 1 single-writer map that applies to this scene has a test here that fails when the
conflict comes back: SW-01, SW-03, SW-06, SW-08, SW-09, SW-10.
"""
import copy

import pytest

from openamrobot_build1_scene import launch_support as support
from openamrobot_build1_scene import single_writer as sw

ALL = ("nav2", "moveit", "follower")


def graph(command_map, kind, components=ALL):
    """The publisher graph the map describes."""
    return {topic: sw.expected_writers(entry, kind, components) for topic, entry in command_map["topics"].items()}


def test_every_movable_joint_has_one_owner_and_one_path_per_profile(command_map, devices_urdf, direct_urdf):
    assert sw.joint_map_problems(command_map, support.sim_description(devices_urdf)) == []
    assert sw.joint_map_problems(command_map, direct_urdf) == []
    assert command_map["dynamics_owner"] == "gazebo"
    assert set(command_map["profiles"]) == set(support.RUN_PROFILES)
    for profile, entry in support.RUN_PROFILES.items():
        assert command_map["profiles"][profile] == ("devices" if entry["devices"] else "direct")


def test_a_joint_missing_from_the_map_is_found(command_map, direct_urdf):
    broken = copy.deepcopy(command_map)
    del broken["joints"]["lift_joint"]
    assert any("lift_joint" in problem for problem in sw.joint_map_problems(broken, direct_urdf))
    broken = copy.deepcopy(command_map)
    broken["joints"]["left_gripper"]["joints"].append("openarm_left_joint1")
    assert sw.joint_map_problems(broken, direct_urdf) == ["openarm_left_joint1 is in the map twice"]
    broken = copy.deepcopy(command_map)
    broken["joints"]["lift_joint"]["devices"] = "lift_controller > somewhere"
    assert any("does not end in Gazebo" in problem for problem in sw.joint_map_problems(broken, direct_urdf))


def test_the_map_is_consistent_with_itself(command_map):
    assert sw.map_problems(command_map) == []
    broken = copy.deepcopy(command_map)
    broken["topics"]["/cmd_vel"]["writers_when"]["nav2"].append("/dock_trigger")
    assert any("/cmd_vel" in problem for problem in sw.map_problems(broken))


@pytest.mark.parametrize("kind", sw.KINDS)
def test_the_described_graph_passes(command_map, kind):
    assert sw.topic_problems(command_map, graph(command_map, kind), kind, ALL) == []


def test_sw01_a_second_cmd_vel_writer_is_found(command_map):
    for kind in sw.KINDS:
        publishers = graph(command_map, kind)
        assert publishers["/cmd_vel"] == ["/collision_monitor"]
        publishers["/cmd_vel"] = ["/collision_monitor", "/dock_trigger"]
        problems = sw.topic_problems(command_map, publishers, kind, ALL)
        assert len(problems) == 1 and problems[0].startswith("/cmd_vel:") and "/dock_trigger" in problems[0]


def test_sw01_without_nav2_nothing_in_the_launch_writes_cmd_vel(command_map):
    assert sw.expected_writers(command_map["topics"]["/cmd_vel"], "devices", ()) == []


def test_sw03_the_drive_emulator_is_the_only_shaper_and_the_adapter_the_only_wheel_writer(command_map):
    publishers = graph(command_map, "devices")
    assert publishers["/sil/drive/gz_cmd_vel"] == ["/sil_zlac8015d"]
    assert publishers["/build1/drive/left_wheel_cmd_vel"] == publishers["/build1/drive/right_wheel_cmd_vel"] == ["/wheel_command_adapter"]
    single, silent = sw.gz_topics(command_map, "devices", ())
    assert "/cmd_vel" in silent and "/cmd_vel" not in single, "with devices nothing may publish the DiffDrive input"
    counts = {topic: 1 for topic in single}
    assert sw.gz_problems(command_map, "devices", counts) == []
    counts["/cmd_vel"] = 1
    assert sw.gz_problems(command_map, "devices", counts) == ["gz /cmd_vel: 1 publishers, none allowed in this profile"]
    counts = {topic: 1 for topic in single}
    counts["/build1/gz/left_wheel_cmd_vel"] = 2
    assert len(sw.gz_problems(command_map, "devices", counts)) == 1
    assert "wheel_command_adapter" in command_map["joints"]["left_wheel_joint"]["devices"]
    assert "DiffDrive" not in command_map["joints"]["left_wheel_joint"]["devices"]


def claimed(command_map, kind):
    interfaces = {}
    for manager in command_map["controller_managers"][kind]:
        interfaces[manager["name"]] = {name: True for joint in sw.joints_of_groups(command_map, manager["joints"])
                                       for name in sw.expected_claims(command_map, manager, joint)}
    return interfaces


def test_sw06_the_lift_has_one_owner_per_profile(command_map):
    assert "sil_ctl017" in command_map["joints"]["lift_joint"]["devices"] and "lift_controller" not in command_map["joints"]["lift_joint"]["devices"]
    assert "lift_controller" in command_map["joints"]["lift_joint"]["direct"]
    for kind in sw.KINDS:
        assert sw.interface_problems(command_map, kind, claimed(command_map, kind)) == []
    interfaces = claimed(command_map, "devices")
    interfaces["/sim/controller_manager"]["lift_joint/position"] = False
    problems = sw.interface_problems(command_map, "devices", interfaces)
    assert problems == ["/sim/controller_manager: lift_joint must have no command interface, found ['lift_joint/position']"]
    servers = {action: [entry["server"]] for action, entry in command_map["actions"].items()}
    assert sw.action_problems(command_map, servers, "direct") == []
    assert sw.action_problems(command_map, servers, "devices") == [
        "action /lift_controller/follow_joint_trajectory: servers ['/lift_controller'], the map says []"]


def test_two_claims_on_one_joint_are_found(command_map):
    interfaces = claimed(command_map, "devices")
    interfaces["/sim/controller_manager"]["openarm_left_joint1/position"] = True
    assert len(sw.interface_problems(command_map, "devices", interfaces)) == 1
    interfaces = claimed(command_map, "direct")
    del interfaces["/controller_manager"]["lift_joint/effort"]
    assert len(sw.interface_problems(command_map, "direct", interfaces)) == 1
    assert sw.interface_problems(command_map, "direct", {}) == ["/controller_manager did not answer list_hardware_interfaces"]


def test_sw08_state_topics_have_an_owner_in_every_profile(command_map):
    for kind, owner in (("devices", "/base_state_adapter"), ("direct", "/build1_control_bridge")):
        publishers = graph(command_map, kind)
        assert publishers["/odom"] == [owner]
        assert publishers["/joint_states"] == ["/joint_state_publisher"]
        assert owner in publishers["/tf"] and "/robot_state_publisher" in publishers["/tf"]
        publishers["/odom"] = []
        assert any(problem.startswith("/odom:") for problem in sw.topic_problems(command_map, publishers, kind, ALL))
    publishers = graph(command_map, "devices")
    publishers["/joint_states"] = ["/joint_state_publisher", "/joint_state_broadcaster"]
    assert len(sw.topic_problems(command_map, publishers, "devices", ALL)) == 1


def test_sw09_the_follower_is_a_writer_only_when_the_engine_has_no_mimic(command_map):
    topic = command_map["topics"]["/build1/fingers/left_finger2_cmd_pos"]
    assert sw.expected_writers(topic, "devices", ("follower",)) == ["/finger_mimic_follower"]
    assert sw.expected_writers(topic, "devices", ()) == []
    assert "/build1/gz/left_finger2_cmd_pos" in sw.gz_topics(command_map, "devices", ("follower",))[0]
    assert "/build1/gz/left_finger2_cmd_pos" not in sw.gz_topics(command_map, "devices", ())[0]
    assert set(command_map["mimic_owner"]) == {"engine", "follower"}


def chain(kind, nav2=True):
    pairs = {("base_footprint", "odom"), ("base_link", "base_footprint"), ("lift_carriage_link", "lift_column_base_link")}
    return pairs | ({("odom", "map")} if nav2 else set())


def test_sw10_one_frame_chain_and_one_parent_per_frame(command_map):
    assert command_map["tf_chain"] == ["odom", "base_footprint", "base_link"]
    for kind in sw.KINDS:
        assert sw.tf_problems(command_map, chain(kind), kind, ("nav2",)) == []
        assert sw.tf_problems(command_map, chain(kind, nav2=False), kind, ()) == []
    # The frame chain of the real path of the platform: odom -> base_link and base_link -> base_footprint.
    real_path = {("base_link", "odom"), ("base_footprint", "base_link")}
    assert len(sw.tf_problems(command_map, real_path, "devices", ())) >= 2
    both = chain("devices") | {("base_link", "odom")}
    assert any("base_link has 2 parents" in problem for problem in sw.tf_problems(command_map, both, "devices", ("nav2",)))
    assert any("odom -> base_footprint" in problem for problem in sw.tf_problems(command_map, {("base_link", "base_footprint")}, "devices", ()))


def test_a_command_topic_outside_the_map_is_found(command_map):
    publishers = graph(command_map, "devices")
    publishers["/sil/lift/second_command"] = ["/someone"]
    publishers["/sil/drive/status"] = ["/sil_stm32_host_client"]
    publishers["/cmd_vel_teleop"] = []
    assert sw.unlisted_command_topics(command_map, publishers) == ["/sil/lift/second_command"]


def test_test_tools_are_not_counted_as_writers(command_map):
    publishers = graph(command_map, "devices")
    publishers["/sil/lift/command"] = ["/build1_scn02_lift_permit"]
    assert sw.topic_problems(command_map, publishers, "devices", ALL) == []
    publishers["/sil/lift/command"] = ["/another_node"]
    assert len(sw.topic_problems(command_map, publishers, "devices", ALL)) == 1


def test_one_node_with_several_endpoints_is_one_writer(command_map):
    publishers = graph(command_map, "direct")
    publishers["/cmd_vel_nav"] = ["/behavior_server"] * 5 + ["/controller_server"]
    assert sw.topic_problems(command_map, publishers, "direct", ALL) == []


def test_unknown_profile(command_map):
    assert sw.kind_of(command_map, "faults") == "devices" and sw.kind_of(command_map, "fine") == "direct"
    with pytest.raises(ValueError):
        sw.kind_of(command_map, "geometry")


def test_sw06_without_devices_lift_controller_claims_effort_on_the_lift_and_position_elsewhere(command_map):
    """docs/build1_lift_and_base_diagnosis.md: through the position interface DART cannot lift the joint off L1000."""
    assert "effort interface" in command_map["joints"]["lift_joint"]["direct"]
    interfaces = claimed(command_map, "direct")["/controller_manager"]
    assert interfaces["lift_joint/effort"] and "lift_joint/position" not in interfaces
    assert interfaces["openarm_left_joint1/position"] and interfaces["openarm_right_finger_joint1/position"]
    wrong = {"/controller_manager": dict(interfaces, **{"lift_joint/position": True})}
    del wrong["/controller_manager"]["lift_joint/effort"]
    assert sw.interface_problems(command_map, "direct", wrong) == [
        "/controller_manager: lift_joint has claimed command interfaces ['lift_joint/position'], the map says ['lift_joint/effort']"]


def test_the_arm_effort_interface_of_the_real_driver_is_claimed_by_the_gravity_controller_only(command_map):
    """With devices each arm joint of /controller_manager has position (trajectory controller) and effort (gravity
    feed-forward) claimed; the grippers have position only. A missing effort claim, or one on a gripper, is found."""
    manager = next(m for m in command_map["controller_managers"]["devices"] if m["name"] == "/controller_manager")
    assert sw.expected_claims(command_map, manager, "openarm_left_joint1") == ["openarm_left_joint1/effort", "openarm_left_joint1/position"]
    assert sw.expected_claims(command_map, manager, "openarm_left_finger_joint1") == ["openarm_left_finger_joint1/position"]
    interfaces = claimed(command_map, "devices")
    assert sw.interface_problems(command_map, "devices", interfaces) == []
    del interfaces["/controller_manager"]["openarm_right_joint4/effort"]
    interfaces["/controller_manager"]["openarm_right_finger_joint1/effort"] = True
    assert len(sw.interface_problems(command_map, "devices", interfaces)) == 2
    for side in ("left", "right"):
        assert command_map["topics"][f"/{side}_arm_gravity_controller/commands"] == {"writers_devices": ["/gravity_feedforward"], "command": True}
