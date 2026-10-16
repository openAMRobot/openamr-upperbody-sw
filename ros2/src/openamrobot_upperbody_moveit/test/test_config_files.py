"""The MoveIt configuration files: names agree with the SRDF and with the controllers the stack really spawns, the limits carry their
provenance tags (ADR-009), and the gripper controller type is still option 1 (the fork is not decided here)."""
import os

import yaml
from ament_index_python.packages import get_package_share_directory

from conftest import load_yaml, srdf_root, urdf_root
from openamrobot_upperbody_moveit.named_poses import GRIPPER_JOINT

CONTROL_YAML = os.path.join(get_package_share_directory("openamrobot_upperbody_control"), "config", "arm_controllers.yaml")
ARM_JOINTS = [f"openarm_{side}_joint{i}" for side in ("left", "right") for i in range(1, 8)]
CONTROLLED = ARM_JOINTS + [GRIPPER_JOINT["left"], GRIPPER_JOINT["right"]]
ACCEL_SECONDS = 2.0                       # the rule written at the top of joint_limits.yaml


def controllers():
    with open(CONTROL_YAML) as f:
        return yaml.safe_load(f)


def test_kinematics_is_kdl_for_both_arms_and_names_srdf_groups():
    kin = load_yaml("kinematics.yaml")
    groups = {g.get("name") for g in srdf_root().findall("group")}
    assert set(kin) == {"left_arm", "right_arm"} and set(kin) <= groups
    assert all(v["kinematics_solver"] == "kdl_kinematics_plugin/KDLKinematicsPlugin" for v in kin.values())


def test_joint_limits_entries_are_tagged_engineering_default_placeholder():
    """ADR-009 point 2: acceleration entries are tagged `source: engineering_default`, `status: placeholder`."""
    limits = load_yaml("joint_limits.yaml")["joint_limits"]
    assert sorted(limits) == sorted(CONTROLLED)
    for joint, entry in limits.items():
        assert entry["source"] == "engineering_default" and entry["status"] == "placeholder", joint
        assert entry["has_acceleration_limits"] is True and entry["max_acceleration"] > 0, joint


def test_joint_limits_follow_their_written_rule_and_repeat_no_urdf_data():
    """The rule is acceleration = velocity / 2 s with the URDF's own velocity limit: a rule, not a measurement. No velocity or position
    limit is repeated (they come from the URDF, which is upstream's), so there is nothing here to drift from it."""
    root = urdf_root()
    limits = load_yaml("joint_limits.yaml")["joint_limits"]
    for joint, entry in limits.items():
        velocity = float(root.find(f"joint[@name='{joint}']/limit").get("velocity"))
        assert abs(entry["max_acceleration"] - round(velocity / ACCEL_SECONDS, 2)) < 1e-9, joint
        assert set(entry) == {"has_acceleration_limits", "max_acceleration", "source", "status"}, joint


def test_joint_limits_header_names_the_issue_the_upstream_file_and_the_adr():
    with open(os.path.join(get_package_share_directory("openamrobot_upperbody_moveit"), "config", "joint_limits.yaml")) as f:
        header = f.read().split("joint_limits:")[0]
    for needle in ("manipulation issue #10", "openarm_bimanual_moveit_config/config/openarm_v2.0/joint_limits.yaml", "4e837e1",
                   "ADR-009", "NOT vendor data", "engineering_default"):
        assert needle.replace("manipulation issue", "openamrobot-manipulation issue") in header or needle in header, needle


def test_the_moveit_controllers_are_the_ones_the_stack_spawns_with_the_joints_they_command():
    ctl = controllers()["controller_manager"]["ros__parameters"]
    ours = load_yaml("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    assert sorted(ours["controller_names"]) == sorted(["left_arm_controller", "right_arm_controller",
                                                       "left_gripper_controller", "right_gripper_controller"])
    for name in ours["controller_names"]:
        assert name in ctl, f"{name} is not a controller of arm_controllers.yaml"
    for side in ("left", "right"):
        assert ours[f"{side}_arm_controller"]["joints"] == controllers()[f"{side}_arm_controller"]["ros__parameters"]["joints"]
        assert ours[f"{side}_gripper_controller"]["joints"] == [GRIPPER_JOINT[side]]
        assert controllers()[f"{side}_gripper_controller"]["ros__parameters"]["joint"] == GRIPPER_JOINT[side]


def test_the_gripper_fork_is_option_1_and_the_controller_type_is_unchanged():
    """Option 1 of ADR-009 point 4: GripperCommand served by position_controllers/GripperActionController. A change here is a decision
    for Om and Adnan (manipulation#11), not ours."""
    ctl = controllers()["controller_manager"]["ros__parameters"]
    ours = load_yaml("moveit_controllers.yaml")["moveit_simple_controller_manager"]
    for side in ("left", "right"):
        assert ctl[f"{side}_gripper_controller"]["type"] == "position_controllers/GripperActionController"
        assert ours[f"{side}_gripper_controller"]["type"] == "GripperCommand"
        assert ours[f"{side}_gripper_controller"]["action_ns"] == "gripper_cmd"
        assert ours[f"{side}_arm_controller"]["type"] == "FollowJointTrajectory"
        assert ours[f"{side}_arm_controller"]["action_ns"] == "follow_joint_trajectory"
    assert load_yaml("moveit_controllers.yaml")["moveit_controller_manager"] == "moveit_simple_controller_manager/MoveItSimpleControllerManager"


def test_ompl_planning_covers_the_four_groups_and_the_defaults_are_moveits():
    ompl = load_yaml("ompl_planning.yaml")
    groups = {g.get("name") for g in srdf_root().findall("group")}
    assert groups <= set(ompl)
    defaults = os.path.join(get_package_share_directory("moveit_configs_utils"), "default_configs", "ompl_planning.yaml")
    with open(defaults) as f:
        theirs = yaml.safe_load(f)
    for key in ("planning_plugins", "request_adapters", "response_adapters"):
        assert ompl[key] == theirs[key], f"{key} is not MoveIt's default for the installed version"
    assert "planner_configs" not in ompl, "planner configurations come from MoveIt's ompl_defaults.yaml, untouched"


def test_pilz_limits_are_upstreams_numbers():
    limits = load_yaml("pilz_cartesian_limits.yaml")["cartesian_limits"]
    assert limits == {"max_trans_vel": 1.0, "max_trans_acc": 2.25, "max_trans_dec": -5.0, "max_rot_vel": 1.57}
