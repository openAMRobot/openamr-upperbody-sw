# SPDX-License-Identifier: MIT
"""The one place the combined robot's MoveIt configuration is assembled, with moveit_configs_utils' MoveItConfigsBuilder.

The builder is handed the combined xacro with its args (option B's four configuration IDs and `hardware_profile`), not a pre-expanded URDF, so the robot_description
MoveIt sees is the one the rest of the stack expands from the same file. The SRDF, kinematics, limits and controller mapping are this
package's own files; none is read from upstream's moveit config package (ADR-007).
"""
import os

from ament_index_python.packages import get_package_share_directory

PACKAGE = "openamrobot_upperbody_moveit"
DESCRIPTION = "openamrobot_upperbody_description"
ROBOT_NAME = "openamrobot"          # the <robot name> of combined_robot.xacro and of the SRDF
CONFIG_KEYS = ("column", "base_plate", "head_pitch", "base_tilt")      # option B's configuration IDs, in the order the xacro takes them
PROFILES = ("mock", "gazebo")       # `real` is never launched in A7


def combined_xacro():
    return os.path.join(get_package_share_directory(DESCRIPTION), "urdf", "combined_robot.xacro")


def upperbody_configuration():
    """Option B's config (upperbody_configuration.yaml, merged on main): the one source of the accepted IDs and the baseline."""
    import yaml
    with open(os.path.join(get_package_share_directory(DESCRIPTION), "config", "upperbody_configuration.yaml")) as f:
        return yaml.safe_load(f)


def lift_stops(column=None):
    """{calibration stop: lift_joint position (m)} for `column` (default: the baseline's): L1000 / L1175 / L1350 -> 0.0 / 0.175 / 0.350.
    A stop is a CALIBRATION ID: a position of the joint with a calibration, never an argument of the model or an input of the config id."""
    cfg = upperbody_configuration()
    geo = cfg["geometry"]
    data = cfg["column"][column or cfg["baseline"]["column"]]
    at_zero = geo["base_plate_top_z"] + data["retracted_length"] + geo["arm_mount"]["z_above_column_top"]
    return {stop: round(shoulder - at_zero, 9) for stop, shoulder in cfg["lift_calibration_stops"].items()}


def moveit_config(base_plate=None, hardware_profile="mock", srdf_file=None, *, column=None, head_pitch=None, base_tilt=None):
    """MoveItConfigs for the combined robot at option B's configuration (`base_plate`, `column`, `head_pitch`, `base_tilt`; each
    defaults to the baseline). The lift is not an argument: it is a joint, and MoveIt sees it as one (no planning group). `hardware_profile`
    only selects the ros2_control block of the URDF, which MoveIt does not read; it is passed so that this URDF is the same text the
    controller manager and robot_state_publisher of that profile use.
    `srdf_file` replaces config/openamrobot.srdf (an absolute path); the mutation tests of M03 use it, nothing else does."""
    from moveit_configs_utils import MoveItConfigsBuilder
    cfg = upperbody_configuration()
    given = {"column": column, "base_plate": base_plate, "head_pitch": head_pitch, "base_tilt": base_tilt}
    mappings = {key: given[key] or cfg["baseline"][key] for key in CONFIG_KEYS}
    for key, value in mappings.items():
        if value not in cfg["configuration_axes"][key]:
            raise ValueError(f"{key} {value!r}: accepted values are {', '.join(cfg['configuration_axes'][key])}")
    if hardware_profile not in PROFILES:
        raise ValueError(f"hardware_profile {hardware_profile!r}: accepted values are {', '.join(PROFILES)}")
    return (MoveItConfigsBuilder(ROBOT_NAME, package_name=PACKAGE)
            .robot_description(file_path=combined_xacro(), mappings={**mappings, "hardware_profile": hardware_profile})
            .robot_description_semantic(file_path=srdf_file or "config/openamrobot.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
            # move_group does NOT publish robot_description: robot_state_publisher is the one publisher of that topic. Two publishers race,
            # and ros_gz_sim's `create -topic robot_description` took move_group's copy, which has no controllers_file, so the model's
            # gz_ros2_control plugin found "an empty parameters file" and the simulator's controller manager never started (found in M02).
            .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
            .to_moveit_configs())


def srdf_path():
    return os.path.join(get_package_share_directory(PACKAGE), "config", "openamrobot.srdf")
