# SPDX-License-Identifier: MIT
"""The arms' gravity feed-forward and the start-state resolution of the devices profile (docs/build1_arm_diagnosis.md).

The reference torques were computed with pinocchio 4.1.0 (computeGeneralizedGravity, g = 9.80665 m/s^2) on the
combined model expanded at hardware_profile:=gazebo, base_plate:=bp000, openarm_description 14ff67b, on 17 Nov 2026.
pinocchio is not a dependency of this package: the numbers are copied here, and gravity.py must reproduce them.
"""
import pytest

from openamrobot_build1_scene import launch_support as support
from openamrobot_build1_scene.gravity import ARM_JOINTS, GravityModel

JOINTS = ARM_JOINTS["left"] + ARM_JOINTS["right"]
GRASP = dict(zip(ARM_JOINTS["left"], [-1.057, -0.0153, -0.004, 0.6676, -0.0041, -0.1539, -0.0147]))
REFERENCE = {
    "zero": ({}, [0.074164, 0.055382, 0.0, 0.014709, 0.0, 0.03848, -0.04075,
                  -0.074164, -0.027537, 0.0, 0.014709, 0.0, -0.03848, 0.04075]),
    "park": ({name: 0.1 for name in JOINTS if name.endswith("joint4")},
             [-0.365348, 0.055382, 0.0, 0.454221, -0.004139, 0.091191, -0.040547,
              0.365348, -0.027537, 0.0, 0.454221, 0.004139, -0.091191, 0.040547]),
    # the left arm at the hand pose of scenario 4's grasp_reach (pinocchio IK on the same model)
    "grasp": (GRASP, [-10.705193, -0.055523, -0.065845, 4.360353, -0.048667, 0.529263, -1e-05,
                      -0.074164, -0.027537, 0.0, 0.014709, 0.0, -0.03848, 0.04075]),
}


@pytest.fixture(scope="module")
def model(devices_urdf):
    return GravityModel(devices_urdf)


@pytest.mark.parametrize("case", sorted(REFERENCE))
def test_the_holding_torque_is_the_models_gravity_torque(model, case):
    positions, expected = REFERENCE[case]
    assert model.torques(positions, JOINTS) == pytest.approx(expected, abs=2e-6)


def test_the_holding_torque_of_the_grasp_pose_is_what_leaves_the_arm_low_without_it(model):
    """Joint 1 needs 10.7 N m at the grasp pose. With tau 0 the driver's kp of 70 N m/rad leaves it 0.15 rad short."""
    tau = model.torques(GRASP, ARM_JOINTS["left"])
    assert tau[0] == pytest.approx(-10.705, abs=1e-3)
    assert abs(tau[0]) / 70.0 > 0.1


def test_the_driver_gets_one_effort_controller_per_arm_on_the_arm_joints_only():
    source = {"controller_manager": {"ros__parameters": {"update_rate": 100, "left_arm_controller": {}, "right_arm_controller": {}}},
              "left_arm_controller": {"ros__parameters": {"joints": ARM_JOINTS["left"], "command_interfaces": ["position"]}},
              "right_arm_controller": {"ros__parameters": {"joints": ARM_JOINTS["right"], "command_interfaces": ["position"]}}}
    config = support.driver_controllers(source, 500, True, gravity_feedforward=True)
    manager = config["controller_manager"]["ros__parameters"]
    for side, name in support.GRAVITY_CONTROLLERS.items():
        assert manager[name] == {"type": "effort_controllers/JointGroupEffortController"}
        assert config[name]["ros__parameters"]["joints"] == ARM_JOINTS[side]
        assert config[f"{side}_arm_controller"]["ros__parameters"]["command_interfaces"] == ["position"]
    assert support.driver_controllers_to_spawn(True)[-2:] == list(support.GRAVITY_CONTROLLERS.values())
    plain = support.driver_controllers(source, 500, True)
    assert not set(support.GRAVITY_CONTROLLERS.values()) & set(plain) and support.driver_controllers_to_spawn(False) == list(support.DRIVER_CONTROLLERS)


def test_the_launch_turns_the_feed_forward_on_and_widens_the_clamp_to_the_codec_step_with_devices():
    from conftest import package_path
    with open(package_path("launch", "build1.launch.py"), encoding="utf-8") as f:
        text = f.read()
    assert 'DeclareLaunchArgument("gravity_feedforward", default_value="true"' in text
    assert '{"epsilon": support.DAMIAO_POSITION_RESOLUTION} if devices else {}' in text
    assert 'executable="gravity_feedforward.py"' in text


def _truncating_round_trip(q, p_max=12.5, bits=16):
    """What the emulator's state frame carries: clamp, scale, truncate (codec.cpp float_to_uint), then decode."""
    step = (1 << bits) - 1
    raw = int((min(max(q, -p_max), p_max) + p_max) / (2 * p_max) * step)
    return raw / step * 2 * p_max - p_max


def test_the_resolution_is_one_step_of_the_position_field_and_covers_what_ci_read():
    assert support.DAMIAO_POSITION_RESOLUTION == pytest.approx(3.8148e-4, rel=1e-4)
    assert _truncating_round_trip(0.0) == pytest.approx(-1.9074e-4, abs=1e-8), "joint 4 at 0 reads -1.907e-4 (CI 16 Nov)"
    # openarm_right_joint1 on its lower limit -1.3963 read -1.39639 (CI 16 Nov, scenario 3): one step or less past it
    for limit in (-1.3963, 0.0, -0.7854, -3.4907):
        assert 0.0 <= limit - _truncating_round_trip(limit) <= support.DAMIAO_POSITION_RESOLUTION
    assert -1.3963 - (-1.39639) <= support.DAMIAO_POSITION_RESOLUTION


def test_the_clamp_with_the_codec_step_takes_the_reading_ci_failed_on_and_still_refuses_a_real_overshoot():
    from openamrobot_upperbody_moveit import joint_state_clamp as jsc
    lower, upper = -1.3963, 3.4907
    assert jsc.clamp_position(-1.39639, lower, upper)[2] == "over_epsilon", "the old epsilon refused it: MoveIt error -26"
    assert jsc.clamp_position(-1.39639, lower, upper, support.DAMIAO_POSITION_RESOLUTION) == (lower, pytest.approx(9e-5), "clamped")
    assert jsc.clamp_position(lower - 2 * support.DAMIAO_POSITION_RESOLUTION, lower, upper,
                              support.DAMIAO_POSITION_RESOLUTION)[2] == "over_epsilon"
