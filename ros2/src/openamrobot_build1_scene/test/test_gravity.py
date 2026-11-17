# SPDX-License-Identifier: MIT
"""The arms' gravity feed-forward and the start-state resolution of the devices profile (docs/build1_arm_diagnosis.md).

The reference torques were computed with pinocchio 4.1.0 (computeGeneralizedGravity, g = 9.80665 m/s^2) on the
combined model expanded at hardware_profile:=gazebo, base_plate:=bp000, openarm_description 14ff67b, on 17 Nov 2026.
pinocchio is not a dependency of this package: the numbers are copied here, and gravity.py must reproduce them.
"""
import math

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


def _quaternion_matrix(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


@pytest.mark.parametrize("side", ["left", "right"])
def test_the_grasp_puts_the_models_tcp_on_the_item_not_an_estimated_depth(model, side):
    """D9 step 2. The scenario estimated 0.105 m from the hand base to the grasp centre; the model's TCP is 0.138 m."""
    import numpy as np
    from openamrobot_build1_scene.arm_frames import HAND_FRAME, SIDE_GRASP, TCP_FRAME, hand_for_tcp_at, side_grasp_offset
    links, _ = model._frames({})
    (R_hand, p_hand), (_, p_tcp) = links[HAND_FRAME[side]], links[TCP_FRAME[side]]
    offset = R_hand.T @ (p_tcp - p_hand)                       # what /tf gives the scenario
    assert side_grasp_offset(offset)[0] == pytest.approx(0.138, abs=1e-6)
    item = (0.75, 0.16 if side == "left" else -0.16, 0.925)
    hand = hand_for_tcp_at(item, offset)
    tcp_world = np.array(hand) + np.array(_quaternion_matrix(*SIDE_GRASP)) @ offset
    assert tcp_world == pytest.approx(item, abs=1e-9)
    assert item[0] - hand[0] == pytest.approx(0.138, abs=1e-6) and item[0] - hand[0] > 0.105 + 0.03


# ---------------------------------------------------------------- staleness and payload, on a model small enough to check by hand
# A shoulder about y at the root, a massless upper arm of 0.5 m along x, a fixed tool frame at its end turned 90 degrees
# about z. One arm of one joint: no xacro, no ROS.
SHOULDER_URDF = """<robot name="shoulder">
  <link name="base"/>
  <link name="arm"><inertial><origin xyz="0.25 0 0"/><mass value="1.0"/></inertial></link>
  <link name="tool"/>
  <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/></joint>
  <joint name="tool_mount" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0.5 0 0" rpy="0 0 1.5707963267948966"/></joint>
</robot>"""
G = 9.80665
ONE_ARM = {"left": ["shoulder"]}


@pytest.fixture()
def shoulder():
    return GravityModel(SHOULDER_URDF)


def test_a_fresh_state_gives_the_model_torque(shoulder):
    from openamrobot_build1_scene.gravity import FeedForward
    logic = FeedForward(shoulder, max_age_s=0.2, arms=ONE_ARM)
    logic.on_state(["shoulder"], [0.0], now_s=10.0)
    commands, stale = logic.command(now_s=10.15)
    assert stale == []
    assert commands["left"] == pytest.approx([-1.0 * G * 0.25], abs=1e-9)
    assert commands["left"] == pytest.approx(shoulder.torques({"shoulder": 0.0}, ["shoulder"]), abs=1e-12)


@pytest.mark.parametrize("case", ["never_received", "older_than_max_age", "incomplete"])
def test_a_stale_or_missing_state_gives_zeros_every_cycle(shoulder, case):
    """D4 refinement: JointGroupEffortController keeps its last command for ever, so the node must send tau 0."""
    from openamrobot_build1_scene.gravity import FeedForward
    arms = {"left": ["shoulder"], "right": ["shoulder_r"]} if case == "incomplete" else ONE_ARM
    logic = FeedForward(shoulder, max_age_s=0.2, arms=arms)
    if case != "never_received":
        logic.on_state(["shoulder"], [0.0], now_s=10.0)
    now = 10.25 if case == "older_than_max_age" else 10.05
    if case == "incomplete":
        commands, stale = logic.command(now_s=now)
        assert stale == ["right"] and commands["right"] == [0.0]
        assert commands["left"] == pytest.approx([-G * 0.25], abs=1e-9), "the other arm keeps its model torque"
        return
    for cycle in range(3):
        commands, stale = logic.command(now_s=now + 0.01 * cycle)
        assert stale == ["left"] and commands == {"left": [0.0]}


def test_the_payload_adds_m_g_r_at_the_shoulder_with_the_com_in_the_tool_frame(shoulder):
    """0.15 kg, CoM 0.1 m along x of the tool frame, which is y of the root: lever arm 0.5 m, not 0.6 m."""
    from openamrobot_build1_scene.gravity import FeedForward
    logic = FeedForward(shoulder, max_age_s=0.2, arms=ONE_ARM)
    logic.on_state(["shoulder"], [0.0], now_s=1.0)
    bare = logic.command(now_s=1.0)[0]["left"][0]
    logic.payload = shoulder.payload(0.15, [0.1, 0.0, 0.0], "tool")
    held = logic.command(now_s=1.0)[0]["left"][0]
    assert held - bare == pytest.approx(-0.15 * G * 0.5, abs=1e-9)
    # pitched down 30 degrees the lever arm shrinks with the cosine
    logic.on_state(["shoulder"], [0.5236], now_s=1.0)
    pitched = logic.command(now_s=1.0)[0]["left"][0] - shoulder.torques({"shoulder": 0.5236}, ["shoulder"])[0]
    assert pitched == pytest.approx(-0.15 * G * 0.5 * math.cos(0.5236), abs=1e-6)


def test_the_payload_is_off_by_default_and_a_bad_value_is_refused(shoulder):
    from openamrobot_build1_scene.gravity import FeedForward
    assert FeedForward(shoulder, 0.2).payload is None
    assert shoulder.payload(0.0, [0.0, 0.0, 0.0], "") is None
    for mass, com, link in ((0.15, [0.0, 0.0, 0.0], ""), (0.15, [0.0, 0.0, 0.0], "nowhere"), (-0.1, [0.0, 0.0, 0.0], "tool"),
                            (0.15, [0.0, 0.0], "tool"), (float("nan"), [0.0, 0.0, 0.0], "tool")):
        with pytest.raises(ValueError):
            shoulder.payload(mass, com, link)


def test_the_node_publishes_the_feed_forward_command_for_every_arm_every_cycle():
    """The node is a thin ROS shell around FeedForward: no early return in the timer, one publish per arm per cycle."""
    import ast
    from conftest import package_path
    with open(package_path("scripts", "gravity_feedforward.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    timer = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef) and n.name == "on_timer")
    assert not [n for n in ast.walk(timer) if isinstance(n, ast.Return)], "a timer that returns early publishes nothing"
    text = ast.unparse(timer)
    assert "self.logic.command(" in text and "for side, torques in commands.items()" in text
    assert {"payload_mass_kg", "payload_com_xyz", "payload_link", "max_state_age_s"} <= {
        n.args[0].value for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "attr", "") == "declare_parameter"}


def test_scenario_4_adds_the_held_item_only_after_the_grasp_is_reported_and_clears_it_after_the_release():
    from conftest import package_path
    with open(package_path("openamrobot_build1_scene", "scenarios.py"), encoding="utf-8") as f:
        text = f.read()
    body = text[text.index("def grasp(rig, log):"):text.index("def _held_payload(")]
    closed, held = body.index("closed = rig.gripper(side, CLOSED"), body.index("_held_payload(rig, log, side) if closed is not None")
    lift, release = body.index('"grasp_lift"'), body.index("released = True")
    assert closed < held < lift < release
    # Cleared after the release, in the finally block of the grasp: also on a failed reach, close or lift.
    clear = body.index('{"payload_mass_kg": 0.0}')
    assert clear > release and body.rindex("finally:", 0, clear) > release and "if held:" in body[body.rindex("finally:", 0, clear):clear]
