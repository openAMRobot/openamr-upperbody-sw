# SPDX-License-Identifier: MIT
"""The engine-override file: schema, sources, and every transcribed number against the file it cites."""
import json
import pathlib
import re

import jsonschema
import yaml

from openamr_sim.mjcf import OVERRIDES

SCHEMA = OVERRIDES.parent / "engine_overrides.schema.json"
SIM = pathlib.Path(__file__).resolve().parents[1]


def walk(node, path=()):
    if isinstance(node, dict):
        yield path, node
        for k, v in node.items():
            yield from walk(v, path + (k,))
    elif isinstance(node, list):
        for i, v in enumerate(node):
            yield from walk(v, path + (i,))


def test_schema(overrides):
    schema = json.loads(SCHEMA.read_text())
    jsonschema.Draft7Validator.check_schema(schema)
    errors = [f"{list(e.path)}: {e.message}" for e in jsonschema.Draft7Validator(schema).iter_errors(overrides)]
    assert not errors, errors


def test_every_source_is_declared(overrides):
    for path, node in walk(overrides):
        if "source" in node and isinstance(node["source"], str):
            assert node["source"] in overrides["sources"], f"{path}: undeclared source {node['source']}"


def test_no_value_without_status(overrides):
    """Below the top-level sections, a mapping that holds a number as `value` names its status (inherited from its parent block if any)."""
    for path, node in walk(overrides):
        if "value" in node and isinstance(node["value"], (int, float, list)) and path[0] not in ("sources",):
            assert "status" in node or path[-1] in ("sag", "zeta", "tau"), f"{path}: value without status"


def test_arm_gains_equal_upstream_control_gains(overrides, openarm_description):
    src = yaml.safe_load((openarm_description / "assets/robot/openarm_v2.0/config/arm/control/control_gains.yaml").read_text())
    ours = overrides["actuators"]["arm_position_gains"]["per_joint"]
    assert ours == {k: {"kp": v["kp"], "kd": v["kd"]} for k, v in src.items()}


def test_gripper_gains_equal_upstream_driver(overrides, openarm_ros2):
    header = (openarm_ros2 / "openarm_hardware/include/openarm_hardware/openarm_simple_hardware.hpp").read_text().splitlines()
    assert re.search(r"GRIPPER_KP\s*=\s*([0-9.]+)", header[110]).group(1) == str(overrides["actuators"]["gripper_position_gains"]["kp"])
    assert re.search(r"GRIPPER_KD\s*=\s*([0-9.]+)", header[111]).group(1) == str(overrides["actuators"]["gripper_position_gains"]["kd"])
    source = (openarm_ros2 / "openarm_hardware/src/openarm_simple_hardware.cpp").read_text().splitlines()
    assert "joint_to_motor_radians(double joint_value)" in source[374]
    assert source[377].strip() == "return joint_value;"  # identity for pinch_gripper


def test_wheel_friction_equals_platform_sw_135(overrides, platform_sw_main):
    lines = (platform_sw_main / "ros2/src/openamrobot_description/urdf/gazebo_control.xacro").read_text().splitlines()
    c = overrides["contact"]
    assert lines[17].strip() == f"<mu1>{c['drive_wheel_sliding_friction']['value']}</mu1>"
    assert lines[18].strip() == f"<mu2>{c['drive_wheel_lateral_friction']['value']}</mu2>"
    assert lines[19].strip() == "<fdir1>1 0 0</fdir1>"
    assert lines[25].strip() == "<mu1>0.0</mu1>"  # castor slider value of #135, not applied to rolling castors (see the file)


def test_mimic_solver_values_equal_openarm_mujoco(overrides, openarm_mujoco):
    import xml.etree.ElementTree as ET
    root = ET.parse(openarm_mujoco / "v2/openarm_bimanual.xml").getroot()
    eqs = root.findall("equality/joint")
    assert len(eqs) == 2
    for e in eqs:
        assert [float(x) for x in e.get("solref").split()] == overrides["mimic"]["mujoco"]["solref"]["value"]
        assert [float(x) for x in e.get("solimp").split()] == overrides["mimic"]["mujoco"]["solimp"]["value"]
    opt = root.find("option")
    assert opt.get("integrator") == overrides["contact"]["integrator"]["value"]
    assert opt.get("cone") == overrides["contact"]["cone"]["value"]
    assert float(opt.get("impratio")) == overrides["contact"]["impratio"]["value"]


def test_physics_step_and_rate_equal_the_fine_profile(overrides):
    repo = SIM.parent
    world = (repo / "ros2/src/openamrobot_upperbody_gazebo/worlds/empty.sdf").read_text()
    step = float(re.search(r"<max_step_size>([0-9.eE+-]+)</max_step_size>", world).group(1))
    assert abs(step - overrides["common"]["physics_step"]["value"]) < 1e-8
    overlay = yaml.safe_load((repo / "ros2/src/openamrobot_upperbody_control/config/sim_overlay.yaml").read_text())
    assert overlay["controller_manager"]["ros__parameters"]["update_rate"] == overrides["common"]["controller_rate"]["value"]


def test_isaac_pins_agree(overrides):
    pins = yaml.safe_load((SIM / "sim_pins.yaml").read_text())["isaac_sim"]
    assert pins["version"] == overrides["isaac"]["version"]["value"]
    assert pins["container_image"] == overrides["isaac"]["container_image"]
    assert overrides["isaac"]["container_image"].endswith(":" + pins["version"])


def test_profile_and_overrides_cover_the_same_arm_joints(overrides):
    from openamr_sim.mjcf import PROFILE, load_yaml
    prof = load_yaml(PROFILE)
    assert set(overrides["mimic"]["joints"]) == {j for j, v in prof["joints"].items() if not v["controlled"]}
