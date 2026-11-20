# SPDX-License-Identifier: MIT
"""Arm gravity feed-forward in the MuJoCo control path (openamr_sim/arm_gravity.py) and the wheel contact diagnostic option.

The feed-forward mirrors the Build 1 real-driver path (pull request 73, integration/baseline1): q from the trajectory, tau
from the gravity model, kp and kd the driver's, tau 0 on a stale joint state. Simulation evidence only."""
import math
import xml.etree.ElementTree as ET

import pytest

from openamr_sim import arm_gravity, benchmark as bm

PENDULUM = """
<mujoco>
  <option gravity="0 0 -9.81"/>
  <worldbody>
    <body name="link" pos="0 0 1">
      <joint name="j1" type="hinge" axis="0 1 0"/>
      <inertial pos="0.3 0 0" mass="2.0" diaginertia="0.01 0.01 0.01"/>
    </body>
  </worldbody>
</mujoco>
"""


def pendulum():
    import mujoco as mj
    m = mj.MjModel.from_xml_string(PENDULUM)
    return mj, m, mj.MjData(m)


def test_holding_torque_is_the_gravity_term_and_holds_the_joint():
    mj, m, d = pendulum()
    ff = arm_gravity.GravityFeedForward(m, 0.2, arms={"left": ["j1"]})
    for q in (0.0, 0.7, -1.2):
        d.qpos[0], d.qvel[0] = q, 0.0
        ff.on_state(d, 0.0)
        tau = ff.command(0.0)[0]["left"][0]
        # 2 kg at 0.3 m about a horizontal axis: m g l cos(q); the sign is the one that holds the link up
        assert abs(tau) == pytest.approx(2.0 * 9.81 * 0.3 * math.cos(q), rel=1e-9)
        ff.apply(d, ff.command(0.0)[0])
        mj.mj_forward(m, d)
        assert d.qacc[0] == pytest.approx(0.0, abs=1e-9)


def test_stale_or_missing_state_gives_zeros():
    mj, m, d = pendulum()
    ff = arm_gravity.GravityFeedForward(m, 0.2, arms={"left": ["j1"]})
    assert ff.command(0.0) == ({"left": [0.0]}, ["left"])          # never received
    d.qpos[0] = 0.0
    ff.on_state(d, 1.0)
    assert ff.command(1.2)[0]["left"][0] != 0.0 and ff.command(1.2)[1] == []
    assert ff.command(1.21) == ({"left": [0.0]}, ["left"])         # older than max_state_age_s
    with pytest.raises(ValueError):
        arm_gravity.GravityFeedForward(m, 0.0, arms={"left": ["j1"]})


def test_declared_default_is_the_build1_real_driver_path(overrides):
    s = arm_gravity.settings(overrides)
    assert s == {"mode": "build1", "rate_hz": 100.0, "max_state_age_s": 0.2}
    ff = overrides["actuators"]["arm_gravity_feedforward"]
    assert ff["source"] in overrides["sources"] and ff["status"] == "project_proposal"


@pytest.fixture(scope="module")
def gravity_runs(generated, overrides):
    spec = bm.load_scenarios()
    g = generated[spec["configuration"]]
    return {mode: bm.run_mujoco(g["xml"], g["expansion"].text, overrides, spec, arm_gravity_mode=mode) for mode in ("build1", "off")}


def test_feedforward_removes_the_sag_with_the_driver_gains_unchanged(gravity_runs, models, overrides, evidence):
    on, off = gravity_runs["build1"]["results"], gravity_runs["off"]["results"]
    evidence("benchmark_mujoco_arm_gravity_off", gravity_runs["off"])
    assert gravity_runs["build1"]["arm_gravity_feedforward"]["mode"] == "build1"
    # tau 0 (main's real profile): the driver gains alone leave the arm below the command, as found on pull request 106
    assert off["arm_reach"]["max_joint_error_rad"] > 0.05 and off["joint_step"]["final_error_rad"] > 0.04
    # with the feed-forward the same gains hold the commanded joints
    assert on["arm_reach"]["max_joint_error_rad"] < 0.005 and on["joint_step"]["final_error_rad"] < 0.002
    assert gravity_runs["build1"]["arm_gravity_feedforward"]["left_arm_max_effort_fraction"] < 1.0
    m = models["bp000"]
    for k, g in overrides["actuators"]["arm_position_gains"]["per_joint"].items():
        a = m.actuator(f"openarm_left_{k}_position").id
        assert m.actuator_gainprm[a][0] == pytest.approx(g["kp"]) and -m.actuator_biasprm[a][2] == pytest.approx(g["kd"])


def test_wheel_contact_option_replaces_only_the_drive_wheel_geoms(generated):
    text = generated["bp000"]["xml"].read_text()
    before = ET.fromstring(text)
    for option, shape in bm.WHEEL_CONTACT.items():
        after = ET.fromstring(text)
        bm._with_wheel_contact(after, option)
        changed = [b.get("name") for b, a in zip(before.iter("geom"), after.iter("geom")) if b.attrib != a.attrib]
        if shape is None:
            assert changed == []
        else:
            assert sorted(changed) == sorted(bm.DRIVE_WHEEL_GEOMS)
            assert shape["radius"] == pytest.approx(float(shape["size"].split()[0]))
    # platform-sw pull request 171: radius 0.098; cylinder length 0.046, narrow 0.004 (MuJoCo takes the half length)
    assert bm.WHEEL_CONTACT["cylinder"]["size"] == "0.098 0.023" and bm.WHEEL_CONTACT["narrow_cylinder"]["size"] == "0.098 0.002"
