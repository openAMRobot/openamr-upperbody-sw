# SPDX-License-Identifier: MIT
"""MuJoCo physics checks, each inside MuJoCo only. Expectations come from the model's own parameters or from pinocchio.

No result here is compared with Gazebo or Isaac Sim. Parity is not fidelity: these say the MuJoCo model behaves as its parameters
predict, not that it behaves like the robot.
"""
import math

import mujoco
import numpy as np
import pytest

from openamr_sim import physics as ph
from openamr_sim.urdf_model import URDF

CONFIGS = ["bp000", "bp050"]
DRIVE_AND_CASTOR_WHEELS = ["left_wheel_link", "right_wheel_link", "fl_wheel_link", "fr_wheel_link", "bl_wheel_link", "br_wheel_link"]


# ---------------------------------------------------------------------------------------------------- static gravity torques
@pytest.mark.parametrize("name", CONFIGS)
def test_static_gravity_torques_match_pinocchio(models, generated, name, evidence):
    """qfrc_bias at rest against pinocchio's generalized gravity, at the named configurations and 50 seeded configurations."""
    from openamr_sim.parity import named_configurations, tolerances
    tol = tolerances()["gravity_torque"]
    m = models[name]
    d = mujoco.MjData(m)
    text = generated[name]["expansion"].text
    u = URDF(text)
    pg = ph.PinocchioGravity(text)
    rng = np.random.default_rng(58)
    configs = list(named_configurations(u).values())
    for _ in range(50):
        q = {}
        for j in u.movable():
            lo, hi = (j.lower, j.upper) if j.type != "continuous" else (-math.pi, math.pi)
            q[j.name] = float(rng.uniform(lo, hi))
        configs.append(u.resolve(q))
    worst, compared = 0.0, 0
    for q in configs:
        a, b = ph.mujoco_gravity(mujoco, m, d, q), pg.torques(q)
        assert set(b) <= set(a)
        for k, ref in b.items():
            err = abs(a[k] - ref)
            worst = max(worst, err)
            compared += 1
            assert err <= tol["abs"] + tol["rel"] * abs(ref), (k, a[k], ref)
    evidence(f"physics_gravity_{name}", {"configurations": len(configs), "joint_values_compared": compared, "max_abs_error": worst})


# ---------------------------------------------------------------------------------------------------- base on a flat floor
@pytest.mark.parametrize("name", CONFIGS)
def test_base_contacts_on_flat_floor(models, name, evidence):
    m = models[name]
    r = ph.check_base_contacts(mujoco, m, mujoco.MjData(m), "L1000_home", DRIVE_AND_CASTOR_WHEELS)
    evidence(f"physics_base_contacts_{name}", r)
    assert r["only_floor_contacts"]
    assert not r["unexpected_links"], r["unexpected_links"]
    assert {"left_wheel_link", "right_wheel_link"} <= set(r["touching_links"])
    assert any(l.endswith("_wheel_link") and l[:2] in ("fl", "fr", "bl", "br") for l in r["touching_links"])
    assert abs(r["normal_force_sum_N"] - r["weight_N"]) <= 1e-3 * r["weight_N"]
    assert r["max_abs_qvel"] < 1e-2
    assert abs(r["base_roll_deg"]) < 0.05


@pytest.mark.parametrize("name", CONFIGS)
@pytest.mark.parametrize("direction", ["forward", "backward"])
def test_castor_swivel(models, name, direction, overrides, evidence):
    """Driving straight, every castor on the floor swivels to trail its mount: 0 rad forward, pi backward."""
    m = models[name]
    prefixes = overrides["passive_castors"]["prefixes"]
    speed, start, expect = (3.0, math.pi / 2, 0.0) if direction == "forward" else (-3.0, 0.0, math.pi)
    r = ph.check_castor_swivel(mujoco, m, mujoco.MjData(m), "L1000_home", prefixes, speed, 4.0, start)
    evidence(f"physics_castor_swivel_{name}_{direction}", r)
    assert math.copysign(1, r["travel_m"][0]) == math.copysign(1, speed) and abs(r["travel_m"][0]) > 1.0
    on_floor = [p for p in prefixes if r["floor_contact_fraction"][p] > 0.9]
    assert on_floor, "no castor on the floor"
    for p in on_floor:
        assert abs(ph.wrap(r["swivel_rad"][p] - expect)) < math.radians(2.0), (p, r["swivel_rad"][p])


# ---------------------------------------------------------------------------------------------------- lift travel
@pytest.mark.parametrize("name", CONFIGS)
def test_lift_travel(models, generated, name, overrides, evidence):
    m = models[name]
    text = generated[name]["expansion"].text
    lift = URDF(text).joints["lift_joint"]
    r = ph.check_lift_travel(mujoco, m, mujoco.MjData(m), "L1000_home", overrides, lift.lower, lift.upper, lift.velocity)
    evidence(f"physics_lift_travel_{name}", r)
    sag = overrides["actuators"]["lift_position_servo"]["parameters"]["sag"]["value"]
    omega = math.sqrt(9.81 / sag)
    lag = 2 * overrides["actuators"]["lift_position_servo"]["parameters"]["zeta"]["value"] * lift.velocity / omega
    assert r["controller_updates"] == round(r["simulated_s"] * overrides["common"]["controller_rate"]["value"])
    assert 0.0 <= r["final_error_m"] <= 1.05 * sag
    assert r["max_tracking_error_m"] <= 1.1 * (sag + lag)
    assert r["max_position_m"] <= lift.upper + 1e-4
    assert r["max_force_N"] < r["force_limit_N"]
    stops = generated[name]["report"]["keyframes"]
    assert "L1350_home" in stops
    # The shoulder height above the floor frame equals the L1350 stop, less the servo's static error.
    assert abs(r["shoulder_height_in_base_m"] - (1.350 - r["final_error_m"])) < 1e-6
    assert abs(r["base_pitch_deg"]) < 2.0


# ---------------------------------------------------------------------------------------------------- arms holding at 750 Hz
@pytest.mark.parametrize("name", CONFIGS)
@pytest.mark.parametrize("key", ["L1175_hands_up", "L1175_park", "L1350_hands_up", "L1000_home"])
def test_both_arms_hold_against_gravity_at_750_hz(models, generated, name, key, overrides, evidence):
    m = models[name]
    r = ph.check_arm_hold(mujoco, m, mujoco.MjData(m), key, overrides, generated[name]["expansion"].text)
    evidence(f"physics_arm_hold_{name}_{key}", r)
    assert r["controller_rate_hz"] == pytest.approx(750.0, abs=1e-9)
    assert r["controller_updates"] == round(r["simulated_s"] * 750)
    for jn, v in r["joints"].items():
        assert abs(v["velocity"]) < 1e-3, (jn, v)
        assert abs(v["actuator_torque"]) < 0.95 * v["force_limit"], (jn, v)
        expected = v["gravity_torque_pinocchio"]
        if "follower" in v:
            # The follower's weight reaches the leader through the mimic constraint: tau = g1 + multiplier * g2.
            expected += v["follower_multiplier"] * v["gravity_torque_follower_pinocchio"]
        # At rest the actuator carries gravity, up to the joint's static friction (frictionloss) and a small numerical margin.
        assert abs(v["actuator_torque"] - expected) <= v["frictionloss"] + 1e-3, (jn, v)
    for side, err in r["mimic_error_rad"].items():
        assert abs(err) < 1e-3, (side, err)
