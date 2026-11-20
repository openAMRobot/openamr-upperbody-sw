# SPDX-License-Identifier: MIT
"""Backend benchmark: command shaping, metrics, comparison verdicts and the MuJoCo run of every scenario."""
import math

import pytest

from openamr_sim import benchmark as bm


def test_scenario_file_states_a_tolerance_and_a_reason_for_every_metric():
    spec = bm.load_scenarios()
    assert tuple(spec["scenarios"]) == bm.ORDER
    for name, s in spec["scenarios"].items():
        assert s["metrics"], name
        for metric, tol in s["metrics"].items():
            assert ("tolerance_abs" in tol) != ("tolerance_rel" in tol), (name, metric)
            assert tol.get("why"), (name, metric)


def test_limiter_holds_the_acceleration_and_velocity_limits():
    values, integral = bm.shaped_profile(2.0, 3.0, 2.0, 1.0, 0.5, -0.5, 0.01)
    steps = [b - a for a, b in zip([0.0] + values, values)]
    assert max(abs(s) for s in steps) <= 0.5 * 0.01 + 1e-12
    assert max(values) == pytest.approx(1.0)
    assert values[-1] == pytest.approx(0.0, abs=1e-12)
    # ramp up 2 s to 1.0, cruise, ramp down 2 s after the command ends at 3 s: area = 3 - 1 + 1 = 3.0
    assert integral == pytest.approx(3.0, abs=0.02)


def test_wheel_speeds_are_the_diffdrive_kinematics():
    left, right = bm.wheel_speeds(0.2, 0.5, 0.4, 0.1)
    assert (left, right) == pytest.approx(((0.2 - 0.1) / 0.1, (0.2 + 0.1) / 0.1))


def test_cubic_and_ramp_end_where_they_should():
    assert bm.cubic(0.0, 1.0, 2.0, 0.0) == 0.0 and bm.cubic(0.0, 1.0, 2.0, 2.0) == 1.0
    assert bm.cubic(0.0, 1.0, 2.0, 1.0) == pytest.approx(0.5)
    assert bm.lift_ramp(0.0, 0.35, 0.018, 100.0) == 0.35 and bm.lift_ramp(0.35, 0.0, 0.018, 1.0) == pytest.approx(0.332)


def test_unwrap_follows_a_full_turn():
    angles = [((0.3 * k) + math.pi) % (2 * math.pi) - math.pi for k in range(25)]
    assert bm.unwrap(angles)[-1] - bm.unwrap(angles)[0] == pytest.approx(0.3 * 24)


def test_planar_motion_in_the_start_heading():
    mo = bm.planar_motion([0, 0, 0], math.pi / 2, [0.01, 2.0, 0], [math.pi / 2, math.pi / 2 + 0.05])
    assert mo["along_m"] == pytest.approx(2.0) and mo["lateral_m"] == pytest.approx(-0.01) and mo["yaw_change_rad"] == pytest.approx(0.05)


def test_diffdrive_values_come_from_the_expansion(expansions):
    dd = bm.diffdrive_params(expansions["bp000"].text)
    assert dd["wheel_separation"] == pytest.approx(0.400) and dd["wheel_radius"] == pytest.approx(0.100)
    assert dd["max_linear_acceleration"] == pytest.approx(0.5) and dd["max_angular_acceleration"] == pytest.approx(1.0)


def test_compare_verdicts():
    spec = bm.load_scenarios()
    ref = {"results": {"rest": {"base_height_m": 0.0, "base_pitch_deg": 0.5, "base_roll_deg": 0.0},
                       "straight_drive": {"distance_m": 2.0}}}
    other = {"results": {"rest": {"base_height_m": 0.0025, "base_pitch_deg": 0.75, "base_roll_deg": 0.0},
                         "straight_drive": {"distance_m": 2.05}}}
    rows = {(r["scenario"], r["metric"]): r for r in bm.compare({"gazebo": ref, "mujoco": other}, spec)}
    assert rows[("rest", "base_height_m")]["verdicts"]["mujoco"] == "PASS"
    assert rows[("rest", "base_pitch_deg")]["verdicts"]["mujoco"] == "FAIL"
    assert rows[("straight_drive", "distance_m")]["verdicts"]["mujoco"] == "FAIL"  # 2.5 % against 2 % relative
    assert rows[("object_contact", "rest_height_m")]["verdicts"]["mujoco"] == "BLOCKED"
    assert all(r["verdicts"]["isaac"] == "BLOCKED" for r in bm.compare({"gazebo": ref, "isaac": None}, spec))


def test_mujoco_runs_every_scenario(generated, overrides, evidence):
    spec = bm.load_scenarios()
    g = generated[spec["configuration"]]
    doc = bm.run_mujoco(g["xml"], g["expansion"].text, overrides, spec)
    evidence("benchmark_mujoco", doc)
    for name, s in spec["scenarios"].items():
        assert set(s["metrics"]) <= set(doc["results"][name]), name
        assert all(math.isfinite(v) for v in doc["results"][name].values()), name
    r = doc["results"]
    assert r["object_contact"]["rest_height_m"] == pytest.approx(0.05, abs=0.001)
    assert r["straight_drive"]["distance_m"] > 1.9
    assert r["lift_move"]["final_position_m"] == pytest.approx(0.35, abs=0.002)
