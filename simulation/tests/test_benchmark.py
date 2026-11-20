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
    assert {k: v["verdict"] for k, v in doc["execution"].items()} == {"joint_step": "PASS", "arm_reach": "PASS", "lift_move": "PASS"}


# ------------------------------------------------------------------------- execution: success without motion is FAIL
# Regression for the Gazebo lift of #106 (CI job 136512164747 on 32995c8): lift_controller returned error_code 0 (SUCCESSFUL),
# /joint_states read -6.8e-15 m after a commanded 0.35 m, and the comparison took that as the reference.
def _lift_docs(gazebo_lift_m, mujoco_lift_m=0.3495):
    spec = bm.load_scenarios()
    rule = spec["scenarios"]["lift_move"]["execution"]
    gz = {"results": {"lift_move": {"final_position_m": gazebo_lift_m, "max_tracking_error_m": 0.35 - gazebo_lift_m,
                                    "base_pitch_change_deg": 0.0}},
          "execution": {"lift_move": bm.execution(0.35, {"joint_states": abs(gazebo_lift_m), "gazebo_link_pose": abs(gazebo_lift_m)},
                                                  True, rule)}}
    mj = {"results": {"lift_move": {"final_position_m": mujoco_lift_m, "max_tracking_error_m": 0.0008, "base_pitch_change_deg": 0.0025}},
          "execution": {"lift_move": bm.execution(0.35, {"qpos": mujoco_lift_m}, None, rule)}}
    return spec, gz, mj


def test_commanded_lift_move_with_no_measured_motion_is_fail_although_the_controller_reported_success():
    spec, gz, mj = _lift_docs(-6.810574904812846e-15)
    ex = gz["execution"]["lift_move"]
    assert ex["verdict"] == "FAIL" and ex["controller_reported_success"] is True
    assert "success without motion" in ex["reason"]
    rows = [r for r in bm.compare({"gazebo": gz, "mujoco": mj}, spec) if r["scenario"] == "lift_move"]
    assert rows and all(r["verdicts"]["mujoco"] == "FAIL" for r in rows)
    # base_pitch_change_deg is within its 0.2 deg tolerance (0 against 0.0025) and still may not pass: the lift never rose
    assert all("not executed in gazebo" in r["notes"]["mujoco"] for r in rows)


def test_two_backends_that_both_stood_still_do_not_agree():
    spec, gz, mj = _lift_docs(0.0, 0.0)
    rows = [r for r in bm.compare({"gazebo": gz, "mujoco": mj}, spec) if r["scenario"] == "lift_move"]
    assert all(r["verdicts"]["mujoco"] == "FAIL" for r in rows)


def test_one_measurement_short_is_enough_to_fail():
    rule = bm.load_scenarios()["scenarios"]["lift_move"]["execution"]
    assert bm.execution(0.35, {"joint_states": 0.35, "gazebo_link_pose": 0.0}, True, rule)["verdict"] == "FAIL"
    assert bm.execution(0.35, {"joint_states": None, "gazebo_link_pose": None}, True, rule)["verdict"] == "BLOCKED"
    assert bm.execution(0.35, {"joint_states": 0.3499, "gazebo_link_pose": None}, True, rule)["verdict"] == "PASS"


def test_executed_lift_compares_on_its_values():
    spec, gz, mj = _lift_docs(0.3499)
    rows = {r["metric"]: r for r in bm.compare({"gazebo": gz, "mujoco": mj}, spec) if r["scenario"] == "lift_move"}
    assert rows["final_position_m"]["verdicts"]["mujoco"] == "PASS"
    assert rows["base_pitch_change_deg"]["verdicts"]["mujoco"] == "PASS"


def test_a_document_without_an_execution_record_is_blocked_for_commanded_moves():
    spec, gz, mj = _lift_docs(0.3499)
    del gz["execution"]
    rows = [r for r in bm.compare({"gazebo": gz, "mujoco": mj}, spec) if r["scenario"] == "lift_move"]
    assert all(r["verdicts"]["mujoco"] == "BLOCKED" for r in rows)


def test_every_commanded_joint_move_has_an_execution_rule():
    spec = bm.load_scenarios()
    for name in ("joint_step", "arm_reach", "lift_move"):
        rule = spec["scenarios"][name]["execution"]
        assert 0.0 < rule["min_travel_fraction"] <= 1.0 and rule["why"], name


def test_gazebo_runner_never_takes_the_controller_result_as_motion():
    """Static: every FollowJointTrajectory result in the Gazebo runner feeds bm.execution with a measured travel."""
    import pathlib
    text = (pathlib.Path(bm.__file__).resolve().parents[1] / "benchmark" / "gazebo_benchmark.py").read_text()
    for scen in ("joint_step", "arm_reach", "lift_move"):
        assert f'executions["{scen}"] = bm.execution(' in text, scen
    assert '"gazebo_link_pose"' in text and "CARRIAGE" in text
