# SPDX-License-Identifier: MIT
"""compare.py on fixture sessions: all inside the bands, one outside, and missing files (never a PASS)."""
import csv
import os

import yaml

import compare

HERE = os.path.dirname(os.path.abspath(__file__))


def session(d, rows):
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "metrics.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["test", "metric", "value", "unit", "source"])
        for m, v in rows:
            w.writerow(["T", m, v, "1", "fixture"])
    return d


def manual(d, t2_along=1.0, t5_mm=139.0):
    os.makedirs(d, exist_ok=True)
    for name in ("t1_lift", "t3_base_rotate"):
        src = os.path.join(HERE, "..", "templates", f"{name}_manual.csv")
        open(os.path.join(d, f"{name}_manual.csv"), "w").write(open(src).read())
    with open(os.path.join(d, "t2_base_straight_manual.csv"), "w") as f:
        f.write(f"leg,commanded_m,tape_along_m,tape_lateral_m,odom_m,instrument,notes\nforward,1.0,{t2_along},0.01,1.3,tape,\n")
    with open(os.path.join(d, "t5_gripper_manual.csv"), "w") as f:
        f.write(f"side,state,command_raw,caliper_jaw_mm,sign_observed,settle_observed_s,instrument,notes\nleft,open,0.7854,{t5_mm},,,,\n")
    return d


BANDS = {"lift_speed_mps": 0.018, "bands": [
    {"test": "T2", "metric": "(forward|back)_sim_over_commanded", "bench_metric": r"T2_bench_\1_real_over_commanded", "judge": "bench", "lo": 0.95, "hi": 1.05},
    {"test": "T4", "metric": r"openarm_left_joint\d_park_rms_rad", "judge": "robot", "lo": 0.0, "hi": 0.01},
    {"test": "T5", "metric": "(left)_(open)_model_jaw_gap_m", "bench_metric": r"T5_bench_\1_\2_jaw_gap_m", "judge": "delta", "lo": -0.005, "hi": 0.005}]}


def run(tmp_path, robot_rms, t2_along, t5_mm, drop=None):
    bands = tmp_path / "bands.yaml"
    bands.write_text(yaml.safe_dump(BANDS))
    sim = session(str(tmp_path / "sim"), [("forward_sim_over_commanded", 0.99), ("left_open_model_jaw_gap_m", 0.139), ("openarm_left_joint1_park_rms_rad", 0.001)])
    rob = session(str(tmp_path / "robot"), [("openarm_left_joint1_park_rms_rad", robot_rms)])
    man = manual(str(tmp_path / "manual"), t2_along, t5_mm)
    if drop:
        os.remove(os.path.join(man if drop.endswith("_manual.csv") else rob, drop))
    out = tmp_path / "report.md"
    rc = compare.main(["--sim", sim, "--robot", rob, "--manual", man, "--bands", str(bands), "--out", str(out)])
    return rc, out.read_text()


def test_all_inside_the_bands_passes(tmp_path):
    rc, text = run(tmp_path, 0.004, 1.01, 140.0)
    assert rc == 0, text
    assert "3 PASS, 0 FAIL, 0 MISSING" in text
    assert "Proposed" in text


def test_one_outside_fails(tmp_path):
    rc, text = run(tmp_path, 0.004, 0.80, 140.0)
    assert rc == 1
    assert "1 FAIL" in text
    assert "| T2 | `forward_sim_over_commanded` | 0.99 |  | 0.8 |" in text


def test_delta_band_fails_on_the_difference_not_the_value(tmp_path):
    rc, text = run(tmp_path, 0.004, 1.0, 150.0)
    assert rc == 1 and "1 FAIL" in text


def test_missing_robot_session_file_is_missing_not_pass(tmp_path):
    rc, text = run(tmp_path, 0.004, 1.0, 140.0, drop="metrics.csv")
    assert rc == 2
    assert "## Missing inputs" in text and "metrics.csv" in text
    assert "0 FAIL, 1 MISSING" in text


def test_missing_manual_file_is_listed(tmp_path):
    rc, text = run(tmp_path, 0.004, 1.0, 140.0, drop="t5_gripper_manual.csv")
    assert rc == 2
    assert "t5_gripper_manual.csv" in text


def test_floor_turn_angle_matches_common():
    import common
    pts = ((0.1, 0.2), (0.1, -0.38), (0.5, 0.0), (1.08, 0.05))
    assert compare.floor_turn_angle(*pts) == common.floor_turn_angle(*pts)


def test_bands_file_is_proposed_and_matches_the_lift_datum():
    import common
    b = yaml.safe_load(open(os.path.join(HERE, "..", "bands.yaml")))
    assert b["status"] == "proposed"
    assert b["lift_speed_mps"] == common.datum("lift_max_velocity_dold_v4_350")


def test_blank_template_cells_are_missing_not_fail(tmp_path):
    bands = tmp_path / "bands.yaml"
    bands.write_text(yaml.safe_dump({"lift_speed_mps": 0.018, "bands": [
        {"test": "T1", "metric": r"T1_bench_stroke_error_(L\d+)_([0-9.]+)", "judge": "bench", "lo": -0.005, "hi": 0.005}]}))
    out = tmp_path / "r.md"
    rc = compare.main(["--manual", os.path.join(HERE, "..", "templates"), "--bands", str(bands), "--out", str(out)])
    assert rc == 2
    assert "0 PASS, 0 FAIL, 1 MISSING" in out.read_text()
