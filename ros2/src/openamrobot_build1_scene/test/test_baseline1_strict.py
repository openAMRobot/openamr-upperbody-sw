# SPDX-License-Identifier: MIT
"""The strict Baseline 1 verdict of the acceptance workflow (scripts/baseline1_strict_verdict.py), without ROS.

PASS needs two legs, each 8 of 8, on identical pinned commits, CycloneDDS with the raised participant index at runtime,
station alignment on and used, and the sim-oracle profile declared. Every other case is FAIL, and one leg is NOT STRICT.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))
import baseline1_strict_verdict as strict  # noqa: E402
from test_baseline1_phases import run_mission  # noqa: E402

URI = ('<CycloneDDS><Domain Id="any"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>'
       '</Domain></CycloneDDS>')
PINS = {"openamr-upperbody-sw": "a" * 40, "openamr-platform-sw": "b" * 40, "openamrobot-interfaces": "c" * 40,
        "openarm_ros2": "d" * 40, "openarm_can": "e" * 40, "openarm_description": "f" * 40}


def declaration(leg, **changes):
    value = {"leg": leg, "run_id": 1, "pins": dict(PINS), "profile": "sim-oracle", "station_approach": "1",
             "station_alignment": "1", "rmw_implementation": "rmw_cyclonedds_cpp", "rmw_runtime": "rmw_cyclonedds_cpp",
             "cyclonedds_uri": URI, "seed": "20261117", "build_ok": 1, "env_ok": 1, "env_reason": ""}
    value.update(changes)
    return value


def mission(failing_reach=False, action=strict.ALIGNMENT_ACTION, pose_source="ground_truth"):
    _log, tracker = run_mission(failing_reach=failing_reach)
    return {"result": "FAIL" if failing_reach else "PASS", "phases": tracker.result(), "failures": [], "trace": [],
            "run": {"profile": "firmware_sil", "planning_pose_source": pose_source},
            "baseline1": {"navigate_action": action}}


def write_leg(root, leg, decl=None, data=None):
    leg_dir = root / f"baseline1-acceptance-leg{leg}-attempt1"
    (leg_dir / "ws" / "build1_logs").mkdir(parents=True)
    (leg_dir / "declaration.json").write_text(json.dumps(decl if decl is not None else declaration(leg)))
    if data is not False:
        (leg_dir / "ws" / "build1_logs" / "baseline1_mission.json").write_text(json.dumps(data or mission()))
    return str(leg_dir)


def verdict(tmp_path, legs, *extra):
    out = tmp_path / "verdict.json"
    rc = strict.main([*legs, "--out", str(out), *extra])
    return rc, json.loads(out.read_text())


def test_two_legs_8_of_8_on_the_same_pins_pass(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2)])
    assert (rc, record["verdict"], record["problems"]) == (0, "PASS", [])
    assert record["pins"] == PINS and [leg["phases_passed"] for leg in record["legs"]] == [8, 8]


def test_one_leg_short_of_8_of_8_fails(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2, data=mission(failing_reach=True))])
    assert (rc, record["verdict"]) == (1, "FAIL")
    assert any("leg 2" in p and "phases passed" in p for p in record["problems"])


def test_different_pinned_commits_fail(tmp_path):
    pins = dict(PINS, **{"openamr-platform-sw": "9" * 40})
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2, decl=declaration(2, pins=pins))])
    assert (rc, record["verdict"]) == (1, "FAIL") and "the legs ran on different pinned commits" in record["problems"]


def test_a_missing_leg_or_mission_log_fails(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1)])
    assert (rc, record["verdict"]) == (1, "FAIL")
    other = tmp_path / "b"
    other.mkdir()
    rc, record = verdict(other, [write_leg(other, 1), write_leg(other, 2, data=False)])
    assert rc == 1 and any("no baseline1_mission.json" in p for p in record["problems"])


def test_fast_dds_or_the_default_participant_cap_fail(tmp_path):
    fast = declaration(2, rmw_runtime="rmw_fastrtps_cpp")
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2, decl=fast)])
    assert rc == 1 and any("middleware 'rmw_fastrtps_cpp'" in p for p in record["problems"])
    other = tmp_path / "b"
    other.mkdir()
    low = URI.replace(">120<", ">32<")
    rc, record = verdict(other, [write_leg(other, 1, decl=declaration(1, cyclonedds_uri=low)),
                                 write_leg(other, 2, decl=declaration(2, cyclonedds_uri=low))])
    assert rc == 1 and any("MaxAutoParticipantIndex 32" in p for p in record["problems"])


def test_alignment_must_be_declared_and_used(tmp_path):
    direct = mission(action="/navigate_to_pose")
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1, data=direct), write_leg(tmp_path, 2)])
    assert rc == 1 and any("not /station_alignment/navigate_to_station" in p for p in record["problems"])
    other = tmp_path / "b"
    other.mkdir()
    rc, record = verdict(other, [write_leg(other, 1, decl=declaration(1, station_alignment="0")),
                                 write_leg(other, 2, decl=declaration(2, station_alignment="0"))])
    assert rc == 1 and any("station_alignment declared '0'" in p for p in record["problems"])


def test_the_sim_oracle_profile_must_be_declared_in_the_run_identity(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1, data=mission(pose_source="map")), write_leg(tmp_path, 2)])
    assert rc == 1 and any("planning_pose_source 'map'" in p for p in record["problems"])


def test_one_validation_leg_is_never_strict(tmp_path, capsys):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1)], "--expect", "1")
    assert (rc, record["verdict"], record["problems"]) == (1, "NOT STRICT", [])
    assert "never the strict two-run acceptance" in capsys.readouterr().out


def test_find_reads_one_leg_at_the_root_and_two_legs_in_their_own_directories(tmp_path):
    """download-artifact: one matching artifact lands straight in the path, several land in one directory each."""
    single = tmp_path / "single"
    leg_dir = write_leg(single, 1)
    for name in os.listdir(leg_dir):
        os.rename(os.path.join(leg_dir, name), single / name)
    rc, record = verdict(tmp_path, ["--find", str(single), "--expect", "1"])
    assert (record["verdict"], record["problems"]) == ("NOT STRICT", [])
    double = tmp_path / "double"
    write_leg(double, 1), write_leg(double, 2)
    rc, record = verdict(tmp_path, ["--find", str(double)])
    assert (rc, record["verdict"]) == (0, "PASS") and len(record["legs"]) == 2
    empty = tmp_path / "empty"
    empty.mkdir()
    rc, record = verdict(tmp_path, ["--find", str(empty)])
    assert (rc, record["verdict"]) == (1, "FAIL") and "legs found [], expected 1 to 2" in record["problems"]
