# SPDX-License-Identifier: MIT
"""The strict Baseline 1 verdict of the acceptance workflow (scripts/baseline1_strict_verdict.py), without ROS.

PASS needs two legs of one dispatch, each 8 of 8, on identical pinned commits, CycloneDDS with the raised participant
index at runtime, station alignment on and used, the sim-oracle profile declared, and the gates of baseline1_gates.py:
mission result PASS, component counters alive for the whole run, Stop and cancellation checks, no unaccounted contact,
complete evidence. Every other case is FAIL, and one leg is NOT STRICT.
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, os.path.join(os.path.dirname(HERE), "scripts"))
import baseline1_strict_verdict as strict  # noqa: E402
from openamrobot_build1_scene import baseline1_gates as gates  # noqa: E402
from test_baseline1_phases import run_mission  # noqa: E402

URI = ('<CycloneDDS><Domain Id="any"><Discovery><MaxAutoParticipantIndex>120</MaxAutoParticipantIndex></Discovery>'
       '</Domain></CycloneDDS>')
PINS = {"openamr-upperbody-sw": "a" * 40, "openamr-platform-sw": "b" * 40, "openamrobot-interfaces": "c" * 40,
        "openarm_ros2": "d" * 40, "openarm_can": "e" * 40, "openarm_description": "f" * 40}


def declaration(leg, **changes):
    value = {"leg": leg, "run_id": 1, "run_attempt": 1, "pins": dict(PINS), "profile": "sim-oracle", "station_approach": "1",
             "station_alignment": "1", "rmw_implementation": "rmw_cyclonedds_cpp", "rmw_runtime": "rmw_cyclonedds_cpp",
             "cyclonedds_uri": URI, "seed": "20261117", "build_ok": 1, "env_ok": 1, "env_reason": ""}
    value.update(changes)
    return value


STOP_CHECKS = ("stop_settle_time", "stop_base_speed", "stop_base_yaw_rate", "authorize_motion_without_permit",
               "authorize_command_refused", "authorize_permit_fresh")
COUNTERS = ("live_drive_samples", "live_bms_answers", "live_lift_permit_rx", "live_joint_states")
GRASP = {"phase": "grasp", "a": "openarm_left_ee_link1", "b": "item::link::collision", "first_t": 12.0, "last_t": 13.0,
         "count": 40, "max_force_n": 6.1}


def check(name, value=True):
    return {"name": name, "value": value, "ok": value is True or isinstance(value, float), "low": None, "high": None}


def mission(failing_reach=False, action=strict.ALIGNMENT_ACTION, pose_source="ground_truth", cancel=True,
            dead_counter=None, events=(GRASP,), sensors_on=True):
    """A mission log as baseline1.mission writes it: phases, trace with the work counters, contacts, run identity."""
    _log, tracker = run_mission(failing_reach=failing_reach)
    records = tracker.result()
    stop = next(r for r in records if r["phase"] == "stop_authorize")
    stop["quantities"] += [check(name) for name in STOP_CHECKS]
    if cancel:
        stop["quantities"].append(check("cancel_lift_goal_held_still"))
    trace = []
    for i in range(60):                       # 0.5 s of Gazebo time per sample, 30 s in all
        sample = {"sim": float(i) * 0.5, "phase": None}
        for key in COUNTERS:
            sample[key] = (min(i, 10) if key == dead_counter else i)   # the dead counter stops at 5 s
        trace.append(sample)
    return {"result": "FAIL" if failing_reach else "PASS", "phases": records, "failures": [], "trace": trace,
            "run": {"profile": "firmware_sil", "planning_pose_source": pose_source},
            "baseline1": {"navigate_action": action, "seed": 20261117},
            "contacts": {"sensors_on": sensors_on, "sensor_links": [f"link{i}" for i in range(21)],
                         "messages": {"openarm_left_ee_link1": 40}, "events": list(events)}}


def write_leg(root, leg, decl=None, data=None, buses=gates.load()["evidence"]["can_buses"], bag=True):
    leg_dir = root / f"baseline1-acceptance-leg{leg}-attempt1"
    logs = leg_dir / "ws" / "build1_logs"
    logs.mkdir(parents=True)
    (leg_dir / "declaration.json").write_text(json.dumps(decl if decl is not None else declaration(leg)))
    if data is not False:
        (logs / "baseline1_mission.json").write_text(json.dumps(data or mission()))
    if bag:
        (logs / "bag").mkdir()
        (logs / "bag" / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n")
    for bus in buses:
        (logs / f"record_candump_{bus}.log").write_text(f"(1.0) {bus} 601#2300\n")
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


# ---------------------------------------------------------------- gates added 20 Nov 2026 (Alex's acceptance rules)
def two(tmp_path, data1=None, data2=None, **kw):
    return verdict(tmp_path, [write_leg(tmp_path, 1, data=data1, **kw), write_leg(tmp_path, 2, data=data2, **kw)])


def test_legs_of_different_dispatches_fail(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2, decl=declaration(2, run_id=2))])
    assert rc == 1 and any("different dispatches" in p for p in record["problems"])


def test_a_failed_check_outside_the_phases_fails(tmp_path):
    """rig.run adds the component liveness checks of #95 after the mission function: outside every phase."""
    data = mission()
    data["result"], data["failures"] = "FAIL", ["host_client_bms_alive = False bool, allowed True to True"]
    rc, record = two(tmp_path, data2=data)
    assert rc == 1 and any("outside the phases: host_client_bms_alive" in p for p in record["problems"])
    assert any("mission result 'FAIL'" in p for p in record["problems"])


def test_a_component_that_stops_during_the_run_fails(tmp_path):
    rc, record = two(tmp_path, data2=mission(dead_counter="live_bms_answers"))
    assert rc == 1 and any("liveness battery" in p and "did not advance for 24.5 s" in p for p in record["problems"])


def test_a_run_without_the_work_counters_fails(tmp_path):
    data = mission()
    for sample in data["trace"]:
        del sample["live_lift_permit_rx"]
    rc, record = two(tmp_path, data2=data)
    assert rc == 1 and any("liveness lift: counter live_lift_permit_rx not recorded" in p for p in record["problems"])


def test_a_stop_check_that_did_not_run_fails(tmp_path):
    data = mission()
    stop = next(r for r in data["phases"] if r["phase"] == "stop_authorize")
    stop["quantities"] = [q for q in stop["quantities"] if q["name"] != "authorize_command_refused"]
    rc, record = two(tmp_path, data2=data)
    assert rc == 1 and any("check authorize_command_refused did not run" in p for p in record["problems"])


def test_a_mission_without_a_cancellation_check_fails(tmp_path):
    """The Baseline 1 mission of 20 Nov 2026 cancels nothing: the strict verdict cannot pass it."""
    rc, record = two(tmp_path, data1=mission(cancel=False), data2=mission(cancel=False))
    assert rc == 1 and any("cancellation: 0 cancellation checks ran" in p for p in record["problems"])


def test_an_unaccounted_contact_fails_and_an_allowed_one_passes(tmp_path):
    palm = {"phase": "retreat", "a": "openarm_left_ee_base_link", "b": "fixtures::work_shelf_board_1",
            "first_t": 20.0, "last_t": 20.2, "count": 3, "max_force_n": 4.9}
    rc, record = two(tmp_path, data2=mission(events=(GRASP, palm)))
    assert rc == 1 and any("unaccounted contact in retreat: openarm_left_ee_base_link" in p for p in record["problems"])
    other = tmp_path / "b"
    other.mkdir()
    early = dict(GRASP, phase="arm_approach")           # a finger on the item before the grasp is not intended
    rc, record = two(other, data2=mission(events=(early,)))
    assert rc == 1 and any("unaccounted contact in arm_approach" in p for p in record["problems"])


def test_contact_sensors_off_or_not_recorded_fail(tmp_path):
    rc, record = two(tmp_path, data2=mission(sensors_on=False))
    assert rc == 1 and any("contact sensors were not on" in p for p in record["problems"])
    other = tmp_path / "b"
    other.mkdir()
    data = mission()
    del data["contacts"]
    rc, record = two(other, data2=data)
    assert rc == 1 and any("contacts: not recorded" in p for p in record["problems"])


def test_incomplete_evidence_fails(tmp_path):
    rc, record = verdict(tmp_path, [write_leg(tmp_path, 1), write_leg(tmp_path, 2, bag=False, buses=("vcan_drive",))])
    assert rc == 1
    assert any("no ROS bag" in p for p in record["problems"]) and any("no candump of vcan_bms" in p for p in record["problems"])
    other = tmp_path / "b"
    other.mkdir()
    data = mission()
    data["baseline1"]["seed"] = 1
    del data["run"]
    short = dict(PINS, openarm_can="e" * 7)
    rc, record = verdict(other, [write_leg(other, 1, decl=declaration(1, pins=short)),
                                 write_leg(other, 2, decl=declaration(2, pins=short), data=data)])
    assert rc == 1
    assert any("openarm_can missing or not a full hash" in p for p in record["problems"])
    assert any("mission seed 1 differs" in p for p in record["problems"])


def test_contact_events_are_grouped_per_phase_and_pair():
    rows = [{"t": 1.0, "a": "x", "b": "y", "fn": 2.0}, {"t": 1.5, "a": "x", "b": "y", "fn": -3.0},
            {"t": 9.0, "a": "x", "b": "y", "fn": float("nan")}]
    events = gates.contact_events(rows, lambda t: "grasp" if t < 5 else None)
    assert [(e["phase"], e["count"], e["max_force_n"]) for e in events] == [("grasp", 2, 3.0), ("outside", 1, 0.0)]
