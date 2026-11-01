# SPDX-License-Identifier: MIT
"""P24: the exit code of every process of a launch is DATA in the sidecar and in the A7 report, so a crash like `parameter_bridge` -11 at shutdown is
not only a console line. No ROS graph, no simulator.

What is pinned: launch_guard records the codes BEFORE it asserts (a launch that fails the check still leaves them); the A7 report lists them per launch;
and a SYNTHETIC -11, planted in a sidecar, comes out of the report as a rate that tools/check_evidence_backing.py derives and backs (the canary); a
different number of launches or of crashes is not backed. Nothing is allow-listed by any of it."""
import importlib.util
import json
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, *[os.pardir] * 4))
sys.path.insert(0, os.path.join(HERE, os.pardir, os.pardir, "openamrobot_upperbody_description", "scripts"))
import launch_guard as lg  # noqa: E402


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def FakeProcInfo(codes):
    """What launch_testing hands a post-shutdown test: the real ProcInfoHandler, filled with ProcessExited-like events (a returncode, and an action
    with process_details['name']), so launch_testing's own assertExitCodes reads it as it reads a real launch."""
    from launch_testing.proc_info_handler import ProcInfoHandler

    class Action:             # hashable, as an ExecuteProcess is
        def __init__(self, name):
            self.process_details = {"name": name, "cmd": [name], "cwd": None, "env": None}

    class Event:
        def __init__(self, action, code):
            self.action, self.returncode, self.process_name = action, code, action.process_details["name"]

    handler = ProcInfoHandler()
    for name, code in codes.items():
        action = Action(name)
        handler.append(Event(action, code))
    return handler


def test_the_exit_codes_of_every_process_are_recorded_in_name_order():
    records = lg.process_exit_records(FakeProcInfo({"parameter_bridge-4": -11, "gazebo-1": -2, "move_group-7": -11, "spawner-3": 0, "odd-9": None}))
    assert records == [{"process": "gazebo-1", "exit_code": -2}, {"process": "move_group-7", "exit_code": -11}, {"process": "odd-9", "exit_code": None},
                       {"process": "parameter_bridge-4", "exit_code": -11}, {"process": "spawner-3", "exit_code": 0}]


def test_the_codes_are_recorded_before_the_check_fails_and_the_check_still_fails():
    seen = []
    info = FakeProcInfo({"parameter_bridge-4": -11, "gazebo-1": -2})
    with pytest.raises(AssertionError) as raised:
        lg.assert_launch_healthy(info, expected=["gazebo"], allowed=lg.SIM_ALLOWED, record=seen.append)
    assert "parameter_bridge-4 exited with code -11" in str(raised.value)       # -11 is still not allowed for parameter_bridge: nothing was widened
    assert seen == [[{"process": "gazebo-1", "exit_code": -2}, {"process": "parameter_bridge-4", "exit_code": -11}]]
    assert lg.SIM_ALLOWED["parameter_bridge"] == [lg.SIGABRT]


def test_a_launch_with_nothing_to_record_is_checked_as_before():
    lg.assert_launch_healthy(FakeProcInfo({"gazebo-1": -2, "move_group-2": -11}), expected=["gazebo"], allowed=lg.SIM_ALLOWED)
    with pytest.raises(AssertionError):
        lg.assert_launch_healthy(FakeProcInfo({"gazebo-1": -2}), expected=["gazebo", "move_group"], allowed=lg.SIM_ALLOWED, record=lambda r: None)


def sidecar(check, name, codes):
    return {"id": check, "file": name, "details": {"process_exits": [{"process": p, "exit_code": c} for p, c in codes.items()]}}


def synthetic_sidecars(crash_in="G02"):
    codes = lambda check: {"parameter_bridge-4": -11 if check == crash_in else 0, "gazebo-1": -2, "move_group-7": -11}
    return {c: [sidecar(c, f"{c}_x.json", codes(c))] for c in ("G01", "G02", "M02", "M07")}


def test_the_a7_report_lists_the_exit_codes_per_launch_and_skips_a_launch_that_recorded_none():
    b = load("build_a7_evidence_p24", os.path.join(HERE, os.pardir, "scripts", "build_a7_evidence.py"))
    sidecars = synthetic_sidecars()
    sidecars["M01"] = [{"id": "M01", "file": "M01_mock_L1000.json", "details": {}}]            # a launch whose test recorded nothing
    listed = b.process_exits_of(sidecars)
    assert [x["check"] for x in listed] == ["G01", "G02", "M02", "M07"]
    g02 = next(x for x in listed if x["check"] == "G02")
    assert {"process": "parameter_bridge-4", "exit_code": -11} in g02["processes"] and g02["sidecar"] == "G02_x.json"


def test_canary_a_synthetic_minus_11_in_a_sidecar_derives_a_rate_the_backing_check_backs(tmp_path):
    """The whole path with a planted fault: sidecars -> the report's process_exits -> a ledger entry of kind launch_exits -> BACKED, and only for the
    numbers the data give."""
    sys.path.insert(0, os.path.join(REPO, "tools"))
    import check_evidence_backing as ceb
    import yaml
    b = load("build_a7_evidence_p24b", os.path.join(HERE, os.pardir, "scripts", "build_a7_evidence.py"))
    (tmp_path / "evidence" / "reports").mkdir(parents=True)
    (tmp_path / "docs").mkdir()
    report = {"profile": "a7", "checks": [{"id": "G02", "result": "FAIL"}], "limitations": [], "process_exits": b.process_exits_of(synthetic_sidecars())}
    (tmp_path / "evidence" / "reports" / "20260101T000000Z_a7.json").write_text(json.dumps(report))
    entry = {"doc": "docs/BODY.md", "claim": "1 of 4", "kind": "launch_exits", "glob": "evidence/reports/*_a7.json", "process": "parameter_bridge", "exit_code": -11}

    def run(text, claim):
        (tmp_path / "docs" / "BODY.md").write_text(text)
        (tmp_path / "ledger.yaml").write_text(yaml.safe_dump({"claims": [{**entry, "claim": claim}]}))
        return ceb.run(str(tmp_path), str(tmp_path / "ledger.yaml"), ["docs/BODY.md"], [])

    good = run("The bridge ended in -11 in 1 of 4 launches.\n", "1 of 4")
    assert good["status"] == "PASS" and [r["status"] for r in good["claims"]] == ["BACKED"]
    for wrong in ("2 of 4", "1 of 5", "0 of 4"):
        bad = run(f"The bridge ended in -11 in {wrong} launches.\n", wrong)
        assert bad["status"] == "FAIL" and [r["status"] for r in bad["claims"]] == ["MISMATCH"], wrong
    # the same data without a crash gives 0 of 4, and a report from before P24 (no process_exits) contributes no launch at all
    (tmp_path / "evidence" / "reports" / "20260101T000000Z_a7.json").write_text(
        json.dumps({**report, "process_exits": b.process_exits_of(synthetic_sidecars(crash_in="none"))}))
    assert run("0 of 4 launches.\n", "0 of 4")["status"] == "PASS"
    (tmp_path / "evidence" / "reports" / "20260101T000000Z_a7.json").write_text(json.dumps({k: v for k, v in report.items() if k != "process_exits"}))
    assert run("0 of 0 launches.\n", "0 of 0")["status"] == "PASS" and run("1 of 4 launches.\n", "1 of 4")["status"] == "FAIL"
