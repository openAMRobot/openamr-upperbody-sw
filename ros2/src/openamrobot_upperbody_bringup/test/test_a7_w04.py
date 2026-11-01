# SPDX-License-Identifier: MIT
"""W04 (P18): a gz server the launch's stop helper had to force is a WARN of the A7 report; a survivor is still the launch test's FAIL. No ROS graph.

The aggregation reads the stop-helper report each simulator launch test (G01, G02, M02, M07) leaves in its sidecar. NOT_RUN, never PASS, when one is
missing: a W04 that cannot see is not a W04 that saw nothing."""
import importlib.util
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def builder():
    spec = importlib.util.spec_from_file_location("build_a7_evidence_w04", os.path.join(HERE, os.pardir, "scripts", "build_a7_evidence.py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sidecar(state, **extra):
    return [{"details": {"gz_server_after_shutdown": {"survivors": [], "stop_helper": {"state": state, "elapsed_s": 1.4, **extra}}}}]


def sidecars(g01="stopped", g02="stopped", m02="stopped", m07="stopped"):
    return {"G01": sidecar(g01), "G02": sidecar(g02), "M02": sidecar(m02), "M07": sidecar(m07)}


def test_every_launch_stopped_on_request_is_a_pass():
    result, details = builder().gz_force_stop_check(sidecars())
    assert result == "PASS" and details["forced"] == []


def test_one_forced_server_is_a_warn_naming_the_launch():
    result, details = builder().gz_force_stop_check(sidecars(g01="force_stopped"))
    assert result == "WARN" and details["forced"] == ["G01"] and details["problems"]


def test_a_launch_with_no_helper_report_is_not_run_not_a_pass():
    b = builder()
    missing = sidecars()
    missing["G02"] = [{"details": {"gz_server_after_shutdown": {"survivors": [], "stop_helper": None}}}]
    assert b.gz_force_stop_check(missing)[0] == b.be.NOT_RUN
    assert b.gz_force_stop_check({"G01": sidecar("stopped")})[0] == b.be.NOT_RUN


def test_a_forced_server_of_m07_is_counted_too():
    result, details = builder().gz_force_stop_check(sidecars(m07="force_stopped"))
    assert result == "WARN" and details["forced"] == ["M07"]


def test_w04_is_listed_m07_stays_listed_and_w03_is_not_a_check():
    """M07 is Alex's and stands (P19); W03 is evidence files and a helper, not an entry of the report."""
    b = builder()
    ids = [i for i, _ in b.A7_CHECKS]
    assert "W04" in ids and "M07" in ids and "W03" not in ids
    assert "M07" in b.SIDECAR_REQUIRED and "W03" not in b.SIDECAR_REQUIRED


# --- evidence that is not there is never a PASS (Mohammad's review of 9406af6) ------------------------------------------------------------------


def with_helper(helper, launch="G02"):
    docs = sidecars()
    docs[launch] = [{"details": {"gz_server_after_shutdown": {"survivors": [], "stop_helper": helper}}}]
    return docs


def test_a_helper_report_still_in_state_started_is_not_run_not_a_pass():
    b = builder()
    result, details = b.gz_force_stop_check(with_helper({"state": "started", "shell_pid": 1, "retry_after_s": 3.0, "grace_s": 8.0}))
    assert result == b.be.NOT_RUN and "G02" in details["reason"] and "started" in details["reason"]


def test_a_nonempty_report_with_no_state_is_not_run():
    b = builder()
    assert b.gz_force_stop_check(with_helper({"shell_pid": 1, "elapsed_s": 1.4}))[0] == b.be.NOT_RUN


def test_an_unrecognised_or_malformed_state_is_not_run():
    b = builder()
    for helper in ({"state": "banana"}, {"state": None}, {"state": 7}, {"state": ["stopped"]}, {"state": ""}, {"state": "STOPPED"}):
        assert b.gz_force_stop_check(with_helper(helper))[0] == b.be.NOT_RUN, helper
    for helper in ("stopped", ["stopped"], 7):
        assert b.gz_force_stop_check(with_helper(helper))[0] == b.be.NOT_RUN, helper


def test_no_server_is_not_run_it_shows_no_server_stopping_on_request():
    b = builder()
    result, details = b.gz_force_stop_check(with_helper({"state": "no_server", "servers_at_request": []}))
    assert result == b.be.NOT_RUN and "no_server" in details["reason"]


def test_incomplete_evidence_beats_a_force_stop_elsewhere_and_both_are_reported():
    b = builder()
    docs = sidecars(g01="force_stopped")
    docs["G02"] = with_helper({"state": "started"})["G02"]
    result, details = b.gz_force_stop_check(docs)
    assert result == b.be.NOT_RUN and details["forced"] == ["G01"] and "G02" in details["reason"]


def test_the_controls_still_hold_completed_stopped_force_stopped_and_absent():
    """Mohammad's three controls: completed `stopped` PASS, `force_stopped` WARN, an absent helper NOT_RUN."""
    b = builder()
    assert b.gz_force_stop_check(sidecars())[0] == "PASS"
    assert b.gz_force_stop_check(sidecars(m02="force_stopped"))[0] == "WARN"
    assert b.gz_force_stop_check(with_helper(None))[0] == b.be.NOT_RUN


def test_the_timed_out_helper_path_end_to_end(tmp_path, monkeypatch):
    """The path the absent-helper case does not cover: a helper that wrote {"state": "started"} and never finished. launch_guard's reader waits for
    it, gives up, and gz_server_shutdown_report keeps that object in the sidecar; W04 must read it as not run. Nothing here is faked but the report
    file (written by hand in the helper's own first-write shape) and the wait, which is shortened so the test takes a fraction of a second."""
    sys.path.insert(0, os.path.join(HERE, os.pardir, os.pardir, "openamrobot_upperbody_description", "scripts"))
    import launch_guard as lg
    report_file = tmp_path / "gz_stop_w04_timeout.json"
    report_file.write_text(json.dumps({"state": "started", "shell_pid": 4242, "retry_after_s": 3.0, "grace_s": 8.0, "retried_with_service": False}))
    monkeypatch.setattr(lg, "gz_stop_report_path", lambda partition: str(report_file))
    original = lg.read_gz_stop_report
    monkeypatch.setattr(lg, "read_gz_stop_report", lambda partition: original(partition, wait_s=0.2, poll_s=0.05))
    shutdown = lg.gz_server_shutdown_report("w04_timeout", grace_s=0.1, reap=False, poll_s=0.05)
    assert shutdown["stop_helper"]["state"] == "started" and shutdown["survivors"] == []        # what reaches the sidecar
    b = builder()
    docs = sidecars()
    docs["M02"] = [{"details": {"gz_server_after_shutdown": shutdown}}]
    result, details = b.gz_force_stop_check(docs)
    assert result == b.be.NOT_RUN and "M02" in details["reason"] and "started" in details["reason"]
    # and the same report once the helper HAS finished is what PASS is made of
    report_file.write_text(json.dumps({"state": "stopped", "elapsed_s": 1.4, "servers_at_request": [1]}))
    done = lg.gz_server_shutdown_report("w04_timeout", grace_s=0.1, reap=False, poll_s=0.05)
    docs["M02"] = [{"details": {"gz_server_after_shutdown": done}}]
    assert b.gz_force_stop_check(docs)[0] == "PASS"
