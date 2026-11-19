# SPDX-License-Identifier: MIT
"""tools/check_recording.py on the topic counts of two recorded e2e runs, without ROS: the run that recorded no
/control_room/ topic (44445938978, Fast DDS at the LOCALHOST participant cap) must fail; the CycloneDDS run with the
raised participant index (44476577635) passes."""
import importlib.util
import json

from conftest import package_path

spec = importlib.util.spec_from_file_location("check_recording", package_path("tools", "check_recording.py"))
check_recording = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_recording)

REQUIRE = ["/control_room/can/decoded", "/control_room/can/summary"]


def fixture(name):
    return package_path("test", "fixtures", "recordings", name)


def test_run_without_control_room_topics_fails(capsys, tmp_path):
    jsonl = tmp_path / "status.jsonl"
    jsonl.write_text("".join(json.dumps({"sim_time": k}) + "\n" for k in range(40)))
    assert check_recording.main([fixture("e2e_44445938978"), "--require", *REQUIRE, "--status-jsonl", str(jsonl)]) == 1
    out = capsys.readouterr().out
    assert "recorded /control_room/ topics: 0" in out and "FAIL: /control_room/status has 0 messages" in out
    assert "discovery failure" in out


def test_run_with_the_raised_participant_index_passes(capsys):
    assert check_recording.main([fixture("e2e_44476577635"), "--require", *REQUIRE]) == 0
    out = capsys.readouterr().out
    assert "recorded /control_room/ topics: 16" in out and "PASS: /control_room/status 1339 messages" in out


def test_missing_bag_and_too_few_messages_fail(tmp_path, capsys):
    assert check_recording.main([str(tmp_path / "nothing")]) == 1
    ok, lines = check_recording.verdict({"/control_room/status": 3}, min_status=10)
    assert not ok and any("3 messages" in line for line in lines)
    ok, _ = check_recording.verdict({"/control_room/status": 30}, require=["/control_room/can/decoded"])
    assert not ok
