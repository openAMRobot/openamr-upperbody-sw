# SPDX-License-Identifier: MIT
"""tools/session_cycles.py without ROS: the leftover, daemon and vcan checks on a stand-in /proc and /sys/class/net,
and three real start/stop cycles of small shell sessions: a clean one passes, one that leaves a process behind, one
that never becomes ready (nothing running) and one that leaves a ROS 2 daemon each fail."""
import importlib.util
import os
import sys

from conftest import package_path

spec = importlib.util.spec_from_file_location("session_cycles", package_path("tools", "session_cycles.py"))
sc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(sc)


def fake_proc(root, pid, cmd, env=None, ppid=1):
    d = root / str(pid)
    d.mkdir(parents=True)
    (d / "cmdline").write_bytes(b"\0".join(part.encode() for part in cmd.split()) + b"\0")
    (d / "environ").write_bytes(b"\0".join(f"{k}={v}".encode() for k, v in (env or {}).items()) + b"\0")
    (d / "stat").write_text(f"{pid} ({cmd.split()[0]}) S {ppid} 0 0")


def test_leftovers_daemons_and_vcan_on_stand_ins(tmp_path):
    proc = tmp_path / "proc"
    fake_proc(proc, 10, "/usr/bin/bash runner")
    fake_proc(proc, 11, "/usr/bin/python3 /opt/ros/jazzy/bin/_ros2_daemon --rmw-implementation rmw_fastrtps_cpp")
    before = sc.processes(str(proc))
    fake_proc(proc, 20, "python3 control_room_monitor.py", env={sc.MARKER: "s1"})
    fake_proc(proc, 21, "sleep 300", env={sc.MARKER: "s1"})
    fake_proc(proc, 22, "sleep 300", env={sc.MARKER: "other"})
    fake_proc(proc, 23, "/usr/bin/python3 /opt/ros/jazzy/bin/_ros2_daemon --rmw-implementation rmw_cyclonedds_cpp")
    fake_proc(proc, 24, "gz sim -s -r world.sdf")     # no marker (sudo or a cleared environment), known name
    after = sc.processes(str(proc))
    left = sc.leftovers(before, after, "s1")
    assert sorted(left) == [20, 21, 23, 24]
    assert left[20]["why"] == "carries the session marker" and left[24]["why"] == "new process of a known stack program"
    assert 22 not in left and 10 not in left and 11 not in left          # another session's and pre-existing ones
    assert list(sc.new_daemons(before, after)) == [23]
    net = tmp_path / "net"
    for name, kind, device in (("eth0", "1", True), ("vcan_drive", "280", False), ("can0", "280", True)):
        (net / name).mkdir(parents=True)
        (net / name / "type").write_text(kind + "\n")
        if device:
            (net / name / "device").mkdir()
    assert sc.vcan_interfaces(str(net)) == ["vcan_drive"]


def test_verdict_needs_every_part():
    record = {"signal": "INT", "ready": {"cr": {"ok": True}}, "stopped": {"cr": {"exited": True}},
              "leftovers": {}, "daemons": {}, "vcan_left": []}
    assert sc.cycle_verdict(record) == (True, [])
    for key, value in (("ready", {"cr": {"ok": False, "detail": "no status"}}), ("stopped", {"cr": {"exited": False, "timeout": 5}}),
                       ("leftovers", {5: {"why": "x", "cmd": "sleep"}}), ("daemons", {6: {"cmd": "_ros2_daemon"}}),
                       ("vcan_left", ["vcan_aux"])):
        ok, reasons = sc.cycle_verdict(dict(record, **{key: value}))
        assert not ok and reasons, key


def session(command, **extra):
    return dict({"name": "s", "command": ["sh", "-c", command], "ready": {"line": "^ready$"}, "ready_timeout": 5,
                 "stop_timeout": 10}, **extra)


def test_three_clean_cycles_pass(tmp_path):
    records = sc.run({"cycles": 3, "stop_signals": ["INT", "TERM"], "processes": [session("echo ready; exec sleep 300")]},
                     str(tmp_path), settle=0.5)
    assert [r["ok"] for r in records] == [True, True, True], [r["reasons"] for r in records]
    assert [r["signal"] for r in records] == ["INT", "TERM", "INT"]


def test_a_background_job_that_ignores_sigint_is_found(tmp_path):
    """A background job of a non-interactive shell starts with SIGINT ignored: Ctrl-C stops the shell and leaves the
    job running (the reason run_grocery_demo_gui.sh starts its jobs with env --default-signal). SIGTERM reaches it."""
    records = sc.run({"cycles": 2, "stop_signals": ["INT", "TERM"], "processes": [session("sleep 300 & echo ready; wait")]},
                     str(tmp_path), settle=0.5)
    assert not records[0]["ok"] and any("orphan" in r and "sleep 300" in r for r in records[0]["reasons"]), records[0]
    assert records[1]["ok"], records[1]["reasons"]


def test_a_process_left_behind_fails(tmp_path):
    # setsid: the child leaves the process group, so the stop does not reach it; it still carries the marker.
    records = sc.run({"cycles": 1, "processes": [session("setsid sleep 300 & echo ready; wait")]}, str(tmp_path), settle=0.5)
    assert not records[0]["ok"] and any("orphan" in r and "sleep 300" in r for r in records[0]["reasons"]), records[0]
    marked = [p for p, i in sc.processes().items() if i["env"].get(sc.MARKER) == records[0]["session"]]
    assert marked == [], "the leftover was not cleaned up"


def test_nothing_running_fails(tmp_path):
    records = sc.run({"cycles": 1, "processes": [session("exit 0", ready_timeout=2)]}, str(tmp_path), settle=0.2)
    assert not records[0]["ok"] and "never became ready" in records[0]["reasons"][0]


def test_a_stale_daemon_fails(tmp_path):
    # A stand-in "daemon" in its own session, as `ros2` starts the real one; its name is what the check looks for.
    stub = tmp_path / "_ros2_daemon"
    stub.write_text("#!/bin/sh\nexec sleep 300\n")
    stub.chmod(0o755)
    records = sc.run({"cycles": 1, "processes": [session(f"setsid {stub} & echo ready; wait")]}, str(tmp_path), settle=0.5)
    assert not records[0]["ok"], records[0]
    assert any("stale ROS 2 daemon" in r or "orphan" in r for r in records[0]["reasons"]), records[0]["reasons"]
