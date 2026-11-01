# SPDX-License-Identifier: MIT
"""The detector of a simulator that outlives its launch (launch_guard.gz_server_shutdown_report), on decoy processes. No ROS graph, no Gazebo.

P16 task 2. The decoys are `sleep` processes whose argv[0] is `gz sim -r -s ...`, which is what the forked server's command line looks like
(it rewrites argv into one string; seen on the orphan of the first G02 run of P16), with `GZ_PARTITION` set or not. What the tests pin:
  - it keys on THIS run's partition: a `gz sim` of another partition, and one with none, are never matched;
  - it does not match what only mentions gz (a `gz service` shutdown request, a `gz topic` probe);
  - a server that is still going when the grace period ends is a survivor, and the verdict is taken BEFORE anything is signalled;
  - a server that stops within the grace period is not a finding;
  - an empty partition is refused (it would match every simulator on the machine).
"""
import json
import os
import signal
import subprocess
import sys
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "scripts"))
import launch_guard as lg  # noqa: E402

SERVER_ARGV0 = "gz sim -r -s -v 2 --physics-engine gz-physics-bullet-featherstone-plugin /ws/empty.sdf"


def decoy(argv0, partition, seconds=60):
    env = {k: v for k, v in os.environ.items() if k != "GZ_PARTITION"}
    if partition is not None:
        env["GZ_PARTITION"] = partition
    proc = subprocess.Popen(["bash", "-c", f'exec -a "{argv0}" sleep {seconds}'], env=env,
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:       # until bash has exec'd: the command line is bash's own before that
        try:
            if open(f"/proc/{proc.pid}/cmdline", "rb").read().startswith(argv0.split()[0].encode() + b" "):
                break
        except OSError:
            pass
        time.sleep(0.02)
    return proc


@pytest.fixture(autouse=True)
def evidence_dir(tmp_path, monkeypatch):
    """The helper's report lives in the evidence directory (P17): every test of this module gets its own, so none reads or leaves a file in the
    workspace's real one."""
    monkeypatch.setenv("OPENAMR_EVIDENCE_DIR", str(tmp_path / "evidence"))
    return tmp_path / "evidence"


@pytest.fixture
def decoys():
    started = []

    def make(*args, **kwargs):
        proc = decoy(*args, **kwargs)
        started.append(proc)
        return proc
    yield make
    for proc in started:
        if proc.poll() is None:
            os.kill(proc.pid, signal.SIGKILL)
        proc.wait()


def part(tag):
    return f"p16_{tag}_{os.getpid()}"


def test_the_server_of_this_partition_is_found_and_the_others_are_not(decoys):
    mine = decoys(SERVER_ARGV0, part("mine"))
    other = decoys(SERVER_ARGV0, part("other"))
    none = decoys(SERVER_ARGV0, None)
    found = lg.gz_sim_processes_of(part("mine"))
    assert [p["pid"] for p in found] == [mine.pid]
    assert found[0]["cmdline"].startswith("gz sim -r -s")
    assert other.pid not in [p["pid"] for p in lg.gz_sim_processes_of(part("mine"))]
    assert none.pid not in [p["pid"] for p in lg.gz_sim_processes_of(part("mine"))]


def test_a_partition_that_merely_starts_the_same_is_not_this_one(decoys):
    decoys(SERVER_ARGV0, part("run1") + "0")
    assert lg.gz_sim_processes_of(part("run1")) == []


@pytest.mark.parametrize("argv0", ["gz service -s /server_control --reqtype gz.msgs.ServerControl", "gz topic -e -t /world/empty/dynamic_pose/info",
                                   "python3 test_g02.py --about gz sim"])
def test_a_process_that_only_mentions_gz_is_not_a_simulator(decoys, argv0):
    decoys(argv0, part("mention"))
    assert lg.gz_sim_processes_of(part("mention")) == []


def test_the_ruby_wrapper_of_this_run_is_also_found(decoys):
    wrapper = decoys("ruby /opt/ros/jazzy/opt/gz_tools_vendor/bin/gz sim -r -s -v 2 /ws/empty.sdf", part("wrapper"))
    assert [p["pid"] for p in lg.gz_sim_processes_of(part("wrapper"))] == [wrapper.pid]


def test_an_empty_partition_is_refused():
    with pytest.raises(ValueError):
        lg.gz_sim_processes_of("")
    with pytest.raises(ValueError):
        lg.gz_server_shutdown_report("")


def test_a_survivor_is_judged_before_it_is_signalled_and_is_then_stopped(decoys):
    server = decoys(SERVER_ARGV0, part("survivor"))
    report = lg.gz_server_shutdown_report(part("survivor"), grace_s=1.0)
    assert [p["pid"] for p in report["survivors"]] == [server.pid]
    assert report["waited_s"] >= 1.0, "the grace period was not waited: a server told to stop would be called a survivor at once"
    assert [r["pid"] for r in report["reaped"]] == [server.pid] and report["reaped"][0]["signal"] in ("SIGTERM", "SIGKILL")
    server.wait(timeout=10)
    assert lg.gz_sim_processes_of(part("survivor")) == []
    assert "survived the launch's teardown" in lg.gz_survivor_message(report)


def test_with_reap_off_nothing_is_signalled(decoys):
    server = decoys(SERVER_ARGV0, part("noreap"))
    report = lg.gz_server_shutdown_report(part("noreap"), grace_s=0.3, reap=False)
    assert [p["pid"] for p in report["survivors"]] == [server.pid] and report["reaped"] == []
    assert server.poll() is None


def test_assert_fails_on_a_survivor_and_passes_when_there_is_none(decoys):
    server = decoys(SERVER_ARGV0, part("assert"))
    with pytest.raises(AssertionError, match="survived the launch's teardown"):
        lg.assert_no_gz_server_survives(part("assert"), grace_s=0.5)
    server.wait(timeout=10)
    assert lg.assert_no_gz_server_survives(part("assert"), grace_s=0.5)["survivors"] == []


def test_a_server_that_stops_within_the_grace_period_is_not_a_finding(decoys):
    server = decoys(SERVER_ARGV0, part("slow"), seconds=1.5)
    started = time.monotonic()
    report = lg.gz_server_shutdown_report(part("slow"), grace_s=8.0)
    assert report["survivors"] == [] and report["reaped"] == []
    assert time.monotonic() - started < 6.0
    server.wait(timeout=5)


def test_a_zombie_is_not_a_running_server(decoys, tmp_path):
    """A fake /proc with one entry in state Z: left out. (The real thing: a server that has exited and is waiting to be reaped.)"""
    pid_dir = tmp_path / "4242"
    pid_dir.mkdir()
    (pid_dir / "environ").write_bytes(b"GZ_PARTITION=zz\0")
    (pid_dir / "cmdline").write_bytes(b"gz sim -r -s\0")
    (pid_dir / "stat").write_text("4242 (gz sim) Z 1 4242 4242 0 -1 0")
    assert lg.gz_sim_processes_of("zz", proc_root=str(tmp_path)) == []
    (pid_dir / "stat").write_text("4242 (gz sim) S 1 4242 4242 0 -1 0")
    assert [p["pid"] for p in lg.gz_sim_processes_of("zz", proc_root=str(tmp_path))] == [4242]


# --- P17: the stop helper's report is part of the verdict ----------------------------------------------------------------------------------
def write_helper_report(partition, **fields):
    path = lg.gz_stop_report_path(partition)
    assert os.path.dirname(path) == os.environ["OPENAMR_EVIDENCE_DIR"], "the report is in the evidence directory, not a temp directory"
    with open(path, "w") as f:
        json.dump(fields, f)


@pytest.fixture
def helper_report():
    paths = []

    def make(partition, **fields):
        write_helper_report(partition, **fields)
        paths.append(lg.gz_stop_report_path(partition))
    yield make
    for path in paths:
        if os.path.exists(path):
            os.unlink(path)


def test_a_server_the_helper_had_to_force_and_that_is_gone_is_recorded_not_a_survivor(helper_report):
    """P18: forced and gone is a WARN (W04 in the A7 report), not a FAIL of the orphan check: nothing is left running."""
    helper_report(part("forced"), state="force_stopped", survivors=[424242], signals=["SIGTERM", "SIGKILL"], gone=True)
    report = lg.gz_server_shutdown_report(part("forced"), grace_s=0.2)
    assert report["survivors"] == [] and report["reaped"] == []
    assert report["forced"] == [{"pid": 424242, "signals": ["SIGTERM", "SIGKILL"], "gone": True}]
    assert not os.path.exists(lg.gz_stop_report_path(part("forced"))), "the report is consumed, so a later run of the same partition cannot inherit it"


def test_a_server_that_survives_its_own_force_stop_is_still_an_orphan_and_still_fails(decoys, helper_report):
    """The survivor check is not weakened: the helper says it forced the server, the server is STILL RUNNING when the check looks."""
    server = decoys(SERVER_ARGV0, part("forcedalive"))
    helper_report(part("forcedalive"), state="force_stopped", survivors=[server.pid], signals=["SIGTERM", "SIGKILL"], gone=False)
    report = lg.gz_server_shutdown_report(part("forcedalive"), grace_s=0.5)
    assert [p["pid"] for p in report["survivors"]] == [server.pid], "alive after the force-stop: a survivor"
    assert report["forced"][0]["pid"] == server.pid
    server.wait(timeout=10)
    helper_report(part("forcedalive2"), state="force_stopped", survivors=[1], signals=["SIGTERM"], gone=False)
    server2 = decoys(SERVER_ARGV0, part("forcedalive2"))
    with pytest.raises(AssertionError, match="survived the launch's teardown"):
        lg.assert_no_gz_server_survives(part("forcedalive2"), grace_s=0.5)
    server2.wait(timeout=10)


def test_the_check_waits_for_the_helper_to_act_before_it_judges(decoys, helper_report):
    """A server still running when the check starts, ended by the helper a moment later, is not a survivor: the check judges after the force-stop."""
    server = decoys(SERVER_ARGV0, part("late"), seconds=60)
    helper_report(part("late"), state="force_stopped", survivors=[server.pid], signals=["SIGTERM", "SIGKILL"], gone=True)
    import threading
    threading.Timer(1.0, lambda: os.kill(server.pid, signal.SIGKILL)).start()
    report = lg.gz_server_shutdown_report(part("late"), grace_s=8.0)
    assert report["survivors"] == [] and report["waited_s"] >= 0.8 and report["forced"]


def test_a_helper_that_stopped_the_server_on_request_is_not_a_finding(helper_report):
    helper_report(part("fine"), state="stopped", servers_at_request=[1], elapsed_s=1.7)
    report = lg.gz_server_shutdown_report(part("fine"), grace_s=0.2)
    assert report["survivors"] == [] and report["stop_helper"]["state"] == "stopped"


def test_no_helper_report_is_not_a_finding_and_is_recorded_as_none():
    report = lg.gz_server_shutdown_report(part("nohelper"), grace_s=0.2)
    assert report["survivors"] == [] and report["stop_helper"] is None


def test_a_helper_still_watching_is_waited_for_and_its_state_is_not_called_a_pass(helper_report, monkeypatch):
    helper_report(part("watching"), state="started")
    monkeypatch.setattr(lg, "HELPER_REPORT_WAIT_S", 0.5)
    started = time.monotonic()
    report = lg.gz_server_shutdown_report(part("watching"), grace_s=0.1)
    assert time.monotonic() - started >= 0.5, "a report in state `started` must be waited for"
    assert report["stop_helper"]["state"] == "started" and report["survivors"] == []
