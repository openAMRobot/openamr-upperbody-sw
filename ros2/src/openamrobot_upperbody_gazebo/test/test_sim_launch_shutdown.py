"""How sim.launch.py stops the simulator (P17 task 1), on decoy processes and on the launch description itself. No Gazebo, no ROS graph.

What is pinned, each against a failure that was observed:
  - the stop is requested ONCE per launch, however many Shutdown events arrive (launch emits a second one when the shell around `gz sim` dies,
    `on_exit_shutdown`; the old handler re-executed the same ExecuteProcess, which raised "executed more than once" and abandoned the rest of
    the shutdown on every run);
  - a server that goes after the request is reported "stopped" and nothing is signalled;
  - a server that accepts the request and does not go, and ignores SIGTERM (the orphan of P16/P17), is stopped by SIGKILL inside the budget and
    the report says "force_stopped" BEFORE the signal is sent, so launch_guard can fail the run that needed it;
  - the helper's report file is the one launch_guard reads.
"""
import importlib.util
import json
import os
import signal
import subprocess
import sys
import time

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, os.pardir, "launch", "sim.launch.py")


@pytest.fixture(scope="module")
def sim():
    spec = importlib.util.spec_from_file_location("sim_launch_under_test", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def server_tree(ignore_term):
    """A shell with a child whose command line is `gz sim -r -s ...` (what the forked server's looks like), as ros_gz_sim's `/bin/sh -c ruby ...`
    wraps the real one. `ignore_term` makes the child ignore SIGTERM, which is what the hung server does (it only dies of SIGKILL)."""
    trap = 'trap "" TERM; ' if ignore_term else ""
    shell = subprocess.Popen(["bash", "-c", f'{trap}(exec -a "gz sim -r -s -v 2 --physics-engine x /w/empty.sdf" sleep 120) & wait'],
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        for name in os.listdir("/proc"):
            if name.isdigit():
                try:
                    if open(f"/proc/{name}/cmdline", "rb").read().startswith(b"gz sim -r -s"):
                        stat = open(f"/proc/{name}/stat").read().rpartition(")")[2].split()
                        if int(stat[1]) == shell.pid:
                            return shell, int(name)
                except OSError:
                    pass
        time.sleep(0.02)
    shell.kill()
    raise AssertionError("the decoy server did not start")


@pytest.fixture
def trees():
    made = []

    def make(ignore_term=False):
        shell, server = server_tree(ignore_term)
        made.append((shell, server))
        return shell, server
    yield make
    for shell, server in made:
        for pid in (server, shell.pid):
            try:
                os.kill(pid, signal.SIGKILL)
            except OSError:
                pass
        shell.wait()


def alive(pid):
    try:
        return open(f"/proc/{pid}/stat").read().rpartition(")")[2].split()[0] not in ("Z", "X")
    except OSError:
        return False


def nothing():
    return True, "no-op"


def test_a_server_that_goes_on_the_interrupt_is_stopped_and_nothing_else_is_done(sim, trees, tmp_path):
    shell, server = trees()
    path = str(tmp_path / "report.json")
    seen = {}

    def interrupt():
        seen["before_the_interrupt"] = json.load(open(path))       # the report exists while the helper is still watching
        os.kill(server, signal.SIGINT)                             # the server honours it
        return True, "SIGINT"

    def request():
        raise AssertionError("the service request is the second step: not needed when the interrupt worked")

    report = sim.stop_gz_server(shell.pid, interrupt=interrupt, request=request, retry_after_s=2.0, grace_s=5.0, report_path=path)
    assert seen["before_the_interrupt"]["state"] == "started"
    assert report["state"] == "stopped" and "signals" not in report and report["retried_with_service"] is False
    assert report["servers_at_request"] == [server]
    assert json.load(open(path))["state"] == "stopped"


def test_a_server_that_ignores_the_interrupt_is_asked_through_the_service_and_that_is_recorded(sim, trees, tmp_path):
    shell, server = trees()
    calls = []

    def request():
        calls.append("service")
        os.kill(server, signal.SIGINT)
        return True, "data: true"

    report = sim.stop_gz_server(shell.pid, interrupt=nothing, request=request, retry_after_s=0.3, grace_s=5.0, report_path=str(tmp_path / "r.json"))
    assert calls == ["service"]
    assert report["state"] == "stopped" and report["retried_with_service"] is True and "signals" not in report


def test_a_server_that_ignores_both_and_sigterm_is_force_stopped_and_the_report_says_so_first(sim, trees, tmp_path, monkeypatch):
    shell, server = trees(ignore_term=True)
    path = str(tmp_path / "report.json")
    order = []
    real_kill = os.kill

    def kill_spy(pid, sig):
        if pid == server:
            order.append((sig, json.load(open(path))["state"]))   # what the file said at the moment of each signal
        return real_kill(pid, sig)
    monkeypatch.setattr(sim.os, "kill", kill_spy)

    started = time.monotonic()
    report = sim.stop_gz_server(shell.pid, interrupt=nothing, request=nothing, retry_after_s=0.2, grace_s=0.6, term_wait_s=0.4, report_path=path)
    assert time.monotonic() - started < 5.0
    assert report["state"] == "force_stopped" and report["survivors"] == [server]
    assert report["signals"] == ["SIGTERM", "SIGKILL"], "SIGTERM is ignored by the hung server: only SIGKILL ends it"
    assert report["gone"] and not alive(server)
    assert order and all(state == "force_stopped" for _, state in order), "the file must say force_stopped before the first SIGTERM/SIGKILL"
    assert json.load(open(path))["state"] == "force_stopped"


def test_a_server_that_outlives_its_shell_is_still_followed_and_forced(sim, trees, tmp_path):
    """launch SIGTERMs the shell at 5 s: the server is then reparented and is no longer below it. The first version of the helper walked the tree,
    called that 'gone' and reported `stopped` over a server that was still running (seen in P17: the driver found it 40 s later)."""
    shell, server = trees(ignore_term=True)

    def interrupt():
        os.kill(shell.pid, signal.SIGKILL)         # the shell goes first, as launch's SIGTERM does
        shell.wait()
        assert alive(server), "the decoy server must outlive its shell for this test to mean anything"
        return True, "the shell is gone, the server is not"

    report = sim.stop_gz_server(shell.pid, interrupt=interrupt, request=nothing, retry_after_s=0.2, grace_s=0.6, term_wait_s=0.4,
                                report_path=str(tmp_path / "r.json"))
    assert report["state"] == "force_stopped", report
    assert report["survivors"] == [server] and report["gone"] and not alive(server)


def test_a_tree_with_no_server_is_reported_as_such(sim, tmp_path):
    report = sim.stop_gz_server(None, interrupt=nothing, request=nothing, grace_s=0.2, report_path=str(tmp_path / "r.json"))
    assert report["state"] == "no_server"


def test_the_default_interrupt_signals_only_gz_sim_processes_below_the_shell(sim, trees):
    shell, server = trees()
    bystander = subprocess.Popen(["sleep", "60"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        sent, text = sim.request_server_stop_by_signal(shell.pid)
        assert sent and str(server) in text
        deadline = time.monotonic() + 3
        while alive(server) and time.monotonic() < deadline:
            time.sleep(0.05)
        assert not alive(server)
        assert bystander.poll() is None, "a process that is not a `gz sim` below the shell must not be signalled"
    finally:
        bystander.kill()
        bystander.wait()


def test_the_shell_itself_is_never_signalled_by_the_helper(sim, trees, tmp_path, monkeypatch):
    """The shell is launch's own to end; the helper touches only the `gz sim` processes below it."""
    shell, server = trees(ignore_term=True)
    signalled = []
    real_kill = os.kill

    def kill_spy(pid, sig):
        signalled.append(pid)
        return real_kill(pid, sig)
    monkeypatch.setattr(sim.os, "kill", kill_spy)
    report = sim.stop_gz_server(shell.pid, interrupt=nothing, request=nothing, retry_after_s=0.1, grace_s=0.4, term_wait_s=0.3,
                                report_path=str(tmp_path / "r.json"))
    assert report["state"] == "force_stopped"
    assert signalled and set(signalled) == {server}, f"signalled {set(signalled)}, the shell is {shell.pid}"


def test_the_shutdown_handler_asks_once_however_many_shutdowns_arrive(sim, monkeypatch):
    """Both Shutdown events of a launch (the first request to stop, then the shell's death) go through the same handler object."""
    from launch import LaunchContext
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    from launch.events import Shutdown
    description = sim.generate_launch_description()
    handlers = [e.event_handler for e in description.entities if isinstance(e, RegisterEventHandler) and isinstance(e.event_handler, OnShutdown)]
    assert len(handlers) == 1, "exactly one OnShutdown handler: the stop"
    spawned = []

    class Popen:
        def __init__(self, cmd, **kwargs):
            spawned.append((cmd, kwargs))

    monkeypatch.setattr(sim.subprocess, "Popen", Popen)
    context = LaunchContext()
    for _ in range(3):
        assert handlers[0].handle(Shutdown(reason="test"), context) is None
    assert len(spawned) == 1, f"the stop helper was started {len(spawned)} times"
    cmd, kwargs = spawned[0]
    assert cmd[1].endswith("sim.launch.py") and cmd[2] == "--stop-gz-server"
    assert kwargs.get("start_new_session") is True, "detached: it must outlive a launch that is already shutting down"


def test_the_helper_reports_to_the_file_launch_guard_reads(sim, tmp_path, monkeypatch):
    monkeypatch.setenv("OPENAMR_EVIDENCE_DIR", str(tmp_path))
    description = os.path.join(HERE, os.pardir, os.pardir, "openamrobot_upperbody_description", "scripts")
    sys.path.insert(0, description)
    try:
        import launch_guard
    finally:
        sys.path.remove(description)
    ours, theirs = sim.gz_stop_report_path("g02_1234"), launch_guard.gz_stop_report_path("g02_1234")
    assert ours is not None and ours == theirs == str(tmp_path / "gz_stop_g02_1234.json")


def test_run_as_a_script_the_launch_file_is_the_helper_and_imports_nothing_of_ros(tmp_path):
    """`python3 sim.launch.py --stop-gz-server 0 REPORT` must start in well under a second and exit 0 with no ROS environment at all: that is
    what makes the stop go out before the shutdown has done anything else. With no pid there is no server: the report says so."""
    env = {"PATH": os.environ["PATH"]}
    report = tmp_path / "gz_stop_helper.json"
    started = time.monotonic()
    done = subprocess.run([sys.executable, SIM_LAUNCH, "--stop-gz-server", "0", str(report)], env=env, capture_output=True, text=True, timeout=30)
    assert done.returncode == 0, done.stderr
    assert time.monotonic() - started < 10
    assert json.load(open(report))["state"] == "no_server"


# --- the sim test support (P17): the one G02 failure in 36 live runs was a helper parsing two messages as one -------------------------------
def test_the_first_json_message_is_taken_when_gz_topic_prints_two():
    sys.path.insert(0, HERE)
    import sim_test_support as support
    one = '{"pose": [{"name": "a"}]}'
    assert support.first_json_message(one) == {"pose": [{"name": "a"}]}
    assert support.first_json_message(one + "\n" + '{"pose": [{"name": "b"}]}\n') == {"pose": [{"name": "a"}]}, "json.loads raised 'Extra data' here"
    with pytest.raises(ValueError):
        support.first_json_message("  \n")
