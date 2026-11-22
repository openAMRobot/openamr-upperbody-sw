# SPDX-License-Identifier: MIT
"""The /clock fail-fast and the diagnostic capture of build1.launch.py (gate owner decision on #77, 22 Nov 2026).

- With driver_clock:=sim the real arm driver (ros2_control_node and its joint_state_broadcaster spawner) starts only
  after wait_for_clock.py has seen the first ROS /clock message; when none arrives within clock_gate.BOUND_S of the
  launch start the launch ends with the named reason "no /clock".
- BUILD1_CLOCK_CAPTURE=1: GZ_VERBOSE=1 on gazebo and build1_control_bridge, UDP counters before and 15 s after the
  launch start, and the gz topic / ros2 topic info probes when there is no /clock at 10 s. Evidence only.
Pure Python: the launch file is read, not run.
"""
import ast
import json
import os
import subprocess
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.dirname(HERE)
REPOSITORY = os.path.abspath(os.path.join(PACKAGE_DIR, "..", "..", ".."))
sys.path.insert(0, PACKAGE_DIR)
from openamrobot_build1_scene import clock_gate  # noqa: E402

LAUNCH = os.path.join(PACKAGE_DIR, "launch", "build1.launch.py")
WORKFLOW = os.path.join(REPOSITORY, ".github", "workflows", "baseline1-acceptance.yml")


def _launch_tree():
    with open(LAUNCH, encoding="utf-8") as f:
        return ast.parse(f.read())


def _function(tree, name):
    return next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == name)


def _calls(node, name):
    return [n for n in ast.walk(node) if isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id == name]


def _names(node):
    return {n.id for n in ast.walk(node) if isinstance(n, ast.Name)}


# ---------------------------------------------------------------- the gate's own rule
class Wall:
    def __init__(self, t):
        self.t = t

    def __call__(self):
        return self.t


def test_the_gate_waits_until_the_bound_and_then_names_no_clock():
    wall = Wall(1000.0)
    gate = clock_gate.Gate(launch_start=985.0, now=wall)        # started 15 s after the launch, as after sim_rest
    assert gate.poll() is None
    wall.t = 985.0 + clock_gate.BOUND_S - 0.1
    assert gate.poll() is None
    wall.t = 985.0 + clock_gate.BOUND_S
    assert gate.poll() == clock_gate.EXIT_NO_CLOCK
    record = gate.record()
    assert clock_gate.fired(record) and record["result"] == "no /clock" and record["first_clock_after_launch_s"] is None
    assert "BLOCKED (sim infrastructure): no /clock" in record["reason"]


def test_the_first_clock_message_opens_the_gate_and_is_recorded():
    wall = Wall(1015.0)
    gate = clock_gate.Gate(launch_start=1000.0, now=wall)
    wall.t = 1016.5
    gate.on_clock(5.25)
    wall.t = 1100.0                     # later messages or the bound passing change nothing
    gate.on_clock(80.0)
    assert gate.poll() == 0
    record = gate.record()
    assert (record["result"], record["fired"], record["first_clock_after_launch_s"], record["first_clock_sim_s"]) == (
        "clock", False, 16.5, 5.25)
    assert not clock_gate.fired(record)


def test_a_gate_started_after_the_bound_still_waits_its_minimum():
    wall = Wall(1000.0 + clock_gate.BOUND_S + 20.0)
    gate = clock_gate.Gate(launch_start=1000.0, now=wall)
    assert gate.poll() is None
    wall.t += clock_gate.MIN_WAIT_S
    assert gate.poll() == clock_gate.EXIT_NO_CLOCK


def test_the_bound_is_the_one_stated_in_the_pr():
    """Today's eight passing legs: first ROS /clock 7.0 to 11.7 s after the launch start; 60 s is about five times that."""
    assert clock_gate.BOUND_S == 60.0 and clock_gate.EXIT_NO_CLOCK not in (0, 1)
    assert (clock_gate.PROBE_AT_S, clock_gate.NETSTAT_AT_S) == (10.0, 15.0)


# ---------------------------------------------------------------- the launch: the wait in front of the real driver
def test_the_real_driver_starts_only_through_the_clock_gate_with_sim_time():
    setup = _function(_launch_tree(), "launch_setup")
    # real_control is handed to the launch only as real_driver, and with sim time real_driver is the gate alone
    for call in _calls(setup, "then"):
        assert "real_control" not in _names(call), ast.unparse(call)
    assignments = [n for n in ast.walk(setup) if isinstance(n, ast.Assign)
                   and [ast.unparse(t) for t in n.targets] == ["real_driver"]]
    assert [ast.unparse(a.value) for a in assignments] == ["[real_control, real_broadcaster]", "[gate]"]
    sim_time = next(n for n in ast.walk(setup) if isinstance(n, ast.If) and ast.unparse(n.test) == "driver_sim_time"
                    and any("wait_for_clock.py" in ast.unparse(s) for s in n.body))
    body = "\n".join(ast.unparse(s) for s in sim_time.body)
    assert "after_clock(gate, real_driver)" in body and "real_driver = [gate]" in body
    assert "'--bound', str(clock_gate.BOUND_S)" in body and "'--launch-start', repr(launch_start)" in body
    starts = sorted(ast.unparse(c) for c in _calls(setup, "then") if "real_driver" in _names(c))
    assert starts == ["then(sim_broadcaster, real_driver, \"the simulator's joint_state_broadcaster spawner\")",
                      "then(sim_rest, real_driver, 'the arm emulator spawner')"]


class _Stub:
    def __init__(self, *args, **kwargs):
        self.args, self.kwargs = args, kwargs


class _Event:
    def __init__(self, returncode):
        self.returncode = returncode


def _after_clock():
    """after_clock() of the launch file with stand-ins for the launch actions."""
    function = _function(_launch_tree(), "after_clock")
    namespace = {"clock_gate": clock_gate, "LogInfo": _Stub, "Shutdown": _Stub,
                 "RegisterEventHandler": lambda handler: handler,
                 "OnProcessExit": lambda target_action, on_exit: (target_action, on_exit)}
    exec(compile(ast.Module(body=[function], type_ignores=[]), LAUNCH, "exec"), namespace)  # noqa: S102
    return namespace["after_clock"]


def test_no_clock_ends_the_launch_with_the_named_reason_and_a_clock_starts_the_driver():
    target, on_exit = _after_clock()("gate", ["real_control", "real_broadcaster"])
    assert target == "gate"
    assert on_exit(_Event(0), None) == ["real_control", "real_broadcaster"]
    stopped = on_exit(_Event(clock_gate.EXIT_NO_CLOCK), None)
    assert [a.kwargs.get("reason") for a in stopped if "reason" in a.kwargs] == ["no /clock"]
    assert "BLOCKED (sim infrastructure): no /clock" in stopped[0].kwargs["msg"]
    failed = on_exit(_Event(1), None)
    assert [a.kwargs.get("reason") for a in failed if "reason" in a.kwargs] == ["clock gate failed"]


def test_the_gate_script_exits_with_the_gate_status_and_is_installed():
    with open(os.path.join(PACKAGE_DIR, "scripts", "wait_for_clock.py"), encoding="utf-8") as f:
        script = f.read()
    assert "default=clock_gate.BOUND_S" in script and "clock_gate.Gate(" in script and "write_record" in script
    assert "return 1 if status is None else status" in script
    with open(os.path.join(PACKAGE_DIR, "CMakeLists.txt"), encoding="utf-8") as f:
        cmake = f.read()
    assert "scripts/wait_for_clock.py" in cmake and "scripts/clock_capture.py" in cmake


# ---------------------------------------------------------------- the capture: evidence only
def test_the_capture_sets_gz_verbose_on_gazebo_and_the_control_bridge_only_when_asked():
    setup = ast.unparse(_function(_launch_tree(), "launch_setup"))
    assert "capture = clock_gate.capture_enabled(os.environ)" in setup
    assert "gz_verbose = {'GZ_VERBOSE': '1'} if capture else {}" in setup
    assert ("GroupAction([SetEnvironmentVariable('GZ_VERBOSE', '1'), simulator], scoped=True) if capture else simulator"
            in setup)
    bridge = next(c for c in ast.walk(_function(_launch_tree(), "launch_setup")) if isinstance(c, ast.Call)
                  and "name='build1_control_bridge'" in ast.unparse(c) and ast.unparse(c.func) == "Node")
    assert "additional_env=gz_verbose" in ast.unparse(bridge)
    assert "executable='clock_capture.py'" in setup
    assert clock_gate.capture_enabled({"BUILD1_CLOCK_CAPTURE": "1"}) and not clock_gate.capture_enabled({})


def test_the_capture_probes_are_bounded_and_the_ones_decided():
    with open(os.path.join(PACKAGE_DIR, "scripts", "clock_capture.py"), encoding="utf-8") as f:
        tree = ast.parse(f.read())
    probes = next(n for n in tree.body if isinstance(n, ast.Assign) and ast.unparse(n.targets[0]) == "PROBES")
    commands = [" ".join(ast.literal_eval(item.elts[1])) for item in probes.value.elts]
    assert commands == ["timeout 10 gz topic -e -t /clock -n 1", "timeout 10 gz topic -i -t /clock",
                        "timeout 30 ros2 topic info /clock -v --no-daemon"]
    text = ast.unparse(tree)
    assert "clock_gate.PROBE_AT_S" in text and "clock_gate.NETSTAT_AT_S" in text and "'netstat', '-su'" in text
    assert "return 0" in text   # evidence only: never a failure of the leg


def test_the_workflow_turns_the_capture_on_and_records_the_counters_before_the_launch():
    with open(WORKFLOW, encoding="utf-8") as f:
        workflow = yaml.safe_load(f)
    assert workflow["env"]["BUILD1_CLOCK_CAPTURE"] == "1"
    mission = next(s for s in workflow["jobs"]["baseline1"]["steps"] if s.get("id") == "mission")["run"]
    before = mission.index("netstat_su_before.txt")
    assert before < mission.index("colcon test ")
    upload = next(s for s in workflow["jobs"]["baseline1"]["steps"] if "upload-artifact" in s.get("uses", ""))
    assert "ws/build1_logs/" in upload["with"]["path"] and "ws/build1_logs_blocked_try*/" in upload["with"]["path"]


# ---------------------------------------------------------------- the rerun inside the leg job, and the report
def test_the_mission_step_reruns_only_a_try_the_blocked_rule_accepts():
    with open(WORKFLOW, encoding="utf-8") as f:
        workflow = yaml.safe_load(f)
    assert int(workflow["env"]["BASELINE1_MISSION_TRIES"]) >= 2
    mission = next(s for s in workflow["jobs"]["baseline1"]["steps"] if s.get("id") == "mission")["run"]
    rule = mission.index('baseline1_strict_verdict.py" --blocked-try "$BUILD1_LOG_DIR"')
    assert mission.index('mv "$BUILD1_LOG_DIR" "$moved"') > rule and 'moved="$PWD/build1_logs_blocked_try${try}"' in mission
    verdict = workflow["jobs"]["verdict"]
    assert verdict["permissions"]["actions"] == "read"
    run = next(s for s in verdict["steps"] if s.get("name") == "Verdict")["run"]
    assert "--jobs jobs.jsonl" in run
    result = next(s for s in verdict["steps"] if s.get("name") == "Result")["run"]
    assert "'BLOCKED'" in result


def test_the_phase_table_names_no_clock_as_blocked_when_the_fail_fast_fired(tmp_path):
    wall = Wall(1000.0)
    gate = clock_gate.Gate(launch_start=1000.0, now=wall)
    wall.t += clock_gate.BOUND_S
    clock_gate.write_record(str(tmp_path), gate.record())
    done = subprocess.run([sys.executable, os.path.join(PACKAGE_DIR, "scripts", "baseline1_report.py"), str(tmp_path)],
                          capture_output=True, text=True)
    assert done.returncode == 1
    spawn = next(line for line in done.stdout.splitlines() if line.startswith("| 1 | spawn"))
    assert "| BLOCKED | ENV/INFRA |" in spawn and "no /clock" in spawn
    assert "/clock fail-fast: " in done.stdout and json.loads(done.stdout.split("/clock fail-fast: ")[1].splitlines()[0])["fired"]
