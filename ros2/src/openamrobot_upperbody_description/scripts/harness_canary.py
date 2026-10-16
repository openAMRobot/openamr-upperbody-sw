#!/usr/bin/env python3
"""Plant deliberately failing tests ("canaries") in the packages that contribute checks, so that the evidence harness can be shown to report them.

    harness_canary.py inject  [--modes assert,collect,timeout,signal,nostart] [--kinds pytest,launch] PACKAGE_DIR...
    harness_canary.py remove  PACKAGE_DIR...

A harness that only ever sees passing tests has never been tested. This writes, into each package's `test/` directory and between
`# BEGIN CANARY` / `# END CANARY` markers in its CMakeLists.txt, one failing test per failure mode and per kind of suite:

  assert    the test body fails an assertion
  collect   the module raises while it is being imported (an error during collection)
  timeout   the test outlives ctest's TIMEOUT (a few seconds here; the process is killed by ctest, so it writes no result file)
  signal    pytest: the test process kills itself with SIGSEGV. launch: a process of the launch dies of SIGSEGV during the test, which
            itself passes (the shape of `move_group` / `moveit_py` crashing at teardown, which is the case most likely to be swallowed)
  unlisted  pytest only: an assertion failure in a test whose name carries a check-ID prefix that no report lists (`test_q99_...`)
  nostart   launch only: a process of the launch description cannot be started (the executable does not exist), and the test passes

and `kind` is `pytest` (ament_add_pytest_test), `launch` (add_launch_test, launch_testing, a module with no process check, as launch tests were written
before P12A) or `launchg` (the same with the post-shutdown process check of launch_guard.py, signal and nostart only: the way a launch test is
written now). Every canary is named `test_canary_<kind>_<mode>`
so a report can be searched for it. They are planted for an experiment and removed after it: `remove` deletes exactly what `inject`
wrote, and a canary left in a tree is a deliberately failing test left in a tree, which `git status` shows.
"""
import argparse
import os
import re
import sys

MODES = ("assert", "collect", "timeout", "signal", "nostart", "unlisted")
KINDS = ("pytest", "launch", "launchg")
BEGIN, END = "# BEGIN CANARY", "# END CANARY"
CTEST_TIMEOUT = 12            # seconds: far below what the timeout canary sleeps, far above what the others need

PYTEST = '''"""CANARY (harness_canary.py): a deliberately failing test, mode {mode}. Not part of the package; planted for an experiment."""
import os
import signal
import time

MODE = "{mode}"
if MODE == "collect":
    raise ImportError("canary: a deliberate error during collection")


def {fname}():
    if MODE in ("assert", "unlisted"):
        assert False, "canary: a deliberate assertion failure"
    if MODE == "timeout":
        time.sleep(300)
    if MODE == "signal":
        os.kill(os.getpid(), signal.SIGSEGV)
        time.sleep(30)
'''

LAUNCH = '''"""CANARY (harness_canary.py): a deliberately failing launch test, mode {mode}. Not part of the package; planted for an experiment."""
import time
import unittest

import launch
import launch.actions
import launch_testing.actions
import pytest

MODE = "{mode}"
if MODE == "collect":
    raise ImportError("canary: a deliberate error during collection")


@pytest.mark.launch_test
def generate_test_description():
    # A healthy process that outlives the test keeps the launch alive, as move_group does in the real launch tests: launch_testing stops the
    # test with "Processes under test stopped before tests completed" when EVERY process has died, which would flag the doomed one by accident.
    actions = [launch.actions.ExecuteProcess(cmd=["sleep", "60"], output="screen")]
    if MODE == "signal":
        actions.append(launch.actions.ExecuteProcess(
            cmd=["python3", "-c", "import os, signal, time; time.sleep(1); os.kill(os.getpid(), signal.SIGSEGV)"], output="screen"))
    if MODE == "nostart":
        actions.append(launch.actions.ExecuteProcess(cmd=["/nonexistent/canary_process_that_cannot_start"], output="screen"))
    actions.append(launch_testing.actions.ReadyToTest())
    return launch.LaunchDescription(actions)


class TestCanary(unittest.TestCase):
    def {fname}(self):
        if MODE == "assert":
            self.fail("canary: a deliberate assertion failure")
        if MODE == "timeout":
            time.sleep(300)
        time.sleep(4)         # signal / nostart: give the doomed process time to die, then pass: the test itself is fine
'''


GUARD = '''

import os
import sys

from ament_index_python.packages import get_package_prefix

sys.path.insert(0, os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
from launch_guard import SIGINT, assert_launch_healthy, assert_no_process_died  # noqa: E402

EXPECTED_PROCESSES = ["sleep", "python3" if MODE == "signal" else "canary_process_that_cannot_start"]


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_canary_processes_started_and_exited_cleanly(self, proc_info):
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed={{"sleep": [SIGINT]}})
'''


def modes_for(kind, modes):
    if kind == "launchg":
        return [m for m in modes if m in ("signal", "nostart")]
    return [m for m in modes if not (kind == "pytest" and m == "nostart") and not (kind == "launch" and m == "unlisted")]


def inject(package_dir, modes, kinds):
    test_dir = os.path.join(package_dir, "test")
    os.makedirs(test_dir, exist_ok=True)
    lines = [BEGIN, "if(BUILD_TESTING)", "  find_package(ament_cmake_pytest REQUIRED)", "  find_package(launch_testing_ament_cmake REQUIRED)"]
    for kind in kinds:
        for mode in modes_for(kind, modes):
            name = f"test_canary_{kind}_{mode}.py"
            with open(os.path.join(test_dir, name), "w") as f:
                fname = ("test_q99_canary_pytest_unlisted" if mode == "unlisted" else f"test_canary_{kind}_{mode}")
                f.write({"pytest": PYTEST, "launch": LAUNCH, "launchg": LAUNCH + GUARD}[kind].format(mode=mode, fname=fname))
            target = f"canary_{kind}_{mode}"
            lines.append(f"  ament_add_pytest_test({target} test/{name} TIMEOUT {CTEST_TIMEOUT})" if kind == "pytest"
                         else f"  add_launch_test(test/{name} TARGET {target} TIMEOUT {CTEST_TIMEOUT})")
    lines += ["endif()", END]
    path = os.path.join(package_dir, "CMakeLists.txt")
    with open(path) as f:
        text = f.read()
    if BEGIN in text:
        raise SystemExit(f"{path}: canaries are already planted")
    head, tail = text.rsplit("ament_package()", 1)
    with open(path, "w") as f:
        f.write(head + "\n".join(lines) + "\n\nament_package()" + tail)


def remove(package_dir):
    path = os.path.join(package_dir, "CMakeLists.txt")
    with open(path) as f:
        text = f.read()
    text = re.sub(re.escape(BEGIN) + r".*?" + re.escape(END) + r"\n\n", "", text, flags=re.S)
    with open(path, "w") as f:
        f.write(text)
    test_dir = os.path.join(package_dir, "test")
    for name in sorted(os.listdir(test_dir)) if os.path.isdir(test_dir) else []:
        if name.startswith("test_canary_") and name.endswith(".py"):
            os.remove(os.path.join(test_dir, name))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("action", choices=("inject", "remove"))
    parser.add_argument("packages", nargs="+")
    parser.add_argument("--modes", default=",".join(MODES))
    parser.add_argument("--kinds", default=",".join(KINDS))
    args = parser.parse_args(argv)
    modes, kinds = args.modes.split(","), args.kinds.split(",")
    bad = [m for m in modes if m not in MODES] + [k for k in kinds if k not in KINDS]
    if bad:
        parser.error(f"unknown: {bad}")
    for package in args.packages:
        if args.action == "inject":
            inject(package, modes, kinds)
        else:
            remove(package)
        print(f"{args.action}: {package}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
