# SPDX-License-Identifier: MIT
"""How sim.launch.py starts the controllers, on the launch description itself. No Gazebo, no ROS graph.

What is pinned, against the failure seen on 16 Oct (a 4-core machine with CPU-only rendering): the joint_state_broadcaster's first switch
timed out after the spawner's default 5 s, its spawner exited 1, and the arm and gripper spawners were started anyway (an OnProcessExit with
a fixed action list), so the stack ran with four of five controllers active.
  - the broadcaster's spawner is given --switch-timeout 30, and a service-call timeout longer than that (with the default 10 s the call was
    abandoned and retried and the spawner exited 1 although the switch completed);
  - the arm and gripper spawners are started only when that spawner exits 0, and not at all when it exits non-zero.
"""
import importlib.util
import os
import types

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_LAUNCH = os.path.join(HERE, os.pardir, "launch", "sim.launch.py")
ARM_AND_GRIPPER = ["left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller"]


@pytest.fixture(scope="module")
def sim():
    spec = importlib.util.spec_from_file_location("sim_launch_spawners_under_test", SIM_LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def spawner_arguments(node):
    """The literal arguments of a spawner Node, after the executable (every argument is a plain string here)."""
    return ["".join(getattr(part, "text", "") for part in argument) for argument in node.cmd[1:]]


def controller_of(node):
    return spawner_arguments(node)[0]


def is_spawner(entity):
    return type(entity).__name__ == "Node" and entity.node_executable == "spawner"


def broadcaster_and_chain(sim):
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnProcessExit
    description = sim.generate_launch_description()
    broadcasters = [e for e in description.entities if is_spawner(e) and controller_of(e) == "joint_state_broadcaster"]
    handlers = [e.event_handler for e in description.entities
                if isinstance(e, RegisterEventHandler) and isinstance(e.event_handler, OnProcessExit)]
    assert len(broadcasters) == 1, f"expected one joint_state_broadcaster spawner, found {len(broadcasters)}"
    assert len(handlers) == 1, f"expected one OnProcessExit (broadcaster -> the rest), found {len(handlers)}"
    return broadcasters[0], handlers[0]


def started_after_exit(sim, returncode):
    """The spawners the launch would start when the broadcaster's spawner exits with `returncode`."""
    from launch import LaunchContext
    from launch.events.process import ProcessExited
    broadcaster, handler = broadcaster_and_chain(sim)
    event = ProcessExited(action=broadcaster, name="spawner-6", cmd=["spawner"], cwd=None, env=None, pid=1, returncode=returncode)
    assert handler.matches(event), "the OnProcessExit does not target the broadcaster's spawner"
    return [a for a in (handler.handle(event, LaunchContext()) or []) if is_spawner(a)]


def test_the_broadcaster_spawner_has_a_30_s_switch_timeout(sim):
    broadcaster, _ = broadcaster_and_chain(sim)
    arguments = spawner_arguments(broadcaster)
    assert arguments[0] == "joint_state_broadcaster"
    assert "--switch-timeout" in arguments, f"no --switch-timeout on the broadcaster spawner: {arguments}"
    assert float(arguments[arguments.index("--switch-timeout") + 1]) == 30.0
    # the spawner's service-call timeout (default 10 s) must not end the wait first: it did, on 16 Oct, and the spawner exited 1
    assert "--service-call-timeout" in arguments, f"no --service-call-timeout on the broadcaster spawner: {arguments}"
    assert float(arguments[arguments.index("--service-call-timeout") + 1]) > 30.0


def test_the_arm_and_gripper_spawners_start_after_a_broadcaster_that_exited_0(sim):
    assert sorted(controller_of(a) for a in started_after_exit(sim, 0)) == sorted(ARM_AND_GRIPPER)


@pytest.mark.parametrize("returncode", [1, -2])
def test_no_arm_or_gripper_spawner_starts_after_a_broadcaster_that_failed(sim, returncode):
    assert started_after_exit(sim, returncode) == [], "spawners started after the broadcaster failed"
