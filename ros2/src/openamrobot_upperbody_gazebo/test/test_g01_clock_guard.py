# SPDX-License-Identifier: MIT
"""G01 cannot hang on /clock, discard observation time, or hide bad samples."""
import importlib.util
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from sim_clock_guard import spin_sim_seconds  # noqa: E402


def test_frozen_clock_fails_without_waiting_for_the_outer_test_timeout():
    # Isolate the wait: reverting the guard must fail this test, not hang pytest.
    script = """
import time
from sim_clock_guard import spin_sim_seconds
try:
    spin_sim_seconds(0.5, sim_now=lambda: 1.0, spin_once=lambda: time.sleep(0.001),
                     wall_deadline=time.monotonic()+0.5, frozen_after_s=0.03)
except AssertionError as error:
    assert 'clock frozen' in str(error), str(error)
    assert 'advanced=0.000000' in str(error), str(error)
else:
    raise AssertionError('a frozen clock passed')
"""
    result = subprocess.run([sys.executable, "-c", script], cwd=HERE, capture_output=True, text=True, timeout=2)
    assert result.returncode == 0, result.stderr


def test_advancing_but_too_slow_clock_cannot_extend_the_wall_deadline():
    clock = [0.0, 10.0]

    def spin():
        clock[0] += 0.1
        clock[1] += 0.001

    with pytest.raises(AssertionError, match="wall-clock budget.*RTF="):
        spin_sim_seconds(3.0, sim_now=lambda: clock[1], spin_once=spin,
                         wall_now=lambda: clock[0], wall_deadline=1.0)
    assert clock[0] <= 1.1
    assert clock[1] < 10.02


def test_complete_simulation_duration_is_required_even_on_a_slow_clock():
    clock = [0.0, 1.0]

    def spin():
        clock[0] += 0.25
        clock[1] += 0.125

    spin_sim_seconds(3.0, sim_now=lambda: clock[1], spin_once=spin,
                     wall_now=lambda: clock[0], wall_deadline=10.0)
    assert clock == [6.0, 4.0]


@pytest.mark.parametrize("next_sim", [0.5, float("nan"), float("inf")])
def test_invalid_clock_fails_instead_of_satisfying_the_duration(next_sim):
    clock = [1.0]
    with pytest.raises(AssertionError, match="invalid or moved backwards"):
        spin_sim_seconds(3.0, sim_now=lambda: clock[0], spin_once=lambda: clock.__setitem__(0, next_sim),
                         wall_now=lambda: 0.0, wall_deadline=10.0)


def test_expired_deadline_does_not_issue_another_motion_command():
    commands = []
    with pytest.raises(AssertionError, match="wall-clock budget"):
        spin_sim_seconds(3.0, sim_now=lambda: 0.0, spin_once=lambda: None,
                         wall_now=lambda: 10.0, wall_deadline=10.0, on_tick=lambda: commands.append(1))
    assert commands == []


@pytest.fixture
def smoke(monkeypatch):
    # Load the actual launch-test methods without starting a graph or simulation.
    # Keep its isolated environment changes local to this fixture.
    for name in ("ROS_DOMAIN_ID", "ROS_AUTOMATIC_DISCOVERY_RANGE", "GZ_PARTITION"):
        monkeypatch.setenv(name, "fixture")
    spec = importlib.util.spec_from_file_location("g01_fixture", HERE / "test_g01_gazebo_smoke.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    case = module.TestGazeboSmoke()
    clock, waits, callbacks = [10.0], [], {}

    def subscribe(_kind, topic, callback, _qos):
        callbacks[topic] = callback
        return topic

    def wait(seconds, on_tick=None):
        waits.append(seconds)
        clock[0] += seconds

    case.node = SimpleNamespace(create_subscription=subscribe, destroy_subscription=lambda sub: None)
    case._sim_now = lambda: clock[0]
    case._spin_sim_seconds = wait
    case._start_observations()
    return module, case, clock, waits, callbacks


def test_motion_time_counts_for_both_complete_observation_windows(smoke):
    module, case, clock, waits, callbacks = smoke
    clock[0] += 7.0
    marker = object()
    callbacks["/tf"](marker)
    callbacks["/joint_states"](SimpleNamespace(name=["joint", "joint"]))
    assert case._finish_observation(module.TF_OBSERVATION_SECONDS)["sim_elapsed_s"] == 7.0
    assert case._finish_observation(module.JOINT_OBSERVATION_SECONDS)["sim_elapsed_s"] == 7.0
    assert waits == []
    assert case.tf_messages == [marker]
    assert case.sources["/joint_states"] == [["joint", "joint"]]


def test_incomplete_observation_waits_for_every_remaining_simulated_second(smoke):
    module, case, clock, waits, _ = smoke
    clock[0] += 1.0
    assert case._finish_observation(module.TF_OBSERVATION_SECONDS)["sim_elapsed_s"] == 5.0
    assert waits == [4.0]


def test_early_duplicate_joint_sample_still_fails_check_six(smoke):
    _, case, clock, _, callbacks = smoke
    callbacks["/joint_states"](SimpleNamespace(name=["joint", "joint"]))
    clock[0] += 7.0
    with pytest.raises(AssertionError, match="joint appears twice"):
        case.test_g01_6_joint_states_carry_wheels_and_arms_each_once()


def test_early_duplicate_tf_stamp_still_fails_check_five(smoke):
    _, case, clock, _, callbacks = smoke
    case._robot_description = lambda: '<link name="base_footprint"/><child_frame_id>base_footprint</child_frame_id>'
    for n in range(251):
        stamp = n // 2 * 0.02  # Duplicate stamps cannot be hidden by later good samples.
        callbacks["/tf"](SimpleNamespace(transforms=[SimpleNamespace(
            header=SimpleNamespace(frame_id="odom", stamp=SimpleNamespace(sec=int(stamp), nanosec=int(stamp % 1 * 1e9))),
            child_frame_id="base_footprint")]))
    clock[0] += 7.0
    with pytest.raises(AssertionError, match="repeated or out-of-order"):
        case.test_g01_5_exactly_one_publisher_of_odom_to_the_diffdrive_child_frame()


def test_base_is_commanded_to_stop_when_clock_wait_fails(smoke):
    _, case, _, _, _ = smoke
    commands = []
    case._base_xy = lambda: (0, 0)
    case.odom = []
    case.cmd_vel = SimpleNamespace(publish=commands.append)

    def frozen(*args, **kwargs):
        kwargs["on_tick"]()
        raise AssertionError("simulation clock frozen")

    case._spin_sim_seconds = frozen
    with pytest.raises(AssertionError, match="clock frozen"):
        case.test_g01_4_cmd_vel_moves_the_base_and_odom_reports_it()
    assert len(commands) == 2
    assert commands[0].linear.x == 0.2
    assert commands[-1].linear.x == 0.0
