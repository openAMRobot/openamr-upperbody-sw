# SPDX-License-Identifier: MIT
"""The software Stop lifecycle check (openamrobot_build1_scene/software_stop.py), without ROS."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import software_stop as ss  # noqa: E402

GOOD = dict(gate_expected=True, gate_present=True, initial_state=ss.STATE_LOCKED, moved_m=0.0,
            release_ok=True, release_message="released", final_state=ss.STATE_RUNNING)


def test_the_fail_closed_start_and_an_operator_release_pass():
    assert ss.problems(**GOOD) == []
    assert ss.problems(**{**GOOD, "initial_state": ss.STATE_RELEASE_REFUSED}) == []


def test_a_platform_without_the_gate_needs_nothing():
    assert ss.problems(**{**GOOD, "gate_expected": False, "gate_present": False}) == []


def test_each_broken_step_fails():
    cases = {
        "gate_present": False,                     # the gate is missing although the platform ships it
        "initial_state": ss.STATE_RUNNING,         # it did not start engaged
        "moved_m": 0.02,                           # the base moved before the first goal
        "release_ok": False,                       # the release was refused
        "final_state": ss.STATE_LOCKED,            # still engaged after the release
    }
    for key, value in cases.items():
        assert ss.problems(**{**GOOD, key: value}), key
    assert ss.problems(**{**GOOD, "moved_m": None}), "an unmeasured motion is not a pass"


def test_a_gate_on_a_platform_that_does_not_declare_one_is_reported():
    assert ss.problems(**{**GOOD, "gate_expected": False})


def test_the_launch_enables_only_the_navigation_producers():
    """build1.launch.py starts no docking server, so the Stop gate must not wait for dock_robot or undock_robot
    (navigation_launch.py's own default enables them, and Resume is then refused forever)."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "launch", "build1.launch.py")
    source = open(path, encoding="utf-8").read()
    assert 'NAV2_ONLY_PRODUCERS = {"enabled_producers": "navigate_to_pose,navigate_through_poses"}' in source
    assert '("navigation_launch.py", NAV2_ONLY_PRODUCERS)' in source
