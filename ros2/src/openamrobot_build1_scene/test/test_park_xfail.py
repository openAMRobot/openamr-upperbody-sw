# SPDX-License-Identifier: MIT
"""The approved expected failure of the measured park check (#108, #126) is exactly as narrow as approved, and the
counter never counts it as passed.

Scope (code owner, upperbody #130, 22 Nov 2026): scn17 only, the DART engine only, joint 4 of each arm only, issue #126.
Any other unmoved joint, scenario or engine fails; an unexpected pass fails too. No ROS graph: a fake rig gives the
joint readings. The scenario module imports rclpy, so the rig and scope cases run where ROS 2 Jazzy is installed (CI).
"""
import os
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "scripts"))
import count_build1_tests as counter  # noqa: E402

DART = "gz-physics-dartsim-plugin"
BULLET = "gz-physics-bullet-featherstone-plugin"


# ------------------------------------------------------------------ the counter (no ROS)
def _case(skip_text=None):
    case = ET.Element("testcase", classname="c", name="n")
    if skip_text is not None:
        ET.SubElement(case, "skipped", message=skip_text)
    return case


def test_the_counter_counts_xfail_apart_and_never_as_passed():
    assert counter.classify(_case("XFAIL (#126): scn17 ... did not move toward park"))[0] == "xfail"
    assert counter.classify(_case("BLOCKED: no /clock"))[0] == "blocked"
    assert counter.classify(_case("some other skip"))[0] == "skipped"
    assert counter.classify(_case("expected XFAIL without an issue"))[0] == "skipped"
    rows = [("c", "a", "passed", ""), ("c", "b", "xfail", "XFAIL (#126): x")]
    totals, text = counter.report(rows, [], "label")
    assert totals["passed"] == 1 and totals["xfail"] == 1 and "XFAIL 1" in text
    assert counter.exit_status(totals) == 0
    assert counter.exit_status({**totals, "passed": 0}) == 1, "XFAIL alone is not a passing suite"


# ------------------------------------------------------------------ the scope and the park check (rclpy)
@pytest.fixture
def scenarios():
    pytest.importorskip("rclpy", reason="the scenario module imports rclpy; runs in the Build 1 scene CI (#126)")
    from openamrobot_build1_scene import scenarios as module
    return module


class FakeRig:
    def __init__(self, engine, positions):
        self.physics_engine, self.positions = engine, positions

    def joint(self, name):
        return self.positions[name]

    def physics_joint(self, name):
        return self.positions[name]


def _park_check(scenarios, scenario, engine, side, unmoved):
    """Run _require_park_motion for an arm that starts 0.2 rad off PARK on every joint and reaches PARK except `unmoved`."""
    from openamrobot_build1_scene import rig as rig_module
    names = scenarios._arm_joints(side)
    start = {n: p + 0.2 for n, p in zip(names, scenarios.PARK)}
    end = {n: (start[n] if n in unmoved else p) for n, p in zip(names, scenarios.PARK)}
    log = rig_module.Log(scenario)
    before = (dict(start), dict(start))
    scenarios._require_park_motion(FakeRig(engine, end), log, side, before, f"test_{side}_park")
    return log


def test_scope_is_scn17_dart_joint4_only(scenarios):
    assert scenarios.park_xfail_scope("scn17_base_yaw_fast", DART, "left") == {"openarm_left_joint4"}
    assert scenarios.park_xfail_scope("scn17_base_yaw_fast", DART, "right") == {"openarm_right_joint4"}
    assert scenarios.park_xfail_scope("scn17_base_yaw_fast", BULLET, "left") == set()
    assert scenarios.park_xfail_scope("scn03_arms_reach", DART, "left") == set()
    assert scenarios.PARK_XFAIL_ISSUE == "#126"


def test_scn17_dart_joint4_alone_is_xfail_not_a_failure(scenarios):
    log = _park_check(scenarios, "scn17_base_yaw_fast", DART, "left", {"openarm_left_joint4"})
    assert log.failures == []
    assert [x["issue"] for x in log.data["expected_failures"]] == ["#126"]


@pytest.mark.parametrize("scenario, engine, side, unmoved", [
    ("scn17_base_yaw_fast", DART, "left", {"openarm_left_joint4", "openarm_left_joint2"}),   # another joint too
    ("scn17_base_yaw_fast", DART, "right", {"openarm_right_joint1"}),                      # a different joint
    ("scn17_base_yaw_fast", BULLET, "left", {"openarm_left_joint4"}),                      # another engine
    ("scn03_arms_reach", DART, "left", {"openarm_left_joint4"}),                           # another scenario
])
def test_anything_outside_the_scope_still_fails(scenarios, scenario, engine, side, unmoved):
    log = _park_check(scenarios, scenario, engine, side, unmoved)
    assert log.failures and "did not move" in log.failures[0]
    assert not log.data.get("expected_failures")


def test_an_unexpected_pass_fails_so_the_expectation_is_reviewed(scenarios):
    log = _park_check(scenarios, "scn17_base_yaw_fast", DART, "left", set())
    assert len(log.failures) == 1 and log.failures[0].startswith("XPASS") and "#126" in log.failures[0]
