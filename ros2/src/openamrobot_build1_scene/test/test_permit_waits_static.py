# SPDX-License-Identifier: MIT
"""No mission or scenario step reads permit_fresh from the cached emulator state (review of #94, 20 Nov 2026).

stop_authorize in baseline1.py and scenarios.py (scn02's withdraw and grant, scn05's grant and fresh-again) waited on
rig.get(emulator)["permit_fresh"], which can be older than the change they wait for; they passed by timing, as
lift_move did before #94. They now use Rig.wait_permit_state / wait_permit_fresh_after_grant, which read only the second
state after the call. Pure Python: no ROS import.
"""
import ast
import os

PACKAGE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _source(name):
    with open(os.path.join(PACKAGE, "openamrobot_build1_scene", name), encoding="utf-8") as f:
        return f.read()


def _cached_reads(text):
    """Lambdas passed to rig.wait that read ['permit_fresh'] from rig.get(...)."""
    found = []
    for node in ast.walk(ast.parse(text)):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "wait":
            for arg in node.args:
                if isinstance(arg, ast.Lambda) and "permit_fresh" in ast.unparse(arg):
                    found.append(node.lineno)
    return found


def _calls(text, name):
    return [n for n in ast.walk(ast.parse(text)) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)
            and n.func.attr == name]


def test_no_cached_permit_fresh_wait_in_the_mission_or_the_scenarios():
    for name in ("baseline1.py", "scenarios.py"):
        assert _cached_reads(_source(name)) == [], name


def test_every_permit_change_in_those_steps_is_followed_by_the_after_call_wait():
    baseline1, scenarios = _source("baseline1.py"), _source("scenarios.py")
    assert len(_calls(baseline1, "wait_permit_state")) == 1 and len(_calls(baseline1, "wait_permit_fresh_after_grant")) == 1
    assert len(_calls(scenarios, "wait_permit_state")) == 2 and len(_calls(scenarios, "wait_permit_fresh_after_grant")) == 2
    rig = _source("rig.py")
    assert "self.count(emulator) >= after + 2" in rig and "def wait_permit_state(" in rig
