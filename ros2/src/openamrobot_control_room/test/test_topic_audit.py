# SPDX-License-Identifier: MIT
"""The topics the monitor reads against the topics two recorded runs actually had (ros2 bag info of Baseline 1 run
44309817813 and grocery run 44299879412, test/fixtures/runs). A topic the monitor waits for that no run has, and that
is not listed with its reason in state.NOT_IN_RECORDED_RUNS, fails: the control room would show an empty field as if
the robot had nothing to say. No ROS."""
import re

import pytest

from conftest import package_path
from openamrobot_control_room import state

RUNS = ("baseline1_44309817813", "grocery_44299879412")


def bag_topics(run):
    with open(package_path("test", "fixtures", "runs", run, "bag_info.txt"), encoding="utf-8") as f:
        return set(re.findall(r"Topic: (\S+) \|", f.read()))


def test_bag_info_fixtures_have_topics():
    for run in RUNS:
        assert len(bag_topics(run)) > 20, run


@pytest.mark.parametrize("topic", sorted(state.MONITOR_TOPICS))
def test_monitor_topic_exists_in_a_recorded_run_or_is_labelled_absent(topic):
    seen = [run for run in RUNS if topic in bag_topics(run)]
    if topic in state.NOT_IN_RECORDED_RUNS:
        assert not seen, f"{topic} is labelled absent ({state.NOT_IN_RECORDED_RUNS[topic]}) but {seen} recorded it"
    else:
        assert seen, f"{topic}: no recorded run has it; label it in state.NOT_IN_RECORDED_RUNS with the reason"


def test_grocery_mission_topics_are_the_grocery_runs():
    assert {"/grocery_demo/phase", "/grocery_demo/events"} <= bag_topics("grocery_44299879412")
    assert not {"/grocery_demo/phase", "/grocery_demo/events"} & bag_topics("baseline1_44309817813")


def test_action_status_topics_are_not_in_any_bag():
    """Hidden topics: the recorders do not pass --include-hidden-topics. The live monitor still reads them."""
    for run in RUNS:
        assert not [t for t in bag_topics(run) if "/_action/" in t]
