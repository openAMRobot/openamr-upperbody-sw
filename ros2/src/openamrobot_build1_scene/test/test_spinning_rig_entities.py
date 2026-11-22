# SPDX-License-Identifier: MIT
"""No harness step destroys a ROS entity while the rig's executor spins the node in another thread.

rclpy raises InvalidHandle in the executor thread when a subscription or publisher it waits on is destroyed
("cannot use Destroyable because destruction was requested"); the thread ends and the rig receives nothing more
(run 44848867373, job 136675191420: host_client_drive_thread_alive False). 7327af6 fixed baseline1_cancel; these
tests cover single_writer_live (scn09, spinning=True) and the departure retreat option."""
import ast
import os

from openamrobot_build1_scene import single_writer_live

HERE = os.path.dirname(os.path.abspath(__file__))
PACKAGE = os.path.join(HERE, "..", "openamrobot_build1_scene")


class SpinningNode:
    """A node another thread spins: destroying an entity raises, as rclpy does there."""

    def __init__(self):
        self.destroyed = []

    def destroy_subscription(self, subscription):
        raise RuntimeError("InvalidHandle: cannot use Destroyable because destruction was requested")


class IdleNode:
    def __init__(self):
        self.destroyed = []

    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)


def test_release_keeps_subscriptions_when_another_thread_spins():
    stop = []
    single_writer_live.release(SpinningNode(), ["tf", "tf_static"], spinning=True, stop=stop)
    assert stop == [True]


def test_release_destroys_subscriptions_without_another_spinner():
    node, stop = IdleNode(), []
    single_writer_live.release(node, ["tf", "tf_static"], spinning=False, stop=stop)
    assert node.destroyed == ["tf", "tf_static"] and stop == [True]


def _calls(module_file, attribute):
    tree = ast.parse(open(os.path.join(PACKAGE, module_file)).read())
    return [n.lineno for n in ast.walk(tree)
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == attribute]


def test_the_departure_retreat_does_not_destroy_its_publisher():
    assert _calls("station_departure.py", "destroy_publisher") == []


def test_single_writer_live_destroys_subscriptions_only_in_release():
    tree = ast.parse(open(os.path.join(PACKAGE, "single_writer_live.py")).read())
    owners = []
    for function in [n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)]:
        for n in ast.walk(function):
            if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute) and n.func.attr == "destroy_subscription":
                owners.append(function.name)
    assert set(owners) == {"release"}, owners
