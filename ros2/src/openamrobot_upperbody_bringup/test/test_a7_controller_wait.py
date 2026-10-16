"""M02 waits for its controllers to be active, and FAILs if they never are (P17 task 4), against a fake controller manager. No simulator.

The wait is `a7_support.wait_for_controllers_active`. What is pinned: it returns as soon as every named controller is `active`; it does NOT
return ok while one is `inactive` or missing, and gives up at its timeout reporting the states it saw; and with no controller manager at all
it gives up at the timeout with no states. A caller that sends a pose after a False is measuring the harness, not the robot."""
import os
import random
import sys
import threading
import time

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"

import pytest  # noqa: E402
import rclpy  # noqa: E402
from controller_manager_msgs.msg import ControllerState  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import a7_support  # noqa: E402

NAMES = a7_support.ARM_CONTROLLERS


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def manager():
    """A fake /controller_manager/list_controllers whose states the test sets, spun in a thread."""
    node = rclpy.create_node(f"fake_controller_manager_{random.randint(0, 10 ** 6)}")
    states = {}

    def handle(request, response):
        response.controller = [ControllerState(name=n, state=s) for n, s in states.items()]
        return response

    node.create_service(ListControllers, "/controller_manager/list_controllers", handle)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    yield states
    executor.shutdown()
    thread.join(timeout=5)
    node.destroy_node()


@pytest.fixture
def client():
    node = rclpy.create_node(f"wait_client_{random.randint(0, 10 ** 6)}")
    yield node
    node.destroy_node()


def test_all_active_returns_at_once(manager, client):
    manager.update({n: "active" for n in NAMES})
    started = time.monotonic()
    ok, states, waited = a7_support.wait_for_controllers_active(client, timeout_s=20.0)
    assert ok and set(states) == set(NAMES) and time.monotonic() - started < 10.0


def test_one_controller_that_never_activates_fails_at_the_timeout_and_the_states_are_reported(manager, client):
    manager.update({n: "active" for n in NAMES})
    manager["right_arm_controller"] = "inactive"
    started = time.monotonic()
    ok, states, waited = a7_support.wait_for_controllers_active(client, timeout_s=3.0)
    assert not ok
    assert states["right_arm_controller"] == "inactive" and states["left_arm_controller"] == "active"
    assert 2.5 <= time.monotonic() - started < 15.0, "it must wait out the timeout, not give up early and not hang"


def test_a_controller_that_activates_late_is_waited_for(manager, client):
    manager.update({n: "active" for n in NAMES})
    manager["left_gripper_controller"] = "inactive"
    threading.Timer(1.5, lambda: manager.__setitem__("left_gripper_controller", "active")).start()
    ok, states, waited = a7_support.wait_for_controllers_active(client, timeout_s=15.0)
    assert ok and waited >= 1.0 and states["left_gripper_controller"] == "active"


def test_a_missing_controller_is_not_active(manager, client):
    manager.update({n: "active" for n in NAMES if n != "right_gripper_controller"})
    ok, states, _ = a7_support.wait_for_controllers_active(client, timeout_s=2.0)
    assert not ok and "right_gripper_controller" not in states


def test_no_controller_manager_at_all_fails_at_the_timeout(client):
    started = time.monotonic()
    ok, states, _ = a7_support.wait_for_controllers_active(client, timeout_s=2.0, manager="/no_such_controller_manager")
    assert not ok and states == {}
    assert time.monotonic() - started < 15.0
