# SPDX-License-Identifier: MIT
"""The MoveIt guard in front of a fake move_group: a MoveGroup goal is refused with the reason while the lift is missing or stale, forwarded
while it is fresh, and a goal that is running when the lift goes stale is cancelled."""
import threading
import time

import pytest
import rclpy
from conftest import FakeClock
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from openamrobot_upperbody_lift.guard_node import LiftGuard


class Rig:
    def __init__(self, upstream_runs_for=0.0):
        self.clock = FakeClock()
        self.guard = LiftGuard(clock=self.clock)
        self.peer = rclpy.create_node("fake_move_group_and_client")
        group = ReentrantCallbackGroup()
        self.upstream_goals = 0
        self.upstream_cancelled = False
        self.upstream_runs_for = upstream_runs_for
        self.server = ActionServer(self.peer, MoveGroup, "move_action_unguarded", self.run_upstream, callback_group=group,
                                   goal_callback=lambda g: GoalResponse.ACCEPT, cancel_callback=self.on_cancel)
        self.plan_calls = 0
        self.peer.create_service(GetMotionPlan, "plan_kinematic_path_unguarded", self.plan_upstream, callback_group=group)
        self.client = ActionClient(self.peer, MoveGroup, "move_action", callback_group=group)
        self.plan_client = self.peer.create_client(GetMotionPlan, "plan_kinematic_path", callback_group=group)
        self.joint_pub = self.peer.create_publisher(JointState, "/joint_states", 10)
        self.executor = MultiThreadedExecutor(num_threads=8)
        self.executor.add_node(self.guard)
        self.executor.add_node(self.peer)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()
        assert self.client.wait_for_server(timeout_sec=10.0)
        assert self.plan_client.wait_for_service(timeout_sec=10.0)

    def on_cancel(self, goal):
        self.upstream_cancelled = True
        return CancelResponse.ACCEPT

    def run_upstream(self, goal_handle):
        self.upstream_goals += 1
        end = time.monotonic() + self.upstream_runs_for
        while time.monotonic() < end:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveGroup.Result()
            time.sleep(0.01)
        goal_handle.succeed()
        result = MoveGroup.Result()
        result.error_code.val = MoveItErrorCodes.SUCCESS
        return result

    def plan_upstream(self, request, response):
        self.plan_calls += 1
        response.motion_plan_response.error_code.val = MoveItErrorCodes.SUCCESS
        return response

    def lift_sample(self):
        self.joint_pub.publish(JointState(name=["lift_joint"], position=[0.175]))
        time.sleep(0.2)

    def send_goal(self, timeout=20.0):
        future = self.client.send_goal_async(MoveGroup.Goal())
        self._wait(future, timeout)
        handle = future.result()
        assert handle.accepted
        result = handle.get_result_async()
        self._wait(result, timeout)
        return handle, result.result()

    @staticmethod
    def _wait(future, timeout):
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        assert future.done(), "timed out"

    def close(self):
        self.executor.shutdown()
        self.thread.join(timeout=5)
        self.guard.destroy_node()
        self.peer.destroy_node()


@pytest.fixture
def rig():
    rclpy.init()
    r = Rig()
    yield r
    r.close()
    rclpy.shutdown()


@pytest.fixture
def slow_rig():
    rclpy.init()
    r = Rig(upstream_runs_for=30.0)
    yield r
    r.close()
    rclpy.shutdown()


def test_refuses_when_the_lift_state_is_missing_and_says_why(rig):
    _, result = rig.send_goal()
    assert result.result.error_code.val == MoveItErrorCodes.INVALID_ROBOT_STATE
    assert "no lift_joint state" in result.result.error_code.message and result.result.error_code.source == "lift_guard"
    assert rig.upstream_goals == 0


def test_forwards_while_the_lift_is_fresh(rig):
    rig.lift_sample()
    _, result = rig.send_goal()
    assert result.result.error_code.val == MoveItErrorCodes.SUCCESS and rig.upstream_goals == 1


def test_refuses_when_the_lift_state_goes_stale(rig):
    rig.lift_sample()
    rig.clock.advance(1.0)
    _, result = rig.send_goal()
    assert result.result.error_code.val == MoveItErrorCodes.INVALID_ROBOT_STATE and "stale" in result.result.error_code.message
    assert rig.upstream_goals == 0


def test_plan_service_is_guarded_too(rig):
    done = rig.plan_client.call_async(GetMotionPlan.Request())
    rig._wait(done, 20.0)
    code = done.result().motion_plan_response.error_code
    assert code.val == MoveItErrorCodes.INVALID_ROBOT_STATE and "no lift_joint state" in code.message and rig.plan_calls == 0
    rig.lift_sample()
    done = rig.plan_client.call_async(GetMotionPlan.Request())
    rig._wait(done, 20.0)
    assert done.result().motion_plan_response.error_code.val == MoveItErrorCodes.SUCCESS and rig.plan_calls == 1


def test_a_running_goal_is_cancelled_when_the_lift_goes_stale(slow_rig):
    rig = slow_rig
    rig.lift_sample()
    future = rig.client.send_goal_async(MoveGroup.Goal())
    rig._wait(future, 20.0)
    result = future.result().get_result_async()
    end = time.monotonic() + 10
    while rig.upstream_goals == 0 and time.monotonic() < end:
        time.sleep(0.01)
    assert rig.upstream_goals == 1
    rig.clock.advance(1.0)                                   # the lift state goes stale while the goal runs
    rig._wait(result, 20.0)
    assert rig.upstream_cancelled
    assert result.result().result.error_code.val == MoveItErrorCodes.INVALID_ROBOT_STATE
