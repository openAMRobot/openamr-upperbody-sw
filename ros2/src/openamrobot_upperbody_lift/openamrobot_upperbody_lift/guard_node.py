# SPDX-License-Identifier: MIT
"""lift_guard: MoveIt refuses to plan or execute while the lift state is missing or stale, and says why.

Why a guard and not only the relay: when the relay stops publishing lift_joint, move_group and robot_state_publisher keep the LAST value
(issue #45, point 3), so plans would go on being made from a lift height that is no longer known. The guard sits in front of move_group's
action servers, as a proxy of the same action types:

    client -> /move_action              (this node)  -> move_group's move_action, remapped to /move_action_unguarded
    client -> /execute_trajectory       (this node)  -> move_group's execute_trajectory, remapped to /execute_trajectory_unguarded
    client -> /plan_kinematic_path      (this node)  -> move_group's plan_kinematic_path service, remapped to /plan_kinematic_path_unguarded

A goal is forwarded only while the lift_joint sample on /joint_states is fresh (lift_guard_logic.LiftSampleGuard). Otherwise it is aborted at
once (the planning service answers) with MoveItErrorCodes INVALID_ROBOT_STATE and the reason in error_code.message (source "lift_guard"). A goal that is running when the
lift state goes stale is cancelled upstream and aborted, so an execution does not carry on blind. Nothing about the lift is commanded here.

This is a planning and execution request check, the simpler of the two options in the work package. It protects clients that talk to
/move_action and /execute_trajectory (RViz MotionPlanning, MoveGroupInterface, scripts). It is NOT a safety function: the base controller owns the
lift interlock (I8 rev 3), and the hardwired E-stop chain is the safety path.

Publishes ~/status (diagnostic_msgs/DiagnosticStatus) and /diagnostics, OK while the lift state is fresh, else ERROR with the reason.
"""
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

from openamrobot_upperbody_lift import guard_logic as gl

REFUSAL_CODE = MoveItErrorCodes.INVALID_ROBOT_STATE
UPSTREAM_WAIT_S = 2.0
PLAN_WAIT_S = 60.0
POLL_S = 0.02


def refusal_error_code(reason):
    code = MoveItErrorCodes(val=REFUSAL_CODE)
    if hasattr(code, "message"):                 # moveit_msgs 2.6 (Jazzy) has message and source
        code.message = reason
        code.source = gl.REASON_SOURCE
    return code


def move_group_refusal(reason):
    result = MoveGroup.Result()
    result.error_code = refusal_error_code(reason)
    return result


def execute_refusal(reason):
    result = ExecuteTrajectory.Result()
    result.error_code = refusal_error_code(reason)
    return result


class GuardedAction:
    """One guarded action: a server under `public_name`, forwarding to the client of `upstream_name` while the guard allows it."""

    def __init__(self, node, action_type, public_name, upstream_name, refusal, group):
        self.node, self.refusal, self.public_name = node, refusal, public_name
        self.client = ActionClient(node, action_type, upstream_name, callback_group=group)
        self.server = ActionServer(node, action_type, public_name, execute_callback=self.execute, goal_callback=lambda goal: GoalResponse.ACCEPT,
                                   cancel_callback=lambda goal: CancelResponse.ACCEPT, callback_group=group)

    def execute(self, goal_handle):
        node = self.node
        decision = node.guard.check(node.clock())
        if not decision:
            node.refused += 1
            node.get_logger().warn(f"refused {self.public_name}: {decision.reason}")
            goal_handle.abort()
            return self.refusal(decision.reason)
        if not self.client.wait_for_server(timeout_sec=UPSTREAM_WAIT_S):
            node.get_logger().error(f"{self.public_name}: the upstream server is not there")
            goal_handle.abort()
            return self.refusal("move_group is not available behind the lift guard")
        done = threading.Event()
        sent = self.client.send_goal_async(goal_handle.request, feedback_callback=lambda fb: goal_handle.publish_feedback(fb.feedback))
        sent.add_done_callback(lambda _: done.set())
        if not done.wait(UPSTREAM_WAIT_S) or not sent.result().accepted:
            goal_handle.abort()
            return self.refusal("move_group did not accept the goal")
        upstream = sent.result()
        finished = threading.Event()
        result_future = upstream.get_result_async()
        result_future.add_done_callback(lambda _: finished.set())
        reason = None
        while not finished.wait(POLL_S):
            if goal_handle.is_cancel_requested:
                upstream.cancel_goal_async()
                finished.wait(UPSTREAM_WAIT_S)
                goal_handle.canceled()
                return result_future.result().result if result_future.done() else self.refusal("cancelled")
            decision = node.guard.check(node.clock())
            if not decision:                    # the lift state went stale while the goal was running: stop it, do not let it carry on
                reason = decision.reason
                node.refused += 1
                node.get_logger().error(f"cancelling {self.public_name}: {reason}")
                upstream.cancel_goal_async()
                finished.wait(UPSTREAM_WAIT_S)
                break
        if reason is not None:
            goal_handle.abort()
            return self.refusal(reason)
        outcome = result_future.result()
        if outcome.status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return outcome.result


class GuardedPlanService:
    """/plan_kinematic_path (GetMotionPlan): the same check for clients that plan without the action."""

    def __init__(self, node, group):
        self.node = node
        self.client = node.create_client(GetMotionPlan, "plan_kinematic_path_unguarded", callback_group=group)
        node.create_service(GetMotionPlan, "plan_kinematic_path", self.handle, callback_group=group)

    def handle(self, request, response):
        node = self.node
        decision = node.guard.check(node.clock())
        if decision and not self.client.wait_for_service(timeout_sec=UPSTREAM_WAIT_S):
            decision = gl.GuardDecision(False, "move_group is not available behind the lift guard")
        if not decision:
            node.refused += 1
            node.get_logger().warn(f"refused plan_kinematic_path: {decision.reason}")
            response.motion_plan_response.error_code = refusal_error_code(decision.reason)
            return response
        done = threading.Event()
        future = self.client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(PLAN_WAIT_S) or future.result() is None:
            response.motion_plan_response.error_code = refusal_error_code("move_group did not answer the planning request")
            return response
        return future.result()


class LiftGuard(Node):
    def __init__(self, clock=time.monotonic):
        super().__init__("lift_guard")
        self.declare_parameter("max_sample_age_s", gl.MAX_SAMPLE_AGE_S)
        self.declare_parameter("diagnostic_period_s", 0.5)
        self.declare_parameter("relay_status_topic", "/lift_relay/status")
        self.guard = gl.LiftSampleGuard(self.get_parameter("max_sample_age_s").value)
        self.clock = clock
        self.refused = 0
        group = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, qos_profile_sensor_data, callback_group=group)
        self.create_subscription(DiagnosticStatus, self.get_parameter("relay_status_topic").value,
                                 lambda msg: self.guard.on_relay_status(msg.message), 10, callback_group=group)
        self.status_pub = self.create_publisher(DiagnosticStatus, "~/status", 10)
        self.diagnostics_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(float(self.get_parameter("diagnostic_period_s").value), self.on_timer, callback_group=group)
        self.actions = [GuardedAction(self, MoveGroup, "move_action", "move_action_unguarded", move_group_refusal, group),
                        GuardedAction(self, ExecuteTrajectory, "execute_trajectory", "execute_trajectory_unguarded", execute_refusal, group)]
        self.plan_service = GuardedPlanService(self, group)
        self.get_logger().info("lift guard up: /move_action, /execute_trajectory and /plan_kinematic_path are forwarded only while lift_joint is fresh")

    def on_joint_state(self, msg):
        self.guard.on_joint_state(msg.name, msg.position, self.clock())

    def on_timer(self):
        decision = self.guard.check(self.clock())
        status = DiagnosticStatus(level=DiagnosticStatus.OK if decision else DiagnosticStatus.ERROR, name="lift_guard",
                                  hardware_id="base_controller:aux_status", message=decision.reason)
        status.values = [KeyValue(key="allowed", value=str(bool(decision)).lower()), KeyValue(key="refused_goals", value=str(self.refused))]
        self.status_pub.publish(status)
        array = DiagnosticArray(status=[status])
        array.header.stamp = self.get_clock().now().to_msg()
        self.diagnostics_pub.publish(array)


def main(args=None):
    rclpy.init(args=args)
    node = LiftGuard()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
