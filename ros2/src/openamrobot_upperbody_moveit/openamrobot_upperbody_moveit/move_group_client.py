# SPDX-License-Identifier: MIT
"""A small client for move_group's `/move_action` (moveit_msgs/action/MoveGroup): plan to a joint goal, optionally execute it, and read the
joint states back. What M01, M02 and M04 use, so each of them goes through the real move_group node and, for execution, through MoveIt's
trajectory execution manager, the moveit_simple_controller_manager and the controllers' own action servers: the whole chain a user's
`moveit.launch.py` runs, not a shortcut around it.

Named poses come from the SRDF (named_poses.read_group_states): a named pose is a joint goal made of exactly the values the SRDF says.
"""
import json
import time

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes, PlanningOptions
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

ERROR_NAMES = {value: name for name, value in vars(MoveItErrorCodes).items() if name.isupper() and isinstance(value, int)}
# There is no table of "refusal" codes here on purpose (there was one until P12A): MoveIt 2.12.4 answers the generic FAILURE (99999) to the action and
# to /plan_kinematic_path alike, for a start state outside bounds and for a goal in collision, so a client cannot tell a refusal from a failed search
# by the code. The reason exists only in move_group's log ("... outside bounds by", "Planner 'OMPL' failed with error code GOAL_STATE_INVALID"), which is
# what M04 and the clamp's loop test read.


def error_name(code):
    return ERROR_NAMES.get(code, f"UNKNOWN({code})")


class Outcome:
    """What one request did. `accepted` is whether move_group took the goal at all; `code` is MoveItErrorCodes.val."""

    def __init__(self, accepted, code, planning_time, wall_seconds, executed_points=0):
        self.accepted, self.code = accepted, code
        self.planning_time, self.wall_seconds, self.executed_points = planning_time, wall_seconds, executed_points

    @property
    def success(self):
        return self.accepted and self.code == MoveItErrorCodes.SUCCESS

    @property
    def name(self):
        return "GOAL_NOT_ACCEPTED" if not self.accepted else error_name(self.code)

    def as_dict(self):
        return {"accepted": self.accepted, "error_code": self.code, "error_name": self.name,
                "planning_time_s": round(self.planning_time, 3), "wall_s": round(self.wall_seconds, 3)}


class MoveGroupClient:
    def __init__(self, node, action_name="/move_action", joint_states_topic="/joint_states"):
        self.node = node
        self.client = ActionClient(node, MoveGroup, action_name)
        self.plan_service = node.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.joint_states = {}
        node.create_subscription(JointState, joint_states_topic, self._on_joint_state, 10)

    def _on_joint_state(self, msg):
        self.joint_states.update(dict(zip(msg.name, msg.position)))

    def spin_until(self, predicate, seconds):
        """Spin the node until `predicate()` or `seconds` of the node's own clock pass (simulated time with use_sim_time)."""
        import rclpy
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def wait_for_server(self, seconds):
        return self.client.wait_for_server(timeout_sec=seconds)

    def wait_for_joints(self, names, seconds):
        return self.spin_until(lambda: all(n in self.joint_states for n in names), seconds)

    def request(self, group, joint_goal, velocity_scaling, acceleration_scaling, planning_time, tolerance=1e-3, attempts=1):
        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = group
        req.pipeline_id = "ompl"
        req.num_planning_attempts = attempts
        req.allowed_planning_time = float(planning_time)
        req.max_velocity_scaling_factor = float(velocity_scaling)
        req.max_acceleration_scaling_factor = float(acceleration_scaling)
        req.start_state.is_diff = True                                  # start from move_group's current state
        constraints = Constraints()
        for joint, value in joint_goal.items():
            constraints.joint_constraints.append(JointConstraint(joint_name=joint, position=float(value), tolerance_above=tolerance,
                                                                 tolerance_below=tolerance, weight=1.0))
        req.goal_constraints = [constraints]
        goal.request = req
        goal.planning_options = PlanningOptions()
        return goal

    def send(self, goal, execute, result_timeout):
        """Send a prepared MoveGroup goal. execute=False plans only. `result_timeout` bounds the wait for the result (node clock)."""
        goal.planning_options.plan_only = not execute
        started = time.monotonic()
        send = self.client.send_goal_async(goal)
        if not self.spin_until(send.done, 15.0):
            return Outcome(False, 0, 0.0, time.monotonic() - started)
        handle = send.result()
        if not handle.accepted:
            return Outcome(False, 0, 0.0, time.monotonic() - started)
        result = handle.get_result_async()
        if not self.spin_until(result.done, result_timeout):
            handle.cancel_goal_async()
            return Outcome(True, MoveItErrorCodes.TIMED_OUT, 0.0, time.monotonic() - started)
        response = result.result().result
        points = len(response.executed_trajectory.joint_trajectory.points)
        return Outcome(True, response.error_code.val, response.planning_time, time.monotonic() - started, points)

    def move(self, group, joint_goal, execute=True, velocity_scaling=0.5, acceleration_scaling=0.5, planning_time=5.0, result_timeout=90.0):
        """Plan (and execute) a joint goal, once. There is no retry: a request MoveIt refuses is an outcome of the check that sent it. (There
        used to be a bounded retry for a start state 1e-7 rad past a joint limit; the joint-state clamp, joint_state_clamp.py, took that noise
        off at its source, and a retry that never fires is dead code, and one that fires says something else is wrong.)"""
        goal = self.request(group, joint_goal, velocity_scaling, acceleration_scaling, planning_time)
        return self.send(goal, execute, result_timeout)

    def clamp_stats(self, timeout=10.0):
        """The counters of joint_state_clamp.py (its `~/get_stats` service) as a dict, or None when no clamp is running."""
        client = self.node.create_client(Trigger, "/joint_state_clamp/get_stats")
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None
            future = client.call_async(Trigger.Request())
            self.spin_until(future.done, timeout)
            return json.loads(future.result().message) if future.done() else None
        finally:
            self.node.destroy_client(client)

    def wait_inside_limits(self, limits, seconds, stable_seconds=1.0, tolerance=0.0):
        """Wait until every joint in `limits` ({joint: (lower, upper)}) has been inside its limits, widened by `tolerance` (the clamp's epsilon:
        float noise that far out is the clamp's to count, not this wait's), in every /joint_states message, for `stable_seconds`. Returns (ok, transient): `transient` lists the out-of-limit readings seen on the way, as (joint, value,
        lower, upper), so a recording says what the start-up transient was instead of hiding it.

        Why: MoveIt's CheckStartStateBounds refuses a request whose start state is even slightly outside a joint's limit
        (the reply is the generic FAILURE; the reason is only in move_group's log), and a joint resting on its limit (the grippers' fingers at 0) can read a few 1e-5 rad past it while a
        simulated model settles after spawning. That is a property of the start-up of the stack, not of the poses under test."""
        transient, clean_since = [], [None]

        def inside():
            bad = [(j, self.joint_states[j], lo, hi) for j, (lo, hi) in limits.items()
                   if j in self.joint_states and not lo - tolerance <= self.joint_states[j] <= hi + tolerance]
            now = time.monotonic()
            if bad:
                clean_since[0] = None
                transient.extend(b for b in bad if len(transient) < 20 and b[0] not in [t[0] for t in transient])
            elif clean_since[0] is None:
                clean_since[0] = now
            return clean_since[0] is not None and now - clean_since[0] >= stable_seconds

        return self.spin_until(inside, seconds), transient

    def plan_through_the_service(self, goal, seconds=60.0):
        """Plan only, through move_group's /plan_kinematic_path service instead of the MoveGroup action. Same pipeline, but the reply carries
        the pipeline's OWN error code and planning time: the action collapses every planning failure into the generic FAILURE (99999) and
        reports no planning time on failure, so it cannot tell a refusal from a search that ran out of time (found in M04)."""
        started = time.monotonic()
        if not self.plan_service.wait_for_service(timeout_sec=30.0):
            return Outcome(False, 0, 0.0, time.monotonic() - started)
        future = self.plan_service.call_async(GetMotionPlan.Request(motion_plan_request=goal.request))
        self.spin_until(future.done, seconds)
        if not future.done():
            return Outcome(True, MoveItErrorCodes.TIMED_OUT, 0.0, time.monotonic() - started)
        response = future.result().motion_plan_response
        return Outcome(True, response.error_code.val, response.planning_time, time.monotonic() - started)

    def settle(self, seconds):
        """Let joint states keep arriving for `seconds` so the last one read is the settled one."""
        self.spin_until(lambda: False, seconds)

    def max_error(self, joint_goal):
        """Largest |joint state - goal| over the goal's joints, from the latest /joint_states."""
        return max(abs(self.joint_states[j] - v) for j, v in joint_goal.items())
