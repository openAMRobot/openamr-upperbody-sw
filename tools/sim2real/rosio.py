# SPDX-License-Identifier: MIT
"""ROS side of the sim-to-real scripts: one node, recorders, and senders that respect --dry-run and the stop guard.
Every sender goes through Session.log_sent first; with --dry-run that is all it does."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint

import common


class Io:
    def __init__(self, session, guard, name):
        self.s, self.guard = session, guard
        rclpy.init()
        sim = session.args.target == "sim"
        self.node = rclpy.create_node(name, parameter_overrides=[Parameter("use_sim_time", value=sim)])
        self.js_rows, self.js_last, self.js_names = [], {}, None
        self.node.create_subscription(JointState, "/joint_states", self._js, 200)
        self.odom_rows = []
        self.urdf = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.node.create_subscription(String, "/robot_description", lambda m: setattr(self, "urdf", m.data), qos)
        self._goals = []
        self._twist_pubs = []
        guard.add(self.cancel_all)
        guard.add(self.zero_twist)

    # ---- time and spinning
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def spin(self, wall_s):
        end = time.time() + wall_s
        while time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def spin_sim(self, sim_s, wall_timeout=None):
        """Spin until `sim_s` of the node's clock (sim time in sim) has passed."""
        t0 = self.now()
        end = time.time() + (wall_timeout if wall_timeout else max(30.0, 10 * sim_s))
        while self.now() - t0 < sim_s and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def wait(self, fut, wall_s=60.0):
        end = time.time() + wall_s
        while not fut.done() and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.02)
        return fut.result() if fut.done() else None

    # ---- recorders
    def _js(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = {}
        for i, n in enumerate(m.name):
            d[n] = (m.position[i] if i < len(m.position) else math.nan,
                    m.velocity[i] if i < len(m.velocity) else math.nan,
                    m.effort[i] if i < len(m.effort) else math.nan)
        self.js_last.update({k: v for k, v in d.items()})
        self.js_rows.append((t, d))

    def record_odom(self, topic):
        def cb(m):
            p, o = m.pose.pose.position, m.pose.pose.orientation
            self.odom_rows.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, p.x, p.y, common.yaw_from_quat(o.x, o.y, o.z, o.w)))
        self.node.create_subscription(Odometry, topic, cb, 50)

    def wait_for_joint_states(self, names, wall_s=60.0):
        end = time.time() + wall_s
        while time.time() < end and not all(n in self.js_last for n in names):
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return all(n in self.js_last for n in names)

    def wait_for_urdf(self, wall_s=30.0):
        end = time.time() + wall_s
        while self.urdf is None and time.time() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)
        return self.urdf

    def pos(self, names):
        return [self.js_last[n][0] for n in names]

    def dump_joint_states(self, path, names):
        hdr = ["stamp_s"] + [f"{n}/{k}" for n in names for k in ("position", "velocity", "effort")]
        rows = [[t] + [v for n in names for v in d.get(n, (math.nan,) * 3)] for t, d in self.js_rows if any(n in d for n in names)]
        common.write_csv(path, hdr, rows)

    def dump_odom(self, path):
        common.write_csv(path, ["stamp_s", "x_m", "y_m", "yaw_rad"], self.odom_rows)

    # ---- senders
    def follow_joint_trajectory(self, action, joint_names, points, what, result_wall_s=120.0):
        """points: [(positions, t_from_start_s)]. Returns (accepted, error_code, error_string, t_sent, t_done)."""
        payload = {"action": action, "joints": joint_names, "points": [[list(p), t] for p, t in points]}
        self.s.log_sent(what, payload)
        if self.s.args.dry_run:
            return (None, None, "dry-run: not sent", None, None)
        common.confirm(self.s.args, what)
        ac = ActionClient(self.node, FollowJointTrajectory, action)
        if not ac.wait_for_server(timeout_sec=30.0):
            return (False, None, f"no action server {action}", None, None)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = list(joint_names)
        for p, t in points:
            sec = int(t)
            g.trajectory.points.append(JointTrajectoryPoint(positions=[float(x) for x in p],
                                                             time_from_start=Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))))
        t_sent = self.now()
        h = self.wait(ac.send_goal_async(g), 30.0)
        if h is None or not h.accepted:
            return (False, None, "goal rejected", t_sent, None)
        self._goals.append(h)
        r = self.wait(h.get_result_async(), result_wall_s)
        self._goals.remove(h)
        if r is None:
            h.cancel_goal_async()
            return (True, None, "no result in time (goal cancelled)", t_sent, self.now())
        return (True, r.result.error_code, r.result.error_string, t_sent, self.now())

    def gripper(self, side, position, max_effort, what):
        eff = common.cap_gripper_effort(max_effort, self.s.caps)
        action = common.GRIPPER_ACTION[side]
        self.s.log_sent(what, {"action": action, "position_raw": position, "max_effort": eff})
        if self.s.args.dry_run:
            return {"sent": False}
        common.confirm(self.s.args, what)
        ac = ActionClient(self.node, GripperCommand, action)
        if not ac.wait_for_server(timeout_sec=30.0):
            return {"sent": False, "error": f"no action server {action}"}
        g = GripperCommand.Goal()
        g.command.position, g.command.max_effort = float(position), float(eff)
        t_sent = self.now()
        h = self.wait(ac.send_goal_async(g), 30.0)
        if h is None or not h.accepted:
            return {"sent": True, "accepted": False, "t_sent": t_sent}
        self._goals.append(h)
        r = self.wait(h.get_result_async(), 60.0)
        self._goals.remove(h)
        if r is None:
            h.cancel_goal_async()
            return {"sent": True, "accepted": True, "result": None, "t_sent": t_sent}
        return {"sent": True, "accepted": True, "t_sent": t_sent, "t_done": self.now(), "status": r.status,
                "result": {"position": r.result.position, "effort": r.result.effort, "stalled": r.result.stalled,
                           "reached_goal": r.result.reached_goal}}

    def twist_publisher(self, topic):
        pub = self.node.create_publisher(Twist, topic, 10)
        self._twist_pubs.append(pub)
        return pub

    def drive(self, pub, topic, linear_x, angular_z, duration_s, what, rate_hz=20.0):
        """Publish a capped Twist for `duration_s` of the node's clock, then zero. Returns (t_start, t_end) or None (dry-run)."""
        lx, az = common.cap_base_twist(linear_x, angular_z, self.s.caps)
        self.s.log_sent(what, {"topic": topic, "linear_x": lx, "angular_z": az, "duration_s": duration_s, "rate_hz": rate_hz})
        if self.s.args.dry_run:
            return None
        common.confirm(self.s.args, what)
        msg = Twist()
        msg.linear.x, msg.angular.z = lx, az
        t0 = self.now()
        next_wall = time.time()
        while self.now() - t0 < duration_s:
            if time.time() >= next_wall:
                pub.publish(msg)
                next_wall += 1.0 / rate_hz
            rclpy.spin_once(self.node, timeout_sec=0.005)
        t1 = self.now()
        self.zero_twist()
        return (t0, t1)

    # ---- stop actions (also run by the guard on Ctrl+C)
    def cancel_all(self):
        for h in list(self._goals):
            h.cancel_goal_async()
        if self._goals:
            self.spin(0.5)

    def zero_twist(self):
        if self.s.args.dry_run:
            return
        for pub in self._twist_pubs:
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(0.02)

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()


def park_check(io, tol=0.05):
    """AD-30 precondition: both arms within `tol` rad of SRDF `park` (all zero, joint4 0.1). Returns (ok, worst_joint, error)."""
    worst, wj = 0.0, None
    for side in ("left", "right"):
        for i in range(1, 8):
            n = f"openarm_{side}_joint{i}"
            target = 0.1 if i == 4 else 0.0
            if n not in io.js_last:
                return False, n, math.inf
            e = abs(io.js_last[n][0] - target)
            if e > worst:
                worst, wj = e, n
    return worst <= tol, wj, worst


PARK = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0]          # SRDF openamrobot.srdf group_state park (left_arm / right_arm)
HOME = [0.0] * 7                                    # SRDF home
HANDS_UP = [0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0]      # SRDF hands_up


def arm_joints(side):
    return [f"openarm_{side}_joint{i}" for i in range(1, 8)]


def move_arm(io, side, target, scaling, vel_limits, what):
    names = arm_joints(side)
    q0 = io.pos(names)
    dur = max(1.0, common.segment_duration(q0, target, vel_limits, scaling))
    r = io.follow_joint_trajectory(common.ARM_CONTROLLER[side] + "/follow_joint_trajectory", names, [(target, dur)], what)
    return r, dur
