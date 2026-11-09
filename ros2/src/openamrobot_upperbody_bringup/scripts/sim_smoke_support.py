# SPDX-License-Identifier: MIT
"""Measurements on a running integrated simulation (robot_sim.launch.py): used by sim_smoke_measure.py (prints the numbers) and by
test/test_integrated_sim_smoke.py (asserts on them). Needs the ROS graph and the gz transport partition of the run (GZ_PARTITION).
Simulation evidence only: nothing here says anything about the physical robot."""
import json
import math
import subprocess
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from trajectory_msgs.msg import JointTrajectoryPoint

WORLD = "empty"
MODEL = "openamrobot"
CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller", "left_gripper_controller",
               "right_gripper_controller", "lift_controller")
# one of each: these nodes exist exactly once in a healthy integrated run
SINGLE_NODES = ("/robot_state_publisher", "/joint_state_publisher", "/controller_manager", "/platform_bridge")
# topic -> the publisher count a healthy run has (the real robot's names, D-10 for the IMU)
SINGLE_PUBLISHER_TOPICS = ("/clock", "/odom", "/scan", "/imu/data_raw", "/joint_states", "/robot_description")
LIFT_STOPS_M = {"L1000": 0.0, "L1175": 0.175, "L1350": 0.350}     # lift_joint positions of the SRDF states
LIFT_SECONDS = 14.0     # sim time per 0.175 m move; the DOLD no-load speed in the model is 0.018 m/s (9.7 s), plus margin
ARMS = {side: [f"openarm_{side}_joint{i}" for i in range(1, 8)] for side in ("left", "right")}
ARM_POSE = {"park": [0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0], "raised": [0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0]}


def yaw_of(q):
    return math.atan2(2 * (q["w"] * q["z"] + q["x"] * q["y"]), 1 - 2 * (q["y"] ** 2 + q["z"] ** 2))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def gz_json(topic, timeout_s=20):
    out = subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", "1", "--json-output"], capture_output=True, text=True, timeout=timeout_s)
    return json.loads(out.stdout.strip().splitlines()[0])


def ground_truth_pose():
    """(x, y, yaw) of the model in the world, from the simulator's own pose stream, not from odometry."""
    for entry in gz_json(f"/world/{WORLD}/dynamic_pose/info")["pose"]:
        if entry["name"] == MODEL:
            p, q = entry.get("position", {}), entry.get("orientation", {})
            q = {k: q.get(k, 0.0) for k in "xyzw"}
            return p.get("x", 0.0), p.get("y", 0.0), yaw_of(q)
    raise RuntimeError(f"no pose for {MODEL}")


def real_time_factor(window_s=10.0):
    """Sim seconds per wall second over `window_s` wall seconds, from the world statistics."""
    def now():
        s = gz_json(f"/world/{WORLD}/stats")
        return float(s["simTime"].get("sec", 0)) + float(s["simTime"].get("nsec", 0)) * 1e-9, time.monotonic()
    (s0, w0), _ = now(), time.sleep(window_s)
    s1, w1 = now()
    return (s1 - s0) / (w1 - w0)


class Smoke:
    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("integrated_sim_smoke", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.odom, self.clock_stamps = [], []
        self.node.create_subscription(Odometry, "/odom", self.odom.append, 100)
        self.node.create_subscription(Clock, "/clock", lambda m: self.clock_stamps.append((time.monotonic(), m.clock.sec + m.clock.nanosec * 1e-9)), 100)
        self.cmd_vel = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.list_controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.joint_state = {}
        from sensor_msgs.msg import JointState
        self.node.create_subscription(JointState, "/joint_states", lambda m: self.joint_state.update(zip(m.name, m.position)), 10)

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.05)

    def wait_until(self, predicate, timeout_s):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if predicate():
                return True
        return False

    def controller_states(self, timeout_s=180.0):
        """{name: state} once all CONTROLLERS are active, or the last answer after the timeout."""
        states = {}

        def active():
            if not self.list_controllers.service_is_ready():
                return False
            future = self.list_controllers.call_async(ListControllers.Request())
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            if future.result() is None:
                return False
            states.clear()
            states.update({c.name: c.state for c in future.result().controller})
            return all(states.get(n) == "active" for n in CONTROLLERS)
        self.wait_until(active, timeout_s)
        return states

    def graph(self):
        """node names (with duplicates kept), topics with publisher counts."""
        names = [f"{ns.rstrip('/')}/{n}" for n, ns in self.node.get_node_names_and_namespaces()]
        topics = {name: len(self.node.get_publishers_info_by_topic(name)) for name, _ in self.node.get_topic_names_and_types()}
        return names, topics

    def clock_rate(self):
        """(messages per wall second, sim seconds per wall second) over what /clock delivered since the node started."""
        (w0, s0), (w1, s1) = self.clock_stamps[0], self.clock_stamps[-1]
        return (len(self.clock_stamps) - 1) / (w1 - w0), (s1 - s0) / (w1 - w0)

    def _drive(self, linear, angular, sim_seconds):
        """Open loop: publish for `sim_seconds` of /clock time, then zero. Returns the ground-truth pose before and after."""
        before = ground_truth_pose()
        odom_before = self.odom[-1]
        start = self.clock_stamps[-1][1]
        msg = Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        while self.clock_stamps[-1][1] - start < sim_seconds:
            self.cmd_vel.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)
        self.cmd_vel.publish(Twist())
        self.spin_for(2.0)         # let it stop (the plugin's own deceleration, platform-sw's)
        self.cmd_vel.publish(Twist())
        self.spin_for(1.0)
        return before, ground_truth_pose(), odom_before, self.odom[-1]

    def drive_forward(self, distance_m=1.0, speed=0.2):
        b, a, ob, oa = self._drive(speed, 0.0, distance_m / speed)
        truth = math.hypot(a[0] - b[0], a[1] - b[1])
        odom = math.hypot(oa.pose.pose.position.x - ob.pose.pose.position.x, oa.pose.pose.position.y - ob.pose.pose.position.y)
        return {"commanded_m": distance_m, "ground_truth_m": truth, "odom_m": odom, "odom_minus_truth_m": odom - truth}

    def rotate(self, angle_rad=math.pi / 2, rate=0.4):
        b, a, ob, oa = self._drive(0.0, rate, angle_rad / rate)
        truth = wrap(a[2] - b[2])
        odom = wrap(yaw_of({"x": oa.pose.pose.orientation.x, "y": oa.pose.pose.orientation.y, "z": oa.pose.pose.orientation.z,
                            "w": oa.pose.pose.orientation.w})
                    - yaw_of({"x": ob.pose.pose.orientation.x, "y": ob.pose.pose.orientation.y, "z": ob.pose.pose.orientation.z,
                              "w": ob.pose.pose.orientation.w}))
        return {"commanded_deg": math.degrees(angle_rad), "ground_truth_deg": math.degrees(truth), "odom_deg": math.degrees(odom),
                "odom_minus_truth_deg": math.degrees(odom - truth)}

    def follow(self, controller, joints, positions, seconds, timeout_s=240.0):
        """One FollowJointTrajectory goal; returns (error_code, max |final error| of the goal's joints, seconds of sim time taken)."""
        client = ActionClient(self.node, FollowJointTrajectory, f"/{controller}/follow_joint_trajectory")
        if not client.wait_for_server(timeout_sec=30.0):
            raise RuntimeError(f"/{controller}/follow_joint_trajectory not available")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        point = JointTrajectoryPoint(positions=[float(p) for p in positions])
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        goal.trajectory.points = [point]
        t0 = self.clock_stamps[-1][1]
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send, timeout_sec=30.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            return None, math.inf, 0.0
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result, timeout_sec=timeout_s)
        if result.result() is None:
            return None, math.inf, math.inf
        self.spin_for(1.0)
        error = max(abs(self.joint_state.get(j, math.inf) - p) for j, p in zip(joints, positions))
        return result.result().result.error_code, error, self.clock_stamps[-1][1] - t0

    def lift_through_stops(self):
        rows, position = [], 0.0
        for stop in ("L1175", "L1350"):
            target = LIFT_STOPS_M[stop]
            code, error, took = self.follow("lift_controller", ["lift_joint"], [target], LIFT_SECONDS * abs(target - position) / 0.175)
            rows.append({"stop": stop, "target_m": target, "error_code": code, "final_error_m": error, "sim_seconds": took})
            position = target
        return rows

    def arms_to(self, pose):
        rows = []
        for side, joints in ARMS.items():
            code, error, took = self.follow(f"{side}_arm_controller", joints, ARM_POSE[pose], 5.0)
            rows.append({"arm": side, "pose": pose, "error_code": code, "final_error_rad": error, "sim_seconds": took})
        return rows
