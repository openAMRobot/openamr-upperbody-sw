# SPDX-License-Identifier: MIT
"""The test rig of the Build 1 scenarios: one ROS node that watches the running stack, commands it the way a user
would, and writes a quantitative log.

Rules the rig keeps
- Durations are Gazebo time. A wait has a Gazebo time limit and a much longer wall limit, so a slow machine does
  not fail a functional check. The measured real-time factor goes into every log.
- BLOCKED is not PASS. A scenario raises Blocked when a prerequisite is missing or when a wall-clock device rule
  cannot hold at the measured real-time factor. The caller reports that as BLOCKED.
- A fault scenario records what the stack does today. `expected` entries come from config/expected_reactions.yaml,
  a simulation-only reference. They are expected behaviour, not certified safety behaviour.

Simulation only.
"""
import json
import math
import os
import threading
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Bool, Int8, String
from tf2_ros import Buffer, TransformException, TransformListener

MIN_RTF_FOR_WALL = 0.05          # wall limit of a wait = Gazebo limit / this, plus a margin
LIFT_IDLE, LIFT_FAULT = 3, 6     # CTL-017 states (vendored emulator: INIT, UNHOMED, HOMING, IDLE, MOVING, STOPPING, FAULT)
LIFT_REFUSED = 0x80              # openamrobot_sil config/sil.yaml lift.permit.refused_result, a SIL test value
LIFT_STOP_M = (0.0, 0.175, 0.350)   # lift_joint at L1000, L1175, L1350
# The CTL-017 model raises F02 when the Hall rate falls below half of the nominal one (sil-topology.md section 6).
LIFT_MIN_RTF = 0.55

# The baseline lock the results name (rule 1 of run_profiles.md). A proposal held in audits, not a release pin.
BASELINE_LOCK = "audits audit/2026-09-28 at be17653, digital-twin/phase1/digital_twin.lock.yaml of 2026-11-15 (Proposed)"
FINGERS = "/build1/ground_truth/finger_joint_states"

JSON_TOPICS = ("/sil/drive/status", "/sil/drive/emulator_state", "/sil/lift/status", "/sil/lift/emulator_state",
               "/sil/bms/status", "/sil/bms/emulator_state", "/build1/rtf_stats", "/sil/fw/status")


class Blocked(Exception):
    """A prerequisite is missing. Never a pass."""


class ScenarioFailed(AssertionError):
    pass


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def pose2d(odometry):
    p = odometry.pose.pose
    return p.position.x, p.position.y, yaw_of(p.orientation)


class Log:
    """Quantities and events of one scenario. Written as JSON and printed line by line."""

    def __init__(self, scenario):
        self.data = {"scenario": scenario, "evidence_class": "simulation", "result": None, "quantities": [], "events": [],
                     "disclaimer": "Simulation only. Not hardware, physical acceptance or safety evidence.",
                     "started_wall": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
        self.failures = []

    def quantity(self, name, value, unit="", low=None, high=None, kind="measured", note=""):
        """Record a number. With low or high it is also a check: outside the range fails the scenario at the end."""
        ok = None
        if low is not None or high is not None:
            ok = value is not None and (low is None or value >= low) and (high is None or value <= high)
            if not ok:
                self.failures.append(f"{name} = {value} {unit}, allowed {low} to {high}")
        shown = round(value, 5) if isinstance(value, float) else value
        self.data["quantities"].append({"name": name, "value": shown, "unit": unit, "low": low, "high": high, "ok": ok,
                                        "kind": kind, "note": note})
        print(f"BUILD1 {self.data['scenario']} {name} = {shown} {unit}"
              + (f" [{low}, {high}] {'ok' if ok else 'OUTSIDE'}" if ok is not None else "") + (f" ({note})" if note else ""), flush=True)
        return value

    def observed(self, name, value, unit="", expected=None, reference="", note=""):
        """Record what the stack does today in a fault case. `expected` is the simulation-only reference. It never
        fails the scenario: the log says whether today's behaviour matches the reference."""
        matches = None if expected is None else (value == expected)
        shown = round(value, 5) if isinstance(value, float) else value
        self.data["quantities"].append({"name": name, "value": shown, "unit": unit, "kind": "observed_today",
                                        "expected_reference": expected, "matches_reference": matches, "reference": reference,
                                        "note": note or "expected behaviour from a simulation-only reference, "
                                                        "not certified safety behaviour"})
        print(f"BUILD1 {self.data['scenario']} OBSERVED {name} = {shown} {unit}"
              + (f" (reference expects {expected}: {'matches' if matches else 'DIFFERS'})" if expected is not None else ""), flush=True)
        return value

    def event(self, text):
        self.data["events"].append({"wall": round(time.time(), 3), "text": text})
        print(f"BUILD1 {self.data['scenario']} event: {text}", flush=True)

    def require(self, condition, text):
        if not condition:
            self.failures.append(text)
            print(f"BUILD1 {self.data['scenario']} FAILED CHECK: {text}", flush=True)
        return bool(condition)

    def write(self, result, directory=None, reason=""):
        self.data["result"], self.data["reason"] = result, reason
        self.data["failures"] = list(self.failures)
        directory = directory or os.environ.get("BUILD1_LOG_DIR", os.path.join(os.getcwd(), "build1_logs"))
        os.makedirs(directory, exist_ok=True)
        path = os.path.join(directory, f"{self.data['scenario']}.json")
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.data, f, indent=1, sort_keys=True)
        print(f"BUILD1 {self.data['scenario']} RESULT {result} {reason} log {path}", flush=True)
        return path


class Rig:
    def __init__(self, scenario, node_name=None):
        self.log = Log(scenario)
        self.owns_rclpy = not rclpy.ok()
        if self.owns_rclpy:
            rclpy.init()
        self.node = Node(node_name or f"build1_{scenario}", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.last, self.stamp, self.counts = {}, {}, {}
        self.lock = threading.Lock()
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for topic in JSON_TOPICS:
            self.node.create_subscription(String, topic, lambda m, t=topic: self._store(t, self._json(m)), 10)
        for topic, kind, qos in (("/odom", Odometry, 10), ("/build1/ground_truth/odom", Odometry, 10),
                                 ("/sil/gz/odom", Odometry, 10),
                                 ("/joint_states", JointState, 10), ("/sim/joint_states", JointState, 10),
                                 ("/sil/gz/joint_states", JointState, 10), ("/arm/joint_states", JointState, 10),
                                 (FINGERS, JointState, 10),
                                 ("/scan", LaserScan, sensor_qos), ("/scan_filtered", LaserScan, 10)):
            self.node.create_subscription(kind, topic, lambda m, t=topic: self._store(t, m), qos)
        for name in ("item", "tote", "payload"):
            topic = f"/build1/ground_truth/{name}/pose"
            self.node.create_subscription(PoseStamped, topic, lambda m, t=topic: self._store(t, m), 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.node.create_subscription(String, "/build1/planning_scene/objects",
                                      lambda m: self._store("/build1/planning_scene/objects", m.data), latched)
        self.node.create_subscription(String, "/build1/run_info", lambda m: self._store("/build1/run_info", self._json(m)), latched)
        self.devices = True              # set from /build1/run_info in wait_ready
        self.physics = "/sim/joint_states"
        self.plant_switches = 0
        self._plant = None
        self.pub = {"permit": self.node.create_publisher(Bool, "/sil/lift/permit", 10),
                    "permit_silent": self.node.create_publisher(Bool, "/sil/lift/permit_silent", 10),
                    "lift": self.node.create_publisher(Int8, "/sil/lift/command", 10),
                    "can1_silent": self.node.create_publisher(Bool, "/sil/stm32/can1_silent", 10),
                    "drive_inject": self.node.create_publisher(String, "/sil/drive/inject", 10),
                    "lift_inject": self.node.create_publisher(String, "/sil/lift/inject", 10),
                    "bms_inject": self.node.create_publisher(String, "/sil/bms/inject", 10),
                    "exclude": self.node.create_publisher(String, "/build1/planning_scene/exclude", 10)}
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.actions = {}
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    # ------------------------------------------------------------ storage and time
    @staticmethod
    def _json(msg):
        try:
            return json.loads(msg.data)
        except ValueError:
            return None

    def _store(self, topic, value):
        if topic == "/sil/drive/emulator_state" and isinstance(value, dict):
            # SW-03: the drive emulator falls back to its internal plant without an error when Gazebo feedback
            # goes stale. The rig counts every such switch after the stack was ready.
            feedback = value.get("feedback")
            if self._plant == "gazebo" and feedback != "gazebo":
                self.plant_switches += 1
            self._plant = feedback
        with self.lock:
            self.last[topic] = value
            self.stamp[topic] = time.monotonic()
            self.counts[topic] = self.counts.get(topic, 0) + 1

    def get(self, topic):
        with self.lock:
            return self.last.get(topic)

    def age(self, topic):
        """Wall seconds since the last message on `topic`, or None."""
        with self.lock:
            stamp = self.stamp.get(topic)
        return None if stamp is None else time.monotonic() - stamp

    def count(self, topic):
        with self.lock:
            return self.counts.get(topic, 0)

    def sim_time(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def rtf(self):
        stats = self.get("/build1/rtf_stats") or {}
        return stats.get("rtf_last")

    def wait(self, condition, sim_s, what="", step=0.02):
        """Poll `condition` until it holds or `sim_s` seconds of Gazebo time pass. -> True when it held."""
        start_sim, start_wall = self.sim_time(), time.monotonic()
        wall_limit = sim_s / MIN_RTF_FOR_WALL + 30.0
        while True:
            try:
                if condition():
                    return True
            except (KeyError, TypeError, ValueError, AttributeError, IndexError):
                pass
            if self.sim_time() - start_sim > sim_s:
                return False
            if time.monotonic() - start_wall > wall_limit:
                self.log.event(f"wall limit {wall_limit:.0f} s reached while waiting for: {what}")
                return False
            time.sleep(step)

    def sleep_sim(self, seconds):
        self.wait(lambda: False, seconds)

    def measure_rtf(self, sim_s=3.0):
        """Mean real-time factor over the next `sim_s` seconds of Gazebo time, from this node's own clocks."""
        sim0, wall0 = self.sim_time(), time.monotonic()
        self.sleep_sim(sim_s)
        wall = time.monotonic() - wall0
        return (self.sim_time() - sim0) / wall if wall > 0 else 0.0

    # ------------------------------------------------------------ poses and joints
    def truth(self):
        return pose2d(self.get("/build1/ground_truth/odom"))

    def odom(self):
        return pose2d(self.get("/odom"))

    def truth_speed(self):
        twist = self.get("/build1/ground_truth/odom").twist.twist
        return math.hypot(twist.linear.x, twist.linear.y), abs(twist.angular.z)

    def joint(self, name, topic="/joint_states", field="position"):
        message = self.get(topic)
        return getattr(message, field)[list(message.name).index(name)]

    def lift_truth(self):
        """The lift position in the physics."""
        return self.joint("lift_joint", "/sil/gz/joint_states" if self.devices else "/arm/joint_states")

    def physics_joint(self, name, field="position"):
        """A joint of the plant: /sim/joint_states with devices, the simulator's own broadcaster without."""
        return self.joint(name, self.physics, field)

    def finger_mimic_error(self):
        """Largest |finger_joint2 - finger_joint1| of the two hands in the physics (SW-09)."""
        return max(abs(self.joint(f"openarm_{side}_finger_joint2", FINGERS) - self.joint(f"openarm_{side}_finger_joint1", FINGERS))
                   for side in ("left", "right"))

    def object_position(self, name):
        p = self.get(f"/build1/ground_truth/{name}/pose").pose.position
        return p.x, p.y, p.z

    def transform(self, target, source):
        """Translation and yaw of `source` in `target` from /tf, or None."""
        try:
            t = self.buffer.lookup_transform(target, source, Time()).transform
        except TransformException:
            return None
        return (t.translation.x, t.translation.y, t.translation.z), t.rotation

    # ------------------------------------------------------------ readiness
    def wait_ready(self, nav2=False, moveit=False, arms=True, lift=True, sim_s=180.0):
        """Wait until the stack answers. Raises Blocked when the launch said BLOCKED, ScenarioFailed otherwise."""
        log = self.log
        if not self.wait(lambda: self.get("/build1/rtf_stats") is not None and self.get("/build1/ground_truth/odom") is not None,
                         120.0, "Gazebo clock and ground truth"):
            raise ScenarioFailed("Gazebo did not start: no /clock or no ground truth")
        if not self.wait(lambda: self.get("/build1/run_info") is not None, 60.0, "run identity"):
            raise ScenarioFailed("no /build1/run_info: the launch did not start rtf_monitor.py")
        info = self.get("/build1/run_info")
        self.devices = bool(info.get("devices"))
        self.physics = "/sim/joint_states" if self.devices else "/arm/joint_states"
        self.log.data["run"] = dict(info, baseline_lock=BASELINE_LOCK)
        log.event(f"run profile {info.get('profile')}, step {info.get('physics_step')}, engine {info.get('physics_engine')}, "
                  f"model sha256 {info.get('model_sha256')}")
        checks = {"odometry": lambda: self.get("/odom") is not None}
        if self.devices:
            checks.update({"drive configured over CAN1": lambda: self.get("/sil/drive/status")["configured"],
                           "drive emulator fed by Gazebo": lambda: self.get("/sil/drive/emulator_state")["feedback"] == "gazebo",
                           "BMS answers over CAN2": lambda: self.get("/sil/bms/status")["answers"] > 0})
            if lift:
                checks["lift idle on CAN3"] = lambda: (self.get("/sil/lift/emulator_state")["plant"] == "gazebo"
                                                       and self.get("/sil/lift/status")["state"] == LIFT_IDLE)
        if arms:
            checks["arm joint states"] = lambda: len(self.get("/arm/joint_states").name) >= 16
        for what, check in checks.items():
            if not self.wait(check, sim_s, what, step=0.2):
                raise ScenarioFailed(f"not ready after {sim_s} s of Gazebo time: {what}")
            log.event(f"ready: {what}")
        if arms:
            self.wait_controllers("/controller_manager", ["joint_state_broadcaster", "left_arm_controller", "right_arm_controller",
                                                          "left_gripper_controller", "right_gripper_controller",
                                                          *([] if self.devices else ["lift_controller"])], sim_s)
        if nav2:
            self.wait_nav2(sim_s)
        if moveit:
            if not self.action("/move_action", "moveit_msgs.action", "MoveGroup").wait_for_server(timeout_sec=300.0):
                raise ScenarioFailed("move_group action server did not appear")
            log.event("ready: move_group")
        self.sleep_sim(1.0)
        factor = self.measure_rtf(2.0)
        log.quantity("gazebo_rtf_at_start", factor, "1", note="Gazebo time over wall time, 2 s window")
        self.plant_switches = 0
        return factor

    def wait_controllers(self, manager, names, sim_s=120.0):
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, f"{manager}/list_controllers")

        def active():
            if not client.service_is_ready():
                return False
            future = client.call_async(ListControllers.Request())
            end = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < end:
                time.sleep(0.05)
            if not future.done() or future.result() is None:
                return False
            states = {c.name: c.state for c in future.result().controller}
            return all(states.get(name) == "active" for name in names)
        if not self.wait(active, sim_s, f"controllers of {manager}", step=0.5):
            raise ScenarioFailed(f"controllers of {manager} not active: {names}")
        self.log.event(f"ready: {manager} has {', '.join(names)} active")

    def wait_nav2(self, sim_s=240.0):
        from std_srvs.srv import Trigger
        client = self.node.create_client(Trigger, "/lifecycle_manager_navigation/is_active")

        def active():
            if not client.service_is_ready():
                return False
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < end:
                time.sleep(0.05)
            return future.done() and future.result() is not None and future.result().success
        if not self.wait(active, sim_s, "Nav2 lifecycle", step=1.0):
            raise ScenarioFailed("Nav2 did not become active")
        self.log.event("ready: Nav2 lifecycle manager reports active")

    # ------------------------------------------------------------ actions
    def action(self, name, module, kind):
        if name not in self.actions:
            action_type = getattr(__import__(module, fromlist=[kind]), kind)
            self.actions[name] = ActionClient(self.node, action_type, name)
        return self.actions[name]

    def send_goal(self, name, module, kind, goal, server_wall_s=120.0):
        """Send a goal. -> goal handle, or None when the server is missing or rejects it."""
        client = self.action(name, module, kind)
        if not client.wait_for_server(timeout_sec=server_wall_s):
            self.log.event(f"action server {name} did not appear")
            return None
        future = client.send_goal_async(goal)
        end = time.monotonic() + 60.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            self.log.event(f"goal on {name} was not accepted")
            return None
        return handle

    def wait_result(self, handle, sim_s, what="action result"):
        """-> (status, result) of a goal handle, or (None, None) on a timeout."""
        future = handle.get_result_async()
        if not self.wait(future.done, sim_s, what):
            return None, None
        wrapped = future.result()
        return wrapped.status, wrapped.result

    def cancel(self, handle):
        future = handle.cancel_goal_async()
        end = time.monotonic() + 30.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        return future.done()

    @staticmethod
    def succeeded(status):
        return status == GoalStatus.STATUS_SUCCEEDED

    # ------------------------------------------------------------ commands
    def publish(self, key, message, repeat=3):
        for _ in range(repeat):
            self.pub[key].publish(message)
            time.sleep(0.05)

    def lift_permit(self, on):
        self.publish("permit", Bool(data=bool(on)))

    def lift_command(self, value):
        self.pub["lift"].publish(Int8(data=int(value)))

    def lift_go(self, stop, sim_s=40.0):
        """Command the lift to a stop with the permit already granted. -> True when it arrived.
        Raises Blocked when the CTL-017 model faulted and the real-time factor was below its Hall rate rule."""
        sim0, wall0 = self.sim_time(), time.monotonic()
        self.lift_command(stop)
        arrived = self.wait(lambda: self.get("/sil/lift/status")["active_stop"] == stop
                            and self.get("/sil/lift/status")["state"] == LIFT_IDLE, sim_s, f"lift at stop {stop}", step=0.05)
        wall = time.monotonic() - wall0
        factor = (self.sim_time() - sim0) / wall if wall > 0 else 0.0
        state = self.get("/sil/lift/emulator_state") or {}
        if not arrived and state.get("faults") and factor < LIFT_MIN_RTF:
            raise Blocked(f"the CTL-017 model raised fault bits {state.get('faults')} while Gazebo ran at real-time factor "
                          f"{factor:.2f}. Its Hall rate rule runs on the host clock and needs about {LIFT_MIN_RTF} or more "
                          "(sil-topology.md section 6). The machine is too slow for this check")
        return arrived

    def lift_move(self, stop, sim_s=60.0):
        """Move the lift to a calibrated stop the way the run profile allows. -> True when it arrived.
        With devices: permit on, CTL-017 command over CAN3, permit off. Without: the lift trajectory controller."""
        if self.devices:
            self.lift_permit(True)
            self.wait(lambda: self.get("/sil/lift/emulator_state")["permit_fresh"], 3.0, "permit fresh")
            try:
                return self.lift_go(stop, sim_s)
            finally:
                self.lift_permit(False)
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        seconds = abs(LIFT_STOP_M[stop] - self.lift_truth()) / 0.015 + 2.0      # below the 0.018 m/s limit of the joint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["lift_joint"]
        point = JointTrajectoryPoint(positions=[LIFT_STOP_M[stop]])
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9))
        goal.trajectory.points = [point]
        handle = self.send_goal("/lift_controller/follow_joint_trajectory", "control_msgs.action", "FollowJointTrajectory", goal)
        if handle is None:
            return False
        status, _ = self.wait_result(handle, seconds + sim_s, f"lift to stop {stop}")
        return self.succeeded(status)

    def follow_joint_trajectory(self, side, positions, duration_s):
        """Send one point to an arm controller. -> goal handle or None."""
        from builtin_interfaces.msg import Duration
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [f"openarm_{side}_joint{i}" for i in range(1, 8)]
        point = JointTrajectoryPoint(positions=[float(p) for p in positions])
        point.time_from_start = Duration(sec=int(duration_s), nanosec=int((duration_s % 1.0) * 1e9))
        goal.trajectory.points = [point]
        return self.send_goal(f"/{side}_arm_controller/follow_joint_trajectory", "control_msgs.action", "FollowJointTrajectory", goal)

    def gripper(self, side, position, sim_s=20.0):
        """GripperCommand to a finger joint position (rad). -> (reached_goal, stalled, final position) or None."""
        from control_msgs.action import GripperCommand
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = 3.0
        handle = self.send_goal(f"/{side}_gripper_controller/gripper_cmd", "control_msgs.action", "GripperCommand", goal)
        if handle is None:
            return None
        status, result = self.wait_result(handle, sim_s, f"{side} gripper")
        if result is None:
            return None
        return result.reached_goal, result.stalled, result.position

    def set_emulator_parameter(self, side, name, value):
        """Fault injection of the arm emulator: one parameter of /sim/<side>_arm_emulator."""
        from rcl_interfaces.msg import Parameter as ParameterMsg, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        client = self.node.create_client(SetParameters, f"/sim/{side}_arm_emulator/set_parameters")
        if not client.wait_for_service(timeout_sec=30.0):
            raise ScenarioFailed(f"/sim/{side}_arm_emulator has no parameter service")
        if isinstance(value, bool):
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
        else:
            parameter_value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value))
        future = client.call_async(SetParameters.Request(parameters=[ParameterMsg(name=name, value=parameter_value)]))
        end = time.monotonic() + 30.0
        while not future.done() and time.monotonic() < end:
            time.sleep(0.02)
        if not future.done() or not all(r.successful for r in future.result().results):
            raise ScenarioFailed(f"could not set {name} on /sim/{side}_arm_emulator")

    def base_stopped(self):
        """The provisional base-stopped rule of the firmware owner decisions of 13 Nov 2026: below 0.01 m/s and
        0.02 rad/s. The caller adds the 500 ms hold. Ground truth speeds."""
        v, w = self.truth_speed()
        return v < 0.01 and w < 0.02

    def wait_stopped(self, sim_s=10.0, hold_s=0.5):
        """-> Gazebo seconds until the base has been stopped for `hold_s`, or None."""
        start = self.sim_time()
        since = [None]

        def held():
            if self.base_stopped():
                since[0] = since[0] if since[0] is not None else self.sim_time()
                return self.sim_time() - since[0] >= hold_s
            since[0] = None
            return False
        if not self.wait(held, sim_s, "base stopped"):
            return None
        return since[0] - start

    # ------------------------------------------------------------ end
    def close(self):
        self.executor.shutdown(timeout_sec=5.0)
        self.node.destroy_node()
        if self.owns_rclpy and rclpy.ok():
            rclpy.shutdown()


def run(scenario_name, function, needs_devices=False, **ready):
    """Run one scenario function against a running stack. -> (result, reason, log path).
    result is PASS, FAIL or BLOCKED."""
    rig = Rig(scenario_name)
    log = rig.log
    result, reason = "FAIL", ""
    try:
        rig.wait_ready(**ready)
        if needs_devices and not rig.devices:
            raise ScenarioFailed("this scenario needs the device emulators: run it with profile firmware_sil or faults")
        function(rig, log)
        if rig.devices:
            profile = (rig.get("/build1/run_info") or {}).get("profile")
            if profile == "faults":
                log.observed("drive_emulator_plant_switches", rig.plant_switches, "switches",
                             note="times the drive emulator left Gazebo feedback for its internal plant model (SW-03)")
            else:
                log.quantity("drive_emulator_plant_switches", rig.plant_switches, "switches", low=0, high=0,
                             note="the drive emulator must stay on Gazebo feedback for the whole run (SW-03)")
        stats = rig.get("/build1/rtf_stats") or {}
        for key in ("rtf_mean", "rtf_min", "rtf_max"):
            log.quantity(f"gazebo_{key}_whole_run", stats.get(key), "1", note="from rtf_monitor.py since its start")
        if log.failures:
            reason = "; ".join(log.failures)
        else:
            result = "PASS"
    except Blocked as blocked:
        result, reason = "BLOCKED", str(blocked)
    except ScenarioFailed as failed:
        reason = str(failed)
    except Exception as error:  # noqa: BLE001 - the log must say what broke
        reason = f"{type(error).__name__}: {error}"
    finally:
        path = log.write(result, reason=reason)
        rig.close()
    return result, reason, path
