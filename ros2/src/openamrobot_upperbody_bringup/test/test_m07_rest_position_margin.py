"""M07: at rest in Gazebo, no joint sits within a margin of either of its limits (P17 task 2).

Why this is a check of its own. In option B's body the right finger rests ON its open limit (-4.7e-6 .. +2.7e-8 rad); the old body rested 6.0e-5
rad inside it. A joint that rests on a limit straddles it: solver noise carries it across, the joint-state clamp fires thousands of times a run
(the count scatters 2-3x on an unchanged tree, which is why that count no longer gates anything), and every downstream number moves with it.
That is a modelling defect whether or not anything notices, so it is FAIL, not WARN. A version of this check that passes on the right finger
is wrong.

What it measures. Nothing is commanded: the simulator is started (headless, Bullet-Featherstone like G02 and M02, so the fingers behave), the
five controllers are waited for until active, the model settles, and the RAW joint states the physics produced (`/arm/joint_states`, the
joint_state_broadcaster's own topic: not the clamped copy move_group sees, not joint_state_publisher's merge) are read for a window of
simulated time. For every joint with limits the result keeps the lowest and highest reading and the distance of each from the nearer limit
(negative = past it). A joint FAILs when either extreme is closer than the margin.

The margin is a datum, `joint_rest_margin_rad` in datums.yaml, with its source and status there: not a literal here. The test reads it from the
installed description package and records the value it used.

What it does not cover, said so in the sidecar and never silently skipped: `lift_joint` (not under ros2_control; its state is
joint_state_publisher's default zero, which is its lower stop by construction, so there is no physics rest position to read) and the wheels
(continuous, no limits).

`simulation` class: how this model rests in this simulator on this machine. It says nothing about a physical robot or its limits.
"""
import os
import random
import sys
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"m07_{os.getpid()}"

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
import yaml  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a7_support import A7_ENABLED, SKIP_REASON, stack_alive, stack_healthy, write_run_sidecar  # noqa: E402

from openamrobot_upperbody_moveit.named_poses import urdf_joint_limits  # noqa: E402

ENGINE = "gz-physics-bullet-featherstone-plugin"       # passed explicitly, as G02 and M02 do: DART does not move the fingers
CONTROLLERS = ("joint_state_broadcaster", "left_arm_controller", "right_arm_controller", "left_gripper_controller", "right_gripper_controller")
ACTIVE_WITHIN_S = 120.0       # wall s for the controllers (startup of a headless server is about 10-25 s; M02 allows 150 for the whole stack)
SETTLE_SIM_S = 6.0            # simulated s between "controllers active" and the first reading: the spawn drop and the controllers' first hold
WINDOW_SIM_S = 6.0            # simulated s of readings the extremes are taken over
MIN_MESSAGES = 50             # fewer readings than this is not a measurement of a rest (the window did not run)
DATUM = "joint_rest_margin_rad"
EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner"]
NOT_MEASURED = {
    "lift_joint": "not under ros2_control: its state is joint_state_publisher's default zero (= its lower stop L1000 by construction), "
                          "not a physics reading, so there is no rest position of the simulator to compare with its limits",
}


def margin_datum():
    """(value, the datum's own record) from the installed datums.yaml. A missing or non-positive margin is an error of the check, never a pass."""
    share = get_package_share_directory("openamrobot_upperbody_description")
    with open(os.path.join(share, "config", "datums.yaml")) as f:
        doc = yaml.safe_load(f)
    record = doc["datums"][DATUM]
    value = record["value"]
    assert isinstance(value, (int, float)) and value > 0, f"datum {DATUM} must be a positive number, got {value!r}"
    return float(value), {k: record.get(k) for k in ("value", "source", "status", "ref", "owner")}, doc["revision"]


def joint_limits_with_names():
    import xacro
    path = os.path.join(get_package_share_directory("openamrobot_upperbody_description"), "urdf", "combined_robot.xacro")
    return urdf_joint_limits(xacro.process_file(path, mappings={"hardware_profile": "gazebo"}).toxml())


def rest_rows(readings, limits, margin):
    """[{joint, min, max, lower, upper, to_lower, to_upper, nearest, distance, within_margin}] for every joint with limits that was read.
    `distance` is the smaller of (lowest reading - lower) and (upper - highest reading); negative means the joint was read past a limit."""
    rows = []
    for joint in sorted(readings):
        if joint not in limits:
            continue
        values = readings[joint]
        low, high = min(values), max(values)
        lower, upper = limits[joint]
        to_lower, to_upper = low - lower, upper - high
        nearest = "lower" if to_lower <= to_upper else "upper"
        distance = min(to_lower, to_upper)
        rows.append({"joint": joint, "readings": len(values), "min_rad": low, "max_rad": high, "lower_limit_rad": lower, "upper_limit_rad": upper,
                     "distance_to_lower_rad": to_lower, "distance_to_upper_rad": to_upper, "nearest_limit": nearest,
                     "distance_to_nearest_limit_rad": distance, "within_margin": bool(distance < margin)})
    return rows


@pytest.mark.launch_test
def generate_test_description():
    if not A7_ENABLED:
        return launch.LaunchDescription([launch.actions.ExecuteProcess(cmd=["sleep", "30"]), launch_testing.actions.ReadyToTest()]), {}
    sim = os.path.join(get_package_share_directory("openamrobot_upperbody_gazebo"), "launch", "sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                                launch_arguments={"headless": "true", "physics_engine": ENGINE}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)
class TestRestPositionMargin(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m07_test", parameter_overrides=[Parameter("use_sim_time", value=True)])
        cls.started = time.monotonic()

    @classmethod
    def tearDownClass(cls):
        rows = cls.record.get("joints", [])
        measured = bool(rows) and not cls.record.get("error")
        failing = [r["joint"] for r in rows if r["within_margin"]]
        result = "PASS" if measured and not failing else "FAIL"
        write_run_sidecar("M07", "rest_position_margin", "no joint rests within the margin of either of its limits", result,
                          {"physics_engine_used": ENGINE, "physics_engine_passed_explicitly": True, "reading_topic": "/arm/joint_states",
                           "window_sim_s": WINDOW_SIM_S, "settle_sim_s": SETTLE_SIM_S, "failing_joints": failing,
                           "not_measured": NOT_MEASURED, **cls.record})
        cls.node.destroy_node()
        rclpy.shutdown()

    def _spin_until(self, predicate, wall_seconds):
        deadline = time.monotonic() + wall_seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def _wait_for_controllers(self):
        """FAIL (not hang, not skip) if the five controllers are not all active within ACTIVE_WITHIN_S."""
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        self.assertTrue(client.wait_for_service(timeout_sec=ACTIVE_WITHIN_S), "no controller_manager within the budget")
        states = {}
        while time.monotonic() < self.started + ACTIVE_WITHIN_S:
            future = client.call_async(ListControllers.Request())
            self._spin_until(future.done, 2.0)
            states = {c.name: c.state for c in future.result().controller} if future.done() and future.result() else {}
            if all(states.get(name) == "active" for name in CONTROLLERS):
                return states
            time.sleep(0.25)
        self.fail(f"controllers not all active within {ACTIVE_WITHIN_S:.0f} s: {states}")

    def test_m07_1_no_joint_rests_within_the_margin_of_a_limit(self):
        margin, record, revision = margin_datum()
        limits = joint_limits_with_names()
        self.record.update({"margin_rad": margin, "margin_datum": record, "datums_revision": revision})
        try:
            self.record["controllers"] = self._wait_for_controllers()
        except AssertionError as error:
            self.record["error"] = str(error)
            raise
        readings, count = {}, [0]
        window = {"on": False}

        def on_state(msg):
            if not window["on"]:
                return
            count[0] += 1
            for name, position in zip(msg.name, msg.position):
                readings.setdefault(name, []).append(position)

        self.node.create_subscription(JointState, "/arm/joint_states", on_state, QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE))
        clock = self.node.get_clock()
        # simulated time: the clock only moves once /clock arrives, so wait for it to start
        self.assertTrue(self._spin_until(lambda: clock.now().nanoseconds > 0, 30.0), "no /clock: the simulated time never started")
        t0 = clock.now().nanoseconds
        self.assertTrue(self._spin_until(lambda: (clock.now().nanoseconds - t0) * 1e-9 >= SETTLE_SIM_S, 120.0), "the settle period did not elapse")
        window["on"] = True
        t1 = clock.now().nanoseconds
        self.assertTrue(self._spin_until(lambda: (clock.now().nanoseconds - t1) * 1e-9 >= WINDOW_SIM_S, 120.0), "the reading window did not elapse")
        window["on"] = False
        self.record["messages_in_window"] = count[0]
        rows = rest_rows(readings, limits, margin)
        self.record["joints"] = rows
        self.record["joints_read"] = sorted(readings)
        self.record["joints_with_limits_read"] = len(rows)
        self.assertGreaterEqual(count[0], MIN_MESSAGES, f"only {count[0]} joint-state messages in a {WINDOW_SIM_S} s window: not a measurement of a rest")
        self.assertTrue(rows, f"no joint of /arm/joint_states has limits in the URDF (read: {sorted(readings)})")
        failing = [r for r in rows if r["within_margin"]]
        self.assertFalse(failing, "joints resting within %.3g rad of a limit: %s" % (margin, "; ".join(
            f"{r['joint']} min {r['min_rad']:+.3e} max {r['max_rad']:+.3e} vs [{r['lower_limit_rad']:+.4f}, {r['upper_limit_rad']:+.4f}] "
            f"(nearest {r['nearest_limit']}, {r['distance_to_nearest_limit_rad']:+.3e})" for r in failing)))


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)
class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        stack_alive(proc_info)


@launch_testing.post_shutdown_test()
@unittest.skipUnless(A7_ENABLED, SKIP_REASON)
class TestShutdown(unittest.TestCase):
    def test_m07_processes_started_and_exited_cleanly(self, proc_info):
        stack_healthy(proc_info, expected=EXPECTED_PROCESSES, simulator=True, sidecar=("M07", "rest_position_margin"))
