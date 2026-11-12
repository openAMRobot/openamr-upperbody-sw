# SPDX-License-Identifier: MIT
"""Integrated simulation smoke: robot_sim.launch.py headless, one of each node, the stack up, the arms move, and what does not yet work.

    colcon test --packages-select openamrobot_upperbody_bringup --ctest-args -R integrated_sim_smoke
    launch_test install/openamrobot_upperbody_bringup/share/openamrobot_upperbody_bringup/test/test_integrated_sim_smoke.py   (after sourcing)

Needs Gazebo Harmonic and ros_gz (apt: ros-jazzy-ros-gz, ros-jazzy-gz-ros2-control) and the platform-sw packages at the commit in
config/upstream_pins.yaml. Startup is about a minute; the whole file stays under the TIMEOUT in CMakeLists.txt.

TestStack (must pass)
  1  the six controllers are active;
  2  no node name appears twice, and the nodes that must be single (robot_state_publisher, joint_state_publisher, controller_manager,
     platform_bridge) exist once: this is the A7 finding L-01 (two stacks, two /clock, two robot_state_publishers) kept closed;
  3  /clock, /odom, /scan, /imu/data_raw (the real robot's name, D-10), /joint_states and /robot_description have one publisher each;
  4  /clock runs (messages per wall second above a floor) and the simulator's real-time factor is above RTF_FLOOR;
  5  both arms go to `raised` and back to `park` through their trajectory controllers.

TestKnownDefects (pass while the defect is there; see docs/sim_smoke/2026-11-09.md)
  6  the base, driven 1 m at 0.2 m/s, moves about 1 m in /odom and almost nothing in the simulator's own pose;
  7  the lift trajectory goal reports success and lift_joint stays at 0 in /joint_states.
Each asserts the defect as measured, so it fails the day the cause is fixed; then it is replaced by the plain assertion (see the docstrings).

Simulation evidence only. Nothing here says anything about the physical robot."""
import os
import random
import sys
import time
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"integrated_{os.getpid()}"
PARTITION = os.environ["GZ_PARTITION"]

import launch  # noqa: E402
import launch_testing  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
from ament_index_python.packages import get_package_prefix, get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402

sys.path.insert(0, os.path.join(get_package_prefix("openamrobot_upperbody_bringup"), "lib", "openamrobot_upperbody_bringup"))
sys.path.insert(0, os.path.join(get_package_prefix("openamrobot_upperbody_description"), "lib", "openamrobot_upperbody_description"))
import sim_smoke_support as support  # noqa: E402
from launch_guard import SIM_ALLOWED, assert_launch_healthy, assert_no_process_died, gz_server_shutdown_report  # noqa: E402

EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner"]
CLOCK_FLOOR_MSGS_PER_S = 50.0       # the world steps at 1 kHz; a hosted runner at 0.3x real time still delivers several hundred
RTF_FLOOR = 0.1
ARM_TOLERANCE_RAD = 0.01
DRIVE_TOLERANCE_M = 0.2             # of 1 m, on the simulator's own pose
LIFT_TOLERANCE_M = 0.01


@pytest.mark.launch_test
def generate_test_description():
    launch_file = os.path.join(get_package_share_directory("openamrobot_upperbody_bringup"), "launch", "robot_sim.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(PythonLaunchDescriptionSource(launch_file), launch_arguments={"headless": "true"}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


class Base(unittest.TestCase):
    smoke = None

    @classmethod
    def setUpClass(cls):
        if Base.smoke is None:
            Base.smoke = support.Smoke()
            Base.states = Base.smoke.controller_states(timeout_s=240.0)
            Base.smoke.wait_until(lambda: len(Base.smoke.odom) > 5 and len(Base.smoke.clock_stamps) > 5, 60.0)

    @classmethod
    def tearDownClass(cls):
        pass


class TestStack(Base):
    def test_1_controllers_active(self):
        self.assertEqual({n: Base.states.get(n) for n in support.CONTROLLERS}, {n: "active" for n in support.CONTROLLERS})

    def test_2_one_of_each_node(self):
        names, _ = Base.smoke.graph()
        duplicates = sorted({n for n in names if names.count(n) > 1})
        self.assertEqual(duplicates, [], "node names present more than once")
        for single in support.SINGLE_NODES:
            self.assertEqual(names.count(single), 1, f"{single}: {names.count(single)} in {sorted(names)}")

    def test_3_one_publisher_per_topic(self):
        _, topics = Base.smoke.graph()
        counts = {t: topics.get(t, 0) for t in support.SINGLE_PUBLISHER_TOPICS}
        self.assertEqual(counts, {t: 1 for t in support.SINGLE_PUBLISHER_TOPICS})

    def test_4_clock_runs(self):
        Base.smoke.spin_for(3.0)
        msgs_per_s, sim_per_wall = Base.smoke.clock_rate()
        self.assertGreater(msgs_per_s, CLOCK_FLOOR_MSGS_PER_S)
        self.assertGreater(support.real_time_factor(window_s=5.0), RTF_FLOOR)
        self.assertGreater(sim_per_wall, RTF_FLOOR)

    def test_5_arms_raised_and_back_to_park(self):
        for pose in ("raised", "park"):
            for row in Base.smoke.arms_to(pose):
                self.assertEqual(row["error_code"], 0, row)
                self.assertLess(row["final_error_rad"], ARM_TOLERANCE_RAD, row)


class TestKnownDefects(Base):
    """Each test states the defect as measured on 9 Nov 2026 and passes while it is there. When one fails, the defect is fixed (or changed):
    replace it with the plain assertion in its docstring and say so in docs/sim_smoke."""

    def test_6_base_moves_in_odom_but_not_in_ground_truth(self):
        """Fixed when: ground_truth_m is within DRIVE_TOLERANCE_M of 1.0."""
        row = Base.smoke.drive_forward(1.0, 0.2)
        self.assertGreater(row["odom_m"], 1.0 - DRIVE_TOLERANCE_M, row)             # the wheels turned and the DiffDrive plugin integrated them
        self.assertLess(row["ground_truth_m"], DRIVE_TOLERANCE_M, row)              # the chassis did not follow

    def test_7_lift_goal_succeeds_but_lift_joint_stays_at_zero(self):
        """Fixed when: the error is below LIFT_TOLERANCE_M."""
        code, error, _ = Base.smoke.follow("lift_controller", ["lift_joint"], [support.LIFT_STOPS_M["L1175"]], support.LIFT_SECONDS)
        self.assertEqual(code, 0)
        self.assertGreater(error, support.LIFT_STOPS_M["L1175"] - LIFT_TOLERANCE_M)


class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        assert_no_process_died(proc_info)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_processes_started_and_exited_cleanly(self, proc_info):
        assert_launch_healthy(proc_info, expected=EXPECTED_PROCESSES, allowed=SIM_ALLOWED, record=lambda records: None)

    def test_no_gz_server_of_this_run_survives_the_teardown(self):
        self.assertEqual(gz_server_shutdown_report(PARTITION)["survivors"], [])
