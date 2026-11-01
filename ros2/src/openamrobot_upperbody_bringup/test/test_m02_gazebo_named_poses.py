# SPDX-License-Identifier: MIT
"""M02: every named pose of both arms (and both grippers) plans and executes in Gazebo at option B's baseline configuration, through
move_group, with a final joint error below 0.01 rad. The lift is at L1000 (lift_joint at its zero, which is where joint_state_publisher
reports it and where the unactuated carriage rests on its limit): the simulator is not asked to move it (we have no authority to).

The stack is `moveit.launch.py sim:=gazebo headless:=true`: Gazebo Harmonic (server only), the base on platform-sw's DiffDrive plugin, the
arms on gz_ros2_control, the five controllers, move_group on the simulator's clock. The poses are the same sequence as M01.

THE PHYSICS ENGINE: this test passes `physics_engine:=gz-physics-bullet-featherstone-plugin` explicitly. sim.launch.py defaults to DART (the
engine platform-sw tuned the base on; decided 6 Oct), and on DART the fingers do not follow the position command at the default gain and
`finger_joint2` has no mimic constraint, so a `closed` gripper pose would not close. The arm poses do not depend on the engine; the gripper
poses do, and they are part of what this check is for (the per-hand sign, live). That is the same reason G02 passes the engine. What was
and was not compared on DART: see the gazebo package's README.

`simulation` class: it says how the model behaves in this simulator on this machine, under the Gazebo position interface (a velocity
servo), and nothing about a physical robot or its limits. The acceleration limits MoveIt plans under are engineering defaults (ADR-009).
"""
import os
import random
import sys
import unittest

os.environ["ROS_DOMAIN_ID"] = str(random.randint(101, 200))
os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
os.environ["GZ_PARTITION"] = f"m02_{os.getpid()}"

import launch  # noqa: E402
import launch_testing.actions  # noqa: E402
import pytest  # noqa: E402
import rclpy  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource  # noqa: E402

from openamrobot_upperbody_moveit.move_group_client import MoveGroupClient  # noqa: E402
from openamrobot_upperbody_moveit.named_poses import GRIPPER_JOINT, SIDES  # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from a7_support import (A7_ENABLED, ACCELERATION_SCALING, ARM_CONTROLLERS, CONTROLLERS_ACTIVE_WITHIN_S, SKIP_REASON, TOLERANCE,  # noqa: E402
                        VELOCITY_SCALING, clamp_gate, clamp_record, named_pose_plan, run_sequence, stack_alive, stack_healthy,
                        wait_for_controllers_active, write_run_sidecar)

LIFT_STOP = "L1000"            # joint_state_publisher reports lift_joint at its default zero; no ros2_control interface moves it
# The engine this check REQUIRES, passed explicitly on purpose (see the docstring); the launch's own default is DART.
ENGINE_THE_GRIPPER_POSES_NEED = "gz-physics-bullet-featherstone-plugin"
READY_WITHIN_S = 150.0


@pytest.mark.launch_test
def generate_test_description():
    if not A7_ENABLED:
        # A launch with no process at all races its own tests ("Processes under test stopped before tests completed": about one run in
        # three failed, found in P12A, and the A6 report dropped those failures because it did not list M01/M02). One process that outlives
        # the skipped test keeps the launch up until launch_testing stops it.
        return launch.LaunchDescription([launch.actions.ExecuteProcess(cmd=["sleep", "30"]), launch_testing.actions.ReadyToTest()]), {}
    moveit = os.path.join(get_package_share_directory("openamrobot_upperbody_bringup"), "launch", "moveit.launch.py")
    return launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            PythonLaunchDescriptionSource(moveit),
            launch_arguments={"sim": "gazebo", "rviz": "false", "headless": "true",
                              "physics_engine": ENGINE_THE_GRIPPER_POSES_NEED}.items()),
        launch_testing.actions.ReadyToTest(),
    ]), {}


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestNamedPosesInGazebo(unittest.TestCase):
    record = {}

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("m02_test")
        cls.client = MoveGroupClient(cls.node)

    @classmethod
    def tearDownClass(cls):
        rows = cls.record.get("poses", [])
        gate = clamp_record(cls.client)
        ok = bool(rows) and all(r["ok"] for r in rows) and gate["gate"]["passes"] and cls.record.get("controllers_active", False)
        details = {"lift_stop": LIFT_STOP, "lift_joint_position_m": 0.0, "base_plate": "bp000",
                   "lift_state_source": "joint_state_publisher's default (zero), not the physics: platform-sw's JointStatePublisher is "
                                        "scoped to the wheels and the lift has no ros2_control interface",
                   "hardware": "gz_ros2_control/GazeboSimSystem (Gazebo Harmonic, headless)",
                   "physics_engine_used": ENGINE_THE_GRIPPER_POSES_NEED, "physics_engine_passed_explicitly": True,
                   "why_this_engine": "the launch defaults to DART, on which the gripper poses do not close (fingers do not follow the "
                                      "position command at the default gain; no mimic constraint); the arm poses do not depend on it",
                   "tolerance_rad": TOLERANCE, "velocity_scaling": VELOCITY_SCALING, "acceleration_scaling": ACCELERATION_SCALING,
                   "gripper_interface": "GripperCommand via moveit_simple_controller_manager (ADR-009 option 1)",
                   "poses_run": len(rows), "poses_ok": sum(1 for r in rows if r["ok"]),
                   "requests_per_pose": 1,       # no retry: a refused request is a FAILed pose (see MoveGroupClient.move)
                   "worst_final_error_rad": max((r["final_max_joint_error_rad"] for r in rows), default=None),
                   "clamp": gate,
                   "criteria": "exactly two numbers gate the clamp part of M02: excursions beyond epsilon (must be 0) and the largest overshoot "
                               "magnitude (must be below epsilon = 1e-5 rad). Activation and episode counts are DIAGNOSTIC ONLY and gate nothing",
                   "poses": rows, **{k: v for k, v in cls.record.items() if k != "poses"}}
        write_run_sidecar("M02", f"gazebo_{LIFT_STOP}", f"named poses plan and execute in Gazebo at the baseline, lift at {LIFT_STOP}",
                          "PASS" if ok else "FAIL", details)
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_m02_1_every_named_pose_plans_and_executes_with_a_final_error_under_tolerance(self):
        self.assertTrue(self.client.wait_for_server(READY_WITHIN_S), f"move_group's /move_action did not appear within {READY_WITHIN_S:.0f} s")
        # Before the first pose: the arm controllers (and the rest of the five) must be ACTIVE. A pose sent while a controller is still
        # configuring is refused or dropped and then reads as a robot failure; that would be measuring the harness. If they never activate this
        # FAILs here, with their states, and no pose is sent.
        ok, states, waited = wait_for_controllers_active(self.node, CONTROLLERS_ACTIVE_WITHIN_S)
        self.record["controllers"] = {"required": list(ARM_CONTROLLERS), "states": states, "waited_s": waited, "timeout_s": CONTROLLERS_ACTIVE_WITHIN_S}
        self.record["controllers_active"] = ok
        self.assertTrue(ok, f"the controllers did not all reach active within {CONTROLLERS_ACTIVE_WITHIN_S:.0f} s (waited {waited} s): {states}; "
                            "no pose was sent")
        joints = [f"openarm_{side}_joint{i}" for side in SIDES for i in range(1, 8)] + [GRIPPER_JOINT[s] for s in SIDES]
        self.assertTrue(self.client.wait_for_joints(joints, 120.0), "/joint_states never listed all 16 controlled joints")
        self.record["poses"], failures, self.record["start"] = run_sequence(self.client, named_pose_plan(), result_timeout=240.0)
        self.assertFalse(failures, "\n".join(failures))
        self.assertEqual(len(self.record["poses"]), 4 * 2 + 2 * 2)

    def test_m02_2_the_clamp_stayed_inside_its_epsilon(self):
        """The two numbers (criteria in the module's sidecar): excursions beyond epsilon = 0 and the largest overshoot < 1e-5 rad. Runs after the
        pose test (unittest orders by name) and reads the same clamp, so it reports on the whole sequence."""
        gate = clamp_gate(self.client.clamp_stats())
        self.record["clamp_gate"] = gate
        self.assertTrue(gate["passes"], "; ".join(gate["reasons"]))


EXPECTED_PROCESSES = ["gazebo", "robot_state_publisher", "parameter_bridge", "create", "joint_state_publisher", "spawner", "move_group", "joint_state_clamp"]


@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestProcessesAlive(unittest.TestCase):
    def test_no_process_of_the_launch_died_while_the_tests_were_running(self, proc_info):
        stack_alive(proc_info)


@launch_testing.post_shutdown_test()
@unittest.skipUnless(A7_ENABLED, SKIP_REASON)  # tracking: openAMRobot/openamr-upperbody-sw#16
class TestShutdown(unittest.TestCase):
    def test_m02_processes_started_and_exited_cleanly(self, proc_info):
        stack_healthy(proc_info, expected=EXPECTED_PROCESSES, simulator=True, sidecar=("M02", f"gazebo_{LIFT_STOP}"))
