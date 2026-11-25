# SPDX-License-Identifier: MIT
"""What every scenario launch test shares: the launch description, the isolation and the mapping of a scenario result
to a test result. PASS passes, FAIL fails, BLOCKED skips with a text that starts with BLOCKED and is counted apart:
BLOCKED is not a pass, and the CI job does not pass with one.
"""
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

DEVICE_PROFILES = ("firmware_sil", "faults")
STATE = {"blocked": None}


def description(**arguments):
    """The launch description of one scenario test: build1.launch.py, headless, in its own ROS domain and Gazebo
    partition. With a device profile and no virtual CAN nothing is launched and the test reports BLOCKED."""
    import launch
    import launch_testing.actions
    from ament_index_python.packages import get_package_share_directory
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    # Evidence for upperbody #133: processes of an earlier launch that are still alive when this one starts.
    import subprocess
    try:
        left = subprocess.run(["pgrep", "-a", "-f", "gz sim|ros2_control_node|move_group|controller_server|parameter_bridge"],
                              capture_output=True, text=True, timeout=5).stdout.strip().splitlines()
    except Exception as error:  # noqa: BLE001 - evidence only
        left = [f"pgrep failed: {error}"]
    print(f"BUILD1 STARTUP survivors of an earlier launch: {len(left)}" + "".join("\n  BUILD1 STARTUP survivor: " + x[:160] for x in left), flush=True)
    domain = str(random.randint(101, 200))
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", GZ_PARTITION=f"build1_{domain}_{os.getpid()}")
    arguments = {"profile": "firmware_sil", "headless": "true", **arguments}
    arguments["rviz"] = "true"  # SCRATCH (rc.1 RViz gate, not for merge): the workflow provides an Xvfb display
    # The planning scene's pose source of a whole test run (planning_scene.py): map unless the environment asks for the
    # simulation oracle. An explicit argument of the test wins.
    if os.environ.get("BUILD1_PLANNING_POSE_SOURCE"):
        arguments.setdefault("planning_pose_source", os.environ["BUILD1_PLANNING_POSE_SOURCE"])
    if os.environ.get("BUILD1_OBJECT_POSE_SOURCE"):
        arguments.setdefault("object_pose_source", os.environ["BUILD1_OBJECT_POSE_SOURCE"])
    if arguments["profile"] in DEVICE_PROFILES:
        from vcan_support import blocked_reason
        STATE["blocked"] = blocked_reason()
        arguments.setdefault("create_vcan", "false")
    if STATE["blocked"]:
        return launch.LaunchDescription([launch_testing.actions.ReadyToTest()])
    return launch.LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("openamrobot_build1_scene"), "launch", "build1.launch.py")),
            launch_arguments=arguments.items()),
        launch_testing.actions.ReadyToTest()])


def run(test_case, name, log_name=None):
    """Run scenario `name` of scenarios.SCENARIOS inside a unittest.TestCase."""
    if STATE["blocked"]:
        test_case.skipTest(STATE["blocked"])
    from openamrobot_build1_scene import rig, scenarios
    function, needs_devices, ready = scenarios.SCENARIOS[name]
    os.environ.setdefault("BUILD1_LOG_DIR", os.path.join(os.getcwd(), "build1_logs"))
    # SCRATCH (rc.1 RViz gate, not for merge): rviz2 with the shipped configuration and a topic and TF check beside it
    import subprocess, sys, tempfile
    check_file = tempfile.NamedTemporaryFile("w+", prefix="rviz_check_", suffix=".log", delete=False)
    checker = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "rviz_check.py")],
                               stdout=check_file, stderr=subprocess.STDOUT)
    try:
        result, reason, path = rig.run(log_name or name, function, needs_devices=needs_devices,
                                        uses_base=name in scenarios.BASE_SCENARIOS, **ready)
    finally:
        checker.terminate()
        try:
            checker.wait(timeout=40)
        except Exception:  # noqa: BLE001 - evidence only
            checker.kill()
        check_file.seek(0)
        for line in check_file:
            print(line.rstrip(), flush=True)
    print(f"BUILD1 {log_name or name}: {result}. Log: {path}")
    if result == "BLOCKED":
        test_case.skipTest(f"BLOCKED: {reason}")
    if result == "XFAIL":
        # An approved, tracked expected failure (scenarios.PARK_XFAIL): counted as XFAIL by count_build1_tests.py,
        # never as passed.
        test_case.skipTest(reason)
    test_case.assertEqual(result, "PASS", reason)
