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
    domain = str(random.randint(101, 200))
    os.environ.update(ROS_DOMAIN_ID=domain, ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST", GZ_PARTITION=f"build1_{domain}_{os.getpid()}")
    arguments = {"profile": "firmware_sil", "headless": "true", **arguments}
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
    result, reason, path = rig.run(log_name or name, function, needs_devices=needs_devices,
                                    uses_base=name in scenarios.BASE_SCENARIOS, **ready)
    print(f"BUILD1 {log_name or name}: {result}. Log: {path}")
    if result == "BLOCKED":
        test_case.skipTest(f"BLOCKED: {reason}")
    test_case.assertEqual(result, "PASS", reason)
