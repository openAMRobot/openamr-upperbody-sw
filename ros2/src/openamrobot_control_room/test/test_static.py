# SPDX-License-Identifier: MIT
"""The read-only rule on the files of the control room: RViz configuration, viewer layout, foxglove_bridge
configuration, launch file and the node sources. No ROS needed."""
import ast
import json
import os
import re

import yaml

from conftest import PACKAGE_DIR, REPOSITORY, package_path, read
from openamrobot_control_room import readonly

EM_DASH = chr(0x2014)
SOURCES = [os.path.join(root, name) for root, _, names in os.walk(PACKAGE_DIR) for name in names
           if name.endswith(".py") and "/test" not in root.replace(PACKAGE_DIR, "")]


def test_rviz_config_has_no_write_tool_or_display():
    assert readonly.rviz_violations(read("rviz", "control_room.rviz")) == []


def test_rviz_check_catches_goal_and_point_tools():
    # The revert case: the toolbar of a default RViz configuration (2D Goal Pose, 2D Pose Estimate, Publish Point).
    config = yaml.safe_load(read("rviz", "control_room.rviz"))
    config["Visualization Manager"]["Tools"] += [{"Class": "rviz_default_plugins/SetGoal"},
                                                 {"Class": "rviz_default_plugins/SetInitialPose"},
                                                 {"Class": "rviz_default_plugins/PublishPoint"},
                                                 {"Class": "rviz_default_plugins/Interact"}]
    config["Visualization Manager"]["Displays"].append({"Class": "moveit_rviz_plugin/MotionPlanning", "Name": "MP"})
    found = readonly.rviz_violations(yaml.safe_dump(config))
    assert len(found) == 5, found


def test_rviz_config_covers_the_displays():
    config = yaml.safe_load(read("rviz", "control_room.rviz"))
    displays = config["Visualization Manager"]["Displays"]
    topics = {str(d.get("Topic", {}).get("Value") if isinstance(d.get("Topic"), dict) else d.get("Trajectory Topic"))
              for d in displays}
    for topic in ("/scan_filtered", "/amcl_pose", "/build1/ground_truth/odom", "/control_room/nav_goal", "/plan",
                  "/display_planned_path", "/baseline1/observer/image", "/head_camera/rgbd/image",
                  "/build1/ground_truth/item/pose"):
        assert topic in topics, topic


def test_layout_has_no_publish_service_or_parameter_panel():
    assert readonly.layout_violations(read("config", "lichtblick_layout.json")) == []


def test_layout_check_catches_teleop_and_publish():
    layout = json.loads(read("config", "lichtblick_layout.json"))
    layout["configById"]["Teleop!x"] = {"topic": "/cmd_vel"}
    layout["configById"]["3D!scene"]["publish"] = {"type": "pose", "poseTopic": "/goal_pose"}
    assert len(readonly.layout_violations(json.dumps(layout))) == 2


def test_layout_panels_are_all_placed():
    layout = json.loads(read("config", "lichtblick_layout.json"))
    placed = set(re.findall(r'"([A-Za-z0-9]+![A-Za-z0-9_]+)"', json.dumps(layout["layout"])))
    assert placed == set(layout["configById"])


def test_foxglove_config_offers_no_write_capability():
    config = yaml.safe_load(read("config", "foxglove_bridge.yaml"))
    assert readonly.foxglove_violations(config["control_room_foxglove_bridge"]["ros__parameters"]) == []


def test_foxglove_check_catches_the_bridge_defaults():
    defaults = {"capabilities": ["clientPublish", "parameters", "parametersSubscribe", "services", "connectionGraph", "assets"],
                "client_topic_whitelist": [".*"], "service_whitelist": [".*"], "param_whitelist": [".*"]}
    assert len(readonly.foxglove_violations(defaults)) >= 9


def test_publisher_guard():
    assert readonly.publisher_violations(["/control_room/status", "/rosout", "/parameter_events"]) == []
    for topic in ("/cmd_vel", "/sil/lift/permit", "/sil/lift/command", "/goal_pose", "/left_arm_controller/joint_trajectory",
                  "/build1/drive/left_wheel_cmd_vel", "/tf", "/control_room/cmd_vel"):
        assert readonly.publisher_violations([topic]) == [topic], topic
    try:
        readonly.guard_publisher_topic("/cmd_vel")
    except readonly.ReadOnlyViolation:
        pass
    else:
        raise AssertionError("guard_publisher_topic accepted /cmd_vel")


def _calls(path):
    tree = ast.parse(open(path, encoding="utf-8").read())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            yield f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", "")


def test_sources_create_no_client_goal_or_parameter_write():
    forbidden = {"create_client", "ActionClient", "send_goal_async", "send_goal", "call_async", "call", "set_parameters",
                 "set_parameters_atomically", "send", "send_periodic", "create_service"}
    for path in SOURCES:
        found = forbidden & set(_calls(path))
        assert not found, f"{path}: {found}"


def test_no_subscription_to_action_feedback():
    # The ROS graph (ros2 node info, rcl_action) reports a node that subscribes to an _action/feedback topic as an
    # action client of that action. The control room must not even look like one.
    for path in SOURCES:
        assert "_action/feedback" not in open(path, encoding="utf-8").read().replace("an _action/feedback", ""), path


def test_every_publisher_goes_through_the_guard():
    for path in SOURCES:
        text = open(path, encoding="utf-8").read()
        for match in re.finditer(r"create_publisher\(([^\n]*)", text):
            assert "guard_publisher_topic" in match.group(1), f"{path}: {match.group(0)}"


def test_launch_starts_only_monitoring_components():
    tree = ast.parse(read("launch", "control_room.launch.py"))
    started = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, "id", "") in ("Node", "ExecuteProcess"):
            kw = {k.arg: k.value for k in node.keywords}
            assert getattr(node.func, "id") == "Node", "the launch runs no bare process"
            package = kw["package"].value if isinstance(kw["package"], ast.Constant) else "openamrobot_control_room"
            started.add((package, kw["executable"].value))
    assert started == {("openamrobot_control_room", "control_room_monitor.py"), ("openamrobot_control_room", "vcan_monitor.py"),
                       ("rviz2", "rviz2"), ("foxglove_bridge", "foxglove_bridge")}


def test_no_em_dash_and_licence_headers():
    for root, _, names in os.walk(PACKAGE_DIR):
        for name in names:
            path = os.path.join(root, name)
            if "__pycache__" in path:
                continue     # ignored by .gitignore, as *.pyc; test_no_compiled_file_is_tracked checks git
            text = open(path, encoding="utf-8").read()
            assert EM_DASH not in text, path
            if name.endswith((".py", ".yaml", ".rviz")) or name == "CMakeLists.txt":
                assert "SPDX-License-Identifier: MIT" in "".join(text.splitlines()[:3]), path
    doc = os.path.join(REPOSITORY, "docs", "demo", "control_room.md")
    assert os.path.exists(doc) and EM_DASH not in open(doc, encoding="utf-8").read()


def test_no_compiled_file_is_tracked():
    import subprocess
    done = subprocess.run(["git", "-C", PACKAGE_DIR, "ls-files", "."], capture_output=True, text=True)
    if done.returncode != 0:
        return       # not a git checkout (an installed copy): nothing tracked to check
    assert not [f for f in done.stdout.split() if f.endswith(".pyc") or "__pycache__" in f]
