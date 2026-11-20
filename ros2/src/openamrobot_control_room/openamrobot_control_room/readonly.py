# SPDX-License-Identifier: MIT
"""The read-only rule of the control room, as data and as checks. No ROS import.

The control room watches the Digital Twin. It holds no command authority: it publishes nothing but its own
/control_room/ summaries, sends no action goal, calls no service and writes no parameter. The command arbiter of
the stack stays the only path to the robot. The checks here are applied to the nodes' ROS graph in the tests, to the
RViz configuration, to the viewer layout and to the foxglove_bridge configuration.
"""
import json
import re

import yaml

CONTROL_ROOM_PREFIX = "/control_room/"
# Every node publishes these two by itself (rclpy); they carry no command.
ROS_INFRASTRUCTURE_TOPICS = ("/rosout", "/parameter_events")

# Topics that move, power or authorize something in the Build 1 stack (build1.launch.py, rig.py, openamrobot_sil,
# ros2_control, Nav2, MoveIt) and the RViz and viewer tool topics. The control room never publishes on any of them;
# the tests check it from the ROS graph, which is stronger than this list: any topic outside /control_room/ fails.
COMMAND_TOPIC_PATTERNS = (
    r"^/cmd_vel.*", r".*/cmd_vel$", r"^/sil/lift/(command|permit|permit_silent|gz_cmd_vel)$", r"^/sil/drive/gz_cmd_vel$",
    r"^/sil/stm32/can1_silent$", r"^/sil/[a-z_]+/inject$", r"^/sil/fw/command$", r"^/build1/drive/.*_cmd_vel$",
    r"^/build1/fingers/.*", r"^/build1/planning_scene/exclude$", r".*_controller/(joint_trajectory|commands)$",
    r"^/goal_pose$", r"^/initialpose$", r"^/clicked_point$", r"^/move_base_simple/goal$", r".*/_action/.*",
    r"^/planning_scene$", r"^/attached_collision_object$", r"^/collision_object$", r"^/trajectory_execution_event$",
)
_COMMAND = [re.compile(p) for p in COMMAND_TOPIC_PATTERNS]


def is_command_topic(topic):
    return any(p.match(topic) for p in _COMMAND)


def publisher_violations(topics):
    """Published topics that a control room node may not have: anything outside /control_room/ and the two
    infrastructure topics. A command topic is always a violation, even under the prefix."""
    return sorted(t for t in topics
                  if is_command_topic(t) or not (t.startswith(CONTROL_ROOM_PREFIX) or t in ROS_INFRASTRUCTURE_TOPICS))


class ReadOnlyViolation(RuntimeError):
    pass


def guard_publisher_topic(topic):
    """Raise before a node creates a publisher outside the control room namespace."""
    violations = publisher_violations([topic])
    if violations:
        raise ReadOnlyViolation(f"the control room may not publish on {topic}")
    return topic


# ---------------------------------------------------------------- RViz

# Tools that only move the view or read the scene. SetGoal, SetInitialPose and PublishPoint publish; Interact sends
# interactive-marker feedback (MoveIt's planning markers), so it is left out too.
RVIZ_ALLOWED_TOOLS = ("rviz_default_plugins/MoveCamera", "rviz_default_plugins/Select", "rviz_default_plugins/FocusCamera",
                      "rviz_default_plugins/Measure")
# Displays with a write path: MotionPlanning has the Plan and Execute panel, InteractiveMarkers sends feedback.
RVIZ_FORBIDDEN_DISPLAYS = ("moveit_rviz_plugin/MotionPlanning", "rviz_default_plugins/InteractiveMarkers")


def rviz_violations(text):
    config = yaml.safe_load(text)
    manager = config.get("Visualization Manager", {})
    out = [f"tool {t.get('Class')}" for t in manager.get("Tools", []) if t.get("Class") not in RVIZ_ALLOWED_TOOLS]

    def walk(displays):
        for display in displays or []:
            if display.get("Class") in RVIZ_FORBIDDEN_DISPLAYS:
                out.append(f"display {display.get('Class')} ({display.get('Name')})")
            walk(display.get("Displays"))
    walk(manager.get("Displays"))
    for panel in config.get("Panels", []):
        if "MotionPlanning" in str(panel.get("Class", "")):
            out.append(f"panel {panel.get('Class')}")
    return out


# ---------------------------------------------------------------- viewer layout (Lichtblick / Foxglove layout JSON)

# Panels that publish, call services or set parameters.
LAYOUT_FORBIDDEN_PANELS = ("Teleop", "Publish", "CallService", "Parameters", "ServiceCall", "Joystick")


def layout_violations(text):
    layout = json.loads(text)
    out = []
    for panel_id, config in layout.get("configById", {}).items():
        kind = panel_id.split("!")[0]
        if kind in LAYOUT_FORBIDDEN_PANELS:
            out.append(f"panel {panel_id}")
        if isinstance(config, dict) and config.get("publish"):
            out.append(f"panel {panel_id} has a publish tool configured")
    if layout.get("userNodes") or layout.get("userScripts"):
        out.append("user scripts are not part of the control room layout")
    return out


# ---------------------------------------------------------------- foxglove_bridge

FOXGLOVE_WRITE_CAPABILITIES = ("clientPublish", "parameters", "parametersSubscribe", "services", "assets")
NEVER_MATCH = "(?!)"


def foxglove_violations(parameters):
    """`parameters`: the ros__parameters of the bridge node."""
    out = [f"capability {c}" for c in parameters.get("capabilities", FOXGLOVE_WRITE_CAPABILITIES)
           if c in FOXGLOVE_WRITE_CAPABILITIES]
    for key in ("client_topic_whitelist", "service_whitelist", "param_whitelist", "asset_uri_allowlist"):
        if parameters.get(key) != [NEVER_MATCH]:
            out.append(f"{key} is not [{NEVER_MATCH!r}]")
    for key in ("remote_access", "publish_client_count", "sysinfo"):
        if parameters.get(key) is not False:
            out.append(f"{key} is not false")
    return out
