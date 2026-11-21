# SPDX-License-Identifier: MIT
"""Single-writer rules of the Build 1 scene, as pure functions over a snapshot of the system. No ROS import.

rclpy in Jazzy gives no publisher identity per message. Frame ownership on /tf is therefore checked in two parts:
the set of nodes that publish /tf is exactly the declared one, and no child frame is seen under two parents.
"""
import xml.etree.ElementTree as ET

KINDS = ("direct", "devices")


def kind_of(command_map, profile):
    """direct or devices, for a run profile name."""
    try:
        return command_map["profiles"][profile]
    except KeyError:
        raise ValueError(f"profile {profile!r} is not in the map: {', '.join(command_map['profiles'])}") from None


def expected_writers(entry, kind, components):
    writers = list(entry.get("writers", [])) + list(entry.get(f"writers_{kind}", []))
    for component, extra in (entry.get("writers_when") or {}).items():
        if component in components:
            writers += extra
    return sorted(writers)


def ignored(node, prefixes):
    return any(node.startswith(prefix) for prefix in prefixes)


def topic_problems(command_map, publishers, kind, components):
    """publishers: {topic: [node full name, one entry per publisher endpoint]}. -> list of problem texts."""
    problems = []
    prefixes = command_map.get("ignored_node_prefixes", [])
    for topic, entry in command_map["topics"].items():
        # A node counts once, however many publisher endpoints it holds on the topic.
        actual = sorted({node for node in publishers.get(topic, []) if not ignored(node, prefixes)})
        expected = expected_writers(entry, kind, components)
        if actual != expected:
            problems.append(f"{topic}: writers {actual}, the map says {expected}")
    return problems


def map_problems(command_map):
    """The map against its own rule: more than one writer needs an arbitration note."""
    problems = []
    for topic, entry in command_map["topics"].items():
        for kind in KINDS:
            most = expected_writers(entry, kind, ("nav2", "moveit", "follower"))
            if len(most) > 1 and not entry.get("arbitration"):
                problems.append(f"{topic}: {len(most)} writers in the map for {kind} without an arbitration note")
            if entry.get("command") and len(most) > 1 and topic != "/cmd_vel_nav":
                problems.append(f"{topic}: a command topic with {len(most)} writers")
    return problems


STATE_SUFFIXES = ("/status", "/emulator_state", "/odom", "/joint_states", "/audit", "/battery_state", "/console",
                  "/wheel_speeds", "/collision_points_marker", "_state", "/activity")


def unlisted_command_topics(command_map, publishers, patterns=("cmd_vel", "/sil/", "_cmd", "/command")):
    """Topics that look like a command, have a writer and are not in the map. A command topic must be in the map."""
    known = set(command_map["topics"])
    return sorted(topic for topic, nodes in publishers.items()
                  if nodes and topic not in known and any(p in topic for p in patterns) and not topic.endswith(STATE_SUFFIXES)
                  and "/_action/" not in topic and "transition_event" not in topic and not topic.startswith("/sil/fw/")
                  and not topic.startswith("/sil/gz/"))


def action_problems(command_map, status_publishers, kind):
    """status_publishers: {action name: [node full names publishing <action>/_action/status]}."""
    problems = []
    for action, entry in command_map["actions"].items():
        expected = [] if entry.get("only", kind) != kind else [entry["server"]]
        actual = sorted(set(status_publishers.get(action, [])))
        if actual != expected:
            problems.append(f"action {action}: servers {actual}, the map says {expected}")
    return problems


def tf_owners(command_map, kind, components):
    owners = dict(command_map["tf_frames"][kind])
    if "nav2" in components:
        owners.update(command_map["tf_frames"]["nav2"])
    return owners


def tf_problems(command_map, pairs, kind, components):
    """pairs: set of (child frame, parent frame) seen on /tf and /tf_static. -> problems."""
    problems = []
    parents = {}
    for child, parent in sorted(pairs):
        parents.setdefault(child, set()).add(parent)
    for child, seen in sorted(parents.items()):
        if len(seen) > 1:
            problems.append(f"/tf: frame {child} has {len(seen)} parents {sorted(seen)}: two writers")
    for node, frames in tf_owners(command_map, kind, components).items():
        for child, parent in frames.items():
            if parents.get(child) != {parent}:
                problems.append(f"/tf: {parent} -> {child} of {node}: seen parents {sorted(parents.get(child, []))}")
    chain = command_map["tf_chain"]
    for parent, child in zip(chain, chain[1:]):
        if parents.get(child) != {parent}:
            problems.append(f"/tf chain {' > '.join(chain)}: {child} hangs under {sorted(parents.get(child, []))}, not under {parent}")
    return problems


def joints_of_groups(command_map, groups):
    joints = []
    for name, entry in command_map["joints"].items():
        if entry["group"] in groups:
            joints += entry.get("joints", [name])
    return joints


def expected_claims(command_map, manager, joint):
    """The command interfaces of `joint` the map says `manager` has claimed: its `claimed` kind (or the
    `claimed_by_group` kind of the joint's group), plus each kind of `also_claimed` whose groups hold the joint."""
    kinds = [claimed_interface(manager, mapped_joints(command_map)[joint])]
    for kind, groups in (manager.get("also_claimed") or {}).items():
        if joint in joints_of_groups(command_map, groups):
            kinds.append(kind)
    return sorted(f"{joint}/{kind}" for kind in kinds)


def interface_problems(command_map, kind, interfaces_by_manager):
    """interfaces_by_manager: {manager name: {"joint/kind": claimed bool}} from list_hardware_interfaces."""
    problems = []
    for manager in command_map["controller_managers"][kind]:
        interfaces = interfaces_by_manager.get(manager["name"])
        if interfaces is None:
            problems.append(f"{manager['name']} did not answer list_hardware_interfaces")
            continue
        for joint in manager.get("absent_joints", []):
            present = sorted(name for name in interfaces if name.startswith(f"{joint}/"))
            if present:
                problems.append(f"{manager['name']}: {joint} must have no command interface, found {present}")
        for joint in joints_of_groups(command_map, manager["joints"]):
            claimed = sorted(name for name, is_claimed in interfaces.items() if name.startswith(f"{joint}/") and is_claimed)
            expected = expected_claims(command_map, manager, joint)
            if claimed != expected:
                problems.append(f"{manager['name']}: {joint} has claimed command interfaces {claimed}, the map says {expected}")
    return problems


def claimed_interface(manager, group):
    """The command interface the map says a controller manager claims for a joint group: `claimed`, unless
    `claimed_by_group` names another one for that group."""
    return manager.get("claimed_by_group", {}).get(group, manager["claimed"])


def mapped_joints(command_map):
    """Every joint the map gives a command path, with its group."""
    joints = {}
    for name, entry in command_map["joints"].items():
        for joint in entry.get("joints", [name]):
            if joint in joints:
                raise ValueError(f"{joint} is in the map twice")
            joints[joint] = entry["group"]
    return joints


def urdf_movable_joints(urdf_text):
    """{joint: mimic or not} of every non-fixed joint of a URDF."""
    root = ET.fromstring(urdf_text)
    return {joint.get("name"): joint.find("mimic") is not None for joint in root.findall("joint") if joint.get("type") != "fixed"}


def joint_map_problems(command_map, urdf_text):
    """The map against the model: every movable joint has exactly one entry with one path per kind of profile."""
    problems = []
    try:
        mapped = mapped_joints(command_map)
    except ValueError as error:
        return [str(error)]
    movable = urdf_movable_joints(urdf_text)
    mimics = set(command_map.get("mimic_joints", []))
    for joint, is_mimic in movable.items():
        if is_mimic:
            if joint not in mimics:
                problems.append(f"{joint} is a mimic joint of the model and is not listed under mimic_joints")
            if joint in mapped:
                problems.append(f"{joint} is a mimic joint and must have no command path")
        elif joint not in mapped:
            problems.append(f"{joint} is movable in the model and has no entry in the map")
    for joint in list(mapped) + sorted(mimics):
        if joint not in movable:
            problems.append(f"{joint} is in the map and is not a movable joint of the model")
    for name, entry in command_map["joints"].items():
        for kind in KINDS:
            path = entry.get(kind)
            if not isinstance(path, str) or not path.strip():
                problems.append(f"{name}: no command path for {kind}")
            elif not path.rstrip().endswith("> Gazebo"):
                problems.append(f"{name}: the {kind} path does not end in Gazebo")
    if command_map.get("dynamics_owner") != "gazebo":
        problems.append("dynamics_owner must be gazebo")
    for profile, kind in command_map["profiles"].items():
        if kind not in KINDS:
            problems.append(f"profile {profile}: kind {kind!r}")
    return problems


def gz_topics(command_map, kind, components):
    """(topics with exactly one publisher, topics with none) on Gazebo transport."""
    single = list(command_map["gz_command_topics"][kind])
    for component, topics in command_map.get("gz_command_topics_when", {}).items():
        if component in components:
            single += topics
    return single, list(command_map.get("gz_silent_topics", {}).get(kind, []))


def gz_problems(command_map, kind, gz_publishers, components=()):
    """gz_publishers: {gz topic: number of publishers}."""
    single, _silent = gz_topics(command_map, kind, components)
    problems = [f"gz {topic}: {gz_publishers.get(topic)} publishers, exactly 1 allowed"
                for topic in single if gz_publishers.get(topic) != 1]
    problems += [f"gz {topic}: {gz_publishers.get(topic)} publishers, none allowed in this profile"
                 for topic in command_map.get("gz_silent_topics", {}).get(kind, []) if gz_publishers.get(topic)]
    return problems
