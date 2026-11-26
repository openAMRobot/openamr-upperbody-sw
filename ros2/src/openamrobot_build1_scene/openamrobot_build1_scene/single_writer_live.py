# SPDX-License-Identifier: MIT
"""Takes the snapshot single_writer.py judges, from a running system. Needs rclpy."""
import shutil
import subprocess
import threading
import time

from openamrobot_build1_scene import single_writer


def full_name(info):
    return info.node_namespace.rstrip("/") + "/" + info.node_name


def publishers_by_topic(node):
    return {topic: sorted(full_name(info) for info in node.get_publishers_info_by_topic(topic))
            for topic, _types in node.get_topic_names_and_types()}


def settled_graph(node, settle_s, limit_s=180.0, spin=None):
    """The publisher map once it has not changed for `settle_s` wall seconds."""
    spin = spin or (lambda: time.sleep(0.5))
    last, since, end = None, time.monotonic(), time.monotonic() + limit_s
    while time.monotonic() < end:
        spin()
        graph = publishers_by_topic(node)
        if graph != last:
            last, since = graph, time.monotonic()
        elif time.monotonic() - since >= settle_s:
            break
    return last or {}


def hardware_interfaces(node, manager, spin_until):
    from controller_manager_msgs.srv import ListHardwareInterfaces
    client = node.create_client(ListHardwareInterfaces, f"{manager}/list_hardware_interfaces")
    if not client.wait_for_service(timeout_sec=30.0):
        return None
    future = client.call_async(ListHardwareInterfaces.Request())
    spin_until(future, 30.0)
    if not future.done() or future.result() is None:
        return None
    return {interface.name: bool(interface.is_claimed) for interface in future.result().command_interfaces}


def gz_publisher_counts(topics):
    """{gz topic: number of publishers} from `gz topic -i`, or {} when gz is not there."""
    if shutil.which("gz") is None:
        return {}
    counts = {}
    for topic in topics:
        done = subprocess.run(["gz", "topic", "-i", "-t", topic], capture_output=True, text=True, timeout=60)
        text = done.stdout + done.stderr
        section = text.split("Subscribers")[0] if "Subscribers" in text else text
        counts[topic] = sum(1 for line in section.splitlines() if "tcp://" in line)
    return counts


def release(node, subscriptions, spinning, lock=None, stop=None):
    """End the /tf recording. When another thread spins `node` (rig.Rig's MultiThreadedExecutor, scn09 with
    spinning=True), destroying a subscription there kills that executor thread (rclpy InvalidHandle, "cannot use
    Destroyable because destruction was requested": run 44848867373, job 136675191420), after which the rig receives
    nothing. Then the subscriptions stay until the node is destroyed and only the recording stops (`stop` gets a
    marker under `lock`). Without another spinner they are destroyed as before."""
    if stop is not None:
        if lock is not None:
            with lock:
                stop.append(True)
        else:
            stop.append(True)
    if spinning:
        return
    for subscription in subscriptions:
        node.destroy_subscription(subscription)


def check(node, command_map, profile, components, settle_s=20.0, spinning=False):
    """-> {"problems": [...], "facts": [...], counts}. `spinning`: another thread already spins `node`."""
    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from tf2_msgs.msg import TFMessage
    kind = single_writer.kind_of(command_map, profile)
    pairs, lock = set(), threading.Lock()
    stopped = []

    def on_tf(message):
        with lock:
            if stopped:
                return
            for transform in message.transforms:
                pairs.add((transform.child_frame_id, transform.header.frame_id))
    subscriptions = [node.create_subscription(TFMessage, "/tf", on_tf, 100),
                     node.create_subscription(TFMessage, "/tf_static", on_tf,
                                              QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def spin():
        if spinning:
            time.sleep(0.5)
        else:
            end = time.monotonic() + 0.5
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.1)

    def spin_until(future, seconds):
        end = time.monotonic() + seconds
        while not future.done() and time.monotonic() < end:
            if spinning:
                time.sleep(0.05)
            else:
                rclpy.spin_once(node, timeout_sec=0.1)

    graph = settled_graph(node, settle_s, spin=spin)
    prefixes = command_map.get("ignored_node_prefixes", [])
    filtered = {topic: [n for n in nodes if not single_writer.ignored(n, prefixes)] for topic, nodes in graph.items()}
    problems = single_writer.map_problems(command_map)
    problems += single_writer.topic_problems(command_map, graph, kind, components)
    for topic in single_writer.unlisted_command_topics(command_map, filtered):
        problems.append(f"{topic}: looks like a command topic, has writers {filtered[topic]} and is not in the map")
    status = {action: filtered.get(f"{action}/_action/status", []) for action in command_map["actions"]}
    problems += single_writer.action_problems(command_map, status, kind)
    with lock:
        seen = set(pairs)
    problems += single_writer.tf_problems(command_map, seen, kind, components)
    interfaces = {manager["name"]: hardware_interfaces(node, manager["name"], spin_until)
                  for manager in command_map["controller_managers"][kind]}
    problems += single_writer.interface_problems(command_map, kind, interfaces)
    single, silent = single_writer.gz_topics(command_map, kind, components)
    gz = gz_publisher_counts(single + silent)
    problems += single_writer.gz_problems(command_map, kind, gz, components) if gz else ["gz is not installed: the Gazebo command topics were not checked"]
    release(node, subscriptions, spinning, lock=lock, stop=stopped)
    facts = [f"{topic}: {filtered.get(topic, [])}" for topic in command_map["topics"]]
    facts += [f"action {action}: {status[action]}" for action in command_map["actions"]]
    facts += [f"gz {topic}: {count} publishers" for topic, count in gz.items()]
    for name, found in interfaces.items():
        if found is not None:
            facts.append(f"claimed in {name}: {sorted(interface for interface, claimed in found.items() if claimed)}")
    return {"problems": problems, "facts": facts, "topics_checked": len(command_map["topics"]),
            "actions_checked": len(command_map["actions"]), "frames_seen": len(seen)}
