# SPDX-License-Identifier: MIT
"""Read a rosbag2 recording for the report, and write the small synthetic Baseline 1 bag the tests and CI use when no
recorded bag is at hand. Needs rosbag2_py and the message packages (a sourced ROS 2 Jazzy).

The synthetic bag is a fixture: invented numbers on the real topic names and types. A report made from it shows that
the generator works, nothing about the robot.
"""
import json
import math

TOPICS = None    # every topic of the bag is read; extract() keeps what it knows


def read(uri, topics=TOPICS):
    """Yield (topic, message, receive time ns) in bag order."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topics is not None and topic not in topics:
            continue
        if topic not in classes:
            try:
                classes[topic] = get_message(types[topic])
            except (AttributeError, ModuleNotFoundError, ValueError):
                classes[topic] = None      # a type this machine does not have: counted, not decoded
        if classes[topic] is None:
            continue
        yield topic, deserialize_message(data, classes[topic]), t_ns


# Phases of the synthetic mission, Gazebo seconds: name, start, end.
SYNTHETIC_PHASES = (("spawn", 0.0, 2.0), ("nav_to_station", 2.0, 30.0), ("stop_authorize", 30.0, 34.0),
                    ("lift_to_target", 34.0, 50.0), ("arm_approach", 50.0, 62.0), ("grasp", 62.0, 66.0),
                    ("lift_object", 66.0, 72.0), ("retreat", 72.0, 80.0))
SYNTHETIC_STATION = (2.0, 1.4, math.pi / 2)


def synthetic_mission():
    """A mission JSON in the shape baseline1.py writes, for the synthetic bag."""
    phases = []
    for name, start, end in SYNTHETIC_PHASES:
        phases.append({"phase": name, "ran": True, "failures": [], "exception": None, "sim_start": start, "sim_end": end,
                       "note": "", "quantities": [{"name": f"{name}_synthetic", "value": 0.0, "unit": "", "low": 0.0,
                                                    "high": 1.0, "ok": True, "kind": "measured", "note": "fixture"}]})
    phases[-1]["failures"] = [{"text": "retreat_along_heading = -0.2 m, allowed -0.35 to -0.25", "class": "CONTROL"}]
    return {"scenario": "baseline1_mission", "result": "FAIL", "reason": "synthetic fixture", "evidence_class": "simulation",
            "baseline1": {"seed": 20261117, "station": "work_shelf", "fixture": True}, "phases": phases, "trace": [],
            "failures": [phases[-1]["failures"][0]["text"]]}


def synthetic_samples(duration_s=80.0, rate_hz=10.0):
    """The synthetic run as plain values, one dict per step: the base drives to the station, the lift goes up and down,
    the left arm moves, the item rises 0.08 m and the base backs away. Invented numbers on the Baseline 1 time line."""
    gx, gy, gyaw = SYNTHETIC_STATION
    for k in range(int(duration_s * rate_hz) + 1):
        t = k / rate_hz
        f = min(1.0, max(0.0, (t - 2.0) / 28.0))                  # navigation progress
        x, y, yaw = gx * f + 0.01 * math.sin(t), gy * f, gyaw * f
        if t > 72.0:                                                # retreat
            y -= 0.03 * (t - 72.0)
        lift = 0.175 * min(1.0, max(0.0, (t - 34.0) / 6.0)) if t < 44.0 else 0.175 * max(0.0, 1.0 - (t - 44.0) / 6.0)
        arm = min(1.0, max(0.0, (t - 50.0) / 12.0)) if t < 72.0 else max(0.0, 1.0 - (t - 72.0) / 4.0)
        yield {"t": t, "k": k, "truth": (x, y, yaw),
               "amcl": (x + 0.02 * math.cos(0.3 * t), y + 0.015, yaw + 0.01) if k % 5 == 0 else None,
               "lift_js": lift + 0.001, "lift": lift,
               "joints": [0.2 * i * arm * (1 if s == "left" else 0.1) for s in ("left", "right") for i in range(1, 8)],
               "rtf": round(0.6 + 0.1 * math.sin(t / 7.0), 3) if k % int(rate_hz) == 0 else None,
               "item": (gx, gy + 0.5, 0.80 + (0.08 if 66.0 <= t <= 72.0 else 0.0))}


JOINT_NAMES = ["lift_joint"] + [f"openarm_{s}_joint{i}" for s in ("left", "right") for i in range(1, 8)]


def synthetic_messages(types):
    """(topic, message, receive time ns) of the synthetic run. `types` maps the message names (Clock, Odometry,
    PoseWithCovarianceStamped, JointState, String, PoseStamped, Time) to classes: the ROS ones for a bag, or plain
    attribute objects for the tests without ROS."""
    for s in synthetic_samples():
        t = s["t"]
        stamp = types["Time"](sec=int(t), nanosec=int(round((t % 1.0) * 1e9)) % 1000000000)
        t_ns = int(t * 1e9) + 1
        yield "/clock", types["Clock"](clock=stamp), t_ns
        odom = types["Odometry"]()
        odom.header.stamp, odom.header.frame_id = stamp, "world"
        x, y, yaw = s["truth"]
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        yield "/build1/ground_truth/odom", odom, t_ns
        if s["amcl"]:
            amcl = types["PoseWithCovarianceStamped"]()
            amcl.header.stamp, amcl.header.frame_id = stamp, "map"
            ax, ay, ayaw = s["amcl"]
            amcl.pose.pose.position.x, amcl.pose.pose.position.y = ax, ay
            amcl.pose.pose.orientation.z, amcl.pose.pose.orientation.w = math.sin(ayaw / 2), math.cos(ayaw / 2)
            yield "/amcl_pose", amcl, t_ns
        joints = types["JointState"]()
        joints.header.stamp, joints.name, joints.position = stamp, list(JOINT_NAMES), [s["lift_js"]] + s["joints"]
        yield "/joint_states", joints, t_ns
        physics = types["JointState"]()
        physics.header.stamp, physics.name, physics.position = stamp, ["lift_joint"], [s["lift"]]
        yield "/sil/gz/joint_states", physics, t_ns
        if s["rtf"] is not None:
            yield "/build1/rtf_stats", types["String"](data=json.dumps({"rtf_last": s["rtf"]})), t_ns
        item = types["PoseStamped"]()
        item.header.stamp = stamp
        item.pose.position.x, item.pose.position.y, item.pose.position.z = s["item"]
        yield "/build1/ground_truth/item/pose", item, t_ns


TOPIC_TYPES = {"/clock": "rosgraph_msgs/msg/Clock", "/build1/ground_truth/odom": "nav_msgs/msg/Odometry",
               "/amcl_pose": "geometry_msgs/msg/PoseWithCovarianceStamped", "/joint_states": "sensor_msgs/msg/JointState",
               "/sil/gz/joint_states": "sensor_msgs/msg/JointState", "/build1/rtf_stats": "std_msgs/msg/String",
               "/build1/ground_truth/item/pose": "geometry_msgs/msg/PoseStamped"}


class Plain:
    """An attribute bag that grows the nested fields a ROS message has (msg.header.stamp, msg.pose.pose.position)."""

    def __init__(self, **fields):
        self.__dict__.update(fields)

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        value = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}.get(name, Plain())    # the ROS defaults (Quaternion w 1)
        setattr(self, name, value)
        return value


def plain_types():
    """Stand-ins for ros_types() without ROS: the same fields, no serialization."""
    return {name: Plain for name in ("Time", "Clock", "Odometry", "PoseWithCovarianceStamped", "JointState", "String",
                                     "PoseStamped")}


def ros_types():
    from builtin_interfaces.msg import Time
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    return {"Time": Time, "Clock": Clock, "Odometry": Odometry, "PoseWithCovarianceStamped": PoseWithCovarianceStamped,
            "JointState": JointState, "String": String, "PoseStamped": PoseStamped}


def write_synthetic(uri, storage_id="mcap"):
    """Write the synthetic bag at `uri`. Returns the number of messages written."""
    import rosbag2_py
    from rclpy.serialization import serialize_message
    writer = rosbag2_py.SequentialWriter()
    try:
        writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id=storage_id), rosbag2_py.ConverterOptions("cdr", "cdr"))
    except RuntimeError:
        writer = rosbag2_py.SequentialWriter()
        writer.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    for i, (name, kind) in enumerate(TOPIC_TYPES.items()):
        try:     # Jazzy's TopicMetadata takes an id first; older ones do not
            metadata = rosbag2_py.TopicMetadata(id=i, name=name, type=kind, serialization_format="cdr")
        except TypeError:
            metadata = rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr")
        writer.create_topic(metadata)
    count = 0
    for topic, msg, t_ns in synthetic_messages(ros_types()):
        writer.write(topic, serialize_message(msg), t_ns)
        count += 1
    del writer
    return count


def synthetic_candump(lines_per_bus=200, start=1763400000.0):
    """candump -L lines per bus for the synthetic report: a steady rate on each bus."""
    out = {}
    for b, bus in enumerate(("vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r")):
        rate = 20.0 * (b + 1)
        out[bus] = [f"({start + i / rate:.6f}) {bus} {0x181 + b:03X}#0011223344556677" for i in range(lines_per_bus)]
    return out
