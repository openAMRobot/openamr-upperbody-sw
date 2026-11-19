# SPDX-License-Identifier: MIT
"""REPLAY mode of the control room: plays a recorded run (rosbag2 bags plus the mission JSON) on its own simulation
time, under /control_room/replay/ only, and labels everything it publishes REPLAY.

  control_room_replay.py --bag BUILD1_LOG_DIR/bag [--bag CONTROL_ROOM_BAG] --mission BUILD1_LOG_DIR/<mission>.json
      [--rate 1.0] [--start SIM_S] [--end SIM_S] [--loop]

Published (all under /control_room/replay/, so a replay never looks like the live robot and never reaches a node of a
running simulation that listens on the original names):
  clock                       rosgraph_msgs/Clock: the recorded simulation time (RViz and Lichtblick follow it)
  <recorded topic>            the recorded message, unchanged, on /control_room/replay/<original name>
  status                      std_msgs/String JSON, 2 Hz host time: {"mode": "REPLAY", "sim_time", "phase", "verdict",
                              "phases", "bags", "not_replayed"} with the phase looked up in the mission JSON at the
                              current recorded simulation time
  mission_phase               std_msgs/String "REPLAY <phase> (<verdict>)"
  label                       visualization_msgs/MarkerArray: a REPLAY text above the recorded ground-truth base pose
Command topics of the recording (/cmd_vel*, controller commands, lift command and permit, action topics) are never
replayed: readonly.is_command_topic decides, and they are listed in the status. The replay holds no command authority.
Pacing: recorded simulation seconds per host second = --rate. A REPLAY of a recording is not a live result.
"""
import bisect
import heapq
import json
import time

from openamrobot_control_room import readonly

PREFIX = "/control_room/replay"
LATCHED = ("/tf_static", "/robot_description", "/build1/run_info")


def replay_topic(topic):
    return PREFIX + topic


def replayable(topic):
    """A recorded topic is replayed unless it carries a command (or is a recording of the replay itself)."""
    return not readonly.is_command_topic(topic) and not topic.startswith(PREFIX + "/") and topic != "/clock"


class Phases:
    """The mission JSON's phases on the simulation time line."""

    def __init__(self, mission):
        from openamrobot_control_room.report import _verdict
        rows = [r for r in (mission or {}).get("phases", []) if r.get("sim_start") is not None]
        self.rows = sorted(rows, key=lambda r: r["sim_start"])
        self.starts = [r["sim_start"] for r in self.rows]
        self.verdicts = {r["phase"]: _verdict(r) for r in (mission or {}).get("phases", [])}
        self.result = (mission or {}).get("result")
        self.scenario = (mission or {}).get("scenario")

    def at(self, t):
        """(phase, verdict) at simulation time t, or (None, None) outside every phase."""
        if t is None:
            return None, None
        i = bisect.bisect_right(self.starts, t) - 1
        if i < 0:
            return None, None
        row = self.rows[i]
        if row.get("sim_end") is not None and t > row["sim_end"]:
            return None, None
        return row["phase"], self.verdicts.get(row["phase"])

    def table(self):
        return [{"phase": r["phase"], "verdict": self.verdicts.get(r["phase"]), "sim_start": r.get("sim_start"),
                 "sim_end": r.get("sim_end")} for r in self.rows]


def open_reader(uri):
    import rosbag2_py
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=uri, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    return reader


def merged(uris):
    """(t_ns, topic, serialized data, type name) of every message of every bag, in time order."""
    readers = [open_reader(uri) for uri in uris]
    types = {}
    for reader in readers:
        types.update({t.name: t.type for t in reader.get_all_topics_and_types()})

    def stream(reader):
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            yield t_ns, topic, data
    for t_ns, topic, data in heapq.merge(*(stream(r) for r in readers), key=lambda item: item[0]):
        yield t_ns, topic, data, types[topic]


class Replay:
    def __init__(self, node, bags, mission, rate=1.0, start=None, end=None):
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from visualization_msgs.msg import MarkerArray
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        self.node, self.bags, self.rate, self.start, self.end = node, list(bags), float(rate), start, end
        self.phases = Phases(mission)
        self.String, self.Clock, self.MarkerArray = String, Clock, MarkerArray
        self.latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pubs, self.classes, self.not_replayed = {}, {}, set()
        self.clock_pub = node.create_publisher(Clock, readonly.guard_publisher_topic(replay_topic("/clock")), 10)
        self.status_pub = node.create_publisher(String, readonly.guard_publisher_topic(replay_topic("/status")), self.latched)
        self.phase_pub = node.create_publisher(String, readonly.guard_publisher_topic(replay_topic("/mission_phase")), self.latched)
        self.label_pub = node.create_publisher(MarkerArray, readonly.guard_publisher_topic(replay_topic("/label")), 10)
        self.sim_time, self.truth, self.messages, self.finished = None, None, 0, False
        self.last_phase = object()

    def publisher(self, topic, type_name):
        if topic not in self.pubs:
            from rosidl_runtime_py.utilities import get_message
            try:
                kind = get_message(type_name)
            except (AttributeError, ModuleNotFoundError, ValueError):
                kind = None
            self.classes[topic] = kind
            qos = self.latched if topic in LATCHED else 10
            name = readonly.guard_publisher_topic(replay_topic(topic))
            self.pubs[topic] = None if kind is None else self.node.create_publisher(kind, readonly.guard_publisher_topic(name), qos)
        return self.pubs[topic], self.classes[topic]

    def status(self):
        phase, verdict = self.phases.at(self.sim_time)
        return {"mode": "REPLAY", "label": "REPLAY of a recorded run: not live, not the robot",
                "sim_time": self.sim_time, "phase": phase, "verdict": verdict, "mission": self.phases.scenario,
                "mission_result": self.phases.result, "phases": self.phases.table(), "bags": self.bags,
                "messages_replayed": self.messages, "not_replayed": sorted(self.not_replayed), "rate": self.rate,
                "finished": self.finished}

    def publish_status(self):
        status = self.status()
        self.status_pub.publish(self.String(data=json.dumps(status, sort_keys=True)))
        if status["phase"] != self.last_phase:
            self.last_phase = status["phase"]
            self.phase_pub.publish(self.String(data=f"REPLAY {status['phase'] or 'between phases'} ({status['verdict'] or '-'})"))
        self.publish_label(status)

    def publish_label(self, status):
        from visualization_msgs.msg import Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.ns, marker.id, marker.type, marker.action = "replay", 0, Marker.TEXT_VIEW_FACING, Marker.ADD
        x, y = (self.truth or (0.0, 0.0))[:2]
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, 1.9
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.15
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.55, 0.0, 1.0
        t = "-" if self.sim_time is None else f"{self.sim_time:.1f} s"
        marker.text = f"REPLAY  {status['phase'] or '-'}  {status['verdict'] or ''}  t={t}"
        self.label_pub.publish(self.MarkerArray(markers=[marker]))

    def run(self, spin_once, steady=time.monotonic, sleep=time.sleep):
        """Play every bag once. `spin_once()` lets the node's timers run between messages."""
        from rclpy.serialization import deserialize_message
        wall0 = sim0 = None
        for t_ns, topic, data, type_name in merged(self.bags):
            t = t_ns * 1e-9
            if self.start is not None and t < self.start and topic not in LATCHED:
                continue
            if self.end is not None and t > self.end:
                break
            if wall0 is None:
                wall0, sim0 = steady(), t
            delay = (t - sim0) / self.rate - (steady() - wall0)
            while delay > 0:
                spin_once(min(delay, 0.05))
                delay = (t - sim0) / self.rate - (steady() - wall0)
            if topic == "/clock":
                kind = self.Clock
                msg = deserialize_message(data, kind)
                self.sim_time = msg.clock.sec + msg.clock.nanosec * 1e-9
                self.clock_pub.publish(msg)
                continue
            if not replayable(topic):
                self.not_replayed.add(topic)
                continue
            pub, kind = self.publisher(topic, type_name)
            if pub is None:
                self.not_replayed.add(topic)
                continue
            msg = deserialize_message(data, kind)
            if topic == "/build1/ground_truth/odom":
                self.truth = (msg.pose.pose.position.x, msg.pose.pose.position.y)
            pub.publish(msg)
            self.messages += 1
        self.finished = True
        self.publish_status()


def main(args=None):
    import argparse
    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.executors import ExternalShutdownException
    from openamrobot_control_room.monitor import make_node
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--bag", action="append", required=True, help="a rosbag2 directory; give it again for more bags")
    parser.add_argument("--mission", help="the mission JSON of the same run (phases and verdicts)")
    parser.add_argument("--rate", type=float, default=1.0, help="recorded simulation seconds per host second")
    parser.add_argument("--start", type=float, help="first simulation second to play")
    parser.add_argument("--end", type=float, help="last simulation second to play")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--hold", type=float, default=0.0, help="host seconds to keep publishing the status after the end")
    known, ros_args = parser.parse_known_args(args)
    mission = None
    if known.mission:
        with open(known.mission, encoding="utf-8") as f:
            mission = json.load(f)
    rclpy.init(args=ros_args)
    node = make_node("control_room_replay")
    replay = Replay(node, known.bag, mission, known.rate, known.start, known.end)
    node.create_timer(0.5, replay.publish_status, clock=Clock(clock_type=ClockType.STEADY_TIME))
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while True:
            replay.run(lambda timeout: executor.spin_once(timeout_sec=timeout))
            if not known.loop:
                break
            replay.finished = False
        end = time.monotonic() + known.hold
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):     # Ctrl-C or the launch's SIGINT: a normal stop
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
