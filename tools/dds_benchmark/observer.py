#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""DDS benchmark observer (Proposed, docs/integration/dds_benchmark.md). Diagnostics only: never a check of Baseline 1.

Started by .github/workflows/dds-benchmark.yml before the mission, in the ROS domain the mission's harness will draw
(build1_harness.description: random.randint(101, 200) after random.seed(BASELINE1_SEED)) and with the same discovery
range (LOCALHOST), so that it creates its DDS participant first and takes participant index 0. A participant with a low
index is announced to by every other participant, so this observer sees the whole graph even where two late
participants cannot see each other. It publishes nothing (no rosout, no parameter services, no command topics).

Per run it writes into --out:
  observer_summary.json   QoS of the key topics (every publisher and subscriber the graph shows), received counts,
                          stamp-sequence estimates of the published counts, latency (stamp to receipt, simulation time
                          and wall time), RTF over time, the participant index of every process (from its bound
                          RTPS discovery port), the late view (ros2 topic info --verbose from a fresh participant)
  processes.csv           CPU and memory per process every SAMPLE_S wall seconds (psutil), and the totals
Simulation evidence only; the observer is one more participant and one more subscriber of every key topic.
"""
import argparse
import bisect
import csv
import json
import os
import re
import signal
import statistics
import struct
import subprocess
import sys
import threading
import time

# RTPS well-known ports (DDSI-RTPS 2.x section 9.6.1.1), used by Fast DDS and Cyclone DDS alike:
# metatraffic (discovery) unicast port = PB + DG * domain + d1 + PG * participant_index
PB, DG, PG, D1, D3 = 7400, 250, 2, 10, 11
MAX_INDEX = 120
SAMPLE_S = 5.0
GRAPH_S = 30.0
LATE_VIEW_AFTER_S = 120.0
LATE_SPIN_S = 15.0

HEADER_TYPES = {"nav_msgs/msg/Odometry", "sensor_msgs/msg/LaserScan", "sensor_msgs/msg/JointState", "sensor_msgs/msg/Image",
                "geometry_msgs/msg/PoseWithCovarianceStamped", "geometry_msgs/msg/PoseStamped",
                "apriltag_msgs/msg/AprilTagDetectionArray", "sensor_msgs/msg/CameraInfo"}


# ----------------------------------------------------------------------------------------------- pure functions (tested)
def harness_domain(seed):
    """The ROS domain build1_harness.description draws for this seed (its first random draw after random.seed)."""
    import random
    rng = random.Random(int(seed))
    return rng.randint(101, 200)


def discovery_port(domain, index):
    return PB + DG * int(domain) + D1 + PG * int(index)


def participant_index(port, domain):
    """Participant index of a metatraffic unicast port in this domain, or None (user traffic, other domain, other use)."""
    offset = int(port) - (PB + DG * int(domain) + D1)
    if offset < 0 or offset % PG != 0 or offset // PG >= MAX_INDEX:
        return None
    return offset // PG


SS_LINE = re.compile(r":(\d+)\s+\S+\s+users:\((.*)\)\s*$")
SS_PID = re.compile(r'\("([^"]*)",pid=(\d+),fd=\d+\)')


def parse_ss(text, domain):
    """`ss -H -u -a -n -p` text to {index: [pid, ...]} for the discovery ports of `domain`."""
    found = {}
    for line in text.splitlines():
        match = SS_LINE.search(line)
        if not match:
            continue
        index = participant_index(int(match.group(1)), domain)
        if index is None:
            continue
        for _, pid in SS_PID.findall(match.group(2)):
            found.setdefault(index, [])
            if int(pid) not in found[index]:
                found[index].append(int(pid))
    return found


def process_name(cmdline):
    """A readable name of a process: its ROS node name, its Python script or its executable."""
    if not cmdline:
        return "?"
    for arg in cmdline:
        if arg.startswith("__node:="):
            return arg.split(":=", 1)[1]
    base = os.path.basename(cmdline[0])
    if base.startswith("python") and len(cmdline) > 1:
        for arg in cmdline[1:]:
            if not arg.startswith("-"):
                return os.path.basename(arg)
    if base == "ros2" and len(cmdline) > 2:
        return "ros2 " + " ".join(cmdline[1:3])
    return base


def subscriber_reliability(publisher_reliabilities):
    """RELIABLE when every publisher offers reliable (so the observer asks what the stack's reliable readers ask),
    BEST_EFFORT otherwise (a reliable reader does not match a best-effort writer)."""
    values = [str(r).upper() for r in publisher_reliabilities]
    if values and all("RELIABLE" in v and "BEST" not in v for v in values):
        return "RELIABLE"
    return "BEST_EFFORT"


def delivery_estimate(stamps):
    """Received count against the count the publisher's stamp sequence implies: unique stamps, nominal period = median
    positive stamp step, expected = span / period + 1. An estimate of the publisher-side count for a periodic publisher
    of simulation-time stamps; not a publisher-side counter."""
    unique = sorted(set(round(s, 6) for s in stamps))
    if len(unique) < 3:
        return {"received": len(stamps), "unique_stamps": len(unique), "expected": None, "ratio": None, "period_s": None}
    steps = [b - a for a, b in zip(unique, unique[1:]) if b > a]
    period = statistics.median(steps)
    expected = int(round((unique[-1] - unique[0]) / period)) + 1 if period > 0 else len(unique)
    return {"received": len(stamps), "unique_stamps": len(unique), "expected": expected,
            "ratio": round(len(unique) / expected, 4) if expected else None, "period_s": round(period, 6),
            "largest_gap_s": round(max(steps), 6), "first_stamp": unique[0], "last_stamp": unique[-1]}


def percentiles(values):
    if not values:
        return None
    ordered = sorted(values)

    def at(q):
        return round(ordered[min(len(ordered) - 1, int(q * (len(ordered) - 1) + 0.5))], 6)
    return {"n": len(ordered), "p50": at(0.5), "p95": at(0.95), "max": round(ordered[-1], 6)}


def wall_at(clock_sim, clock_wall, sim):
    """Wall time at which the observer first received a /clock at or after simulation time `sim`."""
    i = bisect.bisect_left(clock_sim, sim)
    return clock_wall[i] if i < len(clock_wall) else None


def rtf_windows(clock_sim, clock_wall, window_s=30.0):
    rows, start = [], 0
    for i in range(1, len(clock_wall)):
        if clock_wall[i] - clock_wall[start] >= window_s:
            rows.append({"wall": round(clock_wall[i], 1), "sim": round(clock_sim[i], 3),
                         "rtf": round((clock_sim[i] - clock_sim[start]) / (clock_wall[i] - clock_wall[start]), 4)})
            start = i
    return rows


def header_stamp(raw):
    """sec, nanosec of the std_msgs/Header that starts a CDR-serialized message (after the 4-byte encapsulation)."""
    if len(raw) < 12:
        return None
    endian = "<" if raw[1] in (1, 3) else ">"
    sec, nsec = struct.unpack_from(endian + "iI", raw, 4)
    return sec + nsec * 1e-9


# --------------------------------------------------------------------------------------------------------- ROS observer
class Observer:
    def __init__(self, args):
        import rclpy
        from rclpy.node import Node
        self.rclpy = rclpy
        self.args = args
        self.topics = [t for t in args.topics.split(",") if t]
        rclpy.init()
        self.node = Node("dds_benchmark_observer", enable_rosout=False, start_parameter_services=False)
        self.start_wall = time.time()
        self.lock = threading.Lock()
        self.clock_sim, self.clock_wall = [], []
        self.subs, self.records = {}, {}
        self.qos_seen, self.endpoint_history, self.participants = {}, [], []
        self.late_view, self.late_view_done = {}, False
        self.first_odom_wall = None
        self.domains_seen = {}
        self.stop = threading.Event()
        try:
            self.rmw = rclpy.utilities.get_rmw_implementation_identifier()
        except Exception:  # noqa: BLE001 - informational
            self.rmw = os.environ.get("RMW_IMPLEMENTATION")
        self.node.create_timer(2.0, self.discover)
        self.node.create_timer(GRAPH_S, self.graph_snapshot)
        self.sampler = threading.Thread(target=self.sample_processes, daemon=True)
        self.csv = open(os.path.join(args.out, "processes.csv"), "w", newline="")
        self.writer = csv.writer(self.csv)
        self.writer.writerow(["wall_s", "pid", "name", "cpu_percent", "rss_mb"])

    # -- subscriptions
    def discover(self):
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from rosidl_runtime_py.utilities import get_message
        types = dict(self.node.get_topic_names_and_types())
        for topic in ["/clock"] + self.topics:
            if topic in self.subs or topic not in types:
                continue
            publishers = self.node.get_publishers_info_by_topic(topic)
            if not publishers:
                continue
            type_name = types[topic][0]
            reliability = subscriber_reliability([p.qos_profile.reliability.name for p in publishers])
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=200, durability=DurabilityPolicy.VOLATILE,
                             reliability=ReliabilityPolicy.RELIABLE if reliability == "RELIABLE" else ReliabilityPolicy.BEST_EFFORT)
            msg_type = get_message(type_name)
            raw = type_name in HEADER_TYPES or topic == "/clock"
            self.records[topic] = {"type": type_name, "subscriber_reliability": reliability, "rx": []}
            self.subs[topic] = self.node.create_subscription(msg_type, topic, lambda m, t=topic: self.on_message(t, m), qos, raw=raw)

    def on_message(self, topic, msg):
        wall = time.time()
        if topic == "/clock":
            sec, nsec = struct.unpack_from("<iI" if msg[1] in (1, 3) else ">iI", msg, 4)
            sim = sec + nsec * 1e-9
            if not self.clock_sim or sim >= self.clock_sim[-1]:
                self.clock_sim.append(sim)
                self.clock_wall.append(wall)
            return
        sim_now = self.clock_sim[-1] if self.clock_sim else None
        rx = self.records[topic]["rx"]
        if topic == "/tf":
            for tr in msg.transforms:
                stamp = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                rx.append((f"{tr.header.frame_id}>{tr.child_frame_id}", stamp, sim_now, wall))
            return
        stamp = header_stamp(msg) if isinstance(msg, (bytes, bytearray)) else None
        rx.append(("", stamp, sim_now, wall))
        if topic == "/odom" and self.first_odom_wall is None:
            self.first_odom_wall = wall

    # -- graph
    def graph_snapshot(self):
        row = {"wall": round(time.time() - self.start_wall, 1), "nodes": len(self.node.get_node_names_and_namespaces()),
               "topics": {}}
        for topic in self.topics:
            pubs = self.node.get_publishers_info_by_topic(topic)
            subs = self.node.get_subscriptions_info_by_topic(topic)
            row["topics"][topic] = [len(pubs), len(subs)]
            # Kept from the last snapshot with publishers: at shutdown the publishers leave before the observer does.
            if pubs or (subs and topic not in self.qos_seen):
                self.qos_seen[topic] = {"publishers": [endpoint(e) for e in pubs], "subscriptions": [endpoint(e) for e in subs],
                                        "wall": row["wall"]}
        self.endpoint_history.append(row)
        self.participant_snapshot()
        if (not self.late_view_done and self.first_odom_wall is not None
                and time.time() - self.first_odom_wall > LATE_VIEW_AFTER_S):
            self.late_view_done = True
            threading.Thread(target=self.take_late_view, daemon=True).start()

    def participant_snapshot(self):
        done = subprocess.run(["ss", "-H", "-u", "-a", "-n", "-p"], capture_output=True, text=True, check=False)
        found = parse_ss(done.stdout, self.args.domain)
        rows = {}
        for index, pids in sorted(found.items()):
            rows[index] = [{"pid": pid, "name": name_of(pid)} for pid in pids]
        self.participants.append({"wall": round(time.time() - self.start_wall, 1), "count": len(rows),
                                  "max_index": max(rows) if rows else None, "indices": rows})
        for pid in os.listdir("/proc"):
            if pid.isdigit():
                try:
                    env = open(f"/proc/{pid}/environ", "rb").read().split(b"\0")
                except OSError:
                    continue
                for item in env:
                    if item.startswith(b"ROS_DOMAIN_ID="):
                        value = item.split(b"=", 1)[1].decode()
                        self.domains_seen[value] = self.domains_seen.get(value, 0) + 1

    def take_late_view(self):
        """`ros2 topic info --verbose` from a fresh participant, created late like the bag recorder: what a late
        participant sees, next to what this index-0 observer sees."""
        for topic in self.topics:
            command = ["ros2", "topic", "info", "--verbose", "--no-daemon", "--spin-time", str(LATE_SPIN_S), topic]
            done = subprocess.run(command, capture_output=True, text=True, check=False, timeout=120)
            self.late_view[topic] = {"rc": done.returncode, "publisher_nodes": re.findall(r"Node name: (\S+)", done.stdout.split("Subscription count")[0]),
                                     "subscription_nodes": re.findall(r"Node name: (\S+)", done.stdout.split("Subscription count")[-1]),
                                     "text": done.stdout[-6000:], "stderr": done.stderr[-1000:]}
            own = self.qos_seen.get(topic, {})
            if done.returncode != 0:
                self.late_view[topic]["note"] = "ros2 topic info exited non-zero"
            self.late_view[topic]["observer_publisher_nodes"] = [e["node"] for e in own.get("publishers", [])]
            self.late_view[topic]["observer_subscription_nodes"] = [e["node"] for e in own.get("subscriptions", [])]
        # The same question from a fresh participant that spins LATE_SPIN_S for discovery (the CLI may spin less).
        done = subprocess.run([sys.executable, os.path.abspath(__file__), "late", "--topics", ",".join(self.topics)],
                              capture_output=True, text=True, check=False, timeout=180)
        try:
            self.late_view["_late_probe"] = json.loads(done.stdout.strip().splitlines()[-1])
        except (ValueError, IndexError):
            self.late_view["_late_probe"] = {"rc": done.returncode, "stderr": done.stderr[-1000:]}

    # -- processes
    def sample_processes(self):
        import psutil
        procs = {}
        user = psutil.Process().username()
        psutil.cpu_percent(None)
        while not self.stop.is_set():
            now = round(time.time() - self.start_wall, 1)
            for proc in psutil.process_iter(["pid", "username"]):
                if proc.info["username"] == user and proc.pid not in procs:
                    procs[proc.pid] = proc
                    try:
                        proc.cpu_percent(None)
                    except psutil.Error:
                        pass
            rows = []
            for pid, proc in list(procs.items()):
                try:
                    with proc.oneshot():
                        rows.append([now, pid, process_name(proc.cmdline()), proc.cpu_percent(None),
                                     round(proc.memory_info().rss / 2**20, 1)])
                except psutil.Error:
                    procs.pop(pid, None)
            memory = psutil.virtual_memory()
            rows.append([now, 0, "TOTAL", psutil.cpu_percent(None) * psutil.cpu_count(), round(memory.used / 2**20, 1)])
            with self.lock:
                self.writer.writerows(rows)
                self.csv.flush()
            self.stop.wait(SAMPLE_S)

    # -- summary
    def summary(self):
        topics = {}
        for topic, record in self.records.items():
            rx = record["rx"]
            keys = sorted({k for k, *_ in rx})
            per_key = {}
            for key in keys:
                items = [r for r in rx if r[0] == key and r[1] is not None]
                sim_lat = [r[2] - r[1] for r in items if r[2] is not None]
                wall_lat = []
                for _, stamp, _, wall in items:
                    at = wall_at(self.clock_sim, self.clock_wall, stamp)
                    if at is not None:
                        wall_lat.append(wall - at)
                per_key[key or "all"] = {**delivery_estimate([r[1] for r in items]),
                                         "latency_sim_s": percentiles(sim_lat), "latency_wall_s": percentiles(wall_lat)}
            topics[topic] = {"type": record["type"], "subscriber_reliability": record["subscriber_reliability"],
                             "received": len(rx), "streams": per_key}
        return {"rmw": self.rmw, "domain": self.args.domain, "camera_topic": self.args.camera, "domains_seen_in_proc_environ": self.domains_seen,
                "env": {k: os.environ.get(k) for k in ("RMW_IMPLEMENTATION", "ROS_AUTOMATIC_DISCOVERY_RANGE", "FASTDDS_BUILTIN_TRANSPORTS",
                                                       "FASTRTPS_DEFAULT_PROFILES_FILE", "CYCLONEDDS_URI", "BENCH_SETTING", "BENCH_DISCOVERY_CAP")},
                "wall_s": round(time.time() - self.start_wall, 1), "clock_samples": len(self.clock_sim),
                "sim_span_s": round(self.clock_sim[-1] - self.clock_sim[0], 3) if self.clock_sim else None,
                "rtf": rtf_windows(self.clock_sim, self.clock_wall), "qos": self.qos_seen, "topics": topics,
                "endpoint_history": self.endpoint_history, "participants": self.participants, "late_view": self.late_view}

    def run(self):
        self.sampler.start()
        signal.signal(signal.SIGTERM, lambda *_: self.stop.set())
        signal.signal(signal.SIGINT, lambda *_: self.stop.set())
        last_write = time.time()
        while not self.stop.is_set():
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            if time.time() - last_write > 120.0:
                self.write()
                last_write = time.time()
        self.write()
        self.csv.close()

    def write(self):
        path = os.path.join(self.args.out, "observer_summary.json")
        with open(path + ".tmp", "w") as f:
            json.dump(self.summary(), f, indent=1, default=str)
        os.replace(path + ".tmp", path)


def endpoint(info):
    q = info.qos_profile
    return {"node": f"{info.node_namespace.rstrip('/')}/{info.node_name}", "type": info.topic_type,
            "reliability": q.reliability.name, "durability": q.durability.name, "history": q.history.name, "depth": q.depth,
            "liveliness": q.liveliness.name}


def name_of(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            return process_name([a.decode(errors="replace") for a in f.read().split(b"\0") if a])
    except OSError:
        return "?"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    domain = sub.add_parser("domain", help="print the ROS domain build1_harness draws for a seed")
    domain.add_argument("seed")
    ports = sub.add_parser("peers-xml", help="Fast DDS XML profile: unicast initial peers for participant indices 0..n-1")
    ports.add_argument("domain", type=int)
    ports.add_argument("--count", type=int, default=MAX_INDEX)
    late = sub.add_parser("late", help="a fresh participant: publishers and subscribers it sees after LATE_SPIN_S")
    late.add_argument("--topics", required=True)
    run = sub.add_parser("run", help="observe the running stack until SIGTERM")
    run.add_argument("--domain", type=int, required=True)
    run.add_argument("--out", required=True)
    run.add_argument("--topics", required=True, help="comma-separated key topics")
    run.add_argument("--camera", default="", help="the key topic that is the camera image")
    args = parser.parse_args(argv)
    if args.command == "domain":
        print(harness_domain(args.seed))
        return 0
    if args.command == "peers-xml":
        print(peers_xml(args.domain, args.count))
        return 0
    if args.command == "late":
        print(json.dumps(late_probe([t for t in args.topics.split(",") if t])))
        return 0
    os.makedirs(args.out, exist_ok=True)
    Observer(args).run()
    return 0


def late_probe(topics):
    import rclpy
    from rclpy.node import Node
    rclpy.init()
    node = Node("dds_benchmark_late_probe", enable_rosout=False, start_parameter_services=False)
    end = time.time() + LATE_SPIN_S
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    result = {"nodes": len(node.get_node_names_and_namespaces()), "topics": {}}
    for topic in topics:
        result["topics"][topic] = {"publisher_nodes": [endpoint(e)["node"] for e in node.get_publishers_info_by_topic(topic)],
                                   "subscription_nodes": [endpoint(e)["node"] for e in node.get_subscriptions_info_by_topic(topic)]}
    node.destroy_node()
    rclpy.shutdown()
    return result


def peers_xml(domain, count=MAX_INDEX):
    """Fast DDS 2.x profile: the default participant gets 127.0.0.1 initial peers with explicit discovery ports for
    participant indices 0..count-1. rmw_fastrtps (Jazzy, LOCALHOST range) keeps a profile's initial peers and only adds
    127.0.0.1 without a port, which Fast DDS expands to indices 0..31 (maxInitialPeersRange 32)."""
    locators = "\n".join(f"          <locator><udpv4><address>127.0.0.1</address><port>{discovery_port(domain, i)}</port></udpv4></locator>"
                         for i in range(count))
    return f"""<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="dds_benchmark_raised_cap" is_default_profile="true">
      <rtps>
        <builtin>
          <initialPeersList>
{locators}
          </initialPeersList>
        </builtin>
      </rtps>
    </participant>
  </profiles>
</dds>"""


if __name__ == "__main__":
    sys.exit(main())
