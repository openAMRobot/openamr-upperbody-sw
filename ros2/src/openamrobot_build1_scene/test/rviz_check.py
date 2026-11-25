#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""SCRATCH (rc.1 RViz gate, not for merge): headless check of rviz/build1.rviz against the running combined twin.

The scratch harness starts the scene with rviz:=true (the documented path, with the MoveIt parameters) under Xvfb.
This node counts the messages on every topic a display names, records which of them the rviz node itself subscribed
to and whether that node stays present, and checks that /tf and /tf_static form one tree under the fixed frame. Prints one line: RVIZCHECK {json}.
It does not look at pixels. A person still has to look at the screen."""
import json
import os
import signal
import sys
import time

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage

CONFIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "rviz", "build1.rviz")
STOP = False


def displays(config):
    out = []

    def walk(items):
        for d in items or []:
            topics = []
            t = d.get("Topic")
            if isinstance(t, dict) and t.get("Value"):
                topics.append(t["Value"])
            elif isinstance(t, str) and t:
                topics.append(t)
            dt = d.get("Description Topic")
            if isinstance(dt, dict) and dt.get("Value"):
                topics.append(dt["Value"])
            if d.get("Planning Scene Topic"):
                topics.append(d["Planning Scene Topic"])
            out.append({"name": d.get("Name"), "class": d.get("Class"), "enabled": bool(d.get("Enabled")), "topics": topics})
            walk(d.get("Displays"))
    walk(config["Visualization Manager"]["Displays"])
    return out


class Check(Node):
    def __init__(self, topics):
        super().__init__("rviz_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.topics = {t: {"type": None, "publishers": 0, "messages": 0, "rviz_subscribed": False} for t in topics}
        self.parent = {}
        latched = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 200)
        self.create_subscription(TFMessage, "/tf_static", self.on_tf, latched)

    def on_tf(self, msg):
        for t in msg.transforms:
            self.parent[t.child_frame_id.lstrip("/")] = t.header.frame_id.lstrip("/")

    def poll(self):
        for name, rec in self.topics.items():
            infos = self.get_publishers_info_by_topic(name)
            rec["publishers"] = max(rec["publishers"], len(infos))
            for s in self.get_subscriptions_info_by_topic(name):
                if "rviz" in s.node_name:
                    rec["rviz_subscribed"] = True
            if rec["type"] is None and infos:
                rec["type"] = infos[0].topic_type
                try:
                    self.create_subscription(get_message(rec["type"]), name,
                                             lambda _m, r=rec: r.__setitem__("messages", r["messages"] + 1),
                                             infos[0].qos_profile, raw=True)
                except Exception as exc:  # noqa: BLE001 - evidence only
                    rec["error"] = str(exc)[:200]

    def tree(self, fixed):
        frames = set(self.parent) | set(self.parent.values())
        roots = sorted(f for f in frames if f not in self.parent)

        def root_of(f):
            seen = set()
            while f in self.parent and f not in seen:
                seen.add(f)
                f = self.parent[f]
            return f
        under = sorted(f for f in frames if root_of(f) == root_of(fixed)) if fixed in frames else []
        return {"frames": len(frames), "roots": roots, "fixed_frame": fixed, "fixed_frame_known": fixed in frames,
                "frames_connected_to_fixed": len(under), "frames_not_connected": sorted(frames - set(under))[:40]}


def main():
    global STOP
    signal.signal(signal.SIGTERM, lambda *_: globals().__setitem__("STOP", True))
    config = yaml.safe_load(open(CONFIG, encoding="utf-8"))
    fixed = config["Visualization Manager"]["Global Options"].get("Fixed Frame")
    shown = displays(config)
    topics = sorted({t for d in shown for t in d["topics"]})
    started = time.monotonic()
    seen = {"polls": 0, "polls_with_rviz_node": 0, "names": []}
    rclpy.init()
    node = Check(topics)
    try:
        next_poll = 0.0
        while rclpy.ok() and not STOP:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_poll:
                node.poll()
                names = sorted(n for n in node.get_node_names() if "rviz" in n and n != "rviz_check")
                seen["polls"] += 1
                seen["polls_with_rviz_node"] += 1 if names else 0
                seen["names"] = sorted(set(seen["names"]) | set(names))
                next_poll = time.monotonic() + 1.0
    except KeyboardInterrupt:
        pass
    node.poll()
    result = {"config": "rviz/build1.rviz", "started_by": "build1.launch.py rviz:=true under Xvfb", "fixed_frame": fixed,
              "wall_s": round(time.monotonic() - started, 1), "rviz_node": seen,
              "displays": [dict(d, topics={t: node.topics[t] for t in d["topics"]}) for d in shown],
              "tf": node.tree(fixed)}
    print("RVIZCHECK " + json.dumps(result), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
