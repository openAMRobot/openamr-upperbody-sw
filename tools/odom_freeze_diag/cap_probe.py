# SPDX-License-Identifier: MIT
"""Diagnostics only (do not merge): does the LOCALHOST discovery range cap which participants see each other?

`cap_probe.py member N OUT`: participant N publishes /cap/pN (std_msgs/String, a test topic of this probe only) and,
on SIGTERM, writes OUT/N.json with the /cap publishers its own graph shows. Run 40 of them one after the other with
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST (as build1_harness.py sets) and compare who sees whom.
`cap_probe.py report OUT DOMAIN SS_FILE`: one line per member: its participant index (from its bound discovery port) and
how many of the other members it sees. Nothing here touches the robot stack.
"""
import json
import os
import re
import signal
import sys
import time


def member(n, out):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    rclpy.init()
    node = Node(f"cap_member_{n}", enable_rosout=False, start_parameter_services=False)
    pub = node.create_publisher(String, f"/cap/p{n}", 10)
    node.create_timer(0.5, lambda: pub.publish(String(data=str(n))))
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGUSR1, lambda *_: write(node, n, out))   # the view is taken while every member still runs
    while not stop:
        rclpy.spin_once(node, timeout_sec=0.2)


def write(node, n, out):
    seen = sorted(int(t[len("/cap/p"):]) for t, _ in node.get_topic_names_and_types()
                  if t.startswith("/cap/p") and node.get_publishers_info_by_topic(t))
    with open(os.path.join(out, f"{n}.json"), "w") as f:
        json.dump({"member": n, "pid": os.getpid(), "seen": seen}, f)


def report(out, domain, ss_file):
    base = 7400 + 250 * int(domain) + 10
    index_of_pid = {}
    for line in open(ss_file):
        m = re.search(r":(\d+)\s+\S+\s+users:\((.*)\)", line)
        if not m:
            continue
        offset = int(m.group(1)) - base
        if offset >= 0 and offset % 2 == 0 and offset // 2 < 120:
            for pid in re.findall(r"pid=(\d+)", m.group(2)):
                index_of_pid.setdefault(int(pid), offset // 2)
    rows = [json.load(open(os.path.join(out, f))) for f in os.listdir(out) if f.endswith(".json")]
    total = 40
    print("CAPPROBE " + json.dumps({"members_reporting": len(rows), "of": total}))
    for r in sorted(rows, key=lambda r: r["member"]):
        others = [s for s in r["seen"] if s != r["member"]]
        print("CAPPROBE " + json.dumps({"member": r["member"], "index": index_of_pid.get(r["pid"]), "sees_others": len(others),
                                        "of": total - 1, "missing": sorted(set(range(total)) - set(r["seen"]) - {r["member"]})}))


if __name__ == "__main__":
    if sys.argv[1] == "member":
        member(int(sys.argv[2]), sys.argv[3])
    else:
        report(sys.argv[2], sys.argv[3], sys.argv[4])
