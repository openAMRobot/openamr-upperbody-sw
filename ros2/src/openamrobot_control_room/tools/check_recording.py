#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fail when the control room's own recording does not contain what the control room published.

    check_recording.py BAG_DIR [--status-jsonl FILE] [--min-status N] [--require TOPIC ...]

Reads BAG_DIR/metadata.yaml (rosbag2; no ROS needed). PASS needs at least N messages (default 10) on
/control_room/status and at least one on every --require topic. With --status-jsonl (the monitor's own JSON lines,
written without DDS) a FAIL also says whether the node ran: many status lines next to an empty recording means the
recorder never matched the monitor's publishers (discovery), not that the monitor was absent.
Run 44445938978 recorded /baseline1/mission_status and /robot_description and no /control_room/ topic, and its
step passed: this check makes that a failure. Simulation tooling only; not hardware or safety evidence.
"""
import argparse
import os
import sys

import yaml

STATUS = "/control_room/status"


def topic_counts(bag_dir):
    """{topic: message count} from a rosbag2 metadata.yaml. Raises FileNotFoundError without one."""
    with open(os.path.join(bag_dir, "metadata.yaml"), encoding="utf-8") as f:
        meta = yaml.safe_load(f) or {}
    info = meta.get("rosbag2_bagfile_information", {})
    return {entry["topic_metadata"]["name"]: int(entry.get("message_count", 0))
            for entry in info.get("topics_with_message_count") or []}


def verdict(counts, min_status=10, require=(), status_lines=None):
    """(ok, lines): the check's result and the lines to print."""
    lines, ok = [], True
    ours = {t: n for t, n in sorted(counts.items()) if t.startswith("/control_room/")}
    lines.append(f"recorded /control_room/ topics: {len(ours)}; other topics: {len(counts) - len(ours)}")
    for topic, n in ours.items():
        lines.append(f"  {topic}: {n}")
    if counts.get(STATUS, 0) < min_status:
        ok = False
        lines.append(f"FAIL: {STATUS} has {counts.get(STATUS, 0)} messages, at least {min_status} needed")
    for topic in require:
        if counts.get(topic, 0) < 1:
            ok = False
            lines.append(f"FAIL: {topic} has no message in the recording")
    if not ok and status_lines is not None:
        if status_lines >= min_status:
            lines.append(f"the monitor ran ({status_lines} status lines in its own JSON lines file) but the recorder "
                         "matched none or too few of its publishers: a discovery failure between the two, not a "
                         "missing monitor")
        else:
            lines.append(f"the monitor's own JSON lines file has {status_lines} lines: the monitor did not run (long enough)")
    if ok:
        lines.append(f"PASS: {STATUS} {counts[STATUS]} messages" + (f", required topics present: {', '.join(require)}" if require else ""))
    return ok, lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("bag")
    parser.add_argument("--status-jsonl")
    parser.add_argument("--min-status", type=int, default=10)
    parser.add_argument("--require", nargs="*", default=[])
    args = parser.parse_args(argv)
    try:
        counts = topic_counts(args.bag)
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        print(f"FAIL: no readable recording in {args.bag}: {error}")
        return 1
    status_lines = None
    if args.status_jsonl:
        try:
            with open(args.status_jsonl, encoding="utf-8") as f:
                status_lines = sum(1 for line in f if line.strip())
        except OSError:
            status_lines = 0
    ok, lines = verdict(counts, args.min_status, args.require, status_lines)
    print("\n".join(lines))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
