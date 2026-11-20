#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""DDS benchmark: per-run console digest and the comparison table across settings (docs/integration/dds_benchmark.md).

  compare.py console MISSION_CONSOLE.txt > dds_symptoms.json
      DDS and delivery symptoms the mission's own console shows: error lines of the middleware, Cyclone's participant
      index exhaustion, the #167 detector's published station poses (publisher-side count) and the alignment node's
      received count, base_state_adapter's stale warnings.
  compare.py table ARTIFACT_ROOT [--summary FILE]
      One row per downloaded run directory (dds-benchmark-<setting>-r<repeat>-attempt<n>): mission phases, RTF,
      CPU and memory, delivery and latency per key topic, participants and the late-participant view.
Diagnostics only: the table judges nothing; docs/integration/dds_benchmark.md says how owners read it.
"""
import argparse
import csv
import glob
import json
import os
import re
import statistics
import sys
from collections import defaultdict

PATTERNS = {
    "failed_to": r"Failed to",
    "shm": r"\bSHM\b|shared memory|SharedMem",
    "rtps_error": r"RTPS_|\[RTPS",
    "port_in_use": r"Address already in use|port .* in use",
    "free_participant_index": r"free participant index|participant index",
    "cyclone": r"ddsi_|cyclonedds|ddsrt_",
    "no_wheel_speed_sample": r"no wheel speed sample",
    "rcl_error": r"RCLError|rcl_.*failed",
}
DETECTOR = re.compile(r"\[station_marker_detector\]: stats .*?messages=(\d+) published=(\d+)")
DETECTOR_NONE = re.compile(r"\[station_marker_detector\]: stats messages=(\d+)(?! published)")
ALIGNMENT = re.compile(r"\[station_alignment\]: (station alignment [^\n]*)")
RECEIVED = re.compile(r"(\d+) received")
PHASE = re.compile(r"^\| (\d+) \| (\w+) \|.*\| (PASS|FAIL|NOT RUN|BLOCKED) \|")
KEY = ("/odom", "/tf", "/scan", "/joint_states", "/sil/base/odom", "/station_alignment/station_pose", "CAMERA")


def console_digest(text):
    lines = text.splitlines()
    out = {"lines": len(lines), "patterns": {}, "examples": {}}
    for name, pattern in PATTERNS.items():
        rx = re.compile(pattern, re.IGNORECASE)
        hits = [line for line in lines if rx.search(line)]
        out["patterns"][name] = len(hits)
        out["examples"][name] = sorted(set(re.sub(r"\[\d{10}\.\d+\]", "", h)[:240] for h in hits))[:5]
    messages = published = 0
    for match in DETECTOR.finditer(text):
        messages += int(match.group(1))
        published += int(match.group(2))
    for match in DETECTOR_NONE.finditer(text):
        messages += int(match.group(1))
    out["detector_detection_messages"] = messages
    out["detector_station_poses_published"] = published
    out["alignment_lines"] = sorted(set(m.group(1)[:240] for m in ALIGNMENT.finditer(text)))[:10]
    received = [int(m.group(1)) for line in out["alignment_lines"] for m in RECEIVED.finditer(line)]
    out["alignment_station_poses_received"] = received[0] if received else None
    return out


def phases(text):
    rows = [m.groups() for m in (PHASE.match(line.strip()) for line in text.splitlines()) if m]
    passed = sum(1 for _, _, result in rows if result == "PASS")
    first_fail = next((f"{name} ({result})" for _, name, result in rows if result != "PASS"), "-")
    return {"rows": len(rows), "passed": passed, "first_not_passed": first_fail}


def process_stats(path):
    """Mean CPU and peak RSS per process name, and the totals, from processes.csv."""
    per = defaultdict(lambda: {"cpu": [], "rss": []})
    totals = {"cpu": [], "rss": []}
    with open(path) as f:
        for row in csv.DictReader(f):
            target = totals if row["name"] == "TOTAL" else per[row["name"]]
            target["cpu"].append(float(row["cpu_percent"]))
            target["rss"].append(float(row["rss_mb"]))
    top = sorted(((name, statistics.mean(v["cpu"]), max(v["rss"])) for name, v in per.items() if v["cpu"]),
                 key=lambda r: -r[1])[:12]
    return {"total_cpu_mean": round(statistics.mean(totals["cpu"]), 1) if totals["cpu"] else None,
            "total_mem_peak_mb": round(max(totals["rss"]), 1) if totals["rss"] else None,
            "top_cpu": [{"name": n, "cpu_mean": round(c, 1), "rss_peak_mb": r} for n, c, r in top]}


def topic_cell(summary, topic):
    record = summary.get("topics", {}).get(topic)
    if not record:
        return "not received"
    streams = record["streams"]
    if topic == "/tf":
        stream = streams.get("odom>base_footprint")
        label = "odom>base_footprint "
    else:
        stream = next(iter(streams.values()), None)
        label = ""
    if not stream:
        return f"{label}not received"
    ratio = stream.get("ratio")
    lat = stream.get("latency_wall_s") or {}
    lat_sim = stream.get("latency_sim_s") or {}
    return (f"{label}{stream['unique_stamps']}/{stream['expected']} ({'-' if ratio is None else f'{ratio:.3f}'}); "
            f"wall p50/p95 {lat.get('p50', '-')}/{lat.get('p95', '-')} s; sim p95 {lat_sim.get('p95', '-')} s")


def late_missing(summary):
    """Publisher nodes the index-0 observer sees and the late probe does not, per key topic."""
    probe = summary.get("late_view", {}).get("_late_probe", {}).get("topics", {})
    missing = {}
    for topic, view in probe.items():
        own = {e["node"].lstrip("/") for e in summary.get("qos", {}).get(topic, {}).get("publishers", [])}
        seen = {n.lstrip("/") for n in view.get("publisher_nodes", [])}
        if own - seen:
            missing[topic] = sorted(own - seen)
    return missing


def run_row(directory):
    name = os.path.basename(directory.rstrip("/"))
    row = {"run": name}
    summary_path = glob.glob(os.path.join(directory, "**", "observer_summary.json"), recursive=True)
    summary = json.load(open(summary_path[0])) if summary_path else {}
    row["rmw"] = summary.get("rmw")
    rtf = [w["rtf"] for w in summary.get("rtf", [])]
    row["rtf_median"] = round(statistics.median(rtf), 3) if rtf else None
    row["rtf_min"] = round(min(rtf), 3) if rtf else None
    participants = summary.get("participants", [])
    row["participants_max"] = max((p["count"] for p in participants), default=None)
    row["max_index"] = max((p["max_index"] for p in participants if p["max_index"] is not None), default=None)
    camera = summary.get("camera_topic") or next((t for t in summary.get("topics", {}) if "image" in t), None)
    row["topics"] = {t: topic_cell(summary, camera if t == "CAMERA" else t) for t in KEY}
    row["late_missing"] = late_missing(summary)
    table = glob.glob(os.path.join(directory, "**", "phase_table.txt"), recursive=True)
    row["phases"] = phases(open(table[0]).read()) if table else None
    digest = glob.glob(os.path.join(directory, "**", "dds_symptoms.json"), recursive=True)
    row["console"] = json.load(open(digest[0])) if digest else None
    procs = glob.glob(os.path.join(directory, "**", "processes.csv"), recursive=True)
    row["processes"] = process_stats(procs[0]) if procs else None
    return row


def markdown(rows):
    out = ["### DDS benchmark comparison (simulation diagnostics, Proposed; not acceptance evidence)", "",
           "| run | rmw | phases passed | first not passed | RTF median (min) | CPU total mean % | mem peak MB | participants (max index) | DDS console lines | station poses published / received by alignment |",
           "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        ph = r["phases"] or {}
        pr = r["processes"] or {}
        co = r["console"] or {}
        pats = co.get("patterns", {})
        dds = ", ".join(f"{k} {v}" for k, v in pats.items() if v and k not in ("no_wheel_speed_sample",)) or "none"
        out.append(f"| {r['run']} | {r['rmw']} | {ph.get('passed', '-')}/{ph.get('rows', '-')} | {ph.get('first_not_passed', '-')} | "
                   f"{r['rtf_median']} ({r['rtf_min']}) | {pr.get('total_cpu_mean', '-')} | {pr.get('total_mem_peak_mb', '-')} | "
                   f"{r['participants_max']} ({r['max_index']}) | {dds} | {co.get('detector_station_poses_published', '-')} / "
                   f"{co.get('alignment_station_poses_received', '-')} |")
    out += ["", "Delivery to the index-0 observer: unique stamps received / count implied by the publisher's stamp sequence "
            "(ratio); latency from header stamp to receipt.", "",
            "| run | " + " | ".join(KEY) + " |", "|---|" + "---|" * len(KEY)]
    for r in rows:
        out.append(f"| {r['run']} | " + " | ".join(r["topics"][t] for t in KEY) + " |")
    out += ["", "Publishers the index-0 observer sees and a late participant (spun 15 s) does not:", ""]
    for r in rows:
        out.append(f"- {r['run']}: {json.dumps(r['late_missing']) if r['late_missing'] else 'none'}")
    return "\n".join(out) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)
    c = sub.add_parser("console")
    c.add_argument("path")
    t = sub.add_parser("table")
    t.add_argument("root")
    t.add_argument("--summary")
    t.add_argument("--json")
    args = parser.parse_args(argv)
    if args.command == "console":
        print(json.dumps(console_digest(open(args.path, errors="replace").read()), indent=1))
        return 0
    dirs = sorted(d for d in glob.glob(os.path.join(args.root, "*")) if os.path.isdir(d))
    rows = [run_row(d) for d in dirs]
    text = markdown(rows)
    print(text)
    if args.summary:
        with open(args.summary, "a") as f:
            f.write(text)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(rows, f, indent=1)
    return 0 if rows else 1


if __name__ == "__main__":
    sys.exit(main())
