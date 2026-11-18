#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Print the grocery dark store demonstrator's phase table and evidence summary from grocery_demo_mission.json.

  grocery_demo_report.py LOG_DIR [--build-ok 0|1] [--env-ok 0|1] [--env-reason TEXT] [--summary FILE]

Without a log the stage that stopped the run classifies every phase (BUILD, ENV/INFRA or STARTUP), as Baseline 1's
report does. Exit status 0 only when every phase passed in this one run; 1 otherwise. No ROS import.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import baseline1_phases as b1  # noqa: E402
from openamrobot_build1_scene import grocery_phases as phases  # noqa: E402


def load(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def key_numbers(data):
    """The measured values a reader looks for first, by phase."""
    wanted = ("station_reach_position_error", "station_reach_yaw_error", "nav_ground_truth_position_error", "nav_status",
              "item_height_gain_after_lift", "item_height_gain_after_backup", "item_height_gain_at_packing",
              "item_distance_to_tcp_at_packing", "place_lowering_distance", "item_in_tote_along", "item_in_tote_deep",
              "item_height_above_tote_floor", "item_speed_after_release", "finger_open_after_place",
              "profile_pose_source_ground_truth")
    out = []
    for record in (data or {}).get("phases", []):
        for q in record["quantities"]:
            if q["name"] in wanted:
                limits = "" if q.get("ok") is None else f" [{q['low']}, {q['high']}] {'ok' if q['ok'] else 'OUTSIDE'}"
                out.append(f"{record['phase']}: {q['name']} = {q['value']} {q['unit']}{limits}")
    return out


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir")
    parser.add_argument("--build-ok", type=int, default=1)
    parser.add_argument("--env-ok", type=int, default=1)
    parser.add_argument("--env-reason", default="")
    parser.add_argument("--summary")
    args = parser.parse_args(argv)
    data = load(os.path.join(args.log_dir, "grocery_demo_mission.json"))
    table = phases.rows(data, build_ok=bool(args.build_ok), env_ok=bool(args.env_ok), env_reason=args.env_reason)
    out = [phases.render(table), ""]
    listing = os.listdir(args.log_dir) if os.path.isdir(args.log_dir) else []
    out += ["### Evidence summary", "", "```"] + phases.evidence_summary(data, listing) + ["```", ""]
    if data is not None:
        out += [f"Mission result: {data.get('result')}. Reason: {data.get('reason') or '-'}", ""]
        held = {f["text"] for record in data.get("phases", []) for f in record["failures"]}
        extra = [t for t in data.get("failures", []) if t not in held]
        if extra:
            out += ["Whole-run checks that failed (outside the phases):"] + [f"- {t} [{b1.classify(t)}]" for t in extra] + [""]
        out += ["Key measurements:", "```"] + key_numbers(data) + ["```", ""]
        if data.get("phases"):
            out += ["Synchronized trace, per phase (Gazebo time base):", "```"]
            out += b1.trace_summary(data.get("trace", []), data["phases"]) + ["```", ""]
    else:
        out += [f"No mission log in {args.log_dir}.", ""]
    contacts = load(os.path.join(args.log_dir, "grocery_contacts.json"))
    if contacts and contacts.get("rows"):
        robot = [r for r in contacts["rows"] if "openamrobot::" in r["pair"]]
        out += [f"Contact events of the robot with the world (gz contact sensors, per phase; simulation forces, not "
                f"measurements; {len(contacts['rows']) - len(robot)} resting object contacts are in grocery_contacts.json):", "```"]
        out += [f"{r['phase']} | {r['sensor']} | {r['pair']} | messages {r['messages']} | max force {r['max_force_n']:.2f} N"
                for r in robot[:80]] or ["none"]
        out += ["```", ""]
    out += ["Simulation evidence only (sim-oracle profile): not hardware, physical acceptance, perception or safety evidence."]
    text = "\n".join(out)
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0 if phases.complete(table) else 1


if __name__ == "__main__":
    sys.exit(main())
