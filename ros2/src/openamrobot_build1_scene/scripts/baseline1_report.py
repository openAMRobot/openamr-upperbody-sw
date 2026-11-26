#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Print the Baseline 1 phase table, the failure classes and the trace summary from baseline1_mission.json.

  baseline1_report.py LOG_DIR [--build-ok 0|1] [--env-ok 0|1] [--env-reason TEXT] [--summary FILE]

Without a log the stage that stopped the run classifies every phase (BUILD, ENV/INFRA or STARTUP). When the launch's
/clock fail-fast fired (LOG_DIR/clock_gate.json, gate owner decision on #77) phase 1 is BLOCKED with that named reason.
Exit status 0 only when every phase passed in this one run; 1 otherwise. No ROS import.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import baseline1_phases as phases  # noqa: E402
from openamrobot_build1_scene import clock_gate  # noqa: E402


def whole_run_failures(data):
    """Failed checks of the whole run that no phase holds (rig.run's checks after the mission)."""
    held = {f["text"] for record in data.get("phases", []) for f in record["failures"]}
    return [{"text": text, "class": phases.classify(text)} for text in data.get("failures", []) if text not in held]


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("log_dir")
    parser.add_argument("--build-ok", type=int, default=1)
    parser.add_argument("--env-ok", type=int, default=1)
    parser.add_argument("--env-reason", default="")
    parser.add_argument("--summary")
    args = parser.parse_args(argv)
    path = os.path.join(args.log_dir, "baseline1_mission.json")
    data = None
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
    gate = clock_gate.read_record(args.log_dir)
    launch_reason = gate["reason"] if clock_gate.fired(gate) else ""
    table = phases.rows(data, build_ok=bool(args.build_ok), env_ok=bool(args.env_ok), env_reason=args.env_reason,
                        launch_reason=launch_reason)
    out = [phases.render(table), ""]
    if data is not None:
        out += [f"Mission result: {data.get('result')}. Reason: {data.get('reason') or '-'}",
                f"Run: {json.dumps(data.get('run', {}), sort_keys=True)}",
                f"Baseline 1 settings: {json.dumps(data.get('baseline1', {}), sort_keys=True)}", ""]
        extra = whole_run_failures(data)
        if extra:
            out += ["Whole-run checks that failed (outside the phases):"] + [f"- {e['text']} [{e['class']}]" for e in extra] + [""]
        if data.get("phases"):
            out += ["Synchronized trace, per phase (Gazebo time base):", "```"]
            out += phases.trace_summary(data.get("trace", []), data["phases"]) + ["```", ""]
    else:
        out += [f"No mission log at {path}.", ""]
    if gate is not None:
        out += [f"/clock fail-fast: {json.dumps(gate, sort_keys=True)}", ""]
    text = "\n".join(out)
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0 if phases.complete(table) else 1


if __name__ == "__main__":
    sys.exit(main())
