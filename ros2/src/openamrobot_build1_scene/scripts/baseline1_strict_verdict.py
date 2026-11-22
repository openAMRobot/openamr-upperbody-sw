#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Decide the strict Baseline 1 verdict from the mission legs of one acceptance dispatch.

  baseline1_strict_verdict.py [LEG_DIR ...] [--find ROOT] [--expect 2] [--require-profile sim-oracle]
      [--require-rmw rmw_cyclonedds_cpp] [--require-alignment 1] [--min-participant-index 120]
      [--gates config/baseline1_gates.json] [--summary FILE] [--out FILE]

--find ROOT adds every directory under ROOT that holds a declaration.json: actions/download-artifact puts each
artifact in its own directory when several match, and one matching artifact straight into ROOT.
Each leg is one artifact: declaration.json (written by the workflow: leg number, pinned commits, middleware,
settings, build and environment outcome) and ws/build1_logs/baseline1_mission.json (written by the mission).

PASS only when all of these hold:
- exactly --expect legs, numbered 1 to N;
- every leg declares the same pinned commits (every entry of "pins") and the same settings;
- every leg ran the required middleware (the identifier rclpy reported, not only the environment variable) with a
  CycloneDDS MaxAutoParticipantIndex of at least --min-participant-index;
- every leg declares station alignment on and the mission sent its goal through the alignment action;
- every leg declares the required profile, and for sim-oracle the run identity says planning_pose_source ground_truth;
- every leg passed all eight phases (baseline1_phases.complete, the same rule as baseline1_report.py);
- both legs come from this one dispatch: the same run id and run attempt (two consecutive missions);
- every leg passes the gates of openamrobot_build1_scene/baseline1_gates.py with the --gates file: the mission result
  is PASS with no failed check outside the phases; the drive, battery, lift and arm work counters advance through the
  whole run; the Stop checks of stop_authorize ran and passed and a cancellation check ran and passed; contact sensors
  were on and every contact is in the allowed list with its reason; the evidence is complete (bag, one candump per
  virtual bus, every phase timed, run_info, full pinned hashes, seed and settings declared).
With --expect below 2 the verdict is never PASS: it is NOT STRICT (a single validation run), whatever the phases.
Exit status 0 only for PASS. No ROS import. Simulation evidence only.
"""
import argparse
import json
import os
import re
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from openamrobot_build1_scene import baseline1_gates as gates  # noqa: E402
from openamrobot_build1_scene import baseline1_phases as phases  # noqa: E402

ALIGNMENT_ACTION = "/station_alignment/navigate_to_station"   # baseline1.ALIGNMENT_ACTION (that module imports ROS)
SETTING_KEYS = ("profile", "station_approach", "station_alignment", "rmw_implementation", "cyclonedds_uri", "seed")
MISSION_PATHS = (os.path.join("ws", "build1_logs", "baseline1_mission.json"),
                 os.path.join("build1_logs", "baseline1_mission.json"), "baseline1_mission.json")


def _load(path):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def read_leg(leg_dir):
    declaration = _load(os.path.join(leg_dir, "declaration.json"))
    mission = None
    for rel in MISSION_PATHS:
        mission = _load(os.path.join(leg_dir, rel))
        if mission is not None:
            break
    return declaration, mission


def find_legs(root):
    """Every directory under root (root included) that holds a declaration.json, sorted."""
    return sorted(dirpath for dirpath, _dirs, files in os.walk(root) if "declaration.json" in files)


def participant_index(uri):
    match = re.search(r"<MaxAutoParticipantIndex>\s*(\d+)\s*</MaxAutoParticipantIndex>", uri or "")
    return int(match.group(1)) if match else None


def check_leg(declaration, mission, args, leg_dir="."):
    """Return (phases passed, phase count, problems) for one leg."""
    problems = []
    if declaration is None:
        return 0, len(phases.PHASES), ["no declaration.json"]
    table = phases.rows(mission, build_ok=bool(declaration.get("build_ok", 0)),
                        env_ok=bool(declaration.get("env_ok", 0)), env_reason=declaration.get("env_reason", ""))
    passed = sum(1 for row in table if row[3] == "PASS")
    if mission is None:
        problems.append("no baseline1_mission.json")
    if not phases.complete(table):
        failed = [row[0] for row in table if row[3] != "PASS"]
        problems.append(f"{passed} of {len(phases.PHASES)} phases passed; not passed: {', '.join(failed)}")
    rmw = declaration.get("rmw_runtime")
    if rmw != args.require_rmw:
        problems.append(f"middleware {rmw!r} at runtime, {args.require_rmw} required")
    index = participant_index(declaration.get("cyclonedds_uri"))
    if index is None or index < args.min_participant_index:
        problems.append(f"CycloneDDS MaxAutoParticipantIndex {index}, at least {args.min_participant_index} required")
    if str(declaration.get("station_alignment")) != args.require_alignment:
        declared = declaration.get("station_alignment")
        problems.append(f"station_alignment declared {declared!r}, {args.require_alignment} required")
    if args.require_alignment == "1" and mission is not None:
        action = (mission.get("baseline1") or {}).get("navigate_action")
        if action != ALIGNMENT_ACTION:
            problems.append(f"the mission sent its goal to {action!r}, not {ALIGNMENT_ACTION}")
    if declaration.get("profile") != args.require_profile:
        problems.append(f"profile declared {declaration.get('profile')!r}, {args.require_profile} required")
    if args.require_profile == "sim-oracle" and mission is not None:
        source = (mission.get("run") or {}).get("planning_pose_source")
        if source != "ground_truth":
            problems.append(f"run identity planning_pose_source {source!r}, sim-oracle declares ground_truth")
    problems += gates.all_gates(leg_dir, declaration, mission, args.gate_config)
    return passed, len(phases.PHASES), problems


def decide(legs, args):
    """legs: list of (leg_dir, declaration, mission). Return the verdict record."""
    record = {"expect": args.expect, "legs": [], "problems": [], "pins": None}
    numbers = []
    for leg_dir, declaration, mission in legs:
        passed, total, problems = check_leg(declaration, mission, args, leg_dir)
        number = (declaration or {}).get("leg")
        numbers.append(number)
        record["legs"].append({"dir": leg_dir, "leg": number, "phases_passed": passed, "phases": total,
                               "run_id": (declaration or {}).get("run_id"), "pins": (declaration or {}).get("pins"),
                               "settings": {k: (declaration or {}).get(k) for k in SETTING_KEYS}, "problems": problems,
                               "allowed_contacts": gates.allowed_contacts(mission, args.gate_config)})
        record["problems"] += [f"leg {number}: {p}" for p in problems]
    numbered = sorted(n for n in numbers if isinstance(n, int))
    if numbered != list(range(1, args.expect + 1)) or len(numbers) != args.expect:
        record["problems"].append(f"legs found {numbers}, expected 1 to {args.expect}")
    pins = [leg["pins"] for leg in record["legs"]]
    if not pins or any(not p for p in pins):
        record["problems"].append("a leg declares no pinned commits")
    elif any(p != pins[0] for p in pins):
        record["problems"].append("the legs ran on different pinned commits")
    else:
        record["pins"] = pins[0]
    dispatch = {((d or {}).get("run_id"), (d or {}).get("run_attempt")) for _dir, d, _m in legs}
    if len(dispatch) > 1:
        record["problems"].append(f"the legs come from different dispatches or attempts: {sorted(dispatch, key=str)}")
    settings = [leg["settings"] for leg in record["legs"]]
    if any(s != settings[0] for s in settings):
        record["problems"].append("the legs declare different settings")
    if args.expect < 2:
        record["verdict"] = "NOT STRICT"
    else:
        record["verdict"] = "FAIL" if record["problems"] else "PASS"
    return record


def render(record):
    lines = [f"### Strict Baseline 1 verdict: {record['verdict']}", "",
             f"Rule: {record['expect']} consecutive missions in this dispatch, each 8 of 8, same pinned commits, "
             "CycloneDDS with the raised participant index, station alignment on, declared profile, components alive "
             "for the whole run, Stop and cancellation checks, no unaccounted contact, complete evidence.", "",
             "| Leg | Run | Phases | Problems |", "|---|---|---|---|"]
    for leg in record["legs"]:
        lines.append(f"| {leg['leg']} | {leg['run_id']} | {leg['phases_passed']} of {leg['phases']} | "
                     f"{'; '.join(leg['problems']).replace('|', '/') or 'none'} |")
    lines += ["", "Pinned commits (identical in every leg):" if record["pins"]
              else "Pinned commits: not identical or missing"]
    for leg in record["legs"]:
        for name, sha in sorted((leg["pins"] or {}).items()):
            lines.append(f"- leg {leg['leg']} {name} {sha}")
    allowed = [(leg["leg"], c) for leg in record["legs"] for c in leg.get("allowed_contacts") or []]
    if allowed:
        lines += ["", "Allowed contacts (reported, not problems):"]
        for number, c in allowed:
            where = c["phase"] + (f" ({c['window']})" if c.get("window") else "")
            cap = f", cap {c['max_force_cap_n']} N" if c.get("max_force_cap_n") is not None else ""
            lines.append(f"- leg {number} {where}: {c['a']} with {c['b']}, {c['count']} samples, "
                         f"max {c['max_force_n']} N{cap}")
    if record["problems"]:
        lines += ["", "Problems:"] + [f"- {p}" for p in record["problems"]]
    if record["verdict"] == "NOT STRICT":
        lines += ["", "A single validation run: never the strict two-run acceptance, whatever its phases."]
    lines += ["", "Simulation evidence only: not hardware, physical or safety acceptance."]
    return "\n".join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("legs", nargs="*")
    parser.add_argument("--find", help="add every directory under this root that holds a declaration.json")
    parser.add_argument("--expect", type=int, default=2)
    parser.add_argument("--require-profile", default="sim-oracle")
    parser.add_argument("--require-rmw", default="rmw_cyclonedds_cpp")
    parser.add_argument("--require-alignment", default="1")
    parser.add_argument("--min-participant-index", type=int, default=120)
    parser.add_argument("--gates", default=gates.GATES_FILE, help="the gate configuration (config/baseline1_gates.json)")
    parser.add_argument("--summary")
    parser.add_argument("--out")
    args = parser.parse_args(argv)
    args.gate_config = gates.load(args.gates)
    dirs = sorted(set(args.legs) | set(find_legs(args.find) if args.find else []))
    legs = [(d, *read_leg(d)) for d in dirs]
    record = decide(legs, args)
    text = render(record)
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as f:
            f.write(text + "\n")
    if args.out:
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(record, f, indent=2, sort_keys=True)
    return 0 if record["verdict"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
