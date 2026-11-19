#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Offline mission report: one static HTML page from a recorded bag and the mission JSON.

  mission_report.py --bag BUILD1_LOG_DIR/bag --mission BUILD1_LOG_DIR/baseline1_mission.json \\
      [--candump BUILD1_LOG_DIR/record_candump_*.log] [--station X Y YAW | --scene SCENE_YAML] -o report.html
  mission_report.py --synthetic DIR -o report.html     writes the synthetic fixture bag to DIR first
  mission_report.py --synthetic-plain -o report.html   the same fixture without ROS or a bag (layout preview)

The station pose defaults to stations.work_shelf of openamrobot_build1_scene's config/scene.yaml when that package
is installed. Exit 0 when the page was written. The page records a simulation run; it decides nothing.
"""
import argparse
import glob
import json
import os
import sys


def station_from_scene(path, name):
    from openamrobot_build1_scene import scene as scene_model
    return scene_model.station(scene_model.load(path), name)


def default_scene():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory("openamrobot_build1_scene"), "config", "scene.yaml")
    except Exception:  # noqa: BLE001 - no ament index or no openamrobot_build1_scene: no default station
        return None


def main(argv=None):
    from openamrobot_control_room import bag, report
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag")
    parser.add_argument("--mission")
    parser.add_argument("--candump", nargs="*", default=[])
    parser.add_argument("--control-room-log", nargs="*", default=[],
                        help="the control room's JSON lines (status.jsonl, can.jsonl): host and Gazebo time pairs")
    parser.add_argument("--time-base-json", help="write the time base and its measured skews to this file")
    parser.add_argument("--station", nargs=3, type=float)
    parser.add_argument("--scene")
    parser.add_argument("--station-name", default="work_shelf")
    parser.add_argument("--synthetic", help="write the synthetic fixture bag and mission JSON into this directory and use them")
    parser.add_argument("--synthetic-plain", action="store_true",
                        help="the synthetic run without ROS or a bag (a preview of the page layout only)")
    parser.add_argument("--title", default="Baseline 1 mission report")
    parser.add_argument("-o", "--output", required=True)
    args = parser.parse_args(argv)
    station = tuple(args.station) if args.station else None
    if args.synthetic_plain:
        series = report.extract(bag.synthetic_messages(bag.plain_types()), station=bag.SYNTHETIC_STATION)
        page = report.render(series, bag.synthetic_mission(), report.candump_rates(bag.synthetic_candump()),
                             title=args.title + " (synthetic fixture, no bag)", source="bag.synthetic_messages, no ROS")
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(page)
        print(f"wrote {args.output} ({len(page)} bytes)")
        return 0
    can_lines = {}
    if args.synthetic:
        os.makedirs(args.synthetic, exist_ok=True)
        args.bag = os.path.join(args.synthetic, "bag")
        if not os.path.exists(args.bag):
            bag.write_synthetic(args.bag)
        args.mission = os.path.join(args.synthetic, "baseline1_mission.json")
        with open(args.mission, "w", encoding="utf-8") as f:
            json.dump(bag.synthetic_mission(), f, indent=1)
        can_lines = bag.synthetic_candump()
        station = station or bag.SYNTHETIC_STATION
        args.title += " (synthetic fixture)"
    if station is None:
        scene = args.scene or default_scene()
        if scene and os.path.exists(scene):
            try:
                station = station_from_scene(scene, args.station_name)
            except (ImportError, KeyError) as error:
                print(f"station: {error}; the station error plot is left empty", file=sys.stderr)
    for pattern in args.candump:
        for path in sorted(glob.glob(pattern)):
            name = os.path.basename(path).replace("record_candump_", "").replace(".log", "")
            with open(path, encoding="utf-8", errors="replace") as f:
                can_lines[name] = f.readlines()
    mission = None
    if args.mission and os.path.exists(args.mission):
        with open(args.mission, encoding="utf-8") as f:
            mission = json.load(f)
    elif args.mission:
        print(f"no mission JSON at {args.mission}: the phase timeline is left empty", file=sys.stderr)
    if not args.bag or not os.path.exists(args.bag):
        print(f"no bag at {args.bag}", file=sys.stderr)
        return 2
    series = report.extract(bag.read(args.bag), station=station)
    from openamrobot_control_room import timebase
    lines = []
    for path in args.control_room_log:
        if os.path.exists(path):
            with open(path, encoding="utf-8", errors="replace") as f:
                lines.extend(f.readlines())
    to_sim, time_base = timebase.run_time_base(can_lines if not args.synthetic else {}, series["lift_status"], lines,
                                               mission, series["mission_status_json"])
    print("time base: " + json.dumps(time_base, sort_keys=True))
    if args.time_base_json:
        with open(args.time_base_json, "w", encoding="utf-8") as f:
            json.dump(time_base, f, indent=1, sort_keys=True)
    decoded = report.candump_decoded(can_lines, to_sim=to_sim) if can_lines and not args.synthetic else None
    rates = report.candump_rates(can_lines, to_sim=to_sim if not args.synthetic else None) if can_lines else None
    page = report.render(series, mission, rates, title=args.title,
                         source=f"bag {args.bag}, mission {args.mission or '-'}", decoded=decoded, time_base=time_base)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        f.write(page)
    counts = {k: len(v) for k, v in series.items() if isinstance(v, list)}
    print(f"wrote {args.output} ({len(page)} bytes); series points {json.dumps(counts, sort_keys=True)}; "
          f"topics {len(series['topics'])}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
