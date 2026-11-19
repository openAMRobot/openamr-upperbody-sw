#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the grocery dark store mission against a stack that grocery_demo_gui.launch.py (or build1.launch.py with
scene:=grocery_scene.yaml) already started, and print the phase table.

    grocery_demo_run.py [--log-dir DIR]

The same mission as the CI launch test (openamrobot_build1_scene/grocery_demo.py). Exit 0 when all twelve phases
passed, 1 otherwise, 3 BLOCKED. Simulation only: a result is development evidence, not hardware or safety evidence.
"""
import argparse
import os
import subprocess
import sys


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--log-dir", default=None)
    args, _ros = parser.parse_known_args(argv)
    if args.log_dir:
        os.environ["BUILD1_LOG_DIR"] = args.log_dir
    os.environ.setdefault("BASELINE1_STATION_APPROACH", "1")
    os.environ.setdefault("BASELINE1_STATION_ALIGNMENT", "0")
    from openamrobot_build1_scene import grocery_demo, rig
    result, reason, path = rig.run("grocery_demo_mission", grocery_demo.mission, needs_devices=True, nav2=True, moveit=True)
    print(f"grocery demo mission: {result}. {reason or ''} Log: {path}", flush=True)
    report = os.path.join(os.path.dirname(os.path.abspath(__file__)), "grocery_demo_report.py")
    subprocess.run([sys.executable, report, os.path.dirname(path)], check=False)
    return {"PASS": 0, "BLOCKED": 3}.get(result, 1)


if __name__ == "__main__":
    sys.exit(main())
