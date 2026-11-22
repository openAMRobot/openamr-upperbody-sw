#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Runs one Build 1 scenario against a stack that build1.launch.py already started.

    run_scenario.py NAME [--log-dir DIR]        NAME: one of the names printed by --list

Exit 0 PASS, 1 FAIL, 3 BLOCKED. BLOCKED is not a pass. The log is DIR/NAME.json.
Simulation only: a result is development evidence, not hardware or safety evidence.
"""
import argparse
import os
import sys

from openamrobot_build1_scene import rig as rig_module
from openamrobot_build1_scene import scenarios


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("name", nargs="?")
    parser.add_argument("--list", action="store_true")
    parser.add_argument("--log-dir", default=None)
    args = parser.parse_args(argv)
    if args.list or args.name is None:
        print("\n".join(scenarios.SCENARIOS))
        return 0
    if args.name not in scenarios.SCENARIOS:
        print(f"unknown scenario {args.name}", file=sys.stderr)
        return 2
    if args.log_dir:
        os.environ["BUILD1_LOG_DIR"] = args.log_dir
    function, needs_devices, ready = scenarios.SCENARIOS[args.name]
    result, _reason, _path = rig_module.run(args.name, function, needs_devices=needs_devices,
                                            uses_base=args.name in scenarios.BASE_SCENARIOS, **ready)
    return {"PASS": 0, "BLOCKED": 3}.get(result, 1)


if __name__ == "__main__":
    sys.exit(main())
