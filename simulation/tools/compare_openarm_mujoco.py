#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare the arms of a generated MuJoCo model with Enactic's openarm_mujoco at its pin, and print the differences.

  python3 simulation/tools/compare_openarm_mujoco.py --model build/sim/bp000/openamrobot_bp000.xml --openarm-mujoco <checkout>
      [--json out.json] [--all]

The checkout must be at the commit pinned in simulation/sim_pins.yaml. Output: a Markdown table of the differences (all rows with --all).
"""
import argparse
import json
import pathlib
import subprocess
import sys

import yaml

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import openarm_compare  # noqa: E402

PINS = pathlib.Path(__file__).resolve().parents[1] / "sim_pins.yaml"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--model", required=True)
    parser.add_argument("--openarm-mujoco", required=True)
    parser.add_argument("--json")
    parser.add_argument("--all", action="store_true")
    opts = parser.parse_args()
    import mujoco

    pin = yaml.safe_load(PINS.read_text())["openarm_mujoco"]["commit"]
    head = subprocess.run(["git", "-C", opts.openarm_mujoco, "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    if head != pin:
        print(f"BLOCKED: openarm_mujoco checkout is at {head or 'no commit'}, the pin is {pin}", file=sys.stderr)
        return 3
    ours = mujoco.MjModel.from_xml_path(opts.model)
    theirs = mujoco.MjModel.from_xml_path(str(pathlib.Path(opts.openarm_mujoco) / "v2" / "openarm_bimanual.xml"))
    rows = openarm_compare.compare(mujoco, ours, theirs)
    if opts.json:
        pathlib.Path(opts.json).write_text(json.dumps(rows, indent=1) + "\n")
    sys.stdout.write(openarm_compare.markdown(rows, only_differences=not opts.all))
    print(f"\n{len(rows)} items compared, {sum(not r['same'] for r in rows)} differ.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
