#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the backend benchmark scenarios (simulation/benchmark/scenarios.yaml) in MuJoCo and write the result JSON.

  source /opt/ros/jazzy/setup.bash
  python3 simulation/tools/benchmark_mujoco.py --platform-sw <pin checkout> --openarm-description <pin checkout> --out build/bench

The model is generated from the locked expansion first (as generate.py does). Simulation evidence only.
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import benchmark  # noqa: E402
from openamr_sim import expand as ex  # noqa: E402
from openamr_sim import mjcf  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--platform-sw", required=True)
    parser.add_argument("--openarm-description", required=True)
    parser.add_argument("--out", default="build/bench")
    parser.add_argument("--arm-gravity", choices=("build1", "off"), default=None,
                        help="arm gravity feed-forward; default: engine_overrides.yaml arm_gravity_feedforward (build1)")
    parser.add_argument("--wheel-contact", choices=tuple(benchmark.WHEEL_CONTACT), default="pinned",
                        help="diagnostic: drive wheel contact shape of platform-sw pull request 171; default: the generated model")
    parser.add_argument("--name", default="benchmark_mujoco", help="result file name without .json")
    opts = parser.parse_args()
    spec = benchmark.load_scenarios()
    out = pathlib.Path(opts.out)
    try:
        expansion = ex.expand(spec["configuration"], opts.platform_sw, opts.openarm_description)
    except (ex.LockMismatch, ex.NotLocked) as err:
        print(f"BLOCKED: {err}", file=sys.stderr)
        return 3
    report = mjcf.generate(expansion, out / "mujoco" / spec["configuration"])
    doc = benchmark.run_mujoco(out / "mujoco" / spec["configuration"] / report["mjcf"], expansion.text, mjcf.load_yaml(mjcf.OVERRIDES),
                               spec, arm_gravity_mode=opts.arm_gravity, wheel_contact=opts.wheel_contact)
    doc["expansion_sha256"] = expansion.sha256
    out.mkdir(parents=True, exist_ok=True)
    (out / f"{opts.name}.json").write_text(json.dumps(doc, indent=1) + "\n")
    print(json.dumps(doc["results"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
