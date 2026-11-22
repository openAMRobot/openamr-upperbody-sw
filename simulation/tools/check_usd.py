#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Check the Isaac Sim USD against the pinned expansion (structure and parameters). Needs usd-core, not Isaac Sim, and no GPU.

  source /opt/ros/jazzy/setup.bash
  python3 simulation/tools/check_usd.py --job build/sim/bp000/isaac/isaac_job.json \
      --platform-sw <checkout at the pin> --openarm-description <checkout at the pin> [--json report.json]

The expansion is computed again and checked against the lock, and the job must name the same hash. Exit 0 pass, 1 a check failed,
2 the job does not belong to this expansion, 3 BLOCKED (no USD: Isaac Sim was not run, which needs a GPU).
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import expand as ex  # noqa: E402
from openamr_sim import isaac  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--job", required=True)
    parser.add_argument("--platform-sw", required=True)
    parser.add_argument("--openarm-description", required=True)
    parser.add_argument("--json")
    opts = parser.parse_args()
    job_path = pathlib.Path(opts.job)
    job = json.loads(job_path.read_text())
    expansion = ex.expand(job["configuration"], opts.platform_sw, opts.openarm_description)
    if expansion.sha256 != job["expansion_sha256"]:
        print(f"FAIL: job {job['expansion_sha256']} is not this expansion {expansion.sha256}", file=sys.stderr)
        return 2
    usd = job_path.parent / job["usd"]
    if not usd.is_file():
        print(f"BLOCKED: {usd} does not exist. Isaac Sim {job['isaac_sim_version']} on a GPU machine writes it; "
              "see docs/simulators.md, section Isaac Sim. BLOCKED is not a pass.", file=sys.stderr)
        return 3
    report = isaac.check_usd(usd, expansion.text, job)
    summary = report.summary()
    if opts.json:
        pathlib.Path(opts.json).write_text(json.dumps({"summary": summary, "failures": report.failures}, indent=1, default=str) + "\n")
    print(json.dumps({"checks": summary["checks"], "failures": summary["failures"]}))
    for f in report.failures[:50]:
        print(f"FAIL {f['parameter']} {f['where']}: reference {f['reference']} usd {f['backend']}", file=sys.stderr)
    return 0 if not report.failures else 1


if __name__ == "__main__":
    sys.exit(main())
