#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Compare backend benchmark results (openamr.sim_benchmark.v1) against Gazebo, the reference, with the tolerances of scenarios.yaml.

  python3 simulation/tools/benchmark_compare.py --doc gazebo=build/bench/benchmark_gazebo.json --doc mujoco=build/bench/benchmark_mujoco.json \
      [--doc isaac=<Isaac result JSON>] --json comparison.json --markdown comparison.md

A missing or failed backend makes its rows BLOCKED (not a pass). A row beyond tolerance is FAIL: a finding to explain in the
declared differences, never a reason to tune a backend. A commanded joint move that a backend did not execute (measured travel,
not the controller's status) makes every row of that scenario FAIL. Exit 0 when every backend ran, 3 when any row is BLOCKED.
"""
import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1]))
from openamr_sim import benchmark as bm  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--doc", action="append", required=True, help="backend=path")
    parser.add_argument("--json")
    parser.add_argument("--markdown")
    opts = parser.parse_args()
    docs = {}
    for item in opts.doc:
        name, path = item.split("=", 1)
        p = pathlib.Path(path)
        docs[name] = json.loads(p.read_text()) if p.is_file() else None
    rows = bm.compare(docs)
    md = (bm.table(rows) +
          "\n\nCommanded joint moves, judged on measured travel (a controller's success is recorded, never taken as motion):\n\n" +
          bm.execution_table(docs))
    counts = {}
    for r in rows:
        for v in r["verdicts"].values():
            counts[v] = counts.get(v, 0) + 1
    if opts.json:
        pathlib.Path(opts.json).write_text(json.dumps({"rows": rows, "verdict_counts": counts,
                                                       "errors": {k: (d or {}).get("meta", {}).get("error") for k, d in docs.items()},
                                                       "execution": {k: (d or {}).get("execution") for k, d in docs.items()}},
                                                      indent=1) + "\n")
    if opts.markdown:
        pathlib.Path(opts.markdown).write_text(md + "\n")
    print(md)
    print(json.dumps(counts))
    return 3 if counts.get("BLOCKED") else 0


if __name__ == "__main__":
    sys.exit(main())
