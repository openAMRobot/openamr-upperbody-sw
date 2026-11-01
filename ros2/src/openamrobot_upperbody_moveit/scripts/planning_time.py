#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""How long do the twelve named-pose plans take, with a given SRDF? (P12A: the cost of enabling every pair that has no reason to be disabled.)

    ros2 run openamrobot_upperbody_moveit planning_time.py [--srdf PATH] [--repeats 10] [--base-plate bp000] [--json OUT]

Plans every named pose of both arms and both grippers, one after the other, offline (moveit_py, OMPL RRTConnect, the same requests as M06 at the
configured limits), `--repeats` times, and reports per pose the median, the best and the worst wall time and how many of the plans succeeded. Run it
with the old SRDF and the new one to see what the matrix costs: a false `Never` costs the planner nothing and hides a collision, a missing one costs
CPU, so the number is what the decision to enable trades against. RRTConnect is random, hence the repeats; the first plan of a process includes
building the planner, so one warm-up round is made and not counted. MoveIt does not expose a collision-check count to moveit_py's plan result (it
carries a planning time and a trajectory), so none is reported.

Leaves with os._exit after writing: moveit_py segfaults when it is torn down.
"""
import argparse
import json
import os
import statistics
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--srdf", default=None)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument("--base-plate", default="bp000", choices=("bp000", "bp050"))
    parser.add_argument("--json", default=None)
    args = parser.parse_args(argv)
    import limits_sensitivity as ls
    moveit_py = ls.make_moveit_py(args.base_plate, srdf_file=args.srdf)
    ls.plan_all(moveit_py, 1.0)                                    # warm-up: not counted
    runs = [ls.plan_all(moveit_py, 1.0) for _ in range(args.repeats)]
    poses = []
    for index, first in enumerate(runs[0]):
        walls = [run[index]["wall_s"] for run in runs]
        poses.append({"group": first["group"], "pose": first["pose"], "from": first["from"], "plans": len(walls),
                      "plans_ok": sum(run[index]["ok"] for run in runs), "median_s": round(statistics.median(walls), 4),
                      "best_s": round(min(walls), 4), "worst_s": round(max(walls), 4)})
    total = [round(sum(run[i]["wall_s"] for i in range(len(run))), 3) for run in runs]
    out = {"srdf": args.srdf or "the installed SRDF", "base_plate": args.base_plate, "repeats": args.repeats, "poses": poses,
           "sequence_total_s": {"median": round(statistics.median(total), 3), "best": min(total), "worst": max(total)},
           "all_plans_ok": all(p["plans_ok"] == p["plans"] for p in poses),
           "collision_check_count": "not exposed by moveit_py's plan result"}
    print(f"planning time, {args.repeats} repeats of the {len(poses)} named-pose plans, SRDF {out['srdf']}")
    for p in poses:
        print(f"  {p['group']:<14}{p['pose']:<10} from {p['from']:<9} ok {p['plans_ok']}/{p['plans']}  median {p['median_s']:.4f} s  worst {p['worst_s']:.4f} s")
    print(f"  whole sequence: median {out['sequence_total_s']['median']} s, worst {out['sequence_total_s']['worst']} s; all plans ok: {out['all_plans_ok']}")
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=1)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0 if out["all_plans_ok"] else 1)


if __name__ == "__main__":
    main()
