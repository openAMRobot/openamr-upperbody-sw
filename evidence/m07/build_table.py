#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Builds the numbers of evidence/m07/per_joint_rest_table.md from files that are committed. No simulator, nothing measured here.

    python3 evidence/m07/build_table.py            # from the repository root; writes evidence/m07/rest_readings.json, joint_limits.json and
                                                   # fills the generated tables between the BEGIN/END markers of per_joint_rest_table.md

Inputs (all under evidence/):
  current body   reports/20261021T085748Z_a7.json, 20261021T090537Z_a7.json, 20261021T091325Z_a7.json  (A7 on 1165166, M07's own `joints` rows:
                 min and max of the raw /arm/joint_states over the 6.0 s window, 600 readings per joint) and, as a second set,
                 reports/20261020T120539Z_a7.json, 20261020T121325Z_a7.json, 20261020T122112Z_a7.json (A7 on 15109f9)
  old body       m02_bisect/effort_old1.json, effort_old2.json, effort_old3.json  (P17: 7daa2ab, 300 readings per joint after 25 s of sim time)
  limits         m07/expanded_new_gazebo_ba5b4ea.urdf, m07/expanded_old_7daa2ab_gazebo.urdf, m07/expanded_upstream_openarm_v20_14ff67b.urdf
Distance convention is M07's (test_m07_rest_position_margin.py, rest_rows): min(lowest reading - lower, upper - highest reading); negative = read past a limit.
"""
import json
import os
import re
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
EV = os.path.dirname(HERE)
JOINTS = ["openarm_left_finger_joint1", "openarm_right_finger_joint1", "openarm_left_joint4", "openarm_right_joint4"]
NEW_SET = ["reports/20261021T085748Z_a7.json", "reports/20261021T090537Z_a7.json", "reports/20261021T091325Z_a7.json"]
P24_SET = ["reports/20261020T120539Z_a7.json", "reports/20261020T121325Z_a7.json", "reports/20261020T122112Z_a7.json"]
OLD_SET = ["m02_bisect/effort_old1.json", "m02_bisect/effort_old2.json", "m02_bisect/effort_old3.json"]
# Line numbers read from the two trees (git show 7daa2ab:...ros2_control.xacro, and the working tree at ba5b4ea): the `initial_value` of the position state interface.
STARTUP = {"current_P25": "`ros2_control.xacro:43`, `initial_value` 0.0 (see section 2)", "old_7daa2ab": "`ros2_control.xacro:42` at 7daa2ab, `initial_value` 0.0 (see section 2)"}
URDFS = {"current": "m07/expanded_new_gazebo_ba5b4ea.urdf", "old": "m07/expanded_old_7daa2ab_gazebo.urdf", "upstream": "m07/expanded_upstream_openarm_v20_14ff67b.urdf"}


def limits(path):
    root = ET.parse(os.path.join(EV, path)).getroot()
    out = {}
    for j in root.findall("joint"):                       # direct children only: <ros2_control> repeats the names without limits
        lim = j.find("limit")
        if lim is not None and lim.get("lower") is not None:
            out[j.get("name")] = (float(lim.get("lower")), float(lim.get("upper")))
    return out


def row(low, high, lo, up, n):
    to_lo, to_up = low - lo, up - high
    return {"min_rad": low, "max_rad": high, "readings": n, "to_lower_rad": to_lo, "to_upper_rad": to_up,
            "nearest": "lower" if to_lo <= to_up else "upper", "distance_rad": min(to_lo, to_up)}


def m07_rows(path):
    d = json.load(open(os.path.join(EV, path)))
    m = [c for c in d["checks"] if c["id"] == "M07"][0]["details"]
    return {"git_hash": d["git_hash"], "dirty": d["dirty"], "window_sim_s": m["window_sim_s"], "settle_sim_s": m["settle_sim_s"],
            "messages_in_window": m["messages_in_window"], "rows": {r["joint"]: r for r in m["joints"]}}


def old_rows(path):
    d = json.load(open(os.path.join(EV, path)))
    return {"messages": d["messages"], "rows": {j: d["joints"][j] for j in JOINTS}}


def fmt(x):
    return f"{x:+.3e}"


def main():
    lim = {k: limits(v) for k, v in URDFS.items()}
    for j in JOINTS:                                      # the three expansions must agree on these four joints, or the table is wrong
        assert lim["current"][j] == lim["old"][j] == lim["upstream"][j], j
    names = sorted(n for n in lim["upstream"] if n.startswith("openarm_"))     # every OpenArm joint with limits in upstream's own expansion
    assert len(names) == 18 and all(lim["current"][n] == lim["old"][n] == lim["upstream"][n] for n in names), "limits differ between expansions"
    data = {"all_openarm_joints_with_limits_identical_in_the_three_expansions": len(names), "limits_rad": {k: {j: lim[k][j] for j in JOINTS} for k in lim}, "runs": {}}
    for label, files, kind in (("current_P25", NEW_SET, "m07"), ("current_P24", P24_SET, "m07"), ("old_7daa2ab", OLD_SET, "old")):
        runs = []
        for f in files:
            if kind == "m07":
                r = m07_rows(f)
                joints = {j: row(r["rows"][j]["min_rad"], r["rows"][j]["max_rad"], *lim["current"][j], r["rows"][j]["readings"]) for j in JOINTS}
                runs.append({"file": "evidence/" + f, "git_hash": r["git_hash"], "dirty": r["dirty"], "window_sim_s": r["window_sim_s"],
                             "settle_sim_s": r["settle_sim_s"], "messages_in_window": r["messages_in_window"], "joints": joints})
            else:
                r = old_rows(f)
                joints = {j: row(r["rows"][j]["position"]["min"], r["rows"][j]["position"]["max"], *lim["old"][j], r["rows"][j]["n"]) for j in JOINTS}
                runs.append({"file": "evidence/" + f, "messages": r["messages"], "joints": joints})
        data["runs"][label] = runs
    json.dump(data, open(os.path.join(HERE, "rest_readings.json"), "w"), indent=1)
    json.dump({k: {j: {"lower": v[0], "upper": v[1]} for j, v in lim[k].items() if j in JOINTS or "finger_joint2" in j} for k in lim},
              open(os.path.join(HERE, "joint_limits.json"), "w"), indent=1)

    def agg(runs, j, body):
        lo, up = lim[body][j]
        rs = [r["joints"][j] for r in runs]
        low, high = min(r["min_rad"] for r in rs), max(r["max_rad"] for r in rs)
        dists = [r["distance_rad"] for r in rs]
        near = sorted({r["nearest"] for r in rs})
        return lo, up, low, high, rs[0]["readings"], min(dists), max(dists), "/".join(near)

    main_rows = []
    for body, label, key in (("current", "current body (ba5b4ea, lift)", "current_P25"), ("old", "old body (7daa2ab, mast)", "old_7daa2ab")):
        for j in JOINTS:
            lo, up, low, high, n, dmin, dmax, near = agg(data["runs"][key], j, body)
            src = STARTUP[key]
            main_rows.append(f"| {label} | `{j}` | {lo:+.4f} | {up:+.4f} | {fmt(low)} | {fmt(high)} | {n} x 3 runs | {near} | {fmt(dmin)} .. {fmt(dmax)} | {src} |")
    t1 = ("| body | joint | URDF lower [rad] | URDF upper [rad] | lowest raw reading, 3 runs [rad] | highest raw reading, 3 runs [rad] | readings per joint, runs "
          "| nearest limit | signed distance to it, smallest .. largest over the 3 runs [rad] (negative = past the limit) | startup position set by |\n|---|---|---|---|---|---|---|---|---|---|\n" + "\n".join(main_rows))

    per_run = []
    for key, label in (("current_P25", "current, 1165166"), ("current_P24", "current, 15109f9"), ("old_7daa2ab", "old, 7daa2ab")):
        for i, run in enumerate(data["runs"][key], 1):
            for j in JOINTS:
                r = run["joints"][j]
                per_run.append(f"| {label} run {i} | `{j}` | {fmt(r['min_rad'])} | {fmt(r['max_rad'])} | {r['readings']} | {r['nearest']} | {fmt(r['distance_rad'])} | `{run['file']}` |")
    t2 = ("| set, run | joint | lowest [rad] | highest [rad] | readings | nearest limit | signed distance [rad] | file |\n|---|---|---|---|---|---|---|---|\n" + "\n".join(per_run))

    path = os.path.join(HERE, "per_joint_rest_table.md")
    if os.path.exists(path):
        s = open(path).read()
        for name, text in (("table1", t1), ("table2", t2)):
            s, n = re.subn(rf"<!-- BEGIN {name} -->\n.*?<!-- END {name} -->", lambda m: f"<!-- BEGIN {name} -->\n{text}\n<!-- END {name} -->", s, flags=re.S)
            assert n == 1, name
        open(path, "w").write(s)
    print(t1)
    print()
    print(t2)


main()
