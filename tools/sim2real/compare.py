#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Sim session(s) + robot session(s) + manual bench CSVs -> report.md with deltas and the Proposed pass bands.

    compare.py --sim SIM_SESSION [...] --robot ROBOT_SESSION [...] --manual MANUAL_DIR --out report.md [--bands bands.yaml]

A session directory holds metrics.csv (test, metric, value, unit, source) written by t1..t5. MANUAL_DIR holds the filled templates
(templates/*.csv). The bands are bands.yaml next to this file unless --bands is given: every band is Proposed until the owner accepts it.

Verdict per band row: PASS (value inside the band), FAIL (outside, or not finite), MISSING (no value: a missing file or metric is never
a PASS). Exit 0 when nothing FAILs and nothing is MISSING, 1 when something FAILs, 2 when only MISSING rows keep it from 0.
Standard library and PyYAML only.
"""
import argparse
import csv
import math
import os
import re
import sys

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))


def floor_turn_angle(a0, b0, a1, b1):
    """theta = atan2(u0 x u1, u0 . u1), u = B - A (the same formula as common.floor_turn_angle)."""
    u0 = (b0[0] - a0[0], b0[1] - a0[1])
    u1 = (b1[0] - a1[0], b1[1] - a1[1])
    return math.atan2(u0[0] * u1[1] - u0[1] * u1[0], u0[0] * u1[0] + u0[1] * u1[1])


def read_metrics(session_dirs):
    out, missing = {}, []
    for d in session_dirs or []:
        p = os.path.join(d, "metrics.csv")
        if not os.path.isfile(p):
            missing.append(p)
            continue
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                try:
                    out[r["metric"]] = (float(r["value"]), r["unit"], d)
                except ValueError:
                    out[r["metric"]] = (math.nan, r["unit"], d)
    return out, missing


def _rows(path):
    with open(path, newline="") as f:
        return [r for r in csv.DictReader(f) if any((v or "").strip() for v in r.values())]


def _f(r, k):
    v = (r.get(k) or "").strip()
    return float(v) if v else math.nan


def read_manual(manual_dir, lift_speed_mps):
    """Bench metrics derived from the filled templates. Returns ({metric: (value, unit, file)}, [missing files])."""
    out, missing = {}, []
    if not manual_dir:
        return out, ["(no --manual directory)"]
    files = {k: os.path.join(manual_dir, f"{k}_manual.csv") for k in ("t1_lift", "t2_base_straight", "t3_base_rotate", "t5_gripper")}
    for k, p in files.items():
        if not os.path.isfile(p):
            missing.append(p)
    if os.path.isfile(files["t1_lift"]):
        rows = _rows(files["t1_lift"])
        base = next((_f(r, "plate_top_height_m") for r in rows if r["stop"] == "L1000"), math.nan)
        prev = 0.0
        for r in rows:
            z, h, tt = _f(r, "target_m"), _f(r, "plate_top_height_m"), _f(r, "travel_time_s")
            out[f"T1_bench_stroke_error_{r['stop']}_{z:.3f}"] = ((h - base) - z, "m", p)
            if abs(z - prev) > 0 and math.isfinite(tt) and tt > 0:
                out[f"T1_bench_speed_ratio_{r['stop']}_from_{prev:.3f}"] = (abs(z - prev) / tt / lift_speed_mps, "1", files["t1_lift"])
            out[f"T1_bench_sag_{r['stop']}_{z:.3f}"] = (_f(r, "plate_rear_edge_height_m") - _f(r, "plate_front_edge_height_m"), "m", files["t1_lift"])
            prev = z
    if os.path.isfile(files["t2_base_straight"]):
        for r in _rows(files["t2_base_straight"]):
            c, d = _f(r, "commanded_m"), _f(r, "tape_along_m")
            out[f"T2_bench_{r['leg']}_real_over_commanded"] = (abs(d) / c if c else math.nan, "1", files["t2_base_straight"])
            out[f"T2_bench_{r['leg']}_lateral_m"] = (_f(r, "tape_lateral_m"), "m", files["t2_base_straight"])
            o = _f(r, "odom_m")
            if math.isfinite(o):
                out[f"T2_bench_{r['leg']}_odom_over_real"] = (o / abs(d) if d else math.nan, "1", files["t2_base_straight"])
    if os.path.isfile(files["t3_base_rotate"]):
        for r in _rows(files["t3_base_rotate"]):
            deg = _f(r, "angle_deg")
            th = floor_turn_angle((_f(r, "a0_x_m"), _f(r, "a0_y_m")), (_f(r, "b0_x_m"), _f(r, "b0_y_m")),
                                  (_f(r, "a1_x_m"), _f(r, "a1_y_m")), (_f(r, "b1_x_m"), _f(r, "b1_y_m")))
            if deg >= 179 and th < 0:
                th += 2 * math.pi
            out[f"T3_bench_turn{deg:.0f}_real_over_commanded"] = (th / math.radians(deg), "1", files["t3_base_rotate"])
    if os.path.isfile(files["t5_gripper"]):
        for r in _rows(files["t5_gripper"]):
            out[f"T5_bench_{r['side']}_{r['state']}_jaw_gap_m"] = (_f(r, "caliper_jaw_mm") / 1000.0, "m", files["t5_gripper"])
    # a blank cell is "not measured": MISSING in the report, never a non-finite value that would read as FAIL
    return {k: v for k, v in out.items() if math.isfinite(v[0])}, missing


def verdict(value, band):
    if value is None:
        return "MISSING"
    if not math.isfinite(value):
        return "FAIL"
    lo, hi = band
    return "PASS" if (lo is None or value >= lo) and (hi is None or value <= hi) else "FAIL"


def build_report(sim, robot, manual, bands, missing):
    lines = ["# Sim-to-real comparison", "",
             "Evidence class: simulation reference and bench measurement. Neither is safety evidence. Every band is **Proposed** until its owner accepts it.", ""]
    if missing:
        lines += ["## Missing inputs", ""] + [f"- `{m}`" for m in missing] + [""]
    lines += ["## Banded metrics", "", "| test | metric | sim | robot | bench | delta (robot or bench - sim) | band (Proposed) | verdict |", "|---|---|---|---|---|---|---|---|"]
    counts = {"PASS": 0, "FAIL": 0, "MISSING": 0}
    for b in bands["bands"]:
        rx = re.compile(b["metric"])
        keys = sorted({k for k in list(sim) + list(robot) + list(manual) if rx.fullmatch(k)})
        if not keys:
            keys = [b["metric"]]
        for k in keys:
            sv = sim.get(k, (None,))[0]
            rv = robot.get(k, (None,))[0]
            bk = b.get("bench_metric")
            mv = manual.get(re.sub(b["metric"], bk, k) if bk else k, (None,))[0]
            ref = mv if mv is not None else rv
            delta = (ref - sv) if (ref is not None and sv is not None and math.isfinite(ref) and math.isfinite(sv)) else None
            judge = b.get("judge", "robot")      # robot | bench | delta (bench or robot minus sim)
            measured = {"robot": rv, "bench": mv, "delta": delta}[judge]
            v = verdict(measured, (b.get("lo"), b.get("hi")))
            counts[v] += 1
            fmt = lambda x: "" if x is None else f"{x:.6g}"
            lines.append(f"| {b['test']} | `{k}` | {fmt(sv)} | {fmt(rv)} | {fmt(mv)} | {fmt(delta)} | [{fmt(b.get('lo'))}, {fmt(b.get('hi'))}] {b.get('unit', '')} | {v} |")
    lines += ["", f"Totals: {counts['PASS']} PASS, {counts['FAIL']} FAIL, {counts['MISSING']} MISSING.", "",
              "## All metrics", "", "| metric | sim | robot | bench |", "|---|---|---|---|"]
    for k in sorted(set(sim) | set(robot) | set(manual)):
        fmt = lambda d: "" if k not in d else f"{d[k][0]:.6g} {d[k][1]}"
        lines.append(f"| `{k}` | {fmt(sim)} | {fmt(robot)} | {fmt(manual)} |")
    lines += ["", "Sign-off: the test owner named in docs/sim_to_real_protocol.md, section Report.", ""]
    return "\n".join(lines), counts


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", nargs="*", default=[])
    p.add_argument("--robot", nargs="*", default=[])
    p.add_argument("--manual", default=None)
    p.add_argument("--bands", default=os.path.join(HERE, "bands.yaml"))
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    with open(a.bands) as f:
        bands = yaml.safe_load(f)
    sim, m1 = read_metrics(a.sim)
    robot, m2 = read_metrics(a.robot)
    manual, m3 = read_manual(a.manual, bands["lift_speed_mps"])
    text, counts = build_report(sim, robot, manual, bands, m1 + m2 + m3)
    with open(a.out, "w") as f:
        f.write(text)
    print(f"{a.out}: {counts}")
    return 1 if counts["FAIL"] else (2 if counts["MISSING"] else 0)


if __name__ == "__main__":
    sys.exit(main())
