#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""EXPERIMENT (diag/spin-test scratch branch): tables and effect estimates of the turn-slip experiment.

    spin_report.py <directory with spin_test_<variant>.json files, searched recursively> [--markdown out.md]

Per variant: one row per turn and per straight drive, then a summary. Each turn's residual (odometry minus ground truth)
is split into the drive chain (odometry minus the kinematic yaw of the Gazebo wheel angles) and the contact (kinematic
minus ground truth). Over the turns of a variant each part is fitted as  part = offset + scale * truth_yaw  (least
squares): offset is the direction-independent error per turn, scale the error proportional to the turn. The effect of
a cause is the baseline value minus the value of the variant that removes it. Simulation only.
"""
import argparse
import glob
import json
import os
import sys

ORDER = ["baseline", "A_iso", "A_axial", "B_sphere", "B_thin", "C_coplanar", "C_friction", "ALL"]
CAUSE = {"A_iso": "A (friction direction)", "A_axial": "A (friction direction)", "B_sphere": "B (flat wide contact)",
         "B_thin": "B (flat wide contact)", "C_coplanar": "C (castor clearance)", "C_friction": "C (frictionless castors)",
         "ALL": "A+B+C"}


def fit(xs, ys):
    """Least squares y = a + b x -> (a, b). Falls back to (mean, 0) with one distinct x."""
    n = len(xs)
    if n == 0:
        return None, None
    mx, my = sum(xs) / n, sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx < 1e-12:
        return my, 0.0
    b = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
    return my - b * mx, b


def f(value, digits=4):
    if value is None:
        return "n/a"
    if isinstance(value, bool):
        return str(value)
    return f"{value:+.{digits}f}" if isinstance(value, (int, float)) else str(value)


def load(directory):
    runs = {}
    for path in sorted(glob.glob(os.path.join(directory, "**", "spin_test_*.json"), recursive=True)):
        with open(path, encoding="utf-8") as handle:
            data = json.load(handle)
        variant = data.get("spin_variant") or os.path.basename(path)[len("spin_test_"):-5]
        runs[variant] = data
    return runs


def summary(data):
    segments = data.get("spin_segments") or []
    turns = [s for s in segments if s["kind"] == "turn"]
    straights = [s for s in segments if s["kind"] == "straight"]
    out = {"result": data.get("result"), "reason": data.get("reason"), "turns": len(turns), "straights": len(straights)}
    if turns:
        truth = [s["truth_yaw_rad"] for s in turns]
        ratios = [s["ratio_odom_truth"] for s in turns if s["ratio_odom_truth"] is not None]
        out.update(ratio_min=min(ratios), ratio_max=max(ratios),
                   summed_ratio=sum(abs(s["odom_yaw_rad"]) for s in turns) / sum(abs(t) for t in truth),
                   residual_mean=sum(s["residual"] for s in turns) / len(turns),
                   residual_abs_mean=sum(abs(s["residual"]) for s in turns) / len(turns),
                   residual_pos_turns=_mean([s["residual"] for s in turns if s["nominal"] > 0]),
                   residual_neg_turns=_mean([s["residual"] for s in turns if s["nominal"] < 0]))
        for part in ("residual", "drive_chain", "contact"):
            a, b = fit(truth, [s[part] for s in turns])
            out[f"{part}_offset"], out[f"{part}_scale"] = a, b
        out["truth_over_commanded_mean"] = _mean([s["truth_yaw_rad"] / s["nominal"] for s in turns])
        out["kinematic_over_demand_mean"] = _mean([s["kinematic_yaw_rad"] / s["demand_yaw_rad"] for s in turns if abs(s["demand_yaw_rad"]) > 1e-6])
        steady = [s["contacts_steady"] for s in turns if s.get("contacts_steady")]
        out["effective_track_mean"] = _mean([c.get("effective_track_m") for c in steady])
        out["effective_track_min"] = _min([c.get("effective_track_m") for c in steady])
        out["effective_track_max"] = _max([c.get("effective_track_m") for c in steady])
        out["caster_share_turns"] = _mean([c.get("caster_share") for c in steady])
        out["normal_left_turns"] = _mean([c.get("normal_left_N") for c in steady])
        out["normal_right_turns"] = _mean([c.get("normal_right_N") for c in steady])
        out["position_shift_mean"] = _mean([s["truth_distance_m"] for s in turns])
    if straights:
        out["straight_ratio_mean"] = _mean([s["odom_distance_m"] / s["truth_distance_m"] for s in straights if s["truth_distance_m"] > 0.1])
        out["straight_heading_truth_mean"] = _mean([s["heading_drift_truth_rad"] for s in straights])
        out["straight_heading_odom_minus_truth_mean"] = _mean([s["heading_drift_odom_rad"] - s["heading_drift_truth_rad"] for s in straights])
    rest = data.get("rest_contacts") or {}
    out["caster_share_rest"] = rest.get("caster_share")
    out["normal_total_rest"] = rest.get("normal_total_N")
    out["effective_track_rest"] = rest.get("effective_track_m")
    out["forces_reported"] = rest.get("forces_reported")
    out["mass_kg"] = (data.get("geometry") or {}).get("mass_kg")
    return out


def _clean(values):
    return [v for v in values if v is not None]


def _mean(values):
    values = _clean(values)
    return sum(values) / len(values) if values else None


def _min(values):
    values = _clean(values)
    return min(values) if values else None


def _max(values):
    values = _clean(values)
    return max(values) if values else None


def variant_table(variant, data):
    lines = [f"### Variant {variant}", ""]
    lines.append(f"Result: {data.get('result')} {data.get('reason') or ''}".rstrip())
    edits = data.get("spin_variant_edits") or []
    lines.append(f"Edits: {'; '.join(edits) if edits else 'none recorded'}")
    rest = data.get("rest_contacts") or {}
    if rest:
        lines.append(f"Rest (arms parked): normal left {f(rest.get('normal_left_N'), 1)} N, right {f(rest.get('normal_right_N'), 1)} N, "
                     f"castors {f(rest.get('normal_caster_N'), 1)} N, other {f(rest.get('normal_other_N'), 1)} N, castor share "
                     f"{f(rest.get('caster_share'), 3)}, effective track {f(rest.get('effective_track_m'), 4)} m")
    lines.append("")
    lines.append("| Turn | Rep | Commanded | Demand L, R (rad) | Wheels L, R (rad) | Kinematic | Odometry | Truth | Odom/truth | Residual | "
                 "Drive chain | Contact | N left, right, castors (N) | Track eff (m) | Castor share |")
    lines.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in data.get("spin_segments") or []:
        if s["kind"] != "turn":
            continue
        c = s.get("contacts_steady") or {}
        lines.append(f"| {s['name']} | {s['repeat']} | {f(s['nominal'], 3)} | {f(s['demand_left_rad'], 3)}, {f(s['demand_right_rad'], 3)} | "
                     f"{f(s['wheel_left_rad'], 3)}, {f(s['wheel_right_rad'], 3)} | {f(s['kinematic_yaw_rad'])} | {f(s['odom_yaw_rad'])} | "
                     f"{f(s['truth_yaw_rad'])} | {f(s['ratio_odom_truth'], 3)} | {f(s['residual'])} | {f(s['drive_chain'])} | "
                     f"{f(s['contact'])} | {f(c.get('normal_left_N'), 0)}, {f(c.get('normal_right_N'), 0)}, {f(c.get('normal_caster_N'), 0)} | "
                     f"{f(c.get('effective_track_m'), 4)} | {f(c.get('caster_share'), 3)} |")
    lines.append("")
    lines.append("| Straight | Rep | Commanded (m) | Truth (m) | Odometry (m) | Kinematic (m) | Odom/truth | Heading truth (rad) | Heading odom (rad) |")
    lines.append("|---|---|---|---|---|---|---|---|---|")
    for s in data.get("spin_segments") or []:
        if s["kind"] != "straight":
            continue
        ratio = s["odom_distance_m"] / s["truth_distance_m"] if s["truth_distance_m"] > 0.1 else None
        lines.append(f"| {s['name']} | {s['repeat']} | {f(s['nominal'], 2)} | {s['truth_distance_m']:.4f} | {s['odom_distance_m']:.4f} | "
                     f"{s['kinematic_distance_m']:.4f} | {f(ratio, 4)} | {f(s['heading_drift_truth_rad'])} | {f(s['heading_drift_odom_rad'])} |")
    lines.append("")
    return lines


def summary_table(runs):
    rows = {v: summary(runs[v]) for v in runs}
    keys = [v for v in ORDER if v in rows] + sorted(v for v in rows if v not in ORDER)
    lines = ["### Summary per variant", "",
             "| Variant | Result | Turns | Odom/truth min..max | Summed ratio | Mean residual (rad) | + turns | - turns | Residual offset (rad/turn) | "
             "Residual scale | Drive chain offset, scale | Contact offset, scale | Track eff mean (min..max) m | Castor share rest, turns | "
             "Straight odom/truth | Straight heading odom-truth (rad) |",
             "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for v in keys:
        r = rows[v]
        if not r.get("turns"):
            lines.append(f"| {v} | {r['result']} | 0 | | | | | | | | | | | | | |")
            continue
        lines.append(f"| {v} | {r['result']} | {r['turns']} | {r['ratio_min']:.3f}..{r['ratio_max']:.3f} | {r['summed_ratio']:.4f} | "
                     f"{f(r['residual_mean'])} | {f(r['residual_pos_turns'])} | {f(r['residual_neg_turns'])} | {f(r['residual_offset'])} | "
                     f"{f(r['residual_scale'])} | {f(r['drive_chain_offset'])}, {f(r['drive_chain_scale'])} | {f(r['contact_offset'])}, "
                     f"{f(r['contact_scale'])} | {f(r['effective_track_mean'], 4)} ({f(r['effective_track_min'], 4)}..{f(r['effective_track_max'], 4)}) | "
                     f"{f(r['caster_share_rest'], 3)}, {f(r['caster_share_turns'], 3)} | {f(r.get('straight_ratio_mean'), 4)} | "
                     f"{f(r.get('straight_heading_odom_minus_truth_mean'))} |")
    lines.append("")
    base = rows.get("baseline")
    if base and base.get("turns"):
        lines += ["### Effect estimate per cause (baseline minus variant)", "",
                  "| Variant | Cause removed | Residual offset removed (rad/turn) | Residual scale removed | Ratio spread removed | "
                  "Mean abs residual removed (rad) |", "|---|---|---|---|---|---|"]
        spread = base["ratio_max"] - base["ratio_min"]
        for v in keys:
            r = rows[v]
            if v == "baseline" or not r.get("turns"):
                continue
            lines.append(f"| {v} | {CAUSE.get(v, '')} | {f(base['residual_offset'] - r['residual_offset'])} | "
                         f"{f(base['residual_scale'] - r['residual_scale'])} | {f(spread - (r['ratio_max'] - r['ratio_min']), 3)} | "
                         f"{f(base['residual_abs_mean'] - r['residual_abs_mean'])} |")
        lines.append("")
    return lines, rows


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("directory")
    parser.add_argument("--markdown")
    parser.add_argument("--json")
    args = parser.parse_args(argv)
    runs = load(args.directory)
    if not runs:
        print(f"no spin_test_*.json under {args.directory}")
        return 1
    lines, rows = summary_table(runs)
    for v in [v for v in ORDER if v in runs] + sorted(v for v in runs if v not in ORDER):
        lines += variant_table(v, runs[v])
    text = "\n".join(lines)
    print(text)
    if args.markdown:
        with open(args.markdown, "w", encoding="utf-8") as handle:
            handle.write(text + "\n")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as handle:
            json.dump(rows, handle, indent=1, sort_keys=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
