#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Per-run tables of the arm diagnostics: one row per diag_grasp and per return-to-park move.

    diag_table.py DIR [DIR ...] [--summary FILE]

Each DIR holds the scenario logs of one run (diag_grasp.json, diag_reach_return.json; BUILD1_LOG_DIR). The run label
is the DIR name. Prints Markdown and appends it to FILE when given. Reads JSON only; no ROS import.
Simulation only: the numbers are the simulator's, not a gripper's or an arm's.
"""
import argparse
import json
import os
import sys


def _load(directory, name):
    path = os.path.join(directory, f"{name}.json")
    if not os.path.isfile(path):
        return None
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _quantity(log, name):
    for entry in log.get("quantities", []):
        if entry["name"] == name:
            return entry["value"]
    return None


def _f(value, digits=3):
    if value is None:
        return "-"
    if isinstance(value, bool):
        return str(value)
    if isinstance(value, (int, float)):
        return f"{value:.{digits}f}"
    return str(value)


GRASP_COLUMNS = (
    ("result", None, 0), ("finger q rad", "q", 3), ("f1 N", "finger1_normal_N_mean", 2), ("f2 N", "finger2_normal_N_mean", 2),
    ("f1 T N", "finger1_tangential_sum_N_mean", 2), ("f2 T N", "finger2_tangential_sum_N_mean", 2),
    ("f1 pts", "finger1_points_per_message_mean", 1), ("f2 pts", "finger2_points_per_message_mean", 1),
    ("need N", "required_normal_per_finger_N", 3), ("tau1 emu", "emulator_finger_torque_mean_Nm", 2), ("kp*err", "emulator_finger_kp_times_error_Nm", 2),
    ("tau1 phys", "openarm_left_finger_joint1_physics_effort_mean_Nm", 2),
    ("tau2 phys", "openarm_left_finger_joint2_physics_effort_mean_Nm", 2), ("tau1 drv", "finger_joint1_driver_effort_mean_Nm", 2),
    ("hand dz", "hand_rise_m", 4), ("TCP dz", "tcp_rise_m", 4), ("item dz", "item_rise_m", 4),
    ("item-TCP dz", "item_rise_minus_tcp_rise_m", 4), ("slip", "slip_total_m", 4), ("slip onset s", "slip_onset_s_after_lift_start", 2),
    ("hand rot deg", "hand_rotation_deg", 1), ("item tilt deg", "item_tilt_deg", 1), ("TCP acc", "tcp_peak_vertical_accel_mps2", 2),
    ("f1 on limit", "openarm_left_finger_joint1_on_limit_fraction", 2),
    ("q2-q1 rad", "finger_joint2_minus_joint1_rad", 3))


def grasp_rows(runs):
    lines = ["| run | " + " | ".join(c[0] for c in GRASP_COLUMNS) + " | gain check |", "|" + "---|" * (len(GRASP_COLUMNS) + 2)]
    for label, directory in runs:
        log = _load(directory, "diag_grasp")
        if log is None:
            lines.append(f"| {label} | no diag_grasp.json |" + " |" * len(GRASP_COLUMNS))
            continue
        numbers = (log.get("diagnostics") or {}).get("grasp") or {}
        cells = []
        for _, key, digits in GRASP_COLUMNS:
            if key is None:
                cells.append(log.get("result") or "-")
            elif key == "q":
                cells.append(_f(_quantity(log, "finger_position_on_item"), digits))
            else:
                cells.append(_f(numbers.get(key), digits))
        lines.append(f"| {label} | " + " | ".join(cells) + f" | {_f(_quantity(log, 'item_height_gain_after_lift'), 4)} |")
    return lines


def return_rows(runs):
    lines = ["| run | move | from park rad | worst joint effort N m | limit hits | path dev rad | plan limit margin rad | contacts (link~object: max N) |",
             "|---|---|---|---|---|---|---|---|"]
    for label, directory in runs:
        for name in ("diag_grasp", "diag_reach_return"):
            log = _load(directory, name)
            if log is None:
                continue
            for move, numbers in ((log.get("diagnostics") or {}).get("returns") or {}).items():
                efforts = {k[:-len("_max_abs_effort_Nm")]: v for k, v in numbers.items() if k.endswith("_max_abs_effort_Nm") and v is not None}
                worst = max(efforts, key=efforts.get) if efforts else None
                hits = [f"{k[:-len('_limit_hit')]}:{v}" for k, v in numbers.items() if k.endswith("_limit_hit") and v]
                contacts = numbers.get("contacts_max_force_N") or {}
                lines.append(f"| {label} | {move} | {_f(_quantity(log, f'{move}_distance_from_park'), 3)} | "
                             f"{f'{worst} {efforts[worst]:.1f}' if worst else '-'} | {', '.join(hits) or '-'} | "
                             f"{_f(numbers.get('path_deviation_rad'), 3)} | {_f(numbers.get('planned_limit_margin_rad'), 3)} | "
                             f"{', '.join(f'{k}: {v}' for k, v in contacts.items()) or 'none'} |")
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("dirs", nargs="+")
    parser.add_argument("--summary")
    args = parser.parse_args(argv)
    runs = [(os.path.basename(os.path.normpath(d)), d) for d in sorted(args.dirs)]
    text = "\n".join(["### Grasp and lift (diag_grasp)", "", *grasp_rows(runs), "",
                      "### Return to park (diag_grasp retreat, diag_reach_return)", "", *return_rows(runs), ""])
    print(text)
    if args.summary:
        with open(args.summary, "a", encoding="utf-8") as f:
            f.write(text + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
