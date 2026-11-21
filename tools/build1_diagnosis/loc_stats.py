# SPDX-License-Identifier: MIT
"""Localization statistics of Baseline 1 runs from their job logs (docs/integration/station_approach.md).
    loc_stats.py JOB_LOG [JOB_LOG ...]
"""
import re, sys, math, statistics as st
KEYS = ["nav_localization_minus_ground_truth", "nav_localization_yaw_minus_ground_truth", "loc_amcl_pose_minus_ground_truth",
        "loc_amcl_pose_yaw_minus_ground_truth", "loc_amcl_sigma_x", "loc_amcl_sigma_y", "loc_amcl_sigma_yaw",
        "loc_amcl_updates_during_nav", "loc_amcl_last_update_age", "nav_odometry_minus_ground_truth",
        "nav_odometry_yaw_minus_ground_truth", "loc_scan_beams_with_return", "scan_no_return_inf",
        "station_reach_position_error", "station_reach_yaw_error", "nav_localization_position_error"]
T95 = {4: 2.132, 5: 2.015, 6: 1.943, 7: 1.895, 8: 1.860, 9: 1.833, 10: 1.812, 11: 1.796, 12: 1.782, 13: 1.771, 14: 1.761}
rows = {}
for path in sys.argv[1:]:
    run = re.search(r"(\d{11})", path).group(1); vals = {}
    for line in open(path, errors="replace"):
        m = re.search(r"baseline1_mission (\w+) = (-?[0-9.e+-]+)", line)
        if m and m.group(1) in KEYS and m.group(1) not in vals:
            vals[m.group(1)] = float(m.group(2))
    rows[run] = vals
print("run | " + " | ".join(KEYS))
for run, v in rows.items():
    print(run + " | " + " | ".join(f"{v[k]:.4f}" if k in v else "-" for k in KEYS))
print("\nstat | n | mean | sd | max | 95% one-sided prediction bound (mean + t*sd*sqrt(1+1/n))")
for k in KEYS:
    xs = [v[k] for v in rows.values() if k in v]
    if len(xs) >= 2:
        n = len(xs); m = st.mean(xs); sd = st.stdev(xs)
        b = m + T95.get(n - 1, 1.645) * sd * math.sqrt(1 + 1 / n)
        print(f"{k} | {n} | {m:.4f} | {sd:.4f} | {max(xs):.4f} | {b:.4f}")
