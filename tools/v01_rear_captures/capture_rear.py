#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""V01 rear captures: one RViz shot from behind per lift stop (L1000, L1175, L1350) at bp000, so the second E-stop on the static column's rear
face is visible. Reuses the P19 capture script (evidence/renders/rviz/capture_rviz.py) unchanged: its shoot() launches `HERE/capture.launch.py`,
so HERE is pointed at this directory, whose capture.launch.py opens rear_capture.rviz (and whose xgrab.py runs the P19 one). Same records as P19 (lift read back from /joint_states,
processes left running).

    source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash
    python3 tools/v01_rear_captures/capture_rear.py <outdir> [L1000:bp000 ...]

Writes <outdir>/rviz_<stop>_<bp>.png and .json (use a directory of its own, e.g. v01_rviz_captures/rear). Pictures for a person, not evidence.
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, os.pardir, os.pardir, "evidence", "renders", "rviz"))
import capture_rviz  # noqa: E402  (the P19 script, unchanged)

capture_rviz.HERE = HERE

if __name__ == "__main__":
    outdir = sys.argv[1]
    os.makedirs(outdir, exist_ok=True)
    wanted = sys.argv[2:] or [f"{s}:bp000" for s in capture_rviz.STOPS]
    for i, item in enumerate(wanted):
        stop, bp = item.split(":")
        capture_rviz.shoot(stop, bp, 181 + i, outdir)
