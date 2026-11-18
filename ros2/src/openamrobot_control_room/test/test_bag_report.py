# SPDX-License-Identifier: MIT
"""The report generator end to end on a rosbag2 bag: the synthetic fixture bag written by bag.write_synthetic. A recorded
Baseline 1 bag is reported by the CI job's dispatch input instead. The page shows that the generator works, nothing
about the robot."""
import os
import re
import subprocess
import sys

import pytest

import ros_support
from conftest import TRACKING, package_path

SCRIPT = package_path("scripts", "mission_report.py")


def _run(*args):
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([package_path(), os.environ.get("PYTHONPATH", "")]))
    return subprocess.run([sys.executable, SCRIPT, *args], capture_output=True, text=True, env=env, timeout=240)


def test_synthetic_bag_to_html(tmp_path):
    ros_support.require_rclpy()
    try:
        import rosbag2_py  # noqa: F401
    except ImportError as error:
        pytest.skip(f"BLOCKED: no rosbag2_py ({error}); tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    out = tmp_path / "report.html"
    done = _run("--synthetic", str(tmp_path / "synthetic"), "-o", str(out))
    print(done.stdout, done.stderr)
    assert done.returncode == 0
    page = out.read_text(encoding="utf-8")
    for chart in ("station", "loc", "joints_left", "joints_right", "lift", "rtf", "can"):
        match = re.search(rf'<svg id="{chart}" class="chart".*?</svg>', page, re.S)
        assert match and "<polyline" in match.group(0), f"{chart} has no plotted line"
    assert 'class="phase pass"' in page and "synthetic fixture" in page
    assert "/build1/ground_truth/odom" in page and "/amcl_pose" in page
    if os.environ.get("CONTROL_ROOM_REPORT_COPY"):
        with open(os.environ["CONTROL_ROOM_REPORT_COPY"], "w", encoding="utf-8") as f:
            f.write(page)

