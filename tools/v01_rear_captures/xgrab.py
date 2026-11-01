#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Runs the P19 window grabber (evidence/renders/rviz/xgrab.py) unchanged: the P19 capture script calls `HERE/xgrab.py`, and capture_rear.py
points HERE at this directory. usage: xgrab.py <window id, 0x..> <out.png>"""
import os
import runpy
import sys

sys.argv[0] = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, os.pardir, "evidence", "renders", "rviz", "xgrab.py")
runpy.run_path(sys.argv[0], run_name="__main__")
