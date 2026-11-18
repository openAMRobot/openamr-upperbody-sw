#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Make the Gazebo GUI camera follow a model, through the GUI's camera tracking services.

    gz_follow.py [--entity openamrobot] [--offset -2.5,0.0,2.0] [--timeout 600]

Waits until the Gazebo window offers /gui/follow, then calls /gui/follow (the model name) and /gui/follow/offset
(the camera offset from the model, m). If the GUI offers only /gui/track, that is used instead. A view aid for a human
watching the run: it commands nothing in the simulation. Without a GUI it gives up after --timeout wall seconds.
"""
import argparse
import subprocess
import sys
import time


def services():
    done = subprocess.run(["gz", "service", "-l"], capture_output=True, text=True, timeout=20, check=False)
    return set(done.stdout.split())


def call(service, reqtype, request):
    done = subprocess.run(["gz", "service", "-s", service, "--reqtype", reqtype, "--reptype", "gz.msgs.Boolean",
                           "--timeout", "3000", "--req", request], capture_output=True, text=True, timeout=20, check=False)
    return done.returncode == 0 and "true" in done.stdout


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--entity", default="openamrobot")
    parser.add_argument("--offset", default="-2.5,0.0,2.0")
    parser.add_argument("--timeout", type=float, default=600.0)
    args, _ros = parser.parse_known_args(argv)
    x, y, z = (float(v) for v in args.offset.split(","))
    end = time.monotonic() + args.timeout
    while time.monotonic() < end:
        try:
            offered = services()
        except (OSError, subprocess.TimeoutExpired):
            offered = set()
        if "/gui/follow" in offered:
            if call("/gui/follow", "gz.msgs.StringMsg", f'data: "{args.entity}"'):
                call("/gui/follow/offset", "gz.msgs.Vector3d", f"x: {x}, y: {y}, z: {z}")
                print(f"gz_follow: the Gazebo camera follows {args.entity}, offset {x} {y} {z} m", flush=True)
                return 0
        if "/gui/track" in offered:
            request = (f'track_mode: FOLLOW, follow_target: {{name: "{args.entity}"}}, '
                       f"follow_offset: {{x: {x}, y: {y}, z: {z}}}")
            if call("/gui/track", "gz.msgs.CameraTrack", request):
                print(f"gz_follow: the Gazebo camera tracks {args.entity} (/gui/track)", flush=True)
                return 0
        time.sleep(3.0)
    print("gz_follow: no Gazebo GUI camera tracking service; follow the robot by hand (docs/demo, Watch it live)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
