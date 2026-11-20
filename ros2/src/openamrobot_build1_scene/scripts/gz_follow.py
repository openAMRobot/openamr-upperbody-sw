#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Make the Gazebo GUI camera follow a model, through the GUI's CameraTracking plugin, and confirm that it does.

    gz_follow.py [--entity openamrobot] [--offset -2.5,0.0,2.0] [--timeout 600]

The mechanism, read in the sources of the versions this demo pins (gz-sim 8.15.0, tag gz-sim8_8.15.0, which uses
gz-gui8; gz-gui tag gz-gui8_8.4.0, src/plugins/camera_tracking/CameraTracking.cc):
  - /gui/track is a gz-transport TOPIC of gz.msgs.CameraTrack (the plugin subscribes to it; the Entity Tree's
    right-click Follow publishes on it). track_mode FOLLOW with follow_target.name and follow_offset makes the camera
    follow the model with the offset in the model's own frame (trackWorldFrame false) and look at it.
  - /gui/follow (gz.msgs.StringMsg) and /gui/follow/offset (gz.msgs.Vector3d) are SERVICES, marked deprecated in
    that source but still offered. They answer true at once, even before the model exists in the GUI scene.
  - /gui/currently_tracked (gz.msgs.CameraTrack) is published at 50 Hz with the mode and target actually in use.
    When the target is not in the GUI scene yet, the plugin drops it ("Unable to follow target") and reports NONE.
So this script publishes on /gui/track, then reads /gui/currently_tracked, and repeats every 3 s until the GUI
reports FOLLOW of the model (the robot is spawned some seconds after the window opens). If the topic route is not
confirmed, it tries the deprecated services the same way. A view aid for a human: it commands nothing in the
simulation. Without a GUI it gives up after --timeout wall seconds and exits 0 (the run goes on).
"""
import argparse
import re
import subprocess
import sys
import time

TRACK_TOPIC = "/gui/track"
STATUS_TOPIC = "/gui/currently_tracked"
FOLLOW_SERVICE = "/gui/follow"
OFFSET_SERVICE = "/gui/follow/offset"


def track_request(entity, offset):
    x, y, z = offset
    return (f'track_mode: FOLLOW, follow_target: {{name: "{entity}"}}, '
            f"follow_offset: {{x: {x}, y: {y}, z: {z}}}")


def is_following(status_text, entity):
    """True when a /gui/currently_tracked message (gz text format) says FOLLOW of `entity`."""
    mode = re.search(r"track_mode:\s*(\w+)", status_text)
    target = re.search(r'follow_target\s*\{[^}]*name:\s*"([^"]*)"', status_text, re.S)
    return bool(mode and mode.group(1).startswith("FOLLOW") and target and target.group(1) == entity)


def gz(arguments, timeout=20):
    try:
        done = subprocess.run(["gz", *arguments], capture_output=True, text=True, timeout=timeout, check=False)
        return done.returncode, done.stdout
    except (OSError, subprocess.TimeoutExpired):
        return 1, ""


def topics():
    return set(gz(["topic", "-l"])[1].split())


def services():
    return set(gz(["service", "-l"])[1].split())


def status():
    return gz(["topic", "-e", "-t", STATUS_TOPIC, "-n", "1"], timeout=5)[1]


def follow_by_topic(entity, offset):
    gz(["topic", "-t", TRACK_TOPIC, "-m", "gz.msgs.CameraTrack", "-p", track_request(entity, offset)])


def follow_by_service(entity, offset):
    x, y, z = offset
    common = ["--reptype", "gz.msgs.Boolean", "--timeout", "3000"]
    gz(["service", "-s", FOLLOW_SERVICE, "--reqtype", "gz.msgs.StringMsg", *common, "--req", f'data: "{entity}"'])
    gz(["service", "-s", OFFSET_SERVICE, "--reqtype", "gz.msgs.Vector3d", *common, "--req", f"x: {x}, y: {y}, z: {z}"])


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--entity", default="openamrobot")
    parser.add_argument("--offset", default="-2.5,0.0,2.0")
    parser.add_argument("--timeout", type=float, default=600.0)
    args, _ros = parser.parse_known_args(argv)
    offset = tuple(float(v) for v in args.offset.split(","))
    end = time.monotonic() + args.timeout
    attempt = 0
    while time.monotonic() < end:
        if STATUS_TOPIC in topics():
            attempt += 1
            # Topic first (the plugin's current interface); every third attempt the deprecated services.
            route = "services /gui/follow and /gui/follow/offset" if attempt % 3 == 0 and FOLLOW_SERVICE in services() \
                else "topic /gui/track"
            (follow_by_service if route.startswith("services") else follow_by_topic)(args.entity, offset)
            time.sleep(1.0)
            if is_following(status(), args.entity):
                print(f"gz_follow: confirmed by {STATUS_TOPIC}: the Gazebo camera follows {args.entity}, offset "
                      f"{offset[0]} {offset[1]} {offset[2]} m in its frame ({route}, attempt {attempt})", flush=True)
                return 0
        time.sleep(3.0)
    print(f"gz_follow: the Gazebo GUI never reported FOLLOW of {args.entity} on {STATUS_TOPIC} within {args.timeout:.0f} s; "
          "follow it by hand: Entity Tree, right-click the model, Follow (docs/demo, Watch it live)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
