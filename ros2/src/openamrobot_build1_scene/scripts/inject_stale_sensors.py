#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Fault injection: makes every sensor topic stale, or fresh again.

    inject_stale_sensors.py on | off | status

`on` stops the process of the sensor bridge (SIGSTOP), so /scan, /imu, the range sensors and every camera topic
stop arriving while Gazebo keeps running. `off` continues it (SIGCONT). Messages queued in between are delivered late.
Test tool for simulation only. It models a sensor link that goes quiet, not a sensor that reports wrong data.
"""
import sys

from openamrobot_build1_scene import faults


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) != 1 or argv[0] not in ("on", "off", "status"):
        print(__doc__, file=sys.stderr)
        return 2
    pids = faults.pids_with_argument(faults.SENSOR_BRIDGE_ARGUMENT)
    if not pids:
        print("the sensor bridge build1_sensor_bridge is not running", file=sys.stderr)
        return 1
    if argv[0] != "status":
        faults.stale_sensors(argv[0] == "on")
    for pid in pids:
        print(f"build1_sensor_bridge pid {pid}: {'stopped' if faults.process_state(pid) == 'T' else 'running'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
