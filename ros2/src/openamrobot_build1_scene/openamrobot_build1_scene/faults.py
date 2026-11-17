# SPDX-License-Identifier: MIT
"""Fault injection helpers that need no ROS. Test tools for simulation only."""
import os
import signal

SENSOR_BRIDGE_ARGUMENT = "__node:=build1_sensor_bridge"


def pids_with_argument(argument, proc="/proc"):
    pids = []
    for name in os.listdir(proc):
        if not name.isdigit():
            continue
        try:
            with open(os.path.join(proc, name, "cmdline"), "rb") as f:
                arguments = f.read().split(b"\0")
        except OSError:
            continue
        if argument.encode() in arguments:
            pids.append(int(name))
    return pids


def process_state(pid, proc="/proc"):
    """The state letter of /proc/<pid>/stat: T is stopped."""
    with open(os.path.join(proc, str(pid), "stat")) as f:
        return f.read().rsplit(")", 1)[1].split()[0]


def stale_sensors(on):
    """Stop (SIGSTOP) or continue (SIGCONT) the sensor bridge. -> the pids it acted on."""
    pids = pids_with_argument(SENSOR_BRIDGE_ARGUMENT)
    for pid in pids:
        os.kill(pid, signal.SIGSTOP if on else signal.SIGCONT)
    return pids
