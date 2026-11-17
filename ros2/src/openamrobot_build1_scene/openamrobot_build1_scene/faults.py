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


# CTL-017 v1 fault ids, bit n-1 of the fault mask (openamr-platform-fw can_sil/ctl017_emulator.py, F01..F09).
CTL017_FAULTS = {1: "F01", 2: "F02 wrong direction or limit reached while driving", 3: "F03 stall (no Hall pulses) or overcurrent",
                 4: "F04 move deadline", 5: "F05", 6: "F06", 7: "F07 supervision lost", 8: "F08", 9: "F09 sensor conflict"}


def ctl017_faults(mask):
    """The CTL-017 fault ids set in `mask`, as text."""
    mask = int(mask or 0)
    names = [CTL017_FAULTS[n] for n in sorted(CTL017_FAULTS) if mask & (1 << (n - 1))]
    return ", ".join(names) if names else "none"
