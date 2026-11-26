# SPDX-License-Identifier: MIT
"""The /clock fail-fast of build1.launch.py and the "no /clock" capture (gate owner decision on upperbody #77,
22 Nov 2026). No ROS import: scripts/wait_for_clock.py and scripts/clock_capture.py wrap it, the launch reads its
constants, and the strict verdict reads the record it writes.

With driver_clock:=sim the real arm driver (ros2_control_node, /controller_manager) runs its control loop on the ROS
clock. Without a /clock message that loop never runs and every switch_controller call times out (60 s, three times),
which hid the real cause in jobs 137151317986 and 137160885173. The launch therefore starts the real driver only
after the first ROS /clock message, and ends the leg with the named reason NO_CLOCK when none arrives within
BOUND_S wall seconds of the launch start.

BOUND_S, derived from today's passing legs (22 Nov 2026, eight job logs, PR text): the first ROS /clock arrived
7.0 to 11.7 s after the launch start (upper estimate from the SIL emulators' first "simulation stalled" line, checked
against rtf_monitor's tenth report). 60 s is about five times the slowest leg. The gate itself starts only after the
simulator's emulator spawner exits (about 15 s after the launch start), so it waits at least MIN_WAIT_S after its own
start even when the bound has already passed.

Simulation evidence only. Not a safety function: a missing clock stops a simulation run, nothing else.
"""
import json
import os
import time

BOUND_S = 60.0            # wall seconds from the launch start to the first ROS /clock message
MIN_WAIT_S = 10.0         # the gate waits at least this long after its own start (discovery of the bridge's writer)
EXIT_NO_CLOCK = 3         # exit status of wait_for_clock.py when the bound passes without a message
NO_CLOCK = "no /clock"    # the named reason of the leg
RECORD = "clock_gate.json"
CAPTURE_DIR = "clock_capture"
CAPTURE_ENV = "BUILD1_CLOCK_CAPTURE"
PROBE_AT_S = 10.0         # capture: probes when no /clock has arrived this long after the launch start
NETSTAT_AT_S = 15.0       # capture: netstat -su this long after the launch start
GZ_VERBOSE_PROCESSES = ("gazebo", "build1_control_bridge")


def capture_enabled(environ):
    return environ.get(CAPTURE_ENV, "") == "1"


class Gate:
    """Waits for the first /clock message. poll() -> None (keep waiting), 0 (clock seen) or EXIT_NO_CLOCK."""

    def __init__(self, launch_start, bound_s=BOUND_S, min_wait_s=MIN_WAIT_S, now=time.time):
        self.launch_start, self.bound_s, self.now = float(launch_start), float(bound_s), now
        self.started = now()
        self.deadline = max(self.launch_start + self.bound_s, self.started + float(min_wait_s))
        self.first_clock = None    # wall time of the first message
        self.first_sim = None

    def on_clock(self, sim_s=None):
        if self.first_clock is None:
            self.first_clock, self.first_sim = self.now(), sim_s

    def poll(self):
        if self.first_clock is not None:
            return 0
        if self.now() >= self.deadline:
            return EXIT_NO_CLOCK
        return None

    def record(self):
        status = self.poll()
        fired = status == EXIT_NO_CLOCK
        return {"result": NO_CLOCK if fired else ("clock" if status == 0 else "waiting"),
                "fired": fired, "bound_s": self.bound_s, "launch_start": self.launch_start,
                "gate_started_after_launch_s": round(self.started - self.launch_start, 3),
                "first_clock_after_launch_s": (None if self.first_clock is None
                                               else round(self.first_clock - self.launch_start, 3)),
                "first_clock_sim_s": self.first_sim,
                "reason": (f"BLOCKED (sim infrastructure): {NO_CLOCK}: no ROS /clock message within {self.bound_s:.0f} s "
                           "of the launch start; the real arm driver was not started") if fired else ""}


def write_record(directory, record):
    os.makedirs(directory, exist_ok=True)
    path = os.path.join(directory, RECORD)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(record, f, indent=2, sort_keys=True)
    return path


def read_record(directory):
    try:
        with open(os.path.join(directory, RECORD), encoding="utf-8") as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def fired(record):
    """True when the record says the fail-fast ended the leg with the named reason."""
    return bool(record) and record.get("result") == NO_CLOCK and bool(record.get("fired"))
