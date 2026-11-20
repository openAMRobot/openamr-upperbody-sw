# SPDX-License-Identifier: MIT
"""The CAN monitor on a kernel vcan interface (socketcan, the path the node uses), receive only.

A sender puts N frames on the interface and a second raw listener counts every frame on it. The monitor must count N
and the listener must see exactly N: one frame more would be a transmission by the monitor. The interface is the
test's own (vcan_cr_test), created and deleted here, so the five Build 1 buses are left alone.
"""
import os
import shutil
import subprocess
import time

import pytest

from conftest import TRACKING
from openamrobot_control_room import can_monitor

IFACE = "vcan_cr_test"


def _sh(*args):
    sudo = [] if os.geteuid() == 0 else ["sudo", "-n"]
    try:
        return subprocess.run(sudo + list(args), capture_output=True, text=True)
    except OSError as error:      # the command itself is missing (modprobe in a container)
        return subprocess.CompletedProcess(args, 127, "", str(error))


@pytest.fixture()
def vcan():
    if shutil.which("ip") is None:
        pytest.skip(f"BLOCKED: no ip command (iproute2); tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    if subprocess.run(["ip", "link", "show", IFACE], capture_output=True).returncode != 0:
        _sh("modprobe", "vcan")
        made = _sh("ip", "link", "add", "dev", IFACE, "type", "vcan")
        if made.returncode != 0:
            pytest.skip(f"BLOCKED: cannot create {IFACE}: {made.stderr.strip()}; tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    _sh("ip", "link", "set", IFACE, "mtu", "72")
    _sh("ip", "link", "set", "up", IFACE)
    yield IFACE
    _sh("ip", "link", "delete", IFACE)


def test_socketcan_receive_only(vcan):
    import can
    sender = can.Bus(interface="socketcan", channel=vcan, fd=True)
    listener = can.Bus(interface="socketcan", channel=vcan, fd=True)
    monitor = can_monitor.Monitor(buses=(vcan,), canopen=(vcan,), opener=can_monitor.open_socketcan).start()
    try:
        deadline = time.monotonic() + 5.0
        while monitor.stats[vcan].state != "open" and time.monotonic() < deadline:
            time.sleep(0.02)
        assert monitor.stats[vcan].state == "open", monitor.summary()
        frames = 50
        for i in range(frames):
            sender.send(can.Message(arbitration_id=0x581 if i % 2 else 0x701, data=[i % 256], is_extended_id=False))
        sender.send(can.Message(arbitration_id=0x101, data=list(range(12)), is_fd=True, is_extended_id=False))
        deadline = time.monotonic() + 5.0
        while monitor.stats[vcan].total < frames + 1 and time.monotonic() < deadline:
            time.sleep(0.02)
        time.sleep(0.5)
        summary = monitor.summary()[vcan]
        assert summary["frames"] == frames + 1, summary
        assert summary["classes"] == {"SDO tx": 25, "heartbeat": 25, "other 11-bit": 1}, summary
        seen = 0
        while listener.recv(0.3) is not None:
            seen += 1
        assert seen == frames + 1, f"the listener saw {seen} frames for {frames + 1} sent: the monitor transmitted"
    finally:
        monitor.stop()
        sender.shutdown()
        listener.shutdown()
