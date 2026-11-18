# SPDX-License-Identifier: MIT
"""The receive-only CAN counters on python-can's in-process virtual bus. No ROS, no kernel vcan."""
import time

import can
import pytest

from openamrobot_control_room import can_monitor


def test_frame_classes():
    assert can_monitor.frame_class(0x701, canopen=True) == "heartbeat"
    assert can_monitor.frame_class(0x581, canopen=True) == "SDO tx"
    assert can_monitor.frame_class(0x601, canopen=True) == "SDO rx"
    assert can_monitor.frame_class(0x181, canopen=True) == "TPDO1"
    assert can_monitor.frame_class(0x080, canopen=True) == "SYNC"
    assert can_monitor.frame_class(0x000, canopen=True) == "NMT"
    assert can_monitor.frame_class(0x18FF50E5, extended=True) == "extended ID"
    assert can_monitor.frame_class(0x101, fd=True) == "CAN FD"
    assert can_monitor.frame_class(0x1, error=True) == "error frame"


def test_rate_window():
    now = [100.0]
    stats = can_monitor.BusStats("vcan_drive", canopen=True, window_s=1.0, clock=lambda: now[0])
    for i in range(10):
        stats.add(0x181, t=99.05 + i * 0.1)
    assert stats.rate(100.0) == 10.0
    assert stats.rate(100.5) == 5.0
    s = stats.summary(100.5)
    assert s["frames"] == 10 and s["classes"] == {"TPDO1": 10} and s["top_ids"] == [["0x181", 10]]


def test_receive_only_bus_has_no_send():
    bus = can_monitor.ReceiveOnlyBus(can.Bus(interface="virtual", channel="cr_unit_send"))
    try:
        assert not hasattr(bus, "send") and not hasattr(bus, "send_periodic")
        with pytest.raises(AttributeError):
            bus.send(can.Message(arbitration_id=1))
    finally:
        bus.shutdown()


def test_monitor_counts_frames_and_sends_none():
    channel = "cr_unit_listen"
    sender = can.Bus(interface="virtual", channel=channel, receive_own_messages=False)
    observer = can.Bus(interface="virtual", channel=channel)
    monitor = can_monitor.Monitor(buses=("vcan_drive",), canopen=("vcan_drive",),
                                  opener=lambda name: can.Bus(interface="virtual", channel=channel)).start()
    try:
        deadline = time.monotonic() + 5.0
        while monitor.stats["vcan_drive"].state != "open" and time.monotonic() < deadline:
            time.sleep(0.02)
        for i in range(20):
            sender.send(can.Message(arbitration_id=0x701 if i % 2 else 0x181, data=[i], is_extended_id=False))
        deadline = time.monotonic() + 5.0
        while monitor.stats["vcan_drive"].total < 20 and time.monotonic() < deadline:
            time.sleep(0.02)
        summary = monitor.summary()["vcan_drive"]
        assert summary["frames"] == 20 and summary["classes"] == {"TPDO1": 10, "heartbeat": 10}
        seen = 0
        while observer.recv(0.2) is not None:
            seen += 1
        assert seen == 20, "the observer saw a frame the sender did not send: the monitor transmitted"
    finally:
        monitor.stop()
        sender.shutdown()
        observer.shutdown()


def test_absent_interface_is_a_state():
    def opener(name):
        raise OSError(f"no such device {name}")
    monitor = can_monitor.Monitor(buses=("vcan_gone",), opener=opener).start()
    try:
        deadline = time.monotonic() + 3.0
        while monitor.stats["vcan_gone"].state != "absent" and time.monotonic() < deadline:
            time.sleep(0.02)
        assert monitor.summary()["vcan_gone"]["state"] == "absent"
    finally:
        monitor.stop()
