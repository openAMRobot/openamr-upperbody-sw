# SPDX-License-Identifier: MIT
"""The arm buses carry CAN FD on virtual CAN, checked on the wire. No test here assumes classic CAN behaviour.

BLOCKED, never passed, where the interfaces cannot be had.
"""
import pytest

from openamrobot_build1_scene import candump
from openamrobot_build1_scene.launch_support import ARM_BUSES, BASE_BUSES
from vcan_support import blocked_reason


@pytest.fixture(scope="module", autouse=True)
def buses():
    reason = blocked_reason()
    if reason:
        pytest.skip(reason)


def test_arm_buses_have_mtu_72_and_base_buses_mtu_16():
    for bus in ARM_BUSES.values():
        assert candump.interface_mtu(bus) == candump.CANFD_MTU == 72
    for bus in BASE_BUSES:
        assert candump.interface_mtu(bus) == candump.CAN_MTU == 16


@pytest.mark.parametrize("bus", sorted(ARM_BUSES.values()))
@pytest.mark.parametrize("length", [8, 12, 64])
def test_fd_frame_with_bit_rate_switch_arrives_as_an_fd_frame(bus, length):
    receiver, sender = candump.open_socket(bus, timeout=2.0), candump.open_socket(bus)
    try:
        frame = candump.Frame(0.0, bus, 0x011, bytes(range(length)), fd=True, brs=True)
        assert sender.send(candump.pack(frame)) == candump.CANFD_MTU
        raw = receiver.recv(candump.CANFD_MTU)
        assert len(raw) == candump.CANFD_MTU, "an FD frame is read as 72 bytes, a classic frame as 16"
        got = candump.unpack(raw, bus)
        assert got.fd and got.brs and got.data == frame.data and got.can_id == 0x011
    finally:
        receiver.close()
        sender.close()


@pytest.mark.parametrize("bus", sorted(ARM_BUSES.values()))
def test_a_classic_only_socket_does_not_see_fd_frames(bus):
    """A reader that assumes classic CAN misses the arm traffic: this is why nothing here may assume it."""
    classic_reader = candump.open_socket(bus, fd_frames=False, timeout=0.5)
    fd_reader, sender = candump.open_socket(bus, timeout=2.0), candump.open_socket(bus)
    try:
        sender.send(candump.pack(candump.Frame(0.0, bus, 0x012, bytes(16), fd=True, brs=True)))
        assert len(fd_reader.recv(candump.CANFD_MTU)) == candump.CANFD_MTU
        with pytest.raises((TimeoutError, OSError)):
            classic_reader.recv(candump.CANFD_MTU)
        sender.send(candump.pack(candump.Frame(0.0, bus, 0x013, b"\x01\x02")))
        assert len(classic_reader.recv(candump.CANFD_MTU)) == candump.CAN_MTU
    finally:
        for sock in (classic_reader, fd_reader, sender):
            sock.close()


@pytest.mark.parametrize("bus", sorted(ARM_BUSES.values()))
def test_an_eight_byte_fd_frame_is_not_a_classic_frame(bus):
    """The motor frames are 8 bytes long. They are still FD frames on this bus and are read as 72 bytes."""
    receiver, sender = candump.open_socket(bus, timeout=2.0), candump.open_socket(bus)
    try:
        sender.send(candump.pack(candump.Frame(0.0, bus, 0x001, bytes(8), fd=True, brs=True)))
        fd = candump.unpack(receiver.recv(candump.CANFD_MTU), bus)
        sender.send(candump.pack(candump.Frame(0.0, bus, 0x001, bytes(8))))
        classic = candump.unpack(receiver.recv(candump.CANFD_MTU), bus)
        assert fd.fd and not classic.fd and fd.data == classic.data
    finally:
        receiver.close()
        sender.close()


@pytest.mark.parametrize("bus", BASE_BUSES)
def test_base_buses_refuse_fd_frames(bus):
    sender = candump.open_socket(bus)
    try:
        with pytest.raises(OSError):
            sender.send(candump.pack(candump.Frame(0.0, bus, 0x123, bytes(12), fd=True)))
        assert sender.send(candump.pack(candump.Frame(0.0, bus, 0x123, bytes(8)))) == candump.CAN_MTU
    finally:
        sender.close()
