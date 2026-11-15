# SPDX-License-Identifier: MIT
"""The replay hook without a bus: the trace format, the frame bytes and the timing."""
import pytest

from conftest import package_path
from openamrobot_build1_scene import candump


def test_classic_extended_and_fd_lines():
    frames = candump.parse("(1.000000) vcan_drive 701#05\n# comment\n\n(1.001000) vcan_bms 18904001#0108010875300226\n"
                           "(1.002000) vcan_arm_l 011##1000102030405060708090A0B\n(1.003) vcan_drive 601#R\n")
    assert len(frames) == 3
    heartbeat, daly, fd = frames
    assert (heartbeat.can_id, heartbeat.data, heartbeat.extended, heartbeat.fd) == (0x701, b"\x05", False, False)
    assert daly.can_id == 0x18904001 and daly.extended and len(daly.data) == 8
    assert fd.fd and fd.brs and not fd.esi and len(fd.data) == 12 and fd.can_id == 0x011


def test_lines_round_trip():
    for line in ("(1700000000.000000) vcan_drive 701#05", "(1700000000.001000) vcan_bms 18904001#0108010875300226",
                 "(1700000000.002000) vcan_arm_l 011##1000102030405060708090A0B"):
        assert candump.format_line(candump.parse_line(line)) == line


@pytest.mark.parametrize("line", ["garbage", "(1.0) vcan0 123#123", "(1.0) vcan0 123#00112233445566778899",
                                  "(1.0) vcan0 123##100112233445566778899", "(1.0) vcan0 FFF#00", "(1.0) vcan0 123##"])
def test_bad_lines_are_refused(line):
    with pytest.raises(ValueError):
        candump.parse_line(line)


def test_timestamps_must_not_go_backwards():
    with pytest.raises(ValueError):
        candump.parse("(2.0) vcan0 123#00\n(1.0) vcan0 123#00\n")


def test_frame_bytes_are_those_of_the_kernel_structs():
    classic = candump.parse_line("(1.0) vcan_bms 18904001#0108010875300226")
    raw = candump.pack(classic)
    assert len(raw) == candump.CAN_MTU == 16
    assert raw[:4] == (0x18904001 | candump.CAN_EFF_FLAG).to_bytes(4, "little") and raw[4] == 8 and raw[8:] == classic.data
    fd = candump.parse_line("(1.0) vcan_arm_l 011##1" + "AB" * 64)
    raw = candump.pack(fd)
    assert len(raw) == candump.CANFD_MTU == 72 and raw[4] == 64 and raw[5] == candump.CANFD_BRS and raw[8:] == b"\xab" * 64
    for frame in (classic, fd):
        back = candump.unpack(candump.pack(frame), frame.interface, frame.timestamp)
        assert back == frame
    with pytest.raises(ValueError):
        candump.unpack(b"\x00" * 20)


def test_replay_keeps_the_spacing_of_the_trace():
    frames = candump.parse("(10.000) vcan0 001#00\n(10.050) vcan0 002#00\n(10.250) vcan0 003#00\n")
    now, sent = [100.0], []

    def sleep(seconds):
        now[0] += seconds
    assert candump.replay(frames, lambda frame: sent.append((round(now[0] - 100.0, 6), frame.can_id)), sleep=sleep, clock=lambda: now[0]) == 3
    assert sent == [(0.0, 1), (0.05, 2), (0.25, 3)]
    now, sent = [0.0], []
    candump.replay(frames, lambda frame: sent.append(round(now[0], 6)), speed=5.0, sleep=sleep, clock=lambda: now[0])
    assert sent == [0.0, 0.01, 0.05]
    assert candump.replay([], lambda frame: None) == 0


def test_a_slow_send_does_not_add_up():
    frames = candump.parse("(0.0) vcan0 001#00\n(0.1) vcan0 002#00\n(0.2) vcan0 003#00\n")
    now, sent = [0.0], []

    def send(frame):
        sent.append(round(now[0], 6))
        now[0] += 0.03          # the send itself takes 30 ms

    def sleep(seconds):
        now[0] += seconds
    candump.replay(frames, send, sleep=sleep, clock=lambda: now[0])
    assert sent == [0.0, 0.1, 0.2]


def test_the_synthetic_traces_of_the_package_parse():
    with open(package_path("traces", "synthetic_bms_26v4_55pct.log"), encoding="utf-8") as f:
        bms = candump.parse(f.read())
    assert len(bms) == 33 and {frame.can_id for frame in bms} == {0x18904001, 0x18934001, 0x18984001}
    assert all(frame.interface == "vcan_bms" and frame.extended and not frame.fd for frame in bms)
    voltage, _acquired, current, soc = (int.from_bytes(bms[0].data[i:i + 2], "big") for i in (0, 2, 4, 6))
    assert (voltage * 0.1, (current - 30000) * 0.1, soc * 0.1) == pytest.approx((26.4, 0.0, 55.0))
    with open(package_path("traces", "synthetic_arm_fd.log"), encoding="utf-8") as f:
        arm = candump.parse(f.read())
    assert [len(frame.data) for frame in arm] == [8, 12, 16, 24, 48, 64, 4]
    assert [frame.fd for frame in arm] == [True] * 6 + [False] and all(frame.brs for frame in arm[:6])


def test_the_replay_tool_refuses_a_real_bus(capsys):
    import importlib.util
    spec = importlib.util.spec_from_file_location("can_replay", package_path("scripts", "can_replay.py"))
    tool = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tool)
    assert tool.main(["--interface", "can0", "--file", package_path("traces", "synthetic_arm_fd.log")]) == 2
    assert "not a virtual CAN interface" in capsys.readouterr().err
