# SPDX-License-Identifier: MIT
"""candump -L traces and raw SocketCAN frames, classic and CAN FD. Standard library only.

Trace line format (can-utils log format):
    (1700000000.000000) vcan_arm_l 011##1AABBCCDD      CAN FD frame, one hex digit of flags after ##
    (1700000000.001000) vcan_bms 18904001#0100...      classic frame, 8 hex digits mean a 29-bit identifier
    (1700000000.002000) vcan_drive 701#R               remote frame (not replayed)
"""
import re
import socket
import struct
import time
from dataclasses import dataclass

CAN_EFF_FLAG = 0x80000000
CAN_RTR_FLAG = 0x40000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x7FF
CANFD_BRS = 0x01
CANFD_ESI = 0x02
CAN_MTU = 16           # sizeof(struct can_frame)
CANFD_MTU = 72         # sizeof(struct canfd_frame)
CAN_RAW_FD_FRAMES = 5
SOL_CAN_RAW = 101
FD_LENGTHS = (0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64)
LINE = re.compile(r"^\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]{3,8})(##?)(\S*)$")


@dataclass(frozen=True)
class Frame:
    timestamp: float
    interface: str
    can_id: int
    data: bytes
    extended: bool = False
    fd: bool = False
    brs: bool = False
    esi: bool = False

    def raw_id(self):
        return (self.can_id | CAN_EFF_FLAG) if self.extended else self.can_id


def parse_line(line):
    """One candump -L line -> Frame, or None for a blank line, a comment or a remote frame."""
    text = line.strip()
    if not text or text.startswith("#"):
        return None
    match = LINE.match(text)
    if not match:
        raise ValueError(f"not a candump -L line: {line!r}")
    stamp, interface, ident, separator, payload = match.groups()
    extended = len(ident) == 8
    can_id = int(ident, 16)
    if can_id > (CAN_EFF_MASK if extended else CAN_SFF_MASK):
        raise ValueError(f"identifier out of range: {line!r}")
    fd = separator == "##"
    flags = 0
    if fd:
        if not payload:
            raise ValueError(f"CAN FD line without the flags digit: {line!r}")
        flags, payload = int(payload[0], 16), payload[1:]
    elif payload[:1] in ("R", "r"):
        return None
    if len(payload) % 2:
        raise ValueError(f"odd number of data digits: {line!r}")
    data = bytes.fromhex(payload)
    if fd and len(data) not in FD_LENGTHS:
        raise ValueError(f"{len(data)} is not a CAN FD data length: {line!r}")
    if not fd and len(data) > 8:
        raise ValueError(f"a classic frame carries at most 8 bytes: {line!r}")
    return Frame(float(stamp), interface, can_id, data, extended, fd, bool(flags & CANFD_BRS), bool(flags & CANFD_ESI))


def parse(text):
    """A whole trace -> [Frame] in file order. Timestamps must not go backwards."""
    frames = [frame for frame in (parse_line(line) for line in text.splitlines()) if frame is not None]
    for before, after in zip(frames, frames[1:]):
        if after.timestamp < before.timestamp:
            raise ValueError(f"timestamps go backwards at {after.timestamp}")
    return frames


def format_line(frame):
    ident = f"{frame.can_id:08X}" if frame.extended else f"{frame.can_id:03X}"
    if frame.fd:
        flags = (CANFD_BRS if frame.brs else 0) | (CANFD_ESI if frame.esi else 0)
        return f"({frame.timestamp:.6f}) {frame.interface} {ident}##{flags:X}{frame.data.hex().upper()}"
    return f"({frame.timestamp:.6f}) {frame.interface} {ident}#{frame.data.hex().upper()}"


def pack(frame):
    """The bytes written to a raw socket: struct canfd_frame (72) for FD, struct can_frame (16) otherwise."""
    if frame.fd:
        flags = (CANFD_BRS if frame.brs else 0) | (CANFD_ESI if frame.esi else 0)
        return struct.pack("=IBB2x64s", frame.raw_id(), len(frame.data), flags, frame.data)
    return struct.pack("=IB3x8s", frame.raw_id(), len(frame.data), frame.data)


def unpack(raw, interface="", timestamp=0.0):
    """Bytes read from a raw socket -> Frame. The size says which kind of frame it is: 16 classic, 72 CAN FD."""
    if len(raw) == CANFD_MTU:
        raw_id, length, flags, data = struct.unpack("=IBB2x64s", raw)
        fd = True
    elif len(raw) == CAN_MTU:
        raw_id, length, data = struct.unpack("=IB3x8s", raw)
        flags, fd = 0, False
    else:
        raise ValueError(f"{len(raw)} bytes is neither a classic ({CAN_MTU}) nor a CAN FD ({CANFD_MTU}) frame")
    extended = bool(raw_id & CAN_EFF_FLAG)
    return Frame(timestamp, interface, raw_id & (CAN_EFF_MASK if extended else CAN_SFF_MASK), data[:length], extended, fd,
                 bool(flags & CANFD_BRS), bool(flags & CANFD_ESI))


def interface_mtu(interface, sys_root="/sys/class/net"):
    try:
        with open(f"{sys_root}/{interface}/mtu") as f:
            return int(f.read().strip())
    except (OSError, ValueError):
        return None


def open_socket(interface, fd_frames=True, timeout=None):
    """A bound raw CAN socket. fd_frames: also receive and send CAN FD frames (CAN_RAW_FD_FRAMES)."""
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    if fd_frames:
        sock.setsockopt(SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1)
    sock.bind((interface,))
    if timeout is not None:
        sock.settimeout(timeout)
    return sock


def receive(sock, interface=""):
    return unpack(sock.recv(CANFD_MTU), interface, time.time())


def replay(frames, send, speed=1.0, sleep=time.sleep, clock=time.monotonic):
    """Send every frame with the trace's own spacing divided by `speed`. -> number of frames sent.
    `send` takes a Frame. The spacing is kept against the start, so a slow send does not add up."""
    if not frames:
        return 0
    start, first = clock(), frames[0].timestamp
    for frame in frames:
        due = start + (frame.timestamp - first) / speed
        wait = due - clock()
        if wait > 0:
            sleep(wait)
        send(frame)
    return len(frames)
