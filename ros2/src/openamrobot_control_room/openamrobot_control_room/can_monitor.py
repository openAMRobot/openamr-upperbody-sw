# SPDX-License-Identifier: MIT
"""Virtual CAN monitor: receive-only summaries of the five Build 1 buses. No ROS import (vcan_monitor.py wraps it).

Frames are counted and classified, never answered and never sent. The bus object is wrapped in ReceiveOnlyBus, which
has no send method, so a send cannot happen through this module. The CANopen function code is the generic CiA 301
split of an 11-bit identifier (NMT, SYNC, EMCY, TPDO, RPDO, SDO, heartbeat); no device payload is decoded, the device
models own their protocols. Simulation only.
"""
import collections
import threading
import time

BUSES = ("vcan_drive", "vcan_aux", "vcan_bms", "vcan_arm_l", "vcan_arm_r")
CANOPEN_BUSES = ("vcan_drive", "vcan_aux")    # ZLAC8015D and CTL-017 speak CANopen (openamrobot_sil)

_CANOPEN = ((0x000, 0x000, "NMT"), (0x080, 0x080, "SYNC"), (0x081, 0x0FF, "EMCY"), (0x100, 0x100, "TIME"),
            (0x180, 0x1FF, "TPDO1"), (0x200, 0x27F, "RPDO1"), (0x280, 0x2FF, "TPDO2"), (0x300, 0x37F, "RPDO2"),
            (0x380, 0x3FF, "TPDO3"), (0x400, 0x47F, "RPDO3"), (0x480, 0x4FF, "TPDO4"), (0x500, 0x57F, "RPDO4"),
            (0x580, 0x5FF, "SDO tx"), (0x600, 0x67F, "SDO rx"), (0x700, 0x77F, "heartbeat"), (0x7E4, 0x7E5, "LSS"))


def frame_class(can_id, extended=False, canopen=False, error=False, fd=False):
    if error:
        return "error frame"
    if extended:
        return "extended ID"
    if canopen:
        for low, high, name in _CANOPEN:
            if low <= can_id <= high:
                return name
        return "other 11-bit"
    return "CAN FD" if fd else "classic 11-bit"


class BusStats:
    """Counters of one bus over the whole run and over a sliding window."""

    def __init__(self, name, canopen=False, window_s=1.0, clock=time.monotonic):
        self.name, self.canopen, self.window_s, self.clock = name, canopen, window_s, clock
        self.total = 0
        self.classes = collections.Counter()
        self.ids = collections.Counter()
        self.recent = collections.deque()
        self.last = None
        self.state = "waiting"
        self.error = ""
        self.lock = threading.Lock()

    def add(self, can_id, extended=False, error=False, fd=False, t=None):
        t = self.clock() if t is None else t
        with self.lock:
            self.total += 1
            self.classes[frame_class(can_id, extended, self.canopen, error, fd)] += 1
            self.ids[can_id] += 1
            self.recent.append(t)
            self.last = t
            self.state = "receiving"

    def rate(self, now=None):
        now = self.clock() if now is None else now
        with self.lock:
            while self.recent and self.recent[0] < now - self.window_s:
                self.recent.popleft()
            return len(self.recent) / self.window_s

    def summary(self, now=None, top=6):
        now = self.clock() if now is None else now
        rate = self.rate(now)
        with self.lock:
            return {"bus": self.name, "state": self.state, "error": self.error, "frames": self.total,
                    "rate_hz": round(rate, 2), "last_frame_age_s": None if self.last is None else round(now - self.last, 3),
                    "classes": dict(sorted(self.classes.items())),
                    "top_ids": [[f"0x{i:03X}", n] for i, n in self.ids.most_common(top)]}


class ReceiveOnlyBus:
    """Exposes recv and shutdown of a python-can bus and nothing else: no send, no periodic send."""
    __slots__ = ("_bus",)

    def __init__(self, bus):
        self._bus = bus

    def recv(self, timeout=None):
        return self._bus.recv(timeout)

    def shutdown(self):
        self._bus.shutdown()


def open_socketcan(channel):
    """A receive-only socketcan bus. fd=True receives classic and FD frames; own frames are not looped back."""
    import can
    return ReceiveOnlyBus(can.Bus(interface="socketcan", channel=channel, fd=True, receive_own_messages=False))


class Monitor:
    """One receive thread per bus. `opener(name)` returns a ReceiveOnlyBus or raises OSError (bus absent)."""

    def __init__(self, buses=BUSES, canopen=CANOPEN_BUSES, opener=open_socketcan, window_s=1.0, clock=time.monotonic):
        self.stats = {b: BusStats(b, b in canopen, window_s, clock) for b in buses}
        self.opener, self.running, self.threads, self.buses = opener, False, [], {}

    def start(self):
        self.running = True
        for name in self.stats:
            thread = threading.Thread(target=self._loop, args=(name,), daemon=True)
            thread.start()
            self.threads.append(thread)
        return self

    def _loop(self, name):
        stats = self.stats[name]
        while self.running:
            try:
                bus = self.opener(name)
            except Exception as error:  # noqa: BLE001 - an absent interface is a state, not a crash
                stats.state, stats.error = "absent", str(error)[:200]
                time.sleep(2.0)
                continue
            if not isinstance(bus, ReceiveOnlyBus):
                bus = ReceiveOnlyBus(bus)
            self.buses[name] = bus
            stats.state, stats.error = "open", ""
            try:
                while self.running:
                    msg = bus.recv(0.2)
                    if msg is not None:
                        stats.add(msg.arbitration_id, bool(msg.is_extended_id), bool(msg.is_error_frame), bool(msg.is_fd))
            except Exception as error:  # noqa: BLE001 - the interface went away: reopen
                stats.state, stats.error = "absent", str(error)[:200]
            finally:
                try:
                    bus.shutdown()
                except Exception:  # noqa: BLE001
                    pass

    def stop(self):
        self.running = False
        for thread in self.threads:
            thread.join(timeout=2.0)

    def summary(self, now=None):
        return {name: stats.summary(now) for name, stats in self.stats.items()}
