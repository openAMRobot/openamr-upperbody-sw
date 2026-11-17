# SPDX-License-Identifier: MIT
"""Listens on the virtual CAN buses and counts classic and CAN FD frames. Standard library only."""
import threading
import time

from openamrobot_build1_scene import candump
from openamrobot_build1_scene.launch_support import ALL_BUSES


def listen(interface, seconds):
    """-> {"classic", "fd", "brs", "ids"} of the frames seen on `interface` in `seconds` of wall time."""
    counts = {"classic": 0, "fd": 0, "brs": 0, "ids": set()}
    sock = candump.open_socket(interface, fd_frames=True, timeout=0.2)
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            try:
                frame = candump.unpack(sock.recv(candump.CANFD_MTU), interface)
            except (TimeoutError, OSError):
                continue
            counts["fd" if frame.fd else "classic"] += 1
            counts["brs"] += 1 if frame.brs else 0
            counts["ids"].add(frame.can_id)
    finally:
        sock.close()
    return counts


def listen_all(seconds, buses=ALL_BUSES):
    results = {}

    def worker(bus):
        results[bus] = listen(bus, seconds)
    threads = [threading.Thread(target=worker, args=(bus,)) for bus in buses]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    return results
