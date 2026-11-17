#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Replay hook: plays a candump -L trace into one virtual CAN bus, in place of that bus's emulator.

    can_replay.py --interface vcan_bms --file trace.log [--loop] [--speed 1.0] [--fd] [--any-interface]

Frames keep the spacing of the trace on the host clock. Only lines recorded on `--interface` are sent, unless
--any-interface is given. --fd requires a CAN FD interface (MTU 72) and is needed for a trace with CAN FD frames.
The tool refuses an interface whose name does not start with "vcan": it never writes to a real bus.
ROS arguments that a launch file appends are ignored. Simulation only.
"""
import argparse
import signal
import sys
import time

from openamrobot_build1_scene import candump


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[1])
    parser.add_argument("--interface", required=True)
    parser.add_argument("--file", required=True)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--fd", action="store_true")
    parser.add_argument("--any-interface", action="store_true")
    args, _ros = parser.parse_known_args(argv)
    if not args.interface.startswith("vcan"):
        print(f"refused: {args.interface} is not a virtual CAN interface", file=sys.stderr)
        return 2
    with open(args.file, encoding="utf-8") as f:
        frames = candump.parse(f.read())
    if not args.any_interface:
        frames = [frame for frame in frames if frame.interface == args.interface]
    if not frames:
        print(f"no frame for {args.interface} in {args.file}", file=sys.stderr)
        return 1
    needs_fd = any(frame.fd for frame in frames)
    mtu = candump.interface_mtu(args.interface)
    if mtu is None:
        print(f"BLOCKED: {args.interface} does not exist", file=sys.stderr)
        return 3
    if needs_fd and (not args.fd or mtu != candump.CANFD_MTU):
        print(f"refused: the trace has CAN FD frames. Needs --fd and MTU {candump.CANFD_MTU}, {args.interface} has MTU {mtu}",
              file=sys.stderr)
        return 2
    sock = candump.open_socket(args.interface, fd_frames=args.fd)
    stop = {"now": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    signal.signal(signal.SIGINT, lambda *_: stop.update(now=True))

    def send(frame):
        if stop["now"]:
            raise KeyboardInterrupt
        sock.send(candump.pack(frame))

    period = frames[-1].timestamp - frames[0].timestamp
    sent, rounds = 0, 0
    print(f"can_replay: {len(frames)} frames, {period:.3f} s, into {args.interface}"
          f"{' (CAN FD)' if args.fd else ''}{', looping' if args.loop else ''}", flush=True)
    try:
        while True:
            started = time.monotonic()
            sent += candump.replay(frames, send, speed=args.speed)
            rounds += 1
            if not args.loop:
                break
            # One trace period per round, at least 10 ms, so that a one-frame trace does not flood the bus.
            rest = max(0.01, period / args.speed if period > 0 else 0.1) - (time.monotonic() - started)
            if rest > 0:
                time.sleep(rest)
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
    print(f"can_replay: sent {sent} frames in {rounds} rounds", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
