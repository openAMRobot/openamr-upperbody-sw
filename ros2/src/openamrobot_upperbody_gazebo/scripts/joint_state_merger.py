#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Run the installed JointStatePublisher with callback-safe signal teardown.

Only lifecycle ownership changes: the upstream class still parses, merges,
limits and publishes every joint using the same parameters and topic names.
JSP 2.4.3's default rclpy handler can invalidate its context during take_message.
A signal here requests a stop; the active callback returns before destruction.
No RuntimeError or other active-context failure is swallowed.
"""
import argparse
import signal
import sys
import threading

import rclpy
from joint_state_publisher.joint_state_publisher import JointStatePublisher
from rclpy.executors import SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions


def spin_until_stopped(node, stopped):
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stopped.is_set() and node.context.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def main(args=None, factory=JointStatePublisher):
    stopped = threading.Event()
    previous = {}
    node = None
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, lambda signum, frame: stopped.set())
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument('description_file', nargs='?', default=None)
        parsed = parser.parse_args(rclpy.utilities.remove_ros_args(args=sys.argv if args is None else ['joint_state_merger', *args])[1:])
        node = factory(parsed.description_file)
        spin_until_stopped(node, stopped)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)
    return 0


if __name__ == '__main__':
    sys.exit(main())
