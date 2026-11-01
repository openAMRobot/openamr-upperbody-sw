#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
"""Clamp the simulated joint state that move_group plans from, to the URDF's own limits, by no more than an epsilon, and count it.

    ros2 run openamrobot_upperbody_moveit joint_state_clamp.py --ros-args -p robot_description:="$(xacro ...)" [-p epsilon:=1e-5]

Subscribes to /joint_states and republishes it on /joint_states_clamped, which move_group is remapped to (move_group.launch.py). Everything else
keeps reading the raw /joint_states, unclamped. A joint reading past a limit by at most `epsilon` is published AT the limit; a reading further
out is published unchanged and logged as an error (not float noise: not this node's to hide). Every episode of clamping is logged with the joint
and the magnitude; every activation is counted. `~/get_stats` (std_srvs/Trigger) returns the counters as JSON in `message`, which is where the
A7 evidence reads them. Why this exists and what it is not: openamrobot_upperbody_moveit/joint_state_clamp.py.
"""
import signal
import sys
import threading
import xml.etree.ElementTree as ET

import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from openamrobot_upperbody_moveit.joint_state_clamp import EPSILON, Clamp, limits_from_urdf

INPUT_TOPIC = "joint_states"
OUTPUT_TOPIC = "joint_states_clamped"
OVER_EPSILON_LOG_PERIOD_S = 1.0


class JointStateClamp(Node):
    def __init__(self):
        super().__init__("joint_state_clamp")
        self.declare_parameter("robot_description", "")
        self.declare_parameter("epsilon", EPSILON)
        description = self.get_parameter("robot_description").value
        if not description:
            raise SystemExit("joint_state_clamp: the robot_description parameter is empty: the limits come from the URDF and from nowhere else")
        limits = limits_from_urdf(ET.fromstring(description))
        epsilon = float(self.get_parameter("epsilon").value)
        self.clamp = Clamp(limits, epsilon)
        self.publisher = self.create_publisher(JointState, OUTPUT_TOPIC, QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        # best effort so that it matches a reliable or a best-effort publisher alike
        self.create_subscription(JointState, INPUT_TOPIC, self.on_joint_state, qos_profile_sensor_data)
        self.create_service(Trigger, "~/get_stats", self.on_stats)
        self.last_over_epsilon_log = -1e9
        self.get_logger().info(f"clamping {len(limits)} joints to their URDF limits by at most {epsilon:g} rad: {INPUT_TOPIC} -> {OUTPUT_TOPIC}")

    def on_joint_state(self, msg):
        positions, events = self.clamp.process(msg.name, msg.position)
        for event in events:
            joint = event["joint"]
            lower, upper = self.clamp.limits[joint]
            if event["kind"] == "clamped":
                limit = lower if event["value"] < lower else upper
                if event["new_episode"]:
                    self.get_logger().warn(f"clamped {joint}: {event['overshoot']:.3e} rad past its limit {limit:g}, published at the limit "
                                           f"(activation {self.clamp.activations}, episode {self.clamp.episodes}, epsilon {self.clamp.epsilon:g})")
                else:
                    self.get_logger().debug(f"clamped {joint}: {event['overshoot']:.3e} rad (activation {self.clamp.activations})")
            else:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self.last_over_epsilon_log >= OVER_EPSILON_LOG_PERIOD_S:
                    self.last_over_epsilon_log = now
                    self.get_logger().error(f"{joint} reads {event['value']!r}: {event['overshoot']:.3e} rad past its limits [{lower:g}, {upper:g}], "
                                            f"beyond the epsilon {self.clamp.epsilon:g}: NOT float noise, passed through unchanged "
                                            f"({self.clamp.over_epsilon} such readings so far)")
        out = JointState(header=msg.header, name=list(msg.name), position=positions, velocity=list(msg.velocity), effort=list(msg.effort))
        self.publisher.publish(out)

    def on_stats(self, request, response):
        response.success = True
        response.message = self.clamp.summary_json()
        return response


def run(node, spin=rclpy.spin, context_ok=rclpy.ok):
    """Spin until the context is shut down and return the process exit code: 0 when a shutdown ended the spin, and anything else raises.

    For externally requested context shutdown, the spin can end in three ways:
    KeyboardInterrupt; ExternalShutdownException (rclpy raises it out of `spin` for every shutdown, SIGTERM included: found by sending the
    signal to this node, 30 of 30 exited 1); or the RuntimeError the executor's `take_message` raises when the signal lands between the
    wait set reporting a message and the take (rclpy 7.1.12, "Unable to convert call argument '0' to Python object": 1 of 30 SIGINTs). The
    third is only a shutdown if the context is no longer ok: the same error with a live context is a defect and is re-raised.
    main() handles process signals separately so an active callback finishes before context shutdown."""
    try:
        spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if context_ok():
            raise
    return 0


def spin_until_requested(node, stop_requested):
    """Finish the active callback before teardown; the signal only requests a stop."""
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    try:
        while not stop_requested.is_set() and node.context.ok():
            executor.spin_once(timeout_sec=0.1)
    finally:
        executor.remove_node(node)
        executor.shutdown()


def main(argv=None):
    # rclpy's default deferred signal handler can invalidate the context while
    # take_message is converting a message. A Python signal handler that only
    # sets this event lets the current callback return with its context alive.
    stop_requested = threading.Event()
    previous = {}
    rclpy.init(args=argv, signal_handler_options=SignalHandlerOptions.NO)
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.signal(sig, lambda signum, frame: stop_requested.set())
    node = None
    try:
        node = JointStateClamp()
        return run(node, spin=lambda current: spin_until_requested(current, stop_requested))
    finally:
        if node is not None:
            print("joint_state_clamp: final counters: " + node.clamp.summary_json(), flush=True)
            node.destroy_node()
        rclpy.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == "__main__":
    sys.exit(main())
