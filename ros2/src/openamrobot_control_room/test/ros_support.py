# SPDX-License-Identifier: MIT
"""rclpy in its own ROS domain for one test module, and graph queries on a node. BLOCKED without ROS 2."""
import os
import random
import time

import pytest

from conftest import TRACKING


def isolate():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(201, 232))
    os.environ["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"


def require_rclpy():
    try:
        import rclpy  # noqa: F401
    except ImportError as error:
        pytest.skip(f"BLOCKED: no ROS 2 Jazzy here ({error}); tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83


def spin_until(nodes, condition, timeout=10.0):
    import rclpy
    executor = rclpy.executors.SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)
    deadline = time.monotonic() + timeout
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            if condition():
                return True
        return False
    finally:
        for node in nodes:
            executor.remove_node(node)


def graph(observer, name, namespace="/"):
    """Publishers, subscriptions, service servers and service clients of one node, as seen from another node."""
    pubs = [t for t, _ in observer.get_publisher_names_and_types_by_node(name, namespace)]
    subs = [t for t, _ in observer.get_subscriber_names_and_types_by_node(name, namespace)]
    servers = [t for t, _ in observer.get_service_names_and_types_by_node(name, namespace)]
    clients = [t for t, _ in observer.get_client_names_and_types_by_node(name, namespace)]
    return {"publishers": sorted(pubs), "subscriptions": sorted(subs), "services": sorted(servers), "clients": sorted(clients)}


def wait_graph(observer, name, condition, timeout=10.0):
    deadline = time.monotonic() + timeout
    found = None
    while time.monotonic() < deadline:
        try:
            found = graph(observer, name)
        except Exception:  # noqa: BLE001 - the node is not discovered yet
            found = None
        if found is not None and condition(found):
            return found
        time.sleep(0.1)
    return found
