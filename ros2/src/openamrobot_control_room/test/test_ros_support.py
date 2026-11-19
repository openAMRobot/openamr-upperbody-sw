# SPDX-License-Identifier: MIT
"""The graph helpers of the ROS tests must not pass on nothing (openamr-upperbody-sw#88): without ROS, on a stand-in
observer that answers like rclpy's graph queries."""
import ros_support


class Observer:
    """get_*_names_and_types_by_node of rclpy: raises for a node that is not on the graph."""

    def __init__(self, nodes):
        self.nodes = nodes

    def _get(self, name, key):
        if name not in self.nodes:
            raise RuntimeError(f"node {name} not found")      # rclpy raises NodeNameNonExistentError
        return [(t, ["std_msgs/msg/String"]) for t in self.nodes[name].get(key, [])]

    def get_publisher_names_and_types_by_node(self, name, namespace):
        return self._get(name, "publishers")

    def get_subscriber_names_and_types_by_node(self, name, namespace):
        return self._get(name, "subscriptions")

    def get_service_names_and_types_by_node(self, name, namespace):
        return self._get(name, "services")

    def get_client_names_and_types_by_node(self, name, namespace):
        return self._get(name, "clients")


def has_publishers(found):
    return len(found["publishers"]) >= 2


def test_wait_graph_without_the_node_is_none():
    assert ros_support.wait_graph(Observer({}), "control_room_monitor", has_publishers, timeout=0.3) is None


def test_wait_graph_with_the_node_seen_empty_is_none():
    """Discovered but without its publishers yet: every list is empty, so "no publisher outside /control_room/" and
    "no clients" would hold on it. The helper must report that the condition never held, not hand back that graph."""
    observer = Observer({"control_room_monitor": {}})
    assert ros_support.wait_graph(observer, "control_room_monitor", has_publishers, timeout=0.3) is None


def test_wait_graph_returns_the_graph_once_the_condition_holds():
    observer = Observer({"control_room_monitor": {"publishers": ["/control_room/status", "/rosout"]}})
    found = ros_support.wait_graph(observer, "control_room_monitor", has_publishers, timeout=1.0)
    assert found["publishers"] == ["/control_room/status", "/rosout"] and found["clients"] == []
