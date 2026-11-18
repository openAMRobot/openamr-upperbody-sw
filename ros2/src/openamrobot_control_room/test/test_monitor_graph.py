# SPDX-License-Identifier: MIT
"""The monitor nodes on a live ROS graph: what they publish, what they serve and call, and what they report.

A control room node may publish only under /control_room/ (plus /rosout and /parameter_events), and may have no
service client (an action client shows as the clients of its _action services), no action client or server in
rcl_action's view of the graph (which counts an _action/feedback subscriber as a client), no service server but Jazzy's
read-only type description service (no parameter services, so no parameter writes into it) and no publisher on a command topic. The graph is read from a second node.
"""
import json
import math

import can
import pytest

import ros_support
from openamrobot_control_room import readonly

ros_support.isolate()


@pytest.fixture(scope="module")
def ros():
    ros_support.require_rclpy()
    import rclpy
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def _nodes(ros):
    from openamrobot_control_room.monitor import ControlRoomMonitor, make_node
    from openamrobot_control_room.vcan_node import VcanMonitorNode
    monitor_node = make_node("control_room_monitor", parameter_overrides=[
        ros.parameter.Parameter("station", value=[2.0, 0.0, 0.0])])
    monitor = ControlRoomMonitor(monitor_node)
    vcan_node = make_node("control_room_vcan_monitor", parameter_overrides=[
        ros.parameter.Parameter("buses", value=["vcan_drive"])])
    vcan = VcanMonitorNode(vcan_node, opener=lambda name: can.Bus(interface="virtual", channel="cr_graph"))
    return monitor_node, monitor, vcan_node, vcan


def test_graph_is_read_only_and_status_reports(ros):
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import Float64, String
    monitor_node, monitor, vcan_node, vcan = _nodes(ros)
    observer = ros.create_node("control_room_test_observer")
    try:
        for name in ("control_room_monitor", "control_room_vcan_monitor"):
            found = ros_support.wait_graph(observer, name, lambda g: len(g["publishers"]) >= 2 and g["subscriptions"])
            assert found is not None, f"{name} not discovered"
            assert readonly.publisher_violations(found["publishers"]) == [], found
            assert found["clients"] == [], f"{name} has service or action clients: {found['clients']}"
            # Jazzy gives every node a read-only type description service; no parameter service may be there.
            assert set(found["services"]) <= {f"/{name}/get_type_description"}, \
                f"{name} serves {found['services']}: parameters could be written into it"
            assert not [t for t in found["publishers"] if readonly.is_command_topic(t)]
            # rcl_action's view: no action client and no action server (it also counts _action/feedback subscribers).
            from rclpy.action import graph as action_graph
            assert action_graph.get_action_client_names_and_types_by_node(observer, name, "/") == [], name
            assert action_graph.get_action_server_names_and_types_by_node(observer, name, "/") == [], name
        monitor_graph = ros_support.graph(observer, "control_room_monitor")
        for topic in ("/amcl_pose", "/build1/ground_truth/odom", "/navigate_to_pose/_action/status", "/sil/lift/status",
                      "/baseline1/mission_status", "/joint_states"):
            assert topic in monitor_graph["subscriptions"], topic

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        truth_pub = observer.create_publisher(Odometry, "/build1/ground_truth/odom", 10)
        amcl_pub = observer.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", latched)
        mission_pub = observer.create_publisher(String, "/baseline1/mission_status", latched)
        lift_pub = observer.create_publisher(String, "/sil/lift/status", 10)
        status, errors = [], []
        observer.create_subscription(String, "/control_room/status", lambda m: status.append(json.loads(m.data)), 10)
        observer.create_subscription(Float64, "/control_room/localization/position_error", lambda m: errors.append(m.data), 10)

        def publish():
            truth = Odometry()
            truth.pose.pose.position.x, truth.pose.pose.orientation.w = 1.9, 1.0
            truth_pub.publish(truth)
            amcl = PoseWithCovarianceStamped()
            amcl.pose.pose.position.x, amcl.pose.pose.position.y, amcl.pose.pose.orientation.w = 1.9, 0.03, 1.0
            amcl_pub.publish(amcl)
            lift_pub.publish(String(data=json.dumps({"state": 0, "active_stop": 1})))
            return False
        mission_pub.publish(String(data=json.dumps({"phase": "nav_to_station", "state": "running"})))
        ros_support.spin_until([monitor_node, vcan_node, observer], lambda: publish() or (
            errors and status and status[-1]["mission"]["status"] and status[-1]["lift"]["ctl017_status"]
            and status[-1]["localization"]["amcl_minus_truth_m"] is not None), timeout=15.0)
        assert errors and math.isclose(errors[-1], 0.03, abs_tol=1e-6), errors
        last = status[-1]
        assert last["mission"]["status"] == {"phase": "nav_to_station", "state": "running"}
        assert last["lift"]["ctl017_status"] == {"active_stop": 1, "state": 0}
        assert math.isclose(last["station"]["distance_m"], 0.1, abs_tol=1e-6)
        assert math.isclose(last["localization"]["amcl_minus_truth_m"], 0.03, abs_tol=1e-6)
        # After all of that traffic, still nothing outside /control_room/ and no clients.
        for name in ("control_room_monitor", "control_room_vcan_monitor"):
            after = ros_support.graph(observer, name)
            assert readonly.publisher_violations(after["publishers"]) == [] and after["clients"] == [], after
    finally:
        vcan.stop()
        for node in (monitor_node, vcan_node, observer):
            node.destroy_node()


def test_guard_refuses_a_command_publisher(ros):
    from openamrobot_control_room.monitor import make_node
    from geometry_msgs.msg import Twist
    node = make_node("control_room_guard_probe")
    try:
        with pytest.raises(readonly.ReadOnlyViolation):
            node.create_publisher(Twist, readonly.guard_publisher_topic("/cmd_vel"), 10)
        assert [t for t, _ in node.get_publisher_names_and_types_by_node("control_room_guard_probe", "/")
                if t == "/cmd_vel"] == []
    finally:
        node.destroy_node()
