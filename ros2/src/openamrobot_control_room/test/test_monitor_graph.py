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
        # Exactly the registry the topic audit checks against the recorded runs, plus the action status topics.
        from openamrobot_control_room import state as state_model
        expected = set(state_model.MONITOR_TOPICS) | set(state_model.ACTION_STATUS_TOPICS)
        subscribed = set(monitor_graph["subscriptions"]) - {"/parameter_events"}
        assert subscribed == expected, (sorted(subscribed - expected), sorted(expected - subscribed))

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


def _recorded(topic, index=0, run="grocery_44299879412"):
    """One message of a recorded run (test/fixtures/runs), as the JSON string the topic carried."""
    from conftest import package_path
    with open(package_path("test", "fixtures", "runs", run, "messages.json"), encoding="utf-8") as f:
        value = json.load(f)[topic]["samples"][index]["value"]
    return value if isinstance(value, str) else json.dumps(value)


def test_recorded_payloads_reach_the_status(ros):
    """Messages recorded in grocery run 44299879412 on the real topics, published unchanged: the status must carry them,
    label the components from the recorded /build1/run_info and see who writes /cmd_vel on the graph."""
    from geometry_msgs.msg import Twist
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from std_msgs.msg import String
    monitor_node, monitor, vcan_node, vcan = _nodes(ros)
    observer = ros.create_node("control_room_test_recorded")
    try:
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        pubs = {t: observer.create_publisher(String, t, latched if t == "/build1/run_info" else 10)
                for t in ("/sil/lift/status", "/sil/drive/status", "/sil/bms/status", "/build1/run_info", "/grocery_demo/phase")}
        observer.create_publisher(Twist, "/cmd_vel", 10)      # a writer on the graph; nothing is published on it
        status = []
        observer.create_subscription(String, "/control_room/status", lambda m: status.append(json.loads(m.data)), 10)
        phase = _recorded("/grocery_demo/phase", 1)       # std_msgs/String with a plain phase name

        def publish():
            pubs["/sil/lift/status"].publish(String(data=_recorded("/sil/lift/status", 1)))
            pubs["/sil/drive/status"].publish(String(data=_recorded("/sil/drive/status", 1)))
            pubs["/sil/bms/status"].publish(String(data=_recorded("/sil/bms/status", 1)))
            pubs["/build1/run_info"].publish(String(data=_recorded("/build1/run_info")))
            pubs["/grocery_demo/phase"].publish(String(data=phase))
            return False

        def ready():
            if not status:
                return False
            last = status[-1]
            return (last["lift"]["ctl017_status"] and last["drive"]["status"] and last["mission"]["grocery_phase"]
                    and isinstance(last["provenance"].get("components"), dict)
                    and "/control_room_test_recorded" in (last["provenance"]["writers"].get("/cmd_vel") or []))
        assert ros_support.spin_until([monitor_node, vcan_node, observer], lambda: publish() or ready(), timeout=20.0), \
            status[-1:] or "no status"
        last = status[-1]
        lift = json.loads(_recorded("/sil/lift/status", 1))
        assert last["lift"]["ctl017_status"]["position_mm"] == lift["position_mm"] == 5.0
        assert last["drive"]["status"]["statusword"] == 55 and last["drive"]["status"]["configured"] is True
        assert last["bms"]["status"]["soc_percent"] == 80.0
        assert last["mission"]["grocery_phase"]["phase"] == phase == "stop_authorize"
        assert last["mission"]["source"].startswith("/grocery_demo/phase")
        components = last["provenance"]["components"]
        assert components["base_controller"].startswith("host test stand-in")
        assert "emulator" in components["lift"] and "unmodified openarm_ros2 driver" in components["arms"]
        assert "/control_room_test_recorded" in last["provenance"]["warning"]
        assert last["scene"]["warning"] and "grocery_scene.yaml" in last["scene"]["warning"]
        assert last["firmware"].startswith("no /sil/fw/status")
    finally:
        vcan.stop()
        for node in (monitor_node, vcan_node, observer):
            node.destroy_node()


def test_vcan_node_decodes_recorded_frames(ros):
    """Recorded vcan_aux frames of grocery run 44299879412 through a python-can virtual bus into the vcan monitor node:
    /control_room/can/decoded must carry the CTL-017 values of the last frames."""
    from conftest import package_path
    from openamrobot_control_room import can_decode
    from openamrobot_control_room.monitor import make_node
    from openamrobot_control_room.vcan_node import VcanMonitorNode
    from std_msgs.msg import String
    node = make_node("control_room_vcan_decode", parameter_overrides=[ros.parameter.Parameter("buses", value=["vcan_aux"])])
    vcan = VcanMonitorNode(node, opener=lambda name: can.Bus(interface="virtual", channel="cr_decode"))
    observer = ros.create_node("control_room_test_decode")
    sender = can.Bus(interface="virtual", channel="cr_decode")
    try:
        decoded = []
        observer.create_subscription(String, "/control_room/can/decoded", lambda m: decoded.append(json.loads(m.data)), 10)
        with open(package_path("test", "fixtures", "runs", "grocery_44299879412", "candump_vcan_aux.log"), encoding="utf-8") as f:
            frames = [can_decode.parse_candump(line) for line in f][:120]
        time_ok = ros_support.spin_until([node, observer], lambda: vcan.monitor.stats["vcan_aux"].state == "open", timeout=5.0)
        assert time_ok
        for _t, _bus, can_id, data, ext, _fd in frames:
            sender.send(can.Message(arbitration_id=can_id, data=data, is_extended_id=ext))
        expected = {}
        for _t, bus, can_id, data, ext, _fd in frames:
            record = can_decode.decode("vcan_aux", can_id, data, ext)
            if record["kind"] == "TPDO1 status":
                expected = record
        assert ros_support.spin_until([node, observer], lambda: decoded and "vcan_aux" in decoded[-1]["decoded"]
                                      and decoded[-1]["decoded"]["vcan_aux"].get("TPDO1 status", {}).get("counter")
                                      == expected["counter"], timeout=10.0), decoded[-1:] or "nothing decoded"
        tpdo1 = decoded[-1]["decoded"]["vcan_aux"]["TPDO1 status"]
        assert (tpdo1["state"], tpdo1["position_mm"], tpdo1["flags"]) == ("IDLE", 5.0, ["position valid", "at stop", "motor power"])
        assert vcan.monitor.stats["vcan_aux"].total == len(frames)
    finally:
        vcan.stop()
        sender.shutdown()
        node.destroy_node()
        observer.destroy_node()


def test_status_log_file(ros, tmp_path):
    """status_log: every status also lands in a JSON lines file (the e2e record that needs no DDS discovery)."""
    from openamrobot_control_room.monitor import ControlRoomMonitor, make_node
    path = tmp_path / "status.jsonl"
    node = make_node("control_room_log_probe", parameter_overrides=[ros.parameter.Parameter("status_log", value=str(path))])
    ControlRoomMonitor(node)
    try:
        assert ros_support.spin_until([node], lambda: path.exists() and len(path.read_text().splitlines()) >= 2, timeout=10.0)
        lines = [json.loads(line) for line in path.read_text().splitlines()]
        assert all("provenance" in line and "mission" in line for line in lines)
    finally:
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


def test_no_contact_sensor_no_force_and_one_pose_no_speed(ros):
    """Unsupported telemetry is unknown, never invented: without a /build1/contacts/* topic the monitor publishes no
    /control_room/contacts/max_force (it sent 0.0 N at 2 Hz in run 44476577635, contact sensors off) and the status says
    null; one item pose gives no /control_room/item/speed. The status must arrive, so the test fails without the node."""
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import Float64, String
    monitor_node, monitor, vcan_node, vcan = _nodes(ros)
    observer = ros.create_node("control_room_test_unknown")
    try:
        status, force, speed = [], [], []
        observer.create_subscription(String, "/control_room/status", lambda m: status.append(json.loads(m.data)), 10)
        observer.create_subscription(Float64, "/control_room/contacts/max_force", lambda m: force.append(m.data), 10)
        observer.create_subscription(Float64, "/control_room/item/speed", lambda m: speed.append(m.data), 10)
        item_pub = observer.create_publisher(PoseStamped, "/build1/ground_truth/item/pose", 10)
        item = PoseStamped()
        item.pose.position.z = 0.9

        def publish():
            item_pub.publish(item)      # the same pose and stamp every time: no motion can be computed
            return False
        assert ros_support.spin_until([monitor_node, vcan_node, observer],
                                      lambda: publish() or (len(status) >= 4 and status[-1]["item"] is not None), timeout=20.0), \
            status[-1:] or "no /control_room/status: the monitor is not running"
        last = status[-1]
        assert last["contacts"]["max_force_n"] is None and last["contacts"]["topics"] == 0, last["contacts"]
        assert last["item"]["speed_mps"] is None, last["item"]
        assert force == [], f"/control_room/contacts/max_force published {force[:5]} without a contact sensor"
        assert speed == [], f"/control_room/item/speed published {speed[:5]} from one pose"
    finally:
        vcan.stop()
        for node in (monitor_node, vcan_node, observer):
            node.destroy_node()
