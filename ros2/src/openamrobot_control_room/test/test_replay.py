# SPDX-License-Identifier: MIT
"""REPLAY on a live ROS graph: a small bag with /clock, ground truth, /tf and a recorded /cmd_vel, played by
replay.Replay. Everything must come out under /control_room/replay/ with the recorded simulation time and the
mission phase at that time, labelled REPLAY, and the recorded /cmd_vel must not be replayed at all."""
import json

import pytest

import ros_support
from conftest import TRACKING
from openamrobot_control_room import readonly

ros_support.isolate()

PHASES = {"scenario": "replay_test", "result": "PASS", "phases": [
    {"phase": "spawn", "ran": True, "failures": [], "exception": None, "sim_start": 1.0, "sim_end": 2.0,
     "quantities": [{"ok": True}]},
    {"phase": "nav_to_station", "ran": True, "failures": ["x"], "exception": None, "sim_start": 2.0, "sim_end": 4.0,
     "quantities": [{"ok": False}]}]}


def _write_bag(path):
    import rosbag2_py
    from geometry_msgs.msg import TransformStamped, Twist
    from nav_msgs.msg import Odometry
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=path, storage_id="mcap"), rosbag2_py.ConverterOptions("cdr", "cdr"))
    for i, (name, kind) in enumerate((("/clock", "rosgraph_msgs/msg/Clock"), ("/cmd_vel", "geometry_msgs/msg/Twist"),
                                      ("/build1/ground_truth/odom", "nav_msgs/msg/Odometry"), ("/tf", "tf2_msgs/msg/TFMessage"))):
        writer.create_topic(rosbag2_py.TopicMetadata(id=i, name=name, type=kind, serialization_format="cdr"))
    for k in range(10, 40):
        t = k / 10.0
        ns = int(t * 1e9)
        clock = Clock()
        clock.clock.sec, clock.clock.nanosec = int(t), int((t % 1) * 1e9)
        writer.write("/clock", serialize_message(clock), ns)
        twist = Twist()
        twist.linear.x = 0.2
        writer.write("/cmd_vel", serialize_message(twist), ns)
        odom = Odometry()
        odom.pose.pose.position.x = t
        writer.write("/build1/ground_truth/odom", serialize_message(odom), ns)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_footprint"
        tf.transform.translation.x, tf.transform.rotation.w = t, 1.0
        writer.write("/tf", serialize_message(TFMessage(transforms=[tf])), ns)
    del writer


@pytest.fixture(scope="module")
def ros():
    ros_support.require_rclpy()
    try:
        import rosbag2_py  # noqa: F401
    except ImportError as error:
        pytest.skip(f"BLOCKED: no rosbag2_py ({error}); tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    import rclpy
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


def test_replay_publishes_only_under_the_replay_prefix(ros, tmp_path):
    import threading
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from openamrobot_control_room.monitor import make_node
    from openamrobot_control_room.replay import Replay
    bag = str(tmp_path / "bag")
    _write_bag(bag)
    node = make_node("control_room_replay")
    replay = Replay(node, [bag], PHASES, rate=5.0)
    observer = ros.create_node("control_room_replay_observer")
    got = {"clock": [], "status": [], "odom": [], "phase": []}
    observer.create_subscription(Clock, "/control_room/replay/clock", lambda m: got["clock"].append(m.clock.sec + m.clock.nanosec * 1e-9), 10)
    observer.create_subscription(String, "/control_room/replay/status", lambda m: got["status"].append(json.loads(m.data)), 10)
    observer.create_subscription(String, "/control_room/replay/mission_phase", lambda m: got["phase"].append(m.data), 10)
    observer.create_subscription(Odometry, "/control_room/replay/build1/ground_truth/odom",
                                 lambda m: got["odom"].append(m.pose.pose.position.x), 10)
    # Discovery first (#92): spin_until uses an executor of its own and takes the node out of it at the end. Wait until
    # the observer's subscriptions and the replay's publishers have matched, not a fixed time; fail if they never do.
    matched = ros_support.spin_until([observer, node], lambda: all(
        observer.count_publishers(t) > 0 and node.count_subscribers(t) > 0
        for t in ("/control_room/replay/clock", "/control_room/replay/status")), timeout=15.0)
    assert matched, "the observer and the replay node never discovered each other"
    executor = ros.executors.SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(observer)
    lock = threading.Lock()

    def spin_once(timeout):
        with lock:
            executor.spin_once(timeout_sec=timeout)
        if int(replay.sim_time or 0) != getattr(spin_once, "last", None):
            spin_once.last = int(replay.sim_time or 0)
            replay.publish_status()
    try:
        replay.run(spin_once)
        for _ in range(200):          # spin_once takes one callback: drain what the observer has queued
            spin_once(0.02)
        topics = [t for t, _ in observer.get_publisher_names_and_types_by_node("control_room_replay", "/")]
        assert readonly.publisher_violations(topics) == [], topics
        assert "/control_room/replay/cmd_vel" not in topics and "/cmd_vel" not in topics
        assert "/control_room/replay/tf" in topics
        assert got["clock"] and 1.0 <= got["clock"][0] < got["clock"][-1] <= 3.9
        assert got["odom"] and got["odom"][-1] == pytest.approx(3.9)
        final = got["status"][-1]
        assert final["mode"] == "REPLAY" and final["finished"] and final["not_replayed"] == ["/cmd_vel"]
        seen = {(s["phase"], s["verdict"]) for s in got["status"]}
        assert ("nav_to_station", "FAIL") in seen and ("spawn", "PASS") in seen, seen
        assert any(p.startswith("REPLAY nav_to_station (FAIL)") for p in got["phase"]), got["phase"]
    finally:
        node.destroy_node()
        observer.destroy_node()
