# SPDX-License-Identifier: MIT
"""The replay hook on a real virtual bus, with short synthetic traces.

1. A CAN FD trace is played into vcan_arm_l and read back frame by frame.
2. A Daly trace is played into vcan_bms in place of the BMS emulator. The host test client of openamrobot_sil,
   unchanged, polls the bus and reports the voltage and state of charge of the trace, not the emulator's defaults.
BLOCKED, never passed, where the interfaces cannot be had.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

import pytest

from conftest import package_path
from openamrobot_build1_scene import candump
from vcan_support import blocked_reason

REPLAY = [sys.executable, package_path("scripts", "can_replay.py")]


@pytest.fixture(scope="module", autouse=True)
def buses():
    reason = blocked_reason()
    if reason:
        pytest.skip(reason)


def environment():
    env = dict(os.environ)
    env["PYTHONPATH"] = package_path() + os.pathsep + env.get("PYTHONPATH", "")
    return env


def test_fd_trace_arrives_frame_for_frame_with_its_spacing():
    trace = package_path("traces", "synthetic_arm_fd.log")
    with open(trace, encoding="utf-8") as f:
        expected = candump.parse(f.read())
    receiver = candump.open_socket("vcan_arm_l", timeout=5.0)
    got = []

    def read():
        for _ in expected:
            raw = receiver.recv(candump.CANFD_MTU)
            got.append((time.monotonic(), candump.unpack(raw, "vcan_arm_l")))
    reader = threading.Thread(target=read)
    reader.start()
    done = subprocess.run([*REPLAY, "--interface", "vcan_arm_l", "--file", trace, "--fd"], env=environment(),
                          capture_output=True, text=True, timeout=60)
    reader.join(timeout=10)
    receiver.close()
    assert done.returncode == 0, done.stderr
    assert "sent 7 frames" in done.stdout
    assert [(f.can_id, f.data, f.fd, f.brs) for _t, f in got] == [(f.can_id, f.data, f.fd, f.brs) for f in expected]
    span, trace_span = got[-1][0] - got[0][0], expected[-1].timestamp - expected[0].timestamp
    print(f"BUILD1 replay fd_frames = 6 frames, classic_frames = 1, trace span {trace_span:.3f} s, replayed in {span:.3f} s")
    assert span >= trace_span - 0.005, "frames must not arrive earlier than the trace spaces them"
    assert span < trace_span + 2.0, "plausibility only: this is thread timing, not a timing measurement"


def test_a_trace_with_fd_frames_needs_the_fd_switch_and_an_fd_bus():
    trace = package_path("traces", "synthetic_arm_fd.log")
    done = subprocess.run([*REPLAY, "--interface", "vcan_arm_l", "--file", trace], env=environment(), capture_output=True, text=True, timeout=60)
    assert done.returncode == 2 and "CAN FD" in done.stderr
    done = subprocess.run([*REPLAY, "--interface", "vcan_drive", "--file", trace, "--fd", "--any-interface"], env=environment(),
                          capture_output=True, text=True, timeout=60)
    assert done.returncode == 2 and "MTU" in done.stderr


def test_bms_trace_in_place_of_the_emulator_reaches_the_host_client():
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import BatteryState
    from std_msgs.msg import String
    from ament_index_python.packages import get_package_share_directory
    config = os.path.join(get_package_share_directory("openamrobot_sil"), "config", "sil.yaml")
    env = environment()
    env["ROS_DOMAIN_ID"] = str(150 + os.getpid() % 50)
    env["ROS_AUTOMATIC_DISCOVERY_RANGE"] = "LOCALHOST"
    os.environ.update(ROS_DOMAIN_ID=env["ROS_DOMAIN_ID"], ROS_AUTOMATIC_DISCOVERY_RANGE="LOCALHOST")
    replay = subprocess.Popen([*REPLAY, "--interface", "vcan_bms", "--file", package_path("traces", "synthetic_bms_26v4_55pct.log"), "--loop"],
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    client = subprocess.Popen(["ros2", "run", "openamrobot_sil", "stm32_host_client", "--ros-args", "-p", f"config_file:={config}"],
                              env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    rclpy.init()
    node = Node("build1_replay_probe")
    seen = {}
    node.create_subscription(BatteryState, "/sil/bms/battery_state", lambda m: seen.__setitem__("battery", m), 10)
    node.create_subscription(String, "/sil/bms/status", lambda m: seen.__setitem__("status", json.loads(m.data)), 10)
    try:
        end = time.monotonic() + 60.0
        while time.monotonic() < end and not (seen.get("battery") and seen.get("status", {}).get("answers", 0) >= 6):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert "battery" in seen, "no /sil/bms/battery_state: the host test client read nothing from the replayed bus"
        battery, status = seen["battery"], seen["status"]
        print(f"BUILD1 replay bms_voltage = {battery.voltage:.2f} V, soc = {battery.percentage * 100:.1f} %, answers = {status['answers']}")
        assert battery.voltage == pytest.approx(26.4, abs=0.05), "the emulator default is 25.6 V. 26.4 V is in the trace only"
        assert battery.percentage == pytest.approx(0.55, abs=0.001), "the emulator default is 80 %. 55 % is in the trace only"
        assert status["answers"] >= 6 and status["age_s"] < 1.0
    finally:
        node.destroy_node()
        rclpy.shutdown()
        replay.send_signal(signal.SIGINT)
        os.killpg(client.pid, signal.SIGINT)
        for process in (replay, client):
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                process.kill()
