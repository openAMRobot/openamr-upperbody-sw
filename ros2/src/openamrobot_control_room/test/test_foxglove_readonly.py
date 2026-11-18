# SPDX-License-Identifier: MIT
"""foxglove_bridge with config/foxglove_bridge.yaml is subscribe only.

The bridge is started as a process in its own ROS domain. A WebSocket client connects and the test checks that:
- the server info offers no clientPublish, services, parameters, parametersSubscribe or assets capability;
- a client advertise of /cmd_vel and a message on it create no ROS publisher on /cmd_vel and deliver nothing;
- a setParameters request does not change a parameter of a probe node;
- a fetchAsset request returns no asset;
- the bridge node publishes nothing outside /rosout, /parameter_events and has no service client to the probe node.
The revert case (the bridge defaults) is test_static.test_foxglove_check_catches_the_bridge_defaults.
"""
import asyncio
import json
import os
import shutil
import socket
import struct
import subprocess
import time

import pytest
import yaml

import ros_support
from conftest import TRACKING, package_path

ros_support.isolate()
CONFIG = package_path("config", "foxglove_bridge.yaml")
WRITE_CAPABILITIES = ("clientPublish", "services", "parameters", "parametersSubscribe", "assets")


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def ros():
    ros_support.require_rclpy()
    import rclpy
    rclpy.init()
    yield rclpy
    rclpy.try_shutdown()


@pytest.fixture(scope="module")
def bridge(ros, tmp_path_factory):
    try:
        import websockets  # noqa: F401
    except ImportError as error:
        pytest.skip(f"BLOCKED: no Python websockets ({error}); tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    if shutil.which("ros2") is None:
        pytest.skip(f"BLOCKED: no ros2 command; tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    found = subprocess.run(["ros2", "pkg", "prefix", "foxglove_bridge"], capture_output=True, text=True)
    if found.returncode != 0:
        pytest.skip(f"BLOCKED: foxglove_bridge is not installed; tracked in {TRACKING}")  # tracking: openamr-upperbody-sw#83
    port = _free_port()
    params = yaml.safe_load(open(CONFIG, encoding="utf-8"))
    params["control_room_foxglove_bridge"]["ros__parameters"]["port"] = port
    path = tmp_path_factory.mktemp("fox") / "bridge.yaml"
    path.write_text(yaml.safe_dump(params))
    log = open(path.with_suffix(".log"), "w")
    process = subprocess.Popen(["ros2", "run", "foxglove_bridge", "foxglove_bridge", "--ros-args", "-r",
                                "__node:=control_room_foxglove_bridge", "--params-file", str(path)],
                               stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        with socket.socket() as s:
            if s.connect_ex(("127.0.0.1", port)) == 0:
                break
        time.sleep(0.2)
    else:
        process.kill()
        print(open(path.with_suffix(".log")).read())
        pytest.fail("foxglove_bridge did not open its port in 30 s")
    yield port
    process.terminate()
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        process.kill()
    log.close()
    print(open(path.with_suffix(".log")).read())


async def _session(port, actions):
    import websockets
    async with websockets.connect(f"ws://127.0.0.1:{port}", subprotocols=["foxglove.websocket.v1", "foxglove.sdk.v1"],
                                  max_size=None, open_timeout=10) as ws:
        received = []
        info = json.loads(await asyncio.wait_for(ws.recv(), 10))
        for kind, payload in actions:
            await ws.send(json.dumps(payload) if kind == "json" else payload)
            await asyncio.sleep(0.3)
        end = time.monotonic() + 3.0
        while time.monotonic() < end:
            try:
                received.append(await asyncio.wait_for(ws.recv(), max(0.05, end - time.monotonic())))
            except asyncio.TimeoutError:
                break
            except Exception:  # noqa: BLE001 - the server may close the connection on a refused request
                break
        return ws.subprotocol, info, received


def test_bridge_is_subscribe_only(ros, bridge):
    from geometry_msgs.msg import Twist
    observer = ros.create_node("control_room_fox_observer")
    probe = ros.create_node("control_room_fox_probe")
    probe.declare_parameter("x", 1)
    twists = []
    observer.create_subscription(Twist, "/cmd_vel", twists.append, 10)
    twist = json.dumps({"linear": {"x": 1.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.5}}).encode()
    actions = [
        ("json", {"op": "advertise", "channels": [{"id": 7, "topic": "/cmd_vel", "encoding": "json",
                                                   "schemaName": "geometry_msgs/msg/Twist"}]}),
        ("bin", struct.pack("<BI", 1, 7) + twist), ("bin", struct.pack("<BI", 1, 7) + twist),
        ("json", {"op": "setParameters", "id": "w1", "parameters": [{"name": "/control_room_fox_probe.x", "value": 5},
                                                                    {"name": "control_room_fox_probe.x", "value": 5}]}),
        ("json", {"op": "getParameters", "id": "r1", "parameterNames": ["/control_room_fox_probe.x"]}),
        ("json", {"op": "fetchAsset", "uri": "package://openamrobot_control_room/package.xml", "requestId": 3}),
    ]
    try:
        protocol, info, received = asyncio.run(_session(bridge, actions))
        print("subprotocol", protocol, "server info", info, "received", [r if isinstance(r, str) else r[:16] for r in received])
        assert info.get("op") == "serverInfo"
        offered = info.get("capabilities", [])
        assert not [c for c in WRITE_CAPABILITIES if c in offered], offered
        ros_support.spin_until([observer, probe], lambda: bool(twists), timeout=3.0)
        assert twists == [], "a client message reached /cmd_vel"
        assert observer.get_publishers_info_by_topic("/cmd_vel") == [], "the bridge advertised /cmd_vel"
        assert probe.get_parameter("x").value == 1, "the bridge changed a parameter"
        for message in received:
            if isinstance(message, (bytes, bytearray)) and message[:1] == b"\x04":
                status = message[5] if len(message) > 5 else None
                assert status != 0, "fetchAsset returned an asset"
            if isinstance(message, str):
                parsed = json.loads(message)
                assert parsed.get("op") != "parameterValues" or not parsed.get("parameters"), parsed
        found = ros_support.wait_graph(observer, "control_room_foxglove_bridge", lambda g: True)
        assert found is not None, "bridge node not on the graph"
        print("bridge graph", found)
        allowed = {"/rosout", "/parameter_events"}
        assert set(found["publishers"]) <= allowed, found["publishers"]
        assert not [c for c in found["clients"] if "control_room_fox_probe" in c], found["clients"]
    finally:
        observer.destroy_node()
        probe.destroy_node()
